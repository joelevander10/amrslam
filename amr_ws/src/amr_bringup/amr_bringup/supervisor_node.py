"""amr_supervisor: the main process of amr.service (unified plan §3, §5, §8).

Owns four process groups - web, base, an optional Foxglove bridge, and at most
one mode layer (mapping or navigation) - and the single source of motion
authority, /amr/control_lease. Boots STARTING -> IDLE with base and web up and
no layer. Mode changes and survey operations are asynchronous operations
(OperationState) driven by one serial loop; every step has a deadline and a
failure lands in FAULT with motion inhibited, never in a half-switched stack.

    /amr/mode_state        ModeState, transient-local, on change + 2 Hz
    /amr/control_lease     ControlLease, 10 Hz, only from the loop while it makes progress
    /amr/mode/request      RequestMode   (IDLE | NAVIGATION on an exact map revision)
    /amr/supervisor/survey RequestSurvey (START | RETURNED | SAVE | ABORT)
    /amr/operations/get    GetOperation
    /amr/supervisor/recover std_srvs/Trigger  (FAULT -> IDLE after cleanup)

The supervisor never publishes MotionPermit and never talks to CAN, the DIO
island or the map files' contents beyond bundle verification.
"""

from __future__ import annotations

import faulthandler
import os
import signal
import sys
import threading
import time
import uuid

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from amr_bringup import domains
from amr_bringup import mode_fsm as fsm
from amr_bringup import operations as ops
from amr_bringup import readiness as rd
from amr_bringup.process_supervisor import Group
from amr_interfaces.msg import (
    CommissioningState,
    ControlLease,
    DriveStatus,
    Event,
    LocalizationState,
    MappingState,
    ModeState,
    MuxState,
    OperationState,
    PanelState,
    RunState,
    WheelStates,
)
from amr_interfaces.srv import GetOperation, RequestMode, RequestSurvey, SaveMap, StartSurvey

RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
SENSOR = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)

LEASE_MANUAL, LEASE_AUTONOMOUS, LEASE_COMMISSIONING = 1, 2, 4
INTERNAL = "/amr/internal/survey"


class Supervisor(Node):
    def __init__(self) -> None:
        super().__init__("amr_supervisor")
        dp = self.declare_parameter
        dp("real", True)
        dp("maps_dir", os.path.expanduser("~/amr_maps"))
        dp("state_dir", os.path.expanduser("~/.amr"))
        dp("web", True)
        dp("foxglove", True)
        dp("lidar", True)
        dp("slip_noise_std", 0.02)  # sim
        dp("clutter_count", 0)  # sim
        dp("world_yaml", "")  # sim
        dp("base_ready_s", 30.0)
        dp("layer_start_s", 30.0)
        dp("layer_stop_s", 12.0)
        dp("mux_ack_s", 2.0)
        dp("still_wait_s", 5.0)
        dp("save_s", 60.0)
        dp("survey_rpc_s", 15.0)  # returned / abort answer budget (review R27)
        p = self.get_parameter
        self.real = bool(p("real").value)
        self.maps_dir = os.path.expanduser(str(p("maps_dir").value))
        self.state_dir = os.path.expanduser(str(p("state_dir").value))
        os.makedirs(self.state_dir, exist_ok=True)
        self.budget = {
            k: float(p(k).value)
            for k in (
                "base_ready_s",
                "layer_start_s",
                "layer_stop_s",
                "mux_ack_s",
                "still_wait_s",
                "save_s",
                "survey_rpc_s",
            )
        }

        self.instance = uuid.uuid4().hex
        self.generation = 1
        self.lease_seq = 0
        self.mode = fsm.STARTING
        self.requested = fsm.STARTING
        self.phase = "starting"
        self.fault_code = ""
        self.reason = ""
        self.active_map = ("", 0, "")
        self.last_survey = ("", 0)
        self.book = ops.OperationBook(os.path.join(self.state_dir, "operations.jsonl"))
        self.snap = rd.Snapshots()
        self.groups: dict[str, Group] = {}
        self.txn: fsm.Transaction | None = None
        self.survey_op: tuple[str, str] | None = None  # (operation_id, kind) in flight
        self.inhibit_manual = False  # during a save
        self._pending_future = None
        self._worker: threading.Thread | None = None
        self._worker_result: bool | None = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._stopping = False  # read without the lock by shutdown()
        self._logs = os.path.join(self.state_dir, "logs")
        os.makedirs(self._logs, exist_ok=True)

        g = ReentrantCallbackGroup()
        self.create_subscription(MuxState, "/amr/mux_state", self.snap.on_mux, RELIABLE_1, callback_group=g)
        self.create_subscription(
            DriveStatus, "/drives/status", self.snap.on_drives, RELIABLE_1, callback_group=g
        )
        self.create_subscription(WheelStates, "/wheel_states", self.snap.on_wheels, SENSOR, callback_group=g)
        self.create_subscription(PanelState, "/amr/panel_state", self.snap.on_panel, 10, callback_group=g)
        self.create_subscription(
            MappingState, "/amr/mapping_state", self.snap.on_mapping, LATCHED, callback_group=g
        )
        self.create_subscription(RunState, "/amr/run_state", self.snap.on_run, LATCHED, callback_group=g)
        self.create_subscription(
            LocalizationState, "/amr/localization_state", self.snap.on_loc, LATCHED, callback_group=g
        )
        self.create_subscription(
            CommissioningState,
            "/amr/commissioning_state",
            self.snap.on_commissioning,
            LATCHED,
            callback_group=g,
        )
        self._pub_lease = self.create_publisher(ControlLease, "/amr/control_lease", RELIABLE_1)
        self._pub_mode = self.create_publisher(ModeState, "/amr/mode_state", LATCHED)
        self._pub_event = self.create_publisher(Event, "/amr/events", 50)
        self._event_seq = 0
        self.create_service(RequestMode, "/amr/mode/request", self._srv_mode, callback_group=g)
        self.create_service(RequestSurvey, "/amr/supervisor/survey", self._srv_survey, callback_group=g)
        self.create_service(GetOperation, "/amr/operations/get", self._srv_get_op, callback_group=g)
        self.create_service(Trigger, "/amr/supervisor/recover", self._srv_recover, callback_group=g)
        self._cli = {
            "start": self.create_client(StartSurvey, f"{INTERNAL}/start", callback_group=g),
            "returned": self.create_client(Trigger, f"{INTERNAL}/returned", callback_group=g),
            "save": self.create_client(SaveMap, f"{INTERNAL}/save", callback_group=g),
            "abort": self.create_client(Trigger, f"{INTERNAL}/abort", callback_group=g),
            "lm_loc": self.create_client(
                ManageLifecycleNodes, "/lifecycle_manager_localization/manage_nodes", callback_group=g
            ),
            "lm_nav": self.create_client(
                ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes", callback_group=g
            ),
            "lm_loc_active": self.create_client(
                Trigger, "/lifecycle_manager_localization/is_active", callback_group=g
            ),
            "lm_nav_active": self.create_client(
                Trigger, "/lifecycle_manager_navigation/is_active", callback_group=g
            ),
        }
        self.create_timer(0.5, self._publish_mode)
        self._thread = threading.Thread(target=self._loop, name="supervisor-loop", daemon=True)
        self.get_logger().info(
            f"instance {self.instance[:8]} real={self.real} maps={self.maps_dir} "
            f"interrupted={[o.operation_id for o in self.book.interrupted]}"
        )

    # ------------------------------------------------------------------ helpers

    def _now(self) -> float:
        return time.monotonic()

    def _env(self) -> dict:
        return dict(os.environ)

    def _launch(self, role: str, file: str, **args) -> Group:
        argv = ["ros2", "launch", "amr_bringup", file] + [f"{k}:={v}" for k, v in args.items()]
        g = Group.spawn(role, argv, env=self._env(), log_path=os.path.join(self._logs, f"{role}.log"))
        self.get_logger().info(f"spawned {g.describe()}: {' '.join(argv[3:])}")
        return g

    def _event(self, level: int, code: str, text: str) -> None:
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source, m.level, m.code, m.text = "amr_supervisor", level, code, text
        self._event_seq += 1
        m.seq = self._event_seq
        try:
            self._pub_event.publish(m)
        except Exception:  # noqa: BLE001 - never let an event stop the loop
            pass

    def _set(self, mode: int, phase: str = "", reason: str = "", fault: str = "") -> None:
        with self._lock:
            if mode != self.mode:
                self.get_logger().info(
                    f"mode {fsm.MODE_NAMES[self.mode]} -> {fsm.MODE_NAMES[mode]} ({phase or reason})"
                )
                lvl = Event.ERROR if mode == fsm.FAULT else Event.INFO
                self._event(
                    lvl,
                    "MODE_CHANGE",
                    f"{fsm.MODE_NAMES[self.mode]} -> {fsm.MODE_NAMES[mode]}: {fault or phase or reason}",
                )
            self.mode, self.phase, self.reason = mode, phase, reason
            if fault:
                self.fault_code = fault
            elif mode != fsm.FAULT:
                self.fault_code = ""
        self._publish_mode()

    def _allowed(self) -> int:
        if self.txn is not None or self.mode in (fsm.STARTING, fsm.TRANSITIONING, fsm.FAULT, fsm.STOPPING):
            return 0
        if self.inhibit_manual:
            return 0
        if self.mode == fsm.NAVIGATION:
            return LEASE_MANUAL | LEASE_AUTONOMOUS
        if self.mode == fsm.IDLE and self.snap.commissioning_active(self._now()):
            return LEASE_COMMISSIONING  # exclusive IDLE substate: no ordinary jog meanwhile
        return LEASE_MANUAL

    def _conditions(self) -> fsm.Conditions:
        return self.snap.conditions(self._now(), self.generation, self.book.pending is not None)

    # ---------------------------------------------------------------- publishers

    def _lease_msg(self) -> ControlLease:
        """Built under the state lock; published outside it (never hold our lock inside rcl)."""
        m = ControlLease()
        m.header.stamp = self.get_clock().now().to_msg()
        m.instance = self.instance
        m.generation = self.generation
        self.lease_seq += 1
        m.seq = self.lease_seq
        m.allowed = self._allowed()
        m.inhibit_reason = "" if m.allowed else (self.phase or fsm.MODE_NAMES[self.mode])
        return m

    def _publish_lease(self) -> None:
        with self._lock:
            m = self._lease_msg()
        self._pub_lease.publish(m)

    def _publish_mode(self) -> None:
        with self._lock:
            m = ModeState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.instance = self.instance
            m.generation = self.generation
            m.mode = self.mode
            m.requested_mode = self.requested
            m.phase = self.phase
            pend = self.book.pending
            m.operation_id = pend.operation_id if pend else ""
            m.base_ready = rd.base_ready(self.snap, self._now())
            m.layer_ready = self.mode in (fsm.MAPPING, fsm.NAVIGATION)
            m.active_map_id, m.active_map_revision, m.active_map_sha256 = self.active_map
            allowed = self._allowed()
            m.manual_available = bool(allowed & LEASE_MANUAL)
            m.autonomous_available = bool(allowed & LEASE_AUTONOMOUS)
            m.fault_code = self.fault_code
            m.reason = self.reason
            m.last_survey_map_id, m.last_survey_revision = self.last_survey
        self._pub_mode.publish(m)

    # ------------------------------------------------------------------ services

    def _check_expected(self, inst: str, gen: int) -> str | None:
        if inst and inst != self.instance:
            return "expected_instance does not match the running supervisor"
        if gen and gen != self.generation:
            return f"expected_generation {gen} != current {self.generation}"
        return None

    def _srv_mode(self, req, res):
        with self._lock:
            why = self._check_expected(req.expected_instance, int(req.expected_generation))
            if why:
                res.accepted, res.message = False, why
                return res
            if req.target not in (ModeState.IDLE, ModeState.NAVIGATION):
                res.accepted, res.message = (
                    False,
                    "target must be IDLE or NAVIGATION (MAPPING is a survey START)",
                )
                return res
            kind = fsm.REQ_IDLE if req.target == ModeState.IDLE else fsm.REQ_NAVIGATION
            existing = (
                self.book.get(self.book._by_request.get(req.request_id, "")) if req.request_id else None
            )
            if existing is not None:
                res.accepted, res.operation_id, res.message = True, existing.operation_id, "duplicate request"
                return res
            d = fsm.admit(self.mode, kind, self._conditions())
            if not d.ok:
                res.accepted, res.message = False, d.reason
                return res
            map_id, rev, sha = "", 0, ""
            if kind == fsm.REQ_NAVIGATION:
                try:
                    map_id, rev, sha = rd.resolve_map(self.maps_dir, req.map_id, int(req.map_revision))
                except Exception as e:  # noqa: BLE001 - reported to the caller
                    res.accepted, res.message = False, f"map: {e}"
                    return res
                if self.mode == fsm.NAVIGATION and (map_id, rev, sha) == self.active_map:
                    # accepted means "poll this id": give the browser a finished operation, not ""
                    op, _ = self.book.submit(
                        req.request_id, kind, map_id=map_id, map_revision=rev, map_sha256=sha
                    )
                    self.book.finish(op.operation_id, ops.SUCCEEDED, "already on that map")
                    res.accepted, res.operation_id, res.message = True, op.operation_id, "already on that map"
                    return res
            op, _ = self.book.submit(req.request_id, kind, map_id=map_id, map_revision=rev, map_sha256=sha)
            target = fsm.IDLE if kind == fsm.REQ_IDLE else fsm.NAVIGATION
            self._begin_transaction(op, target, map_id, rev, sha)
            res.accepted, res.operation_id, res.message = True, op.operation_id, d.reason or "accepted"
            return res

    def _srv_survey(self, req, res):
        with self._lock:
            why = self._check_expected(req.expected_instance, int(req.expected_generation))
            if why:
                res.accepted, res.message = False, why
                return res
            kinds = {
                RequestSurvey.Request.START: fsm.REQ_SURVEY_START,
                RequestSurvey.Request.RETURNED: fsm.REQ_SURVEY_RETURNED,
                RequestSurvey.Request.SAVE: fsm.REQ_SURVEY_SAVE,
                RequestSurvey.Request.ABORT: fsm.REQ_SURVEY_ABORT,
            }
            kind = kinds.get(int(req.operation))
            if kind is None:
                res.accepted, res.message = False, "unknown survey operation"
                return res
            existing = (
                self.book.get(self.book._by_request.get(req.request_id, "")) if req.request_id else None
            )
            if existing is not None:
                res.accepted, res.operation_id, res.message = True, existing.operation_id, "duplicate request"
                return res
            d = fsm.admit(self.mode, kind, self._conditions())
            if not d.ok:
                res.accepted, res.message = False, d.reason
                return res
            if kind == fsm.REQ_SURVEY_START:
                try:
                    rd.check_map_id(req.map_id)
                except ValueError as e:
                    res.accepted, res.message = False, str(e)
                    return res
            op, _ = self.book.submit(req.request_id, kind, map_id=req.map_id, message=req.description)
            if kind == fsm.REQ_SURVEY_START:
                self._begin_transaction(op, fsm.MAPPING, req.map_id, 0, "")
            else:
                self.survey_op = (op.operation_id, kind)
                if kind == fsm.REQ_SURVEY_SAVE:
                    self.inhibit_manual = True
            res.accepted, res.operation_id, res.message = True, op.operation_id, "accepted"
            return res

    def _srv_get_op(self, req, res):
        op = self.book.get(req.operation_id)
        res.found = op is not None
        if op:
            res.state = self._op_msg(op)
        return res

    def _base_failure(self) -> str | None:
        """Why recovery cannot help: the base group (drives, mux, panel, EKF) is dead or never
        came up. The recovery transaction replaces the LAYER and needs a live mux to
        acknowledge the new generation (review Q17); it cannot rebuild the base. Say so,
        with the action that does, instead of starting a transaction that cannot finish."""
        base = self.groups.get("base")
        if base is None or base.poll() is not None:
            return "the base layer (drives, mux, panel, EKF) is not running"
        if self.fault_code in ("BASE_EXITED", "BOOT_ERROR", "BASE_NOT_READY"):
            return f"fault {self.fault_code}: the base layer failed"
        return None

    def _srv_recover(self, req, res):
        with self._lock:
            d = fsm.admit(self.mode, fsm.REQ_RECOVER, self._conditions())
            if not d.ok:
                res.success, res.message = False, d.reason
                return res
            why = self._base_failure()
            if why:
                res.success = False
                res.message = (
                    f"recovery cannot rebuild the base: {why}. Restart the service "
                    "(sudo systemctl restart amr.service) after correcting the cause; motion stays inhibited"
                )
                self._set(fsm.FAULT, "restart required", res.message, self.fault_code or "BASE_EXITED")
                return res
            op, _ = self.book.submit("", fsm.REQ_RECOVER)
            self._begin_transaction(op, fsm.IDLE, "", 0, "", from_fault=True)
            res.success, res.message = True, f"recovering, operation {op.operation_id}"
            return res

    def _op_msg(self, op: ops.Operation) -> OperationState:
        m = OperationState()
        m.operation_id, m.request_id, m.kind, m.phase = op.operation_id, op.request_id, op.kind, op.phase
        m.status = op.status
        m.map_id, m.map_revision, m.map_sha256, m.message = (
            op.map_id,
            op.map_revision,
            op.map_sha256,
            op.message,
        )
        m.started.sec = int(op.started)
        m.finished.sec = int(op.finished)
        return m

    # --------------------------------------------------------------- transaction

    def _begin_transaction(self, op, target, map_id, rev, sha, from_fault=False) -> None:
        self.txn = fsm.Transaction(op.operation_id, target, self.mode, self.generation + 1)
        self.txn.map_id, self.txn.map_revision, self.txn.map_sha256 = map_id, rev, sha
        self.txn.notes.append("from_fault" if from_fault else "")
        self.requested = target
        self._set(fsm.TRANSITIONING, "inhibiting")

    def _fail_active(self, code: str, why: str) -> None:
        """Terminal failure of whatever is in flight (review R25): the transaction's or the
        survey's own operation is finished - never a fresh one left beside a pending original."""
        if self.txn is not None:
            op_id = self.txn.operation_id
        elif self.survey_op is not None:
            op_id = self.survey_op[0]
        else:
            op_id = self.book.submit("", "fault", message=why)[0].operation_id
        self._fail(op_id, code, why)

    def _fail(self, op_id: str, code: str, why: str) -> None:
        self.get_logger().error(f"FAULT {code}: {why}")
        self.book.finish(op_id, ops.FAILED, why)
        if self.survey_op is not None and self.survey_op[0] != op_id:
            self.book.finish(self.survey_op[0], ops.FAILED, f"interrupted by fault {code}")
        if self.txn is not None and self.txn.operation_id != op_id:
            self.book.finish(self.txn.operation_id, ops.FAILED, f"interrupted by fault {code}")
        self.txn = None
        self.survey_op = None
        self.inhibit_manual = False
        self._pending_future = None
        self._set(fsm.FAULT, "fault", why, code)
        layer = self.groups.get("layer")
        if layer is not None and not layer.empty:
            self._start_worker(lambda: layer.stop())

    def _start_worker(self, fn) -> None:
        self._worker_result = None

        def run():
            try:
                self._worker_result = bool(fn())
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"worker: {e!r}")
                self._worker_result = False

        self._worker = threading.Thread(target=run, daemon=True)
        self._worker.start()

    def _worker_done(self) -> bool | None:
        if self._worker is None:
            return True
        if self._worker.is_alive():
            return None
        r = self._worker_result
        self._worker = None
        return r

    def _step_transaction(self, now: float) -> None:
        t = self.txn
        op = t.operation_id
        if t.step == fsm.INHIBIT:
            self.generation = t.generation
            self.snap.reset_layer_state()
            self.book.phase(op, "inhibiting")
            t.deadline = now + self.budget["mux_ack_s"]
            t.advance()
            return
        if t.step == fsm.WAIT_MUX_ACK:
            if self.snap.mux_acknowledged(self.generation, now):
                t.deadline = now + self.budget["still_wait_s"]
                t.advance()
                self.book.phase(op, "waiting for stillness")
            elif now > t.deadline:
                self._fail(op, "MUX_ACK_TIMEOUT", "mux did not acknowledge the inhibited generation")
            return
        if t.step == fsm.WAIT_STILL:
            c = self._conditions()
            if fsm.wheels_still(c) or "from_fault" in t.notes:
                t.advance()
                self.book.phase(op, "stopping old layer")
                layer = self.groups.get("layer")
                if layer is not None:
                    self._start_worker(lambda: layer.stop())
                else:
                    self._worker = None
                t.deadline = now + self.budget["layer_stop_s"] + 2.0
            elif now > t.deadline:
                self._fail(op, "STOP_TIMEOUT", "vehicle did not come to rest within the stop budget")
            return
        if t.step == fsm.STOP_OLD:
            done = self._worker_done()
            if done is None:
                if now > t.deadline:
                    self._fail(op, "LAYER_STOP_TIMEOUT", "old layer did not stop within budget")
                return
            layer = self.groups.pop("layer", None)
            if layer is not None and not layer.empty:
                self.groups["layer"] = layer
                self._fail(op, "LAYER_NOT_EMPTY", f"old layer still has processes: {layer.describe()}")
                return
            t.advance()
            return
        if t.step == fsm.CLEAR_CACHES:
            self.snap.reset_layer_state()
            self.active_map = ("", 0, "")
            t.advance()
            return
        if t.step == fsm.START_NEW:
            if t.target == fsm.IDLE:
                t.step = fsm.COMMIT
                return
            try:
                if t.target == fsm.MAPPING:
                    self.groups["layer"] = self._launch(
                        "layer",
                        "mapping_layer.launch.py",
                        maps_dir=self.maps_dir,
                        generation=self.generation,
                        internal="true",
                    )
                else:
                    self.groups["layer"] = self._launch(
                        "layer",
                        "navigation_layer.launch.py",
                        maps_dir=self.maps_dir,
                        map_id=t.map_id,
                        revision=t.map_revision,
                        generation=self.generation,
                        autostart="false",
                    )
            except Exception as e:  # noqa: BLE001
                self._fail(op, "SPAWN_FAILED", str(e))
                return
            t.deadline = now + self.budget["layer_start_s"]
            t.notes.append("lm:none")
            self.book.phase(op, "starting new layer")
            t.advance()
            return
        if t.step == fsm.WAIT_READY:
            layer = self.groups.get("layer")
            if layer is None or layer.poll() is not None:
                self._fail(
                    op,
                    "LAYER_EXITED",
                    f"new layer exited during startup: {layer.describe() if layer else '?'}",
                )
                return
            if now > t.deadline:
                self._fail(
                    op, "LAYER_START_TIMEOUT", f"new layer not ready in {self.budget['layer_start_s']:.0f} s"
                )
                return
            if t.target == fsm.MAPPING:
                self._ready_mapping(t, now)
            else:
                self._ready_navigation(t, now)
            return
        if t.step == fsm.COMMIT:
            self.txn = None
            self._pending_future = None
            if t.target == fsm.NAVIGATION:
                self.active_map = (t.map_id, t.map_revision, t.map_sha256)
            self.inhibit_manual = False
            msg = next((n for n in t.notes if n and not n.startswith("lm:") and n != "from_fault"), "ok")
            self.book.finish(
                op, ops.SUCCEEDED, msg, map_id=t.map_id, map_revision=t.map_revision, map_sha256=t.map_sha256
            )
            self._set(t.target, "", "")
            return

    def _ready_mapping(self, t: fsm.Transaction, now: float) -> None:
        if not self.snap.mapping_state_for(self.generation):
            return  # coordinator not up yet
        if self._pending_future is None:
            if not self._cli["start"].service_is_ready() or now < getattr(self, "_retry_at", 0.0):
                return
            req = StartSurvey.Request()
            req.map_id = t.map_id
            req.description = self.book.get(t.operation_id).message
            self._pending_future = self._cli["start"].call_async(req)
            self.book.phase(t.operation_id, "starting survey")
            return
        if not self._pending_future.done():
            return
        r = self._pending_future.result()
        self._pending_future = None
        if r is None or not r.accepted:
            msg = getattr(r, "message", "no response")
            if r is not None and msg.startswith("not ready"):
                # SLAM has not produced map->odom / a map yet (plan §5.4 step 3):
                # keep asking until the layer start budget runs out.
                self.book.phase(t.operation_id, f"waiting: {msg}")
                self._retry_at = now + 1.0
                return
            self._fail(t.operation_id, "SURVEY_START_REFUSED", msg)
            return
        t.notes.append(r.message)
        t.step = fsm.COMMIT

    def _ready_navigation(self, t: fsm.Transaction, now: float) -> None:
        stage = [n for n in t.notes if n.startswith("lm:")][-1]
        if self._pending_future is not None:
            if not self._pending_future.done():
                return
            r = self._pending_future.result()
            self._pending_future = None
            ok = bool(getattr(r, "success", False))
            if not ok:
                self._fail(t.operation_id, "LIFECYCLE_STARTUP", f"lifecycle {stage} refused")
                return
            t.notes[t.notes.index(stage)] = {
                "lm:loc": "lm:loc-done",
                "lm:nav": "lm:nav-done",
                "lm:check": "lm:ok",
            }[stage]
            return
        if stage == "lm:none":
            if self._cli["lm_loc"].service_is_ready():
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request.STARTUP
                self._pending_future = self._cli["lm_loc"].call_async(req)
                t.notes[t.notes.index(stage)] = "lm:loc"
                self.book.phase(t.operation_id, "activating localization")
            return
        if stage == "lm:loc-done":
            if self._cli["lm_nav"].service_is_ready():
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request.STARTUP
                self._pending_future = self._cli["lm_nav"].call_async(req)
                t.notes[t.notes.index(stage)] = "lm:nav"
                self.book.phase(t.operation_id, "activating navigation")
            return
        if stage == "lm:nav-done":
            if self._cli["lm_nav_active"].service_is_ready():
                self._pending_future = self._cli["lm_nav_active"].call_async(Trigger.Request())
                t.notes[t.notes.index(stage)] = "lm:check"
            return
        if stage == "lm:ok":
            if self.snap.run_state_for(self.generation) and self.snap.loc_state_for(self.generation):
                t.step = fsm.COMMIT

    # ---------------------------------------------------------- survey follow-ups

    def _step_survey(self, now: float) -> None:
        op_id, kind = self.survey_op
        name = {
            fsm.REQ_SURVEY_RETURNED: "returned",
            fsm.REQ_SURVEY_SAVE: "save",
            fsm.REQ_SURVEY_ABORT: "abort",
        }[kind]
        if self._pending_future is None:
            cli = self._cli[name]
            if not cli.service_is_ready():
                self._fail(op_id, "COORDINATOR_UNAVAILABLE", f"{INTERNAL}/{name} is not available")
                return
            if name == "save":
                req = SaveMap.Request()
                req.note = self.book.get(op_id).message
                self._save_deadline = now + self.budget["save_s"]
            else:
                req = Trigger.Request()
                self._save_deadline = now + self.budget["survey_rpc_s"]
            self._pending_future = cli.call_async(req)
            self.book.phase(op_id, name)
            return
        if not self._pending_future.done():
            if now > self._save_deadline:
                # _fail drops the future: a late answer can no longer finish or advance anything
                if name == "save":
                    self._fail(
                        op_id, "SAVE_UNKNOWN_OUTCOME", "save did not answer within budget; outcome unknown"
                    )
                else:
                    self._fail(op_id, "SURVEY_RPC_TIMEOUT", f"{INTERNAL}/{name} did not answer within budget")
            return
        r = self._pending_future.result()
        self._pending_future = None
        self.survey_op = None
        if name == "returned":
            ok = bool(getattr(r, "success", False))
            self.book.finish(op_id, ops.SUCCEEDED if ok else ops.FAILED, getattr(r, "message", ""))
            return
        if name == "save":
            self.inhibit_manual = False
            if r is None or not r.ok:
                self.book.finish(op_id, ops.FAILED, getattr(r, "message", "no response"))
                return
            rev = int(r.revision)
            map_id = self.snap.mapping_map_id
            sha = rd.bundle_sha(self.maps_dir, map_id, rev)
            self.last_survey = (map_id, rev)
            # The bundle is published; the SAME operation now carries the return
            # to IDLE, so the browser sees one SUCCEEDED only once the vehicle is
            # back in a stable mode (no BUSY window for the next request).
            self.book.phase(op_id, f"saved rev{rev}; returning to idle")
            self._begin_transaction(self.book.get(op_id), fsm.IDLE, map_id, rev, sha)
            self.txn.notes.append(r.message)
            return
        if name == "abort":
            self.book.phase(op_id, "aborted; returning to idle")
            self._begin_transaction(self.book.get(op_id), fsm.IDLE, "", 0, "")

    # ------------------------------------------------------------------ the loop

    def _boot(self) -> None:
        p = self.get_parameter
        if p("web").value:
            argv = [
                sys.executable,
                "-m",
                "amr_web.web_node",
                "--ros-args",
                "-p",
                f"maps_dir:={self.maps_dir}",
            ]
            self.groups["web"] = Group.spawn(
                "web", argv, env=self._env(), log_path=os.path.join(self._logs, "web.log")
            )
        base_args = {"real": str(self.real).lower(), "supervised": "true"}
        if self.real:
            base_args["lidar"] = str(bool(p("lidar").value)).lower()
        else:
            base_args.update(
                slip_noise_std=p("slip_noise_std").value,
                scan_synth="true",
                clutter_count=p("clutter_count").value,
            )
            if p("world_yaml").value:
                base_args["world_yaml"] = p("world_yaml").value
        self.groups["base"] = self._launch("base", "base.launch.py", **base_args)
        if p("foxglove").value:
            argv = ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]
            self.groups["foxglove"] = Group.spawn(
                "foxglove", argv, env=self._env(), log_path=os.path.join(self._logs, "foxglove.log")
            )
        self._boot_deadline = self._now() + self.budget["base_ready_s"]

    def _reap(self, now: float) -> None:
        """An unrequested exit of base or the layer is a fault - once. The dead
        group stays in the table, marked, so the recovery transaction's STOP_OLD
        verifies it is empty (a leader that died may have left members)."""
        for role, g in list(self.groups.items()):
            if g.poll() is None or g.requested_stop:
                continue
            g.requested_stop = True  # handled; never reap it again
            if role in ("base", "layer"):
                self._fail_active(f"{role.upper()}_EXITED", f"{g.describe()} exited unrequested")
            else:
                self.get_logger().warn(f"{g.describe()} exited; optional group, not restarted")
                self.groups.pop(role, None)

    def _loop(self) -> None:
        with self._lock:
            try:
                self._boot()
            except Exception as e:  # noqa: BLE001 - a spawn failure is a fault, not a dead worker thread
                self.get_logger().error(f"boot error: {e!r}")
                self._boot_deadline = self._now()
                self._fail_active("BOOT_ERROR", f"boot failed: {e!r}")
        period = 0.1
        while not self._stop.is_set():
            t0 = self._now()
            with self._lock:
                try:
                    self._tick(t0)
                except Exception as e:  # noqa: BLE001 - the loop must keep publishing the (inhibited) lease
                    self.get_logger().error(f"loop error: {e!r}")
                    try:
                        self._fail_active("LOOP_ERROR", repr(e))
                    except Exception as e2:  # noqa: BLE001 - last resort: nothing in flight survives
                        self.get_logger().error(f"fault handling failed: {e2!r}")
                        self.txn, self.survey_op, self._pending_future = None, None, None
                        self._set(fsm.FAULT, "loop error", repr(e), "LOOP_ERROR")
            try:
                self._publish_lease()  # outside the lock
            except Exception as e:  # noqa: BLE001 - a dead loop would stop IDLE/FAULT state reporting
                self.get_logger().error(f"lease publish failed: {e!r}")
            time.sleep(max(0.0, period - (self._now() - t0)))

    def _tick(self, now: float) -> None:
        self._reap(now)
        if self.mode == fsm.STARTING:
            if rd.base_ready(self.snap, now) and self.snap.mux_acknowledged(self.generation, now):
                self._set(fsm.IDLE, "", "")
            elif now > self._boot_deadline:
                self._set(fsm.FAULT, "base not ready", rd.base_missing(self.snap, now), "BASE_NOT_READY")
            return
        if self.mode == fsm.STOPPING:
            return
        if self.txn is not None:
            self._step_transaction(now)
        elif self.survey_op is not None:
            self._step_survey(now)
        elif self.mode == fsm.FAULT:
            self._worker_done()

    # --------------------------------------------------------------- shutdown

    def shutdown(self) -> None:
        """Ordered teardown (§8.1): revoke, stop layer, stop base (drives disarm), web, bridge.

        The loop is asked to stop first and the state lock is taken with a
        bounded wait: a wedged loop must not keep the children (and the drives)
        alive. The lease stops being published either way, which is what
        actually revokes motion."""
        self._stop.set()
        self._stopping = True
        got = self._lock.acquire(timeout=2.0)
        try:
            self.mode, self.phase = fsm.STOPPING, "stopping"
            if got:
                self._set(fsm.STOPPING, "stopping")
        finally:
            if got:
                self._lock.release()
        if not got:
            self.get_logger().error("shutdown: state lock not acquired in 2 s; stopping children anyway")
        try:
            self._publish_lease() if got else None
        except Exception:  # noqa: BLE001
            pass
        for role in ("layer", "base", "foxglove", "web"):
            g = self.groups.get(role)
            if g is None:
                continue
            ok = g.stop()
            self.get_logger().info(f"stopped {role}: {'clean' if ok else 'NOT EMPTY'} ({g.describe()})")
        pend = self.book.pending
        if pend:
            self.book.finish(pend.operation_id, ops.INTERRUPTED, "service stopped")


def main(args=None) -> None:
    domains.require_vehicle_domain("amr_supervisor") if os.environ.get("AMR_SUPERVISOR_SIM") != "1" else None
    rclpy.init(args=args)
    node = Supervisor()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    stop = threading.Event()

    def on_signal(signum, _frame):
        node.get_logger().info(f"signal {signum}: shutting down")
        stop.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    faulthandler.register(signal.SIGUSR1, all_threads=True)  # `kill -USR1 <pid>` dumps every thread's stack
    node._thread.start()
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.2)
    finally:
        t0 = time.monotonic()
        node.shutdown()
        node.get_logger().info(f"teardown: groups stopped in {time.monotonic() - t0:.1f} s")
        executor.shutdown(timeout_sec=2.0)
        node.get_logger().info(f"teardown: executor down at {time.monotonic() - t0:.1f} s")
        node._thread.join(timeout=2.0)
        print(f"teardown: loop joined at {time.monotonic() - t0:.1f} s", flush=True)
        try:
            node.destroy_node()
        except Exception as e:  # noqa: BLE001
            print(f"teardown: destroy_node {e!r}", flush=True)
        print(f"teardown: node destroyed at {time.monotonic() - t0:.1f} s", flush=True)
        rclpy.try_shutdown()
        print(f"teardown: done at {time.monotonic() - t0:.1f} s", flush=True)


if __name__ == "__main__":
    main()
