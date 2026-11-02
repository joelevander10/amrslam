"""amr_supervisor: the main process of amr.service (unified plan §3, §5, §8).

Owns four process groups - web, base, an optional Foxglove bridge, and at most
one mode layer (mapping or navigation) - and the single source of motion
authority, /amr/control_lease. Boots STARTING -> IDLE with base and web up and
no layer, then on to LINE unless the profile is trackless-only (tracked false).
Mode changes and survey operations are asynchronous operations (OperationState)
driven by one serial loop; every step has a deadline and a
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
from agv_core import config
from geometry_msgs.msg import Pose2D
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from amr_bringup import domains, logs, sdnotify, uptime
from amr_bringup import mode_fsm as fsm
from amr_bringup import operations as ops
from amr_bringup import readiness as rd
from amr_bringup.process_supervisor import Group
from amr_interfaces.msg import (
    CommissioningState,
    ControlLease,
    DriveStatus,
    Event,
    LineState,
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

LEASE_MANUAL, LEASE_AUTONOMOUS, LEASE_COMMISSIONING, LEASE_LINE = 1, 2, 4, 8
# The web (and foxglove) come back by themselves: 10 s apart, three tries, then the
# operator is told to restart the service rather than watching a dead page for ever.
RESPAWN_BACKOFF_S = 10.0
RESPAWN_TRIES = 3
# How long a boot-time event waits for a listener (the web is spawned by this process
# and takes a few seconds to subscribe) before it is given up on and only logged.
DEFER_EVENT_S = 60.0

# Mode replacement targets a client may ask for. MAPPING is absent deliberately: a survey
# starts through /amr/supervisor/survey, not here. Keep the two dicts inverse of each other.
_REQUEST_OF_TARGET = {
    ModeState.IDLE: fsm.REQ_IDLE,
    ModeState.NAVIGATION: fsm.REQ_NAVIGATION,
    ModeState.LINE: fsm.REQ_LINE,
}
_MODE_OF_REQUEST = {
    fsm.REQ_IDLE: fsm.IDLE,
    fsm.REQ_NAVIGATION: fsm.NAVIGATION,
    fsm.REQ_LINE: fsm.LINE,
}
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
        # continue-survey: added to layer_start_s while slam_toolbox loads a saved pose graph
        # (boiacca-map-1 rev1 is 94 MB; the session's own load timeout is 120 s)
        dp("continue_load_s", 150.0)
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
                "continue_load_s",
            )
        }

        # The product, from the profile's top-level `tracked` (config.py note 5):
        # a tape AGV enters LINE by itself once the base is up and refuses maps and
        # routes; a trackless one boots to IDLE and refuses LINE; a vehicle carrying
        # both boots to LINE too (2026-10-08: tracked is the default product, trackless
        # is the operator's switch on Home). Read once, here.
        self.tracked = config.TRACKED  # True tape | False slam | None both
        self.admit_params = fsm.Params(tracked=self.tracked)
        self._auto_line_done = False
        self._boot_reported = False  # the end-of-boot operator event fires once per boot
        self._verdict: uptime.Verdict | None = None  # how the PREVIOUS run ended
        self.disk = rd.DiskStatus(-1, rd.DISK_OK, "")
        self._disk_level = rd.DISK_OK
        self._t_disk = 0.0
        self._deferred: list[tuple[int, str, str, float]] = []
        # Optional groups the supervisor brings back by itself (plan phase 3.1).
        self._respawnable: dict[str, list[str]] = {}
        self._respawn_at: dict[str, float] = {}
        self._respawn_tries: dict[str, int] = {}
        self._notify = sdnotify.Notifier()

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
        self._continue_hint: tuple[float, float, float] | None = None  # START of a continued survey
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
        # Without this the LINE transaction can never COMMIT: _ready_line waits
        # on snap.line_state_for(generation) and nothing would ever feed it, so
        # a swap into LINE would sit there until LAYER_START_TIMEOUT.
        self.create_subscription(
            LineState, "/amr/line_state", self.snap.on_line, LATCHED, callback_group=g
        )
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
        # The layer about to start owns THIS generation's footprint file; earlier ones
        # belong to layers that are gone (power-loss plan W2).
        logs.sweep_generation_files(self.state_dir, self.generation)
        return g

    def _event_when_heard(self, level: int, code: str, text: str) -> None:
        """Hold a BOOT-TIME event until somebody is listening.

        /amr/events is volatile: a late joiner gets nothing. The web is started BY this
        supervisor and subscribes seconds after boot, so the two events that matter most
        - "power was lost last time" and the boot report - were published into an empty
        room and lost (caught in sim, 2026-09-22). Held until the topic has a subscriber,
        or logged alone after the deadline if nothing ever listens."""
        self._deferred.append((level, code, text, self._now() + DEFER_EVENT_S))
        self._flush_deferred()

    def _flush_deferred(self) -> None:
        if not self._deferred:
            return
        try:
            heard = self._pub_event.get_subscription_count() > 0
        except Exception:  # noqa: BLE001 - a counting failure must not lose the event
            heard = True
        now, keep = self._now(), []
        for level, code, text, deadline in self._deferred:
            if heard:
                self._event(level, code, text)
            elif now < deadline:
                keep.append((level, code, text, deadline))
            else:
                self.get_logger().warn(f"{code} not delivered (nobody subscribed): {text}")
        self._deferred = keep

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
        self._mark()
        self._publish_mode()

    def _allowed(self) -> int:
        if self.txn is not None or self.mode in (fsm.STARTING, fsm.TRANSITIONING, fsm.FAULT, fsm.STOPPING):
            return 0
        if self.inhibit_manual:
            return 0
        if self.mode == fsm.NAVIGATION:
            return LEASE_MANUAL | LEASE_AUTONOMOUS
        if self.mode == fsm.LINE:
            # The selector decides who drives (2026-10-02): MANUAL jogs (pendant, browser)
            # in LINE mode too, AUTO is the tape follower's. The two cannot fight - the mux
            # reads the panel first, and the follower ends its run when the selector
            # leaves AUTO.
            return LEASE_MANUAL | LEASE_LINE
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
            m.layer_ready = self.mode in fsm.LAYER_MODES
            m.active_map_id, m.active_map_revision, m.active_map_sha256 = self.active_map
            allowed = self._allowed()
            m.manual_available = bool(allowed & LEASE_MANUAL)
            m.autonomous_available = bool(allowed & LEASE_AUTONOMOUS)
            m.line_available = bool(allowed & LEASE_LINE)
            m.fault_code = self.fault_code
            m.reason = self.reason
            m.product = {True: "tape", False: "slam"}.get(self.tracked, "both")
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
            kind = _REQUEST_OF_TARGET.get(req.target)
            if kind is None:
                res.accepted, res.message = (
                    False,
                    "target must be IDLE, NAVIGATION or LINE (MAPPING is a survey START)",
                )
                return res
            existing = (
                self.book.get(self.book._by_request.get(req.request_id, "")) if req.request_id else None
            )
            if existing is not None:
                res.accepted, res.operation_id, res.message = True, existing.operation_id, "duplicate request"
                return res
            d = fsm.admit(self.mode, kind, self._conditions(), self.admit_params)
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
            self._begin_transaction(op, _MODE_OF_REQUEST[kind], map_id, rev, sha)
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
            d = fsm.admit(self.mode, kind, self._conditions(), self.admit_params)
            if not d.ok:
                res.accepted, res.message = False, d.reason
                return res
            # No room to write the result (power-loss plan W3). Refused for the two
            # operations that CREATE a map; never for anything that is already running.
            if kind in (fsm.REQ_SURVEY_START, fsm.REQ_SURVEY_SAVE) and self.disk.blocks_new_work:
                res.accepted, res.message = (
                    False,
                    f"disk full: {self.disk.free_mb} MB free on {self.disk.path}; "
                    "delete old maps or reports first",
                )
                return res
            cont_rev, cont_sha = int(req.continue_revision), ""
            if kind == fsm.REQ_SURVEY_START:
                try:
                    rd.check_map_id(req.map_id)
                    if cont_rev > 0:  # refused here, before the running layer is torn down
                        cont_sha = rd.continuable(self.maps_dir, req.map_id, cont_rev)
                        rd.check_hint(req.hint.x, req.hint.y, req.hint.theta)
                except ValueError as e:
                    res.accepted, res.message = False, str(e)
                    return res
            op, _ = self.book.submit(req.request_id, kind, map_id=req.map_id, message=req.description)
            if kind == fsm.REQ_SURVEY_START:
                self._continue_hint = (
                    (float(req.hint.x), float(req.hint.y), float(req.hint.theta)) if cont_rev > 0 else None
                )
                self._begin_transaction(op, fsm.MAPPING, req.map_id, cont_rev, cont_sha)
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
        if self.fault_code in ("BASE_EXITED", "BOOT_ERROR"):
            return f"fault {self.fault_code}: the base layer failed"
        # BASE_NOT_READY with the group alive is a late base, not a dead one: the
        # tick returns to IDLE by itself once it is ready, and Recover may try too.
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

    def _auto_enter_line(self) -> None:
        """LINE is the default mode of every vehicle that carries the tape product
        (tracked true, and since 2026-10-08 tracked null = both): start the swap right
        after boot, once. Only a trackless-only vehicle (tracked false) stays in IDLE.
        No panel rule here - nothing can be moving at boot, the mux inhibits
        through the transition, and the follower still needs the physical Start under
        AUTO before a wheel turns. Leaving LINE later (the Home switch to trackless) is
        the operator's request and is not undone."""
        if self.tracked is False or self._auto_line_done or self.txn is not None:
            return
        self._auto_line_done = True
        op, _ = self.book.submit(f"boot-line-{self.instance[:8]}", fsm.REQ_LINE)
        self.get_logger().info(f"profile tracked={self.tracked}: entering LINE (the default mode)")
        self._begin_transaction(op, fsm.LINE, "", 0, "")

    def _mark(self) -> None:
        """Rewrite the running marker (power-loss plan W1). One small atomic write per
        mode change; a cut between writes costs at most the last transition."""
        pend = self.book.pending
        uptime.write(
            self.state_dir,
            self.instance,
            fsm.MODE_NAMES.get(self.mode, str(self.mode)),
            pend.operation_id if pend else "",
            getattr(self.snap, "run_mission", ""),
        )

    def _check_disk(self) -> None:
        """Watch the state and maps filesystems (power-loss plan W3). Edges only, and
        the level is published in ModeState so the web can raise the standing alarm."""
        st = rd.disk_status([self.state_dir, self.maps_dir])
        self.disk = st
        if st.level == self._disk_level:
            return
        was, self._disk_level = self._disk_level, st.level
        if st.level == rd.DISK_STOP:
            self._event(Event.ERROR, "DISK_FULL", f"{st.free_mb} MB free on {st.path}")
        elif st.level == rd.DISK_WARN:
            self._event(Event.WARN, "DISK_LOW", f"{st.free_mb} MB free on {st.path}")
        elif st.level == rd.DISK_UNKNOWN:
            self._event(Event.WARN, "DISK_LOW", "storage free space unknown: no watched filesystem could be read")
        elif was != rd.DISK_OK:
            self._event(Event.INFO, "DISK_LOW", f"storage is no longer low ({st.free_mb} MB free)")

    def _boot_report(self, now: float, missing: str) -> None:
        """One event at the end of boot, in words the operator can act on. `missing` is
        base_missing()'s list; each item is named with what the operator does about it,
        so the 70 s safety-reset case (2026-09-22) reads as a sentence, not a FAULT screen."""
        if self._boot_reported:
            return
        self._boot_reported = True
        after = ""
        if self._verdict is not None and self._verdict.kind == uptime.POWER_LOSS:
            after = "after a power loss, "
        if not missing:
            self._event_when_heard(Event.INFO, "BOOT_READY", f"{after}the vehicle is ready")
            return
        actions = {
            "drives": "press the safety reset on the cabinet",
            "panel": "check the control panel and its I/O island",
            "wheel feedback": "press the safety reset on the cabinet",
            "mux": "call the engineer: the command mux did not start",
        }
        parts = [f"{item} ({actions.get(item, 'see the Alarms page')})" for item in missing.split(", ")]
        self._event_when_heard(Event.WARN, "BASE_NOT_READY", f"{after}not ready: " + "; ".join(parts))

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
                # Explicit dispatch, never a fallthrough: an unhandled target must fail
                # loudly here rather than silently start the wrong layer.
                start = self._LAYER_START.get(t.target)
                if start is None:
                    raise KeyError(f"no layer launch for mode {fsm.MODE_NAMES.get(t.target, t.target)}")
                self.groups["layer"] = start(self, t)
            except Exception as e:  # noqa: BLE001
                self._fail(op, "SPAWN_FAILED", str(e))
                return
            t.deadline = now + self._layer_start_budget(t)
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
                    op, "LAYER_START_TIMEOUT", f"new layer not ready in {self._layer_start_budget(t):.0f} s"
                )
                return
            ready = self._LAYER_READY.get(t.target)
            if ready is None:
                self._fail(
                    op,
                    "LAYER_NO_READINESS",
                    f"no readiness check for mode {fsm.MODE_NAMES.get(t.target, t.target)}",
                )
                return
            ready(self, t, now)
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

    def _layer_start_budget(self, t: fsm.Transaction) -> float:
        """A continued survey also loads a saved pose graph before it is ready."""
        extra = self.budget["continue_load_s"] if t.target == fsm.MAPPING and t.map_revision > 0 else 0.0
        return self.budget["layer_start_s"] + extra

    def _start_mapping(self, t: fsm.Transaction):
        return self._launch(
            "layer",
            "mapping_layer.launch.py",
            maps_dir=self.maps_dir,
            generation=self.generation,
            internal="true",
        )

    def _start_navigation(self, t: fsm.Transaction):
        return self._launch(
            "layer",
            "navigation_layer.launch.py",
            maps_dir=self.maps_dir,
            map_id=t.map_id,
            revision=t.map_revision,
            generation=self.generation,
            autostart="false",
        )

    def _start_line(self, t: fsm.Transaction):
        return self._launch(
            "layer", "line_layer.launch.py", generation=self.generation, real=str(self.real).lower()
        )

    def _ready_line(self, t: fsm.Transaction, now: float) -> None:
        """The line layer is ready as soon as it reports its own state on THIS generation.
        No RPC: unlike a survey, nothing has to be handed to it to start (Increment 1)."""
        if not self.snap.line_state_for(self.generation):
            return
        t.step = fsm.COMMIT

    def _ready_mapping(self, t: fsm.Transaction, now: float) -> None:
        if not self.snap.mapping_state_for(self.generation):
            return  # coordinator not up yet
        if self._pending_future is None:
            if not self._cli["start"].service_is_ready() or now < getattr(self, "_retry_at", 0.0):
                return
            req = StartSurvey.Request()
            req.map_id = t.map_id
            req.description = self.book.get(t.operation_id).message
            if t.map_revision > 0 and self._continue_hint is not None:
                req.continue_revision = int(t.map_revision)
                hx, hy, hyaw = self._continue_hint
                req.hint = Pose2D(x=hx, y=hy, theta=hyaw)
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

    # Layer dispatch. Every mode in fsm.LAYER_MODES must appear in BOTH tables; a missing
    # entry fails the transaction loudly (SPAWN_FAILED / LAYER_NO_READINESS) instead of
    # silently starting the wrong layer, which is what the old bare `else` did.
    _LAYER_START = {
        fsm.MAPPING: _start_mapping,
        fsm.NAVIGATION: _start_navigation,
        fsm.LINE: _start_line,
    }
    _LAYER_READY = {
        fsm.MAPPING: _ready_mapping,
        fsm.NAVIGATION: _ready_navigation,
        fsm.LINE: _ready_line,
    }

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
        # Before anything else: what happened last time? A marker that survived is the
        # only evidence a power cut leaves, and the operator needs it in the first
        # sentence, not after a failed arm (power-loss plan W1).
        self._verdict = uptime.verdict(self.state_dir)
        if self._verdict is not None:
            self._event_when_heard(
                Event.ERROR if self._verdict.kind == uptime.POWER_LOSS else Event.WARN,
                "UNCLEAN_SHUTDOWN" if self._verdict.kind == uptime.POWER_LOSS else "SERVICE_CRASHED",
                self._verdict.sentence(),
            )
            self.get_logger().warn(self._verdict.sentence())
        self._mark()
        # Nothing is live at boot, so every per-generation costmap footprint left behind
        # is stale (power-loss plan W2); they accumulated one per mode change.
        stale = logs.sweep_generation_files(self.state_dir)
        if stale:
            self.get_logger().info(f"swept {len(stale)} stale generation files")
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
            self._respawnable["web"] = argv
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
            self._respawnable["foxglove"] = argv
            self.groups["foxglove"] = Group.spawn(
                "foxglove", argv, env=self._env(), log_path=os.path.join(self._logs, "foxglove.log")
            )
        self._boot_deadline = self._now() + self.budget["base_ready_s"]
        self._boot_reported = False

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
            elif role in self._respawnable:
                # The web is the operator's only window and was the least protected
                # process in the stack: bring it back on a bounded backoff, then say
                # loudly that it is gone (plan phase 3.1).
                self.get_logger().warn(f"{g.describe()} exited; respawning in {RESPAWN_BACKOFF_S:.0f} s")
                self.groups.pop(role, None)
                self._respawn_at[role] = now + RESPAWN_BACKOFF_S
            else:
                self.get_logger().warn(f"{g.describe()} exited; optional group, not restarted")
                self.groups.pop(role, None)
        self._respawn_due(now)

    def _respawn_due(self, now: float) -> None:
        for role, when in list(self._respawn_at.items()):
            if now < when:
                continue
            del self._respawn_at[role]
            tries = self._respawn_tries.get(role, 0) + 1
            self._respawn_tries[role] = tries
            if tries > RESPAWN_TRIES:
                self.get_logger().error(f"{role} died {tries - 1} times; not restarting it again")
                self._event(Event.ERROR, "WEB_DOWN" if role == "web" else "SPAWN_FAILED",
                            f"{role} keeps stopping; restart the service")
                continue
            try:
                self.groups[role] = Group.spawn(
                    role,
                    self._respawnable[role],
                    env=self._env(),
                    log_path=os.path.join(self._logs, f"{role}.log"),
                )
                self.get_logger().warn(f"{role} respawned (attempt {tries} of {RESPAWN_TRIES})")
            except Exception as e:  # noqa: BLE001 - an optional group must never fault the vehicle
                self.get_logger().error(f"{role} respawn failed: {e!r}")
                self._respawn_at[role] = now + RESPAWN_BACKOFF_S

    def _loop(self) -> None:
        with self._lock:
            try:
                self._boot()
            except Exception as e:  # noqa: BLE001 - a spawn failure is a fault, not a dead worker thread
                self.get_logger().error(f"boot error: {e!r}")
                self._boot_deadline = self._now()
                self._fail_active("BOOT_ERROR", f"boot failed: {e!r}")
        # systemd: the unit is Type=notify, so nothing downstream starts until the
        # groups are spawned, and WatchdogSec restarts a WEDGED loop (plan phase 3.2).
        self._notify.ready()
        period = 0.1
        t_cap = 0.0
        while not self._stop.is_set():
            t0 = self._now()
            self._notify.watchdog()
            self._flush_deferred()
            if t0 - self._t_disk >= 30.0:
                self._t_disk = t0
                self._check_disk()
            if t0 - t_cap >= 60.0:
                t_cap = t0
                # A single bad run can fill the disk between restarts; children append,
                # so truncating underneath them is safe (power-loss plan W2).
                for name in logs.cap_dir(self._logs):
                    self.get_logger().warn(f"{name} passed the size cap and was truncated")
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
                self._boot_report(now, "")
                self._set(fsm.IDLE, "", "")
                self._auto_enter_line()
            elif now > self._boot_deadline:
                missing = rd.base_missing(self.snap, now)
                self._boot_report(now, missing)
                self._set(fsm.FAULT, "base not ready", missing, "BASE_NOT_READY")
            return
        if self.mode == fsm.STOPPING:
            return
        if self.mode == fsm.FAULT and self.fault_code == "BASE_NOT_READY" and self.txn is None:
            # The base came up after the boot budget (2026-09-22: the drives sat in
            # "Switch on disabled" until the safety reset was pressed 70 s after boot).
            # The group is alive and healthy, nothing needs rebuilding: enter IDLE the
            # same way STARTING does instead of demanding a service restart.
            if rd.base_ready(self.snap, now) and self.snap.mux_acknowledged(self.generation, now):
                self.get_logger().info("base ready after the boot budget")
                self._boot_report(now, "")
                self._set(fsm.IDLE, "", "")
                self._auto_enter_line()
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
        self._notify.stopping()
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
        # LAST, after the children are down: a kill during teardown must still read as
        # unclean next time (power-loss plan W1). Nothing may be added below this line.
        uptime.clear(self.state_dir)


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
