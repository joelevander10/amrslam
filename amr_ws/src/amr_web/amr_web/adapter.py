"""The ROS side of the web app: one rclpy node, explicit typed calls, no wheels.

Services are called with a bounded wait from Flask's request threads while a
MultiThreadedExecutor spins the node in the background. A service that is not
running (e.g. the executor before nav.launch) answers (False, "... unavailable")
instead of hanging.
"""

from __future__ import annotations

import math
import threading
import time
import traceback
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from amr_interfaces.msg import (
    CommissioningState,
    ControlLease,
    DriveStatus,
    Event,
    IoImage,
    LocalizationState,
    ManualCommand,
    MappingState,
    ModeState,
    MuxState,
    PanelState,
    PpStatus,
    SurveyMoveState,
)
from amr_interfaces.srv import (
    GetOperation,
    PlanCommissioning,
    RequestMode,
    RequestSurvey,
    RunMission,
    SaveMap,
    StartSurvey,
    SurveyMove,
)
from amr_web import live

LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
SCAN_PERIOD_S = 0.1  # live-view scan projection rate cap (the page redraws at 5 Hz)
MAP_ODOM_FRESH_S = 2.0  # newest map->odom stamp older than this: no layer is localising
STATE_NAMES = {0: "IDLE", 1: "MAPPING", 2: "RETURN_REVIEW", 3: "SAVING", 4: "SAVED"}
LOC_NAMES = {0: "UNLOCALIZED", 1: "CHECKING", 2: "READY", 3: "LOST"}
RUN_NAMES = {0: "IDLE", 1: "READY", 2: "EXECUTING", 3: "PAUSED", 4: "BLOCKED", 5: "FAULT", 6: "DONE"}
MODE_NAMES = {
    0: "STARTING",
    1: "IDLE",
    2: "MAPPING",
    3: "NAVIGATION",
    4: "TRANSITIONING",
    5: "FAULT",
    6: "STOPPING",
}
OP_NAMES = {0: "PENDING", 1: "SUCCEEDED", 2: "FAILED", 3: "INTERRUPTED"}
MUX_NAMES = {0: "none", 1: "teleop", 2: "follow", 3: "rotate", 4: "manual", 5: "commissioning", 6: "pendant"}
RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)


def _msg_to_dict(msg) -> dict[str, Any]:
    out = {}
    for name in msg.get_fields_and_field_types():
        v = getattr(msg, name)
        if hasattr(v, "get_fields_and_field_types"):
            v = _msg_to_dict(v)
        elif isinstance(v, (list, tuple)):
            v = [x if isinstance(x, (int, float, str, bool)) else _msg_to_dict(x) for x in v]
        out[name] = v
    return out


def initial_pose_mismatch(mode, map_id: str, map_revision: int, generation: int, sha256: str = "") -> str:
    """Why a pose drawn on (map_id, map_revision) under `generation` must not be applied; "" if it may."""
    if not mode:
        return "no supervisor state"
    if mode.get("mode_name") != "NAVIGATION" or not mode.get("active_map_id"):
        return "no navigation map is active"
    if (map_id, int(map_revision)) != (mode["active_map_id"], int(mode.get("active_map_revision", -1))):
        return (
            f"the pose was drawn on {map_id} rev{map_revision}, but the vehicle is running "
            f"{mode['active_map_id']} rev{mode.get('active_map_revision')}"
        )
    active_sha = mode.get("active_map_sha256") or ""
    if sha256 and active_sha and sha256 != active_sha:
        return "the viewed map's content differs from the active map (sha256)"
    if int(generation) != int(mode.get("generation", -1)):
        return "the vehicle changed mode since the map was shown; look again"
    return ""


class RosAdapter(Node):
    def __init__(self) -> None:
        super().__init__("amr_web")
        self._lock = threading.Lock()
        self._mapping: dict | None = None
        self._survey_move: dict | None = None  # preset survey moves (survey_move_node)
        self._survey_move_t = 0.0
        self._loc: dict | None = None
        self._run: dict | None = None
        self._mapping_t = self._loc_t = self._run_t = 0.0
        self._mode: dict | None = None
        self._mode_t = 0.0
        self._lease: dict | None = None
        self._lease_t = 0.0
        self._panel: dict | None = None
        self._panel_t = 0.0
        self._drives: dict | None = None
        self._drives_t = 0.0
        self._mux: dict | None = None
        self._mux_t = 0.0
        g = ReentrantCallbackGroup()
        self.create_subscription(ModeState, "/amr/mode_state", self._on_mode, LATCHED, callback_group=g)
        self.create_subscription(
            ControlLease, "/amr/control_lease", self._on_lease, RELIABLE_1, callback_group=g
        )
        self.create_subscription(PanelState, "/amr/panel_state", self._on_panel, 10, callback_group=g)
        self.create_subscription(DriveStatus, "/drives/status", self._on_drives, RELIABLE_1, callback_group=g)
        self.create_subscription(MuxState, "/amr/mux_state", self._on_mux, RELIABLE_1, callback_group=g)
        self._manual = self.create_publisher(ManualCommand, "/amr/manual_command", RELIABLE_1)
        # diagnostics (unified plan §7): owner-published snapshots and a bounded event ring
        self._diag: dict[str, dict] = {}
        self._io: dict | None = None
        self._io_t = 0.0
        self._events: list[dict] = []
        self._event_n = 0
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 5, callback_group=g)
        self._commissioning: dict | None = None
        self._commissioning_t = 0.0
        self.create_subscription(
            CommissioningState, "/amr/commissioning_state", self._on_commissioning, LATCHED, callback_group=g
        )
        self.create_subscription(IoImage, "/amr/io", self._on_io, 5, callback_group=g)
        self._pp: dict | None = None
        self._pp_t = 0.0
        self.create_subscription(PpStatus, "/drives/pp_status", self._on_pp, RELIABLE_1, callback_group=g)
        self.create_subscription(Event, "/amr/events", self._on_event, 50, callback_group=g)
        # live view (unified plan §6.4): grid, scan, TF; generation-tagged, dropped on a switch
        self.live = live.LiveStore()
        self._scan_seen_t = 0.0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_subscription(OccupancyGrid, "/map", self.live.on_grid, LATCHED, callback_group=g)
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT),
            callback_group=g,
        )
        self.create_timer(0.2, self._live_pose, callback_group=g)
        self.create_subscription(
            MappingState, "/amr/mapping_state", self._on_mapping, LATCHED, callback_group=g
        )
        self.create_subscription(
            LocalizationState, "/amr/localization_state", self._on_loc, LATCHED, callback_group=g
        )
        self.create_subscription(
            SurveyMoveState, "/amr/survey_move_state", self._on_survey_move, LATCHED, callback_group=g
        )
        try:  # T7 adds RunState; tolerate its absence so the survey pages work without it
            from amr_interfaces.msg import RunState  # noqa: PLC0415

            self.create_subscription(RunState, "/amr/run_state", self._on_run, LATCHED, callback_group=g)
        except ImportError:
            pass
        self._initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self._preview = self.create_publisher(Path, "/amr/route_preview", LATCHED)
        self._markers = self.create_publisher(MarkerArray, "/amr/route_markers", LATCHED)
        self._srv = {
            "survey_start": self.create_client(StartSurvey, "/amr/survey/start", callback_group=g),
            "survey_returned": self.create_client(Trigger, "/amr/survey/returned", callback_group=g),
            "survey_save": self.create_client(SaveMap, "/amr/survey/save", callback_group=g),
            "survey_abort": self.create_client(Trigger, "/amr/survey/abort", callback_group=g),
            "survey_move": self.create_client(SurveyMove, "/amr/survey/move", callback_group=g),
            "survey_move_stop": self.create_client(Trigger, "/amr/survey/move_stop", callback_group=g),
            "loc_confirm": self.create_client(Trigger, "/amr/localization/confirm", callback_group=g),
            "loc_reset": self.create_client(Trigger, "/amr/localization/reset", callback_group=g),
            "run": self.create_client(RunMission, "/amr/run_mission", callback_group=g),
            "pause": self.create_client(Trigger, "/amr/pause", callback_group=g),
            "abort": self.create_client(Trigger, "/amr/abort", callback_group=g),
            "resume": self.create_client(Trigger, "/amr/resume", callback_group=g),
            "ack": self.create_client(Trigger, "/amr/ack_fault", callback_group=g),
            # supervisor (unified plan §4.2)
            "mode": self.create_client(RequestMode, "/amr/mode/request", callback_group=g),
            "survey": self.create_client(RequestSurvey, "/amr/supervisor/survey", callback_group=g),
            "operation": self.create_client(GetOperation, "/amr/operations/get", callback_group=g),
            "recover": self.create_client(Trigger, "/amr/supervisor/recover", callback_group=g),
            "comm_plan": self.create_client(PlanCommissioning, "/amr/commissioning/plan", callback_group=g),
            "comm_clear": self.create_client(Trigger, "/amr/commissioning/clear", callback_group=g),
        }

    # ---- subscriptions --------------------------------------------------------------

    def _now(self) -> float:
        return time.monotonic()

    def _on_mapping(self, m: MappingState) -> None:
        d = _msg_to_dict(m)
        d["state_name"] = STATE_NAMES.get(m.state, str(m.state))
        with self._lock:
            self._mapping, self._mapping_t = d, self._now()

    def _on_survey_move(self, m: SurveyMoveState) -> None:
        d = _msg_to_dict(m)
        d["state_name"] = {0: "IDLE", 1: "MOVING", 2: "SETTLING", 3: "DONE", 4: "ABORTED"}.get(
            m.state, str(m.state)
        )
        with self._lock:
            self._survey_move, self._survey_move_t = d, self._now()

    def _on_loc(self, m: LocalizationState) -> None:
        d = _msg_to_dict(m)
        d["state_name"] = LOC_NAMES.get(m.state, str(m.state))
        with self._lock:
            self._loc, self._loc_t = d, self._now()

    def _on_run(self, m) -> None:
        d = _msg_to_dict(m)
        d["state_name"] = RUN_NAMES.get(m.state, str(m.state))
        with self._lock:
            self._run, self._run_t = d, self._now()

    def _world_frame(self) -> str:
        """map while a layer is PUBLISHING map->odom, else odom (plain base): never draw
        a scan in a frame that is not being published. "Publishing" means the newest
        map->odom is stamped within MAP_ODOM_FRESH_S: both AMCL and slam_toolbox restamp
        it on every scan they take, so a frozen stamp is a dead or replaced layer (its
        transform otherwise survives in the cache for ever - tf2 returns the latest
        sample whatever its age) and a frozen stamp is also how slam_toolbox looked
        while hung (2026-09-17)."""
        try:
            t = self._tf_buffer.lookup_transform("map", "odom", Time())
            age = self._now() - Time.from_msg(t.header.stamp).nanoseconds * 1e-9
            if age <= MAP_ODOM_FRESH_S and self._tf_buffer.can_transform("map", "base_footprint", Time()):
                return "map"
        except Exception:  # noqa: BLE001
            pass
        return "odom"

    def _on_scan(self, m: LaserScan) -> None:
        # The live view redraws at 5 Hz; projecting 1152 beams 34 times a second is waste.
        now = time.monotonic()
        if now - self._scan_seen_t < SCAN_PERIOD_S:
            return
        self._scan_seen_t = now
        frame = self._world_frame()
        # Never wait here. The EKF's odom TF lands 20-40 ms after the scan it covers (more
        # under SLAM load), so a blocking lookup at the scan stamp missed every scan on the
        # vehicle and kept the executor's threads asleep: the scan went stale and the /map
        # and pose callbacks starved with it (2026-09-17). Use the stamp when the buffer
        # already has it, else the latest transform: at survey speed that is under 1 cm.
        try:
            try:
                t = self._tf_buffer.lookup_transform(frame, m.header.frame_id, m.header.stamp)
            except Exception:  # noqa: BLE001 - not there yet
                t = self._tf_buffer.lookup_transform(frame, m.header.frame_id, Time())
        except Exception:  # noqa: BLE001 - no transform at all: draw nothing rather than something wrong
            return
        tr, q = t.transform.translation, t.transform.rotation
        pts = live.scan_points(
            m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, tr.x, tr.y, live.yaw_of(q)
        )
        self.live.set_scan(pts, frame, self._stamp_age(m.header.stamp))

    # A replaced layer's map->odom must not be drawn (plan §5.3 step 5). This used to
    # rebuild the TF listener on a generation change; destroying its subscriptions while
    # another executor thread was taking from them raised InvalidHandle out of spin_once
    # and killed the ROS thread (2026-09-17, right as a survey was saved). The listener
    # now lives as long as the node; staleness is judged by stamp in _world_frame().

    def _live_pose(self) -> None:
        with self._lock:
            gen = self._mode["generation"] if self._mode else 0
        self.live.set_generation(gen)
        frame = self._world_frame()
        try:
            t = self._tf_buffer.lookup_transform(frame, "base_footprint", Time())
        except Exception:  # noqa: BLE001
            return
        tr, q = t.transform.translation, t.transform.rotation
        # the transform's own stamp is the evidence time (Q12): a cached, no-longer-updated
        # odom->base (dead EKF) read every 200 ms is old data, not a fresh pose
        self.live.set_pose(float(tr.x), float(tr.y), live.yaw_of(q), frame, self._stamp_age(t.header.stamp))

    def _stamp_age(self, stamp) -> float:
        """Seconds between now and a ROS stamp, clamped at 0 (AMCL stamps map->odom ahead)."""
        return max(0.0, (self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds) * 1e-9)

    def _on_mode(self, m: ModeState) -> None:
        d = _msg_to_dict(m)
        d["mode_name"] = MODE_NAMES.get(m.mode, str(m.mode))
        d["requested_name"] = MODE_NAMES.get(m.requested_mode, str(m.requested_mode))
        with self._lock:
            self._mode, self._mode_t = d, self._now()

    def _on_lease(self, m: ControlLease) -> None:
        with self._lock:
            self._lease = {"instance": m.instance, "generation": int(m.generation), "allowed": int(m.allowed)}
            self._lease_t = self._now()

    def _on_panel(self, m: PanelState) -> None:
        with self._lock:
            self._panel = {"valid": bool(m.valid), "mode_auto": bool(m.mode_auto)}
            self._panel_t = self._now()

    def _on_drives(self, m: DriveStatus) -> None:
        with self._lock:
            self._drives = {"operational": bool(m.operational), "left": m.left_state, "right": m.right_state}
            self._drives_t = self._now()

    def _on_mux(self, m: MuxState) -> None:
        with self._lock:
            self._mux = {
                "source": MUX_NAMES.get(m.source, str(m.source)),
                "inhibited": bool(m.inhibited),
                "reason": m.reason,
                "generation": int(m.generation),
                "left_rad_s": float(m.left_rad_s),
                "right_rad_s": float(m.right_rad_s),
            }
            self._mux_t = self._now()

    def _on_diag(self, m: DiagnosticArray) -> None:
        now = self._now()
        with self._lock:
            for st in m.status:
                self._diag[st.name] = {
                    "name": st.name,
                    "hardware_id": st.hardware_id,
                    "level": int.from_bytes(st.level, "little")
                    if isinstance(st.level, bytes)
                    else int(st.level),
                    "message": st.message,
                    "values": {kv.key: kv.value for kv in st.values},
                    "t": now,
                }

    def _on_io(self, m: IoImage) -> None:
        with self._lock:
            self._io = _msg_to_dict(m)
            self._io_t = self._now()

    def _on_event(self, m: Event) -> None:
        with self._lock:
            self._event_n += 1
            self._events.append(
                {
                    "n": self._event_n,
                    "t": time.time(),
                    "source": m.source,
                    "level": ["info", "warn", "error"][min(int(m.level), 2)],
                    "code": m.code,
                    "text": m.text,
                    "seq": int(m.seq),
                }
            )
            del self._events[:-300]

    def _on_commissioning(self, m: CommissioningState) -> None:
        d = _msg_to_dict(m)
        d["phase_name"] = ["IDLE", "PREPARED", "RUNNING", "SETTLING", "DONE", "ABORTED"][min(int(m.phase), 5)]
        with self._lock:
            self._commissioning, self._commissioning_t = d, self._now()

    def _on_pp(self, m: PpStatus) -> None:
        d = _msg_to_dict(m)
        d.pop("header", None)
        with self._lock:
            self._pp, self._pp_t = d, self._now()

    def pp_status(self) -> dict | None:
        """The drive owner's pp executor (availability, state, last outcome), with its age."""
        with self._lock:
            return dict(self._pp, age_s=self._now() - self._pp_t) if self._pp else None

    def commissioning(self) -> dict | None:
        with self._lock:
            return (
                dict(self._commissioning, age_s=self._now() - self._commissioning_t)
                if self._commissioning
                else None
            )

    def commissioning_plan(self, plan_json: str) -> tuple[bool, str, str]:
        r = self._call("comm_plan", PlanCommissioning.Request(plan_json=plan_json), timeout=5.0)
        if r is None:
            return False, "commissioning node unavailable", ""
        return bool(r.ok), r.message, r.planned_json

    def commissioning_clear(self) -> tuple[bool, str]:
        return self._trigger("comm_clear")

    def diagnostics(self) -> dict:
        now = self._now()
        with self._lock:
            return {k: dict(v, age_s=now - v["t"]) for k, v in self._diag.items()}

    def io_image(self) -> dict | None:
        with self._lock:
            return dict(self._io, age_s=self._now() - self._io_t) if self._io else None

    def events(self, since: int = 0) -> list[dict]:
        with self._lock:
            return [e for e in self._events if e["n"] > since]

    def supervisor_identity(self) -> tuple[str, int]:
        """(instance, generation) from the latest lease, or ("", 0) without a supervisor."""
        with self._lock:
            if self._lease and self._now() - self._lease_t <= 1.0:
                return self._lease["instance"], self._lease["generation"]
        return "", 0

    def manual_allowed(self) -> tuple[bool, str]:
        """Whether the supervisor's current lease carries the MANUAL class (advisory for the
        web; the mux enforces it regardless)."""
        with self._lock:
            lease, t, mode = self._lease, self._lease_t, self._mode
        if not lease or self._now() - t > 1.0:
            return False, "no supervisor lease"
        if lease["allowed"] & 1:
            return True, ""
        if lease["allowed"] & 4:
            return False, "a commissioning job is held or running; clear it first"
        return False, f"manual control withheld by the supervisor ({mode['mode_name'] if mode else '?'})"

    def state(self) -> dict[str, Any]:
        """Everything the pages poll. Layer state is generation-aware: a
        MappingState / LocalizationState / RunState from a layer the supervisor
        has replaced is reported as stale and its body withheld (§6.2)."""
        now = self._now()
        with self._lock:
            gen = self._mode["generation"] if self._mode else None

            def layer(d, t):
                if d is None:
                    return None, None, False
                stale = gen is not None and int(d.get("generation", 0)) != gen
                return (None if stale else d), now - t, stale

            mapping, mapping_age, mapping_stale = layer(self._mapping, self._mapping_t)
            loc, loc_age, loc_stale = layer(self._loc, self._loc_t)
            run, run_age, run_stale = layer(self._run, self._run_t)
            return {
                "mode": self._mode,
                "mode_age_s": (now - self._mode_t) if self._mode else None,
                "lease": self._lease if self._lease and now - self._lease_t <= 1.0 else None,
                "panel": dict(self._panel, age_s=now - self._panel_t) if self._panel else None,
                "drives": dict(self._drives, age_s=now - self._drives_t) if self._drives else None,
                "mux": dict(self._mux, age_s=now - self._mux_t) if self._mux else None,
                "mapping": mapping,
                "mapping_age_s": mapping_age,
                "mapping_stale": mapping_stale,
                "localization": loc,
                "localization_age_s": loc_age,
                "localization_stale": loc_stale,
                "run": run,
                "run_age_s": run_age,
                "run_stale": run_stale,
                # the node lives in the mapping layer: its latched state outlives it, so only
                # while a survey runs is it shown
                "survey_move": (
                    dict(self._survey_move, age_s=now - self._survey_move_t)
                    if mapping is not None and self._survey_move is not None
                    else None
                ),
                "t": time.time(),
            }

    # ---- supervisor operations (asynchronous; the browser polls the operation) ----

    def request_mode(
        self, target: int, map_id: str, map_revision: int, request_id: str
    ) -> tuple[bool, str, str]:
        inst, gen = self.supervisor_identity()
        if not inst:
            return False, "", "supervisor unavailable"
        req = RequestMode.Request()
        req.request_id, req.expected_instance, req.expected_generation = request_id, inst, gen
        req.target, req.map_id, req.map_revision = int(target), map_id, int(map_revision)
        r = self._call("mode", req, timeout=5.0)
        if r is None:
            return False, "", "supervisor unavailable"
        return bool(r.accepted), r.operation_id, r.message

    def survey_request(
        self, operation: int, map_id: str, description: str, request_id: str
    ) -> tuple[bool, str, str]:
        inst, gen = self.supervisor_identity()
        if not inst:
            return False, "", "supervisor unavailable"
        req = RequestSurvey.Request()
        req.request_id, req.expected_instance, req.expected_generation = request_id, inst, gen
        req.operation, req.map_id, req.description = int(operation), map_id, description
        r = self._call("survey", req, timeout=5.0)
        if r is None:
            return False, "", "supervisor unavailable"
        return bool(r.accepted), r.operation_id, r.message

    def get_operation(self, operation_id: str) -> dict | None:
        r = self._call("operation", GetOperation.Request(operation_id=operation_id), timeout=3.0)
        if r is None or not r.found:
            return None
        d = _msg_to_dict(r.state)
        d["status_name"] = OP_NAMES.get(r.state.status, str(r.state.status))
        return d

    def recover(self) -> tuple[bool, str]:
        return self._trigger("recover")

    def manual_publish(self, cmd) -> None:
        """One ManualCommand per accepted browser refresh. No timer repeats it."""
        m = ManualCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.instance, m.generation, m.session = cmd.instance, int(cmd.generation), cmd.session
        m.seq, m.valid_for_s, m.v, m.w = int(cmd.seq), float(cmd.valid_for_s), float(cmd.v), float(cmd.w)
        self._manual.publish(m)

    # ---- services ------------------------------------------------------------------

    def _call(self, key: str, req, timeout: float = 30.0):
        client = self._srv[key]
        # A latched state topic can arrive before DDS has discovered the
        # corresponding service during unified-stack startup. Keep the wait
        # bounded, but allow the first operator action after IDLE to survive
        # that short discovery race.
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None
        fut = client.call_async(req)
        deadline = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.02)
        return fut.result()

    def _trigger(self, key: str) -> tuple[bool, str]:
        r = self._call(key, Trigger.Request())
        if r is None:
            return False, f"{self._srv[key].srv_name} unavailable"
        return bool(r.success), r.message

    def survey_start(self, map_id: str, description: str) -> tuple[bool, str]:
        r = self._call("survey_start", StartSurvey.Request(map_id=map_id, description=description))
        return (False, "survey service unavailable") if r is None else (bool(r.accepted), r.message)

    def survey_returned(self) -> tuple[bool, str]:
        return self._trigger("survey_returned")

    def survey_save(self, note: str) -> tuple[bool, str]:
        r = self._call("survey_save", SaveMap.Request(note=note), timeout=60.0)
        return (False, "survey service unavailable") if r is None else (bool(r.ok), r.message)

    def survey_abort(self) -> tuple[bool, str]:
        return self._trigger("survey_abort")

    def survey_move(self, kind: str, value: float) -> tuple[bool, str]:
        r = self._call("survey_move", SurveyMove.Request(kind=kind, value=float(value)), timeout=5.0)
        return (
            (False, "survey move service unavailable (survey not running?)")
            if r is None
            else (
                bool(r.accepted),
                r.message,
            )
        )

    def survey_move_stop(self) -> tuple[bool, str]:
        return self._trigger("survey_move_stop")

    def localization_confirm(self) -> tuple[bool, str]:
        return self._trigger("loc_confirm")

    def localization_reset(self) -> tuple[bool, str]:
        return self._trigger("loc_reset")

    def set_initial_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        map_id: str,
        map_revision: int,
        generation: int,
        sha256: str = "",
    ) -> tuple[bool, str]:
        """Operator estimate (spec §4.2): wide covariance, AMCL refines it.

        The coordinates were drawn on one particular map picture. They go out only if
        that is the map the navigation layer is running NOW, under the generation the
        browser saw (review R12): every map shares the frame name "map", so the frame
        alone cannot tell map A's coordinates from map B's.
        """
        with self._lock:
            run, mode = self._run, self._mode
        why = initial_pose_mismatch(mode, map_id, map_revision, generation, sha256)
        if why:
            return False, f"refused: {why}"
        if run and run.get("state_name") == "EXECUTING":
            return False, "refused: a route is executing (no /initialpose during a segment)"
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.5**2
        m.pose.covariance[35] = math.radians(15.0) ** 2
        self._initialpose.publish(m)
        return (
            True,
            f"initial pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg) sent; drive slowly, then confirm",
        )

    def publish_route_preview(self, compiled, frame_id: str) -> None:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, st in enumerate(compiled.steps):
            if st.type in ("straight", "reverse", "arc"):
                for x, y, yaw in st.samples:
                    p = PoseStamped()
                    p.header = path.header
                    p.pose.position.x, p.pose.position.y = x, y
                    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                    path.poses.append(p)
            else:
                m = Marker()
                m.header = path.header
                m.ns, m.id = "turns", i
                m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                m.pose.position = Point(x=st.start[0], y=st.start[1], z=0.3)
                m.pose.orientation.w = 1.0
                m.scale.z = 0.3
                m.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
                deg = abs(math.degrees(st.signed_angle_rad))
                m.text = f"{st.id}: {'CCW' if st.signed_angle_rad > 0 else 'CW'} {deg:.0f}"
                markers.markers.append(m)
        self._preview.publish(path)
        self._markers.publish(markers)

    def run_mission(self, mission_id: str) -> tuple[bool, str]:
        r = self._call("run", RunMission.Request(mission_id=mission_id))
        return (
            (False, "executor unavailable (nav.launch.py not running?)")
            if r is None
            else (bool(r.accepted), r.message)
        )

    def pause(self) -> tuple[bool, str]:
        return self._trigger("pause")

    def abort(self) -> tuple[bool, str]:
        return self._trigger("abort")

    def prepare_resume(self) -> tuple[bool, str]:
        return self._trigger("resume")

    def ack_fault(self) -> tuple[bool, str]:
        return self._trigger("ack")


class Spinner:
    """The adapter's executor on its own thread, with an explicit stop/join
    (unified plan §6.2): the service stop must not depend on a daemon thread
    dying with the process while a request is mid-call."""

    def __init__(self, adapter: RosAdapter) -> None:
        self.executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(adapter)
        self._log = adapter.get_logger()
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="ros", daemon=True)

    def _run(self) -> None:
        # One bad callback must not take the whole adapter down: with this thread dead
        # every page shows a frozen snapshot with no indication (2026-09-17).
        while not self._stop.is_set() and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception:  # noqa: BLE001
                self._log.error(f"adapter executor: callback raised; continuing: {traceback.format_exc()}")
                time.sleep(0.05)

    def start(self) -> Spinner:
        self.thread.start()
        return self

    def stop(self, timeout: float = 3.0) -> None:
        self._stop.set()
        self.thread.join(timeout)
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:  # noqa: BLE001
            pass


def start_spinning(adapter: RosAdapter) -> Spinner:
    return Spinner(adapter).start()
