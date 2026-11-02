"""route_executor_node: runs a drawn route as FollowPath straights and Spin turns (spec §5, §7).

One action at a time. The permit lease names exactly the source that may move
the vehicle (FOLLOW during a straight, ROTATE during a turn, NONE otherwise),
the physical panel authorises every start and every resume, and every check
the spec lists is made here against the estimated pose, not trusted from an
action result alone:

  straight   cross-track from the drawn line (fault > limit), endpoint within
             tolerance AND wheels still 0.3 s before advancing, passing the
             endpoint outside tolerance is a fault, never a reverse;
  reverse    a straight driven backwards, facing forward (RPP allow_reversing):
             same checks along the travel direction; at most 2 m and half the
             base speed because the rear is outside the scanner's field;
  rotate     wheels still first; signed relative Spin; own unwrapped yaw from
             /odometry/filtered verifies direction and travel, centre drift is
             bounded, final map heading within tolerance; a paused turn resumes
             the REMAINING signed angle only;
  always     localisation READY, fresh sensors and panel, no /initialpose.

OBSTACLES ARE NOT THE EXECUTOR'S JOB (operator decision 2026-09-20). The scan-vs-envelope
check that used to stop a run was removed: stopping for something in the way is the safety
chain's work - the nanoScan3 protective field and the E-stop, which take the drives' torque
directly. The executor only reacts to that stop after the fact (the `field` / `estop` holds
below). Note the consequences the decision accepts: the protective field looks FORWARD, so a
reverse step and an in-place rotation have no obstacle detection at all, and nothing stops
the vehicle for a trolley standing in a dynamic area. Speed and the operator's judgement when
drawing the route are what bound that risk.

Auto-resume (plan coding-plan-auto-resume.md, 2026-09-19). A stop whose cause can clear by
itself is a BLOCKED hold with a cause, not a FAULT:

  field      the safety chain took the drives' torque (STO) while the nanoScan3 protective
             field was violated (/output_paths): resumes by itself;
  estop      the same drive dropout with the field clear (the E-stop button): waits for a
             physical Start (no Resume click), unless auto_resume_estop;
  controller the FollowPath/Spin goal ABORTED (RPP "collision ahead"): resumes by itself, at
             most controller_abort_retries times per step, then FAULT;
  pending    wheel feedback vanished while executing: stopped at once, and classified as a
             safety stop when the drive report says torque off within safety_window_s, else FAULT.

A hold resumes once every resume check (on the segment, wheels still, ...), drives with
torque and a clear protective field have held for auto_resume_clear_s.

Pre-move warning (2026-10-08). The drives sit in standby (no power) while nothing asks
for them. Every Start - a new run from READY, or a resume of a PAUSED/BLOCKED run - goes
EXECUTING in PHASE_PREMOVE: RunState.drive_power asks drive_node for power, RunState.premove
sounds the panel's alarm horn, and no goal is sent and no permit given until premove_s has
passed AND the drives report torque. Without torque within premove_timeout_s the run holds
(estop: Start again once the safety chain is reset). Auto-resumed holds keep their power and
skip the warning: nobody pressed anything.

Prerequisites are debounced (operator decision 2026-09-20): a prerequisite must FAIL
CONTINUOUSLY for prereq_grace_s before it faults a run or unreadies a loaded mission, so one
dropped frame is not a fault. This does not extend motion authority - the mux enforces panel,
drive, lease and permit freshness itself and stops the vehicle whatever the executor thinks.
"""

from __future__ import annotations

import math
import os
import threading

import rclpy
from agv_core import alarms
from agv_core import config as vehicle_config
from amr_navigation.compiler import ARC, ROTATE, CompiledStep, wrap
from amr_navigation.route import VEHICLE_ARC_W_MAX
from amr_navigation.validate import load_dynamic, load_keepout, validate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath, Spin
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from amr_interfaces.msg import (
    ControlLease,
    DriveStatus,
    Event,
    LocalizationState,
    MotionPermit,
    PanelState,
    RunState,
    WheelStates,
)
from amr_interfaces.srv import RunMission

try:  # the nanoScan3 driver's output paths (OSSD state); absent in sim and on the laptop
    from sick_safetyscanners2_interfaces.msg import OutputPaths
except ImportError:  # pragma: no cover - depends on the installed driver
    OutputPaths = None
from amr_mission import goal_attempts as ga
from amr_mission import map_bundle as mb
from amr_mission import run_fsm as fsm
from amr_navigation import footprint as fpmod
from amr_navigation import store

SENSOR_DATA = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)

PHASE_INIT, PHASE_GOAL, PHASE_SETTLE, PHASE_PREMOVE = "init", "goal", "settle", "premove"


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def mission_matches_active_map(mission: dict, active: tuple[str, int, str]) -> str | None:
    """P4 (unified plan §5.6): a mission may only load onto the map AMCL is running.

    `active` is (id, revision, sha256) from the launch; ("", 0, "") means an
    unsupervised standalone stack, which binds to nothing (bench behaviour).
    Returns the refusal reason, or None when the mission's map IS the active map.
    """
    aid, arev, asha = active
    if not aid:
        return None
    m = mission.get("map", {})
    if m.get("id") != aid or int(m.get("revision", 0)) != arev:
        return f"mission is for map {m.get('id')} rev{m.get('revision')}, active map is {aid} rev{arev}"
    if asha and m.get("sha256") != asha:
        return "mission map hash differs from the active map bundle"
    return None


class RouteExecutor(Node):
    def __init__(self) -> None:
        super().__init__("route_executor")
        self.declare_parameter("maps_dir", os.path.expanduser("~/amr_maps"))
        self.declare_parameter("footprint_yaml", "")
        # Layer identity (unified plan U0/U4): stamped into every RunState and MotionPermit;
        # the active_map_* values are what a mission must match (U4 enforces).
        self.declare_parameter("generation", 0)
        self.declare_parameter("active_map_id", "")
        self.declare_parameter("active_map_revision", 0)
        self.declare_parameter("active_map_sha256", "")
        self.generation = int(self.get_parameter("generation").value)
        self.active_map = (
            str(self.get_parameter("active_map_id").value),
            int(self.get_parameter("active_map_revision").value),
            str(self.get_parameter("active_map_sha256").value),
        )
        self.declare_parameter("start_gate_m", 0.10)
        self.declare_parameter("start_gate_deg", 5.0)
        self.declare_parameter("settle_s", 0.3)
        self.declare_parameter("stationary_wheel_rad_s", 0.02)
        # Centre drift during a turn is measured on ODOMETRY (precise over one turn), not on
        # AMCL, whose estimate wanders several cm during a spin in place. Spec §5.3 said 0.05 m;
        # 0.10 m since 2026-09-20 (operator request: fewer nuisance faults).
        self.declare_parameter("centre_drift_m", 0.10)
        # The executor's own end-of-step verification against the AMCL pose. RPP's goal
        # checker already stops at 0.05 m / 2 deg (spec §5.2); this re-check must allow
        # the estimator's own noise (sim AMCL p95 ~0.06 m) or it fails good stops.
        # HARDWARE PLACEHOLDERS: sim AMCL wanders 1-3 deg during a spin in place, so a 2-3 deg
        # heading re-check trips on estimator noise, not on the turn (odometry travel is within 2 deg).
        # Widened 0.08 -> 0.15 m and 5 -> 8 deg on 2026-09-20 (operator request). Each step
        # re-projects from the ACTUAL pose and a turn folds its entry error, so the error is
        # corrected, not accumulated - but a stop may now sit further outside the validated
        # footprint margin (0.05 m). Clearance stays the operator's judgement when drawing.
        self.declare_parameter("verify_position_m", 0.15)
        self.declare_parameter("verify_heading_deg", 8.0)
        self.declare_parameter("converge_m", 1.0)  # cross-track grace distance after a step starts
        self.declare_parameter("entry_correction_deg", 10.0)  # max entry-heading error folded into a turn
        # Vehicle 2026-09-17: Spin at 10 Hz with min 0.05 rad/s stops 0-2.2 deg past the commanded
        # angle (turns of 45.0 and 46.6 for 44.4 asked). The overshoot is folded into the next
        # turn by the entry correction and bounded by verify_heading_deg, so this is
        # the stop discretisation, not a looser route. 4 -> 6 deg on 2026-09-20.
        self.declare_parameter("turn_travel_tolerance_deg", 6.0)
        self.declare_parameter("wrong_way_deg", 5.0)
        # Sensor ages. All widened on 2026-09-20 (operator request) and backed by
        # prereq_grace_s below; the mux keeps its own, tighter freshness rules, so a longer
        # age here buys the run tolerance to a gap without ever extending motion authority.
        self.declare_parameter("wheels_age_limit_s", 0.25)  # /wheel_states is 50 Hz
        self.declare_parameter("scan_age_limit_s", 1.0)  # /scan_gated is <= 10 Hz; gate hold_max 0.5 s
        # A prerequisite must fail CONTINUOUSLY this long before it faults a run or unreadies
        # a loaded mission: one dropped frame is not a fault (module docstring).
        self.declare_parameter("prereq_grace_s", 0.5)
        # Part B2: the FollowPath goal sits this far past the endpoint (0 = aim at the endpoint,
        # the rollback). Tune on the vehicle so the mean stop error is ~0; never above the
        # route's position_tolerance_m (clamped).
        self.declare_parameter("goal_overshoot_m", 0.045)
        self.declare_parameter("panel_age_limit_s", 0.50)
        self.declare_parameter("loc_age_limit_s", 2.5)
        # Speed tapers between chained steps (a boosted straight into a normal one, a straight
        # into an arc at its arc speed) and at the end of a boosted straight: the lower speed is asked
        # early enough that the mux decel (d_max 0.5) reaches it before the boundary, from
        # (v^2 - v_next^2) / (2 taper_decel) plus taper_lead_s of travel.
        self.declare_parameter("taper_decel", 0.4)
        self.declare_parameter("taper_lead_s", 0.3)
        # auto-resume after a stop whose cause clears by itself (see the module docstring)
        self.declare_parameter("auto_resume_enabled", True)
        self.declare_parameter("auto_resume_clear_s", 2.0)
        self.declare_parameter("auto_resume_estop", False)  # operator decision 2026-09-19: Start
        self.declare_parameter("safety_window_s", 1.0)
        self.declare_parameter("drives_age_limit_s", 0.5)
        self.declare_parameter("field_output_index", 0)  # /output_paths status[i]: protective field
        # false: ignore the scanner's protective field (/output_paths); a drive dropout then
        # counts as the E-stop. No vehicle uses it now (the AMR QR did 2026-09-24..11-02, on a
        # broken nanoScan3).
        self.declare_parameter("use_protective_field", True)
        self.declare_parameter("controller_abort_retries", 3)
        # The pre-move warning after a Start (module docstring); the profile's timing block.
        self.declare_parameter("premove_s", float(vehicle_config.AUTO_START_DELAY_S))
        self.declare_parameter("premove_timeout_s", float(vehicle_config.PREMOVE_TIMEOUT_S))
        # R08: bound on waiting for a goal's acceptance, and for an obsolete (cancelled) goal to
        # report terminal before a replacement goal may be issued.
        self.declare_parameter("goal_accept_timeout_s", 5.0)
        self.declare_parameter("goal_cancel_timeout_s", 5.0)
        p = self.get_parameter
        self.maps_dir = os.path.expanduser(p("maps_dir").value)
        self.fp = fpmod.load(p("footprint_yaml").value or fpmod.default_path())
        self.start_gate_m, self.start_gate_rad = (
            p("start_gate_m").value,
            math.radians(p("start_gate_deg").value),
        )
        self.settle_s = p("settle_s").value
        self.w_eps = p("stationary_wheel_rad_s").value
        self.centre_drift_m = p("centre_drift_m").value
        self.verify_pos = p("verify_position_m").value
        self.verify_yaw = math.radians(p("verify_heading_deg").value)
        self.converge_m = p("converge_m").value
        self.entry_corr_max = math.radians(p("entry_correction_deg").value)
        self.turn_tol = math.radians(p("turn_travel_tolerance_deg").value)
        self.wrong_way = math.radians(p("wrong_way_deg").value)
        self.wheels_age, self.panel_age, self.loc_age = (
            p("wheels_age_limit_s").value,
            p("panel_age_limit_s").value,
            p("loc_age_limit_s").value,
        )
        self.scan_age = p("scan_age_limit_s").value
        self.prereq_grace_s = float(p("prereq_grace_s").value)
        self.goal_overshoot_m = float(p("goal_overshoot_m").value)
        self.taper_decel = max(0.05, float(p("taper_decel").value))
        self.taper_lead_s = float(p("taper_lead_s").value)
        self.auto_resume_enabled = bool(p("auto_resume_enabled").value)
        self.auto_clear_s = float(p("auto_resume_clear_s").value)
        self.auto_resume_estop = bool(p("auto_resume_estop").value)
        self.safety_window = float(p("safety_window_s").value)
        self.drives_age = float(p("drives_age_limit_s").value)
        self.premove_s = max(0.0, float(p("premove_s").value))
        self.premove_timeout_s = max(self.premove_s, float(p("premove_timeout_s").value))
        self._premove_since: float | None = None
        self.field_index = int(p("field_output_index").value)
        self.abort_retries = int(p("controller_abort_retries").value)
        self._init_hold_state()
        self.goal_accept_timeout = p("goal_accept_timeout_s").value
        self.goal_cancel_timeout = p("goal_cancel_timeout_s").value

        self._lock = threading.RLock()
        self.fsm = fsm.RunFsm()
        self.compiled = None
        self.route = None
        self.manifest = None
        self.grid = None
        self.mission = None
        # one goal attempt (run, pass, step, attempt) at a time; see goal_attempts.py (R08)
        self.goals = ga.GoalAttempts(
            self._lock, self._now, self._on_goal_error, lambda msg: self.get_logger().warn(msg)
        )
        self._reset_step_state()
        self.last_initialpose_t: float | None = None
        self.paused_t: float | None = None
        self.odom_gap = False

        self._loc: LocalizationState | None = None
        self._loc_t = 0.0
        self._panel_t = 0.0
        self._panel: PanelState | None = None
        self._wheels_t = 0.0
        self._wheels_valid = False
        self._wheels_still = False
        self._odom_t = 0.0
        self._odom_yaw_prev: float | None = None
        self._odom_yaw_acc = 0.0
        self._odom_xy = (0.0, 0.0)
        self._scan: LaserScan | None = None
        self._scan_t = 0.0
        self._last_state_key = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        io = ReentrantCallbackGroup()
        srv = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            LocalizationState, "/amr/localization_state", self._on_loc, LATCHED, callback_group=io
        )
        self.create_subscription(PanelState, "/amr/panel_state", self._on_panel, 10, callback_group=io)
        # The permit must carry the supervisor's instance (unified plan §4.3): the mux
        # refuses a permit from another instance/generation. Unsupervised: "" / 0.
        self._lease_instance = ""
        self._permit_seq = 0
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, 10, callback_group=io)
        self.create_subscription(
            WheelStates, "/wheel_states", self._on_wheels, SENSOR_DATA, callback_group=io
        )
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10, callback_group=io)
        self.create_subscription(DriveStatus, "/drives/status", self._on_drives, 10, callback_group=io)
        if not self.get_parameter("use_protective_field").value:
            self.get_logger().warn(
                "use_protective_field=false: the scanner's protective field is IGNORED "
                "(a drive dropout counts as the E-stop)"
            )
        elif OutputPaths is not None:
            self.create_subscription(
                OutputPaths, "/output_paths", self._on_output_paths, SENSOR_DATA, callback_group=io
            )
        else:
            self.get_logger().warn("no sick_safetyscanners2_interfaces: a drive dropout counts as an E-stop")
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_initialpose, RELIABLE_1, callback_group=io
        )
        # /scan_gated: scans whose odom transform already exists, so the obstruction check
        # can project each one at ITS OWN stamp (review Q08) without waiting or falling
        # back to the latest pose. Raw /scan would be 20-40 ms ahead of TF on the vehicle.
        self.create_subscription(LaserScan, "/scan_gated", self._on_scan, SENSOR_DATA, callback_group=io)
        self._follow = ActionClient(self, FollowPath, "/follow_path", callback_group=io)
        self._spin = ActionClient(self, Spin, "/spin", callback_group=io)
        self._permit_pub = self.create_publisher(MotionPermit, "/amr/motion_permit", RELIABLE_1)
        # Per-step speed for the controller (RPP setSpeedLimit, absolute m/s): the approach ramp
        # and lookahead scaling are then planned from the step's speed, while the same number
        # rides in the permit v_max as the mux's hard clamp. controller_server's default
        # speed_limit_topic; plain (not generation-private) because an old layer's controller
        # is being stopped when a new one exists and a limit is harmless to it.
        self._speed_pub = self.create_publisher(SpeedLimit, "/speed_limit", RELIABLE_1)
        self._state_pub = self.create_publisher(RunState, "/amr/run_state", LATCHED)
        self._event_pub = self.create_publisher(Event, "/amr/events", 50)
        self._event_seq = 0
        self._last_event_state: int | None = None
        self._run_started: float | None = None
        self._run_holds: dict[str, int] = {}
        self._run_peak_cross = 0.0
        self.create_service(RunMission, "/amr/run_mission", self._srv_run, callback_group=srv)
        self.create_service(Trigger, "/amr/pause", self._srv_pause, callback_group=srv)
        self.create_service(Trigger, "/amr/abort", self._srv_abort, callback_group=srv)
        self.create_service(Trigger, "/amr/resume", self._srv_resume, callback_group=srv)
        self.create_service(Trigger, "/amr/ack_fault", self._srv_ack, callback_group=srv)
        self.create_timer(0.1, self._tick, callback_group=srv)
        self.create_timer(0.5, self._publish_state, callback_group=io)
        self.get_logger().info(f"IDLE; maps_dir {self.maps_dir}; footprint reach {self.fp.reach_m:.2f} m")

    # ---- inputs ------------------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_loc(self, m: LocalizationState) -> None:
        self._loc, self._loc_t = m, self._now()

    def _on_lease(self, m: ControlLease) -> None:
        if int(m.generation) == self.generation or self.generation == 0:
            self._lease_instance = m.instance

    def _on_panel(self, m: PanelState) -> None:
        self._panel, self._panel_t = m, self._now()
        if m.start_edge:
            with self._lock:
                self._start_edge()
        if m.reset_edge:
            with self._lock:
                self._reset_edge()

    def _reset_edge(self) -> None:
        """Physical Reset (review Q19): acknowledges THIS executor's FAULT, exactly like the
        web's Acknowledge fault, and nothing else - no motion, no resume, no drive or
        supervisor fault clearing, and the unresolved-goal barrier (Q06) is untouched.
        Only while the vehicle is at rest; the operator then localises/loads/Starts again."""
        if self.fsm.state != fsm.FAULT:
            return
        if not self._wheels_still:
            self.get_logger().warn("physical Reset ignored: wheels not at rest")
            return
        if self.fsm.ack():
            self._log_state("fault acknowledged by physical Reset")

    def _on_wheels(self, m: WheelStates) -> None:
        self._wheels_t = self._now()
        # R07: receipt is not evidence. Prerequisites need the drive's validity flags too.
        self._wheels_valid = bool(m.left_valid and m.right_valid)
        self._wheels_still = (
            self._wheels_valid and abs(m.left_vel_rad_s) < self.w_eps and abs(m.right_vel_rad_s) < self.w_eps
        )

    def _init_hold_state(self) -> None:
        self.hold_cause = ""  # why the run is BLOCKED (module docstring), "" otherwise
        self.fault_code = ""  # alarm catalogue code for a FAULT (agv_core/alarms.py)
        self._auto_since: float | None = None  # all-clear since, for auto-resume
        self._pending_since: float | None = None
        self._prereq_since: float | None = None  # a prerequisite has been failing since
        self._aborts = 0  # controller aborts auto-resumed on this step
        self._drives_t: float | None = None
        self._torque_off_msg = False
        self._last_off_t: float | None = None  # last time the drive report said torque off
        self._field_clear: bool | None = None
        self._field_trip_t: float | None = None  # last time the protective field was violated

    def _on_drives(self, m: DriveStatus) -> None:
        t = self._now()
        # torque off = not operational AND neither drive's statusword says Operation enabled
        off = not m.operational and "Operation enabled" not in (m.left_state, m.right_state)
        self._drives_t, self._torque_off_msg = t, off
        if off:
            self._last_off_t = t

    def _on_output_paths(self, m) -> None:
        if len(m.status) <= self.field_index:
            return
        self._field_clear = bool(m.status[self.field_index])
        if not self._field_clear:
            self._field_trip_t = self._now()

    def _torque_off(self, now: float) -> bool:
        return self._torque_off_msg and self._drives_t is not None and now - self._drives_t <= self.drives_age

    def _field_tripped(self, now: float) -> bool:
        if self._field_clear is False:
            return True
        return self._field_trip_t is not None and now - self._field_trip_t <= self.safety_window

    def _safety_cause(self, now: float) -> str:
        return "field" if self._field_tripped(now) else "estop"

    def _auto_causes(self) -> tuple[str, ...]:
        return ("field", "controller") + (("estop",) if self.auto_resume_estop else ())

    def _hold(self, cause: str, why: str) -> None:
        """Stop and hold: BLOCKED with a cause (see the module docstring)."""
        text = {
            "field": f"safety stop (protective field): {why}",
            "estop": f"safety stop (E-stop / safety chain): {why}",
        }.get(cause, why)
        if self.fsm.state == fsm.EXECUTING:
            self.fsm.block(text)
        else:
            self.fsm.reason = text
        self.hold_cause, self._auto_since = cause, None
        self._pending_since = self._now() if cause == "pending" else None
        self._interrupt(text)

    @staticmethod
    def _wheel_prereq(pre: str) -> bool:
        return pre.startswith("wheel feedback")

    def _on_odom(self, m: Odometry) -> None:
        t = self._now()
        yaw = _yaw(m.pose.pose.orientation)
        self._odom_xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self._odom_yaw_prev is not None:
            if t - self._odom_t > 0.5:
                self.odom_gap = True  # continuity lost: a paused turn cannot resume on this
            self._odom_yaw_acc += wrap(yaw - self._odom_yaw_prev)
        self._odom_yaw_prev, self._odom_t = yaw, t

    def _on_initialpose(self, _m) -> None:
        self.last_initialpose_t = self._now()
        with self._lock:
            if self.fsm.state == fsm.EXECUTING:
                self._fault("initial pose injected during a step", "EXEC_POSE_INJECTED")
            elif self.fsm.state in (fsm.PAUSED, fsm.BLOCKED):
                self.fsm.invalidate_resume(
                    "initial pose changed; progress-based resume is invalid, abort the run"
                )
                self.odom_gap = True

    def _on_scan(self, m: LaserScan) -> None:
        self._scan, self._scan_t = m, self._now()

    def _pose(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        return t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)

    # ---- services -------------------------------------------------------------------------

    def _srv_run(self, req: RunMission.Request, res: RunMission.Response):
        with self._lock:
            try:
                mission = store.load_mission(self.maps_dir, req.mission_id)
                why = mission_matches_active_map(mission, self.active_map)
                if why:
                    raise store.StoreError(why)
                manifest, grid = mb.load(self.maps_dir, mission["map"]["id"], int(mission["map"]["revision"]))
                if manifest.sha256 != mission["map"]["sha256"]:
                    raise store.StoreError("map bundle hash differs from the mission manifest")
                route, sha = store.load_route(
                    self.maps_dir,
                    mission["map"]["id"],
                    mission["route"]["id"],
                    int(mission["route"]["revision"]),
                )
                if sha != mission["route"]["sha256"]:
                    raise store.StoreError("route file hash differs from the mission manifest")
                rev_dir = mb.revision_dir(self.maps_dir, manifest.map_id, manifest.revision)
                dynamic = load_dynamic(rev_dir)
                v = validate(route, manifest, grid, self.fp, load_keepout(rev_dir), dynamic)
                if not v.ok:
                    raise store.StoreError(
                        "route invalid on this map: "
                        + "; ".join(f"{i.step_id or ''} {i.message}" for i in v.errors)
                    )
                provisional = [i.step_id or "start" for i in v.issues if i.code == "provisional"]
                if provisional:
                    # Since 2026-09-20 the executor no longer checks scans against the envelope,
                    # so a trolley still standing in one of these areas will NOT stop the run:
                    # only the protective field will. Warn, do not merely inform.
                    self.get_logger().warn(
                        f"steps {', '.join(provisional)} cross mapped cells in dynamic areas; "
                        "the route was cleared on the assumption those cells are empty, and "
                        "nothing but the safety chain will stop the vehicle if they are not"
                    )
                # Q05 relaxed 2026-09-17 (operator request, footprint margin 0.05 m): the
                # lateral envelope execution permits may exceed what validation cleared.
                # Said once per load so the log records the gap.
                lateral = max(route.limits.cross_track_limit_m, self.start_gate_m)
                if lateral > self.fp.margin_m + 1e-9:
                    self.get_logger().warn(
                        f"permitted lateral error {lateral:.2f} m (cross-track limit / start gate) "
                        f"exceeds the validated footprint margin {self.fp.margin_m:.2f} m"
                    )
            except (store.StoreError, mb.BundleError, KeyError, ValueError) as e:
                res.accepted, res.message = False, f"refused: {e}"
                return res
            ok = self.fsm.load(req.mission_id, len(v.compiled.steps), route.repeat_count)
            if not ok:
                res.accepted, res.message = False, self.fsm.reason
                return res
            self.mission, self.manifest, self.grid, self.route, self.compiled = (
                mission,
                manifest,
                grid,
                route,
                v.compiled,
            )
            self._reset_step_state()
            self._log_state()
            res.accepted, res.message = True, self.fsm.reason
            return res

    def _srv_pause(self, _req, res: Trigger.Response):
        with self._lock:
            ok = self.fsm.pause()
            if ok:
                self._interrupt("paused by operator")
            res.success, res.message = ok, self.fsm.reason
            return res

    def _srv_abort(self, _req, res: Trigger.Response):
        with self._lock:
            ok = self.fsm.abort("aborted by operator")
            if ok:
                self._interrupt("aborted")
            res.success, res.message = ok, self.fsm.reason
            return res

    def _srv_resume(self, _req, res: Trigger.Response):
        with self._lock:
            ok, why = self._resume_checks()
            res.success = self.fsm.prepare_resume(ok, why)
            res.message = self.fsm.reason
            self._log_state()
            return res

    def _srv_ack(self, _req, res: Trigger.Response):
        with self._lock:
            res.success, res.message = self.fsm.ack(), self.fsm.reason
            self._log_state()
            return res

    # ---- readiness ------------------------------------------------------------------------

    def _prereqs(self) -> str | None:
        """None when everything a run needs holds; otherwise the first failure."""
        now = self._now()
        barrier = self.goals.barrier()
        if barrier:
            return barrier  # Q06: an action goal with an unknown outcome bars new motion
        if self._loc is None or now - self._loc_t > self.loc_age:
            return "no localisation monitor"
        if self._loc.state != LocalizationState.READY:
            return f"localisation not READY ({self._loc.reason})"
        if now - self._wheels_t > self.wheels_age:
            return "wheel feedback stale"
        if not self._wheels_valid:
            return "wheel feedback invalid"
        if self._panel is None or now - self._panel_t > self.panel_age or not self._panel.valid:
            return "panel stale or invalid"
        if self._scan is None or now - self._scan_t > self.scan_age:
            return "no fresh scan"
        return None

    def _prereq_held(self, now: float, pre: str | None) -> bool:
        """True once a prerequisite has been failing CONTINUOUSLY for prereq_grace_s
        (operator decision 2026-09-20). Any tick that passes clears the timer, so one dropped
        frame never latches a FAULT; a genuine loss still faults, half a second later.

        The reason may change while the timer runs (a stale panel, then a stale scan): what
        is debounced is "something is wrong", not one particular message. Callers that merely
        REFUSE (the Start gate, the resume checks) do not use this - refusing is free, and
        those gates should answer for the instant the operator pressed the button."""
        if not pre:
            self._prereq_since = None
            return False
        if self._prereq_since is None:
            self._prereq_since = now
        return now - self._prereq_since >= self.prereq_grace_s

    def _auto(self) -> bool:
        return self._panel is not None and self._panel.valid and self._panel.mode_auto

    def _start_gate(self) -> tuple[bool, str]:
        pose = self._pose()
        if pose is None:
            return False, "no pose"
        s = self.route.start
        d = math.hypot(pose[0] - s.x_m, pose[1] - s.y_m)
        a = abs(wrap(pose[2] - s.yaw_rad))
        if d > self.start_gate_m or a > self.start_gate_rad:
            return False, f"{d:.2f} m / {math.degrees(a):.1f} deg from the route start; reposition manually"
        return True, ""

    def _start_edge(self) -> None:
        if self.fsm.state == fsm.READY:
            gate_ok, why = self._start_gate()
            pre = self._prereqs()
            if pre:
                gate_ok, why = False, pre
            if self.fsm.start(self._auto(), gate_ok, why):
                self._reset_step_state()
                self._begin_premove()
            self._log_state()
        elif self.fsm.state in (fsm.PAUSED, fsm.BLOCKED):
            if self.fsm.state == fsm.BLOCKED and not self.fsm.resume_prepared and self._auto():
                # a hold continues on Start alone (operator decision 2026-09-19: after the E-stop
                # button, Start without a Resume click) when every resume check holds right now
                # No torque check: the drives are in standby until this Start powers them.
                ok, why = self._resume_checks()
                if not ok:
                    self.fsm.reason = f"Start ignored: {why}"
                    self._log_state()
                    return
                self.fsm.prepare_resume(True, "")
            if self.fsm.resume_prepared:
                # R18: the prepared resume may be stale by the time Start is pressed (vehicle
                # moved, odometry gap): re-run every resume check at the edge itself.
                ok, why = self._resume_checks()
                if not ok:
                    self.fsm.invalidate_resume(why)  # keeps the reason visible; Start does nothing
                    self._log_state()
                    return
            if self.fsm.start(self._auto(), True):
                self.hold_cause, self._auto_since = "", None
                self._begin_premove()
            self._log_state()

    def _begin_premove(self) -> None:
        self.phase, self._premove_since = PHASE_PREMOVE, self._now()
        self.fsm.reason = f"{self.fsm.reason}: warning, moving in {self.premove_s:.1f} s"

    def _premove_tick(self, now: float) -> None:
        """EXECUTING before the first goal after a Start: alarm horn, power asked for. Go once
        the warning has run and the drives have torque; hold (estop) if they never do."""
        waited = now - (self._premove_since if self._premove_since is not None else now)
        drives_fresh = self._drives_t is not None and now - self._drives_t <= self.drives_age
        if waited >= self.premove_s and drives_fresh and not self._torque_off(now):
            self.phase, self._premove_since = PHASE_INIT, None
            self.fsm.reason = f"moving: {self.fsm.progress()}"
            return
        if waited >= self.premove_timeout_s:
            self._premove_since = None
            self._hold(
                "estop",
                f"drives did not power up within {self.premove_timeout_s:.0f} s "
                "(safety chain open? release the E-stop, Reset, then Start)",
            )

    def drive_power(self) -> bool:
        """RunState.drive_power: what this run asks of drive_node's power policy. EXECUTING
        (the pre-move warning included), and BLOCKED holds that keep their power: those that
        resume by themselves, and a pending hold, whose torque reading is its evidence."""
        state = self.fsm.state
        if state == fsm.EXECUTING:
            return True
        if state != fsm.BLOCKED:
            return False
        auto = self.auto_resume_enabled and self.hold_cause in self._auto_causes()
        return auto or self.hold_cause == "pending"

    def _resume_checks(self) -> tuple[bool, str]:
        pre = self._prereqs()
        if pre:
            return False, pre
        if not self._auto():
            return False, "selector is not AUTO"
        if not self._wheels_still:
            return False, "vehicle is moving"
        st = self._step()
        if st is None:
            return False, "no active step"
        pose = self._pose()
        if pose is None:
            return False, "no pose"
        if st.type != ROTATE:
            along, cross = self._along_cross(st, pose)
            if abs(cross) > self.route.limits.cross_track_limit_m:
                return False, f"outside the corridor ({cross:+.2f} m cross-track)"
            if (
                along < -self.route.limits.position_tolerance_m
                or along > st.length_m + self.route.limits.position_tolerance_m
            ):
                return False, "not on the segment"
        else:
            if self.odom_gap:
                return False, "odometry continuity lost since the pause; abort and reposition"
            if self.turn_centre is not None:
                drift = math.hypot(
                    self._odom_xy[0] - self.turn_centre[0], self._odom_xy[1] - self.turn_centre[1]
                )
                if drift > self.centre_drift_m:
                    return False, f"moved {drift:.2f} m off the turn centre; abort and reposition"
        return True, ""

    # ---- step geometry ------------------------------------------------------------------------

    def _step(self) -> CompiledStep | None:
        if (
            self.compiled is None
            or self.fsm.step_index < 0
            or self.fsm.step_index >= len(self.compiled.steps)
        ):
            return None
        return self.compiled.steps[self.fsm.step_index]

    @staticmethod
    def _along_cross(st: CompiledStep, pose) -> tuple[float, float]:
        """Progress along the step's TRAVEL direction (positive = the way the step goes, so a
        reverse step counts up while backing) and the signed lateral offset (positive =
        left of travel). An arc measures both from its centre: progress is the swept angle
        times the radius, the offset is how far inside (left, for a left arc) the circle."""
        if st.type == ARC:
            sign = 1.0 if st.signed_angle_rad >= 0 else -1.0
            cx, cy = st.centre
            phi = sign * wrap(
                math.atan2(pose[1] - cy, pose[0] - cx) - math.atan2(st.start[1] - cy, st.start[0] - cx)
            )
            if phi < -math.pi / 2:  # a 180 deg arc overrun reads as a small negative angle
                phi += 2.0 * math.pi
            r = math.hypot(pose[0] - cx, pose[1] - cy)
            return st.radius_m * phi, sign * (st.radius_m - r)
        c, s = math.cos(st.travel_yaw), math.sin(st.travel_yaw)
        dx, dy = pose[0] - st.start[0], pose[1] - st.start[1]
        return dx * c + dy * s, -dx * s + dy * c

    def _reset_step_state(self) -> None:
        """R18: the one place per-step progress is initialised (new load, new run, next step),
        so no turn geometry of an aborted/faulted step can leak into the next one."""
        self.phase = PHASE_INIT
        self.settle_since: float | None = None
        self.turn_travelled = 0.0  # folded travel of this turn, signed
        self.turn_acc0: float | None = None  # odom yaw accumulator baseline while travel is being counted
        self.turn_started_t: float | None = None
        self.turn_centre: tuple[float, float] | None = None  # odom xy where the turn ACTUALLY began
        self.turn_target: float | None = None  # signed angle to travel: drawn magnitude + entry correction
        self.turn_feedback = 0.0
        self.cross_track = 0.0
        self._aborts = 0

    def _turn_travel(self) -> float:
        """Measured travel of the current turn. Counting is NOT frozen at a pause: rotation
        while decelerating to standstill (and any after) is real travel (R18)."""
        live = self._odom_yaw_acc - self.turn_acc0 if self.turn_acc0 is not None else 0.0
        return self.turn_travelled + live

    def _remaining_turn(self, st: CompiledStep) -> float:
        target = self.turn_target if self.turn_target is not None else st.signed_angle_rad
        return target - self._turn_travel()

    def _cross_track_allowed(self, along: float) -> float:
        limit = self.route.limits.cross_track_limit_m
        return limit if along > self.converge_m else 2.0 * limit

    # ---- goals ---------------------------------------------------------------------------------

    def _follow_path(self, st: CompiledStep, along: float) -> Path:
        """The FollowPath path for a straight from the current projection to its end, plus one
        pose `goal_overshoot_m` PAST the end along the heading (Part B2): the goal checker's
        circle (xy_goal_tolerance) then fires just before the true endpoint instead of the
        vehicle hunting for a small circle around it. Arrival and the overshoot fault are
        still judged against the TRUE endpoint (settle / along > length + 2 tol), and the
        extra pose is clamped to position_tolerance_m so it lies inside the validated
        swept envelope."""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        def add(x, y, yaw):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(ps)

        for sample in st.samples:
            if self._along_cross(st, sample)[0] < along - 0.10:
                continue  # resume: from the current projection onwards
            add(*sample)
        # the chained steps after this one, in the same path: no goal, no stop, between them
        last = st
        for nxt in self._chain(self.fsm.step_index)[1:]:
            for sample in nxt.samples[1:]:
                add(*sample)
            last = nxt
        over = max(0.0, min(self.goal_overshoot_m, self.route.limits.position_tolerance_m))
        if over > 0.0 and path.poses:
            # past the end along the travel direction there: the end tangent for an arc, the
            # (possibly reversed) heading for a line; poses keep the forward heading
            end_dir = last.end[2] if last.type == ARC else last.travel_yaw
            add(last.end[0] + math.cos(end_dir) * over, last.end[1] + math.sin(end_dir) * over, last.end[2])
        return path

    # ---- chains: consecutive forward steps (straight, arc) are driven as ONE FollowPath so the
    # vehicle never stops between them (2026-09-18); a rotate or a reverse ends a chain.

    @staticmethod
    def _chainable(st: CompiledStep) -> bool:
        return st.type != ROTATE and not st.reverse

    def _chain(self, index: int) -> list[CompiledStep]:
        """The steps driven together from `index`: it and every chainable step after it."""
        steps = self.compiled.steps
        if not self._chainable(steps[index]):
            return [steps[index]]
        out = []
        for st in steps[index:]:
            if not self._chainable(st):
                break
            out.append(st)
        return out

    def _step_w_max(self, st: CompiledStep) -> float:
        """The permit's turn cap: the route's spin cap, or the arc ceiling for the WHOLE of a
        chain that holds an arc - RPP's carrot starts curving a lookahead before the arc
        begins, so the straight leading into it needs the arc's yaw rate too."""
        if st.type != ROTATE and any(s.type == ARC for s in self._chain(self.fsm.step_index)):
            return max(float(self.route.limits.w_mps), VEHICLE_ARC_W_MAX)
        return float(self.route.limits.w_mps)

    def _next_in_chain(self) -> CompiledStep | None:
        chain = self._chain(self.fsm.step_index)
        return chain[1] if len(chain) > 1 else None

    def _taper_dist(self, v: float, v_next: float) -> float:
        if v_next >= v:
            return 0.0
        return (v * v - v_next * v_next) / (2.0 * self.taper_decel) + v * self.taper_lead_s

    def _step_speed(self, st: CompiledStep, pose) -> float:
        """The speed the vehicle may run at HERE on a forward step: its compiled speed, tapered
        to the NEXT chained step's speed (a tight arc runs below the trackless speed) early
        enough that the mux decel reaches it at the boundary."""
        v = float(st.v_mps)
        if pose is None:
            return v
        nxt = self._next_in_chain()
        base = float(self.route.limits.linear_mps)
        v_next = float(nxt.v_mps) if nxt is not None else min(v, base)
        if v_next >= v:
            return v
        along, _ = self._along_cross(st, pose)
        if st.length_m - along < self._taper_dist(v, v_next):
            return v_next
        return v

    def _publish_speed(self, v: float) -> None:
        """A speed for the controller (absolute limit). Sent before every FollowPath and
        repeated every tick while a straight runs: the value changes along a boosted step
        (taper), and the repeat also guards against a controller that (re)started."""
        m = SpeedLimit()
        m.header.stamp = self.get_clock().now().to_msg()
        m.percentage = False
        m.speed_limit = v
        self._speed_pub.publish(m)

    def _send_follow(self, st: CompiledStep, pose) -> bool:
        along, _ = self._along_cross(st, pose)
        path = self._follow_path(st, along)
        if len(path.poses) < 2:
            return False
        self._publish_speed(self._step_speed(st, pose))
        goal = FollowPath.Goal(path=path, controller_id="FollowPath", goal_checker_id="precise")
        return self._send(self._follow, goal)

    def _send_spin(self, st: CompiledStep, pose) -> bool:
        remaining = self._remaining_turn(st)
        if self.turn_centre is None:
            self.turn_centre = self._odom_xy
            # A Spin is relative. Fold the entry heading error into this turn (bounded, same
            # direction) so consecutive turns land on the DRAWN heading instead of stacking errors.
            entry = wrap(st.start[2] - pose[2])
            entry = max(-self.entry_corr_max, min(self.entry_corr_max, entry))
            self.turn_target = st.signed_angle_rad + entry
            remaining = self._remaining_turn(st)
            self.get_logger().info(
                f"turn start: odom {self._odom_xy}, amcl {pose[:2]}, "
                f"remaining {math.degrees(remaining):+.1f} deg"
            )
        goal = Spin.Goal()
        goal.target_yaw = float(remaining)
        allowance = st.time_allowance_s * abs(remaining) / max(abs(st.signed_angle_rad), 1e-6) + 2.0
        goal.time_allowance = Duration(seconds=allowance).to_msg()
        # rebase: fold everything measured so far, keep counting from here
        self.turn_travelled, self.turn_acc0 = self._turn_travel(), self._odom_yaw_acc
        self.turn_started_t = self._now()
        self.turn_feedback = 0.0
        return self._send(self._spin, goal)

    def _send(self, client: ActionClient, goal) -> bool:
        if not client.wait_for_server(timeout_sec=2.0):
            self._fault(f"action server {client._action_name} unavailable", "EXEC_ACTION_FAILED")
            return False
        try:
            self.goals.send(
                client, goal, self.fsm.run_id, self.fsm.pass_index, self.fsm.step_index, self._on_feedback
            )
        except Exception as e:  # noqa: BLE001
            self._fault(f"action goal send failed: {e}", "EXEC_ACTION_FAILED")
            return False
        return True

    def _on_goal_error(self, attempt: ga.Attempt, why: str) -> None:
        """Current attempt rejected or failed (called under the lock by GoalAttempts)."""
        if self.fsm.accepts(attempt.run_id):
            self._fault(why)

    def _on_feedback(self, fb) -> None:
        if fb and hasattr(fb.feedback, "angular_distance_traveled"):
            self.turn_feedback = float(fb.feedback.angular_distance_traveled)

    def _interrupt(self, why: str) -> None:
        """Inhibit at once (the permit drops on the next tick), then cancel the action.
        Turn travel keeps being counted from odometry through the stop (R18)."""
        self.goals.revoke()
        self.phase, self.paused_t, self.odom_gap = PHASE_INIT, self._now(), False
        self._log_state(why)

    def _fault(self, why: str, code: str = "EXEC_ACTION_FAILED") -> None:
        """Terminal for this run. `code` is the alarm-catalogue code the operator sees
        (agv_core/alarms.py); `why` stays the engineer's sentence in `reason`."""
        self.fault_code = code
        if self.fsm.fault(why):
            self._interrupt(why)

    # ---- the tick ---------------------------------------------------------------------------------

    def _tick(self) -> None:
        with self._lock:
            state = self.fsm.state
            now = self._now()
            if state != fsm.BLOCKED:
                self.hold_cause, self._pending_since = "", None  # a hold ends with its BLOCKED
            if state in fsm.ACTIVE or state == fsm.READY:
                pre = self._prereqs()
                held = self._prereq_held(now, pre)
                # an explicit MANUAL on a fresh, valid panel image (a stale panel is a fault)
                manual = (
                    state in fsm.ACTIVE
                    and self._panel is not None
                    and self._panel.valid
                    and not self._panel.mode_auto
                    and now - self._panel_t <= self.panel_age
                )
                if manual and (self.hold_cause in ("pending", "field", "estop") or pre is None):
                    self.fsm.abort("manual takeover: selector left AUTO")
                    self._interrupt("manual takeover")
                elif self._torque_off(now) and (
                    (state == fsm.EXECUTING and self.phase != PHASE_PREMOVE)
                    or (state == fsm.BLOCKED and self.hold_cause not in ("field", "estop"))
                ):
                    # the safety chain took the torque: hold, don't fault (auto-resume plan);
                    # a hold that was already on (a controller abort, then someone stepping
                    # into the field) takes the safety cause, so it resumes on the field's terms
                    self._hold(self._safety_cause(now), "drives lost torque")
                elif self.hold_cause == "pending" and state == fsm.BLOCKED:
                    self._classify_pending(now, pre)
                elif pre:
                    if state == fsm.READY:
                        # a momentary lapse while waiting must not throw the loaded mission away
                        if held:
                            self.fsm.unready(pre)
                    elif (
                        self._wheel_prereq(pre)
                        and state == fsm.EXECUTING
                        and self.phase != PHASE_PREMOVE  # nothing moves yet: debounced below
                        and self.auto_resume_enabled
                    ):
                        # stop now, WITHOUT the grace period: losing wheel feedback while moving
                        # is stopped at once and recovers by itself. The drive report (10 Hz)
                        # then says within safety_window_s whether this was the safety chain (a
                        # hold) or a real feedback loss (a fault)
                        self._hold("pending", f"stopped: {pre}")
                    elif self._wheel_prereq(pre) and self._excused_in_hold(now):
                        pass  # no torque: no wheel feedback is expected until the drives re-arm
                    elif held:
                        self._fault(f"{pre} for {self.prereq_grace_s:.1f} s", "EXEC_PREREQ_LOST")
                elif state in fsm.ACTIVE and not self._auto():
                    self.fsm.abort("manual takeover: selector left AUTO")
                    self._interrupt("manual takeover")
            else:
                # IDLE / DONE / FAULT: nothing is waiting on a prerequisite, so the grace timer
                # must not survive into the next mission and fire on its first tick
                self._prereq_since = None
            if self.fsm.state == fsm.EXECUTING:
                self._execute(now)
            elif self.fsm.state in (fsm.PAUSED, fsm.BLOCKED):
                if self.fsm.state == fsm.BLOCKED:
                    self._auto_resume_tick(now)
            self._publish_permit()
            self._log_state()

    def _classify_pending(self, now: float, pre: str | None) -> None:
        if self._torque_off(now) or (
            self._last_off_t is not None and self._last_off_t >= self._pending_since
        ):
            self._hold(self._safety_cause(now), "drives lost torque")
        elif now - self._pending_since > self.safety_window:
            self._fault(f"{pre or 'wheel feedback lost'} without a safety stop", "EXEC_PREREQ_LOST")

    def _excused_in_hold(self, now: float) -> bool:
        """Wheel feedback may be missing during a safety hold: while the drives report no
        torque, and for safety_window_s after they got it back (re-arming)."""
        paused = self.fsm.state == fsm.PAUSED  # an operator Pause, then the E-stop: still paused
        if not paused and (self.fsm.state != fsm.BLOCKED or self.hold_cause not in ("field", "estop")):
            return False
        if self._torque_off(now):
            return True
        return self._last_off_t is not None and now - self._last_off_t <= self.safety_window + 2.0

    def _auto_resume_tick(self, now: float) -> None:
        """Resume a BLOCKED run by itself once its cause cleared and stayed clear."""
        if not self.auto_resume_enabled or self.hold_cause not in self._auto_causes():
            self._auto_since = None
            return
        ok, why = self._resume_checks()
        if ok and self._torque_off(now):
            ok, why = False, "drives have no torque"
        if ok and self._field_clear is False:
            ok, why = False, "protective field not clear"
        base = self.fsm.reason.split(" | ", 1)[0]
        if not ok:
            self._auto_since = None
            self.fsm.reason = f"{base} | auto-resume waiting: {why}"
            return
        if self._auto_since is None:
            self._auto_since = now
            self.fsm.reason = f"{base} | auto-resume: clear, continuing in {self.auto_clear_s:.0f} s"
            return
        if now - self._auto_since < self.auto_clear_s:
            return
        cause = self.hold_cause
        if cause == "controller":
            self._aborts += 1
        if self.fsm.auto_resume(f"auto-resumed after {cause}"):
            self.hold_cause, self._auto_since = "", None
            self.phase = PHASE_INIT
            self._log_state("auto-resume")

    def _execute(self, now: float) -> None:
        st = self._step()
        pose = self._pose()
        if st is None or pose is None:
            self._fault("no step or no pose", "EXEC_PREREQ_LOST")
            return
        if self.phase == PHASE_PREMOVE:
            self._premove_tick(now)
            return
        if self.phase == PHASE_INIT:
            stale = self.goals.obsolete_outstanding()
            if stale:
                # R08: an interrupted goal must report terminal before a replacement is issued
                if now - min(stale.values()) > self.goal_cancel_timeout:
                    self.goals.forget_obsolete()
                    self._fault(
                        f"previous action goal not terminated within {self.goal_cancel_timeout:.1f} s",
                        "EXEC_ACTION_FAILED",
                    )
                return
            if st.type != ROTATE:
                if not self._send_follow(st, pose):
                    if self.fsm.state == fsm.EXECUTING:
                        # nothing left of the segment: treat as arrived, verify in settle
                        self.phase, self.settle_since = PHASE_SETTLE, None
                    return
                if self.fsm.state == fsm.EXECUTING:  # a synchronous rejection may already have faulted
                    self.phase = PHASE_GOAL
            else:
                if not self._wheels_still:
                    return  # translation must stop before rotation
                if abs(self._remaining_turn(st)) < self.turn_tol:
                    self.phase, self.settle_since = PHASE_SETTLE, None
                    return
                if self._send_spin(st, pose) and self.fsm.state == fsm.EXECUTING:
                    self.phase = PHASE_GOAL
            return

        if self.phase == PHASE_GOAL:
            if st.type != ROTATE:
                along, cross = self._along_cross(st, pose)
                self.cross_track = cross
                # After a turn the line starts with the previous stop's error plus the
                # estimator's wander during the spin; RPP pulls back onto the line within
                # a metre. Until then twice the limit. (Q05's clamp to the validated
                # footprint margin was dropped 2026-09-17 with the margin at 0.05 m.)
                allowed = self._cross_track_allowed(along)
                if abs(cross) > allowed:
                    self._fault(
                        f"cross-track {cross:+.2f} m exceeds {allowed:.2f} m at {along:.2f} m along",
                        "EXEC_OFF_PATH",
                    )
                    return
                if along >= st.length_m and self._next_in_chain() is not None:
                    # a chained boundary: the same FollowPath goal carries on into the next step,
                    # the executor just moves its bookkeeping (progress, cross-track, envelope,
                    # speed) to it; no stop, no settle, no verification here
                    self.get_logger().info(
                        f"step {st.id} ({st.type}) passed ({self.fsm.progress()}): "
                        f"{cross:+.3f} m cross-track, continuing"
                    )
                    self.fsm.step_done()
                    self.cross_track = 0.0
                    return
                if along > st.length_m + 2 * self.route.limits.position_tolerance_m:
                    self._fault(f"passed the endpoint by {along - st.length_m:.2f} m", "EXEC_OVERSHOOT")
                    return
                if along >= st.length_m:
                    # Arrived along the line before Nav2's goal checker fired: its window is a
                    # 2.5 cm CIRCLE, and a few cm of lateral offset (normal right after a turn)
                    # takes the vehicle past the endpoint beside it, after which RPP keeps
                    # driving (vehicle 2026-09-17: faulted at +0.10 m). Stop here; settle then
                    # verifies the stop like any other (verify_position_m).
                    self.goals.revoke()
                    self.phase, self.settle_since = PHASE_SETTLE, None
                    return
            else:
                travelled = self._turn_travel()
                drift = math.hypot(
                    self._odom_xy[0] - self.turn_centre[0], self._odom_xy[1] - self.turn_centre[1]
                )
                if drift > self.centre_drift_m:
                    self._fault(f"turn centre drifted {drift:.2f} m", "EXEC_TURN_FAILED")
                    return
                if (
                    now - self.turn_started_t > 1.0
                    and travelled * st.signed_angle_rad < 0
                    and abs(travelled) > self.wrong_way
                ):
                    self._fault(
                        f"turning the wrong way ({math.degrees(travelled):+.1f} deg)", "EXEC_TURN_FAILED"
                    )
                    return
                target = self.turn_target if self.turn_target is not None else st.signed_angle_rad
                if abs(travelled) > abs(target) + self.turn_tol + math.radians(3.0):
                    self._fault(
                        f"turn overshot: {math.degrees(travelled):+.1f} of "
                        f"{math.degrees(st.signed_angle_rad):+.1f} deg",
                        "EXEC_OVERSHOOT",
                    )
                    return
            if self.goals.current is None:
                self._fault("action goal lost", "EXEC_ACTION_FAILED")
                return
            if self.goals.result is None:
                if (
                    self.goals.handle is None
                    and self.goals.sent_t is not None
                    and now - self.goals.sent_t > self.goal_accept_timeout
                ):
                    self._fault(
                        f"action goal not accepted within {self.goal_accept_timeout:.1f} s",
                        "EXEC_ACTION_FAILED",
                    )
                return
            if self.goals.result != ga.SUCCEEDED:
                if self.goals.result == ga.ABORTED and self.auto_resume_enabled:
                    if self._torque_off(now) or self._field_tripped(now):
                        self._hold(self._safety_cause(now), f"{st.type} action aborted during a safety stop")
                        return
                    if self._aborts < self.abort_retries:
                        self._hold("controller", f"controller aborted the {st.type} (collision ahead?)")
                        return
                self._fault(f"action {self.goals.result}", "EXEC_ACTION_FAILED")
                return
            # the chain's goal ends at its LAST step: if it finished while the bookkeeping was
            # still on an earlier chained step, move on so settle verifies the right endpoint
            while self._next_in_chain() is not None and self.fsm.state == fsm.EXECUTING:
                self.fsm.step_done()
            self.phase, self.settle_since = PHASE_SETTLE, None
            return

        if self.phase == PHASE_SETTLE:
            if not self._wheels_still:
                self.settle_since = None
                return
            if self.settle_since is None:
                self.settle_since = now
                return
            if now - self.settle_since < self.settle_s:
                return
            # verify the end of the step (spec §5.2, §5.3): position/heading against the
            # estimated pose with the verification allowance; turn travel from odometry.
            d = math.hypot(pose[0] - st.end[0], pose[1] - st.end[1])
            a = abs(wrap(pose[2] - st.end[2]))
            if st.type != ROTATE:
                if d > self.verify_pos or a > self.verify_yaw:
                    self._fault(
                        f"endpoint missed: {d:.3f} m / {math.degrees(a):.1f} deg", "EXEC_ENDPOINT_MISSED"
                    )
                    return
            else:
                self.turn_travelled, self.turn_acc0 = self._turn_travel(), None  # fold for verification
                target = self.turn_target if self.turn_target is not None else st.signed_angle_rad
                travel_err = abs(abs(self.turn_travelled) - abs(target))
                centre = self.turn_centre or self._odom_xy
                drift = math.hypot(self._odom_xy[0] - centre[0], self._odom_xy[1] - centre[1])
                self.get_logger().info(
                    f"turn check: travelled {math.degrees(self.turn_travelled):+.1f} of "
                    f"{math.degrees(st.signed_angle_rad):+.1f} deg, heading err {math.degrees(a):.2f} deg, "
                    f"odom drift {drift:.3f} m, amcl vs expected {d:.3f} m"
                )
                if travel_err > self.turn_tol or a > self.verify_yaw or drift > self.centre_drift_m:
                    self._fault(
                        f"turn out of tolerance: travelled {math.degrees(self.turn_travelled):+.1f} of "
                        f"{math.degrees(st.signed_angle_rad):+.1f} deg, "
                        f"heading err {math.degrees(a):.1f} deg, odom drift {drift:.3f} m",
                        "EXEC_TURN_FAILED",
                    )
                    return
            self.get_logger().info(
                f"step {st.id} ({st.type}) done ({self.fsm.progress()}): "
                f"{d:.3f} m / {math.degrees(a):.2f} deg from expected"
            )
            self.fsm.step_done()
            self._reset_step_state()

    # ---- outputs ----------------------------------------------------------------------------------

    def _publish_permit(self) -> None:
        m = MotionPermit()
        m.generation = self.generation
        m.instance = self._lease_instance
        self._permit_seq += 1
        m.seq = self._permit_seq
        m.header.stamp = self.get_clock().now().to_msg()
        m.run_id = self.fsm.run_id
        st = self._step()
        if self.fsm.state == fsm.EXECUTING and self.phase == PHASE_GOAL and st is not None:
            m.source = MotionPermit.FOLLOW if st.type != ROTATE else MotionPermit.ROTATE
            m.enabled = True
            m.reason = f"step {st.id} {self.fsm.progress()}"
            # the STEP's caps ride with the permission; the mux enforces them (Q04). A straight
            # runs at its compiled speed (linear_mps, or the long-straight boost); a rotation
            # at the route's angular cap, clamped to the vehicle ceiling like Spin does.
            if st.type != ROTATE:
                m.v_max = self._step_speed(st, self._pose())
                self._publish_speed(m.v_max)
            else:
                m.v_max = float(self.route.limits.linear_mps)
            m.w_max = self._step_w_max(st)
        else:
            m.source, m.enabled, m.reason = MotionPermit.NONE, False, fsm.NAMES[self.fsm.state]
        self._permit_pub.publish(m)

    def _log_state(self, note: str = "") -> None:
        key = (
            self.fsm.state,
            self.fsm.reason,
            self.fsm.pass_index,
            self.fsm.step_index,
            self.phase,
            self.fsm.resume_prepared,
        )
        if key != self._last_state_key:
            self._last_state_key = key
            self.get_logger().info(
                f"{fsm.NAMES[self.fsm.state]} {self.fsm.progress()} {self.phase}: {self.fsm.reason}"
                + (f" [{note}]" if note else "")
            )
            if self.fsm.state != self._last_event_state:
                self._run_summary(self._last_event_state)
                self._event()
                self._last_event_state = self.fsm.state
            self._publish_state()

    def _event(self) -> None:
        """One operator event per RUN-STATE change (never per tick, never per phase)."""
        code = (
            alarms.hold_code(self.hold_cause)
            if self.fsm.state == fsm.BLOCKED
            else (self.fault_code or "EXEC_ACTION_FAILED")
            if self.fsm.state == fsm.FAULT
            else "MODE_CHANGE"
        )
        row = alarms.get(code)
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source = "route_executor"
        m.level = {alarms.ERROR: Event.ERROR, alarms.WARN: Event.WARN}.get(row.severity, Event.INFO)
        m.code = code
        m.text = f"run {fsm.NAMES[self.fsm.state]} {self.fsm.progress()}: {self.fsm.reason}"
        self._event_seq += 1
        m.seq = self._event_seq
        self._event_pub.publish(m)

    def _run_summary(self, previous: int | None) -> None:
        """One line per finished run, for the operator to read back (plan phase 2.3):
        what ran, how long, how far off the line it got, and what held it on the way."""
        state = self.fsm.state
        if state in (*fsm.ACTIVE, fsm.READY) and previous in (None, fsm.IDLE, fsm.DONE, fsm.FAULT):
            self._run_started = self._now()
            self._run_holds = {}
            self._run_peak_cross = 0.0
        if state == fsm.BLOCKED and self.hold_cause:
            self._run_holds[self.hold_cause] = self._run_holds.get(self.hold_cause, 0) + 1
        self._run_peak_cross = max(self._run_peak_cross, abs(self.cross_track))
        if state not in (fsm.DONE, fsm.FAULT, fsm.IDLE) or previous in (None, fsm.IDLE):
            return
        if self._run_started is None:
            return
        holds = ", ".join(f"{k} x{n}" for k, n in sorted(self._run_holds.items())) or "none"
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source = "route_executor"
        m.level = Event.ERROR if state == fsm.FAULT else Event.INFO
        m.code = "RUN_SUMMARY"
        m.text = (
            f"{self.fsm.mission_id or 'run'} {fsm.NAMES[state]} after "
            f"{self._now() - self._run_started:.0f} s; peak cross-track "
            f"{self._run_peak_cross:.2f} m; holds: {holds}"
        )
        self._event_seq += 1
        m.seq = self._event_seq
        self._event_pub.publish(m)
        self._run_started = None

    def _publish_state(self) -> None:
        m = RunState()
        m.generation = self.generation
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = self.fsm.state
        m.run_id, m.mission_id = self.fsm.run_id, self.fsm.mission_id
        if self.manifest is not None:
            m.map_id, m.map_revision = self.manifest.map_id, self.manifest.revision
        if self.route is not None:
            m.route_id, m.route_revision = self.route.route_id, self.route.revision
        st = self._step()
        m.step_index = self.fsm.step_index
        m.step_id, m.step_type = (st.id, st.type) if st is not None else ("", "")
        if st is not None and st.type == ROTATE:
            m.remaining_turn_rad = self._remaining_turn(st)
        m.step_v_mps = float(st.v_mps) if st is not None and st.type != ROTATE else 0.0
        m.cross_track_m = self.cross_track
        m.localization_state = self._loc.state if self._loc is not None else 0
        m.resume_prepared = self.fsm.resume_prepared
        blocked = self.fsm.state == fsm.BLOCKED
        m.hold_cause = self.hold_cause if blocked else ""
        m.fault_code = (
            alarms.hold_code(self.hold_cause)
            if blocked
            else (self.fault_code if self.fsm.state == fsm.FAULT else "")
        )
        m.auto_resume = blocked and self.auto_resume_enabled and self.hold_cause in self._auto_causes()
        m.drive_power = self.drive_power()
        m.premove = self.fsm.state == fsm.EXECUTING and self.phase == PHASE_PREMOVE
        pose = self._pose()
        if pose is not None:
            m.pose_valid, m.pose_x, m.pose_y, m.pose_yaw = True, pose[0], pose[1], pose[2]
        # TODO(interface): RunState has no pass fields; carry them in the reason for now (R20)
        m.reason = self.fsm.reason
        if self.fsm.n_passes > 1 and self.fsm.state in (*fsm.ACTIVE, fsm.DONE):
            m.reason += f" [pass {self.fsm.pass_index + 1}/{self.fsm.n_passes}]"
        self._state_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
