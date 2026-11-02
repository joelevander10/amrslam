"""Command mux + permission gating + slew limit + inverse kinematics (spec §3.5, §3.7).

Sources: the jog pendant levels in /amr/panel_state (panel MANUAL, first
choice while a direction is held), /amr/manual_command (browser jog, panel
MANUAL), /cmd_vel_teleop (engineering keyboard, panel MANUAL,
`teleop_enabled`), the navigation layer's
generation-private /amr/layers/<gen>/cmd_vel (controller_server, permit FOLLOW)
and .../cmd_vel_rotate (behavior_server, permit ROTATE) -> /cmd_wheel_vel at
50 Hz, stamped with the applied supervisor generation.

Authority is decided by amr_base.gating from the supervisor's ControlLease
(`require_supervisor`), the panel image, the drive owner's status and the
executor's MotionPermit; a command must also be fresh (0.2 s). Loss of
authority or a fault zeroes the output at once, not through the ramp. A lease
generation change clears every cached command, permit and slew state and
re-subscribes the navigation inputs: nothing from the old layer can be replayed
into the new one. /amr/mux_state (10 Hz) is the transition barrier's
acknowledgement that a new (inhibited) generation has been applied.

Acceleration limits default to the drives' own 6083h ramp from the profile
(reconciliation D-1): a controller allowed to demand more than the drives can
slew diverges. A parameter may lower them, never raise them above hardware.
"""

import math

import rclpy
from agv_core import config, kinematics
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from amr_base import gating
from amr_base.diff_drive import Geometry, clamp_wheels, inverse, scurve, slew, slew_asym

try:
    # The scanner's field outputs come from the SICK driver. Optional at import so a
    # bench without the package still starts; field_source then decides (see below).
    from sick_safetyscanners2_interfaces.msg import OutputPaths
except ImportError:  # pragma: no cover - present on the vehicle
    OutputPaths = None

from amr_interfaces.msg import (
    ControlLease,
    DriveStatus,
    Event,
    LineState,
    ManualCommand,
    ModeState,
    MotionPermit,
    MuxState,
    PanelState,
    WheelVelocities,
)

# /amr/line_state arrives on change and at 1 Hz: older than this, no U-turn exemption.
LINE_STATE_FRESH_S = 2.5

# Sources a person drives by hand: switching between these is not operator history.
HAND_SOURCES = {"none", "manual", "pendant", "teleop"}

RELIABLE_1 = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)
LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

WHEEL_RAD_S_PER_MOTOR_RPM = 2.0 * math.pi / 60.0 / config.GEAR_RATIO


class CmdMuxKinematics(Node):
    # A class-level default, not only an __init__ assignment: several tests
    # build this node with __new__ and set the attributes they care about by
    # hand, and the 50 Hz tick reads self._line unconditionally. Without this
    # a new cache attribute turns every one of those into an AttributeError
    # from inside the tick, which is a poor way to learn about it.
    _line: gating.Stamped | None = None
    # Same reason: the field state the tick reads (2026-10-02). Assume-clear by default
    # so a test built with __new__ that never sets fp keeps its old behaviour.
    _field: gating.Field | None = None
    fp = gating.FieldParams(assume_clear=True)
    _view = gating.FieldView(True, True, False)
    _scale = 1.0
    _speed_ramp: gating.SpeedRamp | None = None
    line_alpha_max = 0.0  # 0 here = use alpha_max (tests built with __new__)
    line_d_max = 0.0  # 0 here = use d_max (tests built with __new__)
    line_uturn_exempt_mps = 0.0  # 0 here = no U-turn warning-1 exemption (tests built with __new__)
    _line_uturn: tuple[float, str] | None = None
    _target = 1.0

    def __init__(self) -> None:
        super().__init__("cmd_mux_kinematics")
        hw_a_max = config.ACCEL_RPM_S * config.MPS_PER_RPM
        hw_alpha_max = kinematics.max_yaw_accel(config.ACCEL_RPM_S)
        # A warning-field stop brakes at what the distance needs, up to the drive's own
        # deceleration ramp (6084h), not the gentle d_max: 0.85 m/s in 0.40 m is 0.90 m/s^2.
        self.stop_d_max = config.DECEL_RPM_S * config.MPS_PER_RPM
        self.stop_delta_max = kinematics.max_yaw_accel(config.DECEL_RPM_S)

        self.declare_parameter("wheel_radius_m", config.WHEEL_DIA_M / 2.0)
        self.declare_parameter("track_width_m", config.TRACK_M)
        self.declare_parameter("wheel_vel_max_rad_s", config.MOTOR_MAX_RPM * WHEEL_RAD_S_PER_MOTOR_RPM)
        self.declare_parameter("a_max", min(0.5, hw_a_max))
        self.declare_parameter("alpha_max", min(1.0, hw_alpha_max))
        # Deceleration limits (speed shrinking). Default = the acceleration limits, so a stop is
        # never slower than before; base.launch lowers a_max/alpha_max for a gentle start on
        # the vehicle (operator: 0 -> 0.3 m/s in 0.6 s was abrupt, 2026-09-17) without
        # lengthening the stopping distance.
        self.declare_parameter("d_max", 0.0)  # 0 = same as a_max
        self.declare_parameter("delta_max", 0.0)  # 0 = same as alpha_max
        # Manual sources (pendant, browser jog) follow a jerk-limited S-curve instead of the
        # autonomous ramp (2026-09-18): the acceleration builds at manual_jerk to manual_a_max
        # (0.3 m/s^2 asked by the operator), stops still use d_max/delta_max.
        self.declare_parameter("manual_a_max", 0.3)
        self.declare_parameter("manual_jerk", 1.0)  # m/s^3: 0.3 s to full acceleration
        self.declare_parameter("manual_alpha_max", 0.0)  # 0 = same as alpha_max
        self.declare_parameter("manual_jerk_w", 2.0)  # rad/s^3
        # LINE steering slews its yaw rate at the drive's own limit, not at the gentle Nav2
        # alpha_max (2026-10-02): at the 0.85 m/s tape cruise the follower's corrections
        # need ~27 rad/s^2 per metre of wobble, so 0.4 rad/s^2 rate-limited any wobble over
        # ~15 mm and the loop oscillated. gy-demo drove the wheels with only the drive ramp.
        self.declare_parameter("line_alpha_max", 0.0)  # 0 = the hardware yaw limit
        # LINE deceleration at the follower's own rate (tracked-speed-plan-1, 2026-10-08):
        # a measured 0.5 m stop from 0.85 m/s needs 0.72 m/s^2 and a late high-zone exit
        # up to ~0.95 m/s^2; at the Nav2 d_max 0.5 the mux would carry the vehicle past
        # the mark. The follower already shapes every stop and speed change; this only
        # stops the mux reshaping them. Acceleration is unchanged (a_max).
        self.declare_parameter("line_d_max", 0.0)  # 0 = the drive's deceleration (6084h)
        # Warning 1 does not slow a LINE U-turn at or below this body speed (the 0.1 m/s
        # creep, the pivot); warning 2 still stops it. 0 = off (gating.line_uturn_exempt).
        self.declare_parameter("line_uturn_exempt_mps", 0.12)
        self.declare_parameter("teleop_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.2)
        self.declare_parameter("permit_timeout_s", 0.3)
        self.declare_parameter("panel_timeout_s", 0.2)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("require_supervisor", False)  # production: True (unified plan §4.2)
        self.declare_parameter("teleop_enabled", True)  # /cmd_vel_teleop, engineering only
        self.declare_parameter("pendant_v_m_s", 0.50)
        self.declare_parameter("pendant_w_rad_s", 0.39)  # spin in place (0.30 until 2026-09-19, +30 %)
        self.declare_parameter("pendant_turn_ratio", 0.66)  # slow wheel / fast wheel while driving + turning
        # manual spin cap while SURVEYING (supervisor mode MAPPING), pendant and browser jog alike:
        # 0.39 x 0.7 (2026-09-19) - a fast spin smears each scan and a survey came out rotated
        self.declare_parameter("survey_w_max_rad_s", 0.27)
        self.declare_parameter("lease_timeout_s", 0.3)
        self.declare_parameter("drives_timeout_s", 0.3)
        # Scanner fields (2026-10-02): zero AUTO on the protective field, scale it while
        # a warning field is occupied. base.launch fills these from scanner_fields.yaml.
        self.declare_parameter("field_source", "scanner")  # scanner | assume_clear (sim only)
        self.declare_parameter("protective_index", 0)
        self.declare_parameter("warning_indices", [2, 1])  # outer (warning 1) first
        self.declare_parameter("warning_scales", [0.5, 0.0])  # one factor per warning index; 0 = stop
        self.declare_parameter("warning_active_level", False)
        self.declare_parameter("warning_decel_s", 1.0)  # each step down, seconds
        self.declare_parameter("warning_accel_s", 1.0)  # each step up, seconds
        self.declare_parameter("warning_stop_m", 0.40)  # a factor-0 warning stops in this distance
        self.declare_parameter("field_fresh_s", 0.5)

        p = self.get_parameter
        self.geom = Geometry(p("wheel_radius_m").value, p("track_width_m").value)
        self.w_max = p("wheel_vel_max_rad_s").value
        self.a_max = min(p("a_max").value, hw_a_max)
        self.alpha_max = min(p("alpha_max").value, hw_alpha_max)
        self.d_max = min(p("d_max").value or self.a_max, hw_a_max)
        self.delta_max = min(p("delta_max").value or self.alpha_max, hw_alpha_max)
        self.manual_a_max = min(p("manual_a_max").value, hw_a_max)
        self.manual_alpha_max = min(p("manual_alpha_max").value or self.alpha_max, hw_alpha_max)
        self.line_alpha_max = min(p("line_alpha_max").value or hw_alpha_max, hw_alpha_max)
        hw_d_max = config.DECEL_RPM_S * config.MPS_PER_RPM
        self.line_d_max = min(p("line_d_max").value or hw_d_max, hw_d_max)
        self.line_uturn_exempt_mps = max(0.0, self._finite_or(float(p("line_uturn_exempt_mps").value), 0.0))
        self.manual_jerk = max(1e-3, float(p("manual_jerk").value))
        self.manual_jerk_w = max(1e-3, float(p("manual_jerk_w").value))
        if self.a_max < p("a_max").value or self.alpha_max < p("alpha_max").value:
            self.get_logger().warn(
                f"accel limits clamped to hardware: a_max={self.a_max:.3f} "
                f"alpha_max={self.alpha_max:.3f} (6083h = {config.ACCEL_RPM_S} r/min/s)"
            )
        self.gp = gating.Params(
            cmd_timeout_s=p("cmd_timeout_s").value,
            teleop_window_s=p("teleop_timeout_s").value,
            permit_timeout_s=p("permit_timeout_s").value,
            panel_timeout_s=p("panel_timeout_s").value,
            lease_timeout_s=p("lease_timeout_s").value,
            drives_timeout_s=p("drives_timeout_s").value,
            require_supervisor=bool(p("require_supervisor").value),
            teleop_enabled=bool(p("teleop_enabled").value),
            pendant_v=max(0.0, float(p("pendant_v_m_s").value)),
            pendant_w=max(0.0, float(p("pendant_w_rad_s").value)),
            pendant_turn_ratio=min(1.0, max(0.0, float(p("pendant_turn_ratio").value))),
            track_m=float(p("track_width_m").value),
        )
        self.fp = gating.FieldParams(
            protective_index=int(p("protective_index").value),
            warning_indices=tuple(int(i) for i in p("warning_indices").value),
            warning_active_level=bool(p("warning_active_level").value),
            warning_scales=tuple(min(1.0, max(0.0, self._finite_or(float(k), 0.0)))
                                 for k in p("warning_scales").value),
            warning_decel_s=self._finite_or(float(p("warning_decel_s").value), 1.0),
            warning_accel_s=self._finite_or(float(p("warning_accel_s").value), 1.0),
            warning_stop_m=max(0.0, self._finite_or(float(p("warning_stop_m").value), 0.40)),
            fresh_s=self._finite_or(float(p("field_fresh_s").value), 0.5),
            assume_clear=str(p("field_source").value) == "assume_clear",
        )
        self.dt = 1.0 / p("rate_hz").value
        self.survey_w_max = max(0.0, float(p("survey_w_max_rad_s").value))
        self._surveying = False  # the supervisor's last /amr/mode_state said MAPPING

        self._teleop: gating.Stamped | None = None
        self._follow: gating.Stamped | None = None
        self._rotate: gating.Stamped | None = None
        self._permit: gating.Permit | None = None
        self._panel: gating.Panel | None = None
        self._lease: gating.Lease | None = None
        self._manual = gating.ManualIntake()
        self._drives: gating.Drives | None = None
        self._commissioning: gating.Wheels | None = None
        self._line: gating.Stamped | None = None
        self._field: gating.Field | None = None
        self._view = gating.FieldView(False, False, False)
        self._scale = 1.0
        self._speed_ramp = gating.SpeedRamp()  # the ramped warning factor actually applied to AUTO
        self._target = 1.0
        self._wl = self._wr = 0.0  # per-wheel slew state for the COMMISSIONING source
        self._applied_gen = 0  # the lease generation the subscriptions/caches belong to
        self._applied_instance = ""
        self._v = 0.0
        self._wz = 0.0
        self._a = self._alpha = 0.0  # S-curve acceleration state (manual sources only)
        self._source = "none"
        self._reason = ""
        self._last = gating.Selection(gating.NONE, 0.0, 0.0, "", 0, False)

        self.create_subscription(Twist, "/cmd_vel_teleop", self._on_teleop, RELIABLE_1)
        self._nav_subs: list = []
        self._subscribe_nav(0)
        self.create_subscription(MotionPermit, "/amr/motion_permit", self._on_permit, RELIABLE_1)
        self.create_subscription(PanelState, "/amr/panel_state", self._on_panel, 10)
        # The U-turn phase, for the warning-1 exemption. On change + 1 Hz, latched.
        self.create_subscription(LineState, "/amr/line_state", self._on_line_state, LATCHED)
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, RELIABLE_1)
        self.create_subscription(ManualCommand, "/amr/manual_command", self._on_manual, RELIABLE_1)
        self.create_subscription(DriveStatus, "/drives/status", self._on_drives, RELIABLE_1)
        self.create_subscription(ModeState, "/amr/mode_state", self._on_mode, LATCHED)
        self.create_subscription(
            WheelVelocities, "/amr/commissioning_wheels", self._on_commissioning, RELIABLE_1
        )
        if self.fp.assume_clear:
            self.get_logger().warning("field_source=assume_clear: scanner fields ASSUMED clear (sim only)")
        elif OutputPaths is not None:
            sensor = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                durability=QoSDurabilityPolicy.VOLATILE)
            self.create_subscription(OutputPaths, "/output_paths", self._on_output_paths, sensor)
        else:
            self.get_logger().error(
                "sick_safetyscanners2_interfaces missing: fields unknown, supervised AUTO stays zero"
            )
        self._pub = self.create_publisher(WheelVelocities, "/cmd_wheel_vel", RELIABLE_1)
        self._pub_state = self.create_publisher(MuxState, "/amr/mux_state", RELIABLE_1)
        # Operator events: EDGES only (the tick runs at 50 Hz). One per source change and
        # one per inhibit change - the same guard the source log already uses.
        self._pub_event = self.create_publisher(Event, "/amr/events", 50)
        self._event_seq = 0
        self._inhibited = False
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.1, self._publish_state)
        if not self.gp.require_supervisor:
            self.get_logger().warn(
                "require_supervisor=false: unsupervised bench mode, no ControlLease needed"
            )
        self.get_logger().info(
            f"r={self.geom.wheel_radius_m} track={self.geom.track_width_m} "
            f"w_max={self.w_max:.2f} rad/s a_max={self.a_max:.3f} alpha_max={self.alpha_max:.3f}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_teleop(self, msg: Twist) -> None:
        self._teleop = gating.finite_or_zero(self._now(), msg.linear.x, msg.angular.z)

    def _on_follow(self, msg: Twist) -> None:
        self._follow = gating.finite_or_zero(self._now(), msg.linear.x, msg.angular.z)

    def _on_rotate(self, msg: Twist) -> None:
        self._rotate = gating.finite_or_zero(self._now(), msg.linear.x, msg.angular.z)

    def _on_permit(self, msg: MotionPermit) -> None:
        cur = self._permit
        same_stream = cur is not None and cur.instance == msg.instance and cur.generation == msg.generation
        if same_stream and int(msg.seq) <= cur.seq and int(msg.seq) != 0:
            return  # an older sample cannot renew permission
        self._permit = gating.Permit(
            self._now(),
            int(msg.source),
            bool(msg.enabled),
            msg.instance,
            int(msg.generation),
            int(msg.seq),
            float(msg.v_max),
            float(msg.w_max),
        )

    def _on_panel(self, msg: PanelState) -> None:
        self._panel = gating.Panel(
            self._now(),
            bool(msg.valid),
            bool(msg.mode_auto),
            bool(msg.pendant_fwd),
            bool(msg.pendant_rvs),
            bool(msg.pendant_left),
            bool(msg.pendant_right),
        )

    def _on_commissioning(self, msg: WheelVelocities) -> None:
        wl, wr = float(msg.left_rad_s), float(msg.right_rad_s)
        if math.isfinite(wl) and math.isfinite(wr):
            self._commissioning = gating.Wheels(self._now(), wl, wr, int(msg.generation))
        else:
            self._commissioning = None  # never leave an earlier nonzero wheel target in force (R03)

    def _on_drives(self, msg: DriveStatus) -> None:
        self._drives = gating.Drives(self._now(), bool(msg.operational))

    def _on_manual(self, msg: ManualCommand) -> None:
        self._manual.offer(
            self._now(),
            msg.instance,
            int(msg.generation),
            msg.session,
            int(msg.seq),
            float(msg.valid_for_s),
            float(msg.v),
            float(msg.w),
        )

    def _on_lease(self, msg: ControlLease) -> None:
        cur = self._lease
        if cur is not None and cur.instance == msg.instance and cur.generation == msg.generation:
            if int(msg.seq) <= cur.seq:
                return  # stale sample cannot extend a lease
        self._lease = gating.Lease(
            self._now(), msg.instance, int(msg.generation), int(msg.seq), int(msg.allowed)
        )
        if (msg.instance, int(msg.generation)) != (self._applied_instance, self._applied_gen):
            self._apply_generation(msg.instance, int(msg.generation))

    def _apply_generation(self, instance: str, gen: int) -> None:
        """A new layer: forget every command, permit and ramp of the old one."""
        self.get_logger().info(
            f"supervisor generation {self._applied_gen} -> {gen} ({instance[:8]}): caches cleared"
        )
        self._applied_instance, self._applied_gen = instance, gen
        self._teleop = self._follow = self._rotate = None
        self._permit = None
        self._manual.clear()
        self._commissioning = None
        self._line = None
        self._v = self._wz = self._a = self._alpha = 0.0
        self._wl = self._wr = 0.0
        self._subscribe_nav(gen)

    def _subscribe_nav(self, gen: int) -> None:
        for sub in self._nav_subs:
            self.destroy_subscription(sub)
        self._nav_subs = [
            self.create_subscription(Twist, gating.nav_topic("/cmd_vel", gen), self._on_follow, RELIABLE_1),
            self.create_subscription(
                Twist, gating.nav_topic("/cmd_vel_rotate", gen), self._on_rotate, RELIABLE_1
            ),
            # The line follower publishes a body twist and holds no permit, so
            # it joins the generation-private set rather than carrying its own
            # generation field: a replaced layer's stream lands on a topic this
            # mux is no longer subscribed to and cannot look fresh.
            self.create_subscription(
                Twist, gating.nav_topic("/amr/line_cmd", gen), self._on_line, RELIABLE_1
            ),
        ]

    def _on_line_state(self, msg: LineState) -> None:
        phase = msg.uturn_phase if msg.state == LineState.RUNNING else ""
        self._line_uturn = (self._now(), phase)

    def _on_line(self, msg: Twist) -> None:
        self._line = gating.finite_or_zero(self._now(), msg.linear.x, msg.angular.z)

    def _on_mode(self, msg: ModeState) -> None:
        surveying = msg.mode == ModeState.MAPPING
        if surveying != self._surveying:
            self.get_logger().info(
                f"survey spin cap {self.survey_w_max:.2f} rad/s: {'on' if surveying else 'off'}"
            )
        self._surveying = surveying

    @staticmethod
    def _finite_or(x: float, default: float) -> float:
        """A limit parameter must be a finite non-negative number; anything
        else falls back to the shipped default rather than becoming 'no limit'."""
        return x if math.isfinite(x) and x >= 0.0 else default

    def _on_output_paths(self, m) -> None:
        status = getattr(m, "status", None)
        if status is None:
            return  # no evidence at all: the previous sample ages out
        self._field = gating.Field(self._now(), tuple(bool(x) for x in status))

    def _tick(self) -> None:
        sel = gating.select(
            self._now(),
            self._teleop,
            self._follow,
            self._rotate,
            self._permit,
            self._panel,
            self.gp,
            lease=self._lease,
            manual=self._manual.current,
            drives=self._drives,
            commissioning=self._commissioning,
            line=self._line,
        )
        self._view = gating.field_view(self._now(), self._field, self.fp)
        exempt = gating.line_uturn_exempt(
            sel, self._line_uturn, self._now(), LINE_STATE_FRESH_S, self.line_uturn_exempt_mps
        )
        sel, target = gating.field_limit(sel, self._view, self.gp, self.fp, slow_exempt=exempt)
        if self._speed_ramp is None:
            self._speed_ramp = gating.SpeedRamp()
        if sel.source in gating.AUTO_SOURCES:
            # Ramp the factor in time toward what the fields ask for, then apply it.
            k = self._speed_ramp.tick(target, self.dt, self.fp, v_now=self._v)
            sel = gating.apply_scale(sel, k)
            scale = k
        else:
            # Nothing autonomous is driving: no speed to ramp. Track the fields directly
            # so an AUTO start under a warning begins at the reduced factor.
            self._speed_ramp.snap(gating.warning_target(self._view, self.fp))
            scale = 0.0 if sel.code.startswith("FIELD_") else 1.0
        affected = target if sel.source != gating.NONE else 1.0
        if affected != self._target:
            # Changes only, and only while an AUTO source is affected: a warning field
            # that reaches a wall flickers all day under MANUAL and is nobody's history.
            down = affected < self._target
            self._target = affected
            self._event(
                Event.INFO,
                "FIELD_WARNING",
                f"warning field {self._view.warning_level}: auto stopping within {self.fp.warning_stop_m:g} m"
                if affected == 0.0 and down and self.fp.warning_stop_m > 0.0
                else f"warning field {self._view.warning_level}: auto speed ramping to x{affected:g} "
                f"over {self.fp.warning_decel_s:g} s"
                if affected < 1.0 and down
                else f"auto speed ramping up to x{affected:g} over {self.fp.warning_accel_s:g} s",
            )
        self._scale = scale
        name = gating.NAMES.get(sel.source, str(sel.source))  # never KeyError in the 50 Hz tick
        if name != self._source or (sel.source == gating.NONE and sel.reason != self._reason):
            self.get_logger().info(f"command source: {self._source} -> {name} ({sel.reason})")
            # A jog is a hundred edges: every press and release flips manual <-> none, and
            # at 300 entries the ring would hold nothing but somebody's thumb (vehicle,
            # 2026-09-22: 18 of 27 events in the first minute were jog flaps). Only a
            # change that involves an AUTONOMOUS source, or an inhibit, is history.
            if sel.inhibited or {name, self._source} - HAND_SOURCES:
                self._event(
                    Event.WARN if sel.inhibited else Event.INFO,
                    (sel.code or "NO_SOURCE") if sel.source == gating.NONE else "MUX_SOURCE",
                    f"command source {self._source} -> {name}: {sel.reason}",
                )
            self._source, self._reason = name, sel.reason
        if bool(sel.inhibited) != self._inhibited:
            self._inhibited = bool(sel.inhibited)
            # The CLEARING event carries INHIBITED too: it is the end of that condition,
            # and a "no longer inhibited" line filed under whatever came next (NO_SOURCE)
            # is exactly the code/text drift the catalogue exists to prevent.
            self._event(
                Event.WARN if self._inhibited else Event.INFO,
                (sel.code or "INHIBITED") if self._inhibited else "INHIBITED",
                ("motion inhibited: " + sel.reason) if self._inhibited else "motion no longer inhibited",
            )

        if sel.source == gating.NONE or (sel.v == 0.0 and sel.w == 0.0 and sel.reason.endswith("timed out")):
            self._v = self._wz = 0.0  # loss of authority or an expired command: zero at once, never a ramp
            self._a = self._alpha = 0.0
            self._wl = self._wr = 0.0
            wl, wr = 0.0, 0.0
        elif sel.wheels:
            # per-wheel targets (commissioning): the same hardware accel limit, applied per wheel
            a_wheel = self.a_max / self.geom.wheel_radius_m
            self._wl = slew(self._wl, sel.v, a_wheel, self.dt)
            self._wr = slew(self._wr, sel.w, a_wheel, self.dt)
            self._v = self._wz = 0.0
            wl, wr = clamp_wheels(self._wl, self._wr, self.w_max)
        elif sel.source in (gating.PENDANT, gating.MANUAL):
            # manual: jerk-limited S-curve (trapezoidal acceleration), stops at d_max/delta_max
            self._v, self._a = scurve(
                self._v, self._a, sel.v, self.manual_a_max, self.d_max, self.manual_jerk, self.dt
            )
            self._wz, self._alpha = scurve(
                self._wz,
                self._alpha,
                gating.survey_spin_cap(sel.w, self._surveying, self.survey_w_max),
                self.manual_alpha_max,
                self.delta_max,
                self.manual_jerk_w,
                self.dt,
            )
            self._wl = self._wr = 0.0
            wl, wr = clamp_wheels(*inverse(self.geom, self._v, self._wz), self.w_max)
        else:
            # the warning-field stop sets its own deceleration through the factor ramp;
            # the slew must not stretch it, so only the drive's ramp bounds it then
            stopping = self._speed_ramp.stopping
            d_max = max(self.d_max, getattr(self, "stop_d_max", 0.0)) if stopping else self.d_max
            if sel.source == gating.LINE and self.line_d_max:
                d_max = max(d_max, self.line_d_max)
            self._v = slew_asym(self._v, sel.v, self.a_max, d_max, self.dt)
            if sel.source == gating.LINE and self.line_alpha_max:
                yaw_up = yaw_down = self.line_alpha_max
            else:
                yaw_up, yaw_down = self.alpha_max, self.delta_max
            if stopping:
                yaw_down = max(yaw_down, getattr(self, "stop_delta_max", 0.0))
            self._wz = slew_asym(self._wz, sel.w, yaw_up, yaw_down, self.dt)
            self._a = self._alpha = 0.0
            self._wl = self._wr = 0.0
            wl, wr = clamp_wheels(*inverse(self.geom, self._v, self._wz), self.w_max)
        if not (math.isfinite(wl) and math.isfinite(wr)):
            # last line before the drive owner (R03): nothing upstream may turn into full scale
            self.get_logger().error(f"nonfinite wheel output ({wl}, {wr}) from {name}; zeroed")
            self._v = self._wz = self._wl = self._wr = self._a = self._alpha = 0.0
            wl, wr = 0.0, 0.0
        self._last = sel
        self._out = (wl, wr)
        out = WheelVelocities()
        out.header.stamp = self.get_clock().now().to_msg()
        out.generation = self._applied_gen
        out.left_rad_s = wl
        out.right_rad_s = wr
        self._pub.publish(out)

    _out = (0.0, 0.0)

    def _publish_state(self) -> None:
        m = MuxState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.instance = self._applied_instance
        m.generation = self._applied_gen
        m.source = self._last.source
        m.inhibited = bool(self._last.inhibited)
        m.left_rad_s, m.right_rad_s = self._out
        m.reason = self._last.reason
        m.code = self._last.code
        m.field_fresh = self._view.fresh
        m.protective_clear = self._view.protective_clear
        m.warning_active = self._view.warning_active
        m.warning_level = int(self._view.warning_level)
        m.speed_scale = float(self._scale)
        self._pub_state.publish(m)

    def _event(self, level: int, code: str, text: str) -> None:
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source, m.level, m.code, m.text = "cmd_mux_kinematics", level, code, text
        self._event_seq += 1
        m.seq = self._event_seq
        self._pub_event.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdMuxKinematics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A callback running while launch tears the context down raises from
        # the C layer ("Unable to convert call argument"); only real if still ok.
        if rclpy.ok():
            raise
    finally:
        # launch sends SIGINT; the context may already be down by the time we
        # get here, and destroy_node() then raises from the C layer.
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
