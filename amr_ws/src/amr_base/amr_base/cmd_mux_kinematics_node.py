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
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from amr_base import gating
from amr_base.agv_repo import config, kinematics
from amr_base.diff_drive import Geometry, clamp_wheels, inverse, scurve, slew, slew_asym
from amr_interfaces.msg import (
    ControlLease,
    DriveStatus,
    ManualCommand,
    ModeState,
    MotionPermit,
    MuxState,
    PanelState,
    WheelVelocities,
)

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
    def __init__(self) -> None:
        super().__init__("cmd_mux_kinematics")
        hw_a_max = config.ACCEL_RPM_S * config.MPS_PER_RPM
        hw_alpha_max = kinematics.max_yaw_accel(config.ACCEL_RPM_S)

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
        self.declare_parameter("teleop_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.2)
        self.declare_parameter("permit_timeout_s", 0.3)
        self.declare_parameter("panel_timeout_s", 0.2)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("require_supervisor", False)  # production: True (unified plan §4.2)
        self.declare_parameter("teleop_enabled", True)  # /cmd_vel_teleop, engineering only
        self.declare_parameter("pendant_v_m_s", 0.50)
        self.declare_parameter("pendant_w_rad_s", 0.39)  # spin in place (0.30 until 2026-09-19, +30 %)
        self.declare_parameter("pendant_turn_ratio", 0.75)  # slow wheel / fast wheel while driving + turning
        # manual spin cap while SURVEYING (supervisor mode MAPPING), pendant and browser jog alike:
        # 0.39 x 0.7 (2026-09-19) - a fast spin smears each scan and a survey came out rotated
        self.declare_parameter("survey_w_max_rad_s", 0.27)
        self.declare_parameter("lease_timeout_s", 0.3)
        self.declare_parameter("drives_timeout_s", 0.3)

        p = self.get_parameter
        self.geom = Geometry(p("wheel_radius_m").value, p("track_width_m").value)
        self.w_max = p("wheel_vel_max_rad_s").value
        self.a_max = min(p("a_max").value, hw_a_max)
        self.alpha_max = min(p("alpha_max").value, hw_alpha_max)
        self.d_max = min(p("d_max").value or self.a_max, hw_a_max)
        self.delta_max = min(p("delta_max").value or self.alpha_max, hw_alpha_max)
        self.manual_a_max = min(p("manual_a_max").value, hw_a_max)
        self.manual_alpha_max = min(p("manual_alpha_max").value or self.alpha_max, hw_alpha_max)
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
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, RELIABLE_1)
        self.create_subscription(ManualCommand, "/amr/manual_command", self._on_manual, RELIABLE_1)
        self.create_subscription(DriveStatus, "/drives/status", self._on_drives, RELIABLE_1)
        self.create_subscription(ModeState, "/amr/mode_state", self._on_mode, LATCHED)
        self.create_subscription(
            WheelVelocities, "/amr/commissioning_wheels", self._on_commissioning, RELIABLE_1
        )
        self._pub = self.create_publisher(WheelVelocities, "/cmd_wheel_vel", RELIABLE_1)
        self._pub_state = self.create_publisher(MuxState, "/amr/mux_state", RELIABLE_1)
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
        ]

    def _on_mode(self, msg: ModeState) -> None:
        surveying = msg.mode == ModeState.MAPPING
        if surveying != self._surveying:
            self.get_logger().info(
                f"survey spin cap {self.survey_w_max:.2f} rad/s: {'on' if surveying else 'off'}"
            )
        self._surveying = surveying

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
        )
        name = gating.NAMES[sel.source]
        if name != self._source or (sel.source == gating.NONE and sel.reason != self._reason):
            self.get_logger().info(f"command source: {self._source} -> {name} ({sel.reason})")
            self._source, self._reason = name, sel.reason

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
            self._v = slew_asym(self._v, sel.v, self.a_max, self.d_max, self.dt)
            self._wz = slew_asym(self._wz, sel.w, self.alpha_max, self.delta_max, self.dt)
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
        self._pub_state.publish(m)


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
