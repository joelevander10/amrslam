"""fake_base_node (spec §8): replaces drive node + odometry source in sim.

Subscribes /cmd_wheel_vel, publishes /wheel_states (100 Hz, SensorData),
/drives/status (10 Hz, always operational - the simulated drive owner) and
/sim/ground_truth (nav_msgs/Odometry, frame map->base_footprint). Zeroes the wheels
if no command arrives within cmd_timeout_s, mirroring the drive-node watchdog
(spec test 6), and - like drive_node - applies the supervisor lease gate when
`require_supervisor` is set (unified plan §4.3 item 7). Wheel ramp defaults to
the profile's 6083h auto ramp (D-1).
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from amr_base import gating
from amr_base.agv_repo import config
from amr_base.diff_drive import Geometry
from amr_interfaces.msg import ControlLease, DriveStatus, WheelStates, WheelVelocities
from amr_interfaces.srv import SetPose2D
from amr_sim.wheel_model import WheelModel

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
RELIABLE_1 = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)
WHEEL_RAD_S_PER_MOTOR_RPM = 2.0 * math.pi / 60.0 / config.GEAR_RATIO


class FakeBase(Node):
    def __init__(self) -> None:
        super().__init__("fake_base")
        self.declare_parameter("wheel_radius_m", config.WHEEL_DIA_M / 2.0)
        self.declare_parameter("track_width_m", config.TRACK_M)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("cmd_timeout_s", 0.25)
        self.declare_parameter("wheel_accel_rad_s2", config.ACCEL_RPM_S * WHEEL_RAD_S_PER_MOTOR_RPM)
        self.declare_parameter("slip_noise_std", 0.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("truth_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("require_supervisor", False)
        self.declare_parameter("lease_timeout_s", 0.3)

        p = self.get_parameter
        self.model = WheelModel(
            geom=Geometry(p("wheel_radius_m").value, p("track_width_m").value),
            wheel_accel_rad_s2=p("wheel_accel_rad_s2").value,
            cmd_timeout_s=p("cmd_timeout_s").value,
            slip_noise_std=p("slip_noise_std").value,
            seed=p("seed").value,
        )
        self.truth_frame = p("truth_frame").value
        self.base_frame = p("base_frame").value
        self.dt = 1.0 / p("rate_hz").value
        self._last_t: float | None = None
        self._was_timed_out = True

        self._gate = gating.Params(
            require_supervisor=bool(self.get_parameter("require_supervisor").value),
            lease_timeout_s=float(self.get_parameter("lease_timeout_s").value),
        )
        self._lease: gating.Lease | None = None
        self._gate_reason: str | None = None
        self._pub_wheels = self.create_publisher(WheelStates, "/wheel_states", SENSOR_DATA)
        self._pub_truth = self.create_publisher(Odometry, "/sim/ground_truth", SENSOR_DATA)
        self._pub_status = self.create_publisher(DriveStatus, "/drives/status", RELIABLE_1)
        self.create_subscription(WheelVelocities, "/cmd_wheel_vel", self._on_cmd, RELIABLE_1)
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, RELIABLE_1)
        self.create_timer(0.1, self._publish_status)
        self.create_service(SetPose2D, "/sim/set_pose", self._on_set_pose)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"r={self.model.geom.wheel_radius_m} track={self.model.geom.track_width_m} "
            f"accel={self.model.wheel_accel_rad_s2:.2f} rad/s² slip={self.model.slip_noise_std} "
            f"watchdog={self.model.cmd_timeout_s}s"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_set_pose(self, req: SetPose2D.Request, res: SetPose2D.Response):
        self.model.teleport(req.x_m, req.y_m, req.yaw_rad)
        res.ok = True
        res.message = (
            f"ground truth moved to ({req.x_m:.2f}, {req.y_m:.2f}, {math.degrees(req.yaw_rad):.1f} deg)"
        )
        self.get_logger().warn("teleport: " + res.message)
        return res

    def _on_lease(self, msg: ControlLease) -> None:
        cur = self._lease
        if cur is not None and (cur.instance, cur.generation) == (msg.instance, int(msg.generation)):
            if int(msg.seq) <= cur.seq:
                return
        self._lease = gating.Lease(
            time.monotonic(), msg.instance, int(msg.generation), int(msg.seq), int(msg.allowed)
        )

    def _on_cmd(self, msg: WheelVelocities) -> None:
        reason = gating.drive_gate(time.monotonic(), self._lease, int(msg.generation), self._gate)
        if reason != self._gate_reason:
            self._gate_reason = reason
            if reason is not None:
                self.get_logger().warn(f"setpoint gated to zero: {reason}")
        if reason is not None:
            self.model.command(0.0, 0.0, self._now())
            return
        self.model.command(msg.left_rad_s, msg.right_rad_s, self._now())

    def _publish_status(self) -> None:
        m = DriveStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.left_state = m.right_state = "Operation enabled (simulated)"
        m.operational = True
        self._pub_status.publish(m)

    def _tick(self) -> None:
        t = self._now()
        dt = self.dt if self._last_t is None else min(t - self._last_t, 5 * self.dt)
        self._last_t = t
        self.model.step(t, dt)
        if self.model.timed_out != self._was_timed_out:
            self._was_timed_out = self.model.timed_out
            if self.model.timed_out:
                self.get_logger().info("command watchdog: wheels zeroed")
        stamp = self.get_clock().now().to_msg()

        w = WheelStates()
        w.header.stamp = stamp
        w.left_pos_rad = self.model.pos_l
        w.right_pos_rad = self.model.pos_r
        w.left_vel_rad_s = self.model.actual_l
        w.right_vel_rad_s = self.model.actual_r
        w.left_valid = w.right_valid = True
        # emulated raw 6064h counters (vehicle-terms radians -> driver-terms counts)
        cpr = 1080000.0
        sl = -1.0 if config.INVERT_LEFT else 1.0
        sr = -1.0 if config.INVERT_RIGHT else 1.0
        left_counts = int(round(sl * self.model.pos_l / (2 * math.pi) * cpr)) & 0xFFFFFFFF
        right_counts = int(round(sr * self.model.pos_r / (2 * math.pi) * cpr)) & 0xFFFFFFFF
        if left_counts >= 1 << 31:
            left_counts -= 1 << 32
        if right_counts >= 1 << 31:
            right_counts -= 1 << 32
        w.left_counts = left_counts
        w.right_counts = right_counts
        w.counts_valid = True
        w.counts_per_wheel_rev = cpr
        self._pub_wheels.publish(w)

        s = self.model.truth
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.truth_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = s.x
        o.pose.pose.position.y = s.y
        o.pose.pose.orientation.z = math.sin(s.th / 2.0)
        o.pose.pose.orientation.w = math.cos(s.th / 2.0)
        r = self.model.geom.wheel_radius_m
        o.twist.twist.linear.x = r * (self.model.actual_r + self.model.actual_l) / 2.0
        o.twist.twist.angular.z = (
            r * (self.model.actual_r - self.model.actual_l) / self.model.geom.track_width_m
        )
        self._pub_truth.publish(o)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeBase()
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
