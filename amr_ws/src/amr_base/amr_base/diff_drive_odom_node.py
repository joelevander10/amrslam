"""Wheel odometry from /wheel_states position increments (spec §3.6).

Publishes /odom_raw (nav_msgs/Odometry). Does NOT publish TF by default: the
EKF owns the odom transform. publish_tf:=true only for standalone tests.

TF tree (architecture §5 as resolved 2026-09-15): the URDF fixes
base_footprint->base_link, so the moving transform is odom->base_footprint and
child_frame_id here is base_footprint. A child frame has exactly one parent;
publishing odom->base_link as well would give base_link two. The EKF's
base_link_frame must be base_footprint for the same reason (T3).

Pose covariance grows with distance travelled; twist covariance is static and
parameterised. The EKF consumes twist + pose increments, so the absolute pose
covariance is informational.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster

from amr_base.agv_repo import config
from amr_base.diff_drive import Geometry, OdomState, forward, integrate
from amr_interfaces.msg import WheelStates

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
BIG = 1e6


class DiffDriveOdom(Node):
    def __init__(self) -> None:
        super().__init__("diff_drive_odom")
        self.declare_parameter("wheel_radius_m", config.WHEEL_DIA_M / 2.0)
        self.declare_parameter("track_width_m", config.TRACK_M)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("twist_cov_vx", 1e-3)
        self.declare_parameter("twist_cov_vyaw", 1e-3)
        self.declare_parameter("pose_cov_base", 1e-3)
        self.declare_parameter("pose_cov_per_m", 1e-3)

        p = self.get_parameter
        self.geom = Geometry(p("wheel_radius_m").value, p("track_width_m").value)
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.publish_tf = p("publish_tf").value
        self.cov_vx = p("twist_cov_vx").value
        self.cov_vyaw = p("twist_cov_vyaw").value
        self.pose_cov_base = p("pose_cov_base").value
        self.pose_cov_per_m = p("pose_cov_per_m").value

        self.state = OdomState()
        self._last: WheelStates | None = None

        self._pub = self.create_publisher(Odometry, "/odom_raw", SENSOR_DATA)
        self._tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, SENSOR_DATA)
        self.get_logger().info(
            f"r={self.geom.wheel_radius_m} track={self.geom.track_width_m} publish_tf={self.publish_tf}"
        )

    @staticmethod
    def _t(msg: WheelStates) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_wheels(self, msg: WheelStates) -> None:
        finite = math.isfinite(msg.left_pos_rad) and math.isfinite(msg.right_pos_rad)
        if not (msg.left_valid and msg.right_valid and finite):
            self._last = None  # restart the increment chain after a gap
            return
        prev, self._last = self._last, msg
        if prev is None:
            return
        # R06: positions are continuous only under one encoder scale, and a
        # publisher restart can step the clock back. Either is a new baseline,
        # never an increment.
        if msg.counts_per_wheel_rev != prev.counts_per_wheel_rev:
            return
        dt = self._t(msg) - self._t(prev)
        if dt <= 0.0:
            return
        d_l = msg.left_pos_rad - prev.left_pos_rad
        d_r = msg.right_pos_rad - prev.right_pos_rad
        self.state = integrate(self.state, self.geom, d_l, d_r)
        v, wz = forward(self.geom, d_l / dt, d_r / dt)
        self._publish(msg, v, wz)

    def _publish(self, src: WheelStates, v: float, wz: float) -> None:
        s = self.state
        qz, qw = math.sin(s.th / 2.0), math.cos(s.th / 2.0)
        pc = self.pose_cov_base + self.pose_cov_per_m * s.distance

        o = Odometry()
        o.header.stamp = src.header.stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = s.x
        o.pose.pose.position.y = s.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.pose.covariance = _diag(pc, pc, BIG, BIG, BIG, pc)
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = wz
        o.twist.covariance = _diag(self.cov_vx, BIG, BIG, BIG, BIG, self.cov_vyaw)
        self._pub.publish(o)

        if self._tf is not None:
            t = TransformStamped()
            t.header = o.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = s.x
            t.transform.translation.y = s.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf.sendTransform(t)


def _diag(*vals: float) -> list[float]:
    cov = [0.0] * 36
    for i, v in enumerate(vals):
        cov[i * 7] = v
    return cov


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiffDriveOdom()
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
