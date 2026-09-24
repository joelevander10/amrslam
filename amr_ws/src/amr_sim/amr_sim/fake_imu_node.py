"""fake_imu_node (spec §8): yaw rate from ground truth + measured MLS gyro noise.

Publishes /imu/data_raw as the real MLS IMU node (T10) will: RAW, biased.
Noise model is reconciliation D-3, measured on the vehicle 2026-09-03:
bias +0.0574 deg/s, sigma 0.0291 deg/s, plus a small bias random walk so the
bias node has something to track. Orientation is marked unavailable
(covariance[0] = -1): the EKF fuses yaw rate only.
"""

import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
DEG = math.pi / 180.0
G = 9.80665


class FakeImu(Node):
    def __init__(self) -> None:
        super().__init__("fake_imu")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("frame_id", "imu_frame")
        self.declare_parameter("bias_dps", 0.0574)  # D-3
        self.declare_parameter("noise_dps", 0.0291)  # D-3, 1 sigma
        self.declare_parameter("bias_walk_dps_per_sqrt_s", 0.002)
        self.declare_parameter("gravity_scale", 0.9894)  # D-3, 1.1 % scale error
        self.declare_parameter("seed", 0)
        p = self.get_parameter
        self.dt = 1.0 / p("rate_hz").value
        self.frame_id = p("frame_id").value
        self.bias = p("bias_dps").value * DEG
        self.noise = p("noise_dps").value * DEG
        self.walk = p("bias_walk_dps_per_sqrt_s").value * DEG
        self.gravity = p("gravity_scale").value * G
        self._rng = random.Random(p("seed").value)
        self._wz_true = 0.0

        self._pub = self.create_publisher(Imu, "/imu/data_raw", SENSOR_DATA)
        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, SENSOR_DATA)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"bias {p('bias_dps').value:+.4f} deg/s, sigma {p('noise_dps').value:.4f} deg/s, "
            f"{1 / self.dt:.0f} Hz"
        )

    def _on_truth(self, msg: Odometry) -> None:
        self._wz_true = msg.twist.twist.angular.z

    def _tick(self) -> None:
        self.bias += self._rng.gauss(0.0, self.walk * math.sqrt(self.dt))
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.z = self._wz_true + self.bias + self._rng.gauss(0.0, self.noise)
        m.angular_velocity_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, self.noise**2]
        m.linear_acceleration.z = self.gravity
        m.linear_acceleration_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e6]
        self._pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeImu()
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
