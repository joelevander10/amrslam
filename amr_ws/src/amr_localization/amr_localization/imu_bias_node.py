"""/imu/data_raw -> /imu/data with the gyro yaw-rate bias removed (spec §4.3).

Replaces imu_filter_madgwick in the chain (reconciliation D-3: the MLS already
fuses orientation, and the EKF takes yaw rate only). Stationary detection
comes from /wheel_states; nothing is published until the first calibration
window completes. /imu/recalibrate (std_srvs/Trigger) discards the estimate.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from amr_interfaces.msg import WheelStates
from amr_localization.gyro_bias import BiasEstimator

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ImuBias(Node):
    def __init__(self) -> None:
        super().__init__("imu_bias")
        self.declare_parameter("window_s", 2.0)
        self.declare_parameter("settle_s", 0.3)
        self.declare_parameter("stationary_wheel_rad_s", 0.01)
        self.declare_parameter("wheel_timeout_s", 0.2)
        p = self.get_parameter
        self.est = BiasEstimator(window_s=p("window_s").value, settle_s=p("settle_s").value)
        self.w_eps = p("stationary_wheel_rad_s").value
        self.wheel_timeout = p("wheel_timeout_s").value
        self._wheels_t: float | None = None
        self._wheels_still = False
        self._was_calibrated = False
        self._windows = 0

        self._pub = self.create_publisher(Imu, "/imu/data", SENSOR_DATA)
        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, SENSOR_DATA)
        self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, SENSOR_DATA)
        self.create_service(Trigger, "/imu/recalibrate", self._on_recalibrate)
        self.get_logger().info(
            f"waiting for {self.est.settle_s + self.est.window_s:.1f} s stationary to calibrate"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_wheels(self, msg: WheelStates) -> None:
        self._wheels_t = self._now()
        self._wheels_still = (
            msg.left_valid
            and msg.right_valid
            and abs(msg.left_vel_rad_s) < self.w_eps
            and abs(msg.right_vel_rad_s) < self.w_eps
        )

    def _stationary(self, t: float) -> bool:
        # No wheel feedback means we cannot claim stillness.
        return self._wheels_t is not None and t - self._wheels_t < self.wheel_timeout and self._wheels_still

    def _on_imu(self, msg: Imu) -> None:
        t = self._now()
        corrected = self.est.update(t, msg.angular_velocity.z, self._stationary(t))
        if self.est.windows != self._windows:
            self._windows = self.est.windows
            self.get_logger().info(
                f"gyro z bias {self.est.bias * 57.2958:+.4f} deg/s (window {self._windows})"
            )
        if corrected is None:
            return
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity.z = corrected
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)

    def _on_recalibrate(self, _req, res):
        self.est.reset()
        self.get_logger().info("recalibration requested: /imu/data paused until still")
        res.success = True
        res.message = (
            f"bias discarded; keep the vehicle still for {self.est.settle_s + self.est.window_s:.1f} s"
        )
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuBias()
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
