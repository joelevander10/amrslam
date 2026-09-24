"""Scripted open-loop square on /cmd_vel_teleop (spec §8 test 1 driver).

Sides side_a, side_b, side_a, side_b with a CCW 90° turn between them, then
stops publishing so the mux/watchdog path is exercised. Logs "square done"
when finished; launch tests wait for that line.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

RELIABLE_1 = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class SquareDrive(Node):
    def __init__(self) -> None:
        super().__init__("square_drive")
        self.declare_parameter("side_a_m", 4.0)
        self.declare_parameter("side_b_m", 5.0)
        self.declare_parameter("speed_mps", 1.0)
        self.declare_parameter("turn_rate_rad_s", 1.0)
        self.declare_parameter("settle_s", 1.0)
        self.declare_parameter("start_delay_s", 0.0)  # stillness for gyro-bias calibration
        # Deliberate return error (spec §8 "modest return error"): the last
        # straight is shortened and the last turn cut short, so the loop does
        # not close exactly and a survey review has something real to report.
        self.declare_parameter("closing_error_m", 0.0)
        self.declare_parameter("closing_error_deg", 0.0)
        self.declare_parameter("rate_hz", 20.0)
        p = self.get_parameter
        v, w, settle = (
            p("speed_mps").value,
            p("turn_rate_rad_s").value,
            p("settle_s").value,
        )

        # (duration_s, linear, angular)
        self.plan: list[tuple[float, float, float]] = [(settle + p("start_delay_s").value, 0.0, 0.0)]
        for side in (p("side_a_m").value, p("side_b_m").value) * 2:
            self.plan += [
                (side / v, v, 0.0),
                (settle, 0.0, 0.0),
                ((math.pi / 2) / w, 0.0, w),
                (settle, 0.0, 0.0),
            ]
        self._i = 0
        self._seg_start: float | None = None
        self._pub = self.create_publisher(Twist, "/cmd_vel_teleop", RELIABLE_1)
        self._timer = self.create_timer(1.0 / p("rate_hz").value, self._tick)
        self.get_logger().info(
            f"square {p('side_a_m').value} x {p('side_b_m').value} m at {v} m/s, {w} rad/s"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        t = self._now()
        if self._seg_start is None:
            self._seg_start = t
        if t - self._seg_start >= self.plan[self._i][0]:
            self._i += 1
            self._seg_start = t
            if self._i >= len(self.plan):
                self._timer.cancel()
                self.get_logger().info("square done")
                return
        _, v, w = self.plan[self._i]
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SquareDrive()
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
