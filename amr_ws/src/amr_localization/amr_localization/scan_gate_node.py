"""/scan -> /scan_gated: the same scans, released only once odom->laser exists for
their stamp, and no faster than `min_period_s` (see scan_gate.py for why).

slam_toolbox, AMCL, the local costmap, the localization monitor and the route
executor read /scan_gated; only the web live view stays on raw /scan.

despeckle:=true (the AMR QR) also drops lone returns (speckle.py) from the released
scans, so they reach neither the map nor localisation. Expired
scans are counted and logged, never published: a scan whose transform never
came is not the consumer's problem.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from amr_localization.scan_gate import ScanGate
from amr_localization.speckle import despeckle

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ScanGateNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_gate")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("min_period_s", 0.1)
        self.declare_parameter("settle_s", 0.02)
        self.declare_parameter("hold_max_s", 0.5)
        self.declare_parameter("poll_period_s", 0.01)
        self.declare_parameter("despeckle", False)
        self.declare_parameter("despeckle_window", 2)
        self.declare_parameter("despeckle_min_neighbours", 1)
        self.declare_parameter("despeckle_abs_m", 0.05)
        self.declare_parameter("despeckle_rel", 0.03)
        p = self.get_parameter
        self.speckle = (
            dict(
                window=int(p("despeckle_window").value),
                min_neighbours=int(p("despeckle_min_neighbours").value),
                abs_m=float(p("despeckle_abs_m").value),
                rel=float(p("despeckle_rel").value),
            )
            if p("despeckle").value
            else None
        )
        self._speckle_removed = self._speckle_scans = 0
        self.odom_frame = p("odom_frame").value
        self.gate = ScanGate(
            min_period_s=p("min_period_s").value,
            settle_s=p("settle_s").value,
            hold_max_s=p("hold_max_s").value,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._pub = self.create_publisher(LaserScan, "/scan_gated", SENSOR_DATA)
        self.create_subscription(LaserScan, "/scan", self._on_scan, SENSOR_DATA)
        self.create_timer(p("poll_period_s").value, self._poll)
        self.create_timer(30.0, self._report)
        self._reported_expired = 0

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, m: LaserScan) -> None:
        self.gate.push(Time.from_msg(m.header.stamp).nanoseconds * 1e-9, m, self._now())

    def _poll(self) -> None:
        if not self.gate.pending:
            return

        def transformable(t: float) -> bool:
            # the frame of the scan being judged (the oldest), re-read per call: frames may change
            frame = self.gate.frame_of_oldest()
            return frame is not None and self.tf_buffer.can_transform(self.odom_frame, frame, Time(seconds=t))

        for m in self.gate.poll(self._now(), transformable):
            if self.speckle is not None:
                ranges, removed = despeckle(m.ranges, m.range_min, m.range_max, **self.speckle)
                m.ranges = ranges
                self._speckle_removed += removed
                self._speckle_scans += 1
            self._pub.publish(m)

    def _report(self) -> None:
        if self._speckle_scans:
            self.get_logger().info(
                f"despeckle: {self._speckle_removed} lone returns removed from "
                f"{self._speckle_scans} scans in the last 30 s"
            )
            self._speckle_removed = self._speckle_scans = 0
        s = self.gate.stats
        new_expired = s.expired - self._reported_expired
        self._reported_expired = s.expired
        if new_expired:
            self.get_logger().warn(
                f"{new_expired} scans expired without an {self.odom_frame} transform in the last 30 s "
                f"(released {s.released}, thinned {s.thinned}, received {s.received})"
            )


def main() -> None:
    rclpy.init()
    node = ScanGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Ctrl-C landing inside take_message surfaces as a pybind "Unable to convert call
        # argument" error (seen on Jazzy, 2026-09-24); only a real error while running counts
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
