"""/scan -> /scan_gated: the same scans, released only once odom->laser exists for
their stamp, and no faster than `min_period_s` (see scan_gate.py for why).

slam_toolbox, AMCL, the local costmap, the localization monitor and the route
executor read /scan_gated; only the web live view stays on raw /scan.

mask_file:=<yaml> (the AMR QR: ~/.amr/scan_mask.yaml, learnt by scan_mask_learn) drops
the phantom returns of a scratched optics cover (scan_mask.py); robot_fixed:=true drops
returns that turn WITH the vehicle - its body, a bracket, a scratch - learnt while it runs
(robot_fixed.py); despeckle:=true then drops lone returns (speckle.py). None of them reaches
the map, localisation or obstacle detection. Expired
scans are counted and logged, never published: a scan whose transform never
came is not the consumer's problem.
"""

import math
import os

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from amr_localization.robot_fixed import RobotFixedFilter
from amr_localization.scan_gate import ScanGate
from amr_localization.scan_mask import apply_mask, load
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
        self.declare_parameter("mask_file", "")
        self.mask = []
        path = p("mask_file").value
        if path and os.path.isfile(path):
            self.mask = load(path)  # a malformed file is an error, not a silent no-mask
            self.get_logger().info(
                f"scan mask {path}: "
                + ", ".join(
                    f"{s.from_deg:+.1f}..{s.to_deg:+.1f} deg < {s.max_range_m:.2f} m" for s in self.mask
                )
            )
        elif path:
            self.get_logger().info(f"no scan mask ({path} absent)")
        self._mask_removed = 0
        self.declare_parameter("robot_fixed", False)
        self.declare_parameter("robot_fixed_turn_deg", 90.0)
        self.robot_fixed = (
            RobotFixedFilter(need_rot_rad=math.radians(float(p("robot_fixed_turn_deg").value)))
            if p("robot_fixed").value
            else None
        )
        self._rf_removed = 0
        self.declare_parameter("robot_fixed_file", "")
        self._rf_file = str(p("robot_fixed_file").value)
        self._rf_loaded = False
        self._rf_reported: dict = {}
        self._last_geom: tuple[float, float] | None = None
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
            if self.mask:
                ranges, removed = apply_mask(m.ranges, m.angle_min, m.angle_increment, self.mask)
                m.ranges = ranges
                self._mask_removed += removed
            self._last_geom = (m.angle_min, m.angle_increment)
            if self.robot_fixed is not None and not self._rf_loaded:
                self._rf_loaded = True
                self._rf_load(m)
            if self.robot_fixed is not None:
                yaw = self._scanner_yaw(m)
                if yaw is not None:
                    ranges, removed = self.robot_fixed.update(
                        m.ranges, m.range_min, m.range_max, yaw, self._now()
                    )
                    m.ranges = ranges
                    self._rf_removed += removed
            if self.speckle is not None:
                ranges, removed = despeckle(m.ranges, m.range_min, m.range_max, **self.speckle)
                m.ranges = ranges
                self._speckle_removed += removed
                self._speckle_scans += 1
            self._pub.publish(m)

    def _rf_load(self, m) -> None:
        if not self._rf_file or not os.path.isfile(self._rf_file):
            return
        try:
            import yaml  # noqa: PLC0415

            with open(self._rf_file) as fh:
                d = yaml.safe_load(fh)
            n = self.robot_fixed.load_dict(d, len(m.ranges), m.angle_min, m.angle_increment, self._now())
        except Exception as e:  # noqa: BLE001 - a bad file is ignored, learning starts over
            self.get_logger().warn(
                f"robot-fixed: could not read {self._rf_file} ({e}); learning from scratch"
            )
            return
        self._rf_reported = dict(self.robot_fixed.flagged)
        self.get_logger().info(
            f"robot-fixed: {n} learnt beams loaded from {self._rf_file}"
            if n
            else f"robot-fixed: {self._rf_file} is for another scan geometry; learning from scratch"
        )

    def _rf_save(self) -> None:
        if not self._rf_file or self._last_geom is None:
            return
        try:
            import yaml  # noqa: PLC0415

            os.makedirs(os.path.dirname(self._rf_file) or ".", exist_ok=True)
            tmp = self._rf_file + ".tmp"
            with open(tmp, "w") as fh:
                yaml.safe_dump(self.robot_fixed.to_dict(*self._last_geom), fh, sort_keys=False)
            os.replace(tmp, self._rf_file)
        except OSError as e:
            self.get_logger().warn(f"robot-fixed: could not save {self._rf_file} ({e})")

    def _scanner_yaw(self, m) -> float | None:
        """The scanner's heading in odom at the scan's stamp (released scans are transformable)."""
        try:
            q = self.tf_buffer.lookup_transform(
                self.odom_frame, m.header.frame_id, Time.from_msg(m.header.stamp)
            ).transform.rotation
        except Exception:  # noqa: BLE001 - tf2 raises several types; no yaw = no update this scan
            return None
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _report(self) -> None:
        rf = self.robot_fixed
        if rf is not None:
            if rf.flagged != self._rf_reported:
                self._rf_reported = dict(rf.flagged)
                self._rf_save()
                g = self._last_geom
                self.get_logger().info(
                    f"robot-fixed returns: {len(rf.flagged)} beams learnt"
                    + (
                        ": " + ", ".join(f"{a:+.1f}..{b:+.1f} deg @ {r:.2f} m" for a, b, r in rf.sectors(*g))
                        if g is not None and rf.flagged
                        else ""
                    )
                )
            if self._rf_removed:
                self.get_logger().info(f"robot-fixed: {self._rf_removed} returns removed in the last 30 s")
                self._rf_removed = 0
        if self._mask_removed:
            self.get_logger().info(
                f"scan mask: {self._mask_removed} phantom returns removed in the last 30 s"
            )
            self._mask_removed = 0
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
