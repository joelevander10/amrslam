"""fake_gls_node: the GLS621 floor-label camera in sim (gls-wait-coding-plan §B.6).

    /sim/ground_truth (fake_base, map -> base_footprint)
        -> /gls/reading (GlsReading), shaped like gls_node's: one reading per "image" at
           IMAGE_HZ while a configured label is inside the capture window, valid=false at
           5 Hz otherwise

Fakes at the ROS level, not the TCP level: the telegram parser and the link have their own
unit tests, and what the sim needs is the CONTRACT - a calibrated T_LC whenever the camera
is over a label - so the label-fix and monitor paths can run end to end.

The camera point is the profile's mount (gls.mount_*, T_BC). Labels come from the `labels`
parameter, "id,x,y,yaw_rad;id,x,y,yaw_rad" in the map frame (base.launch.py sim_gls_labels).
The window is the whole-label one at the real mount height (~146 x 117 mm field of view):
the label centre within WINDOW_ALONG_M fore-aft and WINDOW_SIDE_M sideways of the camera.
"""

from __future__ import annotations

import math

import rclpy
from agv_core import config
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from amr_interfaces.msg import GlsReading

SENSOR_DATA = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)
IMAGE_HZ = 14.6  # measured on the vehicle, 2026-10-30
INVALID_HZ = 5.0
WINDOW_ALONG_M, WINDOW_SIDE_M = 0.020, 0.035
TRUTH_STALE_S = 0.5


def compose(a, b):
    """SE(2) a (+) b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    yaw = a[2] + b[2]
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(yaw), math.cos(yaw))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


def parse_labels(text: str) -> list[tuple[str, tuple[float, float, float]]]:
    """'id,x,y,yaw;...' -> [(id, (x, y, yaw))]. Empty text = no labels. Malformed raises."""
    out = []
    for item in filter(None, (t.strip() for t in text.split(";"))):
        parts = [p.strip() for p in item.split(",")]
        if len(parts) != 4:
            raise ValueError(f"label {item!r}: expected id,x,y,yaw_rad")
        out.append((parts[0], (float(parts[1]), float(parts[2]), float(parts[3]))))
    return out


def seen(labels, base_in_map, mount):
    """(label_id, T_LC) of the label whose centre is inside the camera's window, or None."""
    cam = compose(base_in_map, mount)
    for label_id, pose in labels:
        lc = compose(inverse(pose), cam)  # the camera in the label frame
        cl = inverse(lc)  # the label centre in the camera frame
        if abs(cl[0]) <= WINDOW_ALONG_M and abs(cl[1]) <= WINDOW_SIDE_M:
            return label_id, lc
    return None


class FakeGls(Node):
    def __init__(self) -> None:
        super().__init__("fake_gls")
        self.declare_parameter("labels", "")
        self.labels = parse_labels(str(self.get_parameter("labels").value))
        self.mount = (config.GLS_MOUNT_X_M, config.GLS_MOUNT_Y_M, config.GLS_MOUNT_YAW_RAD)
        self._truth = None
        self._truth_t = None
        self._ts_ms = 0
        self._last_invalid = 0.0
        self._pub = self.create_publisher(GlsReading, "/gls/reading", 10)
        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, SENSOR_DATA)
        self.create_timer(1.0 / IMAGE_HZ, self._tick)
        where = ", ".join(f"{i} at ({x:.2f}, {y:.2f}, {math.degrees(a):.0f} deg)"
                          for i, (x, y, a) in self.labels)
        self.get_logger().info(f"fake GLS: {len(self.labels)} label(s) {where}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_truth(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._truth, self._truth_t = (m.pose.pose.position.x, m.pose.pose.position.y, yaw), self._now()

    def _tick(self) -> None:
        now = self._now()
        self._ts_ms += round(1000.0 / IMAGE_HZ)
        m = GlsReading()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "gls_frame"
        m.comms_ok = m.config_ok = m.calibrated = True
        m.fps = IMAGE_HZ
        m.detail = "sim"
        fresh = self._truth is not None and now - self._truth_t <= TRUTH_STALE_S
        hit = seen(self.labels, self._truth, self.mount) if fresh else None
        if hit is None:
            if now - self._last_invalid < 1.0 / INVALID_HZ:
                return
            self._last_invalid = now
            m.valid = False
            self._pub.publish(m)
            return
        m.valid, m.label_id = True, hit[0]
        m.x_m, m.y_m, m.yaw_rad = hit[1]
        m.n_codes, m.device_ts_ms = 5, self._ts_ms
        m.angle_deg = math.degrees(hit[1][2]) % 360.0
        self._pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeGls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
