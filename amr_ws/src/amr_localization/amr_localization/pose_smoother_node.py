"""pose_smoother_node: broadcast a smoothed map->odom in place of AMCL's (2026-11-12).

See pose_smoother.py for why and how. Launched by navigation_layer with smooth_pose:=true,
which also turns AMCL's own tf_broadcast off: map->odom has exactly one publisher.

  in   /amcl_pose      AMCL's estimate; with odom->base_footprint at its stamp -> raw map->odom
       /initialpose    take AMCL's next pose at once (relocalisation, operator)
       /scan_gated     AMCL's input: no scans, no map->odom (as when AMCL broadcast it)
       TF odom->base_footprint
  out  TF map->odom                          smoothed, rate_hz, stamped future_s ahead
       /amr/localization/map_odom_raw        AMCL's map->odom (TransformStamped, its stamp):
                                             the localization monitor judges AMCL by this

map->odom stops (and goes stale for every reader, as before) when AMCL has given no pose
yet, when no scan arrived for stale_scan_s, or when the AMR drove stale_travel_m since
AMCL's last pose (AMCL updates every 5 cm of travel: it has stopped).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from amr_localization import pose_smoother as ps

SENSOR_DATA = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)
RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)
RAW_TOPIC = "/amr/localization/map_odom_raw"


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_tf(pose: ps.Pose, stamp, parent: str, child: str) -> TransformStamped:
    m = TransformStamped()
    m.header.stamp = stamp
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y = float(pose[0]), float(pose[1])
    m.transform.rotation.z, m.transform.rotation.w = math.sin(pose[2] / 2.0), math.cos(pose[2] / 2.0)
    return m


class PoseSmootherNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_smoother")
        d = self.declare_parameter
        d("global_frame", "map")
        d("odom_frame", "odom")
        d("base_frame", "base_footprint")
        d("rate_hz", 50.0)
        d("future_s", 0.1)
        d("tau_s", 1.0)
        d("max_rate_m_s", 0.03)
        d("per_m", 0.05)
        d("per_rad_m", 0.02)
        d("max_rate_deg_s", 0.5)
        d("per_m_deg", 1.0)
        d("per_rad", 0.05)
        d("snap_m", 0.50)
        d("snap_deg", 10.0)
        d("still_v_m_s", 0.01)
        d("still_w_deg_s", 0.5)
        d("still_s", 0.15)
        d("stale_scan_s", 2.0)
        d("stale_travel_m", 1.0)
        v = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_f, self.odom_f, self.base_f = v("global_frame"), v("odom_frame"), v("base_frame")
        self.future = Duration(seconds=float(v("future_s")))
        self.smoother = ps.Smoother(
            ps.Params(
                tau_s=float(v("tau_s")),
                max_rate_m_s=float(v("max_rate_m_s")),
                per_m=float(v("per_m")),
                per_rad_m=float(v("per_rad_m")),
                max_rate_rad_s=math.radians(float(v("max_rate_deg_s"))),
                per_m_rad=math.radians(float(v("per_m_deg"))),
                per_rad=float(v("per_rad")),
                snap_m=float(v("snap_m")),
                snap_rad=math.radians(float(v("snap_deg"))),
            )
        )
        self.still = ps.StillDetector(
            float(v("still_v_m_s")), math.radians(float(v("still_w_deg_s"))), float(v("still_s"))
        )
        self.stale_scan_s = float(v("stale_scan_s"))
        self.stale_travel_m = float(v("stale_travel_m"))
        self._snap_next = False  # /initialpose seen: take AMCL's next pose at once
        self._scan_t: float | None = None
        self._last: tuple[float, ps.Pose] | None = None  # previous tick (t, odom->base)
        self._travel_since_amcl = 0.0
        self._gap_log_t = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.raw_pub = self.create_publisher(TransformStamped, RAW_TOPIC, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initialpose, RELIABLE_1)
        self.create_subscription(LaserScan, "/scan_gated", self._on_scan, SENSOR_DATA)
        self.create_timer(1.0 / float(v("rate_hz")), self._tick)
        self.get_logger().info(
            f"smoothing {self.map_f}->{self.odom_f}: tau {v('tau_s')} s, at most {v('max_rate_m_s')} m/s "
            f"+ {v('per_m')} m per m, {v('max_rate_deg_s')} deg/s + {v('per_m_deg')} deg per m"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_base(self, stamp=None) -> ps.Pose | None:
        try:
            tr = self.tf_buffer.lookup_transform(self.odom_f, self.base_f, stamp or Time())
        except Exception:  # noqa: BLE001 - not there (yet) / stamp not covered
            return None
        t = tr.transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def _on_scan(self, _m: LaserScan) -> None:
        self._scan_t = self._now()

    def _on_initialpose(self, _m: PoseWithCovarianceStamped) -> None:
        self._snap_next = True

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        # AMCL's map->odom is its pose at the scan's stamp with odometry at that same stamp
        ob = self._odom_base(Time.from_msg(msg.header.stamp)) or self._odom_base()
        if ob is None:
            return
        p = msg.pose.pose
        raw = ps.compose((p.position.x, p.position.y, yaw_of(p.orientation)), ps.inverse(ob))
        self.smoother.set_raw(raw, snap=self._snap_next)
        self._snap_next = False
        self._travel_since_amcl = 0.0
        self.raw_pub.publish(to_tf(raw, msg.header.stamp, self.map_f, self.odom_f))

    def _tick(self) -> None:
        t = self._now()
        ob = self._odom_base()
        if ob is None:
            return
        dt = travel = turned = 0.0
        if self._last is not None:
            t0, o0 = self._last
            dt = t - t0
            travel = math.hypot(ob[0] - o0[0], ob[1] - o0[1])
            turned = abs(ps.wrap(ob[2] - o0[2]))
        self._last = (t, ob)
        self._travel_since_amcl += travel
        still = self.still.update(t, ob)
        out = self.smoother.step(ob, dt, travel, turned, still)
        if out is None:
            return
        if self._scan_t is None or t - self._scan_t > self.stale_scan_s:
            return  # AMCL gets no scans: map->odom goes stale, as when AMCL broadcast it
        if self._travel_since_amcl > self.stale_travel_m:
            return  # driven a metre without an AMCL update: AMCL stopped
        stamp = (self.get_clock().now() + self.future).to_msg()
        self.tf_pub.sendTransform(to_tf(out, stamp, self.map_f, self.odom_f))
        g = self.smoother.gap(ob)
        if g is not None and (g[0] > 0.10 or g[1] > math.radians(2.0)) and t - self._gap_log_t > 5.0:
            self._gap_log_t = t
            self.get_logger().info(
                f"working in a correction: {g[0]:.3f} m / {math.degrees(g[1]):.2f} deg still to go"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseSmootherNode()
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
