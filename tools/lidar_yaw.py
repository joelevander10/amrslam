#!/usr/bin/env python3
"""lidar_yaw.py - is the lidar's 0 deg really the AMR's straight ahead? (2026-11-09)

Symptom it checks: while mapping, the map turns when the AMR drives STRAIGHT (most after a
180 deg turn). A lidar turned by a few degrees against the URDF's laser_yaw makes a straight
drive look like a diagonal one to SLAM, which then turns the AMR - and the map - to agree.

Set up: AMR parked, service running. Put a THIN upright object (a pole, a chair leg, a bottle)
exactly on the AMR's centre line, D metres in front of the lidar (stretch a string from the
middle of the rear to the middle of the front, measure along it). Nothing else within 0.5 m
of it. Then:

    python3 ~/agv_can/tools/lidar_yaw.py 2.0          # D = 2.0 m
    python3 ~/agv_can/tools/lidar_yaw.py 2.0 1.0      # optional 2nd run at 1.0 m: also the
                                                      # sideways offset (laser_y)

Reads the RAW scan (/scan, laser frame), finds the object (the nearest point within +-30 deg
and 0.4 .. D+1 m, averaged over its own echo cluster and 40 scans) and prints its angle.
On the centre line it should be 0.0 deg. laser_yaw for the URDF is minus the angle.
"""

import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N_SCANS = 40
WINDOW = math.radians(30.0)


class Probe(Node):
    def __init__(self, d: float) -> None:
        super().__init__("lidar_yaw_probe")
        self.d, self.angles, self.ranges = d, [], []
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, m: LaserScan) -> None:
        pts = []
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            if abs(a) <= WINDOW and math.isfinite(r) and 0.4 <= r <= self.d + 1.0:
                pts.append((r, a))
        if not pts:
            return
        r0 = min(p[0] for p in pts)
        a0 = next(p[1] for p in pts if p[0] == r0)
        cluster = [p for p in pts if abs(p[0] - r0) < 0.10 and abs(p[1] - a0) < math.radians(8)]
        self.ranges.append(statistics.mean(p[0] for p in cluster))
        self.angles.append(statistics.mean(p[1] for p in cluster))


def measure(d: float) -> tuple[float, float]:
    node = Probe(d)
    while rclpy.ok() and len(node.angles) < N_SCANS:
        rclpy.spin_once(node, timeout_sec=1.0)
    a, r = statistics.mean(node.angles), statistics.mean(node.ranges)
    sd = statistics.pstdev(node.angles)
    node.destroy_node()
    print(f"object at {r:.3f} m (you said {d:.2f}), angle {math.degrees(a):+.2f} deg "
          f"(scatter {math.degrees(sd):.2f} deg over {N_SCANS} scans)")
    if abs(r - d) > 0.15:
        print("  ! the range is not what you measured: is that the object, nothing nearer in front?")
    return a, r


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    ds = [float(x) for x in sys.argv[1:3]]
    rclpy.init()
    try:
        res = [measure(d) for d in ds]
    finally:
        rclpy.try_shutdown()
    if len(res) == 1:
        a = res[0][0]
        print(f"\nlaser_yaw = {-a:+.4f} rad ({math.degrees(-a):+.2f} deg)  (laser_y assumed 0)")
        if abs(math.degrees(a)) < 0.5:
            print("within 0.5 deg: the lidar is straight; look elsewhere (send this to Claude)")
        return
    (a1, r1), (a2, r2) = res
    # object on the centre line at range r from a lidar offset y sideways and turned by yaw:
    # angle = atan2(-y, r) - yaw  ->  two ranges give both
    k1, k2 = 1.0 / r1, 1.0 / r2
    y = -(a1 - a2) / (k1 - k2)
    yaw = -(a1 + y * k1)
    print(f"\nlaser_yaw = {yaw:+.4f} rad ({math.degrees(yaw):+.2f} deg), laser_y = {y:+.3f} m")


if __name__ == "__main__":
    main()
