#!/usr/bin/env python3
"""map_scale.py - is the MAP the right length along an aisle? (2026-11-09)

Why: scan_odom_sync showed the wheels and the lidar timing are right, yet every localisation
correction is "ahead -" (the AMR thought it was further along than it was). If the map is
SHORTER than the real aisle (SLAM squeezed it where the aisle had little across it), true
wheel distance carries the AMR past where the map says it is, and AMCL pulls it back in
steps. This compares the map's distance between two stops with the wheels' distance.

How (service running, localised READY, MANUAL):
  1.  source ~/agv_can/amr_ws/install/setup.bash
      source ~/agv_can/amr_ws/env/vehicle.sh
      python3 ~/agv_can/tools/map_scale.py
  2. Park at A: a spot where the walls around are clear on the map (the scans line up).
     Press Enter. It holds 3 s, nudges AMCL to update in place, and records A.
  3. Drive straight along the aisle (where the corrections happen) to B, 15 m or more.
     Stop where the walls are clear again. Press Enter. Repeat for C, D ... (Ctrl-C ends).
  Each leg prints the wheels' distance, the map's distance and the difference in %.
  Then drive the same legs back (B -> A) once: a map that is too short reads the same both ways.
"""

import math
import sys
import threading
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty

SETTLE_S = 3.0
NOMOTION_CALLS = 6


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("map_scale")
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.nomotion = self.create_client(Empty, "/request_nomotion_update")

    def pose(self, parent: str) -> tuple[float, float, float] | None:
        try:
            tr = self.buf.lookup_transform(parent, "base_footprint", Time())
        except Exception:  # noqa: BLE001
            return None
        t = tr.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)


def mark(node: Probe) -> tuple | None:
    time.sleep(SETTLE_S)
    for _ in range(NOMOTION_CALLS):
        if node.nomotion.service_is_ready():
            node.nomotion.call_async(Empty.Request())
        time.sleep(0.5)
    m, o = node.pose("map"), node.pose("odom")
    if m is None or o is None:
        print("no map->base_footprint or odom->base_footprint transform: localised and READY?")
        return None
    return m, o


def main() -> int:
    rclpy.init()
    node = Probe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    marks: list[tuple[str, tuple, tuple]] = []
    total_o = total_m = 0.0
    try:
        while True:
            name = chr(ord("A") + len(marks))
            input(f"park at {name}, stand still, then press Enter ")
            r = mark(node)
            if r is None:
                continue
            m, o = r
            print(f"  {name}: map x {m[0]:.3f} y {m[1]:.3f} yaw {math.degrees(m[2]):.1f} deg")
            if marks:
                pn, pm, po = marks[-1]
                d_o = math.hypot(o[0] - po[0], o[1] - po[1])
                d_m = math.hypot(m[0] - pm[0], m[1] - pm[1])
                total_o, total_m = total_o + d_o, total_m + d_m
                pct = 100.0 * (d_m / d_o - 1.0) if d_o > 0.5 else float("nan")
                print(
                    f"  {pn} -> {name}: wheels {d_o:.3f} m, map {d_m:.3f} m, "
                    f"map {d_m - d_o:+.3f} m ({pct:+.2f} %)"
                )
                if d_o < 5.0:
                    print("  (under 5 m: too short to say much)")
            marks.append((name, m, o))
    except (KeyboardInterrupt, EOFError):
        print()
    if total_o > 0.5:
        print(
            f"all legs: wheels {total_o:.3f} m, map {total_m:.3f} m, "
            f"map {100.0 * (total_m / total_o - 1.0):+.2f} % (- = the map is shorter than the floor)"
        )
        print("the wheels were checked against a wall (scan_odom_sync): a difference over ~1 % is the map")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
