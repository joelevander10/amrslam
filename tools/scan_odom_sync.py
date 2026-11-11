#!/usr/bin/env python3
"""scan_odom_sync.py - wheel distance scale AND lidar time lag, from one drive at a wall (2026-11-09).

Why: every localisation correction so far was "ahead -" (along the path, backwards), bigger
at speed. Two causes give that sign:
  * the wheels count too far (wheel_dia_m too small)        -> error grows with DISTANCE
  * the scan is stamped later than it was taken (time lag)  -> error grows with SPEED
This tool measures both against a flat wall, no tape needed: the lidar's distance to the
wall against the wheel odometry's distance, while the AMR starts, drives and stops.
  scale = how far the wall really came closer / how far the wheels say they drove
  lag   = the time shift that best lines the two up (the start and the stop give it away)

How:
  1. A flat, plain wall (no racks, no glass), the AMR 6-8 m from it, FACING it, square.
     Nothing between them. Service running, MANUAL.
  2.  source ~/agv_can/amr_ws/install/setup.bash
      source ~/agv_can/amr_ws/env/vehicle.sh
      python3 ~/agv_can/tools/scan_odom_sync.py
  3. When it says GO: hold W only, drive at the wall, let go about 2 m before it (keep out of
     the scanner's protective field). Wait until it prints the result (2 s still).
  Do it 2-3 times and send all outputs to Claude.
"""

import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HALF_FOV = math.radians(12.0)  # beams around straight ahead used for the wall
MIN_POINTS = 15
INLIER_M = 0.04
MOVING_MPS = 0.05
STILL_S = 2.0
MIN_TRAVEL_M = 2.0


def stamp(h) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def wall_distance(m: LaserScan) -> tuple[float, float] | None:
    """(perpendicular distance lidar -> wall line, wall normal angle) or None."""
    n = len(m.ranges)
    a = m.angle_min + m.angle_increment * np.arange(n)
    r = np.asarray(m.ranges, dtype=float)
    ok = (np.abs(a) <= HALF_FOV) & np.isfinite(r) & (r > max(m.range_min, 0.3)) & (r < min(m.range_max, 15.0))
    if ok.sum() < MIN_POINTS:
        return None
    x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
    keep = np.ones(len(x), dtype=bool)
    for _ in range(3):
        if keep.sum() < MIN_POINTS:
            return None
        cx, cy = x[keep].mean(), y[keep].mean()
        cov = np.cov(np.vstack([x[keep] - cx, y[keep] - cy]))
        w, v = np.linalg.eigh(cov)
        nx, ny = v[:, 0]  # normal = smallest eigenvector
        res = np.abs((x - cx) * nx + (y - cy) * ny)
        keep = res < INLIER_M
    if keep.sum() < MIN_POINTS:
        return None
    d = abs(cx * nx + cy * ny)
    ang = math.atan2(ny, nx)
    if math.cos(ang) < 0:
        ang = math.atan2(-ny, -nx)
    return d, ang


class Rec(Node):
    def __init__(self) -> None:
        super().__init__("scan_odom_sync")
        self.scan: list[tuple[float, float, float, float]] = []  # stamp, arrival, distance, angle
        self.odom: list[tuple[float, float, float]] = []  # stamp, arrival, forward distance
        self.fwd = 0.0
        self.last_pose = None
        self.v = 0.0
        self.moved = False
        self.still_since = None
        self.done = False
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom_raw", self.on_odom, qos_profile_sensor_data)

    def on_scan(self, m: LaserScan) -> None:
        w = wall_distance(m)
        if w is not None:
            self.scan.append((stamp(m.header), time.time(), w[0], w[1]))

    def on_odom(self, m: Odometry) -> None:
        p = m.pose.pose
        th = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        if self.last_pose is not None:
            dx, dy = p.position.x - self.last_pose[0], p.position.y - self.last_pose[1]
            self.fwd += dx * math.cos(th) + dy * math.sin(th)
        self.last_pose = (p.position.x, p.position.y)
        t = stamp(m.header)
        self.odom.append((t, time.time(), self.fwd))
        self.v = abs(m.twist.twist.linear.x)
        if self.v > MOVING_MPS:
            if not self.moved:
                print("driving ...")
            self.moved, self.still_since = True, None
        elif self.moved and abs(self.fwd) >= MIN_TRAVEL_M:
            self.still_since = self.still_since or t
            if t - self.still_since >= STILL_S:
                self.done = True


def analyse(rec: Rec) -> int:
    if len(rec.odom) < 50 or len(rec.scan) < 30:
        print(f"too little data: {len(rec.odom)} odom, {len(rec.scan)} wall scans (is there a wall ahead?)")
        return 1
    od = np.array(rec.odom)
    sc = np.array(rec.scan)
    print(
        f"arrival delay (receive - stamp): scan {1000 * np.median(sc[:, 1] - sc[:, 0]):.0f} ms, "
        f"odom {1000 * np.median(od[:, 1] - od[:, 0]):.0f} ms"
    )
    t_od, d_od = od[:, 0], od[:, 2]
    travel = d_od[-1] - d_od[0]
    span = sc[:, 2].max() - sc[:, 2].min()
    print(f"wheels drove {travel:.3f} m; the wall came {span:.3f} m closer (raw)")
    if travel < MIN_TRAVEL_M:
        print(f"drive {MIN_TRAVEL_M:g} m or more")
        return 1
    best = None
    for lag in np.arange(-0.30, 0.80, 0.002):
        t = sc[:, 0] - lag  # when the scan was really taken, if it is stamped `lag` late
        inside = (t >= t_od[0]) & (t <= t_od[-1])
        if inside.sum() < 30:
            continue
        x = np.interp(t[inside], t_od, d_od)
        y = sc[inside, 2]
        A = np.vstack([np.ones_like(x), -x]).T
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        rms = float(np.sqrt(np.mean((A @ coef - y) ** 2)))
        if best is None or rms < best[0]:
            best = (rms, lag, coef[1])
    rms, lag, k = best
    ang = math.degrees(float(np.median(sc[:, 3])))
    k_true = k / math.cos(math.radians(ang))  # driving not quite square to the wall
    print(f"wall angle {ang:+.1f} deg (0 = square); fit error {1000 * rms:.1f} mm")
    print()
    print(f"SCALE: the wall came {k_true:.4f} m closer per metre the wheels counted")
    print(f"       -> the wheels count {100 * (1 / k_true - 1):+.2f} % (+ = too far)")
    try:
        from agv_core import config  # noqa: PLC0415

        print(f"       -> wheel_dia_m {config.WHEEL_DIA_M:.4f} -> {config.WHEEL_DIA_M * k_true:.4f}")
    except Exception:  # noqa: BLE001
        pass
    print(f"LAG:   the scan is stamped {1000 * lag:+.0f} ms after it was taken")
    print(f"       -> at 0.8 m/s that alone puts localisation {0.8 * lag * 100:.0f} cm behind")
    if rms > 0.02:
        print("       fit error over 20 mm: the wall is not plain/flat or something moved - redo it")
    return 0


def main() -> int:
    rclpy.init()
    rec = Rec()
    print("waiting for /scan and /odom_raw ...")
    t0 = time.time()
    said_go = False
    try:
        while rclpy.ok() and not rec.done:
            rclpy.spin_once(rec, timeout_sec=0.05)
            if not said_go and rec.scan and rec.odom:
                said_go = True
                print(f"wall at {rec.scan[-1][2]:.2f} m. GO: hold W, let go about 2 m before the wall")
            if not said_go and time.time() - t0 > 5.0:
                print("no wall straight ahead in /scan, or no /odom_raw (service running?)")
                return 1
    except KeyboardInterrupt:
        pass
    rec.destroy_node()
    rclpy.shutdown()
    return analyse(rec)


if __name__ == "__main__":
    sys.exit(main())
