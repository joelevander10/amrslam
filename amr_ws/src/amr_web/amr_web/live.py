"""Live map, pose and scan for the browser (unified plan §6.4). Pure helpers +
a small snapshot store; the ROS subscriptions live in the adapter.

  * The occupancy grid is kept as the latest message plus a snapshot id that
    changes with every new grid; the PNG is encoded lazily, at most once per
    snapshot, by whichever request first asks for it, never inside a callback
    and never under the adapter's state lock.
  * Scan points are transformed at the scan timestamp into `map` (or `odom`
    when there is no map frame), decimated to a bounded count.
  * Everything is tagged with the supervisor generation it was received under
    and dropped on a generation change: new-map scans are never drawn over an
    old-map image.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np

from amr_web.png import encode_gray

MAX_POINTS = 400


def grid_to_gray(data: list[int] | np.ndarray, width: int, height: int) -> np.ndarray:
    """OccupancyGrid data (-1 unknown, 0 free .. 100 occupied) -> uint8 image,
    row 0 at the TOP (the bundle PNG convention): 254 free, 205 unknown, 0 occupied."""
    a = np.asarray(data, dtype=np.int16).reshape(height, width)
    img = np.full(a.shape, 205, dtype=np.uint8)
    img[a == 0] = 254
    img[a > 50] = 0
    img[(a > 0) & (a <= 50)] = 254 - (a[(a > 0) & (a <= 50)] * 2).astype(np.uint8)
    return np.flipud(img)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def scan_points(ranges, angle_min: float, angle_inc: float, range_min: float, range_max: float, tx, ty, tyaw):
    """Valid beams as (x, y) in the target frame given the laser pose (tx, ty, tyaw)."""
    n = len(ranges)
    if n == 0:
        return []
    step = max(1, n // MAX_POINTS)
    c, s = math.cos(tyaw), math.sin(tyaw)
    out = []
    for i in range(0, n, step):
        r = ranges[i]
        if not (range_min <= r <= range_max) or not math.isfinite(r):
            continue
        a = angle_min + i * angle_inc
        lx, ly = r * math.cos(a), r * math.sin(a)
        out.append((round(tx + c * lx - s * ly, 3), round(ty + s * lx + c * ly, 3)))
    return out


class LiveStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.generation = 0
        self.grid = None  # latest OccupancyGrid
        self.grid_t = 0.0
        self.snapshot = 0
        self._png: tuple[int, bytes] | None = None
        self.pose = None  # (x, y, yaw, frame)
        self.pose_t = 0.0
        self.scan = None  # list of (x, y)
        self.scan_frame = ""
        self.scan_t = 0.0

    def set_generation(self, gen: int) -> None:
        with self._lock:
            if gen != self.generation:
                self.generation = gen
                self.grid = None
                self._png = None
                self.pose = self.scan = None
                self.snapshot += 1

    def on_grid(self, msg) -> None:
        with self._lock:
            self.grid = msg
            self.grid_t = time.monotonic()
            self.snapshot += 1

    def set_pose(self, x: float, y: float, yaw: float, frame: str, source_age_s: float = 0.0) -> None:
        """`source_age_s`: how old the transform this pose came from already was when read
        (review Q12). Re-reading a frozen cached transform must not make it look fresh, so
        the stored time is the SOURCE time, and age_s grows from it."""
        with self._lock:
            self.pose = (x, y, yaw, frame)
            self.pose_t = time.monotonic() - max(0.0, float(source_age_s))

    def set_scan(self, pts, frame: str, source_age_s: float = 0.0) -> None:
        with self._lock:
            self.scan, self.scan_frame = pts, frame
            self.scan_t = time.monotonic() - max(0.0, float(source_age_s))

    def map_meta(self) -> dict | None:
        with self._lock:
            g = self.grid
            if g is None:
                return None
            o = g.info.origin
            return {
                "snapshot": self.snapshot,
                "generation": self.generation,
                "width": int(g.info.width),
                "height": int(g.info.height),
                "resolution": float(g.info.resolution),
                "origin": [float(o.position.x), float(o.position.y), yaw_of(o.orientation)],
                "frame_id": g.header.frame_id,
                "age_s": time.monotonic() - self.grid_t,
            }

    def map_png(self, snapshot: int) -> bytes | None:
        """Encode outside the lock, once per snapshot."""
        with self._lock:
            if self.grid is None or snapshot != self.snapshot:
                return None
            if self._png and self._png[0] == snapshot:
                return self._png[1]
            g, snap = self.grid, self.snapshot
        png = encode_gray(grid_to_gray(g.data, g.info.width, g.info.height))
        with self._lock:
            if self.snapshot == snap:
                self._png = (snap, png)
        return png

    def pose_scan(self) -> dict:
        now = time.monotonic()
        with self._lock:
            return {
                "generation": self.generation,
                "pose": None
                if self.pose is None
                else {
                    "x": self.pose[0],
                    "y": self.pose[1],
                    "yaw": self.pose[2],
                    "frame": self.pose[3],
                    "age_s": now - self.pose_t,
                },
                "scan": None
                if self.scan is None
                else {"points": self.scan, "frame": self.scan_frame, "age_s": now - self.scan_t},
            }
