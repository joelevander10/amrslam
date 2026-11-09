"""Where the map no longer fits the floor, learnt during runs (2026-11-09). Pure numpy.

The localization monitor classifies every beam of the scan check (scan_consistency): on a
mapped obstacle, on something not on the map (UNMAPPED), or through a mapped obstacle into
mapped floor (THROUGH: the mapped thing is gone). This adds those up per map BLOCK (BLOCK_M
square) while the pose can be trusted - localisation READY and the scan fitting - and at
most once per MOVE_M / MOVE_RAD of travel, so a parked AMR does not weigh one spot forever:

  hit   a mapped obstacle seen in place      (the endpoint's block)
  gone  a mapped obstacle looked through     (the block of the mapped cell the beam passed)
  new   an object where the map has floor    (the endpoint's block)

Counts fade with a half-life of HALF_LIFE_S, so the layer shows the floor as it is lately.
One file per map revision, <state_dir>/map_changes/<map_id>-rev<N>.npz; a new revision
starts from nothing (its map is different).

regions() groups the blocks that changed into areas and says what to do about each:
  gone, seen through nearly always  -> the mapped thing left for good: Re-map area
  gone, sometimes there             -> it comes and goes: Dynamic area
  new                               -> objects not on the map: Dynamic area if they move,
                                       Re-map area if they stay
"""

from __future__ import annotations

import math
import os
import time

import numpy as np

BLOCK_M = 0.20
HALF_LIFE_S = 3 * 86400.0
MOVE_M = 0.20
MOVE_RAD = math.radians(10.0)
MIN_SHOW = 3.0  # a block's gone + new (faded) before it is drawn or grouped
GONE_FOR_GOOD = 0.8  # gone / (gone + hit) at or above: left for good
GONE_SOMETIMES = 0.2  # ... at or above (below GONE_FOR_GOOD): comes and goes
MAX_REGIONS = 12
MAX_BLOCKS = 6000
SUBDIR = "map_changes"

# scan_consistency classes
ON_MAP, UNMAPPED, THROUGH = 1, 2, 3


def path_for(state_dir: str, map_id: str, revision: int) -> str:
    return os.path.join(state_dir, SUBDIR, f"{map_id}-rev{int(revision)}.npz")


class ChangeMap:
    def __init__(self, width: int, height: int, resolution: float, origin_x: float, origin_y: float) -> None:
        self.width, self.height = int(width), int(height)
        self.resolution, self.origin_x, self.origin_y = float(resolution), float(origin_x), float(origin_y)
        self.k = max(1, int(round(BLOCK_M / self.resolution)))  # map cells per block side
        self.bw, self.bh = -(-self.width // self.k), -(-self.height // self.k)
        shape = (self.bh, self.bw)
        self.hit = np.zeros(shape, dtype=np.float32)
        self.gone = np.zeros(shape, dtype=np.float32)
        self.new = np.zeros(shape, dtype=np.float32)
        self.t_fade = time.time()
        self.scans = 0  # scans counted (not faded)
        self.updated = 0.0
        self._last: tuple[float, float, float] | None = None
        self.dirty = False

    def same_geometry(
        self, width: int, height: int, resolution: float, origin_x: float, origin_y: float
    ) -> bool:
        return (
            (self.width, self.height) == (int(width), int(height))
            and abs(self.resolution - resolution) < 1e-9
            and abs(self.origin_x - origin_x) < 1e-6
            and abs(self.origin_y - origin_y) < 1e-6
        )

    # ---- counting ----------------------------------------------------------------

    def moved_enough(self, pose) -> bool:
        if self._last is None:
            return True
        d = math.hypot(pose[0] - self._last[0], pose[1] - self._last[1])
        a = abs(math.atan2(math.sin(pose[2] - self._last[2]), math.cos(pose[2] - self._last[2])))
        return d >= MOVE_M or a >= MOVE_RAD

    def add(self, pose, x, y, cls, ex, ey, now: float | None = None) -> bool:
        """Count one classified scan taken at laser `pose` (x, y, yaw). x / y: every beam's
        endpoint; cls: its class; ex / ey: where the map expected the beam to stop (nan = no
        wall expected). Returns False when skipped (not moved enough since the last one)."""
        if not self.moved_enough(pose):
            return False
        self._last = tuple(pose)
        cls = np.asarray(cls)
        x, y, ex, ey = (np.asarray(v, dtype=np.float64) for v in (x, y, ex, ey))
        self._bump(self.hit, x[cls == ON_MAP], y[cls == ON_MAP])
        self._bump(self.new, x[cls == UNMAPPED], y[cls == UNMAPPED])
        th = cls == THROUGH
        self._bump(self.gone, ex[th], ey[th])
        self.scans += 1
        self.updated = time.time() if now is None else now
        self.dirty = True
        return True

    def _bump(self, arr: np.ndarray, x: np.ndarray, y: np.ndarray) -> None:
        ok = np.isfinite(x) & np.isfinite(y)
        if not ok.any():
            return
        bc = np.floor((x[ok] - self.origin_x) / (self.resolution * self.k)).astype(np.int64)
        br = np.floor((y[ok] - self.origin_y) / (self.resolution * self.k)).astype(np.int64)
        inside = (bc >= 0) & (bc < self.bw) & (br >= 0) & (br < self.bh)
        if not inside.any():
            return
        flat = np.unique(br[inside] * self.bw + bc[inside])  # a block counts once per scan
        arr.reshape(-1)[flat] += 1.0

    def fade(self, now: float | None = None) -> None:
        now = time.time() if now is None else now
        dt = now - self.t_fade
        if dt <= 0:
            return
        f = np.float32(0.5 ** (dt / HALF_LIFE_S))
        self.hit *= f
        self.gone *= f
        self.new *= f
        self.t_fade = now

    # ---- reading -------------------------------------------------------------------

    def _centre(self, br: np.ndarray, bc: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        s = self.resolution * self.k
        return self.origin_x + (bc + 0.5) * s, self.origin_y + (br + 0.5) * s

    def blocks(self, limit: int = MAX_BLOCKS) -> list[list[float]]:
        """[[x, y, gone, new, hit], ...] of the blocks worth drawing, strongest first."""
        score = self.gone + self.new
        br, bc = np.nonzero(score >= MIN_SHOW)
        if br.size == 0:
            return []
        order = np.argsort(-score[br, bc])[:limit]
        br, bc = br[order], bc[order]
        cx, cy = self._centre(br, bc)
        return [
            [
                round(float(a), 3),
                round(float(b), 3),
                round(float(g), 1),
                round(float(n), 1),
                round(float(h), 1),
            ]
            for a, b, g, n, h in zip(
                cx, cy, self.gone[br, bc], self.new[br, bc], self.hit[br, bc], strict=True
            )
        ]

    def regions(self, limit: int = MAX_REGIONS) -> list[dict]:
        """Changed areas (blocks within 2 of each other grouped), strongest first, with advice."""
        sig = (self.gone + self.new) >= MIN_SHOW
        if not sig.any():
            return []
        seen = np.zeros_like(sig)
        out = []
        for r0, c0 in zip(*np.nonzero(sig), strict=True):
            if seen[r0, c0]:
                continue
            stack, cells = [(r0, c0)], []
            seen[r0, c0] = True
            while stack:
                r, c = stack.pop()
                cells.append((r, c))
                for dr in range(-2, 3):
                    for dc in range(-2, 3):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < self.bh and 0 <= cc < self.bw and sig[rr, cc] and not seen[rr, cc]:
                            seen[rr, cc] = True
                            stack.append((rr, cc))
            rs = np.array([p[0] for p in cells])
            cs = np.array([p[1] for p in cells])
            g, n, h = (
                float(self.gone[rs, cs].sum()),
                float(self.new[rs, cs].sum()),
                float(self.hit[rs, cs].sum()),
            )
            s = self.resolution * self.k
            box = [
                self.origin_x + cs.min() * s,
                self.origin_y + rs.min() * s,
                self.origin_x + (cs.max() + 1) * s,
                self.origin_y + (rs.max() + 1) * s,
            ]
            out.append(
                {
                    "box": [round(v, 2) for v in box],
                    "gone": round(g, 1),
                    "new": round(n, 1),
                    "hit": round(h, 1),
                    "blocks": len(cells),
                    **advice(g, n, h),
                }
            )
        out.sort(key=lambda d: -(d["gone"] + d["new"]))
        return out[:limit]

    # ---- files ---------------------------------------------------------------------

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "wb") as fh:
            np.savez_compressed(
                fh,
                hit=self.hit,
                gone=self.gone,
                new=self.new,
                meta=np.array(
                    [
                        self.width,
                        self.height,
                        self.resolution,
                        self.origin_x,
                        self.origin_y,
                        self.t_fade,
                        self.scans,
                        self.updated,
                    ],
                    dtype=np.float64,
                ),
            )
        os.replace(tmp, path)
        self.dirty = False


def advice(gone: float, new: float, hit: float) -> dict:
    if gone >= new:
        ratio = gone / max(gone + hit, 1e-6)
        if ratio >= GONE_FOR_GOOD:
            return {
                "kind": "gone",
                "ratio": round(ratio, 2),
                "advice": "mapped objects are gone (seen through nearly every time): Re-map area, "
                "or Erase to unknown if they will not come back",
            }
        if ratio >= GONE_SOMETIMES:
            return {
                "kind": "varies",
                "ratio": round(ratio, 2),
                "advice": "mapped objects come and go (trolleys, pallets): mark it Dynamic area",
            }
        return {
            "kind": "varies",
            "ratio": round(ratio, 2),
            "advice": "mostly in place, now and then missing: Dynamic area if it keeps showing up",
        }
    return {
        "kind": "new",
        "ratio": 0.0,
        "advice": "objects that are not on the map: Dynamic area if they move (trolleys), "
        "Re-map area if they are there to stay (a new machine)",
    }


def load(path: str) -> ChangeMap | None:
    """The saved counts (faded to now), or None when there are none or they are unreadable."""
    try:
        with np.load(path) as z:
            m = z["meta"]
            cm = ChangeMap(int(m[0]), int(m[1]), float(m[2]), float(m[3]), float(m[4]))
            if z["hit"].shape != cm.hit.shape:
                return None
            cm.hit, cm.gone, cm.new = (z[k].astype(np.float32) for k in ("hit", "gone", "new"))
            cm.t_fade, cm.scans, cm.updated = float(m[5]), int(m[6]), float(m[7])
    except (OSError, KeyError, ValueError, IndexError):
        return None
    cm.fade()
    return cm
