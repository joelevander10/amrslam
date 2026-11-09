"""The latest survey's own scans as an occupancy grid (Re-map area, 2026-11-09). Pure numpy.

Why not slam_toolbox's map: a continued survey loads the old pose graph, and its grid counts
the OLD scans too. A pallet row seen 50 times in the first survey stays occupied however
often the new drive looks through the gap it left. Re-map area needs what the floor looks
like NOW, so the mapping session keeps the continued survey's own scans (keyframes: the
laser pose from SLAM's live estimate, and the ranges) and this module turns only those into
a grid on the saved map's geometry:

  hit   a beam's endpoint (a return within max_range_m)
  pass  every cell the beam crossed before it
  occupied  >= MIN_HITS hits and hits / (hits + passes) >= OCC_RATIO
  free      >= MIN_PASSES passes and not occupied
  unknown   everything else (never seen by the new drive)

No-return beams prove nothing here (glass, black surfaces, out of range) and are skipped.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from amr_maps.grid import Grid, GridMeta

MIN_HITS = 2
MIN_PASSES = 2
OCC_RATIO = 0.25
MAX_RANGE_M = 10.0  # beyond this the cell a beam lands in is uncertain (0.5 deg = 9 cm at 10 m)
BEAM_STEP = 2  # every 2nd beam: the grid cell (5 cm) is coarser than the beam spacing near by


@dataclass
class Keyframe:
    x: float  # laser pose in the map frame
    y: float
    yaw: float
    angle_min: float
    angle_inc: float
    ranges: np.ndarray  # float32, every beam (inf / nan = no return)


def rasterize(
    meta: GridMeta, width: int, height: int, frames: list[Keyframe], max_range_m: float = MAX_RANGE_M
) -> Grid:
    """The keyframes on (meta, width, height): int8 -1 / 0 / 100. Axis-aligned maps only."""
    res = meta.resolution
    n_cells = width * height
    hits = np.zeros(n_cells, dtype=np.int64)
    passes = np.zeros(n_cells, dtype=np.int64)
    step = res * 0.7  # samples along a beam: never skip a cell
    k = int(np.ceil(max_range_m / step))
    d = (np.arange(k, dtype=np.float64) * step)[None, :]  # (1, k)
    hit_idx: list[np.ndarray] = []
    pass_idx: list[np.ndarray] = []
    pending = 0
    for f in frames:
        r = np.asarray(f.ranges, dtype=np.float64)
        idx = np.arange(0, len(r), BEAM_STEP)
        r = r[idx]
        ok = np.isfinite(r) & (r > 0.05) & (r <= max_range_m)
        if not ok.any():
            continue
        a = f.yaw + f.angle_min + f.angle_inc * idx[ok]
        r = r[ok]
        c, s = np.cos(a), np.sin(a)
        hit_idx.append(_cells(f.x + r * c, f.y + r * s, meta, width, height))
        # passes: every beam sampled from the laser up to one cell short of its endpoint
        keep = d < (r[:, None] - res)  # (beams, k)
        pass_idx.append(
            _cells((f.x + d * c[:, None])[keep], (f.y + d * s[:, None])[keep], meta, width, height)
        )
        pending += int(keep.sum())
        if pending > 5_000_000:  # one bincount per few million samples, not per frame
            _flush(hits, hit_idx)
            _flush(passes, pass_idx)
            pending = 0
    _flush(hits, hit_idx)
    _flush(passes, pass_idx)
    hits = hits.reshape(height, width)
    passes = passes.reshape(height, width)
    data = np.full((height, width), -1, dtype=np.int8)
    total = hits + passes
    occ = (hits >= MIN_HITS) & (hits >= OCC_RATIO * np.maximum(total, 1))
    free = (passes >= MIN_PASSES) & ~occ
    data[free] = 0
    data[occ] = 100
    return Grid(data, meta)


def _cells(x: np.ndarray, y: np.ndarray, meta: GridMeta, width: int, height: int) -> np.ndarray:
    col = np.floor((x - meta.origin_x) / meta.resolution).astype(np.int64)
    row = np.floor((y - meta.origin_y) / meta.resolution).astype(np.int64)
    ok = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    return row[ok] * width + col[ok]


def _flush(counts: np.ndarray, parts: list[np.ndarray]) -> None:
    if parts:
        counts += np.bincount(np.concatenate(parts), minlength=counts.size)
        parts.clear()


def to_arrays(frames: list[Keyframe]) -> dict:
    """Keyframes as plain arrays (np.savez), and back with from_arrays."""
    if not frames:
        return {"pose": np.zeros((0, 5)), "ranges": np.zeros((0, 0), dtype=np.float32)}
    n = max(len(f.ranges) for f in frames)
    rng = np.full((len(frames), n), np.inf, dtype=np.float32)
    for i, f in enumerate(frames):
        rng[i, : len(f.ranges)] = f.ranges
    pose = np.array([[f.x, f.y, f.yaw, f.angle_min, f.angle_inc] for f in frames], dtype=np.float64)
    return {"pose": pose, "ranges": rng}


def from_arrays(pose: np.ndarray, ranges: np.ndarray) -> list[Keyframe]:
    return [Keyframe(*map(float, p), ranges=r) for p, r in zip(pose, ranges, strict=True)]
