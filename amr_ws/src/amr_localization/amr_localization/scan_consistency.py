"""Scan-versus-map consistency for the localisation monitor (pure numpy, no ROS).

match: fraction of returned beams whose endpoint lies within `tol` of a mapped
obstacle. Informational: clutter lowers it, a speckled map raises it.

long: fraction of beams (among those the map expects to hit a wall within
range) that measure farther than that wall by more than `tol` AND end in
mapped free space. This is the loss trigger. A beam that passes a mapped wall
and lands on another mapped obstacle is the map being seen THROUGH - wire mesh,
railings, glass (the survey cage is mesh, 2026-09-17) - not a pose error; a
wrong pose puts endpoints in free space wholesale. A beam that ends in unknown
or off the map, or has no return, proves nothing either way.
"""

from __future__ import annotations

import numpy as np
from amr_maps.grid import Grid
from amr_maps.raycast import ScanGeometry, cast

OCCUPIED = 65
# per-beam classes (compare(..., detail={})): no return / on a mapped obstacle / ended in mapped
# free floor (something that is not on the map) / through a mapped wall into free floor / in a
# dynamic area / unknown or off the map
NONE, ON_MAP, UNMAPPED, THROUGH, DYNAMIC, OTHER = 0, 1, 2, 3, 4, 5


def occupied_near(grid: Grid, tol_m: float) -> np.ndarray:
    """Occupied cells dilated by tol_m (square, wrap-free)."""
    occ = grid.data >= OCCUPIED
    cells = max(1, int(round(tol_m / grid.meta.resolution)))
    p = np.pad(occ, cells)
    near = np.zeros_like(occ)
    h, w = occ.shape
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            near |= p[cells + dr : cells + dr + h, cells + dc : cells + dc + w]
    return near


def compare(
    grid: Grid,
    occ_near: np.ndarray,
    lx: float,
    ly: float,
    yaw: float,
    ranges: np.ndarray,
    geom: ScanGeometry,
    tol_m: float,
    dynamic: np.ndarray | None = None,
    detail: dict | None = None,
) -> tuple[float, float]:
    """(match_frac, long_frac) for a scan taken at laser pose (lx, ly, yaw) in the map frame.

    `dynamic` (bool, the map's shape): beams ending in a dynamic area, or whose expected wall
    lies in one, are left out of both fractions: a trolley that is there, gone or new proves
    nothing about the pose (dynamic-mapping plan §1.2).
    `detail` (a dict to fill, optional): x, y (every beam's endpoint in the map frame), cls (the
    classes above) and unmapped_frac (returned beams ending on mapped free floor)."""
    ranges = np.asarray(ranges, dtype=np.float64)
    n = len(ranges)
    expected = cast(grid, lx, ly, yaw, geom)  # inf where the map has nothing within range
    measured = np.where(np.isfinite(ranges), ranges, np.inf)

    valid = np.isfinite(ranges) & (ranges >= geom.range_min) & (ranges < geom.range_max)
    a = geom.angles + yaw
    m = grid.meta

    def cells(r: np.ndarray, ok: np.ndarray):
        d = np.where(ok, r, 0.0)  # inf * cos would raise; masked beams are dropped below
        cols = np.floor((lx + d * np.cos(a) - m.origin_x) / m.resolution)
        rows = np.floor((ly + d * np.sin(a) - m.origin_y) / m.resolution)
        inside = ok & (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
        return inside, rows[inside].astype(int), cols[inside].astype(int)

    inside, ri, ci = cells(ranges, valid)
    wall_expected = np.isfinite(expected)
    if dynamic is not None:
        in_dyn = np.zeros(n, dtype=bool)
        in_dyn[inside] = dynamic[ri, ci]
        exp_in, er, ec = cells(expected, wall_expected)
        wall_dyn = np.zeros(n, dtype=bool)
        wall_dyn[exp_in] = dynamic[er, ec]
        drop = in_dyn | wall_dyn
        valid, wall_expected = valid & ~drop, wall_expected & ~drop
        inside, ri, ci = cells(ranges, valid)
    near_obstacle = np.zeros(n, dtype=bool)
    near_obstacle[inside] = occ_near[ri, ci]
    in_free = np.zeros(n, dtype=bool)
    in_free[inside] = (grid.data[ri, ci] >= 0) & ~near_obstacle[inside]

    long = wall_expected & (measured > expected + tol_m) & in_free
    long_frac = float(long.sum()) / max(1, int(wall_expected.sum()))
    match_frac = float(near_obstacle[valid].mean()) if valid.sum() >= 20 else 0.0
    if detail is not None:
        # for the loss report (2026-11-09): every beam's endpoint and what it says
        # ends in mapped free floor: an object that is not on the map
        short = valid & ~near_obstacle & in_free & ~long
        cls = np.full(n, NONE, dtype=np.int8)
        cls[valid] = OTHER
        cls[short] = UNMAPPED
        cls[long] = THROUGH
        cls[valid & near_obstacle] = ON_MAP
        if dynamic is not None:
            cls[drop & np.isfinite(ranges)] = DYNAMIC
        d = np.where(np.isfinite(ranges), ranges, 0.0)
        detail.update(
            x=lx + d * np.cos(a), y=ly + d * np.sin(a), cls=cls,
            unmapped_frac=float(short.sum()) / max(1, int(valid.sum())),
        )
    return match_frac, long_frac
