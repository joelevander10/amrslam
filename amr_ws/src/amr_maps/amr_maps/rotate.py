"""Rotate a saved map about its frame origin (2026-11-05): straighten a map so a wall drawn
by the operator (the reference line) runs along the X or Y axis.

The map frame M is turned by `theta` (rad, + = counter-clockwise): a point p of the old map
is R(theta) p in the new one. Every layer of a revision moves together - the occupancy grid,
the dynamic / keepout masks, the survey start mark - so localisation, routes and masks keep
lining up. Grids stay axis-aligned (grid.require_axis_aligned): the rotated image is
RESAMPLED onto a new, larger, axis-aligned grid that holds all of the old one.

Resampling: every new cell takes the old cell under its centre (nearest neighbour, unknown
outside the old map), and then every OCCUPIED old cell is splatted onto every new cell that
2 x 2 points spread over it land in. Nearest neighbour alone breaks a 1-cell wall into a
dotted line - free gaps a route could be validated through; the splat keeps walls closed at
the cost of up to one extra cell of thickness. Masks (100 = marked) do the same.

Pure: numpy only, no files, no ROS. map_bundle.derive_rotation writes the revision.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from amr_maps.grid import Grid, GridMeta

MAX_ANGLE_DEG = 180.0
MIN_ANGLE_DEG = 0.05  # below this the resampling would only add noise


class RotateError(ValueError):
    pass


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def check_angle(theta: float) -> float:
    if isinstance(theta, bool) or not isinstance(theta, (int, float)) or not math.isfinite(theta):
        raise RotateError(f"angle must be a finite number, got {theta!r}")
    deg = math.degrees(theta)
    if abs(deg) > MAX_ANGLE_DEG + 1e-9:
        raise RotateError(f"angle must be within +-{MAX_ANGLE_DEG:g} deg, got {deg:.2f}")
    if abs(deg) < MIN_ANGLE_DEG:
        raise RotateError(f"angle {deg:.3f} deg is too small to rotate by (at least {MIN_ANGLE_DEG} deg)")
    return float(theta)


def align_angle(x0: float, y0: float, x1: float, y1: float) -> float:
    """The rotation (rad) that turns the line (x0, y0) -> (x1, y1) onto the NEAREST axis
    direction (0, 90, 180 or -90 deg): at most +-45 deg."""
    if math.hypot(x1 - x0, y1 - y0) < 1e-6:
        raise RotateError("the reference line has no length")
    a = math.atan2(y1 - y0, x1 - x0)
    return wrap(round(a / (math.pi / 2)) * (math.pi / 2) - a)


def rotate_point(x: float, y: float, theta: float) -> tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    return c * x - s * y, s * x + c * y


def rotate_pose(x: float, y: float, yaw: float, theta: float) -> tuple[float, float, float]:
    rx, ry = rotate_point(x, y, theta)
    return rx, ry, wrap(yaw + theta)


def rotated_meta(grid: Grid, theta: float) -> tuple[GridMeta, int, int]:
    """The new axis-aligned geometry (meta, width, height) that holds the whole rotated grid."""
    m, res = grid.meta, grid.meta.resolution
    xs, ys = [], []
    for cx in (m.origin_x, m.origin_x + grid.width * res):
        for cy in (m.origin_y, m.origin_y + grid.height * res):
            rx, ry = rotate_point(cx, cy, theta)
            xs.append(rx)
            ys.append(ry)
    # snap the new origin to the resolution grid, so a 90 deg turn reproduces cells exactly
    ox = math.floor(min(xs) / res + 1e-9) * res
    oy = math.floor(min(ys) / res + 1e-9) * res
    w = int(math.ceil((max(xs) - ox) / res - 1e-9))
    h = int(math.ceil((max(ys) - oy) / res - 1e-9))
    return replace(m, origin_x=ox, origin_y=oy, origin_yaw=0.0), max(1, w), max(1, h)


def _resample(data: np.ndarray, src: GridMeta, dst: GridMeta, w: int, h: int, theta: float,
              outside: int) -> np.ndarray:
    res = src.resolution
    sh, sw = data.shape
    # inverse: the centre of every new cell, back into the old frame
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    xn = dst.origin_x + (cols + 0.5) * res
    yn = dst.origin_y + (rows + 0.5) * res
    c, s = math.cos(theta), math.sin(theta)
    xo, yo = c * xn + s * yn, -s * xn + c * yn  # R(-theta)
    co = np.floor((xo - src.origin_x) / res).astype(np.int64)
    ro = np.floor((yo - src.origin_y) / res).astype(np.int64)
    inside = (co >= 0) & (co < sw) & (ro >= 0) & (ro < sh)
    out = np.full((h, w), outside, dtype=data.dtype)
    out[inside] = data[ro[inside], co[inside]]
    # forward splat of the marked / occupied cells, so thin walls stay closed: 2 x 2 points
    # spread over each old cell, every new cell one of them lands in takes the old value
    r_occ, c_occ = np.nonzero(data >= 65)
    if r_occ.size:
        for du in (0.25, 0.75):
            for dv in (0.25, 0.75):
                x = src.origin_x + (c_occ + du) * res
                y = src.origin_y + (r_occ + dv) * res
                xr, yr = c * x - s * y, s * x + c * y
                cn = np.floor((xr - dst.origin_x) / res).astype(np.int64)
                rn = np.floor((yr - dst.origin_y) / res).astype(np.int64)
                ok = (cn >= 0) & (cn < w) & (rn >= 0) & (rn < h)
                out[rn[ok], cn[ok]] = data[r_occ[ok], c_occ[ok]]
    return out


def rotate_grid(grid: Grid, theta: float, outside: int = -1) -> Grid:
    """The grid turned by theta about the frame origin; `outside` fills what the old grid
    did not cover (-1 unknown for a map, 0 unmarked for a mask)."""
    theta = check_angle(theta)
    meta, w, h = rotated_meta(grid, theta)
    return Grid(_resample(grid.data, grid.meta, meta, w, h, theta, outside), meta)


def turn(grid: Grid, theta: float, outside: int = -1) -> Grid:
    """rotate_grid for a FRAME change (2026-11-12: a continued survey of a rotated revision maps
    in its pose graph's frame): any angle, no operator limits; ~0 returns the grid unchanged."""
    theta = wrap(float(theta))
    if abs(theta) < 1e-9:
        return grid
    meta, w, h = rotated_meta(grid, theta)
    return Grid(_resample(grid.data, grid.meta, meta, w, h, theta, outside), meta)


def rotate_like(mask: Grid, like: Grid, theta: float) -> Grid:
    """A mask aligned with the parent map, rotated onto `like` (the rotated map's geometry)."""
    data = _resample(mask.data, mask.meta, like.meta, like.width, like.height, check_angle(theta), 0)
    return Grid(data, like.meta)
