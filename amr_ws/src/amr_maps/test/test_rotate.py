"""amr_maps.rotate: straighten a map onto a reference line (2026-11-05)."""

import math

import numpy as np
import pytest
from amr_maps.grid import Grid, GridMeta

from amr_maps import rotate as rot

D = math.radians


def grid_with_wall(angle_deg: float, res=0.05, n=200):
    """A 10 x 10 m unknown map with a free disc and a 1-cell wall at angle_deg through (0, 0)."""
    m = GridMeta(res, -5.0, -5.0)
    data = np.full((n, n), -1, dtype=np.int8)
    rows, cols = np.mgrid[0:n, 0:n]
    x, y = -5.0 + (cols + 0.5) * res, -5.0 + (rows + 0.5) * res
    data[x * x + y * y < 16.0] = 0
    a = D(angle_deg)
    for t in np.arange(-3.5, 3.5, res / 4):
        r, c = int((t * math.sin(a) + 5.0) / res), int((t * math.cos(a) + 5.0) / res)
        data[r, c] = 100
    return Grid(data, m)


def test_align_angle_turns_a_line_onto_the_nearest_axis():
    assert math.isclose(rot.align_angle(0, 0, 1, math.tan(D(7))), -D(7), abs_tol=1e-12)
    assert math.isclose(rot.align_angle(0, 0, -math.tan(D(4)), 1), -D(4), abs_tol=1e-12)  # near +Y
    assert math.isclose(rot.align_angle(1, 0, 0, math.tan(D(3))), D(3), abs_tol=1e-12)    # drawn backwards
    with pytest.raises(rot.RotateError):
        rot.align_angle(1, 1, 1, 1)


def test_a_rotated_wall_lies_on_the_axis_and_stays_closed():
    g = grid_with_wall(7.0)
    out = rot.rotate_grid(g, rot.align_angle(0, 0, math.cos(D(7)), math.sin(D(7))))
    rows, cols = np.nonzero(out.data >= 65)
    y = out.meta.origin_y + (rows + 0.5) * out.meta.resolution
    x = out.meta.origin_x + (cols + 0.5) * out.meta.resolution
    assert np.abs(y).max() <= 1.5 * out.meta.resolution, "the wall now runs along X"
    xs = np.unique(cols[np.abs(x) < 3.3])
    assert np.all(np.diff(xs) == 1), "no free gap in the wall"
    assert out.meta.origin_yaw == 0.0


def test_the_whole_old_map_fits_and_the_outside_is_unknown():
    g = grid_with_wall(30.0)
    out = rot.rotate_grid(g, D(30))
    assert out.width > g.width and out.height > g.height  # the turned square needs a bigger box
    assert out.data[0, 0] == -1 and (out.data == 0).sum() == pytest.approx((g.data == 0).sum(), rel=0.03)


def test_a_quarter_turn_moves_every_cell_exactly():
    g = grid_with_wall(0.0)
    out = rot.rotate_grid(g, D(90))
    assert out.width == g.height and out.height == g.width
    assert np.array_equal(out.data, np.rot90(g.data, -1))  # row 0 = bottom: ccw in the map is cw in the array


def test_poses_and_masks_turn_with_the_map():
    x, y, yaw = rot.rotate_pose(1.0, 0.0, 0.0, D(90))
    assert (round(x, 9), round(y, 9), round(yaw, 9)) == (0.0, 1.0, round(D(90), 9))
    g = grid_with_wall(10.0)
    mask = Grid(np.where(g.data >= 65, 100, 0).astype(np.int8), g.meta)
    out = rot.rotate_grid(g, -D(10))
    m2 = rot.rotate_like(mask, out, -D(10))
    assert m2.data.shape == out.data.shape and m2.meta == out.meta
    assert np.array_equal(m2.data >= 65, out.data >= 65)


def test_angle_bounds():
    for bad in (0.0, D(0.01), D(181), float("nan"), True):
        with pytest.raises(rot.RotateError):
            rot.check_angle(bad)
