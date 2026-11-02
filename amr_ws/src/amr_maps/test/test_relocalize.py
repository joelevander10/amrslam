"""Scan-to-map relocalisation for continue-survey: recovers a known pose from an offset hint,
refuses ambiguous corridors and scans that do not see the saved map."""

import math

import numpy as np
import pytest
from amr_maps.grid import Grid, GridMeta

from amr_maps import raycast
from amr_maps import relocalize as rl

RES = 0.05
LASER = (0.35, 0.0, 0.0)  # scanner ahead of base_footprint, as on the vehicle


def _grid(w_m: float, h_m: float, ox: float = -3.0, oy: float = -2.0) -> Grid:
    data = np.zeros((int(h_m / RES), int(w_m / RES)), dtype=np.int8)
    return Grid(data, GridMeta(RES, ox, oy))


def _box(g: Grid, x0, y0, x1, y1, v=100):
    r0, c0 = g.world_to_cell(x0, y0)
    r1, c1 = g.world_to_cell(x1, y1)
    g.data[max(r0, 0) : r1 + 1, max(c0, 0) : c1 + 1] = v


def hall() -> Grid:
    """30 x 20 m hall: outer walls, pillars on an irregular pitch, two partitions."""
    g = _grid(30.0, 20.0)
    ox, oy = g.meta.origin_x, g.meta.origin_y
    _box(g, ox, oy, ox + 30, oy + 0.15)
    _box(g, ox, oy + 19.85, ox + 30, oy + 20)
    _box(g, ox, oy, ox + 0.15, oy + 20)
    _box(g, ox + 29.85, oy, ox + 30, oy + 20)
    for i, x in enumerate((2.0, 6.5, 9.0, 14.5, 19.0, 21.5)):
        y = 3.0 if i % 2 else 9.5
        _box(g, x, y, x + 0.4, y + 0.4)
    _box(g, 11.0, 12.0, 11.15, 17.85)  # partition
    _box(g, 16.0, -2.0, 16.15, 1.5)
    return g


def corridor() -> Grid:
    """Two parallel walls 70 m long and nothing else: position along it is undetermined."""
    g = _grid(70.0, 6.0, ox=-35.0, oy=-3.0)
    _box(g, -35.0, -2.0, 35.0, -1.85)
    _box(g, -35.0, 1.85, 35.0, 2.0)
    return g


def scan_at(g: Grid, pose, geom=None):
    geom = geom or raycast.mounted()
    x, y, yaw = pose
    lx = x + math.cos(yaw) * LASER[0] - math.sin(yaw) * LASER[1]
    ly = y + math.sin(yaw) * LASER[0] + math.cos(yaw) * LASER[1]
    ranges = raycast.cast(g, lx, ly, yaw + LASER[2], geom)
    return rl.scan_points(ranges, geom.angle_min, geom.angle_increment, geom.range_min, geom.range_max, LASER)


@pytest.mark.parametrize(
    "true, hint",
    [
        ((5.0, 6.0, 0.3), (5.8, 5.4, 0.3 + math.radians(18))),  # 1 m off, 18 deg off
        ((20.0, 14.0, -2.6), (19.0, 14.6, -2.6 - math.radians(25))),
        ((8.0, 2.0, 1.4), (8.0, 2.0, 1.4)),  # exact hint stays put
    ],
)
def test_recovers_the_pose_from_an_offset_hint(true, hint):
    g = hall()
    r = rl.match(g, scan_at(g, true), hint)
    assert r.ok, r.reason
    assert math.hypot(r.x - true[0], r.y - true[1]) <= 0.08
    assert abs(math.degrees(rl.wrap(r.yaw - true[2]))) <= 1.0
    assert r.score > 0.8 and r.score >= r.hint_score
    assert r.runner_up < 0.9 * r.score


def test_corridor_is_refused_as_ambiguous():
    g = corridor()
    r = rl.match(g, scan_at(g, (0.0, 0.0, 0.0), raycast.mounted(range_max=20.0)), (0.6, 0.1, 0.05))
    assert not r.ok and r.reason.startswith("ambiguous")


def test_scan_of_unmapped_space_is_refused():
    g = hall()
    other = _grid(30.0, 20.0)  # a different place: one diagonal wall
    for i in range(200):
        _box(other, 2.0 + i * 0.05, 1.0 + i * 0.04, 2.1 + i * 0.05, 1.1 + i * 0.04)
    r = rl.match(g, scan_at(other, (8.0, 6.0, 0.0)), (8.0, 6.0, 0.0))
    assert not r.ok and "poorly" in r.reason


def test_hint_outside_the_window_is_not_found():
    g = hall()
    r = rl.match(g, scan_at(g, (5.0, 6.0, 0.3)), (9.0, 6.0, 0.3))  # 4 m off > 1.5 m window
    assert not (r.ok and math.hypot(r.x - 5.0, r.y - 6.0) < 0.1)


def test_too_few_returns_and_bad_hint_are_refused():
    g = hall()
    r = rl.match(g, np.zeros((5, 2)), (5.0, 6.0, 0.0))
    assert not r.ok and "scan returns" in r.reason
    r = rl.match(g, scan_at(g, (5.0, 6.0, 0.0)), (float("nan"), 6.0, 0.0))
    assert not r.ok and "finite" in r.reason


def test_scan_points_drop_invalid_returns_and_apply_the_laser_mount():
    pts = rl.scan_points([1.0, float("inf"), 0.01, 2.0], 0.0, math.pi / 2, 0.05, 30.0, (0.35, 0.0, 0.0))
    assert pts.shape == (2, 2)
    assert pts[0] == pytest.approx([1.35, 0.0])
    assert pts[1] == pytest.approx([0.35, -2.0], abs=1e-9)  # beam 3 at 270 deg


def test_proximity_field_is_one_on_walls_and_zero_beyond_tolerance():
    g = _grid(2.0, 2.0)
    r, c = 10, 10
    g.data[r, c] = 100  # one occupied cell
    f = rl.proximity(g, 0.15)  # 3 cells
    assert f[r, c] == 1.0
    assert f[r, c + 1] == pytest.approx(0.75) and f[r + 3, c - 3] == pytest.approx(0.25)
    assert f[r, c + 4] == 0.0
