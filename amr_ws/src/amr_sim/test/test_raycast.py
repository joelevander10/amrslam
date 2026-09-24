import math
import time

import numpy as np
import pytest
from amr_maps.generate_sim_factory import build
from amr_maps.grid import Grid, GridMeta
from amr_maps.raycast import ScanGeometry, cast, mounted, nanoscan3


def box_world(size_m: float = 10.0, res: float = 0.05) -> Grid:
    """Empty square room centred on the origin with 1-cell walls."""
    n = int(size_m / res)
    data = np.zeros((n, n), dtype=np.int8)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    return Grid(data, GridMeta(res, -size_m / 2, -size_m / 2))


def test_forward_beam_hits_wall_at_known_range():
    g = box_world()
    geom = ScanGeometry(0.0, 0.0, 1, 0.05, 20.0)
    r = cast(g, 0.0, 0.0, 0.0, geom)
    # Wall cell spans x in [4.95, 5.0); first sample inside it, step 0.025.
    assert r[0] == pytest.approx(4.95, abs=0.03)


def test_yaw_rotates_the_scan():
    g = box_world()
    geom = ScanGeometry(0.0, 0.0, 1, 0.05, 20.0)
    assert cast(g, 2.0, 0.0, 0.0, geom)[0] == pytest.approx(2.95, abs=0.03)
    assert cast(g, 2.0, 0.0, math.pi, geom)[0] == pytest.approx(6.95, abs=0.03)
    assert cast(g, 2.0, 0.0, math.pi / 2, geom)[0] == pytest.approx(4.95, abs=0.03)


def test_no_return_is_inf():
    g = Grid(np.zeros((100, 100), dtype=np.int8), GridMeta(0.05, -2.5, -2.5))
    geom = ScanGeometry(0.0, 0.0, 1, 0.05, 20.0)
    assert np.isinf(cast(g, 0.0, 0.0, 0.0, geom)[0])


def test_nanoscan3_geometry():
    geom = nanoscan3()
    assert math.degrees(geom.angle_max - geom.angle_min) == pytest.approx(275.0)
    assert geom.beams == 551
    assert math.degrees(geom.angle_increment) == pytest.approx(0.5)
    full = nanoscan3(beams=1652)
    assert math.degrees(full.angle_increment) == pytest.approx(275.0 / 1651)


def test_dead_sector_faces_backwards():
    g = box_world()
    geom = nanoscan3(beams=551)
    r = cast(g, 0.0, 0.0, 0.0, geom)
    # Straight ahead (middle beam) sees the east wall at 5 m; there is no beam at 180 deg.
    assert r[275] == pytest.approx(4.95, abs=0.03)
    assert abs(geom.angles).max() < math.pi


def test_sim_factory_scan_from_start_mark():
    g = build()
    r = cast(g, 0.964, 0.0, 0.0, nanoscan3())
    assert np.isfinite(r).all(), "the start mark is enclosed; every beam must return"
    assert r[275] == pytest.approx(26.8 - 0.964, abs=0.05)  # east wall inner face at x = 26.8
    # From the mark the racks (x >= 3) are not yet alongside: north sees the wall at y = 9.8.
    north = int(round((math.pi / 2 - nanoscan3().angle_min) / nanoscan3().angle_increment))
    assert r[north] == pytest.approx(9.8, abs=0.05)
    # Inside the aisle, north sees rack row C's face at y = 1.8.
    r = cast(g, 8.0, 0.0, 0.0, nanoscan3())
    assert r[north] == pytest.approx(1.8, abs=0.05)


def test_speed_budget():
    """Best of 10: the algorithm's cost, not the machine's load (colcon may run other sims)."""
    g = build()
    geom = nanoscan3()
    cast(g, 0.0, 0.0, 0.0, geom)
    best = min(_timed(lambda: cast(g, 0.0, 0.0, 0.3, geom)) for _ in range(10))
    assert best < 0.10, f"{best * 1000:.1f} ms per 551-beam scan at 30 m"


def _timed(fn) -> float:
    t0 = time.perf_counter()
    fn()
    return time.perf_counter() - t0


def test_mounted_window():
    g = mounted()
    assert math.degrees(g.angle_max - g.angle_min) == pytest.approx(190.0)
    assert math.degrees(g.angle_increment) == pytest.approx(0.5)
    r = cast(box_world(), 0.0, 0.0, 0.0, g)
    assert r[190] == pytest.approx(4.95, abs=0.03)  # middle beam straight ahead
