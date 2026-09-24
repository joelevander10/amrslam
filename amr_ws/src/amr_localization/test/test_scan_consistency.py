"""A mesh cage: a partially transparent fence 3 m ahead, a solid wall 5 m ahead."""

import math

import numpy as np
from amr_maps.grid import Grid, GridMeta
from amr_maps.raycast import ScanGeometry, cast

from amr_localization import scan_consistency as sc

RES = 0.05
TOL = 0.15


def cage() -> Grid:
    # 12 m x 12 m, origin at (-6, -6); laser at (0, 0) facing +x
    data = np.zeros((240, 240), dtype=np.int8)
    col = lambda x: int((x + 6.0) / RES)  # noqa: E731
    data[:, col(5.0)] = 100  # solid wall x = 5
    data[::2, col(3.0)] = 100  # mesh fence x = 3: every other cell (the survey saw it half the time)
    data[:, col(-4.0)] = 100  # wall behind
    return Grid(data, GridMeta(RES, -6.0, -6.0))


GEOM = ScanGeometry(math.radians(-60), math.radians(60), 241, 0.1, 12.0)


def test_beams_through_the_mesh_onto_the_wall_are_not_long():
    g = cage()
    near = sc.occupied_near(g, TOL)
    # the scanner sees THROUGH the mesh to the solid wall on every beam
    solid = g.data.copy()
    solid[:, int((3.0 + 6.0) / RES)] = 0  # what the scanner actually returns: wall at x=5 only
    ranges = cast(Grid(solid, g.meta), 0.0, 0.0, 0.0, GEOM)
    match, long = sc.compare(g, near, 0.0, 0.0, 0.0, ranges, GEOM, TOL)
    assert long == 0.0
    assert match > 0.95


def test_wrong_pose_puts_endpoints_in_free_space_and_counts_long():
    g = cage()
    near = sc.occupied_near(g, TOL)
    solid = g.data.copy()
    solid[:, int((3.0 + 6.0) / RES)] = 0
    ranges = cast(Grid(solid, g.meta), 0.0, 0.0, 0.0, GEOM)  # truth: wall at 5 m ahead
    # AMCL believes the laser is 1.5 m further back: the measured endpoints land 1.5 m short
    # of the wall, in mapped free space, on beams the map expects to reach the fence first
    match, long = sc.compare(g, near, -1.5, 0.0, 0.0, ranges, GEOM, TOL)
    assert long > 0.5
    assert match < 0.2


def test_no_return_and_unknown_endpoints_prove_nothing():
    g = cage()
    g.data[:, int((5.0 + 6.0) / RES) + 1 :] = -1  # beyond the wall is unknown
    near = sc.occupied_near(g, TOL)
    ranges = np.full(GEOM.beams, np.inf)  # nothing returned at all
    match, long = sc.compare(g, near, 0.0, 0.0, 0.0, ranges, GEOM, TOL)
    assert long == 0.0 and match == 0.0


def test_beams_into_dynamic_areas_move_neither_fraction():
    """dynamic-mapping plan §1.3: a mapped trolley that moved within its dynamic area puts
    endpoints in mapped free space behind where the map expects it (long) and short of the
    wall where it now stands (no match). Neither says anything about the pose."""
    g = cage()
    g.data[:, int((3.0 + 6.0) / RES)] = 0  # no mesh: a plain wall 5 m ahead
    col = lambda x: int((x + 6.0) / RES)  # noqa: E731
    row = lambda y: int((y + 6.0) / RES)  # noqa: E731
    g.data[row(-1.0) : row(1.0), col(2.0) : col(2.2)] = 100  # the trolley as surveyed, 2 m ahead
    near = sc.occupied_near(g, TOL)
    now = g.data.copy()
    now[row(-1.0) : row(1.0), col(2.0) : col(2.2)] = 0
    now[row(-1.0) : row(1.0), col(2.6) : col(2.8)] = 100  # pushed 0.6 m further today
    ranges = cast(Grid(now, g.meta), 0.0, 0.0, 0.0, GEOM)
    match_plain, long_plain = sc.compare(g, near, 0.0, 0.0, 0.0, ranges, GEOM, TOL)
    assert long_plain > 0.1 and match_plain < 0.9  # without the mask the trolley looks like a pose error
    dyn = np.zeros(g.data.shape, dtype=bool)
    dyn[row(-1.2) : row(1.2), col(1.8) : col(3.0)] = True
    match, long = sc.compare(g, near, 0.0, 0.0, 0.0, ranges, GEOM, TOL, dyn)
    assert long == 0.0 and match > 0.95
    # the same scan with no mask argument is the old behaviour exactly
    assert sc.compare(g, near, 0.0, 0.0, 0.0, ranges, GEOM, TOL, None) == (match_plain, long_plain)
