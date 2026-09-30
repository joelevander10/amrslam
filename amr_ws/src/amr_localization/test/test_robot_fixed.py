"""Robot-fixed filter: what turns with the vehicle goes, the world stays."""

import math

from amr_localization.robot_fixed import RobotFixedFilter

N = 360  # 1 deg beams, index = bearing in the scanner frame (0 = +x)


def world_scan(yaw: float, pos=(0.0, 0.0), phantom: dict[int, float] | None = None) -> list[float]:
    """A square room 4 m x 4 m around the origin seen from pos, plus robot-fixed phantoms."""
    out = []
    for i in range(N):
        a = yaw + math.radians(i)
        c, s = math.cos(a), math.sin(a)
        t = min(
            ((2.0 - pos[0]) / c) if c > 1e-9 else math.inf,
            ((-2.0 - pos[0]) / c) if c < -1e-9 else math.inf,
            ((2.0 - pos[1]) / s) if s > 1e-9 else math.inf,
            ((-2.0 - pos[1]) / s) if s < -1e-9 else math.inf,
        )
        out.append(t)
    for i, r in (phantom or {}).items():
        out[i] = r
    return out


def spin(f, degrees, phantom=None, pos=(0.0, 0.0), step=2.0, t0=0.0):
    out, n = None, 0
    for k in range(int(degrees / step) + 1):
        yaw = math.radians(k * step)
        out, n = f.update(world_scan(yaw, pos, phantom), 0.1, 30.0, yaw, t0 + 0.1 * k)
    return out, n


def test_phantom_turning_with_the_vehicle_is_learnt_and_dropped():
    f = RobotFixedFilter()
    ph = {5: 0.30, 6: 0.31, 7: 0.30}
    out, n = spin(f, 100, ph)
    assert set(f.flagged) == {5, 6, 7}
    assert n == 3 and all(math.isnan(out[i]) for i in ph)
    assert sum(math.isnan(x) for x in out) == 3  # every wall reading survives


def test_walls_are_never_flagged_by_turning():
    f = RobotFixedFilter()
    for pos in ((0.0, 0.0), (1.2, 0.5), (-1.5, -1.5)):  # centre, off-centre, a corner
        spin(f, 360, pos=pos)
        assert f.flagged == {}


def test_driving_straight_along_a_wall_flags_nothing():
    # every beam onto a parallel wall keeps its range while driving straight: must NOT count
    f = RobotFixedFilter()
    for k in range(600):  # 1.2 m along the walls, with heading jitter
        yaw = math.radians(0.3 * (-1) ** k)
        f.update(world_scan(yaw, (k * 0.002 - 0.6, 0.0)), 0.1, 30.0, yaw, 0.1 * k)
    assert f.flagged == {}


def test_a_real_object_on_a_flagged_beam_still_passes():
    f = RobotFixedFilter()
    spin(f, 100, {5: 0.30})
    yaw = math.radians(100.0)
    scan = world_scan(yaw)
    scan[5] = 1.2  # something real beyond the phantom's range
    out, n = f.update(scan, 0.1, 30.0, yaw, 10.0)
    assert out[5] == 1.2 and n == 0


def test_flag_lapses_once_the_return_is_gone():
    f = RobotFixedFilter(unflag_s=5.0)
    spin(f, 100, {5: 0.30})
    assert 5 in f.flagged
    yaw = math.radians(100.0)
    f.update(world_scan(yaw), 0.1, 30.0, yaw, 100.0)  # cover replaced: 5 now sees the wall
    assert 5 not in f.flagged


def test_a_flickering_phantom_is_still_learnt():
    f = RobotFixedFilter()
    for k in range(60):
        yaw = math.radians(k * 2.0)
        ph = {5: 0.30} if k % 3 else {}  # missing every third scan
        f.update(world_scan(yaw, phantom=ph), 0.1, 30.0, yaw, 0.1 * k)
    assert 5 in f.flagged


def test_sectors_summary():
    f = RobotFixedFilter()
    spin(f, 100, {5: 0.30, 6: 0.31, 40: 0.5})
    secs = f.sectors(0.0, math.radians(1.0))
    assert [(round(a), round(b)) for a, b, _ in secs] == [(5, 6), (40, 40)]


def test_a_heading_jump_is_not_a_turn():
    # AMCL/EKF relocalising: yaw jumps 100 deg between two scans - no evidence from that
    f = RobotFixedFilter()
    f.update(world_scan(0.0, phantom={5: 0.3}), 0.1, 30.0, 0.0, 0.0)
    f.update(world_scan(0.0, phantom={5: 0.3}), 0.1, 30.0, math.radians(100.0), 0.1)
    assert f.flagged == {}


def test_learnt_flags_survive_a_restart_on_the_same_geometry_only():
    f = RobotFixedFilter()
    spin(f, 100, {5: 0.30, 6: 0.31})
    d = f.to_dict(0.0, math.radians(1.0))
    g = RobotFixedFilter()
    assert g.load_dict(d, N, 0.0, math.radians(1.0), 0.0) == 2
    out, n = g.update(world_scan(0.0, phantom={5: 0.30, 6: 0.31}), 0.1, 30.0, 0.0, 0.1)
    assert n == 2 and math.isnan(out[5]) and math.isnan(out[6])  # filtered from the first scan
    h = RobotFixedFilter()
    assert h.load_dict(d, N + 1, 0.0, math.radians(1.0), 0.0) == 0  # another scanner config
    assert h.load_dict(d, N, 0.1, math.radians(1.0), 0.0) == 0
    assert h.load_dict(None, N, 0.0, math.radians(1.0), 0.0) == 0
