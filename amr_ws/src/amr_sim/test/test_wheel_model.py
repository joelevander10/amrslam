"""WheelModel: ramp, watchdog, slip, ground truth (spec test 6 at unit level)."""

import pytest

from amr_base.diff_drive import Geometry
from amr_sim.wheel_model import WheelModel

G = Geometry(0.09, 0.487)


def make(**kw) -> WheelModel:
    return WheelModel(geom=G, wheel_accel_rad_s2=100.0, cmd_timeout_s=0.25, **kw)


def run(m: WheelModel, t0: float, seconds: float, dt: float = 0.01) -> float:
    t = t0
    for _ in range(int(seconds / dt)):
        t += dt
        m.step(t, dt)
    return t


def test_no_command_means_no_motion():
    m = make()
    run(m, 0.0, 1.0)
    assert m.actual_l == m.actual_r == 0.0
    assert m.truth.distance == 0.0
    assert m.timed_out


def test_ramp_limits_wheel_accel():
    m = WheelModel(geom=G, wheel_accel_rad_s2=10.0, cmd_timeout_s=1.0)
    m.command(5.0, 5.0, 0.0)
    m.step(0.1, 0.1)
    assert m.actual_l == pytest.approx(1.0)
    run(m, 0.1, 0.5)
    assert m.actual_l == pytest.approx(5.0)


def test_watchdog_zeroes_within_timeout():
    m = make()
    m.command(5.0, 5.0, 0.0)
    run(m, 0.0, 0.2)
    assert m.actual_l == pytest.approx(5.0)
    assert not m.timed_out
    t = run(m, 0.2, 0.06)  # crosses 0.25 s since last command
    assert m.timed_out
    assert m.target_l == m.target_r == 0.0
    run(m, t, 0.1)
    assert m.actual_l == m.actual_r == 0.0


def test_truth_and_reported_agree_without_slip():
    m = make()
    t = 0.0
    for _ in range(100):
        t += 0.01
        m.command(2.0, 2.0, t)
        m.step(t, 0.01)
    assert m.pos_l == pytest.approx(m.pos_r)
    assert m.truth.x == pytest.approx(m.pos_l * G.wheel_radius_m)
    assert m.truth.y == pytest.approx(0.0)


def test_slip_makes_reported_differ_from_truth():
    m = make(slip_noise_std=0.05, seed=1)
    t = 0.0
    for _ in range(200):
        t += 0.01
        m.command(2.0, 2.0, t)
        m.step(t, 0.01)
    assert m.pos_l != pytest.approx(m.truth.x / G.wheel_radius_m, rel=1e-6)
    assert abs(m.pos_l * G.wheel_radius_m - m.truth.x) / m.truth.x < 0.05


def test_deterministic_seed():
    a, b = make(slip_noise_std=0.05, seed=7), make(slip_noise_std=0.05, seed=7)
    for m in (a, b):
        t = 0.0
        for _ in range(50):
            t += 0.01
            m.command(1.0, 3.0, t)
            m.step(t, 0.01)
    assert (a.pos_l, a.pos_r) == (b.pos_l, b.pos_r)


def test_teleport_moves_truth_not_odometry():
    m = make()
    t = 0.0
    for _ in range(100):
        t += 0.01
        m.command(2.0, 2.0, t)
        m.step(t, 0.01)
    pos_before = (m.pos_l, m.pos_r)
    m.teleport(5.0, -1.0, 1.0)
    assert (m.truth.x, m.truth.y, m.truth.th) == (5.0, -1.0, 1.0)
    assert (m.pos_l, m.pos_r) == pos_before
