"""Preset survey moves: bounds, the stop profile against a simple vehicle, aborts, SLAM check."""

import math

import pytest

from amr_mission import survey_move as sm


def drive(ctl, pose=(0.0, 0.0, 0.0), dt=0.05, lag=0.1, t_max=120.0, slip=None):
    """First-order vehicle (time constant `lag`) under the controller; returns
    (pose, t, reason, peak v, peak w)."""
    x, y, yaw = pose
    v = w = 0.0
    t = 0.0
    ctl.start((x, y, yaw), t)
    peak_v = peak_w = 0.0
    while t < t_max:
        cv, cw, done, reason = ctl.step((x, y, yaw), t)
        if done:
            return (x, y, yaw), t, reason, peak_v, peak_w
        k = dt / (lag + dt)
        v += k * (cv - v)
        w += k * (cw - w)
        if slip is not None:
            v, w = slip(t, v, w)
        peak_v, peak_w = max(peak_v, abs(v)), max(peak_w, abs(w))
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = sm.wrap(yaw + w * dt)
        t += dt
    raise AssertionError("never finished")


def test_bounds():
    assert sm.Move.parse("straight", 2.0).target == 2.0
    assert sm.Move.parse("straight", -1.5).label() == "reverse 1.50 m"
    assert sm.Move.parse("rotate", -90).target == pytest.approx(-math.pi / 2)
    assert sm.Move.parse("rotate", 45).label() == "spin left 45 deg"
    assert sm.Move.parse("rotate", -90).label() == "spin right 90 deg"
    for kind, value in (
        ("straight", 0.01),
        ("straight", 10.5),
        ("rotate", 30),
        ("rotate", 270),
        ("strafe", 1.0),
        ("straight", float("nan")),
    ):
        with pytest.raises(sm.MoveError):
            sm.Move.parse(kind, value)


def test_straight_arrives_within_a_centimetre_at_the_survey_speed():
    ctl = sm.MoveController(sm.Move.parse("straight", 3.0))
    (x, y, yaw), t, reason, peak_v, _ = drive(ctl, pose=(1.0, 2.0, math.pi / 2))
    assert reason == "" and abs(ctl.achieved() - 3.0) <= 0.011
    assert (x, y) == (pytest.approx(1.0, abs=0.01), pytest.approx(5.0, abs=0.011))
    assert peak_v <= 0.30 + 1e-9 and peak_v > 0.28
    assert t < ctl.nominal_s + 3.0


def test_reverse_and_heading_hold():
    ctl = sm.MoveController(sm.Move.parse("straight", -1.0))
    # a disturbance turns the vehicle while it backs up; the hold brings the heading back
    (x, y, yaw), _, reason, _, _ = drive(ctl, slip=lambda t, v, w: (v, w + (0.05 if t < 1.0 else 0.0)))
    assert reason == "" and ctl.achieved() == pytest.approx(-1.0, abs=0.011)
    assert abs(math.degrees(yaw)) < 1.5


@pytest.mark.parametrize("deg", [45, -90, 135, 180, -180])
def test_spins_arrive_within_half_a_degree_at_0_2_rad_s(deg):
    ctl = sm.MoveController(sm.Move.parse("rotate", deg))
    (_, _, yaw), _, reason, peak_v, peak_w = drive(ctl, pose=(0.0, 0.0, 3.0))
    assert reason == "" and math.degrees(ctl.achieved()) == pytest.approx(deg, abs=0.55)
    assert peak_v == 0.0 and peak_w <= 0.20 + 1e-9  # unwrapped: a 180 through +-pi is not ambiguous


def test_stall_and_timeout_abort():
    ctl = sm.MoveController(sm.Move.parse("straight", 2.0))
    _, t, reason, _, _ = drive(ctl, slip=lambda t, v, w: (0.0, 0.0) if t > 1.0 else (v, w))
    assert reason.startswith("no progress") and t < 5.0
    slow = sm.MoveController(sm.Move.parse("rotate", 90))
    _, _, reason, _, _ = drive(slow, slip=lambda t, v, w: (v, w * 0.3 if w > 0.01 else w))
    assert reason == "took too long"  # 90 deg at 0.06 rad/s: 26 s > 2 x 9.9 s + 5 s


def test_slam_correction_is_measured_at_the_vehicle():
    base = (20.0, 0.0, 0.0)  # far from the odom origin
    m, rad = sm.slam_correction((0.0, 0.0, 0.0), (0.0, 0.0, math.radians(1.0)), base)
    assert math.degrees(rad) == pytest.approx(1.0)
    assert m == pytest.approx(20.0 * math.radians(1.0), rel=0.01)  # a frame yaw moves the robot
    m, rad = sm.slam_correction((1.0, 1.0, 0.2), (1.0, 1.05, 0.2), base)
    assert m == pytest.approx(0.05) and rad == pytest.approx(0.0)


def test_verdict_thresholds():
    straight = sm.Move.parse("straight", 5.0)
    assert sm.check_verdict(straight, 0.12, math.radians(0.5))[0]  # 3 % of 5 m = 0.15 m
    ok, text = sm.check_verdict(straight, 0.05, math.radians(2.5))
    assert not ok and "slipped" in text
    spin = sm.Move.parse("rotate", 90)
    assert not sm.check_verdict(spin, 0.11, 0.0)[0]
    assert sm.check_verdict(spin, 0.02, math.radians(1.9))[0]
