"""Pure-math checks for amr_base.diff_drive (spec T2: kinematics / wrap math)."""

import math

import pytest

from amr_base.diff_drive import (
    Geometry,
    OdomState,
    clamp_wheels,
    forward,
    integrate,
    inverse,
    slew,
    wrap_angle,
)

G = Geometry(wheel_radius_m=0.09, track_width_m=0.487)


def test_inverse_forward_roundtrip():
    for v, w in [(0.0, 0.0), (0.5, 0.0), (0.0, 1.0), (0.3, -0.7), (-0.2, 0.4)]:
        assert forward(G, *inverse(G, v, w)) == pytest.approx((v, w))


def test_inverse_signs():
    wl, wr = inverse(G, 0.0, 1.0)  # CCW: right wheel forward, left back
    assert wr > 0 > wl
    assert wr == pytest.approx(-wl)


def test_clamp_preserves_curvature():
    wl, wr = inverse(G, 1.0, 2.0)
    cl, cr = clamp_wheels(wl, wr, 5.0)
    assert max(abs(cl), abs(cr)) == pytest.approx(5.0)
    assert cl / cr == pytest.approx(wl / wr)
    assert clamp_wheels(1.0, -2.0, 5.0) == (1.0, -2.0)
    assert clamp_wheels(0.0, 0.0, 5.0) == (0.0, 0.0)


def test_slew():
    assert slew(0.0, 1.0, 0.5, 0.1) == pytest.approx(0.05)
    assert slew(1.0, 0.0, 0.5, 0.1) == pytest.approx(0.95)
    assert slew(0.99, 1.0, 0.5, 0.1) == 1.0
    assert slew(0.0, -1.0, 2.0, 1.0) == -1.0


def test_wrap_angle():
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
    assert wrap_angle(0.3) == pytest.approx(0.3)


def test_integrate_straight():
    s = integrate(OdomState(), G, 1.0, 1.0)
    assert (s.x, s.y, s.th) == pytest.approx((0.09, 0.0, 0.0))
    assert s.distance == pytest.approx(0.09)


def test_integrate_spin_in_place():
    d = (math.pi / 2) * G.track_width_m / G.wheel_radius_m / 2
    s = integrate(OdomState(), G, -d, d)
    assert (s.x, s.y) == pytest.approx((0.0, 0.0), abs=1e-12)
    assert s.th == pytest.approx(math.pi / 2)
    assert s.distance == 0.0


def test_integrate_square_returns_home():
    """4 x 5 m square in small steps: back at the origin, heading wrapped to 0."""
    s = OdomState()
    n = 2000
    for side in (4.0, 5.0, 4.0, 5.0):
        step = side / G.wheel_radius_m / n
        for _ in range(n):
            s = integrate(s, G, step, step)
        turn = (math.pi / 2) * G.track_width_m / G.wheel_radius_m / 2 / n
        for _ in range(n):
            s = integrate(s, G, -turn, turn)
    assert (s.x, s.y) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert s.th == pytest.approx(0.0, abs=1e-9)
    assert s.distance == pytest.approx(18.0)


def test_integrate_arc_midpoint_accuracy():
    """Constant-curvature arc, quarter circle radius 1 m: midpoint rule at 100 Hz-ish steps."""
    R = 1.0
    n = 200
    dth = (math.pi / 2) / n
    d_l = (R - G.track_width_m / 2) * dth / G.wheel_radius_m
    d_r = (R + G.track_width_m / 2) * dth / G.wheel_radius_m
    s = OdomState()
    for _ in range(n):
        s = integrate(s, G, d_l, d_r)
    assert (s.x, s.y) == pytest.approx((R, R), abs=1e-3)
    assert s.th == pytest.approx(math.pi / 2)


def test_slew_asym_starts_gently_and_stops_fast():
    from amr_base.diff_drive import slew_asym

    # accelerating from rest: limited by accel
    assert slew_asym(0.0, 0.3, 0.15, 0.5, 0.02) == pytest.approx(0.003)
    # stopping: limited by decel
    assert slew_asym(0.3, 0.0, 0.15, 0.5, 0.02) == pytest.approx(0.29)
    # reversing through zero counts as shrinking until zero, then growing
    assert slew_asym(0.1, -0.3, 0.15, 0.5, 0.02) == pytest.approx(0.09)
    assert slew_asym(0.0, -0.3, 0.15, 0.5, 0.02) == pytest.approx(-0.003)
    # reaching the target exactly
    assert slew_asym(0.299, 0.3, 0.15, 0.5, 0.02) == pytest.approx(0.3)


def test_scurve_builds_acceleration_gradually_and_never_overshoots():
    from amr_base.diff_drive import scurve

    dt, v, a, t, peak = 0.02, 0.0, 0.0, 0.0, 0.0
    trace = []
    while v < 0.5:
        v, a = scurve(v, a, 0.5, 0.3, 0.5, 1.0, dt)
        t += dt
        peak = max(peak, a)
        trace.append((t, v, a))
        assert v <= 0.5 + 1e-12
    # trapezoidal acceleration: 0.3 s to build 0.3 m/s^2 at 1 m/s^3, a plateau, a ramp-down;
    # the whole start takes ~1.9 s (a plain 0.3 m/s^2 ramp would take 1.67 s)
    assert peak == pytest.approx(0.3)
    assert trace[4][2] == pytest.approx(0.1, abs=0.011)  # 0.1 s in: a = 0.1, not 0.3
    assert 1.8 <= t <= 2.0
    assert trace[-1] == (pytest.approx(t), 0.5, 0.0)  # reaching the target zeroes the accel state
    # a stop uses the (faster) decel limit
    v, a, t = 0.5, 0.0, 0.0
    while v > 0.0:
        v, a = scurve(v, a, 0.0, 0.3, 0.5, 1.0, dt)
        t += dt
        assert v >= 0.0
    assert 1.3 <= t <= 1.6
    # a reversal passes through zero without a step
    v, a = 0.2, 0.0
    vs = []
    while v > -0.2:
        v, a = scurve(v, a, -0.2, 0.3, 0.5, 1.0, dt)
        vs.append(v)
    assert max(abs(x - y) for x, y in zip(vs, vs[1:], strict=False)) <= 0.5 * dt + 1e-9
    assert vs[-1] == -0.2
