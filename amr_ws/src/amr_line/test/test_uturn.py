"""amr_line.uturn: the pivot ends on the tape, centred - not on an encoder angle.

A small geometric world: the vehicle pivots about its axle, which stopped on the
tape line; the sensor sits SENSOR_LOOKAHEAD_M ahead, so the tape behind crosses
it at lateral offset Ls * tan(theta - 180 deg) and is visible only within the
sensor's +/-100 mm. Mechanism checks only - tolerances are settled on the floor.
"""
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_line")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from agv_core import config, kinematics  # noqa: E402

from amr_line import uturn  # noqa: E402

DT = 0.02
CPR = 10000.0 * config.GEAR_RATIO          # counts per wheel turn
START_LEVEL = 6


class Pivot:
    """True body angle in the commanded direction; counts from the commands."""

    def __init__(self, direction, slip=1.0, start_deg=0.0):
        self.direction = direction
        self.sign = -1.0 if direction == "cw" else 1.0
        self.theta = start_deg              # true angle, degrees, commanded sense
        self.slip = slip                    # encoder degrees per true degree
        self.counts = [0, 0]
        self.u = uturn.UTurn(direction, CPR, tuple(self.counts), START_LEVEL)
        self.dropouts = set()               # ticks with no reading regardless

    def reading(self, tick):
        """e in the follower's sign: rotating in the commanded sense drives it to 0."""
        off = math.radians(self.theta - 180.0)
        if tick in self.dropouts or abs(off) >= math.radians(80):
            return None
        e = self.sign * 1000.0 * config.SENSOR_LOOKAHEAD_M * math.tan(off)
        return e if abs(e) <= config.SENSOR_MAX_MM else None

    def step(self, left, right):
        _, omega = kinematics.wheels_to_body(left, right)      # rad/s, CCW positive
        d_true = math.degrees(omega * DT) * self.sign
        self.theta += d_true
        arc = config.TRACK_M / 2.0 * math.radians(d_true * self.slip) * self.sign
        c = arc / (math.pi * config.WHEEL_DIA_M) * CPR
        dl, dr = -c, c
        if config.INVERT_LEFT:
            dl = -dl
        if config.INVERT_RIGHT:
            dr = -dr
        self.counts[0] += int(round(dl))
        self.counts[1] += int(round(dr))

    def run(self, seconds=60.0):
        for tick in range(int(seconds / DT)):
            e = self.reading(tick)
            left, right = self.u.update(tuple(self.counts), e, START_LEVEL if e is not None else None, DT)
            if not self.u.active:
                return tick
            self.step(left, right)
        return None


def centred(p):
    e = p.reading(-1)
    return e is not None and abs(e) <= config.U_TURN_CENTER_TOL_MM


def test_the_sign_convention_of_the_world():
    p = Pivot("cw", start_deg=160.0)
    e = p.reading(0)
    left, right = uturn.UTurn("cw", CPR, (0, 0), START_LEVEL, phase=uturn.CENTER).update((0, 0), e, 6, DT)
    _, omega = kinematics.wheels_to_body(left, right)
    assert omega < 0, "short of 180 on a cw turn, centring keeps turning cw"


def test_cw_and_ccw_end_centred_near_180():
    for direction in ("cw", "ccw"):
        p = Pivot(direction)
        assert p.run() is not None, (direction, p.u.reason)
        assert p.u.phase == uturn.DONE, (direction, p.u.reason)
        assert centred(p), (direction, p.reading(-1))
        tol = math.degrees(math.atan(config.U_TURN_CENTER_TOL_MM / 1000.0 / config.SENSOR_LOOKAHEAD_M))
        assert abs(p.theta - 180.0) <= tol + 0.5, (direction, p.theta)


def test_encoder_over_reading_does_not_stop_it_at_the_sensor_edge():
    """Field run 2026-10-08: the gate opened with the tape at the sensor edge and the
    pivot stalled at -101 mm. Wheel slip makes the encoder read high: 150 deg by
    encoder is ~134 deg true, where the tape is only just in view."""
    p = Pivot("cw", slip=1.12)
    assert p.run() is not None and p.u.phase == uturn.DONE, p.u.reason
    assert centred(p), p.reading(-1)


def test_a_reading_dropout_while_centring_keeps_turning_toward_the_tape():
    p = Pivot("cw")
    # Find when centring starts, then blank the reading for 0.3 s from there.
    probe = Pivot("cw")
    for tick in range(5000):
        e = probe.reading(tick)
        left, right = probe.u.update(tuple(probe.counts), e, 6 if e is not None else None, DT)
        if probe.u.phase == uturn.CENTER:
            break
        probe.step(left, right)
    p.dropouts = set(range(tick + 1, tick + 16))
    assert p.run() is not None and p.u.phase == uturn.DONE, p.u.reason
    assert centred(p)


def test_centring_never_stalls_on_a_vanishing_command():
    u = uturn.UTurn("cw", CPR, (0, 0), START_LEVEL, phase=uturn.CENTER)
    left, right = u.update((0, 0), config.U_TURN_CENTER_TOL_MM + 1.0, 6, DT)
    _, omega = kinematics.wheels_to_body(left, right)
    floor = uturn.spin_omega(uturn.CENTER_MIN_FRACTION * config.AUTO_U_TURN_RPM)
    assert abs(omega) >= floor - 1e-9


def test_a_tape_that_never_comes_back_while_centring_fails_by_angle():
    p = Pivot("cw", start_deg=175.0)
    p.u = uturn.UTurn("cw", CPR, tuple(p.counts), START_LEVEL, phase=uturn.CENTER)
    p.u.update(tuple(p.counts), 30.0, 6, DT)       # seen once, then gone for good
    p.dropouts = set(range(0, 100000))
    p.run()
    assert p.u.phase == uturn.FAILED and "lost while centring" in p.u.reason


def test_the_pivot_never_crawls_it_runs_at_spin_speed_then_stops():
    """Field run 2026-10-08: proportional centring crawled the last ~30 deg and the front
    wheels stuck on the tape's bulge. The commanded yaw is full spin speed or zero."""
    p = Pivot("cw")
    full = uturn.spin_omega(config.AUTO_U_TURN_RPM)
    seen = set()
    for tick in range(5000):
        e = p.reading(tick)
        left, right = p.u.update(tuple(p.counts), e, START_LEVEL if e is not None else None, DT)
        if not p.u.active:
            break
        _, omega = kinematics.wheels_to_body(left, right)
        seen.add(round(abs(omega) / full, 3))
        p.step(left, right)
    assert p.u.phase == uturn.DONE and centred(p)
    assert seen <= {0.0, 1.0}, seen


def test_a_stop_that_lags_the_command_still_lands_in_the_band():
    """The drives decelerate after the zero command: model 0.1 s of full-speed lag."""
    p = Pivot("cw")
    queue = [(0.0, 0.0)] * 5
    for tick in range(5000):
        e = p.reading(tick)
        cmd = p.u.update(tuple(p.counts), e, START_LEVEL if e is not None else None, DT)
        if not p.u.active:
            break
        queue.append(cmd)
        p.step(*queue.pop(0))
    assert p.u.phase == uturn.DONE, p.u.reason
    assert centred(p), p.reading(-1)
