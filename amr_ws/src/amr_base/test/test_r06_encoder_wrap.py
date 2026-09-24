"""R06: wheel positions accumulate wrap-safe count deltas; the INT32 rollover is
one count of travel and a rebaseline never moves odometry."""

import math
from types import SimpleNamespace

import pytest

from amr_base.canopen import WheelPosition, WheelScale

CPR = 1_080_000.0  # measured on the vehicle
S = WheelScale(30.0, False, True, CPR)
ONE = 2 * math.pi / CPR
I32_MAX, I32_MIN = 2**31 - 1, -(2**31)


@pytest.mark.parametrize("left", [True, False])
def test_forward_and_reverse_rollover_is_one_count(left):
    sign = 1.0 if left else -1.0  # right is mounted inverted
    w = WheelPosition()
    w.update(I32_MAX, True, S, left, 1)
    assert w.update(I32_MIN, True, S, left, 1) == pytest.approx(sign * ONE)
    assert w.update(I32_MAX, True, S, left, 1) == pytest.approx(0.0, abs=1e-12)
    assert w.update(I32_MAX - 5, True, S, left, 1) == pytest.approx(-5 * sign * ONE)


def test_invalid_gap_rearm_and_scale_change_rebaseline_without_moving():
    w = WheelPosition()
    w.update(0, True, S, True, 1)
    p = w.update(1000, True, S, True, 1)
    assert p == pytest.approx(1000 * ONE)
    assert w.update(None, False, S, True, 1) == p  # invalid sample
    assert w.update(-123456789, True, S, True, 1) == p  # encoder reset: new baseline only
    assert w.update(-123456789 + 10, True, S, True, 1) == pytest.approx(p + 10 * ONE)
    p = w.rad
    assert w.update(5, True, S, True, 2) == p  # re-armed
    s2 = WheelScale(30.0, False, True, CPR / 2)
    assert w.update(500, True, s2, True, 2) == p  # scale changed
    assert w.update(510, True, s2, True, 2) == pytest.approx(p + 10 * 2 * ONE)
    assert w.update(520, True, WheelScale(30.0, False, True), True, 2) == w.rad  # scale unknown


def _odom():
    from amr_base.diff_drive import Geometry, OdomState
    from amr_base.diff_drive_odom_node import DiffDriveOdom

    published = []
    node = SimpleNamespace(
        geom=Geometry(0.09, 0.487),
        state=OdomState(),
        _last=None,
        _t=DiffDriveOdom._t,
        _publish=lambda msg, v, wz: published.append((v, wz)),
    )
    return node, lambda m: DiffDriveOdom._on_wheels(node, m), published


def _ws(t, left, right, valid=True, cpr=CPR):
    from amr_interfaces.msg import WheelStates

    m = WheelStates()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    m.left_pos_rad, m.right_pos_rad = left, right
    m.left_valid = m.right_valid = valid
    m.counts_per_wheel_rev = cpr
    return m


def test_odometry_stationary_through_rollover_and_rebaseline():
    node, on, _ = _odom()
    pos = {1: WheelPosition(), 2: WheelPosition()}
    raw = [(I32_MAX - 1, I32_MAX), (I32_MIN, I32_MIN + 1), (I32_MIN + 1, I32_MIN + 2)]
    for k, (rl, rr) in enumerate(raw):
        on(_ws(10 + 0.02 * k, pos[1].update(rl, True, S, True, 1), -pos[2].update(rr, True, S, True, 1)))
    assert math.hypot(node.state.x, node.state.y) < 1e-6  # a few counts, not kilometres
    x, th = node.state.x, node.state.th
    on(_ws(11.0, math.inf, 0.0))  # nonfinite: chain restarts, nothing integrated
    on(_ws(11.02, 5.0, 5.0))
    on(_ws(11.04, 50.0, 50.0, cpr=CPR / 2))  # scale changed between samples: baseline only
    assert node.state.x == x and node.state.th == th
    on(_ws(11.06, 50.0 + 1.0, 50.0 + 1.0, cpr=CPR / 2))
    assert node.state.x == pytest.approx(0.09)
