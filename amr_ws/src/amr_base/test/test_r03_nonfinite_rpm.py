"""R03: a nonfinite wheel command is zero - at the drive node's ingress and
again at the final RPM conversion - and never keeps an earlier nonzero target."""

import math
import threading
from types import SimpleNamespace

import pytest

from amr_base.canopen import WheelScale, target_rpm

S = WheelScale(30.0, False, True)
BAD = (math.nan, math.inf, -math.inf, 1e308)


@pytest.mark.parametrize("bad", BAD)
def test_target_rpm_zeroes_nonfinite_or_overflowing_commands(bad):
    assert target_rpm((9.9, bad, 1.0), 10.0, 0.2, S, 4000) == (0, 0)
    assert target_rpm((9.9, 1.0, bad), 10.0, 0.2, S, 4000) == (0, 0)


def test_target_rpm_zeroes_a_nonfinite_limit_and_still_clamps_valid_commands():
    assert target_rpm((9.9, 1.0, 1.0), 10.0, 0.2, S, math.nan) == (0, 0)
    assert target_rpm((9.9, 1.0, 1.0), 10.0, 0.2, S, math.inf) == (0, 0)
    assert target_rpm((9.9, 100.0, 100.0), 10.0, 0.2, S, 4000) == (4000, -4000)
    assert target_rpm((9.9, math.nan, 1.0), math.nan, 0.2, S, 4000) == (0, 0)  # bad clock: not "fresh"


def test_drive_node_ingress_replaces_a_nonzero_command_with_nothing():
    from amr_base.drive_node import DriveNode

    warned = []
    node = SimpleNamespace(
        _lock=threading.Lock(),
        _cmd=None,
        _cmd_gen=0,
        _bad_cmds=0,
        get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: warned.append(a)),
    )
    DriveNode._on_cmd(node, SimpleNamespace(left_rad_s=2.0, right_rad_s=2.0, generation=3))
    assert node._cmd is not None and target_rpm(node._cmd, node._cmd[0], 0.2, S, 4000) != (0, 0)
    for bad in BAD[:3]:
        DriveNode._on_cmd(node, SimpleNamespace(left_rad_s=2.0, right_rad_s=bad, generation=3))
        assert node._cmd is None
        assert target_rpm(node._cmd, 0.0, 0.2, S, 4000) == (0, 0)
    assert node._bad_cmds == 3 and len(warned) == 3


def test_q03_command_age_is_judged_at_transmission_and_a_negative_age_is_zero():
    # a command received at 10.0 with a 0.2 s watchdog: fine at 10.1, expired at 11.0 (the loop's
    # pre-blocking timestamp must not be used), and a clock behind the receipt is not "fresh"
    assert target_rpm((10.0, 1.0, 1.0), 10.1, 0.2, S, 4000) != (0, 0)
    assert target_rpm((10.0, 1.0, 1.0), 11.0, 0.2, S, 4000) == (0, 0)
    assert target_rpm((10.0, 1.0, 1.0), 9.5, 0.2, S, 4000) == (0, 0)
