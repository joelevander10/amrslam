"""WASD teleop: key mapping, hold-to-drive timing, arrow-key parsing (no ROS, no terminal)."""

import pytest

from amr_base.wasd_teleop import FIRST_HOLD_S, REPEAT_HOLD_S, KeyHold, Speeds, key_to_twist, split_keys


def test_wasd_signs():
    sp = Speeds(v=0.1, w=0.3)
    assert key_to_twist("w", sp) == (0.1, 0.0)
    assert key_to_twist("s", sp) == (-0.1, 0.0)  # backward is S
    assert key_to_twist("a", sp) == (0.0, 0.3)  # CCW positive (REP 103)
    assert key_to_twist("d", sp) == (0.0, -0.3)
    assert key_to_twist("W", sp) == (0.1, 0.0)  # caps lock does not matter
    assert key_to_twist("q", sp) == (0.1, pytest.approx(0.15))
    assert key_to_twist("k", sp) is None


def test_arrow_keys_parse():
    assert split_keys("\x1b[A\x1b[Bw\x1bOD") == ["UP", "DOWN", "w", "LEFT"]
    assert key_to_twist("DOWN", Speeds(v=0.2)) == (-0.2, 0.0)


def test_hold_first_press_then_repeats_then_release():
    h = KeyHold()
    h.press("w", 0.0)
    assert h.held(FIRST_HOLD_S - 0.01) == "w"  # bridges the initial repeat delay
    h.press("w", 0.5)  # first repeat
    h.press("w", 0.54)
    assert h.held(0.54 + REPEAT_HOLD_S - 0.01) == "w"
    assert h.held(0.54 + REPEAT_HOLD_S + 0.01) is None  # released: stops within REPEAT_HOLD_S


def test_switching_key_and_space_stop():
    h = KeyHold()
    h.press("w", 0.0)
    h.press("s", 0.1)  # a different key is a new first press, not a repeat
    assert h.held(0.1 + FIRST_HOLD_S - 0.01) == "s"
    h.stop()
    assert h.held(0.2) is None


def test_speed_limits():
    sp = Speeds()
    for _ in range(20):
        sp.adjust("+")
        sp.adjust("]")
    assert sp.v == 0.40 and sp.w == 1.00
    for _ in range(20):
        sp.adjust("-")
        sp.adjust("[")
    assert sp.v == 0.05 and sp.w == 0.10
    assert not sp.adjust("x")
