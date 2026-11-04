"""amr_base.arm_policy: when the drives are powered (2026-10-08). Every unknown is "no power"."""

from amr_base.arm_policy import Ask, Panel, Params, want_power

P = Params(panel_timeout_s=1.0, ask_timeout_s=2.5)
NOW = 100.0


def panel(auto=False, arm=False, valid=True, t=NOW):
    return Panel(t_recv=t, valid=valid, mode_auto=auto, manual_arm=arm)


def test_manual_follows_the_arm_input():
    assert want_power(NOW, panel(arm=True), [], P) == (True, "MANUAL, pendant enable (arm) on")
    on, why = want_power(NOW, panel(arm=False), [], P)
    assert not on and "arm" in why


def test_manual_ignores_a_run_asking_for_power():
    asks = [Ask("run", NOW, True)]
    assert want_power(NOW, panel(arm=False), asks, P)[0] is False


def test_auto_follows_the_run_while_armed():
    assert want_power(NOW, panel(auto=True, arm=True), [], P)[0] is False
    assert want_power(NOW, panel(auto=True, arm=True), [Ask("run", NOW - 1.0, False)], P)[0] is False
    on, why = want_power(NOW, panel(auto=True, arm=True), [Ask("run", NOW - 1.0, True)], P)
    assert on and "run" in why
    on, why = want_power(NOW, panel(auto=True, arm=True), [Ask("run", NOW, False), Ask("run", NOW, True)], P)
    assert on and "run" in why


def test_arm_low_disarms_auto_even_when_a_run_asks():
    # MCQ4-4 DI09 (operator, 2026-10-27): low disarms regardless of auto/manual
    on, why = want_power(NOW, panel(auto=True, arm=False), [Ask("run", NOW, True)], P)
    assert not on and why == "AUTO, pendant enable (arm) off"


def test_a_stale_request_is_no_request():
    # a latched RunState replayed from a layer that died: its own stamp is old
    assert want_power(NOW, panel(auto=True, arm=True), [Ask("run", NOW - 2.6, True)], P)[0] is False
    # a stamp from the future is a clock error, not freshness
    assert want_power(NOW, panel(auto=True, arm=True), [Ask("run", NOW + 0.5, True)], P)[0] is False


def test_a_stale_or_invalid_panel_is_no_power_in_either_mode():
    asks = [Ask("run", NOW, True)]
    assert want_power(NOW, None, asks, P) == (False, "no panel state")
    assert want_power(NOW, panel(auto=True, arm=True, t=NOW - 1.1), asks, P) == (False, "panel state stale")
    assert want_power(NOW, panel(arm=True, t=NOW - 1.1), [], P)[0] is False
    assert want_power(NOW, panel(auto=True, arm=True, valid=False), asks, P) == (False, "panel image invalid")
    assert want_power(NOW, panel(arm=True, valid=False), [], P)[0] is False
    assert want_power(NOW, panel(arm=True, t=NOW + 0.1), [], P)[0] is False


def test_no_power_while_the_emergency_is_not_normal_whatever_else_says_yes():
    """Emergency plan §4.2: checked before the arm and mode rules, so the 2 s re-engage loop
    against STO (SAFETY_RESET_NEEDED every 2 s) never starts."""
    asks = [Ask("run", NOW, True)]
    for auto in (False, True):
        for emg, why in ((1, "E-stop (DI08)"), (2, "waiting for Reset"), (3, "waiting for Reset"),
                         (4, "waiting for Reset")):
            p = Panel(t_recv=NOW, valid=True, mode_auto=auto, manual_arm=True, emergency=emg)
            assert want_power(NOW, p, asks, P) == (False, why), (auto, emg)
    # NORMAL again: power exactly as after boot (a zero target; motion is the mux's)
    p = Panel(t_recv=NOW, valid=True, mode_auto=True, manual_arm=True, emergency=0)
    assert want_power(NOW, p, asks, P)[0] is True
