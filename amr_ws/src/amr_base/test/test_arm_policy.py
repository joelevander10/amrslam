"""amr_base.arm_policy: when the drives are powered (2026-10-08). Every unknown is "no power"."""

from amr_base.arm_policy import Ask, Panel, Params, want_power

P = Params(panel_timeout_s=1.0, ask_timeout_s=2.5)
NOW = 100.0


def panel(auto=False, arm=False, valid=True, t=NOW):
    return Panel(t_recv=t, valid=valid, mode_auto=auto, manual_arm=arm)


def test_manual_follows_the_manual_arm_input():
    assert want_power(NOW, panel(arm=True), [], P) == (True, "MANUAL, Manual Arm on")
    on, why = want_power(NOW, panel(arm=False), [], P)
    assert not on and "DI08" in why


def test_manual_ignores_a_run_asking_for_power():
    asks = [Ask("line", NOW, True)]
    assert want_power(NOW, panel(arm=False), asks, P)[0] is False


def test_auto_follows_the_run_and_ignores_manual_arm():
    assert want_power(NOW, panel(auto=True, arm=True), [], P)[0] is False
    assert want_power(NOW, panel(auto=True), [Ask("line", NOW - 1.0, False)], P)[0] is False
    on, why = want_power(NOW, panel(auto=True), [Ask("line", NOW - 1.0, True)], P)
    assert on and "line" in why
    on, why = want_power(NOW, panel(auto=True), [Ask("line", NOW, False), Ask("run", NOW, True)], P)
    assert on and "run" in why


def test_a_stale_request_is_no_request():
    # a latched LineState replayed from a layer that died: its own stamp is old
    assert want_power(NOW, panel(auto=True), [Ask("line", NOW - 2.6, True)], P)[0] is False
    # a stamp from the future is a clock error, not freshness
    assert want_power(NOW, panel(auto=True), [Ask("run", NOW + 0.5, True)], P)[0] is False


def test_a_stale_or_invalid_panel_is_no_power_in_either_mode():
    asks = [Ask("line", NOW, True)]
    assert want_power(NOW, None, asks, P) == (False, "no panel state")
    assert want_power(NOW, panel(auto=True, t=NOW - 1.1), asks, P) == (False, "panel state stale")
    assert want_power(NOW, panel(arm=True, t=NOW - 1.1), [], P)[0] is False
    assert want_power(NOW, panel(auto=True, valid=False), asks, P) == (False, "panel image invalid")
    assert want_power(NOW, panel(arm=True, valid=False), [], P)[0] is False
    assert want_power(NOW, panel(arm=True, t=NOW + 0.1), [], P)[0] is False
