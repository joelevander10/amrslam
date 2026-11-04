"""The real panel adapter (T12): edges, validity, and the horn rule, without ROS."""

import pytest
from agv_core import config

from amr_base import panel_io

R, S, A = config.PANEL_DI_RESET, config.PANEL_DI_START, config.PANEL_DI_AUTO


def _di(reset=False, start=False, auto=False):
    """`auto` is the selector POSITION; the wire level follows panel.auto_when_on."""
    bits = [False] * 16
    bits[R], bits[S], bits[A] = reset, start, auto == config.PANEL_AUTO_WHEN_ON
    return bits


_ACQ = [0]


def _snap(di, comms=True, detail="ok", age=0.01, connected=True, new=True):
    """One DIO snapshot. Each call is a NEW acquisition unless new=False (audit R01)."""
    if new:
        _ACQ[0] += 1
    return {"comms_ok": comms, "di": di, "detail": detail, "rx_age_s": age, "connected": connected, "scans": _ACQ[0]}


def _adapter(estop_low_scans=1, manual_arm_scans=2):
    return panel_io.PanelAdapter(
        debounce_scans=2, estop_low_scans=estop_low_scans, manual_arm_scans=manual_arm_scans
    )


def _settle(ad, di, n=2, comms=True):
    f = None
    for _ in range(n):
        f = ad.tick(_snap(di, comms))
    return f


def test_first_image_is_a_baseline_with_no_edges_even_if_start_is_held():
    """Anti-tie-down: a taped-down Start at power-on does nothing until released."""
    ad = _adapter()
    f = _settle(ad, _di(start=True, auto=True))
    assert f.valid and f.mode_auto and not f.start_edge and not f.reset_edge
    f = _settle(ad, _di(start=True, auto=True))
    assert not f.start_edge
    _settle(ad, _di(auto=True))  # release
    f = _settle(ad, _di(start=True, auto=True))  # press
    assert f.start_edge and not f.reset_edge


def test_edges_fire_once_and_need_debounce():
    ad = _adapter()
    _settle(ad, _di())
    f = ad.tick(_snap(_di(start=True)))  # one scan: not yet believed
    assert not f.start_edge and f.valid
    f = ad.tick(_snap(_di(start=True)))  # second scan: edge
    assert f.start_edge
    f = ad.tick(_snap(_di(start=True)))  # held: no repeat
    assert not f.start_edge
    f = _settle(ad, _di())  # release: no edge
    assert not f.start_edge
    f = _settle(ad, _di(reset=True))
    assert f.reset_edge and not f.start_edge


def test_comms_loss_invalidates_and_reconnect_rebaselines_without_an_edge():
    """Hazard 3 in core/panel.py: a button pressed during an outage is not a press."""
    ad = _adapter()
    f = _settle(ad, _di(auto=True))
    assert f.valid and f.mode_auto
    f = ad.tick(_snap(_di(auto=True), comms=False, detail="timeout"))
    assert not f.valid and not f.comms_ok
    assert "LOST" in f.changed and "invalid" in f.changed
    # comes back with Start already down and the selector moved
    f = _settle(ad, _di(start=True, auto=False))
    assert f.valid and not f.start_edge and not f.mode_auto
    # only a fresh press counts
    _settle(ad, _di(auto=False))
    f = _settle(ad, _di(start=True, auto=False))
    assert f.start_edge


def test_selector_level_and_seq_and_log_notes():
    ad = _adapter()
    f = _settle(ad, _di(auto=False))
    assert not f.mode_auto and "MANUAL" in (f.changed or "")
    f = _settle(ad, _di(auto=True))
    assert f.mode_auto and "AUTO" in f.changed
    f2 = ad.tick(_snap(_di(auto=True)))
    assert f2.seq == f.seq + 1 and f2.changed is None
    f = _settle(ad, _di(auto=True, start=True))
    assert "START edge" in f.changed


def test_malformed_image_is_not_a_press():
    ad = _adapter()
    _settle(ad, _di())
    f = ad.tick(_snap([True, True], comms=True))  # short image
    assert not f.valid and not f.start_edge


def test_horn_follows_commanded_motion_while_armed_on_a_fresh_command():
    h = panel_io.horn_wanted
    assert h(True, 1.0, 1.0, 0.0, 0.2)
    assert h(True, 0.0, -0.5, 0.1, 0.2)  # rotate in place counts as motion
    assert not h(True, 0.0, 0.0, 0.0, 0.2)  # armed, holding zero: no horn
    assert not h(False, 1.0, 1.0, 0.0, 0.2)  # not armed: cannot be moving
    assert not h(True, 1.0, 1.0, None, 0.2)  # never received a command
    assert not h(True, 1.0, 1.0, 0.3, 0.2)  # stale command is zero


# ---- pendant -----------------------------------------------------------------

F, V, L, RT = (
    config.PENDANT_DI_FWD,
    config.PENDANT_DI_RVS,
    config.PENDANT_DI_LEFT,
    config.PENDANT_DI_RIGHT,
)


def _pdi(fwd=False, rvs=False, left=False, right=False):
    bits = _di()
    bits[F], bits[V], bits[L], bits[RT] = fwd, rvs, left, right
    return bits


def test_pendant_levels_and_log():
    ad = panel_io.PanelAdapter(debounce_scans=2, pendant=True)
    f = _settle(ad, _pdi())
    assert not any(f.pendant) and "pendant" not in (f.changed or "")
    f = _settle(ad, _pdi(fwd=True))
    assert f.pendant == (True, False, False, False) and "pendant FWD" in f.changed
    f = _settle(ad, _pdi(fwd=True))
    assert f.changed is None  # a held level is not repeated in the log
    f = _settle(ad, _pdi(fwd=True, rvs=True, left=True))
    assert f.pendant == (False, False, True, False)  # fwd+rvs cancel; left survives
    f = _settle(ad, _pdi())
    assert not any(f.pendant) and f.changed == "pendant released"


def test_pendant_is_held_at_power_on_and_dropped_on_comms_loss():
    ad = panel_io.PanelAdapter(debounce_scans=2, pendant=True)
    f = ad.tick(_snap(_pdi(fwd=True)))
    assert not any(f.pendant)  # one scan: not yet believed
    f = ad.tick(_snap(_pdi(fwd=True)))
    assert f.pendant.fwd  # no anti-tie-down: a held deadman drives
    f = _settle(ad, _pdi(fwd=True), comms=False)
    assert not any(f.pendant) and not f.valid
    f = ad.tick(_snap(_pdi(fwd=True)))
    assert not any(f.pendant)  # reconnect: needs a fresh debounce
    f = ad.tick(_snap(_pdi(fwd=True)))
    assert f.pendant.fwd


def test_manual_arm_is_a_debounced_level_and_dies_with_the_image():
    """DI09 (panel.di_manual_arm): under MANUAL the drives are powered only while it is HIGH."""
    ad = _adapter()

    def di(arm):
        bits = _di()
        bits[config.PANEL_DI_MANUAL_ARM] = arm
        return bits

    f = _settle(ad, di(False))
    assert f.valid and not f.manual_arm and "Manual Arm" not in (f.changed or "")
    f = ad.tick(_snap(di(True)))
    assert not f.manual_arm  # one scan: not yet believed
    f = ad.tick(_snap(di(True)))
    assert f.manual_arm and "Manual Arm on" in f.changed
    f = ad.tick(_snap(di(True), new=False))
    assert f.manual_arm  # a repeated image keeps the level
    f = ad.tick(_snap(di(True), comms=False))
    assert not f.valid and not f.manual_arm and "Manual Arm off" in f.changed
    f = _settle(ad, di(False))
    assert not f.manual_arm


def test_pendant_disabled_in_profile_publishes_nothing():
    ad = panel_io.PanelAdapter(debounce_scans=2, pendant=False)
    f = _settle(ad, _pdi(fwd=True))
    assert f.valid and not any(f.pendant)
    assert "pendant" not in (f.changed or "")


# ---- coincidence guard (2026-09-17 phantom MANUAL+FWD) ----


def _cdi(auto, fwd=False):
    bits = _di(auto=auto)
    bits[F] = fwd
    return bits


def test_selector_and_pendant_changing_together_are_withheld_and_revert_harmlessly():
    ad = panel_io.PanelAdapter(debounce_scans=2, pendant=True, coincidence_hold_scans=30)
    f = _settle(ad, _cdi(auto=True))
    assert f.mode_auto and not any(f.pendant)
    # the glitch: MANUAL and FWD land in the same scan, hold for 20 scans, then revert
    f = _settle(ad, _cdi(auto=False, fwd=True))
    assert f.mode_auto and not any(f.pendant) and "suspect" in f.changed
    for _ in range(18):
        f = ad.tick(_snap(_cdi(auto=False, fwd=True)))
        assert f.mode_auto and not any(f.pendant)  # nothing reaches the executor or the mux
    f = _settle(ad, _cdi(auto=True))
    assert f.mode_auto and not any(f.pendant) and "cleared" in f.changed
    assert "selector" not in f.changed  # no MANUAL transition was ever published


def test_a_persisting_coincidence_is_accepted_after_the_hold():
    ad = panel_io.PanelAdapter(debounce_scans=2, pendant=True, coincidence_hold_scans=10)
    _settle(ad, _cdi(auto=True))
    f = _settle(ad, _cdi(auto=False, fwd=True))
    assert f.mode_auto
    for _ in range(8):  # detection counted 1; the change is believed on the hold_scans-th scan
        f = ad.tick(_snap(_cdi(auto=False, fwd=True)))
    assert f.mode_auto  # still withheld
    f = ad.tick(_snap(_cdi(auto=False, fwd=True)))
    assert not f.mode_auto and f.pendant[0] and "accepted" in f.changed and "selector MANUAL" in f.changed


def test_separate_selector_and_pendant_changes_are_immediate():
    ad = panel_io.PanelAdapter(debounce_scans=2, pendant=True, coincidence_hold_scans=30)
    _settle(ad, _cdi(auto=True))
    f = _settle(ad, _cdi(auto=False))  # key first
    assert not f.mode_auto and "selector MANUAL" in f.changed
    f = _settle(ad, _cdi(auto=False, fwd=True))  # then the pendant: a hand, not a glitch
    assert f.pendant[0] and "pendant FWD" in f.changed
    # and back to AUTO with the pendant released together (a normal end of jogging) is immediate too
    f = _settle(ad, _cdi(auto=True))
    assert f.mode_auto and not any(f.pendant)


# ---- audit R01 (2026-10-02): freshness and debounce follow DIO ACQUISITIONS ----------------


def test_a_repeated_image_does_not_advance_debounce_or_repeat_an_edge():
    ad = _adapter()
    _settle(ad, _di(auto=True))
    ad.tick(_snap(_di(start=True, auto=True)))  # pressed: debounce 1 of 2
    for _ in range(5):  # the 50 Hz node re-ticking the SAME acquisition
        f = ad.tick(_snap(_di(start=True, auto=True), new=False))
        assert not f.start_edge, "a duplicate image is not a second debounce scan"
    f = ad.tick(_snap(_di(start=True, auto=True)))  # second real scan: accepted
    assert f.start_edge
    f = ad.tick(_snap(_di(start=True, auto=True), new=False))
    assert f.valid and f.mode_auto and not f.start_edge, "the edge is spent, the levels stand"


def test_an_old_image_is_not_panel_input_even_inside_the_warning_age():
    ad = _adapter()
    f = _settle(ad, _di(auto=True))
    assert f.valid
    f = ad.tick(_snap(_di(auto=True), age=config.PANEL_SOURCE_MAX_AGE_S + 0.05, new=False))
    assert config.PANEL_SOURCE_MAX_AGE_S < config.DIO_SILENT_WARN_S
    assert not f.valid and not f.comms_ok, "a stalled scan thread expires the panel"


def test_a_lost_connection_invalidates_at_once():
    ad = _adapter()
    _settle(ad, _di(auto=True))
    f = ad.tick(_snap(_di(auto=True), connected=False, new=False))
    assert not f.valid


def test_a_held_pendant_dies_with_the_source_and_the_coincidence_hold_is_1_5_s():
    ad = panel_io.PanelAdapter(debounce_scans=2, pendant=True)
    fwd = _di(auto=False)
    fwd[config.PENDANT_DI_FWD] = True
    _settle(ad, _di(auto=False))
    f = _settle(ad, fwd)
    assert f.pendant.fwd
    f = ad.tick(_snap(fwd, age=1.0, new=False))  # scan thread stalled 1 s: not a hand now
    assert not f.pendant.fwd
    # the default hold is counted in acquisitions at the DIO scan period: 1.5 s, not 0.6 s
    assert ad.hold_scans * config.DIO_SCAN_PERIOD_S == pytest.approx(config.PANEL_COINCIDENCE_HOLD_S, abs=config.DIO_SCAN_PERIOD_S)


def _estop_di(high):
    bits = _di()
    bits[config.PANEL_DI_ESTOP_OK] = high
    return bits


def test_estop_ok_low_counts_at_once_high_needs_debounce():
    """DI08 (panel.di_estop_ok) with estop_low_scans=1: going to EMERGENCY is one scan, leaving
    it needs debounce_scans consecutive HIGH scans; an invalid image is never OK (fail-safe)."""
    ad = _adapter()
    f = _settle(ad, _estop_di(True))  # the first valid image: already OK, no boot-time LOW
    assert f.valid and f.estop_ok
    f = ad.tick(_snap(_estop_di(False)))  # one LOW scan: at once
    assert not f.estop_ok and "LOW" in f.changed
    f = ad.tick(_snap(_estop_di(True)))
    assert not f.estop_ok, "one HIGH scan after a LOW is not believed"
    f = ad.tick(_snap(_estop_di(True)))
    assert f.estop_ok and "HIGH" in f.changed
    f = ad.tick(_snap(_estop_di(True), new=False))  # same acquisition republished
    assert f.estop_ok
    f = ad.tick(_snap(_estop_di(True), comms=False))
    assert not f.valid and not f.estop_ok


def test_estop_low_filter_rejects_one_scan_pulses():
    """2026-11-02: Q3 read LOW for one scan every 8 scans (400 ms) in bursts. With
    estop_low_scans=2 such a train never trips; two LOW scans in a row do."""
    ad = _adapter(estop_low_scans=2)
    f = _settle(ad, _estop_di(True))
    assert f.estop_ok
    for _ in range(10):  # one LOW scan, seven HIGH, ten times over
        f = ad.tick(_snap(_estop_di(False)))
        assert f.estop_ok and not f.changed, "a one-scan LOW is filtered"
        for _ in range(7):
            f = ad.tick(_snap(_estop_di(True)))
            assert f.estop_ok
    f = ad.tick(_snap(_estop_di(False)))
    f = ad.tick(_snap(_estop_di(False), new=False))  # a republished image is not a 2nd scan
    assert f.estop_ok
    f = ad.tick(_snap(_estop_di(False)))
    assert not f.estop_ok and "LOW" in f.changed, "two LOW scans are a press"
    f = ad.tick(_snap(_estop_di(True)))
    assert not f.estop_ok, "leaving still needs debounce_scans HIGH"
    f = ad.tick(_snap(_estop_di(True)))
    assert f.estop_ok


def test_estop_low_filter_never_delays_an_invalid_image():
    """The filter is for DI08's level only: a comms gap is an emergency on the same tick."""
    ad = _adapter(estop_low_scans=2)
    f = _settle(ad, _estop_di(True))
    assert f.estop_ok
    f = ad.tick(_snap(_estop_di(True), comms=False))
    assert not f.valid and not f.estop_ok


def test_manual_arm_filter_has_its_own_scan_count():
    """DI09 (panel.manual_arm_scans): a level must hold that many scans, either way."""
    ad = _adapter(manual_arm_scans=3)

    def di(arm):
        bits = _di()
        bits[config.PANEL_DI_MANUAL_ARM] = arm
        return bits

    f = _settle(ad, di(True), n=3)
    assert f.manual_arm
    for _ in range(2):  # two LOW scans: not believed
        f = ad.tick(_snap(di(False)))
        assert f.manual_arm
    f = ad.tick(_snap(di(True)))
    assert f.manual_arm, "a two-scan dropout is filtered"
    for _ in range(3):
        f = ad.tick(_snap(di(False)))
    assert not f.manual_arm and "Manual Arm off" in f.changed


def test_estop_ok_returns_with_the_image_after_a_comms_gap():
    """Never valid with DI08 'LOW' for a scan just because the counter lagged the baseline:
    that would be a false ESTOP event at every boot and after every DIO gap."""
    ad = _adapter()
    for comms in (True, True, False, True, True):
        f = ad.tick(_snap(_estop_di(True), comms=comms))
        assert f.estop_ok == f.valid, comms


def _di_c(cont=False, **kw):
    bits = _di(**kw)
    bits[config.PANEL_DI_CONTINUE] = cont
    return bits


def test_continue_is_an_edge_with_starts_rules():
    """PB Continue (DI13, 2026-10-30): one edge per press, none when held at power-on, none
    from a repeated image, and not a Start."""
    ad = _adapter()
    f = _settle(ad, _di_c(cont=True, auto=True))
    assert f.valid and not f.continue_edge, "held at power-on: a baseline, never a press"
    _settle(ad, _di_c(auto=True))
    ad.tick(_snap(_di_c(cont=True, auto=True)))
    f = ad.tick(_snap(_di_c(cont=True, auto=True)))
    assert f.continue_edge and not f.start_edge and "CONTINUE edge" in (f.changed or "")
    f = ad.tick(_snap(_di_c(cont=True, auto=True), new=False))
    assert not f.continue_edge, "a repeated image spends the edge"
    f = _settle(ad, _di_c(cont=True, auto=True))
    assert not f.continue_edge, "held: no repeat"


def _di_d(*dests, **kw):
    bits = _di(**kw)
    for k in dests:  # 1-based, like PanelState.dest_edge
        bits[config.PANEL_DESTINATIONS[k - 1][1]] = True
    return bits


def test_destination_buttons_are_edges_with_starts_rules():
    """DI10-12 (2026-11-03): one edge per press as a 1-based index, none at power-on, none
    from a repeated image, none for two at once, and never a Start or a Continue."""
    itg = [n for n, _c, _r in config.PANEL_DESTINATIONS].index("ITG") + 1
    ad = _adapter()
    f = _settle(ad, _di_d(itg, auto=True))
    assert f.valid and f.dest_edge == 0, "held at power-on: a baseline, never a press"
    _settle(ad, _di_d(auto=True))
    ad.tick(_snap(_di_d(itg, auto=True)))
    f = ad.tick(_snap(_di_d(itg, auto=True)))
    assert f.dest_edge == itg and not f.start_edge and not f.continue_edge
    assert "DEST ITG edge" in (f.changed or "")
    assert ad.tick(_snap(_di_d(itg, auto=True), new=False)).dest_edge == 0, "a repeated image spends it"
    assert _settle(ad, _di_d(itg, auto=True)).dest_edge == 0, "held: no repeat"
    _settle(ad, _di_d(auto=True))
    ad.tick(_snap(_di_d(1, 3, auto=True)))
    f = ad.tick(_snap(_di_d(1, 3, auto=True)))
    assert f.dest_edge == 0 and "pressed together" in (f.changed or "")


def test_destination_press_is_dropped_with_the_image():
    ad = _adapter()
    _settle(ad, _di_d(auto=True))
    ad.tick(_snap(_di_d(2, auto=True), comms=False))
    f = _settle(ad, _di_d(2, auto=True))
    assert f.dest_edge == 0, "a reconnect with the button down is a baseline"
