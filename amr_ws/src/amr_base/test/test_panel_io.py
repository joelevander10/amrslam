"""The real panel adapter (T12): edges, validity, and the horn rule, without ROS."""

from amr_base import panel_io
from amr_base.agv_repo import config

R, S, A = config.PANEL_DI_RESET, config.PANEL_DI_START, config.PANEL_DI_AUTO


def _di(reset=False, start=False, auto=False):
    bits = [False] * 16
    bits[R], bits[S], bits[A] = reset, start, auto
    return bits


def _snap(di, comms=True, detail="ok"):
    return {"comms_ok": comms, "di": di, "detail": detail}


def _adapter():
    return panel_io.PanelAdapter(debounce_scans=2)


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
