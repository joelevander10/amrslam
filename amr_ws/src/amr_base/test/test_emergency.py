"""amr_base.emergency: the PC's emergency picture (emergency-suite plan §3, owner 2026-10-30).

Every transition, fail-safe, and the two ways the Flexi Soft may drive Q3 (bench question Q1):
unlatched (Q3 follows the button) and latched (Q3 LOW until the PLC is reset)."""

import pytest

from amr_base import emergency as em
from amr_base.emergency import (
    ACTIVE,
    NORMAL,
    RESET_FAILED,
    RESET_NEEDED,
    RESETTING,
    Emergency,
    Inputs,
    sto_held,
)

T = 3.0  # panel.estop_reset_timeout_s


def tick(e, now, ok=True, estop=True, reset=False, sto=False):
    return e.step(now, Inputs(ok, estop, reset, sto))


def normal_at(now=0.0):
    e = Emergency(T)
    tick(e, now)
    assert e.state == NORMAL
    return e


def test_the_states_are_the_panel_state_contract():
    assert (NORMAL, ACTIVE, RESET_NEEDED, RESETTING, RESET_FAILED) == (0, 1, 2, 3, 4)
    from amr_interfaces.msg import PanelState

    assert (PanelState.EMG_NORMAL, PanelState.EMG_ACTIVE, PanelState.EMG_RESET_NEEDED,
            PanelState.EMG_RESETTING, PanelState.EMG_RESET_FAILED) == (0, 1, 2, 3, 4)


def test_boot_is_active_and_blind_until_a_trusted_image_reads_high():
    e = Emergency(T)
    assert e.state == ACTIVE and not e.tripped
    assert tick(e, 0.0, ok=False, estop=False) is None and e.state == ACTIVE
    tick(e, 0.1)  # first valid image, DI08 HIGH, drives not in STO: no Reset needed
    assert e.state == NORMAL


def test_boot_with_the_drives_in_sto_asks_for_reset():
    """2026-09-22: the drives sat in Switch on disabled after boot until the safety reset."""
    e = Emergency(T)
    tick(e, 0.0, sto=True)
    assert e.state == RESET_NEEDED


@pytest.mark.parametrize("ok,estop", [(True, False), (False, False), (False, True)])
def test_a_low_glitch_or_an_invalid_or_stale_panel_enters_at_once(ok, estop):
    for start in (NORMAL, RESET_NEEDED, RESETTING, RESET_FAILED):
        e = normal_at()
        e.state, e.tripped = start, start != NORMAL
        assert tick(e, 1.0, ok=ok, estop=estop) is not None
        assert e.state == ACTIVE, start


def test_a_blind_gap_needs_no_reset_unless_the_drives_hold_sto():
    e = normal_at()
    tick(e, 1.0, ok=False, estop=False)  # DIO comms gap
    assert e.state == ACTIVE and not e.tripped
    tick(e, 1.5)
    assert e.state == NORMAL
    tick(e, 2.0, ok=False, estop=False)
    tick(e, 2.5, sto=True)  # pressed and released while the PC was blind
    assert e.state == RESET_NEEDED


def test_low_on_a_valid_image_is_tripped_even_after_a_blind_entry():
    e = normal_at()
    tick(e, 1.0, ok=False, estop=False)
    assert tick(e, 1.1, estop=False) is not None  # now known: an event-worthy transition
    assert e.state == ACTIVE and e.tripped
    tick(e, 1.2, sto=False)  # DI08 HIGH, no Reset: still needs one
    assert e.state == RESET_NEEDED


def test_unlatched_sequence_press_release_reset():
    e = normal_at()
    tick(e, 1.0, estop=False, sto=True)
    assert e.state == ACTIVE
    tick(e, 5.0, sto=True)  # released (DI08 HIGH, debounced by the adapter)
    assert e.state == RESET_NEEDED
    tick(e, 6.0, sto=True)  # no Reset: stays
    assert e.state == RESET_NEEDED
    tick(e, 7.0, reset=True, sto=True)
    assert e.state == RESETTING
    tick(e, 7.5, sto=True)
    assert e.state == RESETTING
    tick(e, 7.6, sto=False)  # drives left Switch on disabled
    assert e.state == NORMAL and not e.tripped


def test_latched_sequence_one_reset_press_is_enough():
    """Q3 stays LOW until Reset: the edge lands while DI08 is LOW, DI08 rises after it."""
    e = normal_at()
    tick(e, 1.0, estop=False, sto=True)
    tick(e, 5.0, estop=False, sto=True)  # released, but Q3 latched: PC cannot tell
    assert tick(e, 6.0, estop=False, reset=True, sto=True) is None  # the edge does nothing visible
    assert e.state == ACTIVE
    tick(e, 6.4, sto=True)  # the PLC reset raised Q3
    assert e.state == RESETTING
    tick(e, 6.6, sto=False)
    assert e.state == NORMAL


def test_latched_reset_held_past_the_window_falls_back_to_reset_needed():
    e = normal_at()
    tick(e, 1.0, estop=False)
    tick(e, 2.0, estop=False, reset=True)
    tick(e, 2.0 + T + 0.1, sto=True)
    assert e.state == RESET_NEEDED


def test_a_reset_with_dio8_low_does_nothing():
    e = normal_at()
    tick(e, 1.0, estop=False, sto=True)
    for t in (2.0, 2.1, 2.2):
        tick(e, t, estop=False, reset=True, sto=True)
        assert e.state == ACTIVE
    tick(e, 2.0 + T + 1.0, sto=True)  # released well after: the early Reset is forgotten
    assert e.state == RESET_NEEDED


def test_a_premature_reset_then_a_quick_release_ends_in_reset_failed():
    """Unlatched Q3, Reset with the E-stop still in (Q4: the PLC ignores it), released inside
    the window: the PC goes RESETTING, the drives never leave STO, so 'press Reset again'."""
    e = normal_at()
    tick(e, 1.0, estop=False, sto=True)
    tick(e, 2.0, estop=False, reset=True, sto=True)
    tick(e, 2.5, sto=True)
    assert e.state == RESETTING
    tick(e, 2.5 + T, sto=True)
    assert e.state == RESET_FAILED
    tick(e, 9.0, reset=True, sto=True)  # Reset again
    assert e.state == RESETTING
    tick(e, 9.2, sto=False)
    assert e.state == NORMAL


def test_the_timeout_reaches_reset_failed_and_a_late_reset_still_clears():
    e = normal_at()
    tick(e, 1.0, estop=False, sto=True)
    tick(e, 2.0, sto=True)
    tick(e, 3.0, reset=True, sto=True)
    tick(e, 3.0 + T - 0.01, sto=True)
    assert e.state == RESETTING
    note = tick(e, 3.0 + T, sto=True)
    assert e.state == RESET_FAILED and "STO" in note
    tick(e, 8.0, sto=None)  # unobservable is not proof
    assert e.state == RESET_FAILED
    tick(e, 8.5, sto=False)  # the drives came out after all
    assert e.state == NORMAL


def test_unobservable_drives_take_the_reset_at_face_value():
    """No fresh DriveStatus (no bus, sim): nothing can be powered and nothing can be checked."""
    e = normal_at()
    tick(e, 1.0, estop=False, sto=None)
    tick(e, 2.0, sto=None)
    tick(e, 3.0, reset=True, sto=None)
    assert e.state == RESETTING
    tick(e, 3.02, sto=None)
    assert e.state == NORMAL


def test_dio8_dropping_during_the_reset_is_a_new_emergency():
    e = normal_at()
    tick(e, 1.0, estop=False, sto=True)
    tick(e, 2.0, sto=True)
    tick(e, 3.0, reset=True, sto=True)
    tick(e, 3.1, estop=False, sto=True)
    assert e.state == ACTIVE and e.tripped


def test_reset_in_normal_and_drives_leaving_sto_in_reset_needed_change_nothing():
    e = normal_at()
    assert tick(e, 1.0, reset=True) is None and e.state == NORMAL
    tick(e, 2.0, estop=False, sto=True)
    tick(e, 3.0, sto=True)
    tick(e, 4.0, sto=False)  # physical Reset only: no auto-clear
    assert e.state == RESET_NEEDED


def test_transitions_report_once_and_carry_the_names():
    e = normal_at()
    note = tick(e, 1.0, estop=False)
    assert note.startswith("emergency NORMAL -> EMERGENCY") and e.since == 1.0
    assert tick(e, 1.02, estop=False) is None


def test_sto_held_reads_only_fresh_linked_drive_reports():
    sod, ready = em.STO_STATE, "Ready to switch on"
    assert sto_held(sod, sod, "standby", 0.1) is True
    assert sto_held(sod, ready, "standby", 0.1) is True  # one still held: not finished
    assert sto_held(ready, ready, "standby", 0.1) is False
    assert sto_held("Operation enabled", "Operation enabled", "armed", 0.1) is False
    assert sto_held(sod, sod, "standby", 0.6) is None  # stale
    assert sto_held(sod, sod, "standby", None) is None
    assert sto_held(sod, sod, "no bus", 0.1) is None
    assert sto_held(sod, sod, "disarmed", 0.1) is None
    assert sto_held("", "", "standby", 0.1) is None
