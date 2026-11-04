"""ControlLease.allowed bits per mode, and the requests a trackless vehicle admits.
Ported from test_tracked_product.py when LINE mode was retired (removal plan P1)."""

import types

from amr_bringup import mode_fsm as fsm
from amr_bringup.supervisor_node import LEASE_AUTONOMOUS, LEASE_COMMISSIONING, LEASE_MANUAL, Supervisor


def cond(**kw):
    base = dict(
        now=100.0,
        wheels_t=100.0,
        wheels_still_since=99.0,
        panel_t=100.0,
        panel_valid=True,
        panel_manual=True,
        run_state=fsm.RUN_IDLE,
        survey_state=None,
    )
    base.update(kw)
    return fsm.Conditions(**base)


def allowed(mode, txn=None, inhibit_manual=False, commissioning=False, emergency=0):
    sv = types.SimpleNamespace(
        txn=txn,
        mode=mode,
        inhibit_manual=inhibit_manual,
        snap=types.SimpleNamespace(commissioning_active=lambda now: commissioning,
                                   emergency=lambda now: emergency),
        _now=lambda: 100.0,
    )
    return Supervisor._allowed(sv)


def test_lease_bits_per_mode():
    assert (LEASE_MANUAL, LEASE_AUTONOMOUS, LEASE_COMMISSIONING) == (1, 2, 4)
    assert allowed(fsm.NAVIGATION) == LEASE_MANUAL | LEASE_AUTONOMOUS
    assert allowed(fsm.IDLE) == LEASE_MANUAL
    assert allowed(fsm.IDLE, commissioning=True) == LEASE_COMMISSIONING
    assert allowed(fsm.MAPPING) == LEASE_MANUAL
    # inhibited: a transaction in flight, an unstable mode, or a save
    assert allowed(fsm.NAVIGATION, txn=object()) == 0
    for mode in (fsm.STARTING, fsm.TRANSITIONING, fsm.FAULT, fsm.STOPPING):
        assert allowed(mode) == 0
    assert allowed(fsm.NAVIGATION, inhibit_manual=True) == 0
    # bit 8 was LINE: retired, never set again
    for mode in fsm.MODE_NAMES:
        for c in (False, True):
            assert not allowed(mode, commissioning=c) & 8


def test_a_trackless_vehicle_admits_navigation_and_a_survey():
    assert fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond()).ok
    assert fsm.admit(fsm.IDLE, fsm.REQ_SURVEY_START, cond()).ok


def test_no_lease_and_no_mode_change_while_the_emergency_is_not_normal():
    """Emergency plan §4.5: allowed = 0 (a second no-motion layer above the mux), and a
    survey start or a mode replacement is refused with the reason; the mode itself stays."""
    for emg in (1, 2, 3, 4):
        for mode in (fsm.NAVIGATION, fsm.IDLE, fsm.MAPPING):
            assert allowed(mode, emergency=emg) == 0
            assert allowed(mode, commissioning=True, emergency=emg) == 0
        for req in (fsm.REQ_SURVEY_START, fsm.REQ_NAVIGATION, fsm.REQ_IDLE):
            d = fsm.admit(fsm.NAVIGATION, req, cond(emergency=emg))
            assert not d.ok and "E-stop active" in d.reason, req
    assert fsm.admit(fsm.NAVIGATION, fsm.REQ_SURVEY_ABORT, cond(emergency=1)).ok is False  # not surveying
    assert fsm.admit(fsm.MAPPING, fsm.REQ_SURVEY_ABORT, cond(emergency=1)).ok, "stopping a survey is fine"


def test_the_supervisor_reads_the_emergency_only_from_a_fresh_panel():
    from amr_bringup.readiness import Snapshots

    s = Snapshots()
    s.on_panel(types.SimpleNamespace(valid=True, mode_auto=False, emergency=2))
    now = s.panel_t
    assert s.emergency(now) == 2 and s.conditions(now, 0, False).emergency == 2
    assert s.emergency(now + 1.5) == 0, "stale: PANEL_STALE's to report"
