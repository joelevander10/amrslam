"""Review R02 (release loses to the seq filter) and R03 (nonfinite commands), mux side."""

import math

from amr_base import gating
from amr_base.gating import LEASE_MANUAL, MANUAL, NONE, Drives, Lease, ManualIntake, Panel, select

SUP = gating.Params(require_supervisor=True, teleop_enabled=False)
INST = "11111111-aaaa"
PANEL = Panel(t_recv=10.0, valid=True, auto=False)
DRIVES = Drives(t_recv=10.0, operational=True)
LEASE = Lease(t_recv=10.0, instance=INST, generation=3, seq=10, allowed=LEASE_MANUAL)


def selected(intake, now=10.0):
    return select(now, None, None, None, None, PANEL, SUP, LEASE, intake.current, DRIVES)


def refresh(intake, session, seq, v=0.2, valid=0.2, now=10.0):
    return intake.offer(now, INST, 3, session, seq, valid, v, 0.0)


def test_release_with_seq_zero_revokes_after_a_later_refresh():
    """The probe from the review: seq 7 accepted, then same-session zero with seq 0."""
    m = ManualIntake()
    refresh(m, "s1", 7)
    assert (selected(m).source, selected(m).v) == (MANUAL, 0.2)
    assert refresh(m, "s1", 0, v=0.0, valid=0.0)  # the server's release
    s = selected(m)
    assert (s.source, s.v) == (NONE, 0.0)  # zero on the next tick, not at command expiry


def test_revoked_session_cannot_be_revived_by_a_delayed_or_reordered_refresh():
    m = ManualIntake()
    refresh(m, "s1", 3)
    refresh(m, "s1", 0, v=0.0, valid=0.0)
    assert not refresh(m, "s1", 4)  # delivered after the release
    assert not refresh(m, "s1", 99)
    assert m.current is None and selected(m).source == NONE
    refresh(m, "s1", 0, v=0.0, valid=0.0)  # duplicate release: harmless
    assert m.current is None
    # a new press gets a new session id and works
    assert refresh(m, "s2", 1)
    assert selected(m).source == MANUAL


def test_release_of_another_session_does_not_drop_the_live_one_but_stop_does():
    m = ManualIntake()
    refresh(m, "s2", 5)
    refresh(m, "old", 0, v=0.0, valid=0.0)
    assert m.current is not None and m.current.session == "s2"
    refresh(m, "", 0, v=0.0, valid=0.0)  # /api/stop
    assert m.current is None
    # Q10: the stopped session is dead - a delayed refresh with a newer seq cannot revive it
    assert not refresh(m, "s2", 6)
    assert m.current is None
    # a fresh press (new session) works; a second global stop with nothing held is harmless
    assert refresh(m, "s3", 1) and m.current.session == "s3"
    refresh(m, "", 0, v=0.0, valid=0.0)
    refresh(m, "", 0, v=0.0, valid=0.0)
    assert m.current is None and not refresh(m, "s3", 2)


def test_seq_ordering_within_a_live_session_still_holds():
    m = ManualIntake()
    refresh(m, "s1", 5, v=0.2)
    assert not refresh(m, "s1", 5, v=0.1)
    assert not refresh(m, "s1", 4, v=0.1)
    assert m.current.v == 0.2


def test_tombstones_are_bounded():
    m = ManualIntake()
    for i in range(ManualIntake.TOMBSTONES + 10):
        refresh(m, f"s{i}", 0, v=0.0, valid=0.0)
    assert len(m._revoked) == ManualIntake.TOMBSTONES


def test_nonfinite_manual_sample_drops_the_held_nonzero_command():
    for bad in (math.nan, math.inf, -math.inf):
        m = ManualIntake()
        refresh(m, "s1", 1, v=0.2)
        assert m.offer(10.0, INST, 3, "s1", 2, 0.2, bad, 0.0)
        assert m.current is None and selected(m).v == 0.0
        m = ManualIntake()
        refresh(m, "s1", 1, v=0.2)
        m.offer(10.0, INST, 3, "s1", 2, bad, 0.2, 0.0)  # nonfinite lifetime
        assert m.current is None


def test_nonfinite_twist_becomes_explicit_zero():
    assert gating.finite_or_zero(1.0, 0.2, 0.1) == gating.Stamped(1.0, 0.2, 0.1)
    for bad in (math.nan, math.inf, -math.inf):
        assert gating.finite_or_zero(1.0, bad, 0.1) == gating.Stamped(1.0, 0.0, 0.0)
        assert gating.finite_or_zero(1.0, 0.2, bad) == gating.Stamped(1.0, 0.0, 0.0)
