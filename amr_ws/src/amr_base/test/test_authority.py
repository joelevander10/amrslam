"""P2 (unified plan §12.2): authority expires at both control layers, independently.

Pure, with an injected clock: gating.select is the mux's decision and
gating.drive_gate is the drive owner's. Neither may accept a nonzero command
without a fresh, matching supervisor lease; freshness returning does not
revive a cached command.
"""

from amr_base import gating
from amr_base.gating import (
    FOLLOW,
    LEASE_AUTONOMOUS,
    LEASE_MANUAL,
    MANUAL,
    NONE,
    TELEOP,
    Drives,
    Lease,
    Manual,
    Panel,
    Params,
    Permit,
    Stamped,
    drive_gate,
    nav_topic,
    select,
)

SUP = Params(require_supervisor=True, teleop_enabled=False)
INST = "11111111-aaaa"


def lease(t, gen=3, allowed=LEASE_MANUAL | LEASE_AUTONOMOUS, seq=10, inst=INST):
    return Lease(t_recv=t, instance=inst, generation=gen, seq=seq, allowed=allowed)


def manual(t, v=0.2, gen=3, inst=INST, valid=0.2, seq=5):
    return Manual(t=t, v=v, w=0.0, instance=inst, generation=gen, session="s1", seq=seq, valid_for_s=valid)


MANUAL_PANEL = Panel(t_recv=10.0, valid=True, auto=False)
AUTO_PANEL = Panel(t_recv=10.0, valid=True, auto=True)
DRIVES_OK = Drives(t_recv=10.0, operational=True)


def test_manual_command_needs_fresh_lease_panel_and_drives():
    s = select(10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), manual(10.0), DRIVES_OK)
    assert (s.source, s.v, s.generation, s.inhibited) == (MANUAL, 0.2, 3, False)
    # lease 0.35 s old: gate closed, inhibited, zero
    s = select(10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(9.65), manual(10.0), DRIVES_OK)
    assert (s.source, s.v, s.inhibited) == (NONE, 0.0, True)
    # lease present but no classes allowed (transition barrier)
    s = select(
        10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0, allowed=0), manual(10.0), DRIVES_OK
    )
    assert s.source == NONE and s.inhibited
    # drives stale / not operational
    s = select(10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), manual(10.0), Drives(9.6, True))
    assert s.source == NONE and s.inhibited
    s = select(
        10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), manual(10.0), Drives(10.0, False)
    )
    assert s.source == NONE and s.inhibited
    # no lease at all
    assert select(10.0, None, None, None, None, MANUAL_PANEL, SUP, None, manual(10.0), DRIVES_OK).inhibited


def test_expired_manual_command_is_zero_at_once_and_bounded_by_its_own_lifetime():
    # older than cmd_timeout: gone, no ramp-down of the previous value
    s = select(10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), manual(9.75), DRIVES_OK)
    assert (s.source, s.v) == (NONE, 0.0)
    # carried lifetime shorter than the mux timeout wins
    s = select(
        10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), manual(9.9, valid=0.05), DRIVES_OK
    )
    assert s.source == NONE
    s = select(
        10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), manual(9.9, valid=0.15), DRIVES_OK
    )
    assert s.source == MANUAL


def test_freshness_recovery_cannot_replay_a_cached_command():
    """The lease comes back; the manual command received before the gap stays dead."""
    cmd = manual(10.0)
    assert (
        select(10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), cmd, DRIVES_OK).source == MANUAL
    )
    assert select(10.5, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), cmd, DRIVES_OK).inhibited
    # fresh lease again at 10.6; the cached command is 0.6 s old -> still zero
    s = select(10.6, None, None, None, None, MANUAL_PANEL, SUP, lease(10.6), cmd, Drives(10.6, True))
    assert (s.source, s.v, s.inhibited) == (NONE, 0.0, False)


def test_wrong_instance_or_generation_is_not_authority():
    s = select(
        10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0, gen=4), manual(10.0, gen=3), DRIVES_OK
    )
    assert s.source == NONE
    s = select(
        10.0, None, None, None, None, MANUAL_PANEL, SUP, lease(10.0), manual(10.0, inst="other"), DRIVES_OK
    )
    assert s.source == NONE
    # an executor permit from a replaced layer under AUTO
    follow = Stamped(10.0, 0.3, 0.0)
    old_permit = Permit(10.0, FOLLOW, True, INST, 2, 1)
    s = select(10.0, None, follow, None, old_permit, AUTO_PANEL, SUP, lease(10.0), None, DRIVES_OK)
    assert s.source == NONE and s.inhibited
    new_permit = Permit(10.0, FOLLOW, True, INST, 3, 1)
    s = select(10.0, None, follow, None, new_permit, AUTO_PANEL, SUP, lease(10.0), None, DRIVES_OK)
    assert (s.source, s.v) == (FOLLOW, 0.3)
    # AUTO class withheld by the supervisor
    s = select(
        10.0,
        None,
        follow,
        None,
        new_permit,
        AUTO_PANEL,
        SUP,
        lease(10.0, allowed=LEASE_MANUAL),
        None,
        DRIVES_OK,
    )
    assert s.source == NONE and s.inhibited


def test_engineering_teleop_is_off_in_production_and_on_at_the_bench():
    tele = Stamped(10.0, 0.3, 0.0)
    assert (
        select(10.0, tele, None, None, None, MANUAL_PANEL, SUP, lease(10.0), None, DRIVES_OK).source == NONE
    )
    bench = Params(require_supervisor=False)
    assert select(10.0, tele, None, None, None, MANUAL_PANEL, bench).source == TELEOP
    prod_with_tele = Params(require_supervisor=True, teleop_enabled=True)
    assert (
        select(
            10.0, tele, None, None, None, MANUAL_PANEL, prod_with_tele, lease(10.0), None, DRIVES_OK
        ).source
        == TELEOP
    )


def test_drive_owner_gate_is_independent_of_the_mux():
    sup = Params(require_supervisor=True)
    assert drive_gate(10.0, lease(10.0), 3, sup) is None
    assert drive_gate(10.0, None, 3, sup) == "no supervisor lease"
    assert drive_gate(10.4, lease(10.0), 3, sup) == "no supervisor lease"  # expired
    assert "inhibited" in drive_gate(10.0, lease(10.0, allowed=0), 3, sup)
    assert "generation" in drive_gate(10.0, lease(10.0, gen=4), 3, sup)  # a mux still on the old layer
    assert drive_gate(10.0, None, 0, Params(require_supervisor=False)) is None  # bench


def test_nav_topics_are_private_per_generation():
    assert nav_topic("/cmd_vel", 0) == "/cmd_vel"
    assert nav_topic("/cmd_vel", 7) == "/amr/layers/g7/cmd_vel"  # a token may not start with a digit
    assert nav_topic("/cmd_vel_rotate", 7) == "/amr/layers/g7/cmd_vel_rotate"
    assert gating.NAMES[MANUAL] == "manual"
