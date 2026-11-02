"""The profile's `tracked` flag: which product the vehicle is, and the boot default."""

import types

from agv_core import config

from amr_bringup import mode_fsm as fsm
from amr_bringup import operations as ops
from amr_bringup.supervisor_node import Supervisor


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


def test_the_profile_carries_the_flag_and_it_is_a_bool_or_null():
    assert config.TRACKED in (True, False, None)
    assert "tracked" in config._TOP_LEVEL_SCALARS


def test_a_tape_agv_refuses_maps_and_routes_and_a_trackless_one_refuses_line():
    tape = fsm.Params(tracked=True)
    for req in (fsm.REQ_NAVIGATION, fsm.REQ_SURVEY_START):
        d = fsm.admit(fsm.IDLE, req, cond(), tape)
        assert not d.ok and "tape AGV" in d.reason
    assert fsm.admit(fsm.IDLE, fsm.REQ_LINE, cond(), tape).ok
    assert fsm.admit(fsm.LINE, fsm.REQ_IDLE, cond(), tape).ok  # leaving to jog is allowed

    slam = fsm.Params(tracked=False)
    d = fsm.admit(fsm.IDLE, fsm.REQ_LINE, cond(), slam)
    assert not d.ok and "trackless" in d.reason
    assert fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond(), slam).ok
    assert fsm.admit(fsm.IDLE, fsm.REQ_SURVEY_START, cond(), slam).ok

    # no gate when unset (bench, tests): both products are admitted
    assert fsm.admit(fsm.IDLE, fsm.REQ_LINE, cond()).ok
    assert fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond()).ok


class Log:
    def info(self, *_):
        pass

    warn = error = info


def fake(tracked):
    sv = types.SimpleNamespace()
    sv.tracked = tracked
    sv._auto_line_done = False
    sv.txn = None
    sv.instance = "abcdef0123456789"
    sv.book = ops.OperationBook(None)
    sv.get_logger = Log
    sv.begun = []
    sv._begin_transaction = lambda op, target, map_id, rev, sha, from_fault=False: sv.begun.append(
        (op.kind, target)
    )
    sv._auto_enter_line = types.MethodType(Supervisor._auto_enter_line, sv)
    return sv


def test_a_tape_agv_enters_line_once_at_boot_and_a_trackless_one_stays_idle():
    sv = fake(tracked=True)
    sv._auto_enter_line()
    assert sv.begun == [(fsm.REQ_LINE, fsm.LINE)]
    sv._auto_enter_line()  # a second boot tick, or a later return to IDLE: not again
    assert len(sv.begun) == 1

    sv = fake(tracked=False)
    sv._auto_enter_line()
    assert sv.begun == []


def test_boot_entry_waits_if_a_transaction_is_already_in_flight():
    sv = fake(tracked=True)
    sv.txn = object()
    sv._auto_enter_line()
    assert sv.begun == [] and not sv._auto_line_done


def test_both_products_boot_to_line_and_offer_every_mode():
    """tracked: null (2026-10-02): one vehicle, Run tracked and Run trackless. Tracked
    is the default (2026-10-08): it boots to LINE; trackless is the Home switch."""
    sv = fake(tracked=None)
    sv._auto_enter_line()
    assert sv.begun == [(fsm.REQ_LINE, fsm.LINE)]
    sv._auto_enter_line()
    assert len(sv.begun) == 1, "once per boot: switching to trackless is not undone"
    both = fsm.Params(tracked=None)
    for req in (fsm.REQ_LINE, fsm.REQ_NAVIGATION, fsm.REQ_SURVEY_START):
        assert "tracked=" not in fsm.admit(fsm.IDLE, req, cond(), both).reason


def test_line_mode_leases_manual_too_so_the_selector_decides_who_drives():
    """2026-10-02: selector MANUAL jogs in LINE mode; AUTO is the follower's."""
    sv = types.SimpleNamespace(txn=None, mode=fsm.LINE, inhibit_manual=False)
    allowed = Supervisor._allowed(sv)
    from amr_bringup.supervisor_node import LEASE_LINE, LEASE_MANUAL

    assert allowed == LEASE_MANUAL | LEASE_LINE
