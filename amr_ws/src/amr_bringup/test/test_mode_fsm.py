"""P1 (unified plan §12.2): mode admission and competing requests. Pure."""

from amr_bringup import mode_fsm as fsm
from amr_bringup import operations as ops


def cond(**kw) -> fsm.Conditions:
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


def test_representative_switches_are_admitted():
    assert fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond()).ok
    assert fsm.admit(fsm.IDLE, fsm.REQ_SURVEY_START, cond()).ok
    assert fsm.admit(fsm.NAVIGATION, fsm.REQ_IDLE, cond(run_state=fsm.RUN_DONE)).ok
    assert fsm.admit(fsm.MAPPING, fsm.REQ_IDLE, cond(survey_state=fsm.SURVEY_SAVED)).ok
    assert fsm.admit(fsm.MAPPING, fsm.REQ_SURVEY_RETURNED, cond(survey_state=fsm.SURVEY_MAPPING)).ok
    assert fsm.admit(fsm.MAPPING, fsm.REQ_SURVEY_SAVE, cond(survey_state=fsm.SURVEY_RETURN_REVIEW)).ok
    assert fsm.admit(fsm.MAPPING, fsm.REQ_SURVEY_ABORT, cond(survey_state=fsm.SURVEY_MAPPING)).ok
    assert fsm.admit(fsm.FAULT, fsm.REQ_RECOVER, cond()).ok


def test_held_job_unsaved_survey_and_stale_wheels_refuse_replacement():
    for held in fsm.JOB_HELD:
        d = fsm.admit(fsm.NAVIGATION, fsm.REQ_IDLE, cond(run_state=held))
        assert not d.ok and "abort" in d.reason
    d = fsm.admit(fsm.NAVIGATION, fsm.REQ_SURVEY_START, cond(run_state=fsm.RUN_FAULT))
    assert not d.ok and "acknowledge" in d.reason
    d = fsm.admit(fsm.MAPPING, fsm.REQ_NAVIGATION, cond(survey_state=fsm.SURVEY_MAPPING))
    assert not d.ok and "not saved" in d.reason
    d = fsm.admit(fsm.MAPPING, fsm.REQ_IDLE, cond(survey_state=fsm.SURVEY_SAVING))
    assert not d.ok and "SAVING" in d.reason
    # stale wheel feedback is not "stopped"; neither is a short dwell
    assert not fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond(wheels_t=99.8)).ok
    assert not fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond(wheels_still_since=99.8)).ok
    assert not fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond(wheels_still_since=None)).ok
    # 2026-10-02: the selector does not gate a mode change; an emergency does
    assert fsm.admit(fsm.IDLE, fsm.REQ_SURVEY_START, cond(panel_manual=False)).ok
    assert fsm.admit(fsm.IDLE, fsm.REQ_SURVEY_START, cond(panel_t=99.5)).ok
    estop = fsm.admit(fsm.IDLE, fsm.REQ_SURVEY_START, cond(drives_ok=False))
    assert not estop.ok and "emergency" in estop.reason
    # save needs stillness, returned does not
    assert not fsm.admit(fsm.MAPPING, fsm.REQ_SURVEY_SAVE, cond(survey_state=1, wheels_still_since=None)).ok
    assert fsm.admit(fsm.MAPPING, fsm.REQ_SURVEY_RETURNED, cond(survey_state=1, wheels_still_since=None)).ok


def test_fault_transitioning_and_pending_operation_block_everything_but_recover():
    for st in (fsm.STARTING, fsm.TRANSITIONING, fsm.STOPPING):
        assert not fsm.admit(st, fsm.REQ_IDLE, cond()).ok
    d = fsm.admit(fsm.FAULT, fsm.REQ_NAVIGATION, cond())
    assert not d.ok and "recover" in d.reason
    assert not fsm.admit(fsm.IDLE, fsm.REQ_RECOVER, cond()).ok
    d = fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond(operation_pending=True))
    assert not d.ok and d.reason.startswith("BUSY")
    d = fsm.admit(fsm.IDLE, fsm.REQ_NAVIGATION, cond(commissioning_active=True))
    assert not d.ok and "commissioning" in d.reason


def test_duplicate_request_returns_existing_operation_and_book_is_bounded(tmp_path):
    book = ops.OperationBook(str(tmp_path / "ops.jsonl"), history=3)
    a, created = book.submit("r1", "mode")
    assert created and book.pending is a
    a2, created2 = book.submit("r1", "mode")
    assert not created2 and a2 is a  # dropped HTTP response, browser retries: same operation
    book.finish(a.operation_id, ops.SUCCEEDED, "ok", map_id="m", map_revision=2)
    assert book.pending is None and book.get(a.operation_id).map_revision == 2
    for i in range(5):
        op, _ = book.submit(f"x{i}", "mode")
        book.finish(op.operation_id, ops.FAILED, "no")
    assert len(book.recent(100)) == 3  # bounded


def test_journal_marks_pending_operations_interrupted_after_restart(tmp_path):
    path = str(tmp_path / "ops.jsonl")
    book = ops.OperationBook(path)
    op, _ = book.submit("r9", "survey_save")
    book.phase(op.operation_id, "serialising")
    del book
    again = ops.OperationBook(path)
    rec = again.get(op.operation_id)
    assert rec is not None and rec.status == ops.INTERRUPTED and "interrupted" in rec.message
    assert again.pending is None and [o.operation_id for o in again.interrupted] == [op.operation_id]
    # the same request id after restart is the interrupted record, not a replay
    same, created = again.submit("r9", "survey_save")
    assert not created and same.status == ops.INTERRUPTED


def test_transaction_steps_in_order():
    t = fsm.Transaction("op", fsm.NAVIGATION, fsm.IDLE, generation=5)
    seen = [t.step]
    while not t.done:
        seen.append(t.advance())
    assert seen == fsm.STEPS


def test_mode_names_cover_every_mode_and_layer_modes_are_named():
    """fsm.MODE_NAMES[...] is subscripted unguarded in the supervisor's serial loop, and
    these values are frozen in ModeState.msg -- a mode added without a name faults the loop."""
    modes = {
        fsm.STARTING,
        fsm.IDLE,
        fsm.MAPPING,
        fsm.NAVIGATION,
        fsm.TRANSITIONING,
        fsm.FAULT,
        fsm.STOPPING,
        fsm.LINE,
    }
    assert modes == set(fsm.MODE_NAMES)
    assert len(set(fsm.MODE_NAMES.values())) == len(fsm.MODE_NAMES)
    # the wire values must not drift: ModeState.msg pins these numbers
    assert (fsm.STARTING, fsm.IDLE, fsm.MAPPING, fsm.NAVIGATION) == (0, 1, 2, 3)
    assert (fsm.TRANSITIONING, fsm.FAULT, fsm.STOPPING, fsm.LINE) == (4, 5, 6, 7)
    assert set(fsm.LAYER_MODES) == {fsm.MAPPING, fsm.NAVIGATION, fsm.LINE}
    assert all(m in fsm.MODE_NAMES for m in fsm.LAYER_MODES)


def test_line_mode_is_admitted_and_an_active_follower_blocks_leaving_it():
    assert fsm.admit(fsm.IDLE, fsm.REQ_LINE, cond()).ok
    assert fsm.admit(fsm.NAVIGATION, fsm.REQ_LINE, cond(run_state=fsm.RUN_IDLE)).ok
    # an unknown kind is still refused
    assert not fsm.admit(fsm.IDLE, "tape", cond()).ok
    # leaving LINE while the follower holds the vehicle is refused, like a held mission
    busy = fsm.admit(fsm.LINE, fsm.REQ_IDLE, cond(line_active=True))
    assert not busy.ok and "disarm" in busy.reason
    assert fsm.admit(fsm.LINE, fsm.REQ_IDLE, cond(line_active=False)).ok
    # the stillness and emergency rules apply to a LINE request like any other;
    # the selector does not (AUTO or MANUAL alike)
    assert not fsm.admit(fsm.IDLE, fsm.REQ_LINE, cond(wheels_still_since=None)).ok
    assert fsm.admit(fsm.IDLE, fsm.REQ_LINE, cond(panel_manual=False)).ok
    assert not fsm.admit(fsm.IDLE, fsm.REQ_LINE, cond(drives_ok=False)).ok
    assert not fsm.admit(fsm.IDLE, fsm.REQ_LINE, cond(commissioning_active=True)).ok


def test_drive_standby_is_not_an_emergency():
    """2026-10-08: the drives are powered only on demand, so a mode change in standby must
    not read as "the drives have no torque"; STO, a fault or no link still does."""
    from amr_bringup.readiness import drives_ok

    ready = "Ready to switch on"
    assert drives_ok(True, "armed", "Operation enabled", "Operation enabled")
    assert drives_ok(False, "standby", ready, ready)
    assert not drives_ok(False, "standby", "Switch on disabled", ready), "STO held"
    assert not drives_ok(False, "standby", ready, "FAULT")
    assert not drives_ok(False, "fault", ready, ready)
    assert not drives_ok(False, "disarmed", ready, ready)
    assert not drives_ok(False, "armed", ready, ready), "armed but not operational: stale or dropped"
    assert not drives_ok(False, "", ready, ready), "a drive report without a link state"
