"""Review R25 (operations left pending after a failure) and R27 (survey RPC deadlines).

The real Supervisor methods run against a plain object carrying just the state they
touch - no ROS graph, no child processes."""

import types

from amr_bringup import mode_fsm as fsm
from amr_bringup import operations as ops
from amr_bringup.supervisor_node import Supervisor


class Log:
    def error(self, *_):
        pass

    info = warn = error


class Future:
    def __init__(self, done=False, result=None):
        self._done, self._result = done, result

    def done(self):
        return self._done

    def result(self):
        return self._result


class Client:
    def __init__(self):
        self.calls = 0
        self.future = Future()

    def service_is_ready(self):
        return True

    def call_async(self, _req):
        self.calls += 1
        return self.future


def fake():
    sv = types.SimpleNamespace()
    sv.book = ops.OperationBook(None)
    sv.txn = None
    sv.survey_op = None
    sv.inhibit_manual = False
    sv._pending_future = None
    sv.groups = {}
    sv.modes = []
    sv.mode = fsm.MAPPING
    sv.budget = {"save_s": 60.0, "survey_rpc_s": 15.0}
    sv._cli = {"returned": Client(), "abort": Client(), "save": Client()}
    sv.get_logger = Log
    sv._set = lambda mode, phase="", reason="", fault="": sv.modes.append((mode, fault))
    sv._start_worker = lambda fn: None
    for name in ("_fail", "_fail_active", "_step_survey", "_reap"):
        setattr(sv, name, types.MethodType(getattr(Supervisor, name), sv))
    return sv


def test_failure_during_a_survey_finishes_the_survey_operation_itself():
    sv = fake()
    op, _ = sv.book.submit("r1", fsm.REQ_SURVEY_RETURNED)
    sv.survey_op = (op.operation_id, fsm.REQ_SURVEY_RETURNED)
    sv._pending_future = Future()
    sv._fail_active("LOOP_ERROR", "boom")
    assert sv.book.get(op.operation_id).status == ops.FAILED
    assert sv.book.pending is None
    assert (sv.txn, sv.survey_op, sv._pending_future) == (None, None, None)
    assert sv.modes[-1] == (fsm.FAULT, "LOOP_ERROR")


def test_failure_in_a_transaction_finishes_it_and_leaves_nothing_pending():
    sv = fake()
    op, _ = sv.book.submit("r2", fsm.REQ_NAVIGATION)
    sv.txn = fsm.Transaction(op.operation_id, fsm.NAVIGATION, fsm.IDLE, 2)
    sv._fail_active("LOOP_ERROR", "boom")
    assert sv.book.get(op.operation_id).status == ops.FAILED and sv.book.pending is None
    assert sv.txn is None


def test_layer_exit_during_a_survey_does_not_orphan_the_survey_operation():
    sv = fake()
    op, _ = sv.book.submit("r3", fsm.REQ_SURVEY_ABORT)
    sv.survey_op = (op.operation_id, fsm.REQ_SURVEY_ABORT)
    dead = types.SimpleNamespace(
        poll=lambda: 1, requested_stop=False, describe=lambda: "layer pgid 1", empty=True
    )
    sv.groups = {"layer": dead}
    sv._reap(0.0)
    assert sv.book.get(op.operation_id).status == ops.FAILED
    assert sv.book.pending is None


def test_returned_and_abort_calls_have_a_deadline_and_late_answers_are_ignored():
    for kind, name in ((fsm.REQ_SURVEY_RETURNED, "returned"), (fsm.REQ_SURVEY_ABORT, "abort")):
        sv = fake()
        op, _ = sv.book.submit("r", kind)
        sv.survey_op = (op.operation_id, kind)
        sv._step_survey(100.0)  # sends the call
        assert sv._cli[name].calls == 1
        sv._step_survey(114.0)  # still inside the budget
        assert sv.book.get(op.operation_id).pending
        sv._step_survey(115.5)
        assert sv.book.get(op.operation_id).status == ops.FAILED
        assert sv.modes[-1] == (fsm.FAULT, "SURVEY_RPC_TIMEOUT")
        # the late success arrives: nothing holds the future any more, nothing is re-finished
        sv._cli[name].future._done = True
        assert sv._pending_future is None and sv.survey_op is None
        assert sv.book.get(op.operation_id).status == ops.FAILED


def test_q17_recovery_refuses_a_dead_base_with_the_restart_instruction():
    class Dead:
        def poll(self):
            return 1

    class Alive:
        def poll(self):
            return None

    sv = fake()
    sv._lock = __import__("threading").RLock()
    sv.mode = fsm.FAULT
    sv._conditions = lambda: fsm.Conditions(now=10.0)
    sv.fault_code = "BASE_EXITED"
    sv.groups = {"base": Dead()}
    for name in ("_srv_recover", "_base_failure"):
        setattr(sv, name, types.MethodType(getattr(Supervisor, name), sv))
    res = types.SimpleNamespace(success=None, message="")
    sv._srv_recover(None, res)
    assert res.success is False and "systemctl restart amr.service" in res.message
    assert sv.book.pending is None and sv.txn is None
    assert sv.modes[-1][0] == fsm.FAULT
    # a live base with a layer fault goes through the normal transaction
    sv.fault_code = "LAYER_EXITED"
    sv.groups = {"base": Alive()}
    assert sv._base_failure() is None
