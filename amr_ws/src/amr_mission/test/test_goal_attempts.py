"""R08: goal attempts with a fake ActionClient and deferred futures (no ROS graph)."""

import threading

from amr_mission import goal_attempts as ga


class Future:
    def __init__(self):
        self._done, self._value, self._exc, self._cbs = False, None, None, []

    def add_done_callback(self, cb):
        if self._done:
            cb(self)
        else:
            self._cbs.append(cb)

    def result(self):
        if self._exc is not None:
            raise self._exc
        return self._value

    def set_result(self, value):
        self._done, self._value = True, value
        for cb in self._cbs:
            cb(self)

    def set_exception(self, exc):
        self._done, self._exc = True, exc
        for cb in self._cbs:
            cb(self)


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancels = 0
        self.result_fut = Future()

    def cancel_goal_async(self):
        self.cancels += 1
        return Future()

    def get_result_async(self):
        return self.result_fut


class Result:
    def __init__(self, status):
        self.status = status


class Client:
    def __init__(self):
        self.sent = []  # (goal, response future, feedback callback)

    def send_goal_async(self, goal, feedback_callback=None):
        fut = Future()
        self.sent.append((goal, fut, feedback_callback))
        return fut


def make():
    errors, clock = [], [0.0]
    g = ga.GoalAttempts(threading.RLock(), lambda: clock[0], lambda a, why: errors.append((a, why)))
    return g, Client(), errors, clock


def test_accept_and_succeed():
    g, c, errors, _ = make()
    a = g.send(c, "goal", "run1", 0, 2)
    assert (a.run_id, a.pass_index, a.step_index) == ("run1", 0, 2) and g.outstanding
    h = Handle()
    c.sent[0][1].set_result(h)
    assert g.handle is h and g.result is None
    h.result_fut.set_result(Result(4))
    assert g.result == ga.SUCCEEDED and g.handle is None and not g.outstanding and not errors


def test_pause_before_acceptance_cancels_the_late_accepted_goal():
    g, c, errors, _ = make()
    g.send(c, "goal", "run1", 0, 0)
    g.revoke()  # pause/abort while acceptance is pending
    h = Handle()
    c.sent[0][1].set_result(h)
    assert h.cancels == 1 and g.handle is None and g.current is None
    assert g.obsolete_outstanding()  # still outstanding until terminal
    h.result_fut.set_result(Result(5))
    assert g.result is None and not g.outstanding and not errors


def test_revoke_after_acceptance_cancels_the_handle():
    g, c, _, _ = make()
    g.send(c, "goal", "run1", 0, 0)
    h = Handle()
    c.sent[0][1].set_result(h)
    g.revoke()
    assert h.cancels == 1 and g.handle is None


def test_old_result_after_new_acceptance_does_not_touch_the_new_attempt():
    g, c, errors, _ = make()
    g.send(c, "goal", "run1", 0, 1)
    old = Handle()
    c.sent[0][1].set_result(old)
    g.revoke()  # pause
    g.send(c, "goal", "run1", 0, 1)  # resume, same run and step: a new attempt
    new = Handle()
    c.sent[1][1].set_result(new)
    old.result_fut.set_result(Result(5))  # the cancelled goal reports late
    assert g.handle is new and g.result is None and not errors
    new.result_fut.set_result(Result(4))
    assert g.result == ga.SUCCEEDED


def test_feedback_only_from_the_current_attempt():
    g, c, _, _ = make()
    seen = []
    g.send(c, "goal", "run1", 0, 0, seen.append)
    g.revoke()
    g.send(c, "goal", "run1", 0, 0, seen.append)
    c.sent[0][2]("old")
    c.sent[1][2]("new")
    assert seen == ["new"]


def test_rejection_and_exceptions_of_the_current_attempt_are_errors():
    g, c, errors, _ = make()
    g.send(c, "goal", "run1", 0, 0)
    c.sent[0][1].set_result(Handle(accepted=False))
    assert [w for _, w in errors] == ["action goal rejected"] and g.current is None and not g.outstanding
    g.send(c, "goal", "run1", 0, 0)
    c.sent[1][1].set_exception(RuntimeError("server gone"))
    assert "server gone" in errors[-1][1] and not g.outstanding
    g.send(c, "goal", "run1", 0, 0)
    h = Handle()
    c.sent[2][1].set_result(h)
    h.result_fut.set_exception(RuntimeError("boom"))
    assert "boom" in errors[-1][1] and g.current is None and g.result is None


def test_obsolete_rejection_or_exception_is_silent():
    g, c, errors, _ = make()
    g.send(c, "goal", "run1", 0, 0)
    g.revoke()
    c.sent[0][1].set_result(Handle(accepted=False))
    g.send(c, "goal", "run1", 0, 0)
    g.revoke()
    c.sent[1][1].set_exception(RuntimeError("x"))
    assert not errors and not g.outstanding


def test_send_exception_drops_the_attempt():
    g, _, _, _ = make()

    class Broken:
        def send_goal_async(self, goal, feedback_callback=None):
            raise RuntimeError("no")

    try:
        g.send(Broken(), "goal", "run1", 0, 0)
    except RuntimeError:
        pass
    assert g.current is None and not g.outstanding


def test_forget_obsolete_keeps_the_current_attempt():
    g, c, _, clock = make()
    g.send(c, "goal", "run1", 0, 0)
    g.revoke()
    clock[0] = 3.0
    cur = g.send(c, "goal", "run1", 0, 0)
    assert list(g.obsolete_outstanding().values()) == [0.0]
    g.forget_obsolete()
    assert list(g.outstanding) == [cur]
    h = Handle()
    c.sent[0][1].set_result(h)  # forgotten, but a late acceptance is still cancelled
    assert h.cancels == 1 and g.handle is None


def test_q06_forgotten_attempt_stays_a_barrier_until_terminal_evidence():
    errors = []
    lock = threading.RLock()
    g = ga.GoalAttempts(lock, lambda: 0.0, lambda a, why: errors.append(why))
    c = Client()
    g.send(c, "goal", "run1", 0, 0)
    g.revoke()
    assert g.forget_obsolete() and not g.outstanding
    assert g.barrier() and "run1" in g.barrier()  # unknown outcome: no new motion
    h = Handle()
    c.sent[0][1].set_result(h)  # late acceptance: cancelled, still unresolved
    assert h.cancels == 1 and g.barrier()
    h.result_fut.set_result(Result(5))  # canceled: terminal evidence lifts the barrier
    assert g.barrier() is None and not g.unresolved and not errors


def test_q06_transport_failures_are_unknown_outcomes_not_terminal_ones():
    errors = []
    lock = threading.RLock()
    g = ga.GoalAttempts(lock, lambda: 0.0, lambda a, why: errors.append(why))
    c = Client()
    g.send(c, "goal", "run1", 0, 0)
    c.sent[0][1].set_exception(RuntimeError("no server"))
    assert errors == ["action goal request failed: no server"] and g.barrier()
    g2 = ga.GoalAttempts(lock, lambda: 0.0, lambda a, why: errors.append(why))
    c2 = Client()
    g2.send(c2, "goal", "run2", 0, 0)
    h = Handle()
    c2.sent[0][1].set_result(h)
    h.result_fut.set_exception(RuntimeError("lost"))
    assert errors[-1] == "action result failed: lost" and g2.barrier()
    # a plain rejection IS terminal: the server never held the goal
    g3 = ga.GoalAttempts(lock, lambda: 0.0, lambda a, why: errors.append(why))
    c3 = Client()
    g3.send(c3, "goal", "run3", 0, 0)
    c3.sent[0][1].set_result(Handle(accepted=False))
    assert errors[-1] == "action goal rejected" and g3.barrier() is None
