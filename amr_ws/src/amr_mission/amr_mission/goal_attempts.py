"""Action goal attempts (review R08). Pure: no rclpy import, so it is testable with fakes.

Every goal the executor sends is one attempt, identified by (run, pass, step, attempt).
Only the CURRENT attempt may set the handle, the result or feed progress. Revoking
(pause, block, abort, fault) makes the current attempt obsolete at once, whether or
not its acceptance has arrived: a late acceptance of an obsolete attempt is cancelled
immediately, and its feedback and result are ignored. An attempt stays OUTSTANDING
until the server reports it terminal (rejected, result, or an exception), so the
executor can refuse to issue a replacement goal while an old one may still move the
vehicle, and fault after a bound instead of waiting forever.
"""

from __future__ import annotations

import itertools
import threading
from collections.abc import Callable
from dataclasses import dataclass

SUCCEEDED, CANCELED, ABORTED = "succeeded", "canceled", "aborted"
# action_msgs/GoalStatus values, duplicated so this module stays rclpy-free
_STATUS_SUCCEEDED, _STATUS_CANCELED = 4, 5


@dataclass(frozen=True)
class Attempt:
    run_id: str
    pass_index: int
    step_index: int
    attempt: int


class GoalAttempts:
    def __init__(
        self,
        lock: threading.RLock,
        clock: Callable[[], float],
        on_error: Callable[[Attempt, str], None],
        log: Callable[[str], None] = lambda _m: None,
    ) -> None:
        """`on_error(attempt, why)` is called under `lock` when the CURRENT attempt is
        rejected or its response/result raises; obsolete attempts never reach it."""
        self._lock, self._clock, self._on_error, self._log = lock, clock, on_error, log
        self._seq = itertools.count(1)
        self.current: Attempt | None = None
        self.handle = None
        self.result: str | None = None
        self.sent_t: float | None = None
        # sent, not yet known terminal -> sent time; once revoked, the revoke time
        self.outstanding: dict[Attempt, float] = {}
        # Attempts the executor stopped WAITING for without terminal evidence (review Q06):
        # a cancellation that never reported, a request or result whose transport failed.
        # Their goal may still be executing on the action server, so they stay a barrier
        # against new motion until a late terminal callback resolves them or the layer
        # (server + this node) is replaced. Operator acknowledgement never clears them.
        self.unresolved: set[Attempt] = set()

    # ---- issuing / revoking ------------------------------------------------------------

    def send(self, client, goal, run_id: str, pass_index: int, step_index: int, on_feedback=None) -> Attempt:
        """Call with `lock` held. Raises whatever send_goal_async raises (attempt then dropped)."""
        a = Attempt(run_id, pass_index, step_index, next(self._seq))
        self.current, self.handle, self.result, self.sent_t = a, None, None, self._clock()
        self.outstanding[a] = self.sent_t
        try:
            fut = client.send_goal_async(
                goal, feedback_callback=lambda fb: self._feedback(a, fb, on_feedback)
            )
        except Exception:
            self.outstanding.pop(a, None)
            self.current = None
            raise
        fut.add_done_callback(lambda f: self._response(a, f))
        return a

    def revoke(self) -> None:
        """Call with `lock` held. The current attempt becomes obsolete; cancel it if accepted,
        otherwise its acceptance (if it ever comes) cancels it."""
        a, h = self.current, self.handle
        self.current, self.handle, self.result, self.sent_t = None, None, None, None
        if a is not None and a in self.outstanding:
            self.outstanding[a] = self._clock()  # the cancellation bound runs from here
        if a is not None and h is not None:
            self._cancel(a, h)

    def is_current(self, a: Attempt) -> bool:
        return a is not None and a == self.current

    def obsolete_outstanding(self) -> dict[Attempt, float]:
        return {a: t for a, t in self.outstanding.items() if a != self.current}

    def forget_obsolete(self) -> list[Attempt]:
        """Give up WAITING for obsolete attempts (after a bounded fault); they become unresolved,
        which still bars new motion (Q06). Late callbacks still cancel and can resolve them."""
        gone = [a for a in self.outstanding if a != self.current]
        for a in gone:
            self.outstanding.pop(a, None)
            self.unresolved.add(a)
        return gone

    def barrier(self) -> str | None:
        """Why new motion may not start: an attempt whose outcome is unknown."""
        if self.unresolved:
            a = min(self.unresolved, key=lambda x: x.attempt)
            return (
                f"action goal of run {a.run_id} step {a.step_index} never reported terminal; "
                "its server may still hold it - restart navigation (Stop, then Start)"
            )
        return None

    # ---- callbacks (executor threads) -----------------------------------------------------

    def _cancel(self, a: Attempt, handle) -> None:
        try:
            handle.cancel_goal_async()
        except Exception as e:  # noqa: BLE001
            self._log(f"cancel of attempt {a} raised: {e}")

    def _terminal(self, a: Attempt) -> None:
        """The server said something final (rejected, or a result status): no longer a barrier."""
        self.outstanding.pop(a, None)
        self.unresolved.discard(a)

    def _unknown(self, a: Attempt, why: str) -> None:
        """Transport failed: the server may or may not hold the goal. Stop waiting, keep the barrier."""
        self.outstanding.pop(a, None)
        self.unresolved.add(a)
        self._log(f"attempt {a} outcome unknown: {why}")

    def _response(self, a: Attempt, fut) -> None:
        with self._lock:
            try:
                handle = fut.result()
            except Exception as e:  # noqa: BLE001
                self._unknown(a, f"goal request failed: {e}")
                if self.is_current(a):
                    self.current = None
                    self._on_error(a, f"action goal request failed: {e}")
                return
            if handle is None or not handle.accepted:
                self._terminal(a)
                if self.is_current(a):
                    self.current = None
                    self._on_error(a, "action goal rejected")
                return
            if not self.is_current(a):
                self._log(f"cancelling late-accepted obsolete goal {a}")
                self._cancel(a, handle)
            else:
                self.handle = handle
            try:
                handle.get_result_async().add_done_callback(lambda f: self._result(a, f))
            except Exception as e:  # noqa: BLE001 - the goal is accepted and running, but we cannot watch it
                self._unknown(a, f"result subscription failed: {e}")
                if self.is_current(a):
                    self.current, self.handle = None, None
                    self._on_error(a, f"action result subscription failed: {e}")

    def _result(self, a: Attempt, fut) -> None:
        with self._lock:
            try:
                status = fut.result().status
            except Exception as e:  # noqa: BLE001
                self._unknown(a, f"result failed: {e}")
                if self.is_current(a):
                    self.current, self.handle = None, None
                    self._on_error(a, f"action result failed: {e}")
                return
            self._terminal(a)
            if not self.is_current(a):
                self._log(f"ignoring result of obsolete goal {a}")
                return
            self.result = {_STATUS_SUCCEEDED: SUCCEEDED, _STATUS_CANCELED: CANCELED}.get(status, ABORTED)
            self.handle = None

    def _feedback(self, a: Attempt, fb, on_feedback) -> None:
        with self._lock:
            if on_feedback is not None and self.is_current(a):
                on_feedback(fb)
