"""Route execution state machine (spec §7.1). Pure: transitions and their reasons only.

The node owns goals, poses and timers; this owns what state the run is in,
which run id is current, and whether a Start edge may continue. Every method
returns True if the transition happened, so callers can log refusals.
"""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass, field

IDLE, READY, EXECUTING, PAUSED, BLOCKED, FAULT, DONE = 0, 1, 2, 3, 4, 5, 6
NAMES = {
    IDLE: "IDLE",
    READY: "READY",
    EXECUTING: "EXECUTING",
    PAUSED: "PAUSED",
    BLOCKED: "BLOCKED",
    FAULT: "FAULT",
    DONE: "DONE",
}
ACTIVE = (EXECUTING, PAUSED, BLOCKED)
MAX_PASSES = 100  # repeat_count bound: a route pass count above this is refused at load


_counter = itertools.count(1)


def new_run_id() -> str:
    return time.strftime("%Y%m%d-%H%M%S") + f"-{next(_counter)}"


@dataclass
class RunFsm:
    state: int = IDLE
    reason: str = "no mission loaded"
    mission_id: str = ""
    run_id: str = ""
    n_steps: int = 0
    step_index: int = -1  # within the current pass
    n_passes: int = 1
    pass_index: int = 0
    resume_prepared: bool = False
    history: list[tuple[int, str]] = field(default_factory=list)

    def _set(self, state: int, reason: str) -> bool:
        self.state, self.reason = state, reason
        self.history.append((state, reason))
        return True

    # ---- loading -------------------------------------------------------------------

    def load(self, mission_id: str, n_steps: int, passes: int = 1) -> bool:
        """A mission may be (re)loaded when nothing is running: IDLE, READY or DONE.
        `passes` is the route's repeat_count: the step list is executed that many times."""
        if self.state not in (IDLE, READY, DONE):
            self.reason = f"cannot load while {NAMES[self.state]}"
            return False
        if type(passes) is not int or not 1 <= passes <= MAX_PASSES:
            self.reason = f"cannot load: repeat_count must be an integer 1..{MAX_PASSES}, got {passes!r}"
            return False
        if type(n_steps) is not int or n_steps < 1:
            self.reason = f"cannot load: route has no steps ({n_steps!r})"
            return False
        self.mission_id, self.n_steps, self.n_passes = mission_id, n_steps, passes
        self.run_id, self.step_index, self.pass_index, self.resume_prepared = "", -1, 0, False
        return self._set(
            READY, f"mission {mission_id} loaded ({n_steps} steps x {passes} passes); AUTO + Start to run"
        )

    def progress(self) -> str:
        """Human-readable pass/step position (RunState has no pass field yet)."""
        return f"pass {self.pass_index + 1}/{self.n_passes} step {self.step_index}"

    def unready(self, reason: str) -> bool:
        """A prerequisite lapsed while waiting (nothing was moving): back to IDLE, no acknowledgement."""
        if self.state != READY:
            return False
        return self._set(IDLE, reason)

    # ---- start / resume -------------------------------------------------------------

    def start(self, auto: bool, gate_ok: bool, gate_reason: str = "") -> bool:
        """Physical Start edge. From READY it begins a run; from PAUSED/BLOCKED with a
        prepared resume it continues the remaining step."""
        if self.state == READY:
            if not auto:
                self.reason = "Start ignored: selector is not AUTO"
                return False
            if not gate_ok:
                self.reason = f"Start refused: {gate_reason}"
                return False
            self.run_id, self.step_index, self.pass_index = new_run_id(), 0, 0
            return self._set(EXECUTING, f"run {self.run_id} started")
        if self.state in (PAUSED, BLOCKED):
            if not auto:
                self.reason = "Start ignored: selector is not AUTO"
                return False
            if not self.resume_prepared:
                self.reason = f"Start ignored: resume not prepared ({NAMES[self.state]})"
                return False
            self.resume_prepared = False
            return self._set(EXECUTING, f"resumed {self.progress()}")
        self.reason = f"Start ignored in {NAMES[self.state]}"
        return False

    def auto_resume(self, reason: str) -> bool:
        """Continue a BLOCKED run without a Start edge (auto-resume plan 2026-09-19): the
        executor calls this once the cause cleared and every resume check held long enough.
        Never from PAUSED: an operator's own Pause waits for the operator."""
        if self.state != BLOCKED:
            return False
        self.resume_prepared = False
        return self._set(EXECUTING, f"{reason} ({self.progress()})")

    def prepare_resume(self, ok: bool, why: str) -> bool:
        if self.state not in (PAUSED, BLOCKED):
            self.reason = f"nothing to resume ({NAMES[self.state]})"
            return False
        self.resume_prepared = ok
        self.reason = "resume prepared: press Start to continue" if ok else f"resume refused: {why}"
        return ok

    def invalidate_resume(self, why: str) -> None:
        if self.resume_prepared:
            self.resume_prepared = False
            self.reason = f"prepared resume withdrawn: {why}"

    # ---- interruptions ----------------------------------------------------------------

    def pause(self, reason: str = "paused by operator") -> bool:
        if self.state != EXECUTING:
            self.reason = f"pause ignored in {NAMES[self.state]}"
            return False
        self.resume_prepared = False
        return self._set(PAUSED, reason)

    def block(self, reason: str) -> bool:
        if self.state != EXECUTING:
            return False
        self.resume_prepared = False
        return self._set(BLOCKED, reason)

    def abort(self, reason: str) -> bool:
        """Operator abort or MANUAL takeover: the run is discarded."""
        if self.state not in (READY, *ACTIVE):
            self.reason = f"abort ignored in {NAMES[self.state]}"
            return False
        self.run_id, self.step_index, self.pass_index, self.resume_prepared = "", -1, 0, False
        return self._set(IDLE, reason)

    def fault(self, reason: str) -> bool:
        if self.state == FAULT:
            return False
        self.resume_prepared = False
        return self._set(FAULT, reason)

    def ack(self) -> bool:
        """Acknowledgement only clears the latch; a new load + Start is required."""
        if self.state != FAULT:
            self.reason = f"nothing to acknowledge ({NAMES[self.state]})"
            return False
        self.run_id, self.step_index, self.pass_index = "", -1, 0
        return self._set(IDLE, "fault acknowledged; load a mission")

    # ---- progress -----------------------------------------------------------------------

    def step_done(self) -> bool:
        if self.state != EXECUTING:
            return False
        self.step_index += 1
        if self.step_index >= self.n_steps:
            if self.pass_index + 1 >= self.n_passes:
                return self._set(DONE, f"run {self.run_id} complete ({self.n_passes} passes)")
            self.pass_index, self.step_index = self.pass_index + 1, 0
        self.reason = f"{self.progress()} of {self.n_steps}"
        return True

    def accepts(self, run_id: str) -> bool:
        """Callbacks from an old run are ignored (spec §7.2)."""
        return bool(run_id) and run_id == self.run_id and self.state in ACTIVE
