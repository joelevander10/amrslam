"""Supervisor mode rules (unified plan §5). Pure: no ROS, no processes, no clock
of its own - every decision takes a Conditions snapshot the caller assembled.

    admit(state, request, cond)  -> Decision(ok, reason)   request admission (§5.2)
    Transaction.step(...)        -> the common transaction (§5.3) as a step list

The FSM decides; the node performs. Fault handling and shutdown never come
through admit(): they bypass the stillness rule to inhibit and stop (§5.2).
"""

from __future__ import annotations

from dataclasses import dataclass, field

# ModeState.mode values (frozen in amr_interfaces/msg/ModeState.msg)
STARTING, IDLE, MAPPING, NAVIGATION, TRANSITIONING, FAULT, STOPPING = range(7)
MODE_NAMES = {
    STARTING: "STARTING",
    IDLE: "IDLE",
    MAPPING: "MAPPING",
    NAVIGATION: "NAVIGATION",
    TRANSITIONING: "TRANSITIONING",
    FAULT: "FAULT",
    STOPPING: "STOPPING",
}

# request kinds
REQ_IDLE = "idle"
REQ_NAVIGATION = "navigation"
REQ_SURVEY_START = "survey_start"
REQ_SURVEY_RETURNED = "survey_returned"
REQ_SURVEY_SAVE = "survey_save"
REQ_SURVEY_ABORT = "survey_abort"
REQ_RECOVER = "recover"

# executor RunState values that hold a job (RunState.msg)
RUN_IDLE, RUN_READY, RUN_EXECUTING, RUN_PAUSED, RUN_BLOCKED, RUN_FAULT, RUN_DONE = range(7)
JOB_HELD = {RUN_READY, RUN_EXECUTING, RUN_PAUSED, RUN_BLOCKED}
# survey MappingState values
SURVEY_IDLE, SURVEY_MAPPING, SURVEY_RETURN_REVIEW, SURVEY_SAVING, SURVEY_SAVED = range(5)
SURVEY_UNSAVED = {SURVEY_MAPPING, SURVEY_RETURN_REVIEW}


@dataclass(frozen=True)
class Params:
    still_wheel_rad_s: float = 0.02
    still_dwell_s: float = 0.5
    wheels_fresh_s: float = 0.10
    panel_fresh_s: float = 0.20


DEFAULT = Params()


@dataclass
class Conditions:
    """Everything admission looks at, as of `now`. None = never received."""

    now: float
    wheels_t: float | None = None  # last valid /wheel_states
    wheels_still_since: float | None = None  # both |w| <= threshold continuously since
    panel_t: float | None = None
    panel_valid: bool = False
    panel_manual: bool = False
    run_state: int | None = None  # executor RunState.state, current generation only
    survey_state: int | None = None  # MappingState.state, current generation only
    operation_pending: bool = False
    commissioning_active: bool = False


@dataclass(frozen=True)
class Decision:
    ok: bool
    reason: str = ""


def wheels_still(c: Conditions, p: Params = DEFAULT) -> bool:
    fresh = c.wheels_t is not None and c.now - c.wheels_t <= p.wheels_fresh_s
    return fresh and c.wheels_still_since is not None and c.now - c.wheels_still_since >= p.still_dwell_s


def panel_manual(c: Conditions, p: Params = DEFAULT) -> bool:
    return c.panel_t is not None and c.now - c.panel_t <= p.panel_fresh_s and c.panel_valid and c.panel_manual


def admit(state: int, request: str, c: Conditions, p: Params = DEFAULT) -> Decision:
    """Request admission (§5.2). Refusals name the first unmet rule."""
    if c.operation_pending:
        return Decision(False, "BUSY: another operation is in progress")
    if state in (STARTING, STOPPING, TRANSITIONING):
        return Decision(False, f"not in a stable mode ({MODE_NAMES[state]})")

    if request == REQ_RECOVER:
        if state != FAULT:
            return Decision(False, "recover only applies in FAULT")
        return Decision(True)
    if state == FAULT:
        return Decision(False, "FAULT: recover to IDLE first")

    # survey operations that do not replace a layer
    if request in (REQ_SURVEY_RETURNED, REQ_SURVEY_SAVE):
        if state != MAPPING:
            return Decision(False, "no survey in progress")
        if request == REQ_SURVEY_SAVE and not wheels_still(c, p):
            return Decision(False, "save needs the vehicle stopped (fresh, still wheels for 0.5 s)")
        return Decision(True)
    if request == REQ_SURVEY_ABORT:
        if state != MAPPING:
            return Decision(False, "no survey in progress")
        return Decision(True)

    # mode replacement: idle / navigation / survey_start
    if request not in (REQ_IDLE, REQ_NAVIGATION, REQ_SURVEY_START):
        return Decision(False, f"unknown request {request!r}")
    if request == REQ_IDLE and state == IDLE:
        return Decision(True, "already IDLE")
    if not wheels_still(c, p):
        return Decision(False, "vehicle not proven stopped (fresh, still wheels for 0.5 s)")
    if not panel_manual(c, p):
        return Decision(False, "mode change needs a valid panel in MANUAL")
    if c.commissioning_active:
        return Decision(False, "a commissioning job is prepared or running; clear it first")
    if state == NAVIGATION and c.run_state in JOB_HELD:
        return Decision(False, "a mission is READY/EXECUTING/PAUSED/BLOCKED; abort it first")
    if state == NAVIGATION and c.run_state == RUN_FAULT:
        return Decision(False, "executor FAULT: acknowledge it first")
    if state == MAPPING:
        if c.survey_state in SURVEY_UNSAVED:
            return Decision(False, "survey not saved: save or abort it first")
        if c.survey_state == SURVEY_SAVING:
            return Decision(False, "survey is SAVING")
    return Decision(True)


# --- the common transaction (§5.3) -------------------------------------------

INHIBIT, WAIT_MUX_ACK, WAIT_STILL, STOP_OLD, CLEAR_CACHES, START_NEW, WAIT_READY, COMMIT = (
    "inhibit",
    "wait_mux_ack",
    "wait_still",
    "stop_old",
    "clear_caches",
    "start_new",
    "wait_ready",
    "commit",
)
STEPS = [INHIBIT, WAIT_MUX_ACK, WAIT_STILL, STOP_OLD, CLEAR_CACHES, START_NEW, WAIT_READY, COMMIT]


@dataclass
class Transaction:
    """One mode replacement in flight. The node advances `step`; budgets are
    deadlines in the node's monotonic clock, set when a step is entered."""

    operation_id: str
    target: int  # IDLE | MAPPING | NAVIGATION
    from_mode: int
    generation: int  # the NEW generation, rotated at INHIBIT
    step: str = INHIBIT
    deadline: float | None = None
    map_id: str = ""
    map_revision: int = 0
    map_sha256: str = ""
    notes: list[str] = field(default_factory=list)

    def advance(self) -> str:
        i = STEPS.index(self.step)
        self.step = STEPS[min(i + 1, len(STEPS) - 1)]
        return self.step

    @property
    def done(self) -> bool:
        return self.step == COMMIT
