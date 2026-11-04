"""The PC's emergency picture (emergency-suite plan §3, owner answers 2026-10-30). Pure, clock-fed.

    NORMAL        clear
    ACTIVE        DI08 LOW, or the panel image invalid/stale: release the E-stop, press Reset
    RESET_NEEDED  DI08 HIGH again, no Reset yet: press Reset
    RESETTING     Reset seen with DI08 HIGH: waiting for the drives to leave STO
    RESET_FAILED  the drives still held STO panel.estop_reset_timeout_s after the Reset

Inputs per tick: the panel image valid AND fresh, DI08 as a level filtered by the adapter
(HIGH over debounce_scans, LOW over estop_low_scans), the Reset edge, and whether the drives report STO
held ("Switch on disabled", sto_held(); None when the drives cannot be observed).

    from          when                                         to
    any           panel not ok, or DI08 LOW                    ACTIVE (immediate)
    ACTIVE        DI08 HIGH, a Reset edge within the timeout   RESETTING      (latched Q3)
    ACTIVE        DI08 HIGH, no recent Reset                   RESET_NEEDED   (unlatched Q3)
    ACTIVE*       DI08 HIGH, drives not holding STO            NORMAL         (* panel was only blind)
    ACTIVE*       DI08 HIGH, drives holding STO                RESET_NEEDED
    RESET_NEEDED  Reset edge                                   RESETTING
    RESETTING     drives not holding STO, or unobservable      NORMAL
    RESETTING     still holding STO at the deadline            RESET_FAILED
    RESET_FAILED  Reset edge                                   RESETTING
    RESET_FAILED  drives leave STO (a late reset)              NORMAL

Works whichever way the Flexi Soft drives Q3 (bench question Q1):
  * unlatched: Q3 follows the button. Press -> ACTIVE, twist out -> RESET_NEEDED, Reset ->
    RESETTING -> NORMAL once the drives leave Switch on disabled.
  * latched: Q3 stays LOW until the PLC is reset, so the PC cannot tell "still pressed" from
    "released". The Reset edge then lands while DI08 is still LOW and changes nothing on its
    own, but it is remembered for reset_timeout_s; when DI08 rises inside that window the
    state goes straight to RESETTING, so one press is enough. RESET_NEEDED is then seen only
    if the Reset was missed or held longer than the window.
A Reset with the E-stop still in does nothing either way; with an unlatched Q3 and a release
inside the window it ends in RESET_FAILED ("press Reset again"), which is the truth.

"Blind" (the image invalid or stale, DI08 never seen LOW in this episode: boot, a DIO
comms gap) is ACTIVE too - fail-safe - but needs no Reset when the image returns HIGH
unless the drives show STO held, which an E-stop pressed and released while blind leaves.

This is the PC's picture only. STO is the Flexi Soft's, in hardware; nothing here can
delay or replace it. Consumers: arm_policy (no power), gating (no motion), the supervisor
(no lease), the executor (abort), panel_node (all outputs LOW), the web (the banner).
"""

from __future__ import annotations

from dataclasses import dataclass

# amr_interfaces/msg/PanelState EMG_* (the contract)
NORMAL, ACTIVE, RESET_NEEDED, RESETTING, RESET_FAILED = 0, 1, 2, 3, 4
NAMES = {
    NORMAL: "NORMAL",
    ACTIVE: "EMERGENCY",
    RESET_NEEDED: "RESET_NEEDED",
    RESETTING: "RESETTING",
    RESET_FAILED: "RESET_FAILED",
}
STO_STATE = "Switch on disabled"  # CiA-402, both drives while the Flexi Soft holds HWTO
STO_LINKS = ("standby", "armed")  # drive_node link states whose drive states are evidence


def sto_held(left_state: str, right_state: str, link_state: str, age_s: float | None,
             fresh_s: float = 0.5) -> bool | None:
    """Do the drives say STO is held? None when they cannot say: no fresh DriveStatus, or a
    link (disarmed, fault, no bus) whose cached states are not evidence. Either drive in
    Switch on disabled counts as held - STO takes both, so one that has not come out yet
    means the reset has not finished."""
    if age_s is None or not 0.0 <= age_s <= fresh_s or link_state not in STO_LINKS:
        return None
    if not left_state or not right_state:
        return None
    return STO_STATE in (left_state, right_state)


@dataclass(frozen=True)
class Inputs:
    panel_ok: bool  # PanelState valid and fresh
    estop_ok: bool  # DI08 HIGH, debounced HIGH-going only (False while invalid)
    reset_edge: bool
    sto: bool | None  # sto_held()


class Emergency:
    """One per panel image owner. Starts ACTIVE and blind: nothing is known at boot."""

    def __init__(self, reset_timeout_s: float) -> None:
        self.timeout = float(reset_timeout_s)
        self.state = ACTIVE
        self.tripped = False  # DI08 seen LOW on a valid image during this episode
        self.since = 0.0  # monotonic time of the last transition
        self._reset_t: float | None = None  # last Reset edge while ACTIVE (latched Q3)
        self._deadline = 0.0

    def step(self, now: float, i: Inputs) -> str | None:
        """Advance; returns the transition text for the log, or None."""
        if not i.panel_ok or not i.estop_ok:
            if i.panel_ok and not self.tripped:
                self.tripped = True
                if self.state == ACTIVE:
                    return self._go(now, ACTIVE, "DI08 LOW (E-stop)")  # was blind, now known
            if i.panel_ok and i.reset_edge and self.state == ACTIVE:
                self._reset_t = now  # does nothing now; a latched Q3 rises after it
            if self.state != ACTIVE:
                self._reset_t = None
                why = "DI08 LOW (E-stop)" if i.panel_ok else "panel image invalid or stale"
                return self._go(now, ACTIVE, why)
            return None
        s = self.state
        if s == ACTIVE:
            if not self.tripped:
                if i.sto:
                    return self._go(now, RESET_NEEDED, "panel back, drives hold STO")
                return self._go(now, NORMAL, "panel back, DI08 HIGH")
            if i.reset_edge or (self._reset_t is not None and now - self._reset_t <= self.timeout):
                return self._resetting(now, "DI08 HIGH after Reset")
            return self._go(now, RESET_NEEDED, "DI08 HIGH (E-stop released)")
        if s in (RESET_NEEDED, RESET_FAILED) and i.reset_edge:
            return self._resetting(now, "Reset pressed")
        if s == RESETTING:
            if i.sto is not True:
                why = "drives out of STO" if i.sto is False else "drives not observable"
                return self._go(now, NORMAL, why)
            if now >= self._deadline:
                return self._go(now, RESET_FAILED, f"drives still in STO {self.timeout:g} s after Reset")
        if s == RESET_FAILED and i.sto is False:
            return self._go(now, NORMAL, "drives out of STO (late)")
        return None

    def _resetting(self, now: float, why: str) -> str:
        self._deadline = now + self.timeout
        return self._go(now, RESETTING, why)

    def _go(self, now: float, state: int, why: str) -> str:
        prev = self.state
        self.state, self.since = state, now
        self._reset_t = None
        if state == NORMAL:
            self.tripped = False
        return f"emergency {NAMES[prev]} -> {NAMES[state]}: {why}"
