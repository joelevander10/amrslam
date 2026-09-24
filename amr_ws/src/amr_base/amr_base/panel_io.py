"""Operator panel and horn over the DIO island (T12), framework-free.

Reuses the repo's own pieces unchanged: core/panel.PanelScan for the
debounced Reset/Start edges and the AUTO/MANUAL level (anti-tie-down at
power-on, re-baseline after a comms gap - see its docstring), and
drivers/dio.DioLink for the Modbus scan thread and the expiring coil claim.

    PanelAdapter.tick(now, snapshot)  -> PanelFrame  (what to publish)
    horn_wanted(...)                  -> bool        (what the coil should be)

The jog pendant rides in the same frame: core/panel.DebouncedLevels over the
four PENDANT_DI_* channels, resolved by core/panel.pendant_intent (an opposing
pair cancels). Levels, reported as-is; the mux turns them into a twist.

Policy that lives here, and nowhere else on the ROS side:

  * The panel image is VALID only while DIO comms are good AND PanelScan has a
    debounced baseline. Anything else publishes valid=false, which the mux and
    the executor treat as no authority at all (gating.Panel).
  * The horn follows the COMMANDED wheel setpoint while the drives are armed -
    the same rule canworker used: high whenever the vehicle is energised and
    asked to move, manual or auto alike, low otherwise. It is a claim renewed
    every tick with config.HORN_HOLD_S, so a dead node drops the coil on the
    next DIO scan. Nothing gates on the horn; a horn that failed cannot stop a
    move (README, safety model).
  * A stale wheel command (older than cmd_timeout_s) counts as zero, so the
    horn cannot outlive the command stream that asked for it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from amr_base.agv_repo import config

import panel as panel_core  # repo module: core/panel.py


@dataclass(frozen=True)
class PanelFrame:
    valid: bool
    mode_auto: bool
    start_edge: bool
    reset_edge: bool
    seq: int
    comms_ok: bool
    changed: str | None = None  # human-readable transition for the log, if any
    pendant: panel_core.PendantIntent = panel_core.PENDANT_IDLE


class PanelAdapter:
    """Turns DioLink snapshots into PanelFrames. One instance per node."""

    def __init__(
        self,
        debounce_scans: int | None = None,
        pendant: bool | None = None,
        coincidence_hold_scans: int | None = None,
    ) -> None:
        scans = config.PANEL_DEBOUNCE_SCANS if debounce_scans is None else debounce_scans
        # Coincidence guard: the selector and the pendant changing in the SAME debounced scan
        # is not a hand (2026-09-17: MANUAL+FWD appeared together for 1.02 s mid-route with
        # nobody at the box, aborting the run and handing the mux a phantom FWD). Both changes
        # are withheld - the previous selector and an idle pendant are published - until the
        # new image has persisted this many scans; if it reverts first, nothing happened.
        if coincidence_hold_scans is None:
            hold_s, period = config.PANEL_COINCIDENCE_HOLD_S, config.DIO_SCAN_PERIOD_S
            coincidence_hold_scans = int(math.ceil(hold_s / period))
        self.hold_scans = max(0, coincidence_hold_scans)
        self._held: tuple[str, panel_core.PendantIntent] | None = None  # (mode, pendant) being withheld
        self._held_for = 0
        self.scan = panel_core.PanelScan(
            config.PANEL_DI_RESET, config.PANEL_DI_START, config.PANEL_DI_AUTO, scans
        )
        self.pendant = None
        if config.PENDANT_ENABLED if pendant is None else pendant:
            self.pendant = panel_core.DebouncedLevels(
                (
                    config.PENDANT_DI_FWD,
                    config.PENDANT_DI_RVS,
                    config.PENDANT_DI_LEFT,
                    config.PENDANT_DI_RIGHT,
                ),
                scans,
            )
        self.seq = 0
        self._last_valid: bool | None = None
        self._last_mode: str | None = None
        self._last_comms: bool | None = None
        self._last_pendant: panel_core.PendantIntent | None = None

    def tick(self, snapshot: dict) -> PanelFrame:
        comms = bool(snapshot.get("comms_ok"))
        intent = self.scan.scan(snapshot.get("di"), comms)
        self.seq += 1
        notes = []
        if comms != self._last_comms:
            notes.append("DIO comms " + ("ok" if comms else f"LOST ({snapshot.get('detail')})"))
            self._last_comms = comms
        if intent.valid != self._last_valid:
            notes.append("panel image " + ("valid" if intent.valid else "invalid"))
            self._last_valid = intent.valid
        # (the selector note is written by _coincidence, once the change is believed)
        if intent.start:
            notes.append("START edge")
        if intent.reset:
            notes.append("RESET edge")

        pend = panel_core.PENDANT_IDLE
        if self.pendant is not None:
            levels = self.pendant.scan(snapshot.get("di"), comms)
            if levels is not None:
                pend = panel_core.pendant_intent(*levels)
        mode, pend = self._coincidence(intent, pend, notes)
        if self.pendant is not None and pend != self._last_pendant:
            held = [n for n in pend._fields if getattr(pend, n)]
            notes.append("pendant " + (" ".join(held).upper() if held else "released"))
            self._last_pendant = pend
        return PanelFrame(
            valid=bool(intent.valid),
            mode_auto=mode == panel_core.AUTO,
            start_edge=bool(intent.start),
            reset_edge=bool(intent.reset),
            seq=self.seq,
            comms_ok=comms,
            changed="; ".join(notes) or None,
            pendant=pend,
        )

    def _coincidence(self, intent, pend, notes) -> tuple[str, panel_core.PendantIntent]:
        """The (mode, pendant) to publish this scan, withholding a simultaneous change."""
        if not intent.valid:
            self._held, self._held_for = None, 0
            return intent.mode, pend
        prev_mode = self._last_mode if self._last_mode is not None else intent.mode
        prev_pend = self._last_pendant if self._last_pendant is not None else panel_core.PENDANT_IDLE
        mode_changed = intent.mode != prev_mode
        pend_pressed = any(pend) and pend != prev_pend
        if self._held is None and self.hold_scans > 0 and mode_changed and pend_pressed:
            self._held, self._held_for = (intent.mode, pend), 1
            notes.append(
                f"panel image suspect: selector {intent.mode.upper()} and pendant changed together; withheld"
            )
            return prev_mode, panel_core.PENDANT_IDLE
        if self._held is not None:
            if (intent.mode, pend) != self._held:
                self._held, self._held_for = None, 0  # reverted or moved on: it never counted
                notes.append("panel image suspect: cleared")
            else:
                self._held_for += 1
                if self._held_for < self.hold_scans:
                    return prev_mode, panel_core.PENDANT_IDLE
                self._held, self._held_for = None, 0
                notes.append("panel image suspect: persisted, accepted")
        if intent.mode != self._last_mode:
            notes.append(f"selector {intent.mode.upper()}")
            self._last_mode = intent.mode
        return intent.mode, pend


def horn_wanted(
    armed: bool, left_rad_s: float, right_rad_s: float, cmd_age_s: float | None, cmd_timeout_s: float
) -> bool:
    """canworker's rule at the ROS boundary: energised AND asked to move, on a fresh command."""
    if not armed or cmd_age_s is None or cmd_age_s > cmd_timeout_s:
        return False
    return (left_rad_s, right_rad_s) != (0.0, 0.0)
