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

from agv_core import config
from agv_core import panel as panel_core  # repo module: core/panel.py


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
    manual_arm: bool = False  # panel.di_manual_arm, debounced; False while invalid


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
            config.PANEL_DI_RESET,
            config.PANEL_DI_START,
            config.PANEL_DI_AUTO,
            scans,
            auto_when_on=config.PANEL_AUTO_WHEN_ON,
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
        # Manual Arm (DI08): a debounced LEVEL, like the pendant. A switch already on at
        # power-on powers the drives once the image is valid - power is not motion.
        self.manual_arm = panel_core.DebouncedLevels((config.PANEL_DI_MANUAL_ARM,), scans)
        self._last_arm: bool | None = None
        self.seq = 0
        self._last_scans: int | None = None  # DIO acquisition count behind the last evaluated frame
        self._last_frame: PanelFrame | None = None
        self._last_valid: bool | None = None
        self._last_mode: str | None = None
        self._last_comms: bool | None = None
        self._last_pendant: panel_core.PendantIntent | None = None

    @staticmethod
    def source_fresh(snapshot: dict) -> bool:
        """Is the DIO image CURRENT physical input? Connected, and a successful scan within
        PANEL_SOURCE_MAX_AGE_S - not the 2 s diagnostic silence age (audit R01)."""
        age = snapshot.get("rx_age_s")
        return (
            bool(snapshot.get("comms_ok"))
            and bool(snapshot.get("connected", True))
            and age is not None
            and age <= config.PANEL_SOURCE_MAX_AGE_S
        )

    def tick(self, snapshot: dict) -> PanelFrame:
        comms = self.source_fresh(snapshot)
        scans = snapshot.get("scans")
        self.seq += 1
        # The node ticks at 50 Hz, the DIO scans at 20 Hz. Debounce, edges and the
        # coincidence hold count ACQUISITIONS: a tick on an image already evaluated
        # republishes the last frame, edges spent.
        if comms and self._last_frame is not None and scans is not None and scans == self._last_scans:
            return PanelFrame(
                valid=self._last_frame.valid,
                mode_auto=self._last_frame.mode_auto,
                start_edge=False,
                reset_edge=False,
                seq=self.seq,
                comms_ok=True,
                pendant=self._last_frame.pendant,
                manual_arm=self._last_frame.manual_arm,
            )
        self._last_scans = scans
        intent = self.scan.scan(snapshot.get("di"), comms)
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
        arm_levels = self.manual_arm.scan(snapshot.get("di"), comms)
        arm = bool(intent.valid and arm_levels is not None and arm_levels[0])
        if arm != self._last_arm:
            if self._last_arm is not None or arm:
                notes.append(f"Manual Arm {'on' if arm else 'off'}")
            self._last_arm = arm
        if self.pendant is not None and pend != self._last_pendant:
            held = [n for n in pend._fields if getattr(pend, n)]
            notes.append("pendant " + (" ".join(held).upper() if held else "released"))
            self._last_pendant = pend
        frame = PanelFrame(
            valid=bool(intent.valid),
            mode_auto=mode == panel_core.AUTO,
            start_edge=bool(intent.start),
            reset_edge=bool(intent.reset),
            seq=self.seq,
            comms_ok=comms,
            changed="; ".join(notes) or None,
            pendant=pend,
            manual_arm=arm,
        )
        self._last_frame = frame
        return frame

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


# AUTO runs that count for the alarm horn: tracked line RUNNING / HOLD, trackless
# EXECUTING / BLOCKED (amr_interfaces LineState / RunState values).
LINE_ACTIVE = (2, 3)
RUN_ACTIVE = (2, 4)
LINE_PREMOVE = 1  # LineState.ARMED: the pre-move warning after a Start


def alarm_wanted(
    mode_auto: bool, line_state: int | None, run_state: int | None,
    field_fresh: bool, protective_clear: bool, warning_active: bool,
    run_premove: bool = False,
) -> bool:
    """dio.alarm_on (operator, 2026-10-02): AUTO RUNNING and (protective stop, warning 1 or
    warning 2). Field state comes from the mux; stale field data does not sound it.

    And the pre-move warning (2026-10-08): from a Start until the vehicle moves (line ARMED,
    RunState.premove) the alarm horn sounds whatever the fields say, then gives way to the
    movement horn once the wheels turn."""
    if mode_auto and (line_state == LINE_PREMOVE or run_premove):
        return True
    running = mode_auto and (line_state in LINE_ACTIVE or run_state in RUN_ACTIVE)
    return running and field_fresh and (not protective_clear or warning_active)


def motion_outputs(turning: bool, alarm: bool, channels, movement_horn) -> dict[int, bool]:
    """dio.motion_on with the horn switch (operator, 2026-10-07): while the alarm horn sounds
    (protective, warning 1 or warning 2 in an AUTO run) the movement horn is OFF, so the
    sound changes; the other motion outputs (DO08) keep following the motors."""
    return {ch: turning and not (alarm and ch in movement_horn) for ch in channels}


def horn_wanted(
    armed: bool, left_rad_s: float, right_rad_s: float, cmd_age_s: float | None, cmd_timeout_s: float
) -> bool:
    """canworker's rule at the ROS boundary: energised AND asked to move, on a fresh command."""
    if not armed or cmd_age_s is None or cmd_age_s > cmd_timeout_s:
        return False
    return (left_rad_s, right_rad_s) != (0.0, 0.0)
