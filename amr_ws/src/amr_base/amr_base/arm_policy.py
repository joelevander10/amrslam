"""Drive power policy (2026-10-08): when drive_node powers the drives. Pure and clock-fed.

    MANUAL   powered while the Manual Arm input (panel.di_manual_arm, DI08) is HIGH
    AUTO     powered while the active layer asks for it: LineState.drive_power (tape) or
             RunState.drive_power (trackless). They ask from the pre-move warning after
             a Start until the run ends or stops in a hold that waits for a human
    else     standby: CANopen up, encoders and the MLS read, power stage off

The drives are no longer powered just because the vehicle is idle. Standby keeps the
wheel counts flowing, so "at Home", station parking and a vehicle pushed by hand stay
measured while nothing is powered.

This is NOT a safety function and decides nothing about whether moving is safe: the
FX3 takes STO in hardware whatever this says, and motion still needs the panel, the
lease, the permit and a fresh command (gating.py). This only keeps the power stage off
while nobody is asking for it. Every unknown answers "no power": a stale or invalid
panel, a stale request, an unknown selector.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Panel:
    t_recv: float  # monotonic receive time
    valid: bool
    mode_auto: bool
    manual_arm: bool


@dataclass(frozen=True)
class Ask:
    """One layer's request. t_stamp is the message's own stamp on the monotonic clock, so a
    latched message replayed from a layer that has since died is not fresh."""

    source: str  # "line" | "run"
    t_stamp: float
    drive_power: bool


@dataclass(frozen=True)
class Params:
    panel_timeout_s: float = 1.0  # /amr/panel_state is 50 Hz
    ask_timeout_s: float = 2.5  # LineState is on change + 1 Hz, RunState on change + 2 Hz


def _fresh(t: float | None, now: float, timeout: float) -> bool:
    # A timestamp from the future is a clock error, not freshness.
    return t is not None and 0.0 <= now - t <= timeout


def want_power(
    now: float, panel: Panel | None, asks: tuple[Ask, ...] | list[Ask], p: Params
) -> tuple[bool, str]:
    """(power wanted, why) for this instant."""
    if panel is None:
        return False, "no panel state"
    if not _fresh(panel.t_recv, now, p.panel_timeout_s):
        return False, "panel state stale"
    if not panel.valid:
        return False, "panel image invalid"
    if not panel.mode_auto:
        if panel.manual_arm:
            return True, "MANUAL, Manual Arm on"
        return False, "MANUAL, Manual Arm (DI08) off"
    for a in asks:
        if a.drive_power and _fresh(a.t_stamp, now, p.ask_timeout_s):
            return True, f"AUTO, the {a.source} run asks for power"
    return False, "AUTO, no run asking for power"
