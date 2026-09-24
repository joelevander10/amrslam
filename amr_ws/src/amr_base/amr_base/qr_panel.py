"""AMR QR operator panel as a PanelState, with a VIRTUAL AUTO/MANUAL selector. Pure.

The QR panel has three inputs on the DIO island and no selector switch:

    qr_base.di_estop   E-stop, normally closed (estop_active_low: open = active)
    panel.di_start     PB START AUTO
    panel.di_reset     PB STOP AUTO

The ROS stack takes ALL motion authority from the panel image (gating.select: MANUAL =
jog/teleop/commissioning, AUTO = the route executor after a physical Start edge), so the
two buttons are turned into the selector level plus the Start/Reset edges it expects:

    START rising, MANUAL   -> selector AUTO (no Start edge: switching modes starts nothing)
    START rising, AUTO     -> Start edge (the executor's physical Start)
    STOP  rising, AUTO     -> selector MANUAL (a running route aborts, as with the switch)
    STOP  rising, MANUAL   -> Reset edge (acknowledges an executor FAULT)
    START + STOP together  -> STOP wins; START is ignored until released

qr_base.auto_hold_s > 0 changes the MANUAL case only: START must be HELD that long to
enter AUTO, and a shorter press is a MANUAL Start edge (sent on release) - what the
Commission page's blind run waits for. With 0 (the default, operator choice 2026-09-23)
a MANUAL Start edge never happens and blind runs cannot be started from this panel.

Trust rules, same as core/panel.PanelScan:
  * a level must read the same on debounce_scans consecutive scans;
  * the first stable image is a BASELINE - a button held at power-on or across a DIO comms
    gap produces no edge until it is released and pressed again (anti-tie-down);
  * no comms / malformed image -> valid=false, and the selector falls back to MANUAL: an
    AUTO that survived a comms gap would be a state nobody chose on the panel.

E-stop does not move the selector (a physical switch would not move either); it takes the
drives out of operation in qr_base_node, which is what the executor already treats as
"no torque" (BLOCKED, resume on Start once released).
"""

from __future__ import annotations

from dataclasses import dataclass

import amr_base.agv_repo  # noqa: F401  (puts the repo layer dirs on sys.path)

import panel as panel_core  # repo module: core/panel.py

MANUAL, AUTO = "manual", "auto"


@dataclass(frozen=True)
class QrPanelFrame:
    valid: bool
    mode_auto: bool
    start_edge: bool
    reset_edge: bool
    estop: bool  # True = E-stop active (or unknown: no trusted image)
    seq: int
    comms_ok: bool
    changed: str | None = None
    pendant: panel_core.PendantIntent = panel_core.PENDANT_IDLE


class VirtualSelector:
    def __init__(
        self,
        di_start: int,
        di_stop: int,
        di_estop: int,
        estop_active_low: bool = True,
        debounce_scans: int = 2,
        auto_hold_s: float = 0.0,
        pendant_channels: tuple[int, int, int, int] | None = None,
    ) -> None:
        self.levels = panel_core.DebouncedLevels((di_start, di_stop, di_estop), debounce_scans)
        self.pendant = (
            panel_core.DebouncedLevels(pendant_channels, debounce_scans) if pendant_channels else None
        )
        self.estop_active_low = estop_active_low
        self.auto_hold_s = max(0.0, float(auto_hold_s))
        self.mode = MANUAL
        self.seq = 0
        self._prev: tuple[bool, bool] | None = None  # (start, stop) after the baseline
        self._start_down_t: float | None = None  # press time of a START held in MANUAL
        self._start_consumed = False  # this press already did its job (hold -> AUTO)
        self._last_valid: bool | None = None
        self._last_comms: bool | None = None
        self._last_estop: bool | None = None

    def tick(self, snapshot: dict, now: float) -> QrPanelFrame:
        comms = bool(snapshot.get("comms_ok"))
        lv = self.levels.scan(snapshot.get("di"), comms)
        self.seq += 1
        notes: list[str] = []
        if comms != self._last_comms:
            notes.append("DIO comms " + ("ok" if comms else f"LOST ({snapshot.get('detail')})"))
            self._last_comms = comms
        valid = lv is not None
        if valid != self._last_valid:
            notes.append("panel image " + ("valid" if valid else "invalid"))
            self._last_valid = valid

        start_edge = reset_edge = False
        estop = True
        if not valid:
            self._prev = None
            self._start_down_t, self._start_consumed = None, False
            if self.mode != MANUAL:
                self.mode = MANUAL
                notes.append("selector MANUAL (no trusted panel image)")
        else:
            start, stop, estop_raw = lv
            estop = (not estop_raw) if self.estop_active_low else bool(estop_raw)
            if self._prev is None:
                self._prev = (start, stop)  # baseline: held buttons are not presses
                self._start_consumed = start
            else:
                p_start, p_stop = self._prev
                self._prev = (start, stop)
                start_rise = start and not p_start
                start_fall = p_start and not start
                stop_rise = stop and not p_stop
                if stop_rise:
                    self._start_down_t = None
                    self._start_consumed = start  # a START held through STOP is spent
                    if self.mode == AUTO:
                        self.mode = MANUAL
                        notes.append("selector MANUAL (STOP)")
                    else:
                        reset_edge = True
                        notes.append("RESET edge (STOP in MANUAL)")
                elif stop:
                    pass  # STOP held: START does nothing
                elif self.mode == AUTO:
                    if start_rise:
                        start_edge = True
                        notes.append("START edge")
                elif self.auto_hold_s <= 0.0:
                    if start_rise:
                        self.mode = AUTO
                        self._start_consumed = True
                        notes.append("selector AUTO (START)")
                else:
                    if start_rise:
                        self._start_down_t, self._start_consumed = now, False
                    if start and not self._start_consumed and self._start_down_t is not None:
                        if now - self._start_down_t >= self.auto_hold_s:
                            self.mode = AUTO
                            self._start_consumed = True
                            notes.append(f"selector AUTO (START held {self.auto_hold_s:g} s)")
                    if start_fall:
                        if not self._start_consumed and self._start_down_t is not None:
                            start_edge = True
                            notes.append("START edge (short press, MANUAL)")
                        self._start_down_t, self._start_consumed = None, False
                if not start and self.mode == AUTO:
                    self._start_consumed = False
        if valid and estop != self._last_estop:
            notes.append("E-STOP " + ("ACTIVE" if estop else "released"))
            self._last_estop = estop
        if not valid:
            self._last_estop = None

        pend = panel_core.PENDANT_IDLE
        if self.pendant is not None and valid:
            levels = self.pendant.scan(snapshot.get("di"), comms)
            if levels is not None:
                pend = panel_core.pendant_intent(*levels)
        return QrPanelFrame(
            valid=valid,
            mode_auto=self.mode == AUTO,
            start_edge=start_edge,
            reset_edge=reset_edge,
            estop=estop,
            seq=self.seq,
            comms_ok=comms,
            changed="; ".join(notes) or None,
            pendant=pend,
        )
