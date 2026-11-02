"""Standing alarms and the one-line vehicle state, computed ONCE, server-side.

Before this module the rail, Status, Run and Alarms pages each re-derived "what is
wrong" from `/api/state` in their own JavaScript, in their own words. They
disagreed, and none of them could say what to press. Here the state snapshot is
turned into catalogue rows (agv_core/alarms.py) exactly once; every page renders
what this returns and invents nothing.

Pure: a dict in, a dict out. `since` bookkeeping lives in the caller (the Flask
app holds a StandingTracker), so this function is trivially testable.
"""

from __future__ import annotations

from typing import Any

from agv_core import alarms as cat

FRESH_S = 0.5  # panel / drives / mux samples older than this are not current
# The scanner runs at 34 Hz. A gap this long is a dead link, not a late frame; the grace
# period stops a page loaded during boot from accusing a scanner that is still starting.
SCAN_STALE_S = 5.0
SCAN_GRACE_S = 20.0

# The headline the operator reads first. Worst wins; the order here IS the priority.
NEEDS_SERVICE = "NEEDS SERVICE"
NOT_READY = "NOT READY"
STOPPED_NEEDS_YOU = "STOPPED (needs you)"
STOPPED_WILL_RESUME = "STOPPED (will resume)"
RUNNING = "RUNNING"
READY = "READY"


def _drive_code(drives: dict | None) -> str | None:
    """A drive row is the one place the operator's action differs by statusword:
    'Switch on disabled' on both drives is the safety chain, not a broken drive."""
    if drives is None:
        return None
    states = f"{drives.get('left', '')} {drives.get('right', '')}"
    if "Fault" in states:
        return "DRIVE_FAULT"
    if not drives.get("operational"):
        if "Switch on disabled" in states:
            return "SAFETY_RESET_NEEDED"
        return None  # disarmed on purpose (IDLE with no one asking to move) is not an alarm
    return None


def standing(state: dict[str, Any]) -> list[dict]:
    """Everything holding the vehicle right now, worst first, as catalogue rows."""
    out: list[dict] = []

    def add(code: str, detail: str = "") -> None:
        out.append(cat.describe(code, detail))

    mode = state.get("mode")
    if not mode or (state.get("mode_age_s") or 0) > 5.0:
        add("SUPERVISOR_DOWN", "no ModeState from the supervisor")
        return out  # nothing else can be trusted without it
    if mode.get("fault_code"):
        add(mode["fault_code"], mode.get("reason", ""))

    panel, panel_age = state.get("panel"), (state.get("panel") or {}).get("age_s")
    if panel is None or (panel_age or 0) > FRESH_S or not panel.get("valid"):
        add("PANEL_STALE", "no fresh, valid panel image" if panel else "no panel image at all")

    drive_code = _drive_code(state.get("drives"))
    if drive_code:
        d = state["drives"]
        add(drive_code, f"{d.get('left', '?')} / {d.get('right', '?')}")

    run = state.get("run")
    if run:
        if run.get("state_name") == "FAULT":
            add(run.get("fault_code") or "EXEC_ACTION_FAILED", run.get("reason", ""))
        elif run.get("state_name") == "BLOCKED":
            add(run.get("fault_code") or cat.hold_code(run.get("hold_cause", "")), run.get("reason", ""))

    line = state.get("line")
    if line and line.get("code"):
        add(line["code"], line.get("message", ""))

    # The scanner is the one failure nothing else reports: its driver is an optional
    # launch member, so an unplugged sensor kills mapping, navigation and localisation
    # while the supervisor still says IDLE and the drives still jog (vehicle, 2026-09-22 -
    # the operator had no way at all to find out). Absence is the evidence, so it is
    # reported after a grace period from web start, not on the first empty poll.
    scan_age = state.get("scan_age_s")
    if (state.get("up_s") or 0.0) > SCAN_GRACE_S:
        if scan_age is None:
            add("SCANNER_SILENT", "no scan since this page's server started")
        elif scan_age > SCAN_STALE_S:
            add("SCANNER_STALE", f"last scan {scan_age:.0f} s ago")

    loc = state.get("localization")
    # Localisation only matters where the vehicle navigates by it: in IDLE (no layer)
    # an UNLOCALIZED monitor is not something the operator has to fix.
    if loc and loc.get("code") and mode.get("mode_name") == "NAVIGATION":
        add(loc["code"], loc.get("reason", ""))

    # Storage (power-loss plan W3). Warned a gigabyte early, because the cure - deleting
    # old maps and reports - is an engineer's job and takes a visit.
    d = state.get("disk") or {}
    if d.get("level") == "stop":
        add("DISK_FULL", f"{d.get('free_mb')} MB free on {d.get('path')}")
    elif d.get("level") == "warn":
        add("DISK_LOW", f"{d.get('free_mb')} MB free on {d.get('path')}")

    mux = state.get("mux")
    if mux and mux.get("inhibited") and mux.get("code"):
        add(mux["code"], mux.get("reason", ""))

    # Worst first, and never the same code twice (a mux inhibit during a supervisor
    # FAULT is the same stop said twice).
    seen, unique = set(), []
    for row in sorted(out, key=lambda r: cat.SEVERITY_ORDER[r["level"]]):
        if row["code"] in seen:
            continue
        seen.add(row["code"])
        unique.append(row)
    return unique


def headline(state: dict[str, Any], rows: list[dict]) -> dict:
    """The one line and the one button for the Home page.

    `rows` is what standing() returned (already worst-first), so the top row decides
    the action; the headline itself comes from what the vehicle is DOING, because
    "RUNNING" with an info-level field stop still reads as a stop to the operator.
    """
    mode = state.get("mode") or {}
    run = state.get("run") or {}
    top = rows[0] if rows else None
    detail = ""

    if top and top["clears_by"] == "restart_service":
        line = NEEDS_SERVICE
    elif mode.get("mode_name") in ("STARTING", "TRANSITIONING") or not mode.get("base_ready"):
        line = NOT_READY
    elif run.get("state_name") == "EXECUTING":
        line = RUNNING
    elif run.get("state_name") == "BLOCKED":
        line = STOPPED_WILL_RESUME if run.get("auto_resume") else STOPPED_NEEDS_YOU
    elif top and top["level"] == "error":
        line = STOPPED_NEEDS_YOU
    elif run.get("state_name") in ("READY", "PAUSED", "DONE"):
        line = f"READY · {run.get('mission_id') or 'no mission'}"
    else:
        line = READY

    if run.get("state_name") in ("EXECUTING", "PAUSED", "BLOCKED") and run.get("mission_id"):
        total = int(run.get("step_index", -1)) + 1
        step = f" · {run.get('step_id')}" if run.get("step_id") else ""
        detail = f"{run['mission_id']} · step {total}{step}"

    return {
        "state": line,
        "detail": detail,
        "action": top["action"] if top else "Nothing to do.",
        "title": top["title"] if top else "The vehicle is fine.",
        "code": top["code"] if top else "",
        "level": top["level"] if top else "info",
        "button": cat.BUTTON.get(top["clears_by"], "") if top else "",
        "clears_by": top["clears_by"] if top else "",
        "run_active": bool(run.get("state_name") in ("READY", "EXECUTING", "PAUSED", "BLOCKED")),
    }


class StandingTracker:
    """Keeps `since` for each standing code across polls, so the Alarms page can say
    'for 4 minutes' and the history can say when it cleared. Not thread-hostile: the
    Flask request threads call it under the app's own GIL-level atomicity only, so it
    takes no lock and does the simplest possible thing."""

    def __init__(self) -> None:
        self._since: dict[str, float] = {}

    def apply(self, rows: list[dict], now: float) -> list[dict]:
        live = {r["code"] for r in rows}
        for code in list(self._since):
            if code not in live:
                del self._since[code]
        for row in rows:
            row["since"] = self._since.setdefault(row["code"], now)
            row["for_s"] = round(now - row["since"], 1)
        return rows
