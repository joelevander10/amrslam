"""Blind-run evidence history and the operator's tape measurement, for /commissioning.

The commissioning node writes one evidence JSON per job into
<state_dir>/commissioning/ (commissioning_node.write_evidence). This module only
READS those, and writes one sidecar per run: <evidence stem>.measured.json, the
position the operator measured on the floor. Commanded vs measured is then a table,
per backend - which is the whole point of the PV-vs-PP comparison.

Path safety: a run is named by its evidence file NAME only (no directories), the
name must match the evidence pattern, the resolved path must stay inside the
evidence directory, and the evidence file must already exist.
"""

from __future__ import annotations

import json
import math
import os
import re
import time

NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,160}\.json")
SIDECAR = ".measured.json"
LIMITS = {"dx_mm": 20000.0, "dy_mm": 20000.0, "heading_deg": 720.0}
NOTE_MAX = 500


class LogError(ValueError):
    pass


def evidence_dir(state_dir: str) -> str:
    return os.path.join(os.path.expanduser(state_dir), "commissioning")


def _path(directory: str, name: str) -> str:
    if not isinstance(name, str) or not NAME.fullmatch(name) or name.endswith(SIDECAR):
        raise LogError("run must be an evidence file name")
    base = os.path.realpath(directory)
    path = os.path.realpath(os.path.join(base, name))
    if os.path.dirname(path) != base:
        raise LogError("run is outside the evidence directory")
    return path


def _sidecar(path: str) -> str:
    return path[: -len(".json")] + SIDECAR


def commanded(evidence: dict) -> dict | None:
    """The single segment's commanded pose change (mm / deg), or None for multi-segment."""
    plan = evidence.get("plan") or {}
    segs = plan.get("segments") or []
    if len(segs) != 1:
        return None
    c = segs[0].get("commanded") or {}
    try:
        return {
            "kind": segs[0].get("kind", ""),
            "spec": segs[0].get("spec", {}),
            "dx_mm": 1000.0 * float(c["dx_m"]),
            "dy_mm": 1000.0 * float(c["dy_m"]),
            "heading_deg": float(c["heading_deg"]),
        }
    except (KeyError, TypeError, ValueError):
        return None


def _row(name: str, ev: dict, measured: dict | None) -> dict:
    cmd = commanded(ev)
    plan = ev.get("plan") or {}
    row = {
        "run": name,
        "plan_id": ev.get("plan_id", ""),
        "backend": ev.get("backend", "pv"),
        "phase": ev.get("phase", ""),
        "reason": ev.get("reason", ""),
        "speed_mps": plan.get("speed_mps"),
        "commanded": cmd,
        "gyro_heading_deg": ev.get("gyro_heading_deg"),
        "pp_error_counts": (ev.get("pp_result") or {}).get("error_counts"),
        "measured": measured,
        "error": None,
    }
    if cmd and measured:
        row["error"] = {k: measured[k] - cmd[k] for k in ("dx_mm", "dy_mm", "heading_deg")}
    return row


def history(directory: str, limit: int = 100) -> list[dict]:
    """Newest first. Unreadable files are listed with their error, not dropped."""
    if not os.path.isdir(directory):
        return []
    names = [n for n in os.listdir(directory) if NAME.fullmatch(n) and not n.endswith(SIDECAR)]
    names.sort(key=lambda n: os.path.getmtime(os.path.join(directory, n)), reverse=True)
    out = []
    for n in names[:limit]:
        path = os.path.join(directory, n)
        try:
            with open(path, encoding="utf-8") as fh:
                ev = json.load(fh)
        except (OSError, ValueError) as e:
            out.append({"run": n, "error_reading": str(e)})
            continue
        measured = None
        side = _sidecar(path)
        if os.path.isfile(side):
            try:
                with open(side, encoding="utf-8") as fh:
                    measured = json.load(fh)
            except (OSError, ValueError):
                measured = None
        out.append(_row(n, ev, measured))
    return out


def save_measurement(directory: str, run: str, values: dict) -> dict:
    """Validate and store the tape measurement for `run`; -> the history row."""
    path = _path(directory, run)
    if not os.path.isfile(path):
        raise LogError(f"no such run: {run}")
    rec = {}
    for key, lim in LIMITS.items():
        try:
            v = float(values.get(key))
        except (TypeError, ValueError):
            raise LogError(f"{key} must be a number") from None
        if not math.isfinite(v) or abs(v) > lim:
            raise LogError(f"{key} must be finite and within +/-{lim:g}")
        rec[key] = v
    note = values.get("note", "")
    if not isinstance(note, str) or len(note) > NOTE_MAX:
        raise LogError(f"note must be text of at most {NOTE_MAX} characters")
    rec["note"] = note
    rec["measured_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    side = _sidecar(path)
    tmp = f"{side}.tmp-{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(rec, fh, indent=1)
    os.replace(tmp, side)  # a re-measurement replaces the previous one, whole
    with open(path, encoding="utf-8") as fh:
        ev = json.load(fh)
    return _row(run, ev, rec)
