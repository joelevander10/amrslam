"""Post-mortem in one press: a zip the operator can name over the phone.

"Verbose" belongs on disk, not on the screen. This packages everything an
engineer would otherwise SSH for - the event log, the last two hours of the
service journal, the profile, the state snapshot and the owners' diagnostics -
into `~/.amr/reports/report-YYYYmmdd-HHMMSS.zip`. The operator reads the file
name out; the engineer collects the file later.

The journal needs a sudoers line (RUNBOOK section 5). Without it the report is
still written, with a note in its place: a partial report beats no report.
"""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
import zipfile

DIRNAME = "reports"
KEEP = 20
JOURNAL_SINCE = "-2h"


def _dir(state_dir: str) -> str:
    return os.path.join(os.path.expanduser(state_dir), DIRNAME)


def _role_logs(state_dir: str) -> list[str]:
    """The live role logs AND the previous run's (`*.log.1`): after a crash or a power
    cut the interesting one is usually the run that ended, not the one that started
    (power-loss plan W2)."""
    log_dir = os.path.join(os.path.expanduser(state_dir), "logs")
    try:
        names = sorted(os.listdir(log_dir))
    except OSError:
        return []
    return [
        os.path.join(log_dir, n)
        for n in names
        if (n.endswith(".log") or n.endswith(".log.1")) and not n.startswith("events")
    ]


def _journal() -> str:
    cmd = ["sudo", "-n", "journalctl", "-u", "amr.service", "--since", JOURNAL_SINCE, "--no-pager"]
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=30)  # noqa: S603
    except (OSError, subprocess.SubprocessError) as e:
        return f"(journal unavailable: {e})\n"
    if p.returncode != 0:
        return (
            "(journal unavailable: the sudoers line for journalctl is not installed; "
            f"see RUNBOOK section 5)\n\n{(p.stderr or '').strip()}\n"
        )
    return p.stdout


_NAME_LOCK = threading.Lock()
_CLAIMED: set[str] = set()  # names being written right now


def build(
    state_dir: str,
    state: dict,
    events: list[dict],
    diagnostics: dict,
    event_files: list[str],
    note: str = "",
) -> str:
    """Write one report; returns its file name. Raises OSError only if the reports
    directory itself cannot be written - a missing input is recorded, not fatal."""
    out_dir = _dir(state_dir)
    os.makedirs(out_dir, exist_ok=True)
    # Unique even for two requests in the same second, and written to a staging name that
    # listing() ignores, then renamed: a listed report is always complete (audit R18).
    with _NAME_LOCK:
        stamp = time.strftime("%Y%m%d-%H%M%S")
        name = f"report-{stamp}.zip"
        n = 1
        while os.path.exists(os.path.join(out_dir, name)) or name in _CLAIMED:
            n += 1
            name = f"report-{stamp}-{n}.zip"
        _CLAIMED.add(name)
    path = os.path.join(out_dir, name)
    staging = os.path.join(out_dir, f".{name}.{os.getpid()}.{threading.get_ident()}.part")
    with zipfile.ZipFile(staging, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("note.txt", note or "(no note)")
        z.writestr("state.json", json.dumps(state, indent=2, default=str))
        z.writestr("events.json", json.dumps(events, indent=2, default=str))
        z.writestr("diagnostics.json", json.dumps(diagnostics, indent=2, default=str))
        z.writestr("journal.txt", _journal())
        for src in list(event_files) + _role_logs(state_dir):
            try:
                z.write(src, os.path.join("logs", os.path.basename(src)))
            except OSError as e:
                z.writestr(f"logs/{os.path.basename(src)}.missing", str(e))
        for src in (
            os.path.join(os.path.expanduser(state_dir), "operations.jsonl"),
            os.path.join(os.path.expanduser("~/agv_can"), "profiles", "agv-01.json"),
        ):
            if os.path.exists(src):
                try:
                    z.write(src, os.path.basename(src))
                except OSError:
                    pass
    os.replace(staging, path)
    with _NAME_LOCK:
        _CLAIMED.discard(name)
    _prune(out_dir)
    return name


def _prune(out_dir: str) -> None:
    try:
        names = sorted(n for n in os.listdir(out_dir) if n.startswith("report-") and n.endswith(".zip"))
    except OSError:
        return
    for name in names[:-KEEP]:
        try:
            os.remove(os.path.join(out_dir, name))
        except OSError:
            pass


def listing(state_dir: str) -> list[dict]:
    out_dir = _dir(state_dir)
    rows = []
    try:
        names = os.listdir(out_dir)
    except OSError:
        return rows
    for name in sorted(names, reverse=True):
        if not (name.startswith("report-") and name.endswith(".zip")):
            continue
        full = os.path.join(out_dir, name)
        try:
            stat = os.stat(full)
        except OSError:
            continue
        rows.append({"name": name, "bytes": stat.st_size, "t": stat.st_mtime})
    return rows


def path_of(state_dir: str, name: str) -> str | None:
    """Resolve a report name to a path, refusing anything that is not a plain name in
    the reports directory (a name comes from a URL)."""
    if not name or "/" in name or "\\" in name or not name.startswith("report-") or not name.endswith(".zip"):
        return None
    path = os.path.join(_dir(state_dir), name)
    return path if os.path.exists(path) else None
