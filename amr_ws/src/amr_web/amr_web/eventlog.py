"""The operator event log that survives a restart.

The event ring lived only in the web adapter's memory, so every service restart
threw away the reason the vehicle stopped - and "it stopped yesterday" had only
journalctl and base.log as evidence, both engineer-only. This appends every event
to `~/.amr/logs/events.jsonl` and reloads the tail on start, so the Alarms page's
history is still there after a restart, a crash or a power cut.

One line per event, size-rotated (5 MB x 5 files). Nothing here may raise into a
ROS callback: a full disk must not stop the vehicle reporting its state, so every
operation is best-effort and failures are counted, not thrown.
"""

from __future__ import annotations

import json
import os
import threading

MAX_BYTES = 5 * 1024 * 1024
KEEP = 5


class EventLog:
    def __init__(self, path: str, max_bytes: int = MAX_BYTES, keep: int = KEEP) -> None:
        self.path = os.path.expanduser(path)
        self.max_bytes = max_bytes
        self.keep = keep
        self.errors = 0
        self._lock = threading.Lock()
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
        except OSError:
            self.errors += 1

    def append(self, event: dict, sync: bool = False) -> None:
        """Append one event. `sync` forces it to the platter before returning.

        Only ERROR events are synced (power-loss plan W4): they are exactly the ones
        you want to read after a cut, and they are rare. Syncing every info event would
        put an fsync on a path that fires on each jog and mode change, for a record
        nobody reads."""
        line = json.dumps(event, separators=(",", ":"), default=str)
        with self._lock:
            try:
                self._rotate_if_needed(len(line) + 1)
                with open(self.path, "a", encoding="utf-8") as fh:
                    fh.write(line + "\n")
                    if sync:
                        fh.flush()
                        os.fsync(fh.fileno())
            except OSError:
                self.errors += 1

    def _rotate_if_needed(self, incoming: int) -> None:
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return
        if size + incoming <= self.max_bytes:
            return
        # events.jsonl -> .1 -> .2 ... ; the oldest falls off the end
        for n in range(self.keep - 1, 0, -1):
            src, dst = f"{self.path}.{n}", f"{self.path}.{n + 1}"
            if os.path.exists(src):
                os.replace(src, dst)
        os.replace(self.path, f"{self.path}.1")

    def tail(self, limit: int = 300) -> list[dict]:
        """The newest `limit` events, oldest first. A truncated or half-written last
        line (a power cut mid-append) is skipped, never raised."""
        out: list[dict] = []
        for path in (f"{self.path}.1", self.path):
            try:
                with open(path, encoding="utf-8") as fh:
                    for raw in fh:
                        raw = raw.strip()
                        if not raw:
                            continue
                        try:
                            out.append(json.loads(raw))
                        except ValueError:
                            continue
            except OSError:
                continue
        return out[-limit:]

    def files(self) -> list[str]:
        """Every log file that exists, newest first: what a report packages."""
        names = [self.path] + [f"{self.path}.{n}" for n in range(1, self.keep + 1)]
        return [p for p in names if os.path.exists(p)]
