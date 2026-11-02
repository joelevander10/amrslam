"""Role logs that cannot eat the disk, and state that does not outlive its generation.

`~/.amr/logs/base.log` was append-only and unbounded: 43 MB on the vehicle by
2026-09-22, every service start since install in one file, and the only thing
standing between the state directory and a full root partition was how often the
vehicle happened to be reinstalled. A full disk is a power-loss problem too - it
is the other way a write is lost.

Two mechanisms, deliberately separate:

  rotate()  at spawn time, so every service start and every layer transition
            begins a fresh file and a report can carry the PREVIOUS run's log.
  cap()     while a child is running, because a single bad run can fill a disk
            between restarts. Children open the log with O_APPEND, so truncating
            underneath them is safe: the next write lands at the new end of file
            rather than at a stale offset (that is exactly what O_APPEND means).

Nothing here raises. A log that cannot be rotated is not a reason to refuse to
start the vehicle.
"""

from __future__ import annotations

import os
import time

KEEP = 5
MAX_BYTES = 50 * 1024 * 1024


def rotate(path: str, keep: int = KEEP) -> None:
    """base.log -> base.log.1 -> ... -> base.log.<keep>; the oldest falls off."""
    try:
        if not os.path.exists(path):
            return
        for n in range(keep - 1, 0, -1):
            src, dst = f"{path}.{n}", f"{path}.{n + 1}"
            if os.path.exists(src):
                os.replace(src, dst)
        os.replace(path, f"{path}.1")
    except OSError:
        pass


def cap(path: str, max_bytes: int = MAX_BYTES) -> bool:
    """Truncate a live log that has grown past `max_bytes`. True when it truncated.

    The marker line goes in AFTER the truncate, so a reader of the shortened file can
    see that something was removed rather than silently reading a beheaded log."""
    try:
        if os.path.getsize(path) <= max_bytes:
            return False
        os.truncate(path, 0)
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(path, "a", encoding="utf-8") as fh:
            fh.write(f"[{stamp}] --- truncated here: the log passed {max_bytes // (1024 * 1024)} MB ---\n")
        return True
    except OSError:
        return False


def cap_dir(directory: str, max_bytes: int = MAX_BYTES) -> list[str]:
    """Cap every live role log in a directory. Returns the ones truncated."""
    out = []
    try:
        names = os.listdir(directory)
    except OSError:
        return out
    for name in sorted(names):
        if name.endswith(".log") and cap(os.path.join(directory, name), max_bytes):
            out.append(name)
    return out


def sweep_generation_files(state_dir: str, keep_generation: int | None = None) -> list[str]:
    """Delete `costmap_footprint_gen<N>.yaml` files that no live layer owns.

    They are written per control generation and never removed, so the state directory
    accumulated one per mode change for the life of the install. At boot nothing is
    live, so `keep_generation=None` removes them all."""
    removed = []
    try:
        names = os.listdir(os.path.expanduser(state_dir))
    except OSError:
        return removed
    for name in sorted(names):
        if not (name.startswith("costmap_footprint_gen") and name.endswith(".yaml")):
            continue
        digits = name[len("costmap_footprint_gen") : -len(".yaml")]
        if keep_generation is not None and digits == str(keep_generation):
            continue
        try:
            os.remove(os.path.join(os.path.expanduser(state_dir), name))
            removed.append(name)
        except OSError:
            pass
    return removed
