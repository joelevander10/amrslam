"""Did the vehicle stop, or was it cut off? (power-loss plan W1)

A clean stop removes a marker file; a power cut cannot. So on every start the
supervisor reads the marker left by the previous run and can say, in one
sentence, what happened - "Power was lost at 14:32 while NAVIGATION, mission m3"
rather than leaving the operator to guess why the drives will not arm.

Telling a POWER CUT from a SERVICE CRASH is the whole point, because the actions
differ: after a cut the drives have latched 8130h and need a power cycle, while a
crash that systemd restarted needs nothing but a look at the log. The test is the
kernel's boot time: a marker written BEFORE this boot began belongs to a previous
boot, so the machine went down; a marker from this boot means only the process
died.

The marker is rewritten on every mode change, so it also carries WHAT was
interrupted. Writes are atomic (tmp + fsync + replace) and never raise: a
read-only or full state dir must not stop the vehicle starting.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass

MARKER = "running.json"
POWER_LOSS, SERVICE_CRASH = "power_loss", "service_crash"


@dataclass(frozen=True)
class Verdict:
    kind: str  # POWER_LOSS | SERVICE_CRASH
    at: float  # wall clock of the last marker write
    mode: str
    operation_id: str
    run_id: str
    instance: str

    @property
    def clock(self) -> str:
        return time.strftime("%H:%M", time.localtime(self.at))

    def sentence(self) -> str:
        """What the operator is told. Names the interrupted work, then the consequence."""
        doing = f" while {self.mode}" if self.mode else ""
        if self.run_id:
            doing += f", mission {self.run_id}"
        elif self.operation_id:
            doing += f", operation {self.operation_id}"
        if self.kind == POWER_LOSS:
            return (
                f"Power was lost at {self.clock}{doing}. The drives lost the PC heartbeat and "
                "latched; they need to be switched off and on."
            )
        return f"The service stopped unexpectedly at {self.clock}{doing} and restarted itself."


def path(state_dir: str) -> str:
    return os.path.join(os.path.expanduser(state_dir), MARKER)


def boot_time() -> float | None:
    """Wall clock when this boot began, from /proc/stat btime. None if unreadable."""
    try:
        with open("/proc/stat", encoding="utf-8") as fh:
            for line in fh:
                if line.startswith("btime "):
                    return float(line.split()[1])
    except (OSError, ValueError, IndexError):
        return None
    return None


def write(state_dir: str, instance: str, mode_name: str, operation_id: str = "", run_id: str = "") -> None:
    """Record what is running now. Atomic and silent: never raises into the loop."""
    p = path(state_dir)
    tmp = f"{p}.tmp"
    body = {
        "written_at": time.time(),
        "instance": instance,
        "mode": mode_name,
        "operation_id": operation_id,
        "run_id": run_id,
        "pid": os.getpid(),
    }
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(body, fh)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, p)
    except OSError:
        try:
            os.unlink(tmp)
        except OSError:
            pass


def clear(state_dir: str) -> None:
    """A clean stop. Anything else leaves the marker, which is what makes it evidence."""
    try:
        os.unlink(path(state_dir))
    except OSError:
        pass


def verdict(state_dir: str, now_wall: float | None = None, boot_wall: float | None = None) -> Verdict | None:
    """Read (and keep) the previous run's marker. None when the last stop was clean,
    the marker is missing, or it is unreadable - an unparseable marker is no evidence,
    and inventing a power cut from it would be worse than silence."""
    try:
        with open(path(state_dir), encoding="utf-8") as fh:
            body = json.load(fh)
        written = float(body["written_at"])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    boot = boot_time() if boot_wall is None else boot_wall
    now = time.time() if now_wall is None else now_wall
    # A marker written before this boot began: the machine itself went down. Without a
    # btime we cannot tell, and the safe assumption is the one whose action is heavier.
    kind = POWER_LOSS if boot is None or written < boot else SERVICE_CRASH
    return Verdict(
        kind=kind,
        at=min(written, now),
        mode=str(body.get("mode", "")),
        operation_id=str(body.get("operation_id", "")),
        run_id=str(body.get("run_id", "")),
        instance=str(body.get("instance", "")),
    )
