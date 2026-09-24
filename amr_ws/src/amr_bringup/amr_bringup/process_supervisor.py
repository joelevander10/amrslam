"""Child process groups for the supervisor (unified plan §3.3). No ROS.

    g = Group.spawn("mapping", ["ros2", "launch", "amr_bringup", "mapping_layer.launch.py", ...])
    g.poll()          -> exit code or None; a launch that ended is a group that ended
    g.alive_pids()    -> every process still in the group's session (leader or not)
    g.stop(...)       -> SIGINT the leader, wait; SIGTERM the group, wait; SIGKILL the
                         group; True only when the group is verifiably EMPTY

Rules:
  * Only an allow-listed executable is spawned, argv as a list, shell=False.
  * Each group is its own session (setsid), so it has a unique process-group
    id and we can signal every member without knowing their PIDs.
  * "Stopped" means no process with our pgid exists any more, checked through
    /proc - a leader that exited while a grandchild lives is NOT stopped.
  * Nothing is killed by name.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field

ALLOWED_EXECUTABLES = ("ros2",)
# Direct module entry points: `ros2 run` does not forward SIGINT to its child,
# so nodes we must stop cleanly are started as `python3 -m <module>` instead.
ALLOWED_MODULES = ("amr_web.web_node", "amr_bringup.supervisor_node")


def _pgid_of(pid: int) -> int | None:
    try:
        with open(f"/proc/{pid}/stat", "rb") as f:
            stat = f.read().decode(errors="replace")
        # field 5 after the last ')' is pgrp
        fields = stat[stat.rindex(")") + 2 :].split()
        return int(fields[2])
    except (OSError, ValueError, IndexError):
        return None


def pids_in_group(pgid: int) -> list[int]:
    out = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        if _pgid_of(int(entry)) == pgid:
            out.append(int(entry))
    return out


@dataclass
class Group:
    role: str
    argv: list[str]
    proc: subprocess.Popen
    pgid: int
    started: float = field(default_factory=time.monotonic)
    requested_stop: bool = False
    exit_code: int | None = None

    @classmethod
    def spawn(cls, role: str, argv: list[str], env: dict | None = None, log_path: str | None = None) -> Group:
        exe = os.path.basename(argv[0]) if argv else ""
        module_entry = len(argv) >= 3 and argv[1] == "-m" and argv[2] in ALLOWED_MODULES
        python = exe.startswith("python3") or argv[:1] == [sys.executable]
        if not argv or not (exe in ALLOWED_EXECUTABLES or (python and module_entry)):
            raise ValueError(f"refusing to spawn {argv[:3]!r}: not an allow-listed executable")
        for a in argv:
            if not isinstance(a, str) or "\0" in a:
                raise ValueError("argv must be plain strings")
        out = open(log_path, "ab", buffering=0) if log_path else subprocess.DEVNULL
        proc = subprocess.Popen(
            argv,
            stdin=subprocess.DEVNULL,
            stdout=out,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,  # own session -> own pgid, signal the whole tree
            close_fds=True,
        )
        if log_path:
            out.close()
        return cls(role=role, argv=list(argv), proc=proc, pgid=proc.pid)

    # -- observation --

    def poll(self) -> int | None:
        if self.exit_code is None:
            code = self.proc.poll()
            if code is not None:
                self.exit_code = code
        return self.exit_code

    def alive_pids(self) -> list[int]:
        return pids_in_group(self.pgid)

    @property
    def leader_alive(self) -> bool:
        return self.poll() is None

    @property
    def empty(self) -> bool:
        self.poll()
        return not self.alive_pids()

    # -- control --

    def _signal_group(self, sig: int) -> None:
        try:
            os.killpg(self.pgid, sig)
        except ProcessLookupError:
            pass

    def _wait_empty(self, deadline: float) -> bool:
        while time.monotonic() < deadline:
            if self.empty:
                return True
            time.sleep(0.05)
        return self.empty

    def stop(self, int_wait_s: float = 6.0, term_wait_s: float = 4.0, kill_wait_s: float = 2.0) -> bool:
        """Bounded escalation. Returns True iff the group is empty afterwards."""
        self.requested_stop = True
        if self.empty:
            return True
        # 1. SIGINT to the launch leader: `ros2 launch` forwards orderly shutdown
        if self.leader_alive:
            try:
                self.proc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        if self._wait_empty(time.monotonic() + int_wait_s):
            return True
        # 2. the leader is gone or ignoring us: TERM every member
        self._signal_group(signal.SIGTERM)
        if self._wait_empty(time.monotonic() + term_wait_s):
            return True
        # 3. last resort
        self._signal_group(signal.SIGKILL)
        return self._wait_empty(time.monotonic() + kill_wait_s)

    def describe(self) -> str:
        code = self.poll()
        leader = "alive" if code is None else str(code)
        return f"{self.role} pgid={self.pgid} leader={leader} members={len(self.alive_pids())}"
