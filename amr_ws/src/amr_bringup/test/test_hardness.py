"""Phase 3 hardness: what happens when a piece of the stack dies.

The mechanisms are tested here without ROS or processes - a fake Group and a fake
clock - because the interesting behaviour is the POLICY (respawn how often, give
up when, say what), not the spawning. The end-to-end version lives in
test_fault_injection_sim.py and is opt-in.
"""

import os
import types

from amr_bringup import sdnotify
from amr_bringup.supervisor_node import RESPAWN_BACKOFF_S, RESPAWN_TRIES, Supervisor


class Log:
    def __init__(self):
        self.lines = []

    def warn(self, s):
        self.lines.append(("warn", s))

    def error(self, s):
        self.lines.append(("error", s))

    info = warn


class Dead:
    requested_stop = False

    def poll(self):
        return 1

    def describe(self):
        return "web pgid=123"


class Alive:
    requested_stop = False

    def poll(self):
        return None

    def describe(self):
        return "web pgid=456"


def sv_with_web(monkeypatch, spawn_result=None, spawn_raises=None):
    sv = types.SimpleNamespace()
    sv.groups = {"web": Dead()}
    sv._respawnable = {"web": ["python3", "-m", "amr_web.web_node"]}
    sv._respawn_at, sv._respawn_tries = {}, {}
    sv._logs = "/tmp"
    sv._env = lambda: {}
    sv.events = []
    sv._event = lambda level, code, text: sv.events.append((code, text))
    sv.log = Log()
    sv.get_logger = lambda: sv.log
    sv.spawned = []

    def spawn(role, argv, env=None, log_path=None):
        sv.spawned.append(role)
        if spawn_raises:
            raise spawn_raises
        return spawn_result or Alive()

    import amr_bringup.supervisor_node as node  # noqa: PLC0415

    # scoped to the test: Group.spawn is the real thing again afterwards
    monkeypatch.setattr(node.Group, "spawn", staticmethod(spawn))
    for name in ("_reap", "_respawn_due"):
        setattr(sv, name, types.MethodType(getattr(Supervisor, name), sv))
    return sv


def test_a_dead_web_comes_back_after_the_backoff_not_immediately(monkeypatch):
    """The web is the operator's only window; it was also the least protected process."""
    sv = sv_with_web(monkeypatch)
    sv._reap(100.0)
    assert sv.spawned == [] and "web" not in sv.groups  # not instantly: a crash loop must not spin
    sv._respawn_due(100.0 + RESPAWN_BACKOFF_S - 0.1)
    assert sv.spawned == []
    sv._respawn_due(100.0 + RESPAWN_BACKOFF_S)
    assert sv.spawned == ["web"] and "web" in sv.groups


def test_a_web_that_keeps_dying_gives_up_and_says_so(monkeypatch):
    sv = sv_with_web(monkeypatch)
    now = 100.0
    for _ in range(RESPAWN_TRIES):
        sv.groups["web"] = Dead()
        sv._reap(now)
        now += RESPAWN_BACKOFF_S
        sv._respawn_due(now)
    assert len(sv.spawned) == RESPAWN_TRIES
    sv.groups["web"] = Dead()
    sv._reap(now)
    sv._respawn_due(now + RESPAWN_BACKOFF_S)
    assert len(sv.spawned) == RESPAWN_TRIES  # no fourth attempt
    assert sv.events and sv.events[-1][0] == "WEB_DOWN"


def test_a_failed_respawn_is_retried_rather_than_lost(monkeypatch):
    sv = sv_with_web(monkeypatch, spawn_raises=OSError("no such executable"))
    sv._reap(100.0)
    sv._respawn_due(100.0 + RESPAWN_BACKOFF_S)
    assert "web" in sv._respawn_at  # scheduled again, not dropped on the floor


def test_a_dead_base_is_still_a_fault_and_is_never_respawned(monkeypatch):
    """Respawning the base would re-arm the drives behind the operator's back."""
    sv = sv_with_web(monkeypatch)
    sv.groups = {"base": Dead()}
    sv.failed = []
    sv._fail_active = lambda code, why: sv.failed.append(code)
    sv._reap(100.0)
    assert sv.failed == ["BASE_EXITED"] and sv.spawned == []


def test_the_watchdog_notifier_is_a_no_op_without_systemd(monkeypatch):
    monkeypatch.delenv("NOTIFY_SOCKET", raising=False)
    n = sdnotify.Notifier()
    assert not n.enabled
    n.ready(), n.watchdog(), n.stopping()  # must not raise off the vehicle


def test_the_watchdog_notifier_pings_the_socket(tmp_path, monkeypatch):
    import socket  # noqa: PLC0415

    path = str(tmp_path / "notify.sock")
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    srv.bind(path)
    srv.settimeout(2.0)
    monkeypatch.setenv("NOTIFY_SOCKET", path)
    monkeypatch.setenv("WATCHDOG_USEC", "30000000")
    n = sdnotify.Notifier()
    assert n.enabled and abs(n.interval_s - 15.0) < 1e-6  # ping at HALF WatchdogSec
    n.ready()
    assert srv.recv(64) == b"READY=1"
    n.watchdog()
    assert srv.recv(64) == b"WATCHDOG=1"
    srv.close()
    os.unlink(path)
