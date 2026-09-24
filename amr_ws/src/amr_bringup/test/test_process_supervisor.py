"""P5 (unified plan §12.2): a failed layer is cleaned before replacement.

Fake worker processes stand in for `ros2 launch` and its nodes: a leader that
spawns a child, and variants that die or ignore SIGINT. ALLOWED_EXECUTABLES is
widened to the interpreter for the test only.
"""

import os
import signal
import sys
import time

import pytest

from amr_bringup import process_supervisor as ps

LEADER = """
import os, signal, subprocess, sys, time
mode = sys.argv[1]
child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
if mode == "ignore-int":
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
if mode == "leader-dies":
    time.sleep(0.3)
    sys.exit(0)          # child keeps running: the classic orphan
def on_int(*_):
    child.terminate(); child.wait(); sys.exit(0)
signal.signal(signal.SIGINT, on_int)
while True:
    time.sleep(0.1)
"""


@pytest.fixture(autouse=True)
def allow_python(monkeypatch):
    monkeypatch.setattr(ps, "ALLOWED_EXECUTABLES", (os.path.basename(sys.executable), "python3", "ros2"))


def spawn(mode: str) -> ps.Group:
    g = ps.Group.spawn("layer", [sys.executable, "-c", LEADER, mode])
    deadline = time.monotonic() + 5
    while len(g.alive_pids()) < 2 and time.monotonic() < deadline:
        time.sleep(0.05)
    assert len(g.alive_pids()) == 2, "leader + child expected"
    return g


def test_clean_sigint_stop_empties_the_group():
    g = spawn("clean")
    assert g.stop(int_wait_s=3) and g.empty and g.alive_pids() == []
    assert g.exit_code == 0


def test_leader_death_with_live_child_is_not_stopped_until_the_child_is_gone():
    """The launch leader exits 0; a node lives on. Not clean, must be reaped."""
    g = spawn("leader-dies")
    deadline = time.monotonic() + 5
    while g.poll() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    assert g.exit_code == 0 and not g.empty, "leader gone, child alive: the orphan case"
    assert len(g.alive_pids()) == 1
    assert g.stop(int_wait_s=0.2, term_wait_s=2) and g.empty


def test_a_child_ignoring_int_and_term_is_killed_within_budget():
    g = spawn("ignore-int")
    t0 = time.monotonic()
    assert g.stop(int_wait_s=0.5, term_wait_s=0.5, kill_wait_s=2)
    assert g.empty and time.monotonic() - t0 < 5


def test_only_allowlisted_executables_and_plain_argv(monkeypatch):
    monkeypatch.setattr(ps, "ALLOWED_EXECUTABLES", ("ros2",))
    with pytest.raises(ValueError, match="allow-listed"):
        ps.Group.spawn("x", ["/bin/sh", "-c", "true"])
    with pytest.raises(ValueError, match="allow-listed"):
        ps.Group.spawn("x", [])
    monkeypatch.setattr(ps, "ALLOWED_EXECUTABLES", ("python3",))
    with pytest.raises(ValueError, match="plain"):
        ps.Group.spawn("x", ["python3", "-c", "x\0"])


def test_signal_to_group_does_not_touch_the_test_process():
    """Own session: killpg on the group must never reach us."""
    g = spawn("clean")
    assert g.pgid != os.getpgid(0)
    os.killpg(g.pgid, signal.SIGTERM)
    time.sleep(0.5)
    assert g.stop(int_wait_s=0.5)
