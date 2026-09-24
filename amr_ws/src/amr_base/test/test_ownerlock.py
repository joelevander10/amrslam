"""P8 (unified plan §12.2): a duplicate device owner is refused before any I/O."""

import os
import subprocess
import sys

import pytest

import amr_base.agv_repo  # noqa: F401  (repo modules on sys.path)

import ownerlock


@pytest.fixture
def lockdir(tmp_path, monkeypatch):
    monkeypatch.setenv("AMR_LOCK_DIR", str(tmp_path))
    return tmp_path


def test_second_owner_is_refused_and_named(lockdir):
    first = ownerlock.acquire("can", "drive_node")
    assert ownerlock.holder("can").startswith("drive_node")
    with pytest.raises(ownerlock.OwnerBusy, match="drive_node"):
        ownerlock.acquire("can", "agv_controller")
    # a different resource is independent
    dio = ownerlock.acquire("dio", "panel_node")
    first.release()
    assert ownerlock.holder("can") is None
    ownerlock.acquire("can", "agv_controller").release()
    dio.release()


def test_lock_dies_with_its_process_and_is_not_inherited(lockdir):
    """Another process holds it -> refused; that process exits -> free. A child
    spawned by the holder must not keep it (close-on-exec)."""
    code = (
        "import os, sys, time, subprocess\n"
        "sys.path.insert(0, sys.argv[1]); import ownerlock\n"
        "l = ownerlock.acquire('can', 'holder')\n"
        "child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])\n"
        "print(child.pid, flush=True)\n"
        "time.sleep(float(sys.argv[2]))\n"
    )
    core = os.path.join(os.path.dirname(ownerlock.__file__))
    holder = subprocess.Popen(
        [sys.executable, "-c", code, core, "3"],
        stdout=subprocess.PIPE,
        text=True,
        env=dict(os.environ, AMR_LOCK_DIR=str(lockdir)),
    )
    child_pid = int(holder.stdout.readline())
    try:
        with pytest.raises(ownerlock.OwnerBusy, match="holder"):
            ownerlock.acquire("can", "me")
        holder.wait(timeout=10)
        # holder exited, its sleeping child is still alive: the lock must be FREE
        assert ownerlock.holder("can") is None
        ownerlock.acquire("can", "me").release()
    finally:
        try:
            os.kill(child_pid, 9)
        except ProcessLookupError:
            pass
        holder.kill()


def test_unknown_resource_is_rejected(lockdir):
    with pytest.raises(ValueError):
        ownerlock.acquire("lidar")
