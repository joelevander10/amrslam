"""qr_base_node end to end against simulated hardware (tools/qr_base_node_sim.py).

Subprocess: the harness stubs rclpy and the message packages for the whole process."""

import os
import subprocess
import sys

import pytest

HARNESS = os.path.join(os.path.dirname(__file__), "..", "tools", "qr_base_node_sim.py")


def test_qr_base_node_against_simulated_hardware():
    pytest.importorskip("can")
    r = subprocess.run([sys.executable, HARNESS], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]
    assert "0 failed" in r.stdout
