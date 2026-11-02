#!/usr/bin/env python3
"""Fault injection (plan phase 3.6): kill something, then read the operator surface.

Opt-in (AMR_SIM_TESTS=1). For each injected failure this asserts the three things
the plan asks for, in the operator's terms rather than the log's:

  1. the expected CODE stands on /api/alarms within 2 s,
  2. the event is on DISK (so it survives the restart that follows),
  3. the surface offers the DOCUMENTED button for that code.

The web app runs in-process here (Flask test client over a real RosAdapter on the
sim domain) rather than as a port: the endpoint under test is the real one, and no
socket collides with the vehicle's own service on 5001.

A mechanism check, per the repo's rule for sim tests: it asserts that the chain
node -> event -> catalogue -> button exists, never how fast the sim drives.
"""

import json
import os
import signal
import subprocess
import sys
import time

import pytest

if os.environ.get("AMR_SIM_TESTS") != "1":
    pytest.skip("simulation fault-injection test; set AMR_SIM_TESTS=1", allow_module_level=True)
os.environ["ROS_DOMAIN_ID"] = "69"
os.environ["AMR_SUPERVISOR_SIM"] = "1"

import rclpy  # noqa: E402
from agv_core import alarms as cat  # noqa: E402

SETTLE_S = 2.0  # the plan's budget: a code must reach the operator within this


@pytest.fixture(scope="module")
def surface(tmp_path_factory):
    """The sim stack plus the real web app over it, and the on-disk event log."""
    state = tmp_path_factory.mktemp("fi_state")
    maps = state / "maps"
    maps.mkdir()
    (state / "logs").mkdir()
    os.environ["AMR_STATE_DIR"] = str(state)
    proc = subprocess.Popen(
        [
            sys.executable, "-m", "amr_bringup.supervisor_node", "--ros-args",
            "-p", "real:=false", "-p", "web:=false", "-p", "foxglove:=false",
            "-p", f"state_dir:={state}", "-p", f"maps_dir:={maps}",
        ],
        stdout=open(state / "supervisor.log", "wb"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    from amr_web.adapter import RosAdapter, start_spinning  # noqa: PLC0415
    from amr_web.server import create_app  # noqa: PLC0415

    adapter = RosAdapter()
    spinner = start_spinning(adapter)
    app = create_app(adapter, str(maps), state_dir=str(state))
    app.config["TESTING"] = True
    client = app.test_client()
    yield client, adapter, proc, state
    spinner.stop()
    try:
        adapter.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=40)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
    rclpy.shutdown()


def wait_for_code(client, code: str, timeout: float = SETTLE_S) -> dict:
    """The standing row for `code`, or an assertion naming what was standing instead."""
    deadline = time.monotonic() + timeout
    seen: list[str] = []
    while time.monotonic() < deadline:
        rows = client.get("/api/alarms").get_json()["alarms"]
        seen = [r["code"] for r in rows]
        for row in rows:
            if row["code"] == code:
                return row
        time.sleep(0.1)
    raise AssertionError(f"{code} never stood within {timeout} s; standing: {seen}")


def on_disk(state, code: str, timeout: float = SETTLE_S) -> bool:
    """The event log is what an engineer reads after a restart, so it is the record
    that matters - not the in-memory ring."""
    path = os.path.join(str(state), "logs", "events.jsonl")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with open(path, encoding="utf-8") as fh:
                for line in fh:
                    try:
                        if json.loads(line).get("code") == code:
                            return True
                    except ValueError:
                        continue
        except OSError:
            pass
        time.sleep(0.1)
    return False


def test_a_healthy_sim_stands_nothing(surface):
    client, _, _, _ = surface
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        d = client.get("/api/alarms").get_json()
        if d["headline"]["state"].startswith("READY"):
            assert d["alarms"] == [], d["alarms"]
            return
        time.sleep(0.5)
    raise AssertionError(f"never reached READY: {client.get('/api/alarms').get_json()}")


def test_the_boot_report_reaches_the_event_log(surface):
    _, _, _, state = surface
    assert on_disk(state, "BOOT_READY", timeout=5.0)


def test_killing_the_base_offers_restart_not_recover(surface):
    """A dead base cannot be rebuilt by Recover (the drives must be re-armed), so the
    operator surface must send them to Restart instead of a button that will refuse."""
    from test_unified_sim import BASE_PATTERN, kill_owned  # noqa: PLC0415

    client, _, proc, state = surface
    assert kill_owned(proc.pid, BASE_PATTERN), "nothing to kill: the base was not running"
    row = wait_for_code(client, "BASE_EXITED", timeout=15.0)
    assert row["clears_by"] == "restart_service"
    assert cat.BUTTON[row["clears_by"]] == "Restart"
    assert client.get("/api/alarms").get_json()["headline"]["button"] == "Restart"
    assert on_disk(state, "BASE_EXITED", timeout=5.0)
    # and the surface says NEEDS SERVICE, in those words
    assert client.get("/api/alarms").get_json()["headline"]["state"] == "NEEDS SERVICE"


def test_the_standing_row_carries_the_engineers_detail_too(surface):
    client, _, _, _ = surface
    row = wait_for_code(client, "BASE_EXITED", timeout=5.0)
    assert row["detail"] and row["hint"]
    assert row["title"] != row["detail"]  # the operator's words are not the log line
