"""Run page (static/run.js) under a fake DOM: the route preview survives the page-load race and
follows the mission the executor holds (vehicle 2026-09-19: a loaded route was not drawn)."""

import json
import pathlib
import shutil
import subprocess

import pytest

NODE = shutil.which("node")


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_route_preview_is_drawn_after_the_first_state_poll_and_follows_the_run():
    harness = pathlib.Path(__file__).with_name("run_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["after_first_state"] == "r2" and out["selected"] == "mis2", out  # the executor's mission
    assert out["steady"] == "r2" and out["no_refetch"], out  # polls do not refetch or clear it
    assert out["followed"] == "r1", out  # a newly loaded mission is drawn


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_initial_pose_at_the_selected_missions_route_start():
    harness = pathlib.Path(__file__).with_name("run_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    body = out["pose_post"]["body"]
    got = (body["x_m"], body["y_m"], body["map_id"], body["map_revision"], body["generation"])
    assert got == (1.5, 2.0, "m1", 1, 3)
    assert body["yaw_rad"] == pytest.approx(1.5707963, abs=1e-6)
