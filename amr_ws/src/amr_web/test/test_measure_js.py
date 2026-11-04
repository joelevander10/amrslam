"""Route editor Measure tool (2026-11-03): static/measure.js geometry and snapping."""

import json
import math
import pathlib
import shutil
import subprocess

import pytest

NODE = shutil.which("node")
pytestmark = pytest.mark.skipif(NODE is None, reason="node not installed")


@pytest.fixture(scope="module")
def out():
    harness = pathlib.Path(__file__).with_name("measure_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    o = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in o, o
    return o


def test_delta_is_b_minus_a(out):
    assert out["delta"] == pytest.approx({"dx": -3.0, "dy": 4.0, "dist": 5.0})


def test_nearest_on_segment_is_clamped_to_its_ends(out):
    assert out["seg_mid"] == pytest.approx([1.0, 0.0])
    assert out["seg_clamped"] == pytest.approx([2.0, 0.0])


def test_nearest_on_arc_stays_within_its_sweep(out):
    # (2, 0) seen from the centre (0, 1) is at -26.6 deg, inside the -90..0 sweep: radially onto the circle
    u = 1.0 / math.sqrt(5.0)
    assert out["arc_on"] == pytest.approx([2 * u, 1.0 - u])
    assert out["arc_outside"] == pytest.approx([0.0, 0.0], abs=1e-12)
    assert out["arc_cw_on"] == pytest.approx([2 * u, -1.0 + u])
    assert out["arc_cw_outside"] == pytest.approx([0.0, 0.0], abs=1e-12)


def test_snap_targets_per_step_type(out):
    k = out["kinds"]
    assert "vertex:draft · start" in k and "vertex:draft · s1 end" in k
    assert "wait:draft · s2 wait hitch" in k
    assert "centre:draft · s3 arc centre" in k
    assert "vertex:draft · s5 turn" in k and "vertex:survey start" in k
    assert out["paths"] == ["on draft · s1", "on draft · s3 arc", "on draft · s4"]


def test_snap_priority_and_reach(out):
    assert out["snap_wait"] == {"x": 2, "y": 0, "kind": "wait", "label": "draft · s2 wait hitch"}
    c = out["snap_centre"]
    assert c["kind"] == "centre" and (c["x"], c["y"]) == (2, 1)
    # s4's end and the s5 turn share (3, 4): same rank, same distance - the first listed wins
    assert out["snap_vertex"] == {"x": 3, "y": 4, "kind": "vertex", "label": "draft · s4 end"}
    on = out["snap_on_line"]
    assert on["kind"] == "path" and on["label"] == "on draft · s1"
    assert (on["x"], on["y"]) == pytest.approx((1.0, 0.0))
    arc = out["snap_on_arc"]
    assert arc["kind"] == "path" and arc["label"] == "on draft · s3 arc"
    assert math.hypot(arc["x"] - 2.0, arc["y"] - 1.0) == pytest.approx(1.0)  # on the circle
    assert out["snap_free"] == {"x": 1.0, "y": 2.5, "kind": "free", "label": "free point"}
    assert out["snap_zero_tol"]["kind"] == "free"
    assert out["snap_extra"]["label"] == "survey start"


def test_arc_points_follow_the_sweep(out):
    h = math.sqrt(0.5)
    flat = [v for pt in out["arc_points"] for v in pt]
    assert flat == pytest.approx([0.0, 0.0, h, 1.0 - h, 1.0, 1.0], abs=1e-12)
