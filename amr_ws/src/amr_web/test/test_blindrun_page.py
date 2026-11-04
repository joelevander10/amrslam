"""/blindrun page: capabilities, evidence history, the tape measurement (and its path
safety), and the real static/blindrun.js form logic under node. PV only (pp removed)."""

import json
import pathlib
import shutil
import subprocess

import pytest
from amr_web.server import create_app

from amr_web import commissioning_log as clog

NODE = shutil.which("node")
FOOTPRINT_YAML = "polygon: [[-0.5, -0.35], [1.1, -0.35], [1.1, 0.35], [-0.5, 0.35]]\nmargin_m: 0.20\n"


class Stub:
    def supervisor_identity(self):
        return ("i", 1)

    def __getattr__(self, name):
        return lambda *a: (True, "ok")


def evidence(plan_id="straight-fwd-1m-pv", backend="pv", dx=1.0, dy=0.0, hdg=0.0):
    return {
        "plan_id": plan_id,
        "phase": "DONE",
        "reason": "",
        "backend": backend,
        "plan": {
            "speed_mps": 0.2,
            "segments": [
                {
                    "kind": "straight",
                    "spec": {"distance_m": dx},
                    "commanded": {"distance_m": dx, "dx_m": dx, "dy_m": dy, "heading_deg": hdg},
                }
            ],
        },
        "gyro_heading_deg": 0.1,
    }


@pytest.fixture()
def env(tmp_path):
    state = tmp_path / "state"
    evdir = state / "commissioning"
    evdir.mkdir(parents=True)
    (evdir / "straight-fwd-1m-pv-20260918-101500-000000001.json").write_text(json.dumps(evidence()))
    (evdir / "straight-fwd-1m-pp-20260918-101700-000000002.json").write_text(
        json.dumps(evidence("straight-fwd-1m-pp", "pp"))
    )
    (evdir / "outside.txt").write_text("not evidence")
    fp = tmp_path / "footprint.yaml"
    fp.write_text(FOOTPRINT_YAML)
    maps = tmp_path / "maps"
    maps.mkdir()
    stub = Stub()
    app = create_app(stub, str(maps), str(fp), state_dir=str(state))
    app.config["TESTING"] = True
    return app.test_client(), stub, evdir


def test_the_page_serves_and_the_engineer_nav_links_it(env):
    client, _, _ = env
    r = client.get("/blindrun")
    assert r.status_code == 200 and 'id="br-form"' in r.get_data(as_text=True)
    assert 'href="/blindrun"' in client.get("/home").get_data(as_text=True)


def test_capabilities_report_the_profile_caps(env):
    client, _, _ = env
    c = client.get("/api/commissioning/capabilities").json
    assert c["max_speed_mps"] == 0.8
    assert c["min_arc_radius_m"] == pytest.approx(0.2435)
    assert c["decel_mps2"] > 0
    assert not any(k.startswith("pp") for k in c)  # profile position is gone


def test_history_lists_evidence_only_with_commanded(env):
    client, _, _ = env
    rows = client.get("/api/commissioning/history").json
    # older evidence from the pp backend still lists, named as such
    assert {r["backend"] for r in rows} == {"pv", "pp"} and len(rows) == 2
    r = next(r for r in rows if r["backend"] == "pv")
    assert r["commanded"]["dx_mm"] == 1000.0 and r["measured"] is None and r["error"] is None


def test_measurement_is_stored_and_the_error_computed(env):
    client, _, evdir = env
    run = "straight-fwd-1m-pv-20260918-101500-000000001.json"
    r = client.post(
        "/api/commissioning/measurement",
        json={"run": run, "dx_mm": 996, "dy_mm": -4, "heading_deg": -0.3, "note": "tape A"},
    )
    assert r.status_code == 200, r.json
    assert r.json["row"]["error"] == {"dx_mm": -4.0, "dy_mm": -4.0, "heading_deg": -0.3}
    assert (evdir / run.replace(".json", ".measured.json")).is_file()
    # a re-measurement replaces the previous one
    client.post(
        "/api/commissioning/measurement", json={"run": run, "dx_mm": 998, "dy_mm": 0, "heading_deg": 0}
    )
    row = next(x for x in client.get("/api/commissioning/history").json if x["run"] == run)
    assert row["measured"]["dx_mm"] == 998.0 and row["measured"]["note"] == ""


@pytest.mark.parametrize(
    "run",
    [
        "../outside.json",
        "..\\x.json",
        "/etc/passwd",
        "sub/dir.json",
        "outside.txt",
        "missing-run.json",
        "straight-fwd-1m-pv-20260918-101500-000000001.measured.json",
        "",
        None,
    ],
)
def test_measurement_refuses_anything_but_an_existing_evidence_name(env, run):
    client, _, evdir = env
    r = client.post(
        "/api/commissioning/measurement", json={"run": run, "dx_mm": 1, "dy_mm": 1, "heading_deg": 1}
    )
    assert r.status_code == 400
    assert not list(evdir.parent.parent.rglob("*.measured.json"))


@pytest.mark.parametrize(
    "body, fragment",
    [
        ({"dx_mm": "a", "dy_mm": 0, "heading_deg": 0}, "dx_mm"),
        ({"dx_mm": float("nan"), "dy_mm": 0, "heading_deg": 0}, "dx_mm"),
        ({"dx_mm": 0, "dy_mm": 99999, "heading_deg": 0}, "dy_mm"),
        ({"dx_mm": 0, "dy_mm": 0, "heading_deg": 1000}, "heading_deg"),
        ({"dx_mm": 0, "dy_mm": 0, "heading_deg": 0, "note": "x" * 600}, "note"),
    ],
)
def test_measurement_values_are_bounded(tmp_path, body, fragment):
    d = tmp_path / "commissioning"
    d.mkdir()
    (d / "a.json").write_text(json.dumps(evidence()))
    with pytest.raises(clog.LogError, match=fragment):
        clog.save_measurement(str(d), "a.json", body)


def test_multi_segment_evidence_has_no_single_commanded(tmp_path):
    ev = evidence()
    ev["plan"]["segments"] *= 2
    assert clog.commanded(ev) is None


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_blindrun_js_form_builds_the_plans_the_job_expects():
    harness = pathlib.Path(__file__).with_name("blindrun_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["straight_rev"]["plan"]["segments"] == [{"kind": "straight", "distance_m": -1.5}]
    assert out["straight_rev"]["plan"]["speed"] == {"speed_mps": 0.25}
    assert out["rotate_cw"]["plan"]["segments"] == [{"kind": "pivot", "angle_deg": -90}]
    assert out["arc_right"]["plan"]["segments"] == [{"kind": "arc", "radius_m": -1.2, "angle_deg": 45}]
    assert out["arc_right"]["plan"]["id"] == "arc-right-45deg-r1.2m"
    assert "backend" not in out["arc_right"]["plan"]  # the node defaults to pv, the only one
    for key in ("straight_rev", "rotate_cw", "arc_right"):
        assert out[key]["errors"] == [], out[key]
    assert any("radius" in e for e in out["arc_too_tight"]["errors"])
    assert any("speed" in e for e in out["too_fast"]["errors"])
    assert any("number" in e for e in out["blank_distance"]["errors"])
    assert out["stop_0p8"] == pytest.approx(0.8 * 0.8 / (2 * 0.2513), rel=1e-3)
