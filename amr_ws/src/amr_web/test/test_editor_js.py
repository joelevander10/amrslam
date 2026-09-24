"""Speed 0.40 plan A3/A6: the real editor.js against a fake DOM - the speed control is a number
input and must show a loaded route's stored cap without rewriting it."""

import json
import pathlib
import shutil
import subprocess

import pytest

NODE = shutil.which("node")


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_speed_control_shows_stored_value_and_defaults_to_055():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert float(out["new_draft_speed"]) == 0.55, out
    assert [float(v) for v in out["new_draft_long"]] == [0.85, 4.0], out
    assert float(out["new_draft_arc"]) == 0.40, out
    assert float(out["loaded_arc_shown"]) == 0.40, out  # shown for a file without it...
    assert float(out["loaded_speed_shown"]) == 0.3, out
    assert out["loaded_long_shown"] == "", out  # a route without a boost shows none
    assert out["saved_limits"] == {"linear_mps": 0.3}, out  # ...never written back; stored values survive
    assert float(out["reset_speed"]) == 0.55 and float(out["reset_long"]) == 0.85, out


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_straights_typed_in_mm_and_shown_in_mm():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["len_no_number"] is True
    assert out["len_preview_m"] == pytest.approx(1.5)
    assert "1500<i>mm</i>" in out["len_step_row"], out["len_step_row"]
    assert out["len_preview_after"] is None
    # (2.0, 0.7) projected onto the +x heading from (1.5, 0): 0.5 m further
    assert out["hover_preview_m"] == pytest.approx(0.5)
    assert out["hover_cleared"] is None
    steps = out["mm_saved_steps"]
    assert len(steps) == 2, steps  # the 20 mm request was refused
    assert steps[0]["to"] == pytest.approx({"x_m": 1.5, "y_m": 0.0})  # the file stays in metres
    assert steps[1]["to"] == pytest.approx({"x_m": 2.0, "y_m": 0.0})
    assert "1500<i>mm</i>" in out["mm_rows"] and "500<i>mm</i>" in out["mm_rows"], out["mm_rows"]


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_marks_boosted_long_straights_and_saves_the_boost_limits():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    rows = out["boost_rows"]
    assert "4000<i>mm</i>" in rows and "4500<i>mm</i>" in rows
    assert rows.count("▲0.85") == 1 and rows.index("▲0.85") > rows.index(
        "4000<i>mm</i>"
    )  # only the 4.5 m one
    assert out["boost_saved_limits"] == {
        "linear_mps": 0.55,
        "long_linear_mps": 0.85,
        "long_min_length_m": 4.0,
        "arc_linear_mps": 0.40,
    }
    assert "▲" not in out["boost_off_rows"]
    assert out["boost_off_limits"]["long_linear_mps"] is None  # empty field = null in the file, not 0.85


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_reverse_step_is_bounded_and_moves_the_end_pose_back():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["reverse_refused"] is True  # 2500 mm > REVERSE_MAX_M
    assert "reverse" in out["reverse_rows"] and "1200<i>mm</i>" in out["reverse_rows"]
    assert out["reverse_saved_step"] == {"id": "s5", "type": "reverse", "distance_m": 1.2}
    # the route stood at x = 10.5 (1.5 + 0.5 + 4.0 + 4.5); after backing 1.2 m a 1.0 m straight ends at 10.3
    assert out["after_reverse_to"] == pytest.approx({"x_m": 10.3, "y_m": 0.0})


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_arc_step_bounds_and_end_pose():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["arc_refused"] is True  # R 0.5 and 30 deg
    assert "arc L" in out["arc_rows"] and "90° R 1.00" in out["arc_rows"]
    assert "≤0.40" in out["arc_rows"]  # arc_linear_mps 0.40 holds at R 1.0 (0.9 x 0.45 x 1.0 = 0.405)
    assert out["arc_saved_step"] == {
        "id": "s7",
        "type": "arc",
        "direction": "ccw",
        "angle_deg": 90,
        "radius_m": 1.0,
    }
    # from (10.3, 0) heading +x, a left 90 deg arc of R 1 ends at (11.3, 1) heading +y; 1 m on: (11.3, 2)
    assert out["after_arc_to"] == pytest.approx({"x_m": 11.3, "y_m": 2.0})


def test_browser_jog_caps_are_the_2026_09_19_manual_speeds():
    from amr_web import jog

    assert (jog.V_MAX, jog.W_MAX) == (0.40, 0.39)
