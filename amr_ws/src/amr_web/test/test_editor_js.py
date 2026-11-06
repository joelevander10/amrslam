"""Speed 0.40 plan A3/A6: the real editor.js against a fake DOM - the speed control is a number
input and must show a loaded route's stored cap without rewriting it."""

import json
import pathlib
import shutil
import subprocess

import pytest

NODE = shutil.which("node")


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_has_no_speed_setting_and_drops_old_speed_keys():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert "no steps" in out["new_draft_rows"], out
    assert out["saved_limits"] == {"angular_rad_s": 0.3}, out  # the old file's speeds are not written back
    assert out["reset_saved"] == {}, out  # a new draft carries no speed at all


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
def test_editor_does_not_mark_long_straights_any_more():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    rows = out["long_rows"]
    assert "4000<i>mm</i>" in rows and "4500<i>mm</i>" in rows and "▲" not in rows
    assert "◂0.30" in out["reverse_rows"]  # reverse at half the 0.60 trackless speed


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
    assert "≤0.41" in out["arc_rows"]  # the 0.60 speed, held to 0.9 x 0.45 x R 1.0 = 0.405 by the turn cap
    assert out["arc_saved_step"] == {
        "id": "s7",
        "type": "arc",
        "direction": "ccw",
        "angle_deg": 90,
        "radius_m": 1.0,
    }
    # from (10.3, 0) heading +x, a left 90 deg arc of R 1 ends at (11.3, 1) heading +y; 1 m on: (11.3, 2)
    assert out["after_arc_to"] == pytest.approx({"x_m": 11.3, "y_m": 2.0})


def test_browser_jog_caps_are_the_2026_10_02_manual_speeds():
    from amr_web import jog

    assert (jog.V_MAX, jog.W_MAX) == (0.50, 0.39)  # = the pendant's pendant_v / pendant_w


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_measure_tool_snaps_reads_out_mm_and_never_edits_the_route():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["measure_on"] is True and out["pan_tools"] == ["measure"]  # a drag still pans
    assert out["geometry_calls"] == 1  # saved routes fetched once, when the tool is selected
    assert "click point A" in out["measure_idle"]
    assert "B?" in out["measure_live"] and "other r2 · s2 wait UBC" in out["measure_live"]
    rows = out["measure_rows"]
    # A = the s7 arc centre (10.3, 1), B = the saved wait (3, 5): B - A = (-7.3, +4.0), 8.324 m
    assert "draft · s7 arc centre" in rows and "other r2 · s2 wait UBC" in rows
    assert "<b>8324</b>" in rows and "<b>-7300</b>" in rows and "<b>+4000</b>" in rows
    assert "B?" not in rows
    assert out["result_untouched"] is True
    free = out["free_rows"]
    assert "free point" in free and "<b>500</b>" in free and "<b>+500</b>" in free and "<b>0</b>" in free
    assert "on draft · s3" in out["path_rows"] and "5.000, 0.000" in out["path_rows"]
    assert "select Measure" in out["measure_cleared"]
    assert out["after_undo_ids"] == ["s1", "s2", "s3", "s4", "s5", "s6", "s7"]


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_speed_waypoints_split_a_straight_and_set_a_slow_zone():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["split_refused"] is True  # 10 mm from the end: under the 50 mm minimum
    assert out["bad_speed_kept"] == ""  # 5 m/s is above the route speed: the box goes back
    # 6 m split at 2 m, its tail at 1.5 m; the middle part is slow; a straight added slow
    assert out["split_saved"] == [
        {"id": "s1", "x": 2000, "v": None},
        {"id": "s2", "x": 3500, "v": 0.3},
        {"id": "s3", "x": 6000, "v": None},
        {"id": "s4", "x": 7000, "v": 0.25},
    ]
    assert "s2 · 1500 mm · 0.30 m/s" in out["split_options"]


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_start_at_the_amr_only_when_localised_on_this_map():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["here_refused_start"] == {"x_m": 0, "y_m": 0, "yaw_deg": 0}  # CHECKING: unchanged
    assert out["here_start"] == {"x_m": 3.25, "y_m": -1.5, "yaw_deg": 90}


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_editor_branch_route_is_main_plus_branch_and_rejoins_exactly():
    harness = pathlib.Path(__file__).with_name("editor_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in out, out
    assert out["branch_open_rows"] is False  # rejoin after s2 refused: the branch ends at s1's end
    assert out["branch_saved"] == ["s1", "b1", "b2", "b3", "s2", "s3"]  # the 0.5 m after rejoin refused
    assert out["branch_meta"] == {
        "main_route_id": "main",
        "main_revision": 1,
        "leave_after": "s1",
        "rejoin_after": "s1",
        "steps": ["b1", "b2", "b3"],
    }
