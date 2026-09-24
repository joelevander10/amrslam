"""dynamic-mapping plan §1.3: the real review.js against a fake DOM (review_harness.js) -
rectangles become map-metre ops, and saving derives a new revision and selects it."""

import json
import pathlib
import shutil
import subprocess

import pytest

NODE = shutil.which("node")


@pytest.fixture(scope="module")
def out():
    if NODE is None:
        pytest.skip("node not installed")
    harness = pathlib.Path(__file__).with_name("review_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    res = json.loads(r.stdout.strip().splitlines()[-1])
    assert "error" not in res, res["error"]
    return res


def test_rectangles_become_ops_and_small_ones_are_ignored(out):
    assert out["first_load"] == ["m1/1"]
    assert out["save_disabled_empty"] is True and out["ops_without_tool"] is True
    assert out["tool_on"] == ["dynamic", True, True]
    # dragged right-to-left, top-to-bottom: still an axis-aligned rectangle, lower-left first
    assert out["preview"] == [[1.0, 0.5], [2.0, 0.5], [2.0, 1.0], [1.0, 1.0]]
    assert out["preview_after"] is None
    rows = out["rows"]
    assert rows.count('class="row three"') == 2  # the 20 mm sliver was not added
    assert (
        "dynamic area" in rows and "1000 × 500" in rows and "erase to unknown" in rows and "500 × 400" in rows
    )
    assert out["save_enabled"] is True
    assert out["after_delete"] == 1 and out["after_undo"] == 2


def test_save_posts_ordered_ops_and_selects_the_new_revision(out):
    saved = out["saved"]
    assert saved["note"] == "trolleys"
    assert [o["op"] for o in saved["ops"]] == ["dynamic", "paint"] and saved["ops"][1]["value"] == "unknown"
    assert saved["ops"][0]["polygon"] == [[1.0, 0.5], [2.0, 0.5], [2.0, 1.0], [1.0, 1.0]]
    assert out["after_save_loads"] == ["m1/1", "m1/2"]
    assert "no edits" in out["after_save_rows"] and "rev2" in out["after_save_msg"]
    assert "from rev1" in out["info"]


def test_refused_save_keeps_the_edits_and_a_map_switch_asks_first(out):
    assert "not saved" in out["refused_msg"] and out["refused_rows"] == 1
    assert out["cancel_switch"] == ["m1/2", 1, 2]
