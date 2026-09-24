"""Review R01 + web-style W4: the real jogpad.js under a fake DOM with deferred press responses."""

import json
import pathlib
import shutil
import subprocess

import pytest

NODE = shutil.which("node")


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_jogpad_release_before_press_response_never_refreshes_and_cells_light():
    harness = pathlib.Path(__file__).with_name("jogpad_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    for ev in ("pointerup", "pointercancel"):
        assert out[f"active_while_pending_{ev}"] and out[f"cleared_{ev}"]
        assert out[f"refreshes_after_{ev}"] == 0, out
        assert out[f"late_session_released_{ev}"], out  # the late session is handed straight back
    assert out["refreshes_after_blur"] == 0 and out["refreshes_after_hidden"] == 0, out
    assert out["hold_refreshes"] >= 1 and out["hold_active"], out
    assert out["refreshes_after_release"] == 0 and out["release_cleared"], out
    assert out["stop_sent"] and out["stop_flash"] and out["stop_flash_cleared"], out
    # Q09: focus into an editable control releases a held key (accepted or pending); Escape works there
    assert out["key_hold_refreshes"] >= 1, out
    assert out["refreshes_after_focus_change"] == 0 and out["released_on_focus"], out
    assert not out["press_while_typing"] and out["escape_in_field_stops"], out
    assert out["refreshes_after_focus_while_pending"] == 0, out


@pytest.mark.skipif(NODE is None, reason="node not installed")
def test_jogpad_2026_09_19_speeds_diagonal_ratio_and_spin():
    harness = pathlib.Path(__file__).with_name("jogpad_harness.js")
    r = subprocess.run([NODE, str(harness)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    track = 0.487
    fr = out["speed_fr"]  # forward-right at 0.40: fast (left) wheel 0.40, slow (right) 0.30
    left, right = fr["v"] - fr["w"] * track / 2, fr["v"] + fr["w"] * track / 2
    assert left == pytest.approx(0.40) and right == pytest.approx(0.30) and fr["w"] < 0
    assert out["speed_r"]["v"] == 0 and out["speed_r"]["w"] == pytest.approx(-0.39)
    assert (out["speed_f"]["v"], out["speed_f"]["w"]) == (pytest.approx(0.4), 0)
    assert out["speed_l_slow"]["w"] == pytest.approx(0.195)  # 1.95 x 0.10: +30 % at every setting
