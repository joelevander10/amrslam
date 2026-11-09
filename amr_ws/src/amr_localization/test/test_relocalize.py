"""Automatic relocalisation after a loss during a run (relocalize.py, owner 2026-11-09)."""

import math

from amr_localization import relocalize as rl


def test_refuses_sensor_losses_and_anything_not_lost():
    r = rl.AutoReloc()
    assert "not LOST" in r.start(0.0, (1.0, 2.0, 0.0), False, "LOC_JUMP")
    assert "sensor" in r.start(0.0, (1.0, 2.0, 0.0), True, "LOC_STREAM_STALE")
    assert "sensor" in r.start(0.0, (1.0, 2.0, 0.0), True, "LOC_GATE_FAILED")
    assert "no pose" in r.start(0.0, None, True, "LOC_JUMP")
    assert not r.active
    for code in ("LOC_JUMP", "LOC_SCAN_MISMATCH", "LOC_COV_GREW", "LOC_LOST"):
        assert rl.AutoReloc().start(0.0, (1.0, 2.0, 0.0), True, code) is None


def test_confirms_only_when_readiness_could_and_the_scan_fits_near_the_stop():
    r = rl.AutoReloc()
    assert r.start(10.0, (5.0, 1.0, 0.0), True, "LOC_JUMP") is None
    near = (5.2, 1.05, math.radians(3))
    assert r.tick(11.0, near, True, False, 0.95) is None  # still converging
    assert r.tick(12.0, near, True, True, 0.60) is None  # converged, fits too little yet
    out = r.tick(13.0, near, True, True, 0.85)
    assert out.startswith("confirm") and "85 %" in out and not r.active


def test_fails_when_it_converges_too_far_away_or_takes_too_long():
    r = rl.AutoReloc()
    r.start(0.0, (0.0, 0.0, 0.0), True, "LOC_JUMP")
    out = r.tick(5.0, (0.8, 0.0, 0.0), True, True, 0.95)
    assert out.startswith("fail") and "0.80 m" in out and not r.active
    r.start(0.0, (0.0, 0.0, 0.0), True, "LOC_JUMP")
    assert r.tick(5.0, (0.0, 0.0, math.radians(20)), True, True, 0.95).startswith("fail")
    r.start(0.0, (0.0, 0.0, 0.0), True, "LOC_JUMP")
    assert r.tick(29.0, (0.0, 0.0, 0.0), True, False, 0.9) is None
    assert "within 30 s" in r.tick(31.0, (0.0, 0.0, 0.0), True, False, 0.9)


def test_cancel_ends_an_attempt():
    r = rl.AutoReloc()
    r.start(0.0, (0.0, 0.0, 0.0), True, "LOC_JUMP")
    r.cancel("someone set the initial pose by hand")
    assert not r.active and "by hand" in r.last
    assert r.tick(1.0, (0.0, 0.0, 0.0), True, True, 0.9) is None
