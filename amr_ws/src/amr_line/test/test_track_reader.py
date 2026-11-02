"""TrackReader: freshness, rate and source gates, and where the rate floor
comes from. No ROS: the LineTrack message is a namespace."""

import os
import sys
import types

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_line")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from amr_line import track as tk  # noqa: E402


def msg(seq, source="tpdo", age=0.002):
    return types.SimpleNamespace(
        seq=seq,
        source=source,
        sample_age_s=age,
        lcp_mm=[0, 0, 0],
        valid=[True, False, False],
        line_good=True,
        nlcp=1,
    )


def stream(reader, n, hz, t0=10.0, source="tpdo"):
    t = t0
    for k in range(n):
        reader.update(msg(k, source), t)
        t += 1.0 / hz
    return t


def test_an_unmeasured_rate_is_not_usable():
    """Two samples are not a rate. Before, `rate is None` passed the gate,
    so the first ticks of any stream were 'measured good' by default."""
    r = tk.TrackReader(timeout_s=0.2, min_hz=40.0)
    assert r.usable(10.0) == (False, "track")
    t = stream(r, 2, 100.0)
    assert r.usable(t) == (False, "rate")
    t = stream(r, 3, 100.0)
    assert r.usable(t) == (True, "")


def test_a_slow_but_fresh_stream_is_refused_by_rate():
    r = tk.TrackReader(timeout_s=0.2, min_hz=40.0)
    t = stream(r, 10, 9.8)
    assert r.usable(t) == (False, "rate")
    fast = tk.TrackReader(timeout_s=0.2, min_hz=40.0)
    t = stream(fast, 10, 100.0)
    assert fast.usable(t) == (True, "")


def test_the_rate_floor_is_the_one_handed_in():
    """F06: the node used to read the floor via getattr(runtime, ..., 40.0)
    and always got 40. Whatever is handed in must be what decides."""
    r = tk.TrackReader(timeout_s=0.2, min_hz=150.0)
    t = stream(r, 10, 100.0)
    assert r.usable(t) == (False, "rate")


def test_sdo_samples_are_refused_unless_opted_in():
    r = tk.TrackReader(timeout_s=0.2, min_hz=40.0)
    t = stream(r, 10, 100.0, source="sdo")
    assert r.usable(t) == (False, "rate")
    r2 = tk.TrackReader(timeout_s=0.2, min_hz=40.0, accept_sdo=True)
    t = stream(r2, 10, 100.0, source="sdo")
    assert r2.usable(t) == (True, "")


def test_a_stopped_stream_goes_stale_whatever_its_rate_was():
    r = tk.TrackReader(timeout_s=0.2, min_hz=40.0)
    t = stream(r, 10, 100.0)
    assert r.usable(t) == (True, "")
    assert r.usable(t + 0.5) == (False, "track")


def test_the_node_reads_the_floor_from_the_vehicle_profile():
    """The wiring mistake itself: runtime does not carry LINE_MIN_TRACK_HZ
    (it is not an engine name), so the node must read agv_core.config."""
    from amr_line import runtime

    runtime.load_from_profile()
    assert not hasattr(runtime, "LINE_MIN_TRACK_HZ")
    src = open(os.path.join(ROOT, "amr_ws", "src", "amr_line", "amr_line", "line_follow_node.py")).read()
    assert 'getattr(runtime, "LINE_MIN_TRACK_HZ"' not in src
    assert "vehicle_config.LINE_MIN_TRACK_HZ" in src
    assert "vehicle_config.AUTO_RESUME_HOLD_S" in src


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
