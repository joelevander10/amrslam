"""amr_line.tape_health: MLS track level per position along the taught tape (mls-diagnostics-plan M3)."""

import json

from amr_line import tape_health as th


def healthy(s, sigma=0.03, leg="J-N:right>X1", circuit="C-MRU", graph="site103 rev4 g2", health="HEALTHY"):
    return {"loc_health": health, "loc_sigma_m": sigma, "loc_s_m": s, "loc_leg": leg,
            "loc_circuit": circuit, "loc_graph": graph}


def test_samples_are_binned_by_position_with_min_mean_and_count():
    h = th.TapeHealth(bin_m=0.25)
    h.where(healthy(1.00))
    for lvl in (6, 5, 7):
        h.track(lvl, 2, False, "tpdo")
    h.where(healthy(1.26))
    h.track(2, 2, False, "tpdo")
    out = h.drain()
    assert [b["s0_m"] for b in out] == [1.0, 1.25]
    assert out[0]["n"] == 3 and out[0]["min"] == 5 and out[0]["mean"] == 6.0
    assert out[1]["min"] == 2 and out[1]["leg"] == "J-N:right>X1" and out[1]["graph"] == "site103 rev4 g2"
    assert h.drain() == []  # drained


def test_nothing_is_placed_without_a_healthy_locator_or_with_a_wide_sigma():
    h = th.TapeHealth(bin_m=0.25)
    for snap in (None, healthy(2.0, health="DEGRADED"), healthy(2.0, health="LOST"), healthy(2.0, sigma=0.3)):
        h.where(snap)
        h.track(5, 2, False, "tpdo")
    assert h.drain() == [] and h.unplaced == 0  # drain reset the counters too
    h.where(healthy(2.0, sigma=0.3))
    h.track(5, 2, False, "tpdo")
    assert h.unplaced == 1 and not h.bins


def test_sdo_samples_are_skipped():
    h = th.TapeHealth()
    h.where(healthy(3.0))
    h.track(5, 2, False, "sdo")
    assert not h.bins


def test_no_track_at_a_placed_position_is_a_gap_not_a_level():
    h = th.TapeHealth()
    h.where(healthy(4.0))
    h.track(0, 0, False, "tpdo")
    h.track(6, 2, False, "tpdo")
    (b,) = h.drain()
    assert b["gaps"] == 1 and b["n"] == 1 and b["min"] == 6


def test_an_event_is_counted_once_on_its_rising_edge_where_it_happened():
    h = th.TapeHealth()
    h.where(healthy(5.0))
    h.track(3, 2, True, "tpdo")
    h.track(3, 2, True, "tpdo")  # still latched
    h.where(healthy(5.6))
    h.track(6, 2, False, "tpdo")
    h.track(2, 2, True, "tpdo")
    a, b = h.drain()
    assert a["events"] == 1 and b["events"] == 1


def test_two_circuits_on_the_same_s_are_different_bins():
    h = th.TapeHealth()
    h.where(healthy(1.0, circuit="C-MRU"))
    h.track(5, 2, False, "tpdo")
    h.where(healthy(1.0, circuit="C-BTU"))
    h.track(4, 2, False, "tpdo")
    assert {b["circuit"] for b in h.drain()} == {"C-MRU", "C-BTU"}


def test_a_run_is_one_json_line_in_the_days_file_and_an_empty_run_writes_nothing(tmp_path):
    bins = [{"graph": "g", "circuit": "C-MRU", "leg": "L", "s0_m": 0.0, "bin_m": 0.25, "n": 1, "mean": 5.0,
             "min": 5, "gaps": 0, "events": 0}]
    p1 = th.write_run(str(tmp_path), bins, {"generation": 3, "result": "DONE"}, now=1792300000.0)
    p2 = th.write_run(str(tmp_path), bins, {"generation": 3, "result": "FAULT"}, now=1792300100.0)
    assert p1 == p2 and p1.startswith(str(tmp_path / "reports" / "tape_health"))
    lines = open(p1, encoding="utf-8").read().splitlines()
    assert len(lines) == 2
    rec = json.loads(lines[1])
    assert rec["result"] == "FAULT" and rec["generation"] == 3 and rec["bins"] == bins and "t" in rec
    assert th.write_run(str(tmp_path), [], {}) is None
