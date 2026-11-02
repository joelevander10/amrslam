"""tools/tape_health_report.py on synthetic runs (no ROS).

    python3 -m pytest -q tools/test_tape_health_report.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tape_health_report as tr  # noqa: E402


def b(s0, mean, mn, events=0, circuit="C-MRU", graph="g2", n=40, gaps=0):
    return {"graph": graph, "circuit": circuit, "leg": "L1", "s0_m": s0, "bin_m": 0.25, "n": n, "mean": mean,
            "min": mn, "gaps": gaps, "events": events}


def write(tmp_path, name, runs):
    with open(tmp_path / name, "w", encoding="utf-8") as f:
        for bins in runs:
            f.write(json.dumps({"t": "x", "bins": bins}) + "\n")
        f.write("not json\n")


def good_run(extra=()):
    return [b(i * 0.25, 6.0, 5) for i in range(8)] + list(extra)


def test_runs_merge_by_circuit_and_position_with_a_weighted_mean(tmp_path):
    write(tmp_path, "2026-10-18.jsonl", [[b(1.0, 6.0, 5, n=10)], [b(1.0, 3.0, 2, n=30)]])
    runs = tr.load_runs(str(tmp_path))
    assert len(runs) == 2  # the bad line is skipped
    (m,) = tr.merge(runs, "g2").values()
    assert m["runs"] == 2 and m["n"] == 40 and m["mean"] == 3.75 and m["min"] == 2


def test_a_low_level_an_event_or_a_drop_below_the_median_is_listed_worst_first():
    runs = [{"bins": good_run([b(2.0, 6.0, 2), b(2.25, 6.0, 5, events=1), b(2.5, 4.0, 4)])}]
    rows = tr.weak(tr.merge(runs, "g2"), level=3, drop=1.5)
    assert [r["s0_m"] for r in rows] == [2.25, 2.0, 2.5]
    assert "field event" in rows[0]["why"] and "level 2 <= 3" in rows[1]["why"] and "below" in rows[2]["why"]


def test_good_tape_lists_nothing_and_another_graph_is_not_mixed_in():
    runs = [{"bins": good_run([b(9.0, 1.0, 0, graph="g1")])}]
    assert tr.newest_graph(runs) == "g1"  # the last bin seen
    assert tr.weak(tr.merge(runs, "g2"), level=3, drop=1.5) == []


def test_the_cli_reads_the_last_runs_and_reports(tmp_path, capsys):
    write(tmp_path, "2026-10-17.jsonl", [good_run([b(3.0, 2.0, 1)])])
    write(tmp_path, "2026-10-18.jsonl", [good_run(), good_run()])
    assert tr.main(["--dir", str(tmp_path), "--runs", "2"]) == 0
    assert "no weak stretch" in capsys.readouterr().out
    assert tr.main(["--dir", str(tmp_path), "--runs", "3", "--json"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["runs"] == 3 and out["weak"][0]["s0_m"] == 3.0 and out["weak"][0]["runs"] == 1
    assert tr.main(["--dir", str(tmp_path / "none")]) == 1
