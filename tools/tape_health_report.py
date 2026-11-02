#!/usr/bin/env python3
"""Merge the line layer's tape-health runs and list the weak stretches (mls-diagnostics-plan M3).

    python3 tools/tape_health_report.py                 # last 10 runs of the newest graph
    python3 tools/tape_health_report.py --runs 30 --level 3 --json

Input: $AMR_STATE_DIR/reports/tape_health/*.jsonl (default ~/.amr), one line per run, written by
amr_line.tape_health at the end of each LINE run. Each bin is 0.25 m of one circuit's route.

A stretch is listed when, over the runs read, its lowest MLS track level (0..7) is at or below
--level, or the MLS raised its magnetic-field event there, or it lies below its circuit's median
level by --drop. A bin seen in only one run is marked: one pass is a hint, the same bin in run
after run is a finding. Read-only; offline (no ROS).
"""
import argparse
import glob
import json
import os
import statistics
import sys


def load_runs(directory):
    """Every run record in date order (file name, then line order). Bad lines are skipped."""
    runs = []
    for path in sorted(glob.glob(os.path.join(directory, "*.jsonl"))):
        with open(path, encoding="utf-8") as f:
            for line in f:
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                if isinstance(rec, dict) and isinstance(rec.get("bins"), list):
                    runs.append(rec)
    return runs


def newest_graph(runs):
    for rec in reversed(runs):
        for b in reversed(rec["bins"]):
            if b.get("graph"):
                return b["graph"]
    return ""


def merge(runs, graph):
    """(circuit, s0_m) -> merged bin over the runs, only bins of `graph`."""
    out = {}
    for k, rec in enumerate(runs):
        for b in rec["bins"]:
            if b.get("graph", "") != graph:
                continue
            key = (b["circuit"], float(b["s0_m"]))
            m = out.setdefault(key, {"circuit": b["circuit"], "s0_m": float(b["s0_m"]), "bin_m": b["bin_m"],
                                     "leg": b["leg"], "runs": set(), "n": 0, "sum": 0.0, "min": None,
                                     "gaps": 0, "events": 0})
            m["runs"].add(k)
            m["gaps"] += int(b.get("gaps", 0))
            m["events"] += int(b.get("events", 0))
            if b.get("n"):
                m["n"] += b["n"]
                m["sum"] += b["mean"] * b["n"]
                m["min"] = b["min"] if m["min"] is None else min(m["min"], b["min"])
    for m in out.values():
        m["mean"] = round(m["sum"] / m["n"], 2) if m["n"] else None
        m["runs"] = len(m.pop("runs"))
        m.pop("sum")
    return out


def weak(merged, level, drop):
    """The listed stretches, worst first: events, then the lowest level, then the largest drop."""
    medians = {}
    for c in {m["circuit"] for m in merged.values()}:
        means = [m["mean"] for m in merged.values() if m["circuit"] == c and m["mean"] is not None]
        medians[c] = statistics.median(means) if means else None
    rows = []
    for m in merged.values():
        med = medians[m["circuit"]]
        below = None if m["mean"] is None or med is None else round(med - m["mean"], 2)
        why = []
        if m["events"]:
            why.append(f"{m['events']} field event(s)")
        if m["min"] is not None and m["min"] <= level:
            why.append(f"level {m['min']} <= {level}")
        if below is not None and below >= drop:
            why.append(f"{below:g} below the circuit median {med:g}")
        if why:
            rows.append(dict(m, below_median=below, why="; ".join(why)))
    rows.sort(key=lambda r: (-r["events"], 99 if r["min"] is None else r["min"], -(r["below_median"] or 0)))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description="Weak stretches of tape from the MLS track level, over runs.")
    state = os.environ.get("AMR_STATE_DIR", os.path.expanduser("~/.amr"))
    ap.add_argument("--dir", default=os.path.join(state, "reports", "tape_health"))
    ap.add_argument("--runs", type=int, default=10, help="the last N runs (default 10)")
    ap.add_argument("--graph", help="tape graph name (default: the newest seen)")
    ap.add_argument("--level", type=int, default=3, help="list bins whose lowest level is <= this (3)")
    ap.add_argument("--drop", type=float, default=1.5, help="...or whose mean is this far below the median")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    runs = load_runs(a.dir)[-max(1, a.runs):]
    if not runs:
        print(f"no tape-health runs in {a.dir}")
        return 1
    graph = a.graph or newest_graph(runs)
    merged = merge(runs, graph)
    rows = weak(merged, a.level, a.drop)
    if a.json:
        print(json.dumps({"graph": graph, "runs": len(runs), "bins": len(merged), "weak": rows}, indent=1))
        return 0
    print(f"{len(runs)} run(s), graph {graph or '-'}, {len(merged)} bins of tape")
    if not rows:
        print("no weak stretch")
        return 0
    print(f"{'circuit':<8} {'s [m]':>8} {'leg':<22} {'runs':>4} {'min':>3} {'mean':>5} {'gaps':>4}  why")
    for r in rows:
        one = " (1 run)" if r["runs"] == 1 else ""
        print(f"{r['circuit']:<8} {r['s0_m']:>8.2f} {r['leg']:<22} {r['runs']:>4} "
              f"{'-' if r['min'] is None else r['min']:>3} {'-' if r['mean'] is None else r['mean']:>5} "
              f"{r['gaps']:>4}  {r['why']}{one}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
