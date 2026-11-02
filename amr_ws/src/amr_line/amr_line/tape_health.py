"""Tape health along the taught tape: the MLS track level per position (mls-diagnostics-plan M3).

Pure and clock-free. The node feeds it every /amr/line_track sample and, each tick, where
the tape locator says the AGV is; at the end of a run it drains one record per bin and
appends them to $AMR_STATE_DIR/reports/tape_health/<date>.jsonl. tools/tape_health_report.py
merges runs and lists the stretches whose level keeps falling.

What it uses (plan E3): `track_level` (TPDO1 status bits 1-3, 0..7, manual table 19) at TPDO
rate, so it adds no bus traffic, and the event flag's rising edge (the MLS magnetic-field
event, manual 8.8). The level is coarse; a stretch that reads lower than its neighbours run
after run is the finding, not any one value.

A sample is placed only while the locator is HEALTHY and its sigma is below the bin
width: a bin is a position, and a position known to +-1 m is not one. Samples from the SDO
fallback are skipped (10 Hz, no fixed place). Nothing here reaches the follower or motion.
"""

from __future__ import annotations

import json
import math
import os
import time

BIN_M = 0.25


class TapeHealth:
    def __init__(self, bin_m: float = BIN_M):
        self.bin_m = bin_m
        self.bins: dict[tuple, dict] = {}
        self.placed = 0
        self.unplaced = 0  # samples while the position was not good enough
        self._where: tuple | None = None
        self._prev_flag = False

    def where(self, snap: dict | None) -> None:
        """TapeRuntime.snapshot() (LineState's loc_* fields), or None with no locator."""
        if (not snap or snap.get("loc_health") != "HEALTHY"
                or not 0.0 <= float(snap.get("loc_sigma_m", math.inf)) < self.bin_m):
            self._where = None
            return
        self._where = (snap.get("loc_graph", ""), snap.get("loc_circuit", ""), snap.get("loc_leg", ""),
                       float(snap["loc_s_m"]))

    def track(self, track_level: int, nlcp: int, event_flag: bool, source: str) -> None:
        """One /amr/line_track sample."""
        rising = bool(event_flag) and not self._prev_flag
        self._prev_flag = bool(event_flag)
        if source != "tpdo":
            return
        if self._where is None:
            self.unplaced += 1
            return
        graph, circuit, leg, s = self._where
        i = math.floor(s / self.bin_m)
        b = self.bins.get((graph, circuit, i))
        if b is None:
            b = self.bins[(graph, circuit, i)] = {"leg": leg, "n": 0, "sum": 0, "min": None, "gaps": 0,
                                                  "events": 0}
        self.placed += 1
        if not nlcp:
            b["gaps"] += 1  # no track at a placed position: a gap, a junction, or tape gone
            return
        lvl = int(track_level)
        b["n"] += 1
        b["sum"] += lvl
        b["min"] = lvl if b["min"] is None else min(b["min"], lvl)
        if rising:
            b["events"] += 1

    def drain(self) -> list[dict]:
        """One record per bin since the last drain, ordered along the route; resets."""
        out = []
        for (graph, circuit, i), b in sorted(self.bins.items(), key=lambda kv: kv[0]):
            out.append({
                "graph": graph, "circuit": circuit, "leg": b["leg"],
                "s0_m": round(i * self.bin_m, 3), "bin_m": self.bin_m,
                "n": b["n"], "mean": round(b["sum"] / b["n"], 2) if b["n"] else None,
                "min": b["min"], "gaps": b["gaps"], "events": b["events"],
            })
        self.bins = {}
        self.placed = self.unplaced = 0
        return out


def write_run(state_dir: str, bins: list[dict], meta: dict, now: float | None = None) -> str | None:
    """Append one run (meta + bins) as one JSON line to reports/tape_health/<date>.jsonl.
    Returns the path, or None when there was nothing to write. Raises OSError."""
    if not bins:
        return None
    now = time.time() if now is None else now
    d = os.path.join(state_dir, "reports", "tape_health")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, time.strftime("%Y-%m-%d", time.localtime(now)) + ".jsonl")
    rec = dict(meta, t=time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)), bins=bins)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, separators=(",", ":")) + "\n")
    return path
