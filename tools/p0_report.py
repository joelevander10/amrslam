#!/usr/bin/env python3
"""P0 of tracked plan 3: how repeatably AMCL knows where the AGV is along the tape.

    python3 tools/p0_report.py ~/.amr/runs/<stamp> [<stamp> ...]     (needs amr_ws sourced)

One run = one lap recorded by tools/run_log.sh while p0_localization.launch.py runs
(runbook section 2). Every run is split at its U-turn pivot into OUT and BACK; each
direction gets a reference path, the mean of all the runs' AMCL tracks (the same
averaging the teach builder uses). Then, per direction:

  M1  along-track repeatability: AMCL's s (latency-compensated: the last pose before the
      read, rolled forward by wheel travel) at every MLS code-3 read, clustered per strip;
      2 sigma per strip. Without code-3 reads: the RFID encounters, per tag (FALLBACK -
      the read zone's own scatter is in the number, so it is an upper bound)
  M2  lateral residual: AMCL distance from the reference on straights, 2 sigma
  M3  jumps: one AMCL update moving more than 0.15 m away from what the wheels did
  M4  latency: bag receive time - /amcl_pose stamp, p95
  M5  CPU busy and package temperature (run_log.sh's cpu_temp.csv), top processes (pidstat)
  M6  odometry scale: wheel travel / reference distance between consecutive strips

Targets and the decision rule: plan 3 section 5. The pure part (analyse) takes plain
lists and is tested without a bag (tools/test_p0_report.py).
"""
import bisect
import csv
import math
import os
import statistics
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "amr_ws", "src", "amr_line"))

from amr_line import tape_graph as tg  # noqa: E402
from amr_line import tape_teach as tt  # noqa: E402

TARGET = {"M1": 0.05, "M2": 0.05, "M3": 0, "M4": 0.2, "M5_cpu": 75.0, "M5_temp": 75.0, "M6": 0.02}
JUMP_M = 0.15
STRIP_GAP_M = 1.0          # reads further apart than this along the path are different strips
STRAIGHT_DEG_PER_M = 3.0   # M2 counts only where the reference turns less than this


# -- the pure part ----------------------------------------------------------------------

def travel_at(wheels, t):
    """wheels: [(t, travel)] sorted. Interpolated travel at t (clamped)."""
    if not wheels:
        return None
    if t <= wheels[0][0]:
        return wheels[0][1]
    if t >= wheels[-1][0]:
        return wheels[-1][1]
    lo, hi = 0, len(wheels) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        lo, hi = (mid, hi) if wheels[mid][0] <= t else (lo, mid)
    (t0, a), (t1, b) = wheels[lo], wheels[hi]
    return a if t1 == t0 else a + (b - a) * (t - t0) / (t1 - t0)


def split(run):
    """A run -> {"out": part, "back": part}, cut at the first pivot (or "out" only)."""
    pivots = run.get("pivots") or []
    cut = pivots[0] if pivots else None
    parts = {}
    for name, keep in (("out", lambda t: cut is None or t < cut[0]),
                       ("back", lambda t: cut is not None and t > cut[1])):
        parts[name] = {
            "poses": [p for p in run["poses"] if keep(p[0])],
            "events": [e for e in run["events"] if keep(e[0])],
        }
    return parts


def _two_sigma(xs):
    return 2.0 * statistics.pstdev(xs) if len(xs) > 1 else float("nan")


def reference(parts):
    """Mean AMCL track of one direction over all runs -> Polyline."""
    segs = [[(x, y) for _t, _st, x, y in p["poses"]] for p in parts if len(p["poses"]) > 10]
    if not segs:
        return None
    leg = tt.mean_leg("A:x>B", "A", "x", "B", segs, [0.0], "none")
    return leg.line


def analyse(runs):
    """runs: [{"name", "wheels": [(t, travel)], "poses": [(t_recv, stamp, x, y)],
    "events": [(t, key)] (code-3 strip reads, or "rfid:<tag>"), "pivots": [(t0, t1)]}].
    Returns {direction: metrics} plus "all"."""
    out = {}
    lat = [p[0] - p[1] for r in runs for p in r["poses"]]
    jumps = []
    for r in runs:
        for a, b in zip(r["poses"], r["poses"][1:], strict=False):
            moved = math.hypot(b[2] - a[2], b[3] - a[3])
            wheels = abs(travel_at(r["wheels"], b[1]) - travel_at(r["wheels"], a[1]))
            if abs(moved - wheels) > JUMP_M:
                jumps.append((r["name"], b[0], moved - wheels))
    out["all"] = {"M3": jumps, "M4": sorted(lat)[int(0.95 * (len(lat) - 1))] if lat else float("nan")}

    for d in ("out", "back"):
        parts = [(r, split(r)[d]) for r in runs]
        ref = reference([p for _r, p in parts])
        if ref is None:
            continue
        reads, lateral = [], []
        for r, p in parts:
            poses = p["poses"]
            along = _track(ref, poses)
            for (_t, _stamp_p, _x, _y), hit in zip(poses, along, strict=True):
                if hit and 0.5 < hit[0] < ref.length - 0.5 and _straight(ref, hit[0]):
                    lateral.append(hit[1])
            stamps = [q[1] for q in poses]
            for t_ev, key in p["events"]:
                k = bisect.bisect_right(stamps, t_ev) - 1
                if k < 0 or along[k] is None:
                    continue
                stamp, hit = poses[k][1], along[k]
                s = hit[0] + travel_at(r["wheels"], t_ev) - travel_at(r["wheels"], stamp)
                reads.append({"run": r["name"], "key": key, "s": s, "travel": travel_at(r["wheels"], t_ev)})
        strips = _cluster(reads)
        m1 = {name: _two_sigma([x["s"] for x in xs]) for name, xs in strips.items()}
        out[d] = {"length": ref.length, "M1": m1, "M1_n": {k: len(v) for k, v in strips.items()},
                  "M2": _two_sigma(lateral), "M2_n": len(lateral), "M6": _scale(strips)}
    return out


def _track(ref, poses):
    """Project a lap's poses in order: each near the last hit (fast, and never onto a part
    of the path the vehicle is not on); a full search only to start or after a miss."""
    out, last = [], None
    for _t, _st, x, y in poses:
        hit = ref.project(x, y, last - 1.0, last + 2.0) if last is not None else None
        if hit is None or abs(hit[1]) > 0.5:
            hit = ref.project(x, y)
        if hit is not None and abs(hit[1]) > 0.5:
            hit = None
        last = hit[0] if hit else last
        out.append(hit)
    return out


def _straight(line, s):
    if s < 0.5 or s > line.length - 0.5:
        return False
    h0, h1 = line.point(s - 0.5)[2], line.point(s + 0.5)[2]
    return math.degrees(tg.heading_error(h0, h1)) < STRAIGHT_DEG_PER_M


def _cluster(reads):
    """Group reads into strips: by key (an RFID tag) and by position along the path."""
    out = {}
    for key in sorted({x["key"] for x in reads}):
        xs = sorted((x for x in reads if x["key"] == key), key=lambda x: x["s"])
        group, k = [], 0
        for x in xs:
            if group and x["s"] - group[-1]["s"] > STRIP_GAP_M:
                k += 1
                out[f"{key}#{k}" if k else key] = group
                group = []
            group.append(x)
        if group:
            out[f"{key}#{k + 1}" if k else key] = group
    return dict(sorted(out.items(), key=lambda kv: statistics.mean(x["s"] for x in kv[1])))


def _scale(strips):
    """Wheel travel / reference distance between consecutive strips, per run -> ratios - 1."""
    names = list(strips)
    ratios = []
    for a, b in zip(names, names[1:], strict=False):
        sa = {x["run"]: x for x in strips[a]}
        for x in strips[b]:
            y = sa.get(x["run"])
            ds = x["s"] - y["s"] if y else 0.0
            if y and ds > 2.0:
                ratios.append((x["travel"] - y["travel"]) / ds - 1.0)
    return ratios


def verdict(ok):
    return "PASS" if ok else "FAIL"


def report(res, cpu=None, fallback=False):
    lines = []
    for d in ("out", "back"):
        r = res.get(d)
        if r is None:
            lines.append(f"{d}: no AMCL track")
            continue
        lines.append(f"{d}: reference {r['length']:.1f} m")
        for name, v in r["M1"].items():
            ok = not math.isnan(v) and v <= TARGET["M1"]
            note = " (RFID FALLBACK: read zone scatter included)" if fallback else ""
            lines.append(f"  M1 {name:>12}: 2 sigma {v * 100:6.1f} cm over {r['M1_n'][name]:3d} reads  "
                         f"{verdict(ok)}{note}")
        if not r["M1"]:
            lines.append("  M1: no reference reads (code 3 or RFID) in this direction")
        lines.append(f"  M2 lateral: 2 sigma {r['M2'] * 100:.1f} cm over {r['M2_n']} poses  "
                     f"{verdict(r['M2'] <= TARGET['M2'])}")
        m6 = r["M6"]
        if m6:
            mean = statistics.mean(m6)
            lines.append(f"  M6 odometry scale: {mean * 100:+.2f} % (spread {_two_sigma(m6) * 100:.2f} %, "
                         f"{len(m6)} spans)  {verdict(abs(mean) <= TARGET['M6'])}"
                         + ("  -> check vehicle.wheel_dia_m first" if abs(mean) > TARGET["M6"] else ""))
        else:
            lines.append("  M6: fewer than two strips per run")
    a = res["all"]
    lines.append(f"M3 jumps (> {JUMP_M} m off the wheels in one update): {len(a['M3'])}  "
                 f"{verdict(not a['M3'])}")
    for name, t, dm in a["M3"][:10]:
        lines.append(f"    {name} at bag time {t:.1f}: {dm:+.2f} m")
    lines.append(f"M4 latency p95: {a['M4']:.3f} s  {verdict(a['M4'] <= TARGET['M4'])}")
    if cpu:
        lines.append(f"M5 CPU busy max {cpu['cpu_max']:.0f} % mean {cpu['cpu_mean']:.0f} %, "
                     f"temperature max {cpu['temp_max']:.1f} C  "
                     f"{verdict(cpu['cpu_max'] < TARGET['M5_cpu'] and cpu['temp_max'] < TARGET['M5_temp'])}")
        for name, pct in cpu.get("top", [])[:8]:
            lines.append(f"    {name:<28} {pct:5.1f} % of one core")
    else:
        lines.append("M5: no cpu_temp.csv (run_log.sh before T2?)")
    return lines


# -- reading the runs -------------------------------------------------------------------

def _stamp(s):
    return s.sec + s.nanosec * 1e-9


def read_run(path):
    """One run directory -> the dict analyse() takes, and whether it used the RFID fallback."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from bag_report import read_bag  # noqa: PLC0415 - needs rosbag2_py (amr_ws sourced)

    from agv_core import config  # noqa: PLC0415

    bag = read_bag(os.path.join(path, "bag"))
    r = config.WHEEL_DIA_M / 2.0
    wheels, base = [], None
    for _t, m in bag.get("/wheel_states", []):
        if not (m.left_valid and m.right_valid):
            continue
        pos = r * (m.left_pos_rad + m.right_pos_rad) / 2.0
        base = pos if base is None else base
        wheels.append((_stamp(m.header.stamp), pos - base))
    poses = [(t, _stamp(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y)
             for t, m in bag.get("/amcl_pose", [])]
    events = [(_stamp(m.stamp), "code3") for _t, m in bag.get("/amr/line_marker", [])
              if not m.heartbeat and m.code == 3]
    fallback = not events
    if fallback:
        events = [(_stamp(m.header.stamp), f"rfid:{m.rfid_tag}") for _t, m in bag.get("/amr/rfid", [])
                  if not m.heartbeat and m.rfid_tag]
    pivots, start = [], None
    for _t, m in bag.get("/amr/line_state", []):
        spinning = m.uturn_phase in ("spin", "center", "settle")
        stamp = _stamp(m.header.stamp)
        if spinning and start is None:
            start = stamp
        elif not spinning and start is not None:
            pivots.append((start, stamp))
            start = None
    return {"name": os.path.basename(os.path.normpath(path)), "wheels": wheels, "poses": poses,
            "events": events, "pivots": pivots}, fallback


def read_cpu(paths):
    busy, temp, top = [], [], {}
    for p in paths:
        try:
            with open(os.path.join(p, "cpu_temp.csv")) as fh:
                for row in csv.DictReader(fh):
                    busy.append(float(row["cpu_busy_pct"]))
                    if row["temp_c"]:
                        temp.append(float(row["temp_c"]))
        except OSError:
            continue
        try:
            with open(os.path.join(p, "pidstat.txt")) as fh:
                head = None
                for line in fh:
                    f = line.split()
                    if line.startswith("#"):
                        head = f[1:]
                        continue
                    if head and len(f) == len(head) and "%CPU" in head:
                        top.setdefault(f[head.index("Command")], []).append(float(f[head.index("%CPU")]))
        except (OSError, ValueError):
            pass
    if not busy:
        return None
    ranked = sorted(((k, statistics.mean(v)) for k, v in top.items()), key=lambda kv: -kv[1])
    return {"cpu_max": max(busy), "cpu_mean": statistics.mean(busy),
            "temp_max": max(temp) if temp else float("nan"), "top": ranked}


def main(paths):
    runs, fallback = [], False
    for p in paths:
        run, fb = read_run(p)
        fallback = fallback or fb
        runs.append(run)
        print(f"{run['name']}: {len(run['poses'])} AMCL poses, {len(run['events'])} reference reads, "
              f"{len(run['pivots'])} pivot(s)" + (" (RFID fallback)" if fb else ""))
    print()
    for line in report(analyse(runs), read_cpu(paths), fallback):
        print(line)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
