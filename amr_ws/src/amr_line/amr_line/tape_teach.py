"""Teaching the tape graph (tracked plan 3, D3: teach and repeat, never draw). Pure.

A PASS is one full circuit, HOME bay to HOME bay, driven on the tape (an ordinary
LINE run or a supervised drive) while AMCL localises on the bound map revision. It
records AMCL positions against wheel travel, the MLS fork episodes and the U-turn.

build() cuts every pass at its NODES - the laid junctions and U-turns of the circuit,
in order - and averages each leg over its passes:

  - a junction node is where the MLS saw a single track again (tape_graph.ForkDetector),
    the same point the locator resyncs at;
  - a U-turn node is where the approach stopped at the tape end;
  - the home leg is the end of the pass (after the last node) joined to its start
    (before the first node): park where the pass began.

A pass whose events do not match the circuit's nodes is refused with both sequences:
an MLS that missed a fork, or a circuit driven differently from the mission, would
otherwise teach a leg that is two legs. Stations are taught separately, at standstill
under MANUAL, on legs that are already built.
"""

import datetime
import math
import statistics

from amr_line import tape_graph as tg
from amr_line.tape_locator import OdomBuffer

# A home leg whose pass ended this far from where it started is refused.
HOME_GAP_M = 0.5
# Quality flags for the Teach page (plan section 4, P1).
MIN_PASSES = 3
MAX_SPREAD_M = 0.05
# Averaging passes: how far along a pass to look for the point matching the reference.
WINDOW_M = 0.5


class TeachError(ValueError):
    pass


class PassRecorder:
    """One pass being recorded. Fed by the node every tick and on every AMCL pose."""

    def __init__(self, circuit, started=""):
        self.circuit = circuit
        self.started = started
        self.samples = []     # [travel, x, y]
        self.forks = []       # [start, end, crossing]
        self.uturns = []      # [travel]
        self.odom = OdomBuffer(span_s=3.0)
        self.forks_seen = tg.ForkDetector()
        self.dropped = 0      # poses without odometry at their stamp

    def tick(self, t, travel, nlcp):
        self.odom.add(t, travel)
        ep = self.forks_seen.update(travel, nlcp)
        if ep is not None:
            self.forks.append([ep[0], ep[1], ep[2]])

    def pose(self, stamp, x, y):
        tr = self.odom.at(stamp)
        if tr is None:
            self.dropped += 1
            return
        if self.samples and tr < self.samples[-1][0] - 0.02:
            self.dropped += 1   # reversing (a jog back): not a pass any more
            return
        self.samples.append([tr, float(x), float(y)])

    def uturn(self, travel):
        self.uturns.append(float(travel))

    def summary(self):
        length = self.samples[-1][0] - self.samples[0][0] if len(self.samples) > 1 else 0.0
        return (f"{self.circuit}: {length:.1f} m, {len(self.samples)} poses, {len(self.forks)} forks, "
                f"{len(self.uturns)} U-turn(s), {self.dropped} poses dropped")

    def to_dict(self):
        return {"circuit": self.circuit, "started": self.started, "samples": self.samples,
                "forks": self.forks, "uturns": self.uturns}


def _interp(samples, travel):
    """(x, y) at a wheel travel inside the samples."""
    lo = 0
    for i in range(len(samples) - 1):
        if samples[i + 1][0] >= travel:
            lo = i
            break
    else:
        lo = len(samples) - 2
    (t0, x0, y0), (t1, x1, y1) = samples[lo], samples[lo + 1]
    f = 0.0 if t1 == t0 else min(max((travel - t0) / (t1 - t0), 0.0), 1.0)
    return x0 + f * (x1 - x0), y0 + f * (y1 - y0)


def _cut(samples, t0, t1):
    pts = [_interp(samples, t0)] + [(x, y) for t, x, y in samples if t0 < t < t1] + [_interp(samples, t1)]
    return pts


def _nodes(mission, circuit_id):
    legs = mission["CIRCUITS"][circuit_id]["legs"]
    return [leg["from"] for leg in legs[:-1]] + [legs[-1]["from"]]


def split_pass(rec, mission):
    """One recorded pass -> {leg id: {"pts", "fork_back"}}. TeachError if it does not match."""
    cid = rec["circuit"]
    legs = mission["CIRCUITS"][cid]["legs"]
    nodes = _nodes(mission, cid)
    samples = sorted(rec["samples"])
    if len(samples) < 10:
        raise TeachError(f"pass {rec.get('started')}: only {len(samples)} AMCL poses")
    events = [(end, "fork", end - start) for start, end, _x in rec["forks"]]
    events += [(t, "uturn", 0.0) for t in rec["uturns"]]
    events = [e for e in sorted(events) if samples[0][0] < e[0] < samples[-1][0]]
    want = ["uturn" if n in mission["UTURNS"] else "fork" for n in nodes]
    got = [kind for _t, kind, _b in events]
    if got != want:
        raise TeachError(
            f"pass {rec.get('started')}: events {got} do not match the nodes of {cid} "
            f"{list(zip(nodes, want, strict=True))} - a missed or extra MLS fork, or a different route")
    b = [t for t, _k, _ in events]
    out = {}
    for i, leg in enumerate(legs[:-1]):
        out[leg["id"]] = {"pts": _cut(samples, b[i], b[i + 1]), "fork_back": events[i + 1][2],
                          "end_kind": events[i + 1][1]}
    # The home leg: from the last node to the end of the pass, then the start of the pass
    # to the first node.
    suffix = _cut(samples, b[-1], samples[-1][0])
    prefix = _cut(samples, samples[0][0], b[0])
    home = legs[-1]["id"]
    if len(suffix) < 2:
        raise TeachError(f"pass {rec.get('started')}: ended at {nodes[-1]}, before the home leg")
    line = tg.Polyline(suffix)
    hit = line.project(*prefix[0])
    if hit and hit[0] < line.length - 1e-6 and abs(hit[1]) <= tg.STATION_MAX_LATERAL_M:
        pts = line.cut(0.0, hit[0]) + prefix[1:]
    else:
        gap = math.dist(suffix[-1], prefix[0])
        if gap > HOME_GAP_M:
            raise TeachError(f"pass {rec.get('started')}: ended {gap:.2f} m from where it started "
                             f"(> {HOME_GAP_M} m): park where the pass began")
        pts = suffix + prefix
    out[home] = {"pts": pts, "fork_back": events[0][2], "end_kind": events[0][1]}
    return out


def mean_leg(ident, frm, order, to, segs, fork_backs, end_kind):
    """Average a leg over its passes -> tape_graph.Leg."""
    lines = [tg.Polyline(tg.resample(p)) for p in segs if len(p) >= 2]
    if not lines:
        raise TeachError(f"leg {ident}: no usable pass")
    lines.sort(key=lambda g: g.length)
    ref = lines[len(lines) // 2]
    # Passes run parallel, so a point projects near the same fraction of every pass: the
    # window keeps it off another stretch of the same tape (a leg that turns back).
    pts = []
    for s_ref, (x, y) in zip(ref.s, ref.pts, strict=True):
        xs, ys = [x], [y]
        for other in lines:
            if other is ref:
                continue
            guess = s_ref * other.length / ref.length
            hit = other.project(x, y, guess - WINDOW_M, guess + WINDOW_M)
            if hit and 1e-6 < hit[0] < other.length - 1e-6:
                px, py, _ = other.point(hit[0])
                xs.append(px)
                ys.append(py)
        pts.append((sum(xs) / len(xs), sum(ys) / len(ys)))
    mean = tg.Polyline(tg.resample(pts))
    # Hinted: a leg may start and end at the same floor point (pivot to pivot on a straight).
    starts = [mean.project(*g.pts[0], hint=0.0)[0] for g in lines]
    ends = [mean.project(*g.pts[-1], hint=mean.length)[0] for g in lines]
    s0, s1 = statistics.median(starts), statistics.median(ends)
    spread = 0.0
    for g in lines:
        for s_g, (x, y) in zip(g.s[1:-1], g.pts[1:-1], strict=True):
            guess = s_g * mean.length / g.length
            hit = mean.project(x, y, guess - WINDOW_M, guess + WINDOW_M)
            if hit and 1e-6 < hit[0] < mean.length - 1e-6:
                spread = max(spread, abs(hit[1]))
    final = tg.resample(mean.cut(s0, s1))
    return tg.Leg(ident, frm, order, to, final, passes=len(lines), spread_m=spread,
                  end_spread_m=max(abs(e - s1) for e in ends),
                  fork_back_m=statistics.median(fork_backs) if end_kind == "fork" else 0.0,
                  end_kind=end_kind)


def build(passes, mission, binding, stations=(), notes=""):
    """Recorded passes (dicts) + the site mission -> (Graph, report lines).

    Refused passes are reported and left out; legs with no pass are simply not in the
    graph (tape_graph.compile_route names them). `stations` are taught station records
    (dicts with "id" and "pose"): re-projected onto the rebuilt legs."""
    report, per_leg = [], {}
    for rec in passes:
        if rec.get("circuit") not in mission["CIRCUITS"]:
            report.append(f"pass {rec.get('started')}: circuit {rec.get('circuit')!r} not in the mission")
            continue
        try:
            parts = split_pass(rec, mission)
        except (TeachError, tg.GraphError) as e:
            report.append(f"REFUSED {e}")
            continue
        for lid, part in parts.items():
            per_leg.setdefault(lid, []).append(part)
    specs = {leg["id"]: leg for c in mission["CIRCUITS"].values() for leg in c["legs"]}
    legs = []
    for lid, parts in sorted(per_leg.items()):
        spec = specs[lid]
        kinds = {p["end_kind"] for p in parts}
        leg = mean_leg(lid, spec["from"], spec["order"], spec["to"], [p["pts"] for p in parts],
                       [p["fork_back"] for p in parts], kinds.pop() if len(kinds) == 1 else "fork")
        flag = ""
        if leg.passes < MIN_PASSES:
            flag += f" (needs {MIN_PASSES} passes)"
        if leg.spread_m > MAX_SPREAD_M:
            flag += f" (spread > {MAX_SPREAD_M} m: check AMCL on this stretch)"
        report.append(f"{lid}: {leg.length:.2f} m, {leg.passes} pass(es), spread {leg.spread_m:.3f} m, "
                      f"end spread {leg.end_spread_m:.3f} m{flag}")
        legs.append(leg)
    g = tg.Graph(binding, legs, mission=mission["MISSION_NAME"], notes=notes)
    for st in stations:
        try:
            g.put_station(project_station(g, mission, st["id"], st["pose"], st.get("taught", ""),
                                          st.get("samples", 0)))
        except tg.GraphError as e:
            report.append(f"station {st['id']} dropped: {e}")
    return g, report


def project_station(graph, mission, sid, pose, taught="", samples=0):
    """A station's taught pose -> tape_graph.Station on its leg (GraphError if not there)."""
    lid = mission["STATIONS"].get(sid)
    if lid is None:
        raise tg.GraphError(f"{sid} is not a station of {mission['MISSION_NAME']}")
    leg = graph.legs.get(lid)
    if leg is None:
        raise tg.GraphError(f"{sid} lies on {lid}, which is not taught yet: record its passes first")
    x, y, yaw = pose
    # Of the passes of the tape near the pose (a leg can run both ways past one point, as
    # through the turnaround of a test straight), the one running the way the vehicle faces.
    hits = leg.line._candidates(x, y, None, None)
    best = min(h[2] for h in hits)
    near = [h for k, h in enumerate(hits) if h[2] <= best + tg.TIE_M
            and (k == 0 or h[2] <= hits[k - 1][2]) and (k == len(hits) - 1 or h[2] <= hits[k + 1][2])]

    def facing(h):
        return min(tg.heading_error(yaw, leg.line.point(h[0] + k)[2]) for k in (-0.05, 0.05))

    s, d, _dist = min(near, key=facing)
    if facing((s, d, 0)) > math.radians(30):
        raise tg.GraphError(f"{sid}: the vehicle faces {math.degrees(yaw):.0f} deg, the leg runs "
                            f"{math.degrees(leg.line.point(s)[2]):.0f} deg here - wrong direction for {lid}")
    return tg.Station(sid, lid, s, d, pose, taught, samples)


def mean_pose(poses):
    """Mean of standstill AMCL poses [(x, y, yaw)] (circular mean on yaw)."""
    if not poses:
        raise TeachError("no AMCL pose while standing")
    n = len(poses)
    return (sum(p[0] for p in poses) / n, sum(p[1] for p in poses) / n,
            math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses)))


def now_iso():
    return datetime.datetime.now().isoformat(timespec="seconds")
