"""The taught tape graph, and one circuit compiled into a route (tracked plan 3, section 2.2).

Pure: no ROS, no clock, no I/O. Dicts in, dicts out; the store (tape_graph_store) owns
the files and the teach builder (tape_teach) makes the legs.

    leg       one directed stretch of tape between two NODES of a circuit: a laid
              junction (where the MLS sees a single track again after the fork) or a
              tape-end U-turn. Id "<from>:<order>><to>", e.g. "J-N:right>X1": the order
              taken at the from-node names the tape, so the north and south bays (both
              J-SPLIT -> J-BMERGE) are two legs. The two directions of one tape are two
              legs, each taught on its own passes (their stations differ anyway).
    station   a taught stop point: its leg and the distance s along it.
    route     one circuit laid end to end, HOME to HOME: the home leg, the circuit's
              legs, the home leg again. Everything the locator and the trip need is a
              distance S along it: stops, the junctions (fork start and end), the U-turn.

Leg geometry is the mean of the taught AMCL passes, resampled every STEP_M, in the map
frame of the bound map revision. s is the polyline's own arc length: wheel odometry
only carries the locator between corrections.

A graph is BOUND to one map revision (map_id, revision, sha256 of the bundle). Taught
under another revision, every station is somewhere else: refused at load, never
"close enough".
"""

import bisect
import math

GRAPH_FORMAT = 1
STEP_M = 0.05
# A station must project this close to its leg when it is taught or loaded.
STATION_MAX_LATERAL_M = 0.10
# The spread floor for junction resyncs: a single pass has no spread to measure.
MIN_END_SPREAD_M = 0.02
# Projection ties: points this close to equally near are one tie, broken by the hint (the
# predicted s). A tape driven both ways inside one window - the turnaround at the end of a
# straight - is equally near on both sides; the nearest point alone would pick at random.
TIE_M = 0.03


class GraphError(ValueError):
    """A graph that cannot be used as written: refused, with the reason."""


def leg_id(frm, order, to):
    return f"{frm}:{order}>{to}"


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def resample(pts, step=STEP_M):
    """A polyline resampled to points `step` apart along its length (ends kept)."""
    pts = [(float(x), float(y)) for x, y in pts]
    out = [pts[0]] if pts else []
    if len(pts) < 2:
        return out
    carry = 0.0  # length walked since the last output point
    for (x0, y0), (x1, y1) in zip(pts, pts[1:], strict=False):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg == 0.0:
            continue
        pos = step - carry
        while pos <= seg + 1e-12:
            f = pos / seg
            out.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0)))
            pos += step
        carry = seg - (pos - step)
    if math.hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1]) > 1e-6:
        out.append(pts[-1])
    return out


class Polyline:
    """Points with their cumulative arc length. Project and interpolate by s."""

    def __init__(self, pts):
        pts = [(float(x), float(y)) for x, y in pts]
        if len(pts) < 2:
            raise GraphError("a polyline needs at least two points")
        s = [0.0]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:], strict=False):
            s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
        if s[-1] <= 0.0:
            raise GraphError("a polyline of zero length")
        self.pts, self.s = pts, s

    @property
    def length(self):
        return self.s[-1]

    def _seg(self, s):
        return min(max(bisect.bisect_right(self.s, s) - 1, 0), len(self.pts) - 2)

    def point(self, s):
        """(x, y, heading) at s, clamped to the ends."""
        s = min(max(s, 0.0), self.length)
        i = self._seg(s)
        (x0, y0), (x1, y1) = self.pts[i], self.pts[i + 1]
        seg = self.s[i + 1] - self.s[i]
        f = 0.0 if seg <= 0 else (s - self.s[i]) / seg
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0), math.atan2(y1 - y0, x1 - x0)

    def cut(self, s0, s1):
        """The points from s0 to s1 (interpolated ends)."""
        s0, s1 = max(0.0, s0), min(self.length, s1)
        pts = [self.point(s0)[:2]]
        pts += [p for p, s in zip(self.pts, self.s, strict=True) if s0 < s < s1]
        pts.append(self.point(s1)[:2])
        return pts

    def project(self, x, y, s_lo=None, s_hi=None, hint=None):
        """Nearest point within [s_lo, s_hi] -> (s, d), d signed (+ = left of travel),
        or None when the window misses the line. With a hint, of the points within TIE_M of
        the nearest, the one closest to s = hint."""
        hits = self._candidates(x, y, s_lo, s_hi)
        return _pick(hits, hint)

    def _candidates(self, x, y, s_lo, s_hi):
        lo = 0.0 if s_lo is None else max(0.0, s_lo)
        hi = self.length if s_hi is None else min(self.length, s_hi)
        if hi < lo:
            return []
        out = []
        for i in range(self._seg(lo), self._seg(hi) + 1):
            (x0, y0), (x1, y1) = self.pts[i], self.pts[i + 1]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            f = 0.0 if seg2 == 0 else ((x - x0) * dx + (y - y0) * dy) / seg2
            s = self.s[i] + max(0.0, min(1.0, f)) * (self.s[i + 1] - self.s[i])
            s = min(max(s, lo), hi)
            px, py, _ = self.point(s)
            dist = math.hypot(x - px, y - py)
            cross = dx * (y - py) - dy * (x - px)
            out.append((s, math.copysign(dist, cross) if dist > 0 else 0.0, dist))
        return out


def _pick(hits, hint):
    """(s, d) of the nearest hit; with a hint, the tied hit closest to it. Hits come in s
    order, one per segment: only local minima of the distance are separate passes of the
    tape (a neighbouring segment's clamped end is the same pass, slightly farther)."""
    if not hits:
        return None
    best = min(h[2] for h in hits)
    minima = [h for k, h in enumerate(hits)
              if (k == 0 or h[2] <= hits[k - 1][2]) and (k == len(hits) - 1 or h[2] <= hits[k + 1][2])]
    tied = [h for h in minima if h[2] <= best + TIE_M] if hint is not None else []
    h = min(tied, key=lambda h: (abs(h[0] - hint), h[2])) if tied else min(hits, key=lambda h: h[2])
    return h[0], h[1]


class Leg:
    def __init__(self, ident, frm, order, to, pts, *, passes=1, spread_m=0.0, end_spread_m=0.0,
                 fork_back_m=0.0, end_kind="fork"):
        if leg_id(frm, order, to) != ident:
            raise GraphError(f"leg {ident!r}: id does not match {leg_id(frm, order, to)!r}")
        if end_kind not in ("fork", "uturn", "none"):
            raise GraphError(f"leg {ident}: end_kind {end_kind!r}")
        self.id, self.frm, self.order, self.to = ident, frm, order, to
        self.line = Polyline(pts)
        self.passes = int(passes)
        self.spread_m = float(spread_m)          # largest lateral spread between passes
        self.end_spread_m = float(end_spread_m)  # spread of the end node along the tape
        self.fork_back_m = float(fork_back_m)    # multi-track first seen this far before the end
        self.end_kind = end_kind

    @property
    def length(self):
        return self.line.length

    def to_dict(self):
        return {"id": self.id, "from": self.frm, "order": self.order, "to": self.to,
                "points": [[round(x, 4), round(y, 4)] for x, y in self.line.pts],
                "passes": self.passes, "spread_m": round(self.spread_m, 4),
                "end_spread_m": round(self.end_spread_m, 4), "fork_back_m": round(self.fork_back_m, 4),
                "end_kind": self.end_kind}

    @classmethod
    def from_dict(cls, d):
        try:
            return cls(d["id"], d["from"], d["order"], d["to"], d["points"], passes=d["passes"],
                       spread_m=d["spread_m"], end_spread_m=d["end_spread_m"],
                       fork_back_m=d["fork_back_m"], end_kind=d["end_kind"])
        except (KeyError, TypeError) as e:
            raise GraphError(f"leg {d.get('id') if isinstance(d, dict) else d!r}: {e}") from None


class Station:
    def __init__(self, ident, leg, s, d=0.0, pose=(0.0, 0.0, 0.0), taught="", samples=0):
        self.id, self.leg, self.s, self.d = ident, leg, float(s), float(d)
        self.pose = tuple(float(v) for v in pose)
        self.taught, self.samples = str(taught), int(samples)

    def to_dict(self):
        return {"id": self.id, "leg": self.leg, "s": round(self.s, 4), "d": round(self.d, 4),
                "pose": [round(v, 4) for v in self.pose], "taught": self.taught, "samples": self.samples}

    @classmethod
    def from_dict(cls, d):
        try:
            return cls(d["id"], d["leg"], d["s"], d["d"], d["pose"], d["taught"], d["samples"])
        except (KeyError, TypeError) as e:
            raise GraphError(f"station {d.get('id') if isinstance(d, dict) else d!r}: {e}") from None


BINDING_KEYS = ("map_id", "map_revision", "map_sha256")


class Graph:
    def __init__(self, binding, legs=(), stations=(), revision=None, mission="", notes=""):
        if set(binding) != set(BINDING_KEYS):
            raise GraphError(f"binding: expected {list(BINDING_KEYS)}, got {sorted(binding)}")
        self.binding = {"map_id": str(binding["map_id"]), "map_revision": int(binding["map_revision"]),
                        "map_sha256": str(binding["map_sha256"])}
        self.revision = revision
        self.mission = mission
        self.notes = notes
        self.legs = {}
        self.stations = {}
        for leg in legs:
            if leg.id in self.legs:
                raise GraphError(f"leg {leg.id} twice")
            self.legs[leg.id] = leg
        for st in stations:
            self.put_station(st)

    def put_station(self, st):
        leg = self.legs.get(st.leg)
        if leg is None:
            raise GraphError(f"station {st.id}: leg {st.leg} is not in the graph")
        if not -1e-6 <= st.s <= leg.length + 1e-6:
            raise GraphError(f"station {st.id}: s {st.s:.3f} outside leg {st.leg} (0..{leg.length:.3f})")
        if abs(st.d) > STATION_MAX_LATERAL_M:
            raise GraphError(f"station {st.id}: {st.d:+.3f} m from its leg (> {STATION_MAX_LATERAL_M} m)")
        self.stations[st.id] = st

    def legs_from(self, node):
        return [leg for leg in self.legs.values() if leg.frm == node]

    def check_binding(self, map_id, map_revision, map_sha256):
        """'' when this graph belongs to that map revision, else why not."""
        b = self.binding
        if b["map_id"] != map_id or b["map_revision"] != int(map_revision):
            return (f"graph g{self.revision} is for map {b['map_id']} rev{b['map_revision']}, "
                    f"the active map is {map_id} rev{map_revision}")
        if b["map_sha256"] != map_sha256:
            return (f"graph g{self.revision}: map {map_id} rev{map_revision} bundle hash changed "
                    f"({b['map_sha256'][:12]} taught, {str(map_sha256)[:12]} now)")
        return ""

    def to_dict(self):
        return {"format": GRAPH_FORMAT, "binding": dict(self.binding), "revision": self.revision,
                "mission": self.mission, "notes": self.notes,
                "legs": [leg.to_dict() for leg in sorted(self.legs.values(), key=lambda g: g.id)],
                "stations": [st.to_dict() for st in sorted(self.stations.values(), key=lambda s: s.id)]}

    @classmethod
    def from_dict(cls, d):
        if not isinstance(d, dict):
            raise GraphError("graph: expected an object")
        if d.get("format") != GRAPH_FORMAT:
            raise GraphError(f"graph format {d.get('format')!r}, this code reads {GRAPH_FORMAT}")
        try:
            legs = [Leg.from_dict(x) for x in d["legs"]]
            g = cls(d["binding"], legs, revision=d.get("revision"), mission=d.get("mission", ""),
                    notes=d.get("notes", ""))
            for x in d["stations"]:
                g.put_station(Station.from_dict(x))
        except (KeyError, TypeError) as e:
            raise GraphError(f"graph: {e}") from None
        return g


# -- the route -----------------------------------------------------------------------


class RouteError(GraphError):
    """A circuit the graph cannot drive yet: legs or stations not taught."""


class Route:
    """One circuit, HOME to HOME, as a single distance S.

    segments  [(S0, leg)] in travel order; the home leg is first and last
    stops     [{"id", "S"}] every stop item of the circuit, HOME first and last
    nodes     [{"id", "kind": "junction"|"uturn", "order", "S", "S_fork", "sigma", "leg_in"}]:
              S is where the leg in ends (the single track again, or the tape end);
              S_fork where the multi-track began on the teach passes
    """

    def __init__(self, circuit_id, segments, stops, nodes):
        self.circuit = circuit_id
        self.segments = segments
        self._starts = [s0 for s0, _ in segments]
        self.stops = stops
        self.nodes = nodes
        self.S_start = stops[0]["S"]
        self.S_end = stops[-1]["S"]

    @property
    def length(self):
        s0, leg = self.segments[-1]
        return s0 + leg.length

    def at(self, S):
        """(index, leg, s) of the segment holding S (clamped to the route)."""
        i = min(max(bisect.bisect_right(self._starts, S) - 1, 0), len(self.segments) - 1)
        s0, leg = self.segments[i]
        return i, leg, min(max(S - s0, 0.0), leg.length)

    def point(self, S):
        _, leg, s = self.at(S)
        return leg.line.point(s)

    def project(self, x, y, S_lo, S_hi, hint=None):
        """Nearest route point to (x, y) with S in [S_lo, S_hi] -> (S, d) or None.

        The window is what keeps the two passes over one tape apart (the MRU line west,
        then east): the same floor point has two S values, and only the predicted one is
        inside the window. Inside the window, the hint (the predicted S) breaks a tie: a
        turnaround leaves both directions of the tape in one window."""
        hits = []
        for s0, leg in self.segments:
            if s0 > S_hi or s0 + leg.length < S_lo:
                continue
            hits += [(s0 + s, d, dist) for s, d, dist in leg.line._candidates(x, y, S_lo - s0, S_hi - s0)]
        return _pick(hits, hint)

    def stop(self, ident):
        return next((s for s in self.stops if s["id"] == ident), None)


def compile_route(circuit_id, circuit, graph):
    """A parsed mission circuit (agv_core.mission site schema) + a graph -> Route.

    RouteError names everything not taught yet, so the Teach page can list it."""
    legs = circuit["legs"]           # [{"id", "from", "order", "to", "stops": [...]}], home leg last
    home_leg = legs[-1]
    order = [home_leg] + legs[:-1] + [home_leg]
    missing = sorted({leg["id"] for leg in legs if leg["id"] not in graph.legs})
    stops_needed = [sid for leg in legs for sid in leg["stops"]]
    missing += sorted({f"station {sid}" for sid in stops_needed if sid not in graph.stations})
    if missing:
        raise RouteError(f"{circuit_id} not taught: {', '.join(missing)}")

    segments, S0 = [], 0.0
    for spec in order:
        leg = graph.legs[spec["id"]]
        segments.append((S0, leg))
        S0 += leg.length

    stops = []
    # The home leg is driven twice: out of the bay (the stops before the first node, HOME
    # at the start) and back into it (the stops after the last node, HOME at the end).
    for k, spec in enumerate(order):
        s0, _leg = segments[k]
        if k == 0:
            ids = circuit["home_depart"]
        elif k == len(order) - 1:
            ids = circuit["home_arrive"]
        else:
            ids = spec["stops"]
        for sid in ids:
            st = graph.stations[sid]
            if st.leg != spec["id"]:
                raise RouteError(f"station {sid} is taught on {st.leg}, the circuit has it on {spec['id']}")
            stops.append({"id": sid, "S": s0 + st.s})
    for a, b in zip(stops, stops[1:], strict=False):
        if b["S"] < a["S"]:
            raise RouteError(f"{circuit_id}: station {b['id']} is taught before {a['id']} on the tape")

    nodes = []
    for k in range(1, len(order)):
        s0, leg = segments[k]
        _p0, prev = segments[k - 1]
        kind = "uturn" if prev.end_kind == "uturn" else "junction"
        nodes.append({"id": leg.frm, "kind": kind, "order": leg.order, "S": s0,
                      "S_fork": s0 - prev.fork_back_m if prev.end_kind == "fork" else s0,
                      "sigma": max(prev.end_spread_m, MIN_END_SPREAD_M), "leg_in": prev.id,
                      "leg_out": leg.id, "fork": prev.end_kind == "fork"})
    return Route(circuit_id, segments, stops, nodes)


def lateral_to(leg, x, y):
    """|distance| from (x, y) to a leg anywhere along it."""
    hit = leg.line.project(x, y)
    return abs(hit[1]) if hit else math.inf


# -- the MLS fork episodes ----------------------------------------------------------

MULTI_NLCP = (3, 6, 7)
SINGLE_NLCP = 2


class ForkDetector:
    """MLS #LCP episodes along the wheel travel: more than one track, then one again.

    The episode ENDS at the first single-track reading after the last multi-track one;
    it is reported once the single track has held for confirm_m, so a flicker inside a
    diverter is one episode, not two. #LCP 0 (no track) holds whatever state it finds.
    The same detector marks node positions when teaching and resyncs the locator, so
    both see the junction at the same point."""

    def __init__(self, confirm_m=0.10):
        self.confirm_m = float(confirm_m)
        self.reset()

    def reset(self):
        self._start = None   # travel at the first multi-track reading
        self._end = None     # travel at the first single after the last multi
        self.multi = False

    def update(self, travel, nlcp):
        """One reading. Returns (start, end, crossing) when an episode is confirmed."""
        if nlcp in MULTI_NLCP:
            if self._start is None:
                self._start, self._cross = travel, False
            self._cross = self._cross or nlcp == 7
            self._end = None
            self.multi = True
            return None
        self.multi = False
        if nlcp != SINGLE_NLCP or self._start is None:
            return None
        if self._end is None:
            self._end = travel
        if travel - self._end >= self.confirm_m:
            ep = (self._start, self._end, self._cross)
            self.reset()
            return ep
        return None


def heading_error(a, b):
    return abs(_wrap(a - b))
