"""The one-dimensional tape locator (tracked plan 3, section 2.3). Pure and clock-fed.

State: the distance S along one compiled route (amr_line.tape_graph.Route), its
uncertainty sigma, and a health. The tape fixes lateral position and heading, so AMCL's
lateral output is only a health check (distance from the taught path).

    predict   signed forward wheel travel; sigma grows by k_odo x |travel|
    correct   an AMCL pose, taken AT ITS STAMP: the travel at the stamp comes from a
              2 s odometry buffer, the innovation is formed there and applied now
              (0.1 s of latency at 0.5 m/s is 5 cm, half the budget). Accepted only
              READY, fresh, near the taught path, and inside the innovation gate
    resync    an MLS fork episode at a known node, a completed U-turn
    health    HEALTHY  a correction accepted within healthy_window_m, sigma under the
                       station limit
              DEGRADED running on odometry; decisions allowed while sigma is under theirs
              LOST     readiness LOST/UNLOCALIZED/stale, lost_rejects rejected in a row,
                       sigma over sigma_lost_m, or a wrong branch (latched until restart).
                       Re-acquired only after readiness has left READY and come back (a
                       revalidation): AMCL that disagreed with the wheels is not trusted
                       just because it is still READY

Not in the safety path (plan D6): it says where the vehicle is along the tape. Every
consumer turns doubt into a hold, never into motion.
"""

import collections
import dataclasses
import math

from amr_line import tape_graph as tg

HEALTHY, DEGRADED, LOST = "HEALTHY", "DEGRADED", "LOST"
# A LOST locator re-acquires (readiness READY again) on two poses that agree this well.
REACQUIRE_AGREE_M = 0.05
REACQUIRE_MIN_WINDOW_M = 1.0


@dataclasses.dataclass(frozen=True)
class Params:
    """The profile's tape_loc block. Provisional values until P0 (plan section 5)."""

    k_odo: float = 0.01               # sigma growth per metre of wheel travel (P0 M6)
    amcl_sigma_floor_m: float = 0.03  # AMCL along-track sigma floor (P0 M1)
    max_lateral_m: float = 0.10       # pose farther from the taught path is rejected (P0 M2)
    innov_min_m: float = 0.10         # innovation gate floor
    pose_max_age_s: float = 0.5
    sigma_station_m: float = 0.025    # station stop limit (2 sigma <= 5 cm)
    sigma_junction_m: float = 0.20
    junction_lead_m: float = 1.0      # junction order latched by S_fork - lead
    lost_rejects: int = 5
    sigma_lost_m: float = 0.5
    healthy_window_m: float = 2.0
    wrong_branch_sep_m: float = 0.15
    wrong_branch_margin_m: float = 0.05
    wrong_branch_run_m: float = 0.5
    wrong_branch_window_m: float = 3.0

    @classmethod
    def from_profile(cls, v):
        return cls(k_odo=v.TAPE_LOC_K_ODO, amcl_sigma_floor_m=v.TAPE_LOC_AMCL_SIGMA_FLOOR_M,
                   max_lateral_m=v.TAPE_LOC_MAX_LATERAL_M, innov_min_m=v.TAPE_LOC_INNOV_MIN_M,
                   pose_max_age_s=v.TAPE_LOC_POSE_MAX_AGE_S, sigma_station_m=v.TAPE_LOC_SIGMA_STATION_M,
                   sigma_junction_m=v.TAPE_LOC_SIGMA_JUNCTION_M, junction_lead_m=v.TAPE_LOC_JUNCTION_LEAD_M,
                   lost_rejects=v.TAPE_LOC_LOST_REJECTS, sigma_lost_m=v.TAPE_LOC_SIGMA_LOST_M,
                   healthy_window_m=v.TAPE_LOC_HEALTHY_WINDOW_M)


class OdomBuffer:
    """(t, travel) samples for span_s; interpolated travel at a past stamp."""

    def __init__(self, span_s=2.0):
        self.span_s = float(span_s)
        self._q = collections.deque()

    def add(self, t, travel):
        q = self._q
        if q and t <= q[-1][0]:
            return
        q.append((float(t), float(travel)))
        while len(q) > 2 and q[1][0] < t - self.span_s:
            q.popleft()

    def at(self, t):
        """Travel at time t, or None outside the buffer (too old, or newer than the last)."""
        q = self._q
        if not q or t < q[0][0] or t > q[-1][0] + 0.05:
            return None
        if t >= q[-1][0]:
            return q[-1][1]
        lo, hi = 0, len(q) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if q[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (t0, s0), (t1, s1) = q[lo], q[hi]
        return s0 if t1 == t0 else s0 + (s1 - s0) * (t - t0) / (t1 - t0)

    def clear(self):
        self._q.clear()


class TapeLocator:
    def __init__(self, route, params=None, graph=None):
        self.route = route
        self.p = params or Params()
        self.odom = OdomBuffer()
        # Wrong-branch candidates: at each ordered diverge, the other taught legs from that node.
        self._alts = {}
        if graph is not None:
            for n in route.nodes:
                if n["kind"] == "junction" and n["order"] in ("left", "right", "straight"):
                    alts = [leg for leg in graph.legs_from(n["id"]) if leg.id != n["leg_out"]]
                    if alts:
                        self._alts[n["S"]] = (n, alts)
        self.started = False
        self.S = self.sigma = 0.0
        self.travel = None
        self.lost = ""                 # why LOST, "" when not
        self.wrong_branch = ""         # latched: the junction and the leg it fits better
        self.rejects = 0               # in a row
        self.rejected = self.accepted = 0
        self.last_reject = ""
        self.last_innov = None
        self.last_lateral = None
        self._accept_travel = None
        self._reacq = None             # last (S_meas at now) seen while LOST
        self._wb_run = 0.0
        self._wb_travel = None
        self._ready = False
        self._revalidated = False      # readiness left READY and came back since LOST

    # -- lifecycle -----------------------------------------------------------------
    def start(self, S, sigma, t, travel):
        self.started = True
        self.S, self.sigma = float(S), float(sigma)
        self.travel = float(travel)
        self.odom.clear()
        self.odom.add(t, travel)
        self.lost = self.wrong_branch = ""
        self._revalidated = False
        self.rejects = 0
        self._accept_travel = self.travel
        self._reacq = None
        self._wb_run, self._wb_travel = 0.0, None

    # -- inputs ----------------------------------------------------------------------
    def predict(self, t, travel):
        """Cumulative signed forward wheel travel at time t (the node integrates the
        wheel positions; a pivot adds ~0)."""
        if self.travel is None:
            self.travel = float(travel)
        d = float(travel) - self.travel
        self.travel = float(travel)
        self.odom.add(t, travel)
        if not self.started:
            return
        self.S += d
        self.sigma += self.p.k_odo * abs(d)
        if self.sigma > self.p.sigma_lost_m and not self.lost:
            self.lost = f"sigma {self.sigma:.2f} m > {self.p.sigma_lost_m:g} m without a correction"

    def readiness(self, ready, why=""):
        """Readiness READY (fresh) or not. LOST/UNLOCALIZED/stale makes the locator LOST;
        it re-acquires on its own once READY again (correct())."""
        if self.lost and ready and not self._ready:
            self._revalidated = True
        self._ready = bool(ready)
        if self.started and not ready and why and not self.lost:
            self.lost = f"localization {why}"

    def correct(self, now, stamp, x, y, var_along=0.0):
        """One AMCL pose. Returns "" when accepted, else why it was rejected."""
        if not self.started:
            return "not started"
        if self.wrong_branch:
            return self._reject("wrong branch latched", count=False)
        if not self._ready:
            return self._reject("localization not READY", count=False)
        if now - stamp > self.p.pose_max_age_s:
            return self._reject(f"pose {now - stamp:.2f} s old", count=False)
        tp = self.odom.at(stamp)
        if tp is None:
            return self._reject("no odometry at the pose stamp", count=False)
        back = self.travel - tp
        S_p = self.S - back
        r = max(math.sqrt(max(var_along, 0.0)), self.p.amcl_sigma_floor_m)
        w = max(4.0 * self.sigma, 0.5)
        hit = self.route.project(x, y, S_p - w, S_p + w, hint=S_p)
        if hit is not None:
            # Before anything else: the rejections a wrong branch causes must not hide it.
            self._wrong_branch_check(S_p, x, y, abs(hit[1]), tp)
            if self.wrong_branch:
                return self._reject(self.wrong_branch, count=False)
        if self.lost:
            if not self._revalidated:
                return self._reject(f"LOST ({self.lost}): needs localization READY again", count=False)
            return self._reacquire(S_p, back, x, y, r)
        if hit is None:
            return self._reject("pose projects off the route window")
        S_m, d = hit
        self.last_lateral = d
        if abs(S_m - S_p) >= w - 1e-6:
            return self._reject(f"innovation beyond the {w:.2f} m window")
        if abs(d) > self.p.max_lateral_m:
            return self._reject(f"{d:+.2f} m from the taught path")
        innov = S_m - S_p
        gate = max(3.0 * math.hypot(self.sigma, r), self.p.innov_min_m)
        if abs(innov) > gate:
            return self._reject(f"innovation {innov:+.2f} m > {gate:.2f} m")
        k = self.sigma ** 2 / (self.sigma ** 2 + r ** 2)
        self.S += k * innov
        self.sigma = math.sqrt((1.0 - k) * self.sigma ** 2)
        self.last_innov = innov
        self.rejects = 0
        self.accepted += 1
        self._accept_travel = self.travel
        return ""

    def _reject(self, why, count=True):
        self.last_reject = why
        self.rejected += 1
        if count:
            self.rejects += 1
            if self.rejects >= self.p.lost_rejects and not self.lost:
                self.lost = f"{self.rejects} corrections rejected in a row ({why})"
        return why

    def _reacquire(self, S_p, back, x, y, r):
        """LOST, READY again: two consecutive poses that agree re-seed S (not a wrong branch)."""
        w = max(3.0 * self.sigma, REACQUIRE_MIN_WINDOW_M)
        hit = self.route.project(x, y, S_p - w, S_p + w, hint=S_p)
        if hit is None or abs(hit[1]) > self.p.max_lateral_m:
            self._reacq = None
            return self._reject("LOST: pose not on the route near the last estimate", count=False)
        S_now = hit[0] + back
        prev, self._reacq = self._reacq, S_now
        if prev is None or abs(prev - S_now) > REACQUIRE_AGREE_M:
            return self._reject("LOST: waiting for a second consistent pose", count=False)
        self.S, self.sigma = S_now, r
        self.lost, self.rejects, self._reacq, self._revalidated = "", 0, None, False
        self._accept_travel = self.travel
        self.accepted += 1
        return ""

    def _wrong_branch_check(self, S_p, x, y, d_route, travel_p):
        p = self.p
        hit = None
        for S_node, (node, alts) in self._alts.items():
            if S_node <= S_p <= S_node + p.wrong_branch_window_m:
                hit = (node, alts, S_p - S_node)
                break
        if hit is None:
            self._wb_run, self._wb_travel = 0.0, None
            return
        node, alts, into = hit
        px, py, _ = self.route.point(S_p)
        lim = into + p.wrong_branch_window_m
        for alt in alts:
            on_alt = alt.line.project(px, py, 0.0, lim)
            here = alt.line.project(x, y, 0.0, lim)
            if on_alt is None or here is None:
                continue
            sep, d_alt = abs(on_alt[1]), abs(here[1])
            if sep >= p.wrong_branch_sep_m and d_alt + p.wrong_branch_margin_m < d_route:
                if self._wb_travel is not None:
                    self._wb_run += max(0.0, travel_p - self._wb_travel)
                self._wb_travel = travel_p
                if self._wb_run >= p.wrong_branch_run_m:
                    self.wrong_branch = (f"wrong branch at {node['id']}: ordered {node['order']} "
                                         f"({node['leg_out']}), the pose fits {alt.id}")
                    self.lost = self.wrong_branch
                return
        self._wb_run, self._wb_travel = 0.0, None

    def fork(self, end_travel, crossing=False):
        """An MLS fork episode ended at wheel travel end_travel. Returns (verdict, node):
        "resync" at a known node, "unexpected" with no node within 3 sigma, "" not started."""
        if not self.started:
            return "", None
        S_e = self.S - (self.travel - end_travel)
        tol = max(3.0 * self.sigma, 0.10)
        near = [n for n in self.route.nodes if n["fork"] and abs(n["S"] - S_e) <= tol]
        best = min(near, key=lambda n: abs(n["S"] - S_e)) if near else None
        if best is None:
            return "unexpected", None
        if self.lost:
            return "resync-skipped", best
        self._measure(best["S"] - S_e, best["sigma"])
        return "resync", best

    def uturn_done(self, node_id):
        """The pivot ended at a taught tape end: S is that node's S."""
        n = next((n for n in self.route.nodes if n["id"] == node_id and n["kind"] == "uturn"), None)
        if n is None or not self.started or self.lost:
            return False
        self.S, self.sigma = n["S"], max(n["sigma"], min(self.sigma, n["sigma"] * 3))
        return True

    def _measure(self, innov, r):
        """A resync as a measurement of S with its taught spread r."""
        k = self.sigma ** 2 / (self.sigma ** 2 + r ** 2)
        self.S += k * innov
        self.sigma = math.sqrt((1.0 - k) * self.sigma ** 2)
        self._accept_travel = self.travel

    # -- outputs --------------------------------------------------------------------
    @property
    def health(self):
        if not self.started:
            return ""
        if self.lost:
            return LOST
        since = self.travel - self._accept_travel
        if abs(since) <= self.p.healthy_window_m and self.sigma <= self.p.sigma_station_m:
            return HEALTHY
        return DEGRADED

    def ok_for(self, limit):
        """May a decision with this sigma limit be taken now?"""
        return self.started and not self.lost and self.sigma <= limit

    def where(self):
        """(leg id, s on it) for display."""
        _, leg, s = self.route.at(self.S)
        return leg.id, s

    def snapshot(self):
        leg, s = self.where() if self.started else ("", 0.0)
        return {"health": self.health, "S": self.S, "sigma": self.sigma, "leg": leg, "s": s,
                "lost": self.lost, "rejects": self.rejects, "accepted": self.accepted,
                "rejected": self.rejected, "last_reject": self.last_reject,
                "lateral": self.last_lateral, "innov": self.last_innov}


class Shadow:
    """P1 shadow mode: where the locator WOULD have stopped and ordered, logged, with the
    check each decision would have met. Motion is not touched."""

    def __init__(self, route, params, stop_distance_m, uturns=None):
        self.p = params
        self.events = []   # (S_trigger, kind, id, detail)
        for st in route.stops[1:]:
            self.events.append((st["S"] - stop_distance_m, "stop", st["id"], st["S"]))
        for n in route.nodes:
            if n["kind"] == "junction" and n["order"] in ("left", "right"):
                self.events.append((n["S_fork"] - params.junction_lead_m, "order", n["id"], n["order"]))
            elif n["kind"] == "uturn":
                lead = (uturns or {}).get(n["id"], {}).get("lead_m", 1.5)
                self.events.append((n["S"] - lead, "uturn", n["id"], n["S"]))
        self.events.sort(key=lambda e: e[0])
        self._next = 0

    @property
    def done(self):
        """Every decision of the lap has been passed (the last is the HOME stop)."""
        return self._next >= len(self.events)

    def tick(self, loc):
        """Events whose trigger S the locator has just passed -> [(level, text)]."""
        out = []
        while self._next < len(self.events) and loc.S >= self.events[self._next][0]:
            S_t, kind, ident, detail = self.events[self._next]
            self._next += 1
            limit = self.p.sigma_station_m if kind == "stop" else self.p.sigma_junction_m
            ok = loc.ok_for(limit)
            verdict = "ok" if ok else f"would HOLD ({loc.health}, sigma {loc.sigma:.3f} > {limit:g})" \
                if not loc.lost else f"would HOLD (LOST: {loc.lost})"
            what = {"stop": f"stop {ident}", "order": f"order {detail} at {ident}",
                    "uturn": f"U-turn approach {ident}"}[kind]
            out.append(("info" if ok else "warn",
                        f"shadow: {what} at S {S_t:.2f} m, sigma {loc.sigma:.3f} m: {verdict}"))
        return out


def route_for(circuit_id, mission, graph):
    """Compile a circuit of a parsed site mission against a graph (RouteError if untaught)."""
    return tg.compile_route(circuit_id, mission["CIRCUITS"][circuit_id], graph)
