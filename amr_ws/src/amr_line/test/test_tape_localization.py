"""Tracked plan 3: tape graph, teach builder, locator, trips. Pure, clock-fed, synthetic.

A small circuit stands in for the site (HOME bay, a merge, ATT, a right diverge, one MRU
stop, the tape-end pivot, the stop on the way back, a left diverge, HOME):

    J-C (0,0) --home leg--> J-A (10,0) --ATT--> J-B (10,10) --MW-1--> U (30,10)
    U --ME-1--> (0,10) --> J-C (0,0)          (the leg back retraces the MRU line)

and J-B has a second, untaken exit straight on to (10,20) for the wrong-branch test.
"""
import copy
import math
import os
import random
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_line")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from agv_core import mission as missions  # noqa: E402

from amr_line import tape_graph as tg  # noqa: E402
from amr_line import tape_locator as tl  # noqa: E402
from amr_line import tape_teach as tt  # noqa: E402
from amr_line import trip as tr  # noqa: E402

DOC = {
    "schema": "site-3", "mission_name": "t", "home": "HOME", "attach": "ATT", "trip_calls": 2,
    "stop_distance_m": 0.5,
    "junctions": {"J-A": {"shape": "Y", "laid": True}, "J-B": {"shape": "Y", "laid": True},
                  "J-C": {"shape": "Y", "laid": True}, "J-X": {"shape": "T", "laid": False}},
    "uturns": {"U": {"direction": "cw", "approach_mps": 0.1, "decel_m": 0.3, "max_approach_m": 2.0,
                     "lead_m": 1.5}},
    "circuits": {"C-M": [{"stop": "HOME"}, {"junction": "J-A", "order": "merge"}, {"stop": "ATT"},
                         {"junction": "J-B", "order": "right"}, {"stop": "MW-1"}, {"stop": "MW-2"},
                         {"uturn": "U"}, {"stop": "ME-2"}, {"stop": "ME-1"},
                         {"junction": "J-X", "order": "straight", "approach": "bar"},
                         {"junction": "J-C", "order": "left"}, {"stop": "HOME"}]},
    "groups": {"M": {"circuit": "C-M", "machines": {"M1": {"release": "MW-1", "take": "ME-1"},
                                                    "M2": {"release": "MW-2", "take": "ME-2"}}}},
}
BINDING = {"map_id": "t", "map_revision": 1, "map_sha256": "ab" * 32}
# True geometry of each leg, in travel order from the start of the home leg.
TRUE = {
    "J-C:left>J-A": [(0, 0), (10, 0)],
    "J-A:merge>J-B": [(10, 0), (10, 10)],
    "J-B:right>U": [(10, 10), (30, 10)],
    "U:pivot>J-C": [(30, 10), (0, 10), (0, 0)],
}
ORDER = ["J-C:left>J-A", "J-A:merge>J-B", "J-B:right>U", "U:pivot>J-C"]
STATIONS = {"HOME": ("J-C:left>J-A", 5.0), "ATT": ("J-A:merge>J-B", 3.0), "MW-1": ("J-B:right>U", 8.0),
            "MW-2": ("J-B:right>U", 14.0), "ME-2": ("U:pivot>J-C", 6.0), "ME-1": ("U:pivot>J-C", 12.0)}


def mission():
    return missions.parse_site(copy.deepcopy(DOC))


def tour():
    """The true path of one pass, HOME to HOME: [(travel, x, y)] every 2 cm, and the
    travel at each node (J-A, J-B, U, J-C) and at the pass end."""
    pts = tg.Polyline(TRUE[ORDER[0]]).cut(5.0, 10.0)
    for lid in ORDER[1:]:
        pts += TRUE[lid][1:]
    pts += tg.Polyline(TRUE[ORDER[0]]).cut(0.0, 5.0)[1:]
    line = tg.Polyline(tg.resample(pts, 0.02))
    nodes = [5.0, 15.0, 35.0, 75.0]
    return line, nodes


def record_pass(seed=0, lateral=0.01, drop_fork=None, scale=1.0):
    """A synthetic PassRecorder dict: AMCL poses with noise, wheel travel with a scale error,
    MLS fork episodes 0.4 m long ending at each junction node."""
    rnd = random.Random(seed)
    line, nodes = tour()
    rec = tt.PassRecorder("C-M", started=f"p{seed}")
    t = 0.0
    forks = {n: k for k, n in enumerate(nodes) if k != 2}
    s = 0.0
    while s <= line.length:
        travel = s * scale
        nlcp = 2
        for n, k in forks.items():
            if k != drop_fork and n - 0.4 <= s < n:
                nlcp = 6
        rec.tick(t, travel, nlcp)
        if abs(s - nodes[2]) < 0.011:
            rec.uturn(travel)
        if round(s / 0.02) % 5 == 0:
            x, y, h = line.point(s)
            n = rnd.gauss(0, lateral)
            rec.pose(t, x - n * math.sin(h), y + n * math.cos(h))
        s += 0.02
        t += 0.04
    return rec.to_dict()


def taught_graph(passes=3):
    m = mission()
    g, report = tt.build([record_pass(seed=i) for i in range(passes)], m, BINDING)
    for sid, (lid, s) in STATIONS.items():
        x, y, h = tg.Polyline(TRUE[lid]).point(s)
        g.put_station(tt.project_station(g, m, sid, (x, y, h)))
    return m, g, report


# -- mission ----------------------------------------------------------------------------

def test_site_mission_legs_and_stations():
    m = mission()
    legs = m["CIRCUITS"]["C-M"]["legs"]
    assert [leg["id"] for leg in legs] == ["J-A:merge>J-B", "J-B:right>U", "U:pivot>J-C", "J-C:left>J-A"]
    assert legs[1]["stops"] == ["MW-1", "MW-2"]
    assert m["STATIONS"]["HOME"] == "J-C:left>J-A" and m["MACHINES"] == {"M1": "M", "M2": "M"}


@pytest.mark.parametrize("mutate, needle", [
    (lambda d: d["circuits"]["C-M"][3].pop("order"), "without an order"),
    (lambda d: d["circuits"]["C-M"][9].update(approach="stem"), "no straight on"),
    (lambda d: d["groups"].update(N={"circuit": "C-M", "machines": {"M1": {"swap": "MW-1"}}}),
     "already in group"),
    (lambda d: d["groups"]["M"]["machines"]["M1"].update(release="ME-1", take="MW-1"), "never pivots"),
    (lambda d: d["groups"]["M"]["machines"].update(M3={"swap": "R9"}), "not a machine stop"),
    (lambda d: d["circuits"]["C-M"].pop(0), "start and end"),
    (lambda d: d["circuits"]["C-M"].pop(2), "passes ATT 0 times"),
    (lambda d: d.update(trip_calls=3), "trip_calls"),
])
def test_site_mission_refusals(mutate, needle):
    d = copy.deepcopy(DOC)
    mutate(d)
    with pytest.raises(missions.MissionError, match=needle):
        missions.parse_site(d)


def test_the_repo_site_mission_loads():
    m = missions.load_site("site-103")
    assert set(m["GROUPS"]) == {"MRU", "BTU"} and len(m["MACHINES"]) == 20
    mru = [leg["id"] for leg in m["CIRCUITS"]["C-MRU"]["legs"]]
    assert "J-ML:merge>U-MRU" in mru and "J-ML:left>J-STEM" in mru   # J-R2E not laid yet
    assert m["STATIONS"]["ATT"] == "J-BMERGE:merge>J-N"
    with pytest.raises(missions.MissionError, match="site mission"):
        missions.load("site-103")
    assert "site-103" not in missions.list_missions()


# -- graph and teach -----------------------------------------------------------------------

def test_polyline_projection_and_resample():
    line = tg.Polyline(tg.resample([(0, 0), (1, 0), (1, 1)], 0.05))
    assert line.length == pytest.approx(2.0)
    s, d = line.project(0.5, 0.2)
    assert s == pytest.approx(0.5) and d == pytest.approx(0.2)        # left of travel is +
    s, d = line.project(1.2, 0.5)
    assert s == pytest.approx(1.5) and d == pytest.approx(-0.2)
    assert line.project(1.2, 0.5, 0.0, 0.8)[0] == pytest.approx(0.8)  # the window clamps


def test_build_averages_passes_into_legs():
    _m, g, report = taught_graph()
    assert sorted(g.legs) == sorted(ORDER), report
    for lid in ORDER:
        leg = g.legs[lid]
        true = tg.Polyline(TRUE[lid]).length
        assert leg.length == pytest.approx(true, abs=0.05), (lid, report)
        assert leg.passes == 3 and leg.spread_m < 0.05
    assert g.legs["J-B:right>U"].end_kind == "uturn"
    assert g.legs["J-A:merge>J-B"].fork_back_m == pytest.approx(0.4, abs=0.03)
    assert g.stations["MW-1"].s == pytest.approx(8.0, abs=0.05)


def test_a_pass_with_a_missed_fork_is_refused():
    m = mission()
    with pytest.raises(tt.TeachError, match="do not match"):
        tt.split_pass(record_pass(drop_fork=1), m)
    _g, report = tt.build([record_pass(0), record_pass(1, drop_fork=1)], m, BINDING)
    assert any(r.startswith("REFUSED") for r in report)


def test_graph_round_trip_and_binding():
    _m, g, _r = taught_graph(1)
    g.revision = 3
    back = tg.Graph.from_dict(copy.deepcopy(g.to_dict()))
    assert back.stations["ME-1"].s == pytest.approx(g.stations["ME-1"].s, abs=1e-3)
    assert back.check_binding("t", 1, "ab" * 32) == ""
    assert "rev2" in back.check_binding("t", 2, "ab" * 32)
    assert "hash changed" in back.check_binding("t", 1, "cd" * 32)


def test_a_station_off_its_leg_or_facing_the_wrong_way_is_refused():
    m, g, _r = taught_graph(1)
    with pytest.raises(tg.GraphError, match="from its leg"):
        g.put_station(tt.project_station(g, m, "ATT", (10.3, 3.0, math.pi / 2)))
    with pytest.raises(tg.GraphError, match="wrong direction"):
        tt.project_station(g, m, "ATT", (10.0, 3.0, -math.pi / 2))


def test_route_compiles_home_to_home():
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    ids = [s["id"] for s in route.stops]
    assert ids == ["HOME", "ATT", "MW-1", "MW-2", "ME-2", "ME-1", "HOME"]
    S = {s["id"]: s["S"] for s in route.stops[:-1]}
    assert S["HOME"] == pytest.approx(5.0, abs=0.05) and S["MW-1"] == pytest.approx(28.0, abs=0.1)
    assert route.S_end == pytest.approx(85.0, abs=0.2)
    assert [n["id"] for n in route.nodes] == ["J-A", "J-B", "U", "J-C"]
    jb = route.nodes[1]
    assert jb["order"] == "right" and jb["S_fork"] == pytest.approx(jb["S"] - 0.4, abs=0.03)
    # The MRU line is one floor tape at two S values: the window picks the predicted one.
    x, y, _ = route.point(S["MW-1"])
    assert route.project(x, y, S["MW-1"] - 1, S["MW-1"] + 1)[0] == pytest.approx(S["MW-1"], abs=0.02)


def test_an_untaught_circuit_names_what_is_missing():
    m = mission()
    g, _ = tt.build([], m, BINDING)
    with pytest.raises(tg.RouteError, match="J-B:right>U"):
        tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)


# -- locator -------------------------------------------------------------------------------

def drive(loc, route, S_from, S_to, *, t=0.0, scale=1.01, latency=0.1, noise=0.02, seed=1,
          pose_every=0.1, tick=0.02, v=0.5, fork_nodes=True, pose_fn=None, ready=True):
    """Move along the true route; wheel travel is S x scale. Returns (t, errors at each pose)."""
    rnd = random.Random(seed)
    det = tg.ForkDetector()
    fork_S = [(n["S_fork"], n["S"]) for n in route.nodes if n["fork"]]
    pending, errs, S = [], [], S_from
    next_pose = t
    loc.readiness(ready)
    while S < S_to:
        S = min(S + v * tick, S_to)
        t += tick
        travel = S * scale
        loc.predict(t, travel)
        if fork_nodes:
            nlcp = 6 if any(a <= S < b for a, b in fork_S) else 2
            ep = det.update(travel, nlcp)
            if ep:
                loc.fork(ep[1])
        if t >= next_pose:
            x, y, _ = pose_fn(S) if pose_fn else route.point(S)
            pending.append((t + latency, t, x + rnd.gauss(0, noise), y + rnd.gauss(0, noise)))
            next_pose += pose_every
        while pending and pending[0][0] <= t:
            _arr, stamp, x, y = pending.pop(0)
            loc.correct(t, stamp, x, y, 0.0)
            errs.append(loc.S - S)
    return t, errs


def started(route, graph=None, params=None):
    loc = tl.TapeLocator(route, params, graph)
    loc.start(route.S_start, 0.02, 0.0, route.S_start * 1.01)
    return loc


def test_locator_follows_a_lap_within_the_station_budget():
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g)
    _t, errs = drive(loc, route, route.S_start, route.S_end)
    tail = errs[20:]
    assert max(abs(e) for e in tail) < 0.05
    assert loc.health == tl.HEALTHY and loc.rejects == 0
    assert loc.accepted > 0.9 * len(errs)


def test_without_latency_compensation_the_error_would_be_the_lag():
    """The odometry buffer is what removes the pose lag: a 0.3 s old pose taken as 'now'
    would sit 0.15 m behind at 0.5 m/s - beyond the station budget."""
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g, tl.Params(pose_max_age_s=0.5))
    _t, errs = drive(loc, route, route.S_start, 30.0, latency=0.3, noise=0.005)
    assert max(abs(e) for e in errs[20:]) < 0.03


def test_a_jump_is_rejected_and_repeated_rejections_mean_lost():
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g)
    t, _ = drive(loc, route, route.S_start, 8.0)
    S0 = loc.S
    jumped = lambda S: route.point(S + 0.8)  # noqa: E731 - AMCL 0.8 m ahead along the tape
    t, _ = drive(loc, route, 8.0, 8.12, t=t, pose_fn=jumped, noise=0.0)
    assert "innovation" in loc.last_reject and abs(loc.S - 8.12) < 0.05 and loc.S > S0
    t, _ = drive(loc, route, 8.12, 9.5, t=t, pose_fn=jumped, noise=0.0)
    assert loc.health == tl.LOST and "rejected in a row" in loc.lost
    assert not loc.ok_for(1.0)
    # Still READY, AMCL still consistent with itself: not trusted until revalidated.
    drive(loc, route, 9.5, 10.5, t=t, pose_fn=jumped, noise=0.0)
    assert loc.health == tl.LOST and "READY again" in loc.last_reject


def test_lost_on_readiness_and_reacquired_when_ready_again():
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g)
    t, _ = drive(loc, route, route.S_start, 8.0)
    loc.readiness(False, "LOST")
    assert loc.health == tl.LOST
    t, _ = drive(loc, route, 8.0, 9.0, t=t, ready=False)
    assert loc.health == tl.LOST
    t, errs = drive(loc, route, 9.0, 11.0, t=t)
    assert loc.health != tl.LOST and abs(errs[-1]) < 0.05


def test_lateral_gate_rejects_a_pose_off_the_path():
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g)
    t, _ = drive(loc, route, route.S_start, 7.0)
    off = lambda S: (route.point(S)[0], route.point(S)[1] + 0.3, 0)  # noqa: E731 - 0.3 m beside
    drive(loc, route, 7.0, 7.3, t=t, pose_fn=off, noise=0.0)
    assert "from the taught path" in loc.last_reject


def test_odometry_alone_degrades_and_a_fork_resyncs():
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g)
    jb = route.nodes[1]
    # No AMCL at all (pose_every huge): odometry with a 2 % scale error carries S.
    t, _ = drive(loc, route, route.S_start, jb["S"] - 1.0, scale=1.0, pose_every=1e9)
    assert loc.health == tl.DEGRADED
    loc.S += 0.12                    # what a scale error would have accumulated
    sig = loc.sigma
    drive(loc, route, jb["S"] - 1.0, jb["S"] + 0.5, t=t, scale=1.0, pose_every=1e9)
    assert abs(loc.S - (jb["S"] + 0.5)) < 0.03 and loc.sigma < sig


def test_a_fork_where_no_junction_is_taught_is_unexpected():
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g)
    drive(loc, route, route.S_start, 20.0, fork_nodes=False)
    assert loc.fork(loc.travel - 0.1)[0] == "unexpected"


def test_wrong_branch_is_detected_within_two_metres():
    m, g, _r = taught_graph()
    straight = tg.Leg("J-B:straight>J-Z", "J-B", "straight", "J-Z", tg.resample([(10, 10), (10, 20)]))
    g.legs[straight.id] = straight
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g)
    jb = route.nodes[1]
    t, _ = drive(loc, route, route.S_start, jb["S"])
    # The vehicle carries straight on while the route says right.
    wrong = lambda S: (10.0, 10.0 + (S - jb["S"]), math.pi / 2)  # noqa: E731
    drive(loc, route, jb["S"], jb["S"] + 2.0, t=t, pose_fn=wrong, noise=0.005)
    assert loc.wrong_branch and "J-B" in loc.wrong_branch and loc.health == tl.LOST


def test_shadow_logs_each_decision_once():
    m, g, _r = taught_graph()
    route = tg.compile_route("C-M", m["CIRCUITS"]["C-M"], g)
    loc = started(route, g)
    sh = tl.Shadow(route, tl.Params(), m["STOP_DISTANCE_M"], m["UTURNS"])
    logged = []
    t = 0.0
    S = route.S_start
    while S < route.S_end:
        t, _ = drive(loc, route, S, min(S + 1.0, route.S_end), t=t)
        S = min(S + 1.0, route.S_end)
        logged += sh.tick(loc)
    text = " | ".join(x for _lvl, x in logged)
    for what in ("stop ATT", "stop MW-1", "U-turn approach U", "order right at J-B", "order left at J-C",
                 "stop HOME"):
        assert text.count(what) == 1, text


# -- trips ---------------------------------------------------------------------------------

def test_a_trip_forms_at_two_calls_of_one_group_in_circuit_order():
    m = missions.load_site("site-103")
    q = tr.CallQueue(m)
    assert q.call("MRU3", 1)[0] and q.call("BTU10", 2)[0]
    assert q.call("MRU3", 3) == (False, "MRU3 is already waiting")
    assert not q.call("STU1", 3)[0]
    assert q.ready_group() is None
    assert q.call("MRU1", 4)[0]
    trip = q.take()
    assert trip.group == "MRU" and [p for p, _ in q.pending] == ["BTU10"]
    assert trip.stops() == ["ATT", "MW-1", "MW-3", "ME-3", "ME-1", "HOME-N"]
    assert trip.work("MW-1") == [("MRU1", "release")]


def test_facing_machines_share_one_stop():
    m = missions.load_site("site-103")
    q = tr.CallQueue(m)
    q.call("BTU16", 1)
    q.call("BTU24", 2)
    trip = q.take()
    assert trip.stops() == ["ATT", "R2-1", "HOME-N"]
    assert sorted(trip.work("R2-1")) == [("BTU16", "swap"), ("BTU24", "swap")]


def test_an_untaught_group_does_not_form_a_trip():
    m = missions.load_site("site-103")
    q = tr.CallQueue(m)
    q.call("BTU16", 1)
    q.call("BTU24", 2)
    assert q.take(available={"MRU"}) is None and len(q.pending) == 2


def test_payload_guard_at_the_mru_pivot_and_home():
    m = missions.load_site("site-103")
    trip = tr.Trip(m, "MRU", ["MRU2", "MRU4"])
    ok, why = trip.may_pivot()
    assert not ok and "MRU2" in why and "MRU4" in why
    trip.serve("MW-2")
    assert not trip.may_pivot()[0]
    trip.confirm_released()
    assert trip.may_pivot() == (True, "")
    trip.serve("ME-4")
    assert trip.missed() == ["MRU2 (take at ME-2)"] and "MISSED" in trip.result()
    p = tr.Payload()
    assert not p.may_leave()[0]
    p.confirm_off()
    assert p.may_leave() == (True, "")
    p.trip_ended()
    assert not p.may_leave()[0]


# -- store ---------------------------------------------------------------------------------

def test_store_publishes_immutable_revisions_and_the_pointer(tmp_path):
    from amr_line import tape_graph_store as store
    _m, g, _r = taught_graph(1)
    maps, state = str(tmp_path / "maps"), str(tmp_path / "state")
    assert store.publish(maps, g) == 1 and store.publish(maps, g) == 2
    assert store.list_revisions(maps, "t") == [1, 2]
    back, sha = store.load(maps, "t", 2)
    assert back.revision == 2 and sorted(back.stations) == sorted(STATIONS) and len(sha) == 64
    with open(tmp_path / "maps" / "t" / "tapegraph" / "g1" / "graph.json", "a") as fh:
        fh.write(" ")
    with pytest.raises(store.StoreError, match="manifest hash"):
        store.load(maps, "t", 1)
    assert store.read_active(state) is None
    store.write_active(state, "t", 1)
    assert store.read_active(state)["graph_revision"] is None
    store.write_active(state, "t", 1, 2, sha)
    assert store.read_active(state)["graph_revision"] == 2


def test_store_teach_workspace(tmp_path):
    from amr_line import tape_graph_store as store
    state = str(tmp_path)
    name = store.save_pass(state, "t", 1, record_pass(0))
    assert store.list_passes(state, "t", 1) == [name]
    m = mission()
    g, _ = tt.build(store.load_passes(state, "t", 1), m, BINDING)
    assert "J-B:right>U" in g.legs
    store.save_station(state, "t", 1, {"id": "ATT", "pose": [10.0, 3.0, math.pi / 2], "taught": "x",
                                       "samples": 5})
    g, report = tt.build(store.load_passes(state, "t", 1), m, BINDING,
                         stations=store.load_stations(state, "t", 1).values())
    assert g.stations["ATT"].s == pytest.approx(3.0, abs=0.05), report
    assert store.delete_pass(state, "t", 1, name) and store.list_passes(state, "t", 1) == []
    with pytest.raises(store.StoreError):
        store.teach_dir(state, "../x", 1)
