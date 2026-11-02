"""amr_line.tape_runtime: teach and shadow inside the LINE layer (tracked plan 3, P1).

Drives the synthetic circuit of test_tape_localization through the runtime the node
uses: select a map, record passes, build, teach stations, publish, activate, then
At home and a shadow lap. Nothing here produces a command.
"""
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(__file__)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from test_tape_localization import BINDING, STATIONS, TRUE, mission, tour  # noqa: E402

from amr_line import tape_graph as tg  # noqa: E402
from amr_line import tape_graph_store as store  # noqa: E402
from amr_line import tape_locator as tl  # noqa: E402
from amr_line import tape_runtime as tr  # noqa: E402

SHA = BINDING["map_sha256"]


def runtime(tmp_path, selection):
    return tr.TapeRuntime(mission(), tl.Params(), maps_dir=str(tmp_path / "maps"),
                          state_dir=str(tmp_path / "state"), selection=selection,
                          resolve_sha=lambda _m, _r: SHA, shadow_circuit="C-M")


def drive_lap(rt, t0=0.0, seed=0, start_travel=0.0, noise=0.01):
    """One lap of the synthetic tour through the runtime: ticks at 50 Hz, a pose every 0.1 s
    with 0.1 s latency, MLS fork episodes before each junction, the U-turn phases."""
    rnd = random.Random(seed)
    line, nodes = tour()
    t, s, pending, next_pose = t0, 0.0, [], t0
    while s < line.length:
        s = min(s + 0.01, line.length)
        t += 0.02
        nlcp = 6 if any(n - 0.4 <= s < n for k, n in enumerate(nodes) if k != 2) else 2
        phase = "approach" if nodes[2] - 1.5 <= s < nodes[2] else ""
        if abs(s - nodes[2]) < 0.006:
            for p in ("stopping", "spin", ""):
                rt.tick(t, start_travel + s, 2, True, p)
        rt.tick(t, start_travel + s, nlcp, False, phase)
        if t >= next_pose:
            x, y, _h = line.point(s)
            pending.append((t + 0.1, t, x + rnd.gauss(0, noise), y + rnd.gauss(0, noise)))
            next_pose += 0.1
        while pending and pending[0][0] <= t:
            _a, stamp, x, y = pending.pop(0)
            rt.amcl(t, stamp, x, y, 0.0, 0.0004, 0.0, 0.0004)
        if round(t * 50) % 25 == 0:
            rt.readiness(t, tr.READY)
    return t, start_travel + s


def stand(rt, t, travel, x, y, yaw, seconds=3.5):
    for _ in range(int(seconds / 0.1)):
        t += 0.1
        rt.tick(t, travel, 2, True)
        rt.amcl(t, t, x, y, yaw, 0.0004, 0.0, 0.0004)
        rt.readiness(t, tr.READY)
    return t


def test_teach_then_shadow(tmp_path):
    sel = {"map_id": "t", "map_revision": 1, "graph_revision": None}
    rt = runtime(tmp_path, sel)
    assert "no graph active" in rt.detail
    ok, msg = rt.teach(0.0, "pass_start", "C-M")
    assert not ok and "READY" in msg
    rt.readiness(0.0, tr.READY)
    t, travel = 0.0, 0.0
    for i in range(3):
        assert rt.teach(t, "pass_start", "C-M")[0]
        t, travel = drive_lap(rt, t, seed=i, start_travel=travel)
        ok, msg = rt.teach(t, "pass_stop")
        assert ok, msg
    ok, msg = rt.teach(t, "build")
    assert ok and "station HOME" in msg          # legs built, stations still missing
    for sid, (lid, s) in STATIONS.items():
        x, y, h = tg.Polyline(TRUE[lid]).point(s)
        t = stand(rt, t, travel, x, y, h)
        ok, msg = rt.teach(t, "station", sid)
        assert ok, msg
    ok, msg = rt.teach(t, "build")
    assert ok and "every circuit compiles" in msg, msg
    ok, msg = rt.teach(t, "publish")
    assert ok and "g1" in msg
    ok, msg = rt.teach(t, "activate", "1")
    assert ok and store.read_active(str(tmp_path / "state"))["graph_revision"] == 1

    # The next layer start: the active graph, At home, a shadow lap.
    rt = runtime(tmp_path, store.read_active(str(tmp_path / "state")))
    assert rt.route is not None and "At home" in rt.detail
    ok, _msg, pose = rt.at_home()
    assert ok and pose[0] == pytest.approx(5.0, abs=0.05)
    assert rt.snapshot()["loc_health"] == "WAITING"
    rt.readiness(t, tr.READY)
    t, travel = drive_lap(rt, t, seed=7, start_travel=travel)
    t = stand(rt, t, travel, 5.0, 0.0, 0.0, seconds=0.5)   # parked at HOME: the lap completes
    snap = rt.snapshot()
    assert snap["loc_health"] in ("HEALTHY", "DEGRADED") and snap["loc_graph"] == "t rev1 g1"
    text = " | ".join(x for _l, x in rt.drain_events())
    for what in ("locator started", "stop ATT", "stop MW-1", "order right at J-B", "U-turn U done",
                 "lap complete"):
        assert what in text, text
    assert "would HOLD" not in text, text


def test_a_graph_for_another_map_revision_is_refused(tmp_path):
    rt = runtime(tmp_path, {"map_id": "t", "map_revision": 1, "graph_revision": None})
    rt.readiness(0.0, tr.READY)
    rt.teach(0.0, "pass_start", "C-M")
    t, _ = drive_lap(rt)
    rt.teach(t, "pass_stop")
    rt.teach(t, "build")
    rt.teach(t, "publish")
    rt2 = tr.TapeRuntime(mission(), tl.Params(), maps_dir=str(tmp_path / "maps"),
                         state_dir=str(tmp_path / "state"),
                         selection={"map_id": "t", "map_revision": 1, "graph_revision": 1},
                         resolve_sha=lambda _m, _r: "cd" * 32, shadow_circuit="C-M")
    assert rt2.route is None and "hash changed" in rt2.detail
    assert not rt2.at_home()[0]


def test_teach_refusals(tmp_path):
    rt = runtime(tmp_path, None)
    assert "no map selected" in rt.teach(0.0, "pass_start", "C-M")[1]
    assert not rt.teach(0.0, "fly")[0]
    rt = runtime(tmp_path, {"map_id": "t", "map_revision": 1, "graph_revision": None})
    rt.readiness(0.0, tr.READY)
    assert "circuit" in rt.teach(0.0, "pass_start", "C-X")[1]
    ok, msg = rt.teach(0.0, "station", "ATT")
    assert not ok and "still" in msg
    t = stand(rt, 0.0, 0.0, 10.0, 3.0, math.pi / 2)
    assert "no legs yet" in rt.teach(t, "station", "ATT")[1]
    assert rt.teach(t, "map", "t 2")[0] and store.read_active(str(tmp_path / "state"))["map_revision"] == 2


# -- the straight-section test (plan 3 section 7) ------------------------------------------
# 10 m of tape, x = 0 (west tape end, U-MRU pivot) to x = 10. T-HOME at x = 9.5 facing west;
# there the operator turns the AGV by hand (MANUAL spin, no tape end in the east).

HOME_X = 9.5
# T-ATT at least 2 m from T-HOME: its braking point must lie beyond the start, where AMCL
# has corrected the At-home sigma (runbook section 2S).
# Only MRU4 lies inside the section (operator, 2026-10-18).
WEST = {"T-HOME": 9.5, "T-ATT": 7.0, "MW-4": 1.8}
EAST = {"ME-4": 1.8}


def straight_runtime(tmp_path, selection):
    from agv_core import mission as missions
    return tr.TapeRuntime(missions.load_site("test-mru-straight"), tl.Params(),
                          maps_dir=str(tmp_path / "maps"), state_dir=str(tmp_path / "state"),
                          selection=selection, resolve_sha=lambda _m, _r: SHA, shadow_circuit="C-STRAIGHT")


def straight_lap(rt, t, travel, seed=0, manual_pivot=False):
    """HOME westbound to the tape end, the pivot, eastbound back to HOME; optionally the
    operator's MANUAL spin at HOME first (AMCL keeps updating, the wheels add ~0 travel)."""
    rnd = random.Random(seed)
    if manual_pivot:
        for k in range(40):
            t += 0.1
            rt.tick(t, travel, 2, False)
            rt.amcl(t, t - 0.05, HOME_X + rnd.gauss(0, 0.01), rnd.gauss(0, 0.01), math.pi * k / 40,
                    4e-4, 0, 4e-4)
            rt.readiness(t, tr.READY)
    pending, next_pose = [], t

    def step(x, phase=""):
        nonlocal t, travel, next_pose
        t += 0.02
        travel += 0.01
        rt.tick(t, travel, 2, False, phase)
        if t >= next_pose:
            pending.append((t + 0.1, t, x + rnd.gauss(0, 0.01), rnd.gauss(0, 0.01)))
            next_pose += 0.1
        while pending and pending[0][0] <= t:
            _a, stamp, px, py = pending.pop(0)
            rt.amcl(t, stamp, px, py, 0.0, 4e-4, 0.0, 4e-4)
        if round(t * 50) % 25 == 0:
            rt.readiness(t, tr.READY)

    x = HOME_X
    while x > 0.0:
        x = max(0.0, x - 0.01)
        step(x, "approach" if x < 1.5 else "")
    for p in ("stopping", "spin", ""):
        rt.tick(t, travel, 2, True, p)
    while x < HOME_X:
        x = min(HOME_X, x + 0.01)
        step(x)
    for _ in range(10):                       # standing at HOME
        t += 0.1
        rt.tick(t, travel, 2, True)
        rt.readiness(t, tr.READY)
    return t, travel


def test_the_straight_section_teach_and_shadow(tmp_path):
    rt = straight_runtime(tmp_path, {"map_id": "t", "map_revision": 1, "graph_revision": None})
    rt.readiness(0.0, tr.READY)
    t, travel = 0.0, 0.0
    for i in range(3):
        assert rt.teach(t, "pass_start", "C-STRAIGHT")[0]
        t, travel = straight_lap(rt, t, travel, seed=i, manual_pivot=i > 0)
        ok, msg = rt.teach(t, "pass_stop")
        assert ok, msg
    assert rt.teach(t, "build")[0]
    for stations, yaw in ((WEST, math.pi), (EAST, 0.0)):
        for sid, x in stations.items():
            t = stand(rt, t, travel, x, 0.0, yaw)
            ok, msg = rt.teach(t, "station", sid)
            assert ok, (sid, msg)
    ok, msg = rt.teach(t, "build")
    assert ok and "every circuit compiles" in msg, msg
    leg = rt.staged.legs["U-MRU:pivot>U-MRU"]
    assert leg.length == pytest.approx(2 * HOME_X, abs=0.1) and leg.end_kind == "uturn"
    assert rt.teach(t, "publish")[0] and rt.teach(t, "activate", "1")[0]

    rt = straight_runtime(tmp_path, store.read_active(str(tmp_path / "state")))
    ok, _m, pose = rt.at_home()
    assert ok and pose[0] == pytest.approx(HOME_X, abs=0.05) and abs(abs(pose[2]) - math.pi) < 0.1
    rt.readiness(t, tr.READY)
    for lap in range(2):
        t, travel = straight_lap(rt, t, travel, seed=10 + lap, manual_pivot=lap > 0)
    text = " | ".join(x for _l, x in rt.drain_events())
    assert text.count("lap complete") == 2, text
    for what in ("stop T-ATT", "stop MW-4", "U-turn approach U-MRU", "U-turn U-MRU done",
                 "stop ME-4", "stop T-HOME"):
        assert text.count(what) == 2, (what, text)
    assert "would HOLD" not in text and "LOST" not in text, text
