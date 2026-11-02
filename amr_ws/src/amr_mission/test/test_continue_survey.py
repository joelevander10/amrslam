"""Continue a saved survey (2026-10-23): the session coordinator matches the live scan on the
saved revision BEFORE slam_toolbox loads its pose graph, loads it with START_AT_GIVEN_POSE,
waits until SLAM agrees, keeps the original start mark and records the lineage at save."""

import math
import os
import threading
import types

import numpy as np
import pytest
from amr_maps.generate_sim_factory import build
from amr_mission.mapping_session_node import MappingSession
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph

from amr_interfaces.msg import MappingState
from amr_interfaces.srv import SaveMap, StartSurvey
from amr_maps import grid as gridio
from amr_maps import raycast
from amr_maps import relocalize as rl
from amr_mission import map_bundle as mb

TRUE = (6.0, 4.0, 3.0)  # an unambiguous spot in the sim factory
LASER = (0.35, 0.0, 0.0)
MARK = {"x_m": 0.5, "y_m": -1.0, "yaw_rad": 0.1, "description": "HP AGV 01"}


class Log:
    def info(self, *_):
        pass

    warn = error = info


def bundle(maps_dir: str, map_id: str = "hall", posegraph: bool = True) -> mb.Manifest:
    g = build()
    stage = mb.staging_dir(maps_dir, map_id, 1)
    m = mb.Manifest(map_id, 1, mb.now_iso(), "map", g.meta.resolution,
                    [g.meta.origin_x, g.meta.origin_y, 0.0], g.width, g.height, dict(MARK),
                    {"dx_m": 0.4, "dy_m": 2.9, "dyaw_rad": 0.1, "note": "ok"})
    stem = None
    if posegraph:
        stem = os.path.join(maps_dir, "pg")
        for ext in (".posegraph", ".data"):
            with open(stem + ext, "wb") as fh:
                fh.write(b"graph")
    mb.stage_bundle(stage, g, stem, m, required_posegraph=posegraph)
    mb.publish(stage, maps_dir, map_id, 1)
    return mb.load_manifest(mb.revision_dir(maps_dir, map_id, 1), map_id, 1)


def scan_at(pose) -> LaserScan:
    g, geom = build(), raycast.mounted()
    x, y, yaw = pose
    ranges = raycast.cast(g, x + LASER[0] * math.cos(yaw), y + LASER[0] * math.sin(yaw), yaw, geom)
    s = LaserScan()
    s.header.frame_id = "laser"
    s.angle_min, s.angle_max, s.angle_increment = geom.angle_min, geom.angle_max, geom.angle_increment
    s.range_min, s.range_max = geom.range_min, geom.range_max
    s.ranges = [float(r) if math.isfinite(r) else float("inf") for r in ranges]
    return s


def session(maps_dir: str, settled: str = "", ready=()):
    sv = types.SimpleNamespace(
        maps_dir=maps_dir, _state=MappingState.IDLE, _lock=threading.Lock(), match_params=rl.MatchParams(),
        load_timeout=120.0, _last_scan_msg=scan_at(TRUE), _deserialize="deserialize", calls=[], events=[],
        _map_id="", _description="", _start=None, _review=None, _saved_path="", _revision=0, _continued=None,
        _message="",
    )
    sv.get_logger = Log
    sv._readiness = lambda: list(ready)
    sv._laser_pose = lambda frame: LASER
    sv._now = lambda: 100.0
    sv._pose_in_map = lambda: TRUE
    sv._wait_continued = lambda target, t_load, area: settled
    sv.event = lambda code, text: sv.events.append(code)

    def _call(client, req, timeout=None):
        sv.calls.append((client, req, timeout))

    def _set(state, message):
        sv._state, sv._message = state, message

    sv._call, sv._set = _call, _set
    sv._refuse = lambda res, msg: MappingSession._refuse(sv, res, msg)
    return sv


def start(sv, rev=1, hint=(6.7, 3.6, 3.0 + 0.25), map_id="hall"):
    req = StartSurvey.Request(map_id=map_id, description="by rack C", continue_revision=rev)
    req.hint.x, req.hint.y, req.hint.theta = hint
    return MappingSession._start_continue(sv, req, StartSurvey.Response())


def test_continue_matches_the_scan_then_loads_the_graph_at_the_matched_pose(tmp_path):
    manifest = bundle(str(tmp_path))
    sv = session(str(tmp_path))
    res = start(sv)
    assert res.accepted, res.message
    [(client, load, timeout)] = sv.calls
    assert client == "deserialize" and timeout == 120.0
    assert load.match_type == DeserializePoseGraph.Request.START_AT_GIVEN_POSE  # never LOCALIZE_AT_POSE
    assert load.filename == os.path.join(mb.revision_dir(str(tmp_path), "hall", 1), "posegraph")
    p = load.initial_pose
    assert math.hypot(p.x - TRUE[0], p.y - TRUE[1]) <= 0.1  # the match, not the 0.8 m-off hint
    assert abs(rl.wrap(p.theta - TRUE[2])) <= math.radians(1.0)
    assert sv._state == MappingState.MAPPING and "continuing 'hall' rev1" in sv._message
    # the reference stays the ORIGINAL floor mark: Returned to start closes the whole map
    assert (sv._start.x_m, sv._start.y_m, sv._start.description) == (0.5, -1.0, "HP AGV 01")
    c = sv._continued
    assert c["revision"] == 1 and c["sha256"] == manifest.sha256 and c["at_description"] == "by rack C"
    assert c["match"]["score"] > 0.8 and sv.events == ["MODE_CHANGE"]


def test_a_revision_without_a_pose_graph_is_refused_before_anything_loads(tmp_path):
    bundle(str(tmp_path), posegraph=False)
    sv = session(str(tmp_path))
    res = start(sv)
    assert not res.accepted and "no saved pose graph" in res.message
    assert sv.calls == [] and sv._state == MappingState.IDLE


def test_a_poor_or_ambiguous_match_is_refused_before_anything_loads(tmp_path):
    bundle(str(tmp_path))
    sv = session(str(tmp_path))
    sv._last_scan_msg = scan_at((14.0, 0.0, -1.2))  # beside repeated racks: a second peak 1 m away
    res = start(sv, hint=(14.4, -0.3, -1.1))
    assert not res.accepted and res.message.startswith("continue refused: ambiguous")
    sv._last_scan_msg.ranges = [float("inf")] * len(sv._last_scan_msg.ranges)  # sees nothing
    res = start(sv)
    assert not res.accepted and "scan returns" in res.message
    assert sv.calls == [] and sv._state == MappingState.IDLE


def test_not_ready_and_missing_revision(tmp_path):
    bundle(str(tmp_path))
    res = start(session(str(tmp_path), ready=["vehicle is moving"]))
    assert not res.accepted and res.message.startswith("not ready")  # the supervisor retries these
    res = start(session(str(tmp_path)), rev=7)
    assert not res.accepted and "cannot continue hall rev7" in res.message


def test_slam_disagreeing_after_the_load_fails_and_says_start_again(tmp_path):
    bundle(str(tmp_path))
    sv = session(str(tmp_path), settled="SLAM put the vehicle 1.20 m / 3.0 deg from the scan match")
    res = start(sv)
    assert not res.accepted and len(sv.calls) == 1
    assert "hall rev1 loaded, but SLAM put the vehicle 1.20 m" in res.message
    assert res.message.endswith("abort and start again") and sv._state == MappingState.IDLE


def _waiter(stamp, pose, area_cells, map_t=101.0, settle=0.3):
    w = types.SimpleNamespace(settle_s=settle, pose_tol_m=0.3, pose_tol_rad=math.radians(5.0))
    w._map_odom_stamp = lambda: stamp
    w._pose_in_map = lambda: pose
    w._map_t = map_t
    w._map = types.SimpleNamespace(info=types.SimpleNamespace(width=area_cells, height=1, resolution=1.0))
    return lambda: MappingSession._wait_continued(w, TRUE, 100.0, 1000.0)


def test_wait_continued_needs_a_fresh_scan_the_full_map_and_the_matched_pose():
    assert _waiter(101.0, TRUE, 1000)() == ""
    assert "took no scan" in _waiter(99.0, TRUE, 1000)()
    assert "published no map" in _waiter(101.0, TRUE, 1000, map_t=99.0)()
    assert "did not load" in _waiter(101.0, TRUE, 500)()
    assert "0.50 m" in _waiter(101.0, (TRUE[0] + 0.5, TRUE[1], TRUE[2]), 1000)()
    assert "6.0 deg" in _waiter(101.0, (TRUE[0], TRUE[1], TRUE[2] + math.radians(6.0)), 1000)()


def test_the_saved_revision_records_what_it_continued(tmp_path, monkeypatch):
    bundle(str(tmp_path))
    sv = session(str(tmp_path))
    assert start(sv).accepted
    sv._review = mb.ReturnReview(0.01, -0.02, 0.003)
    sv._serialize = "serialize"
    sv.save_map_wait_s = 4.5
    sv._pause_slam = lambda paused: None
    grid_msg = types.SimpleNamespace(header=types.SimpleNamespace(frame_id="map"))
    sv._wait_for_map = lambda after, timeout: grid_msg
    monkeypatch.setattr(gridio, "from_occupancy_grid_msg", lambda msg: build())

    def serialize(client, req, timeout=None):  # slam_toolbox writes the graph into the stage
        for ext in (".posegraph", ".data"):
            with open(req.filename + ext, "wb") as fh:
                fh.write(b"graph2")

    sv._call = serialize
    with mb.map_lock(str(tmp_path), "hall"):
        revision, dest, manifest = MappingSession._save_locked(sv, SaveMap.Request(note="joined"), [])
    assert revision == 2 and manifest.start == MARK
    lineage = mb.load_manifest(dest, "hall", 2).review["continued_from"]
    assert lineage["revision"] == 1 and lineage["match"]["score"] > 0.8
    assert manifest.review["note"] == "joined" and manifest.review["dx_m"] == 0.01
    assert np.array_equal(mb.load(str(tmp_path), "hall", 2)[1].data, build().data)


if __name__ == "__main__":
    pytest.main([__file__])
