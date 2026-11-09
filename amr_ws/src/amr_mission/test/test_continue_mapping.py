"""Continue mapping (2026-11-06): the survey starts from a saved revision's pose graph with the
vehicle on its survey mark, and its save is a new revision of the same map in the same frame -
the base cells kept, the new area added, the marks moved onto the bigger map."""

import collections
import os
import threading
import types

import numpy as np
from amr_maps.grid import Grid, GridMeta
from amr_mission.mapping_session_node import MappingSession
from test_map_bundle import fake_posegraph, manifest, small_grid

from amr_interfaces.msg import MappingState
from amr_interfaces.srv import SaveMap, StartSurvey
from amr_maps import edit
from amr_maps import grid as gridio
from amr_mission import map_bundle as mb
from amr_mission import mapping_session_node as msn


class Log:
    def info(self, *_):
        pass

    error = warn = info


def _base(tmp_path):
    maps = str(tmp_path / "maps")
    os.makedirs(maps)
    g = small_grid()
    m = manifest(1)
    m.start = {"x_m": 0.25, "y_m": 0.0, "yaw_rad": 0.5, "description": "mark A"}
    stage = mb.staging_dir(maps, "line_section", 1)
    dyn = np.zeros(g.data.shape, dtype=bool)
    dyn[6:8, 6:9] = True
    gridio.write(edit.mask_grid(g, dyn), os.path.join(stage, "dynamic"))
    mb.stage_bundle(stage, g, fake_posegraph(tmp_path), m)
    mb.publish(stage, maps, "line_section", 1)
    return maps


def _session(maps):
    sv = types.SimpleNamespace(
        _state=MappingState.IDLE,
        maps_dir=maps,
        _lock=threading.Lock(),
        _gls=collections.deque(),
        _still_since=0.0,
        _slam_paused=False,
        calls=[],
        states=[],
    )
    sv._now = lambda: 10.0
    sv._readiness = lambda: []
    sv._pose_in_map = lambda: (3.0, 4.0, 1.0)  # SLAM's fresh-graph pose: NOT the start reference
    sv.get_logger = Log
    sv._deserialize, sv._serialize = "deserialize", "serialize"

    def call(client, req):
        sv.calls.append((client, req))
        if client == "serialize":  # slam_toolbox writes the graph into the stage
            for ext in (".posegraph", ".data"):
                with open(req.filename + ext, "wb") as fh:
                    fh.write(b"\x01" * 64)

    sv._call = call

    def _set(state, message):
        sv._state, sv._message = state, message
        sv.states.append(state)

    sv._set = _set
    sv._continue_from = lambda map_id, rev: MappingSession._continue_from(sv, map_id, rev)
    sv._pause_slam = lambda p: None
    sv.event = lambda *_a: None
    sv._save_home = lambda *_a: ""
    sv._save_locked = lambda req, holder: MappingSession._save_locked(sv, req, holder)
    return sv


def test_continue_loads_the_pose_graph_at_the_survey_mark_and_saves_the_same_frame(tmp_path, monkeypatch):
    maps = _base(tmp_path)
    sv = _session(maps)
    req = StartSurvey.Request()
    req.map_id, req.description, req.continue_revision = "line_section", "", 1
    res = MappingSession._srv_start(sv, req, StartSurvey.Response())
    assert res.accepted, res.message
    ((client, dreq),) = sv.calls
    assert client == "deserialize" and dreq.filename.endswith(os.path.join("rev1", "posegraph"))
    assert (dreq.initial_pose.x, dreq.initial_pose.y, dreq.initial_pose.theta) == (0.25, 0.0, 0.5)
    # the start reference is the base's mark (and its description), not SLAM's fresh pose
    assert (sv._start.x_m, sv._start.y_m, sv._start.yaw_rad, sv._start.description) == (
        0.25,
        0.0,
        0.5,
        "mark A",
    )
    assert "continuing 'line_section' rev1" in sv._message
    # drive, return, save: SLAM's map now reaches 1 m further right
    sv._state, sv._review = MappingState.RETURN_REVIEW, mb.ReturnReview(0.01, 0.0, 0.0)
    slam = Grid(np.full((20, 50), -1, dtype=np.int8), GridMeta(0.05, -1.0, -0.5))
    slam.data[5:15, 5:45] = 0
    slam.data[8, 40] = 100
    monkeypatch.setattr(msn.gridio, "from_occupancy_grid_msg", lambda _m: slam)
    sv._wait_for_map = lambda after, timeout: types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id="map")
    )
    out = MappingSession._srv_save(sv, SaveMap.Request(note="new aisle"), SaveMap.Response())
    assert out.ok and out.revision == 2, out.message
    m2, g2 = mb.load(maps, "line_section", 2)
    assert (g2.width, g2.height) == (50, 20) and g2.data[8, 40] == 100 and g2.data[5, 10] == 100
    assert m2.start["x_m"] == 0.25 and m2.software["continued_from"]["revision"] == 1
    rev2 = mb.revision_dir(maps, "line_section", 2)
    assert int((mb.load_mask(rev2, "dynamic").data >= 65).sum()) == 6
    assert mb.load_edits(rev2)["ops"] == [{"op": "continue_mapping"}]
    assert all(os.path.isfile(os.path.join(rev2, n)) for n in mb.POSEGRAPH)  # it can be continued too


def test_a_revision_without_a_pose_graph_is_refused_and_nothing_starts(tmp_path):
    maps = str(tmp_path / "maps")
    os.makedirs(maps)
    stage = mb.staging_dir(maps, "line_section", 1)
    mb.stage_bundle(stage, small_grid(), None, manifest(1), required_posegraph=False)
    mb.publish(stage, maps, "line_section", 1)
    sv = _session(maps)
    req = StartSurvey.Request()
    req.map_id, req.continue_revision = "line_section", 1
    res = MappingSession._srv_start(sv, req, StartSurvey.Response())
    assert not res.accepted and "no SLAM pose graph" in res.message
    assert sv.calls == [] and sv._state == MappingState.IDLE
