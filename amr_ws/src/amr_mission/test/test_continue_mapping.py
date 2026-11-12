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
    sv._theta = lambda: MappingSession._theta(sv)
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


def test_a_continued_save_carries_its_own_scans_for_re_map_area(tmp_path, monkeypatch):
    """Re-map area (2026-11-09): the continued survey's keyframes become survey_scan, on the
    saved map's geometry; a rotation drops it; a re-map without fixed structure needs force."""
    import math  # noqa: PLC0415

    from amr_maps import survey_scan  # noqa: PLC0415

    maps = _base(tmp_path)
    sv = _session(maps)
    req = StartSurvey.Request()
    req.map_id, req.continue_revision = "line_section", 1
    assert MappingSession._srv_start(sv, req, StartSurvey.Response()).accepted
    assert sv._keyframes == []
    n = 181
    r = np.full(n, 0.6, dtype=np.float32)  # a ring of returns 0.6 m around the laser
    sv._keyframes.append(survey_scan.Keyframe(0.5, 0.0, 0.0, -math.pi / 2, math.pi / (n - 1), r))
    sv._keyframes.append(survey_scan.Keyframe(0.55, 0.0, 0.0, -math.pi / 2, math.pi / (n - 1), r))
    sv._state, sv._review = MappingState.RETURN_REVIEW, mb.ReturnReview(0.01, 0.0, 0.0)
    slam = Grid(np.full((20, 50), -1, dtype=np.int8), GridMeta(0.05, -1.0, -0.5))
    monkeypatch.setattr(msn.gridio, "from_occupancy_grid_msg", lambda _m: slam)
    msg = types.SimpleNamespace(header=types.SimpleNamespace(frame_id="map"))
    sv._wait_for_map = lambda after, timeout: msg
    out = MappingSession._srv_save(sv, SaveMap.Request(note="re-mapped aisle"), SaveMap.Response())
    assert out.ok, out.message
    rev2 = mb.revision_dir(maps, "line_section", 2)
    scan = mb.load_survey_scan(rev2)
    _m2, g2 = mb.load(maps, "line_section", 2)
    assert scan is not None and scan.data.shape == g2.data.shape and (scan.data == 100).any()
    box = edit.rectangle(-0.2, -0.5, 1.4, 0.45)
    _new, changed, check = mb.remap_preview(maps, "line_section", 2, [box])
    assert changed > 0 and check["ok"] in (None, False, True)
    if check["ok"] is None:
        import pytest  # noqa: PLC0415

        with pytest.raises(edit.EditError):
            mb.derive_remap(maps, "line_section", 2, [box])
        _p, rev3, info = mb.derive_remap(maps, "line_section", 2, [box], "aisle", force=True)
        e3 = mb.load_edits(mb.revision_dir(maps, "line_section", rev3))
        assert e3["ops"][0]["op"] == "remap" and info["changed_cells"] == changed
    _p, rot, _g = mb.derive_rotation(maps, "line_section", 2, math.radians(10))
    assert mb.load_survey_scan(mb.revision_dir(maps, "line_section", rot)) is None


def test_a_rotated_revision_is_continued_in_its_pose_graphs_frame_and_saved_in_its_own(tmp_path, monkeypatch):
    """2026-11-12: the operator rotated the map, then needs Continue mapping (Re-map area).
    slam_toolbox gets the mark turned back into the pose graph's frame; the save turns SLAM's
    map, the closure and the survey scan forward into the rotated frame."""
    import math  # noqa: PLC0415

    from amr_maps import rotate as maprot  # noqa: PLC0415
    from amr_maps import survey_scan  # noqa: PLC0415

    maps = _base(tmp_path)
    _p, rot, g_rot = mb.derive_rotation(maps, "line_section", 1, math.radians(90))
    m_rot = mb.verify(mb.revision_dir(maps, "line_section", rot))
    sv = _session(maps)
    req = StartSurvey.Request()
    req.map_id, req.continue_revision = "line_section", rot
    res = MappingSession._srv_start(sv, req, StartSurvey.Response())
    assert res.accepted, res.message
    ((_client, dreq),) = sv.calls
    # the mark in the graph's frame is the ORIGINAL survey mark (0.25, 0, 0.5)
    assert math.isclose(dreq.initial_pose.x, 0.25, abs_tol=1e-9)
    assert math.isclose(dreq.initial_pose.y, 0.0, abs_tol=1e-9)
    assert math.isclose(dreq.initial_pose.theta, 0.5, abs_tol=1e-9)
    assert sv._start.x_m == m_rot.start["x_m"] and math.isclose(sv._theta(), math.pi / 2)
    # back at the mark as SLAM sees it: the closure is ~0 in the rotated frame too
    sv._pose_in_map = lambda: (0.26, 0.0, 0.5)
    MappingSession._srv_returned(sv, None, types.SimpleNamespace(success=False, message=""))
    assert math.isclose(sv._review.dx_m, 0.0, abs_tol=1e-6)
    assert math.isclose(sv._review.dy_m, 0.01, abs_tol=1e-6)
    n = 181
    r = np.full(n, 0.6, dtype=np.float32)
    sv._keyframes.append(survey_scan.Keyframe(0.5, 0.0, 0.0, -math.pi / 2, math.pi / (n - 1), r))
    # SLAM's map (graph frame) sees a new obstacle at (0.98, 0.0): in the rotated map (0.0, 0.98)
    slam = Grid(np.full((20, 50), -1, dtype=np.int8), GridMeta(0.05, -1.0, -0.5))
    slam.data[5:15, 5:45] = 0
    slam.data[10, 39] = 100
    monkeypatch.setattr(msn.gridio, "from_occupancy_grid_msg", lambda _m: slam)
    msg = types.SimpleNamespace(header=types.SimpleNamespace(frame_id="map"))
    sv._wait_for_map = lambda after, timeout: msg
    out = MappingSession._srv_save(sv, SaveMap.Request(note="after rotate"), SaveMap.Response())
    assert out.ok, out.message
    m2, g2 = mb.load(maps, "line_section", out.revision)
    assert m2.software[mb.SLAM_FRAME_KEY] == 90.0 and m2.start["x_m"] == m_rot.start["x_m"]
    wx, wy = maprot.rotate_point(-1.0 + 39.5 * 0.05, -0.5 + 10.5 * 0.05, math.pi / 2)
    assert g2.data[g2.world_to_cell(wx, wy)] == 100  # SLAM's new cell, turned into the rotated frame
    assert mb.load_survey_scan(mb.revision_dir(maps, "line_section", out.revision)) is not None
    # and the result can be continued again, straight from its own pose graph
    _m, stem, theta = mb.continue_base(maps, "line_section", out.revision)
    assert stem.endswith(os.path.join(f"rev{out.revision}", "posegraph"))
    assert math.isclose(theta, math.pi / 2)
