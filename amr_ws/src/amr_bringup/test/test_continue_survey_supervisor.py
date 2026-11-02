"""Continue-survey at the supervisor (2026-10-23): a START that continues a revision is checked
before the running layer is torn down, carries the revision and pose hint to the session,
and gets the pose-graph load added to its layer start budget."""

import math
import os
import threading
import types

import pytest
from amr_maps.generate_sim_factory import build

from amr_bringup import mode_fsm as fsm
from amr_bringup import operations as ops
from amr_bringup import readiness as rd
from amr_bringup.supervisor_node import Supervisor
from amr_interfaces.srv import RequestSurvey
from amr_mission import map_bundle as mb


def bundle(maps_dir, map_id="hall", posegraph=True):
    g = build()
    stage = mb.staging_dir(maps_dir, map_id, 1)
    origin = [g.meta.origin_x, g.meta.origin_y, 0.0]
    m = mb.Manifest(map_id, 1, mb.now_iso(), "map", g.meta.resolution, origin,
                    g.width, g.height, {"x_m": 0, "y_m": 0, "yaw_rad": 0, "description": "mark"},
                    {"dx_m": 0, "dy_m": 0, "dyaw_rad": 0, "note": ""})
    stem = None
    if posegraph:
        stem = os.path.join(maps_dir, "pg")
        for ext in (".posegraph", ".data"):
            with open(stem + ext, "wb") as fh:
                fh.write(b"graph")
    mb.stage_bundle(stage, g, stem, m, required_posegraph=posegraph)
    return mb.publish(stage, maps_dir, map_id, 1)


def test_continuable_needs_a_verified_revision_with_its_pose_graph(tmp_path):
    rev_dir = bundle(str(tmp_path))
    assert rd.continuable(str(tmp_path), "hall", 1) == mb.load_manifest(rev_dir, "hall", 1).sha256
    bundle(str(tmp_path), "merged", posegraph=False)
    with pytest.raises(ValueError, match="no saved pose graph"):
        rd.continuable(str(tmp_path), "merged", 1)
    with pytest.raises(ValueError, match="no such map revision"):
        rd.continuable(str(tmp_path), "hall", 2)
    with open(os.path.join(rev_dir, "posegraph.data"), "ab") as fh:
        fh.write(b"tampered")
    with pytest.raises(ValueError, match="hash mismatch"):
        rd.continuable(str(tmp_path), "hall", 1)


def test_hint_must_be_a_finite_pose():
    rd.check_hint(51.8, -38.9, 0.03)
    for bad in ((math.nan, 0, 0), (0, math.inf, 0), (2e5, 0, 0)):
        with pytest.raises(ValueError):
            rd.check_hint(*bad)


class Client:
    def __init__(self):
        self.reqs = []

    def service_is_ready(self):
        return True

    def call_async(self, req):
        self.reqs.append(req)
        return types.SimpleNamespace(done=lambda: False)


def fake(maps_dir):
    sv = types.SimpleNamespace(maps_dir=maps_dir, _lock=threading.RLock(), book=ops.OperationBook(None),
                               mode=fsm.IDLE, admit_params=None, txn=None, survey_op=None, began=[],
                               _pending_future=None, _continue_hint=None, generation=2,
                               budget={"layer_start_s": 30.0, "continue_load_s": 150.0})
    sv.disk = types.SimpleNamespace(blocks_new_work=False)
    sv._check_expected = lambda inst, gen: ""
    sv._conditions = lambda: None
    sv._begin_transaction = lambda op, target, map_id, rev, sha: sv.began.append((target, map_id, rev, sha))
    sv._cli = {"start": Client()}
    sv.snap = types.SimpleNamespace(mapping_state_for=lambda gen: True)
    return sv


def survey_start(sv, map_id="hall", rev=1, hint=(51.8, -38.9, 0.03)):
    req = RequestSurvey.Request(request_id=f"r{len(sv.began)}-{rev}-{map_id}",
                                operation=RequestSurvey.Request.START,
                                map_id=map_id, description="by rack C", continue_revision=rev)
    req.hint.x, req.hint.y, req.hint.theta = hint
    return Supervisor._srv_survey(sv, req, RequestSurvey.Response())


def test_start_continue_is_checked_before_the_layer_is_replaced(tmp_path, monkeypatch):
    monkeypatch.setattr(fsm, "admit", lambda *a: types.SimpleNamespace(ok=True, reason=""))
    bundle(str(tmp_path))
    bundle(str(tmp_path), "merged", posegraph=False)
    sv = fake(str(tmp_path))
    res = survey_start(sv)
    sha = rd.continuable(str(tmp_path), "hall", 1)
    assert res.accepted and sv.began == [(fsm.MAPPING, "hall", 1, sha)]
    assert sv._continue_hint == (51.8, -38.9, 0.03)
    for kw in ({"map_id": "merged"}, {"rev": 4}, {"hint": (math.nan, 0.0, 0.0)}):
        res = survey_start(sv, **kw)
        assert not res.accepted and res.message
    assert len(sv.began) == 1  # refused STARTs never reached a transaction
    res = survey_start(sv, map_id="fresh", rev=0)  # an ordinary new map is unchanged
    assert res.accepted and sv.began[-1] == (fsm.MAPPING, "fresh", 0, "") and sv._continue_hint is None


def test_the_session_receives_the_revision_and_hint_and_the_budget_grows(tmp_path):
    sv = fake(str(tmp_path))
    op, _ = sv.book.submit("r1", fsm.REQ_SURVEY_START, map_id="hall", message="by rack C")
    t = fsm.Transaction(op.operation_id, fsm.MAPPING, fsm.IDLE, 2, map_id="hall", map_revision=1)
    sv._continue_hint = (51.8, -38.9, 0.03)
    Supervisor._ready_mapping(sv, t, 0.0)
    [req] = sv._cli["start"].reqs
    assert (req.map_id, req.description, req.continue_revision) == ("hall", "by rack C", 1)
    assert (req.hint.x, req.hint.y, req.hint.theta) == (51.8, -38.9, 0.03)
    assert Supervisor._layer_start_budget(sv, t) == 180.0
    t.map_revision = 0
    assert Supervisor._layer_start_budget(sv, t) == 30.0
    nav = fsm.Transaction("x", fsm.NAVIGATION, fsm.IDLE, 2, map_id="hall", map_revision=1)
    assert Supervisor._layer_start_budget(sv, nav) == 30.0  # only a continued survey loads a graph
