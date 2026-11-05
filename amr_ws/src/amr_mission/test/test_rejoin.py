"""Continue route from here (2026-11-05): a run aborted or faulted part-way is remembered
(route_checkpoint.json) and can be loaded again at the step where it stopped. Start then
checks the vehicle is ON that step (rejoin_check) instead of at the route start. Built with
__new__ like test_wait_step."""

import json
import math
from types import SimpleNamespace

from amr_navigation.compiler import ARC, ROTATE, STRAIGHT, WAIT, CompiledStep
from test_route_executor_logic import make_node

from amr_mission import route_executor_node as ren
from amr_mission import run_fsm as fsm

D = math.radians
KW = dict(corridor=0.30, pos_tol=0.05, verify_pos=0.15, verify_yaw=D(8), heading_tol=D(20), turn_tol=D(2))


def straight(sid, x0, x1, y=0.0):
    n = round((x1 - x0) / 0.1)
    return CompiledStep(
        sid, STRAIGHT, (x0, y, 0.0), (x1, y, 0.0), length_m=x1 - x0,
        samples=[(x0 + 0.1 * k, y, 0.0) for k in range(n + 1)], v_mps=0.27,
    )


def turn(sid, x, angle, yaw0=0.0):
    return CompiledStep(sid, ROTATE, (x, 0.0, yaw0), (x, 0.0, ren.wrap(yaw0 + angle)),
                        signed_angle_rad=angle, time_allowance_s=10.0)


def check(st, pose, n=None):
    ac = (0.0, 0.0) if st.type in (ROTATE, WAIT) else ren.RouteExecutor._along_cross(st, pose)
    return ren.rejoin_check(st, pose, ac, **KW)


# ---- the pure check ----------------------------------------------------------------------------


def test_a_straight_is_rejoined_anywhere_along_it_inside_the_corridor():
    st = straight("s1", 0.0, 4.0)
    assert check(st, (1.7, 0.12, D(5)))[0] == ren.REJOIN_OK
    v, why, _ = check(st, (1.7, 0.35, 0.0))
    assert v == ren.REJOIN_FAR and "beside the path" in why
    v, why, _ = check(st, (-0.3, 0.0, 0.0))
    assert v == ren.REJOIN_FAR and "before the step's start" in why
    v, why, _ = check(st, (1.7, 0.0, D(35)))
    assert v == ren.REJOIN_FAR and "heading" in why
    assert check(st, (3.98, 0.02, 0.0))[0] == ren.REJOIN_DONE, "nothing left of it: the next step"


def test_an_arc_compares_the_heading_with_the_arc_s_own_at_that_point():
    r = 1.0
    st = CompiledStep("a1", ARC, (0.0, 0.0, 0.0), (1.0, 1.0, D(90)), length_m=r * D(90),
                      signed_angle_rad=D(90), centre=(0.0, 1.0), radius_m=r)
    p = (math.sin(D(45)), 1.0 - math.cos(D(45)), D(45))  # halfway round, facing along it
    assert check(st, p)[0] == ren.REJOIN_OK
    assert check(st, (p[0], p[1], 0.0))[0] == ren.REJOIN_FAR, "45 deg off the arc's heading there"


def test_a_turn_in_place_turns_only_what_is_left():
    st = turn("t1", 2.0, D(90))
    v, _, left = check(st, (2.03, 0.0, D(30)))
    assert v == ren.REJOIN_OK and math.isclose(left, D(60), abs_tol=1e-9)
    v, _, left = check(st, (2.0, 0.0, -D(5)))  # a little short of its start heading
    assert v == ren.REJOIN_OK and math.isclose(left, D(95), abs_tol=1e-9)
    assert check(st, (2.0, 0.0, D(89)))[0] == ren.REJOIN_DONE
    v, why, _ = check(st, (2.0, 0.0, D(180)))
    assert v == ren.REJOIN_FAR and "not on the turn" in why
    v, why, _ = check(st, (2.3, 0.0, D(30)))
    assert v == ren.REJOIN_FAR and "turn point" in why


def test_a_clockwise_270_turn_is_followed_the_long_way_round():
    st = turn("t1", 0.0, -D(270))
    v, _, left = check(st, (0.0, 0.0, -D(200)))  # 200 deg of 270 done, clockwise
    assert v == ren.REJOIN_OK and math.isclose(left, -D(70), abs_tol=1e-9)


def test_a_wait_is_rejoined_only_at_its_point():
    st = CompiledStep("w1", WAIT, (2.0, 0.0, 0.0), (2.0, 0.0, 0.0), name="load")
    assert check(st, (2.05, 0.0, D(3)))[0] == ren.REJOIN_OK
    assert check(st, (2.4, 0.0, 0.0))[0] == ren.REJOIN_FAR


# ---- the run state machine ---------------------------------------------------------------------


def test_abort_and_fault_part_way_remember_the_step_and_a_load_starts_there():
    f = fsm.RunFsm()
    assert f.load("m1", 4, 2) and f.start(True, True)
    f.step_index, f.pass_index = 2, 1
    assert f.abort("aborted by operator")
    stop = f.last_stop
    assert (stop["step_index"], stop["pass_index"], stop["kind"]) == (2, 1, "aborted")
    assert f.load("m1", 4, 2, start_at=(1, 2)) and "continue" in f.reason
    assert f.start(True, True) and (f.pass_index, f.step_index) == (1, 2) and f.start_at is None
    assert f.fault("endpoint missed") and f.last_stop["kind"] == "fault" and f.last_stop["seq"] == 2
    assert f.ack() and f.state == fsm.IDLE and f.last_stop["step_index"] == 2


def test_nothing_is_remembered_for_a_run_that_never_started():
    f = fsm.RunFsm()
    assert f.load("m1", 3) and f.abort("aborted by operator") and f.last_stop is None
    assert not f.load("m1", 3, 1, start_at=(0, 3)), "no such step"


# ---- the file ----------------------------------------------------------------------------------


MISSION = {
    "map": {"id": "m", "revision": 2, "sha256": "aa"},
    "route": {"id": "r", "revision": 1, "sha256": "bb"},
}


def test_the_checkpoint_survives_a_restart_and_refuses_a_changed_route(tmp_path):
    path = str(tmp_path / "state" / ren.CHECKPOINT_FILE)
    cp = {"mission_id": "mi", "map": MISSION["map"], "route": MISSION["route"], "pass_index": 0,
          "step_index": 1, "step_id": "s2", "n_steps": 2, "n_passes": 1, "kind": "fault", "reason": "x"}
    ren.save_checkpoint(path, cp)
    assert ren.load_checkpoint(path) == cp
    steps = SimpleNamespace(steps=[SimpleNamespace(id="s1"), SimpleNamespace(id="s2")])
    assert ren.checkpoint_mismatch(cp, MISSION, steps, 1) is None
    changed = {**MISSION, "route": {**MISSION["route"], "sha256": "cc"}}
    assert "route changed" in ren.checkpoint_mismatch(cp, changed, steps, 1)
    other = SimpleNamespace(steps=[SimpleNamespace(id="s1"), SimpleNamespace(id="s9")])
    assert "now" in ren.checkpoint_mismatch(cp, MISSION, other, 1)
    ren.save_checkpoint(path, None)
    assert ren.load_checkpoint(path) is None
    (tmp_path / "bad.json").write_text("{not json")
    assert ren.load_checkpoint(str(tmp_path / "bad.json")) is None


# ---- the executor ------------------------------------------------------------------------------


def node(tmp_path, pose):
    steps = [straight("s1", 0.0, 2.0), turn("t1", 2.0, D(90)), straight("s2", 0.0, 0.0)]
    steps[2] = CompiledStep("s2", STRAIGHT, (2.0, 0.0, D(90)), (2.0, 2.0, D(90)), length_m=2.0,
                            samples=[(2.0, 0.1 * k, D(90)) for k in range(21)], v_mps=0.27)
    n = make_node(steps)
    n.verify_pos, n.verify_yaw, n.rejoin_heading = 0.15, D(8), D(20)
    n.checkpoint_path = str(tmp_path / ren.CHECKPOINT_FILE)
    n.checkpoint, n._saved_stop_seq = None, 0
    n.mission = MISSION
    n.where = list(pose)
    n._pose = lambda: tuple(n.where)
    n._publish_state = lambda: n._sync_checkpoint()
    return n


def test_an_aborted_run_continues_on_its_straight_from_where_it_stands(tmp_path):
    n = node(tmp_path, (0.0, 0.0, 0.0))
    n._start_ready()
    assert n.fsm.state == fsm.EXECUTING and n.fsm.step_index == 0
    n.where = [1.2, 0.04, 0.0]
    assert n.fsm.abort("aborted by operator")
    n._publish_state()
    saved = json.load(open(n.checkpoint_path))
    assert saved["step_id"] == "s1" and saved["kind"] == "aborted" and n.checkpoint is not None
    assert "s1" in ren.checkpoint_text(saved) and "aborted" in ren.checkpoint_text(saved)
    n.where = [1.25, 0.18, D(6)]  # jogged a little since
    assert n.fsm.load("m1", 3, 1, start_at=(0, 0))
    n._start_ready()
    assert n.fsm.state == fsm.EXECUTING and n.fsm.step_index == 0
    assert n._resumed_at is not None and n._resumed_at[0] == "s1", "the resume corridor applies"
    assert n.checkpoint is not None, "kept until the run finishes or a new run starts"


def test_too_far_from_the_stopped_step_is_refused_with_the_distance(tmp_path):
    n = node(tmp_path, (1.0, 0.6, 0.0))
    assert n.fsm.load("m1", 3, 1, start_at=(0, 0))
    n._start_ready()
    assert n.fsm.state == fsm.READY and "beside the path" in n.fsm.reason and "jog it back" in n.fsm.reason


def test_a_turn_cut_part_way_turns_only_the_rest(tmp_path):
    n = node(tmp_path, (2.02, 0.0, D(35)))
    assert n.fsm.load("m1", 3, 1, start_at=(0, 1))
    n._start_ready()
    assert n.fsm.state == fsm.EXECUTING and n.fsm.step_index == 1
    assert math.isclose(n._remaining_turn(n.compiled.steps[1]), D(55), abs_tol=1e-9)


def test_a_finished_step_continues_with_the_next(tmp_path):
    n = node(tmp_path, (1.99, 0.01, 0.0))  # "endpoint missed" at the end of s1: on t1's point
    assert n.fsm.load("m1", 3, 1, start_at=(0, 0))
    n._start_ready()
    assert n.fsm.state == fsm.EXECUTING and n.fsm.step_index == 1


def test_a_normal_start_and_a_finished_run_forget_the_checkpoint(tmp_path):
    n = node(tmp_path, (0.0, 0.0, 0.0))
    n._start_ready()
    n.fsm.abort("aborted by operator")
    n._publish_state()
    assert n.checkpoint is not None
    assert n.fsm.load("m1", 3, 1)
    n._start_ready()
    assert n.fsm.state == fsm.EXECUTING and n.checkpoint is None
    assert ren.load_checkpoint(n.checkpoint_path) is None
