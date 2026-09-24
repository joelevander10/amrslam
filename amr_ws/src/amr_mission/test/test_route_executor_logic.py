"""R07/R18/R08 wiring in the route executor, without a ROS graph: the node object is
built with __new__ and only the state these methods read is set."""

import math
import threading
from types import SimpleNamespace

import pytest
from amr_navigation.compiler import ROTATE, STRAIGHT, CompiledStep
from test_goal_attempts import Client, Handle, Result

from amr_interfaces.msg import LocalizationState, MotionPermit, PanelState, WheelStates
from amr_mission import goal_attempts as ga
from amr_mission import route_executor_node as ren
from amr_mission import run_fsm as fsm


class Logger:
    def info(self, *_a, **_k):
        pass

    warn = info


def make_node(steps, passes=1):
    n = ren.RouteExecutor.__new__(ren.RouteExecutor)
    n._lock = threading.RLock()
    n.clock = [100.0]
    n._now = lambda: n.clock[0]
    n.get_logger = lambda: Logger()
    n._log_state = lambda *_a: None
    n.fsm = fsm.RunFsm()
    assert n.fsm.load("m1", len(steps), passes)
    n.compiled = SimpleNamespace(steps=steps)
    n.route = SimpleNamespace(
        start=SimpleNamespace(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        limits=SimpleNamespace(
            cross_track_limit_m=0.1, position_tolerance_m=0.05, linear_mps=0.5, w_mps=0.24
        ),
    )
    n._speed_pub = SimpleNamespace(publish=lambda _m: None)
    n.horizon, n.stopping_time_s = 1.5, 3.0
    n.taper_decel, n.taper_lead_s = 0.4, 0.3
    n.obstacle_points, n.obstacle_persist, n._hit_scans, n._hit_scan_t, n._map_near = 5, 2, 0, None, None
    n.goals = ga.GoalAttempts(n._lock, n._now, n._on_goal_error)
    n.start_gate_m, n.start_gate_rad = 0.1, math.radians(5)
    n.w_eps, n.wheels_age, n.panel_age, n.loc_age = 0.02, 0.1, 0.2, 1.5
    n.scan_age, n._scan_t = float("inf"), 100.0  # freshness is exercised where a test sets scan_age
    n.centre_drift_m, n.turn_tol, n.wrong_way = 0.05, math.radians(2), math.radians(5)
    n.entry_corr_max, n.clear_stable_s = math.radians(10), 1.0
    n.goal_accept_timeout, n.goal_cancel_timeout = 5.0, 5.0
    n.converge_m = 1.0
    n.fp = SimpleNamespace(margin_m=0.20)
    n.odom_gap, n.paused_t = False, None
    n._odom_xy, n._odom_yaw_acc = (0.0, 0.0), 0.0
    loc = LocalizationState()
    loc.state = LocalizationState.READY
    n._loc, n._loc_t = loc, n.clock[0]
    panel = PanelState()
    panel.valid, panel.mode_auto = True, True
    n._panel, n._panel_t = panel, n.clock[0]
    n._scan = object()
    n._pose = lambda: (0.0, 0.0, 0.0)
    n._obstruction = lambda st, pose: None
    n.auto_resume_enabled, n.auto_clear_s, n.auto_resume_estop = True, 2.0, False
    n.safety_window, n.drives_age, n.field_index, n.abort_retries = 1.0, 0.5, 0, 3
    n._init_hold_state()
    n._reset_step_state()
    n.clear_since = n.clock[0] - 10.0
    n.wheels(0.0, valid=True)
    return n


def _wheels(self, vel, valid=True):
    m = WheelStates()
    m.left_valid = m.right_valid = valid
    m.left_vel_rad_s = m.right_vel_rad_s = vel
    self._on_wheels(m)


ren.RouteExecutor.wheels = _wheels


def rotate(angle, sid="t1"):
    return CompiledStep(
        id=sid,
        type=ROTATE,
        start=(0.0, 0.0, 0.0),
        end=(0.0, 0.0, angle),
        signed_angle_rad=angle,
        time_allowance_s=10.0,
    )


def test_r07_fresh_but_invalid_wheel_feedback_fails_prerequisites():
    n = make_node([rotate(1.0)])
    assert n._prereqs() is None
    n.wheels(0.0, valid=False)
    assert n._prereqs() == "wheel feedback invalid"
    n.clock[0] += 1.0
    assert n._prereqs() == "wheel feedback stale"


def test_r18_aborted_turn_geometry_does_not_leak_into_the_next_run():
    n = make_node([rotate(math.pi / 2)])
    n.fsm.start(True, True)
    n.turn_centre, n.turn_target, n.turn_acc0 = (1.0, 1.0), math.pi / 2, 0.0
    n._odom_yaw_acc = 0.7
    n.fsm.abort("operator")
    n._interrupt("aborted")
    # a new mission is loaded: the executor's load path resets all per-step state
    assert n.fsm.load("m2", 1)
    n._reset_step_state()
    assert (n.turn_centre, n.turn_target, n.turn_acc0, n.turn_travelled) == (None, None, None, 0.0)
    assert n._remaining_turn(rotate(-1.0)) == -1.0


def test_r18_start_from_ready_resets_turn_state():
    n = make_node([rotate(1.0)])
    n.turn_centre, n.turn_target, n.turn_acc0 = (5.0, 5.0), 3.0, -2.0
    n._start_edge()
    assert n.fsm.state == fsm.EXECUTING
    assert (n.turn_centre, n.turn_target, n.turn_acc0) == (None, None, None)


def test_r18_pause_keeps_counting_rotation_until_standstill():
    n = make_node([rotate(1.0)])
    n.fsm.start(True, True)
    n.turn_target, n.turn_centre = 1.0, (0.0, 0.0)
    n.turn_acc0, n.phase = 0.0, ren.PHASE_GOAL
    n._odom_yaw_acc = 0.4
    n.fsm.pause()
    n._interrupt("paused by operator")
    n._odom_yaw_acc = 0.5  # still decelerating after the pause
    assert math.isclose(n._remaining_turn(n._step()), 0.5)


def test_r18_start_edge_rechecks_a_prepared_resume():
    n = make_node([rotate(1.0)])
    n.fsm.start(True, True)
    n.turn_centre = (0.0, 0.0)
    n.fsm.pause()
    n._interrupt("paused")
    ok, why = n._resume_checks()
    assert ok, why
    assert n.fsm.prepare_resume(ok, why)
    n._odom_xy = (0.2, 0.0)  # pushed off the turn centre after Prepare resume
    n._start_edge()
    assert n.fsm.state == fsm.PAUSED and "turn centre" in n.fsm.reason
    n._odom_xy = (0.0, 0.0)
    n.odom_gap = True
    assert n.fsm.prepare_resume(True, "")  # (as if prepared before the gap)
    n._start_edge()
    assert n.fsm.state == fsm.PAUSED and "odometry continuity" in n.fsm.reason


def test_r08_resume_waits_for_the_interrupted_goal_then_faults_after_the_bound():
    n = make_node([rotate(1.0)])
    n.fsm.start(True, True)
    c = Client()
    n.goals.send(c, "g", n.fsm.run_id, 0, 0)
    n.phase = ren.PHASE_GOAL
    n.clock[0] += 30.0  # a long step: the bound runs from the pause, not from the send
    n._loc_t = n._panel_t = n.clock[0]
    n.wheels(0.0)
    n.fsm.pause()
    n._interrupt("paused")
    n.fsm.prepare_resume(True, "")
    n._start_edge()
    assert n.fsm.state == fsm.EXECUTING
    n._execute(n.clock[0])
    assert n.phase == ren.PHASE_INIT and len(c.sent) == 1  # no replacement goal yet
    h = Handle()
    c.sent[0][1].set_result(h)  # late acceptance of the paused goal
    assert h.cancels == 1
    n.clock[0] += 6.0
    n._execute(n.clock[0])
    assert n.fsm.state == fsm.FAULT and "not terminated" in n.fsm.reason
    assert not n.goals.outstanding


def test_r08_old_result_cannot_fault_the_resumed_attempt():
    n = make_node([rotate(1.0)])
    n.fsm.start(True, True)
    c = Client()
    n.goals.send(c, "g", n.fsm.run_id, 0, 0)
    old = Handle()
    c.sent[0][1].set_result(old)
    n.fsm.pause()
    n._interrupt("paused")
    old.result_fut.set_result(Result(5))
    n.fsm.prepare_resume(True, "")
    n._start_edge()
    n.goals.send(c, "g", n.fsm.run_id, 0, 0)
    new = Handle()
    c.sent[1][1].set_result(new)
    assert n.goals.result is None and n.goals.handle is new and n.fsm.state == fsm.EXECUTING


def test_r20_step_done_advances_passes_through_the_node_reset():
    n = make_node([rotate(1.0, "a"), rotate(-1.0, "b")], passes=2)
    n.fsm.start(True, True)
    seen = []
    while n.fsm.state == fsm.EXECUTING:
        seen.append((n.fsm.pass_index, n._step().id))
        n.turn_target = 9.0
        n.fsm.step_done()
        n._reset_step_state()
        assert n.turn_target is None
    assert seen == [(0, "a"), (0, "b"), (1, "a"), (1, "b")] and n.fsm.state == fsm.DONE


def test_straight_stops_itself_at_the_endpoint_when_the_goal_checker_misses():
    st = CompiledStep("s1", STRAIGHT, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), length_m=2.0)
    n = make_node([st])
    n.fsm.start(True, True)
    c = Client()
    n.goals.send(c, "g", n.fsm.run_id, 0, 0)
    h = Handle()
    c.sent[0][1].set_result(h)
    n.phase = ren.PHASE_GOAL
    n._pose = lambda: (1.95, 0.04, 0.0)  # still short: keep following
    n._execute(n.clock[0])
    assert n.phase == ren.PHASE_GOAL and h.cancels == 0
    n._pose = lambda: (2.03, 0.04, 0.0)  # past the endpoint, 4 cm beside it: the checker's 2.5 cm miss
    n._execute(n.clock[0])
    assert n.phase == ren.PHASE_SETTLE and h.cancels == 1 and n.fsm.state == fsm.EXECUTING
    n._pose = lambda: (2.13, 0.04, 0.0)  # but well past it is still a fault
    n.phase = ren.PHASE_GOAL
    n._execute(n.clock[0])
    assert n.fsm.state == fsm.FAULT and "passed the endpoint" in n.fsm.reason


def test_cross_track_grace_is_independent_of_the_validated_margin():
    """Q05's clamp was dropped 2026-09-17 with the footprint margin at one cell (0.05 m)."""
    st = CompiledStep("s1", STRAIGHT, (0.0, 0.0, 0.0), (3.0, 0.0, 0.0), length_m=3.0)
    n = make_node([st])
    n.fp = SimpleNamespace(margin_m=0.05)
    n.fsm.start(True, True)
    c = Client()
    n.goals.send(c, "g", n.fsm.run_id, 0, 0)
    c.sent[0][1].set_result(Handle())
    n.phase = ren.PHASE_GOAL
    assert n._cross_track_allowed(0.5) == 0.20 and n._cross_track_allowed(2.0) == 0.10
    n._pose = lambda: (0.5, 0.15, 0.0)  # inside the grace band although beyond the validated clearance
    n._execute(n.clock[0])
    assert n.fsm.state != fsm.FAULT
    n._pose = lambda: (2.0, 0.15, 0.0)  # past the grace distance: the route's own limit applies
    n._execute(n.clock[0])
    assert n.fsm.state == fsm.FAULT and "cross-track" in n.fsm.reason


def test_q06_acknowledged_fault_keeps_the_unresolved_goal_barrier_across_a_new_run():
    n = make_node([rotate(1.0)])
    n.fsm.start(True, True)
    c = Client()
    n.goals.send(c, "g", n.fsm.run_id, 0, 0)
    n.phase = ren.PHASE_GOAL
    n.fsm.pause()
    n._interrupt("paused")
    n.fsm.prepare_resume(True, "")
    n._start_edge()
    n.clock[0] += 6.0  # the cancellation bound passes with the goal's acceptance still pending
    n._loc_t = n._panel_t = n.clock[0]
    n._execute(n.clock[0])
    assert n.fsm.state == fsm.FAULT and "not terminated" in n.fsm.reason
    assert n.fsm.ack() if hasattr(n.fsm, "ack") else n.fsm.acknowledge()
    assert n.fsm.load("m2", 1, 1)
    assert "never reported terminal" in (n._prereqs() or "")  # READY cannot become a run
    h = Handle()
    c.sent[0][1].set_result(h)  # the old server finally answers: cancelled, then terminal
    h.result_fut.set_result(Result(5))
    n.wheels(0.0)  # fresh feedback again: only the barrier was in the way
    assert n._prereqs() is None


def test_q19_physical_reset_acknowledges_a_fault_at_rest_and_nothing_else():
    n = make_node([rotate(1.0)])
    n.fsm.start(True, True)
    n._fault("test fault")
    assert n.fsm.state == fsm.FAULT
    n.goals.unresolved.add(ga.Attempt("old", 0, 0, 1))  # a Q06 barrier Reset must not clear
    n.wheels(0.5)  # rolling: ignored
    n._reset_edge()
    assert n.fsm.state == fsm.FAULT
    n.wheels(0.0)
    n._reset_edge()
    assert n.fsm.state == fsm.IDLE and n.goals.barrier()  # acknowledged; barrier intact
    n._reset_edge()  # a repeated edge outside FAULT does nothing
    assert n.fsm.state == fsm.IDLE


# ---- Part B2: the FollowPath goal sits goal_overshoot_m past the endpoint ----


def _clock_stub(n):
    class T:
        def to_msg(self):
            from builtin_interfaces.msg import Time

            return Time()

    class C:
        def now(self):
            return T()

    n.get_clock = lambda: C()


def test_b2_follow_path_ends_goal_overshoot_past_the_endpoint_clamped_to_tolerance():
    samples = [(x / 10, 0.0, 0.0) for x in range(0, 21)]  # 0..2.0 m every 0.1 m
    st = CompiledStep("s1", STRAIGHT, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), length_m=2.0, samples=samples)
    n = make_node([st])
    _clock_stub(n)
    n.goal_overshoot_m = 0.045
    path = n._follow_path(st, 0.0)
    last = path.poses[-1].pose.position
    assert (round(last.x, 3), round(last.y, 3)) == (2.045, 0.0)
    assert round(path.poses[-2].pose.position.x, 3) == 2.0  # the true endpoint is still on the path
    n.goal_overshoot_m = 0.0  # rollback: the endpoint is the goal
    assert round(n._follow_path(st, 0.0).poses[-1].pose.position.x, 3) == 2.0
    n.goal_overshoot_m = 0.3  # clamped to position_tolerance_m (0.05)
    assert round(n._follow_path(st, 0.0).poses[-1].pose.position.x, 3) == 2.05
    # a resume from 1.0 m along keeps the same goal
    n.goal_overshoot_m = 0.045
    p = n._follow_path(st, 1.0)
    assert round(p.poses[0].pose.position.x, 1) == 0.9 and round(p.poses[-1].pose.position.x, 3) == 2.045
    # heading is honoured: a step along +y overshoots in +y
    st2 = CompiledStep(
        "s2",
        STRAIGHT,
        (0.0, 0.0, math.pi / 2),
        (0.0, 1.0, math.pi / 2),
        length_m=1.0,
        samples=[(0.0, y / 10, math.pi / 2) for y in range(0, 11)],
    )
    last = n._follow_path(st2, 0.0).poses[-1].pose.position
    assert (round(last.x, 3), round(last.y, 3)) == (0.0, 1.045)


def test_permit_carries_the_step_speed_and_the_route_turn_cap():
    st = CompiledStep("s1", STRAIGHT, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), length_m=2.0, v_mps=0.40)
    rot = CompiledStep("s2", ROTATE, (2.0, 0.0, 0.0), (2.0, 0.0, 1.0), signed_angle_rad=1.0)
    n = make_node([st, rot])
    _clock_stub(n)
    n.route.limits.linear_mps, n.route.limits.w_mps = 0.40, 0.24
    n.generation, n._lease_instance, n._permit_seq = 3, "sup", 0
    published, speeds = [], []
    n._permit_pub = SimpleNamespace(publish=published.append)
    n._speed_pub = SimpleNamespace(publish=speeds.append)
    n.fsm.start(True, True)
    n.phase = ren.PHASE_GOAL
    n._publish_permit()
    m = published[-1]
    assert m.enabled and (m.v_max, m.w_max) == (pytest.approx(0.40), pytest.approx(0.24))
    # the controller is told the same number, as an absolute limit
    assert speeds[-1].percentage is False and speeds[-1].speed_limit == pytest.approx(0.40)
    # a boosted long straight: the STEP's speed, not the route's base cap ...
    st.v_mps, st.length_m = 0.70, 6.0
    n._publish_permit()
    assert published[-1].v_max == pytest.approx(0.70) and speeds[-1].speed_limit == pytest.approx(0.70)
    # ... until the taper distance before its end - (0.7^2 - 0.4^2) / (2 x 0.4) + 0.7 x 0.3 s =
    # 0.62 m - where it comes back to the base speed so the mux reaches 0.40 at the boundary
    # and the controller's approach ramp starts from 0.40, not 0.70
    assert n._taper_dist(0.70, 0.40) == pytest.approx(0.6225)
    n._pose = lambda: (5.3, 0.0, 0.0)  # 0.7 m to go: still boosted
    n._publish_permit()
    assert published[-1].v_max == pytest.approx(0.70)
    n._pose = lambda: (5.5, 0.0, 0.0)  # 0.5 m to go: tapered
    n._publish_permit()
    assert published[-1].v_max == pytest.approx(0.40) and speeds[-1].speed_limit == pytest.approx(0.40)
    n._pose = lambda: (0.0, 0.0, 0.0)
    n._publish_permit()
    # the obstruction horizon grows with the step speed: 3 s of travel, never under 1.5 m
    assert n._horizon(st) == pytest.approx(2.1)
    st.v_mps = 0.40
    assert n._horizon(st) == pytest.approx(1.5)
    # a rotation: the route's turn cap, the base linear cap (no straight is running)
    n.fsm.step_done()
    n.phase = ren.PHASE_GOAL
    n._publish_permit()
    assert (published[-1].v_max, published[-1].w_max) == (pytest.approx(0.40), pytest.approx(0.24))


def test_reverse_step_progress_path_and_permit():
    st = CompiledStep(
        "s1",
        "reverse",
        (3.0, 0.0, 0.0),
        (1.5, 0.0, 0.0),
        length_m=1.5,
        samples=[(3.0 - 0.1 * k, 0.0, 0.0) for k in range(16)],
        v_mps=0.25,
        reverse=True,
    )
    n = make_node([st])
    _clock_stub(n)
    # progress counts up while backing; a lateral offset is still the cross-track
    assert n._along_cross(st, (3.0, 0.0, 0.0)) == (pytest.approx(0.0), pytest.approx(0.0))
    along, cross = n._along_cross(st, (2.0, 0.05, 0.0))
    assert along == pytest.approx(1.0) and abs(cross) == pytest.approx(0.05)
    # the FollowPath poses run backwards from the current projection and keep the FORWARD
    # heading; the overshoot pose lies past the end in the travel (reverse) direction
    n.goal_overshoot_m = 0.045
    p = n._follow_path(st, 0.95)  # keeps samples from 0.85 m of progress on: x <= 2.15
    xs = [q.pose.position.x for q in p.poses]
    assert xs[0] == pytest.approx(2.1) and xs[-1] == pytest.approx(1.455) and xs == sorted(xs, reverse=True)
    assert all(q.pose.orientation.w == pytest.approx(1.0) for q in p.poses)
    # the permit is a FOLLOW at the reverse step's (half base) speed
    n.generation, n._lease_instance, n._permit_seq = 3, "sup", 0
    published = []
    n._permit_pub = SimpleNamespace(publish=published.append)
    n.fsm.start(True, True)
    n.phase = ren.PHASE_GOAL
    n._publish_permit()
    assert published[-1].source == MotionPermit.FOLLOW and published[-1].v_max == pytest.approx(0.25)


def test_arc_step_progress_cross_track_path_and_envelope():
    from amr_navigation.compiler import arc_pose

    # ccw 90 deg, R 1 from the origin heading +x: centre (0, 1), end (1, 1) heading +y
    centre, r, theta = (0.0, 1.0), 1.0, math.pi / 2
    samples = [arc_pose(centre, r, 0.0, 1.0, theta * k / 30) for k in range(31)]
    st = CompiledStep(
        "s1",
        "arc",
        (0.0, 0.0, 0.0),
        samples[-1],
        length_m=r * theta,
        samples=samples,
        signed_angle_rad=theta,
        v_mps=0.30,
        centre=centre,
        radius_m=r,
    )
    n = make_node([st])
    _clock_stub(n)
    # on the arc: progress = swept angle x R, no cross-track; inside the circle = positive (left)
    assert n._along_cross(st, (0.0, 0.0, 0.0)) == (pytest.approx(0.0), pytest.approx(0.0))
    along, cross = n._along_cross(st, arc_pose(centre, r, 0.0, 1.0, math.pi / 4))
    assert along == pytest.approx(r * math.pi / 4) and cross == pytest.approx(0.0)
    along, cross = n._along_cross(st, (0.0, 0.05, 0.0))  # 5 cm inside at the start
    assert along == pytest.approx(0.0) and cross == pytest.approx(0.05)
    along, _ = n._along_cross(st, (0.9, 1.1, math.pi / 2))  # a little past the end
    assert along > st.length_m
    # a 180 deg arc overrun by 10 deg reads as +10 deg, not -170
    half = CompiledStep(
        "h",
        "arc",
        (0.0, 0.0, 0.0),
        arc_pose(centre, r, 0.0, 1.0, math.pi),
        length_m=math.pi,
        signed_angle_rad=math.pi,
        centre=centre,
        radius_m=r,
    )
    along, _ = n._along_cross(half, arc_pose(centre, r, 0.0, 1.0, math.pi + math.radians(10)))
    assert along == pytest.approx(r * (math.pi + math.radians(10)))
    # resume from a quarter in: the path keeps the tangent headings and ends past the end
    # along the END tangent (+y)
    n.goal_overshoot_m = 0.045
    p = n._follow_path(st, r * math.pi / 4)
    first, last = p.poses[0].pose, p.poses[-1].pose
    assert math.hypot(first.position.x - centre[0], first.position.y - centre[1]) == pytest.approx(
        r, abs=1e-6
    )
    assert (last.position.x, last.position.y) == (pytest.approx(1.0), pytest.approx(1.045))
    yaws = [2 * math.atan2(q.pose.orientation.z, q.pose.orientation.w) for q in p.poses]
    assert yaws == sorted(yaws) and yaws[0] > 0.6  # from ~45 deg up to 90 deg
    # the permit is a FOLLOW at the arc's capped speed with the ARC turn ceiling (2026-09-19):
    # v/R may reach 0.40 rad/s on the 1 m minimum radius, above the 0.24 spin cap here
    n.generation, n._lease_instance, n._permit_seq = 3, "sup", 0
    published = []
    n._permit_pub = SimpleNamespace(publish=published.append)
    n.fsm.start(True, True)
    n.phase = ren.PHASE_GOAL
    n._publish_permit()
    m = published[-1]
    assert m.source == MotionPermit.FOLLOW and (m.v_max, m.w_max) == (
        pytest.approx(0.30),
        pytest.approx(ren.VEHICLE_ARC_W_MAX),
    )


def test_chained_straight_and_arc_run_as_one_goal_with_a_speed_taper():
    from amr_navigation.compiler import arc_pose

    # 3 m straight at 0.5, then a left 90 deg arc of R 1 at 0.3, then a rotate (ends the chain)
    s1 = CompiledStep(
        "s1",
        STRAIGHT,
        (0.0, 0.0, 0.0),
        (3.0, 0.0, 0.0),
        length_m=3.0,
        samples=[(0.1 * k, 0.0, 0.0) for k in range(31)],
        v_mps=0.5,
    )
    centre, theta = (3.0, 1.0), math.pi / 2
    arc_samples = [arc_pose(centre, 1.0, 0.0, 1.0, theta * k / 30) for k in range(31)]
    s2 = CompiledStep(
        "s2",
        "arc",
        (3.0, 0.0, 0.0),
        arc_samples[-1],
        length_m=theta,
        samples=arc_samples,
        signed_angle_rad=theta,
        v_mps=0.3,
        centre=centre,
        radius_m=1.0,
    )
    s3 = CompiledStep("s3", ROTATE, s2.end, s2.end, signed_angle_rad=1.0)
    n = make_node([s1, s2, s3])
    _clock_stub(n)
    n.route.limits.linear_mps = 0.5
    n.goal_overshoot_m = 0.045
    assert [s.id for s in n._chain(0)] == ["s1", "s2"] and [s.id for s in n._chain(2)] == ["s3"]
    n.fsm.start(True, True)  # step 0 is current
    # one path for both steps: the straight's samples, then the arc's, ending past the arc's end
    p = n._follow_path(s1, 0.0)
    xs = [q.pose.position.x for q in p.poses]
    assert len(p.poses) == 31 + 30 + 1 and xs[30] == pytest.approx(3.0)
    last = p.poses[-1].pose.position
    assert (last.x, last.y) == (pytest.approx(4.0), pytest.approx(1.0 + 0.045))
    # the straight tapers to the arc's speed before the boundary: (0.5^2 - 0.3^2)/0.8 + 0.15 = 0.35 m
    assert n._taper_dist(0.5, 0.3) == pytest.approx(0.35)
    assert n._step_speed(s1, (2.5, 0.0, 0.0)) == 0.5 and n._step_speed(s1, (2.7, 0.0, 0.0)) == 0.3
    # crossing the boundary advances the step without a settle: still EXECUTING in the goal phase
    n.phase = ren.PHASE_GOAL
    n.goals.current = object()
    n.goals.result = None
    n.goals.handle = object()
    n._pose = lambda: (3.02, 0.01, 0.0)
    n._execute(n.clock[0])
    assert n.fsm.state == fsm.EXECUTING and n.fsm.step_index == 1 and n.phase == ren.PHASE_GOAL
    assert n.goals.current is not None  # the FollowPath goal was not revoked
    # the envelope ahead spans the boundary: near the end of the straight it includes arc cells
    import numpy as np
    from amr_maps.grid import Grid, GridMeta

    from amr_navigation import footprint as fpmod

    n.grid = Grid(np.zeros((160, 160), dtype=np.int8), GridMeta(0.05, -2.0, -2.0))
    n.fp = fpmod.Footprint(polygon=[[-0.3, -0.2], [0.5, -0.2], [0.5, 0.2], [-0.3, 0.2]], margin_m=0.0)
    n.fsm.step_index = 0
    mask = n._envelope(s1, 2.8)
    r_, c_ = n.grid.world_to_cell(*arc_samples[15][:2])  # the middle of the arc, ~0.8 m past the boundary
    assert mask[r_, c_]


# ---- dynamic-mapping plan §1.2/§1.3: a mapped trolley in a dynamic area is checked live ------


def _trolley_world():
    import numpy as np  # noqa: PLC0415
    from amr_maps.grid import Grid, GridMeta  # noqa: PLC0415

    # 10 m x 4 m at 0.05 m, origin (-1, -2); a trolley in the map at x 3.0..3.25, |y| <= 0.3
    data = np.zeros((80, 200), dtype=np.int8)
    g = Grid(data, GridMeta(0.05, -1.0, -2.0))
    r0, c0 = g.world_to_cell(3.0, -0.3)
    r1, c1 = g.world_to_cell(3.24, 0.3)
    data[r0 : r1 + 1, c0 : c1 + 1] = 100
    dyn = np.zeros(data.shape, dtype=bool)
    ra, ca = g.world_to_cell(2.5, -1.0)
    rb, cb = g.world_to_cell(4.0, 1.0)
    dyn[ra:rb, ca:cb] = True
    return g, dyn


def test_explained_by_map_excludes_dynamic_areas():
    g, dyn = _trolley_world()
    plain = ren.explained_by_map(g, None, 0.15)
    marked = ren.explained_by_map(g, dyn, 0.15)
    face = g.world_to_cell(3.0, 0.0)
    assert plain[face] and not marked[face]  # the trolley face explains a return only without the mark
    assert plain[g.world_to_cell(2.9, 0.0)]  # the tolerance grows the explanation by 0.15 m
    assert (marked == (plain & ~dyn)).all()


def test_obstruction_blocks_on_a_mapped_trolley_only_inside_a_dynamic_area():
    import numpy as np  # noqa: PLC0415

    from amr_navigation import footprint as fpmod  # noqa: PLC0415

    g, dyn = _trolley_world()
    st = CompiledStep(
        "s1", STRAIGHT, (0.0, 0.0, 0.0), (6.0, 0.0, 0.0), length_m=6.0, samples=[(0.0, 0.0, 0.0)], v_mps=0.5
    )
    n = make_node([st])
    n.grid, n.fp = g, fpmod.Footprint(((-0.5, -0.35), (1.1, -0.35), (1.1, 0.35), (-0.5, 0.35)), 0.05)
    del n._obstruction  # the real method, not make_node's stub
    # the laser at (2, 0) facing +x sees the trolley face 1 m ahead on every beam
    ang = np.linspace(-0.25, 0.25, 51)
    n._scan = SimpleNamespace(
        header=SimpleNamespace(frame_id="laser", stamp=None),
        ranges=list(1.0 / np.cos(ang)),
        angle_min=float(ang[0]),
        angle_increment=float(ang[1] - ang[0]),
        range_min=0.05,
        range_max=30.0,
    )
    tf = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=2.0, y=0.0), rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    )
    n.tf_buffer = SimpleNamespace(lookup_transform=lambda *_a: tf)
    pose = (2.0, 0.0, 0.0)
    n._map_near = ren.explained_by_map(g, None, 0.15)
    assert n._obstruction(st, pose) is None  # the map explains the trolley: driven into (old behaviour)
    n._map_near = ren.explained_by_map(g, dyn, 0.15)
    n.clear_since = None
    assert n._obstruction(st, pose) is None  # one scan: waits for obstacle_persist_scans
    n._scan_t += 0.1  # the next scan
    reason = n._obstruction(st, pose)
    assert reason is not None and "inside the straight envelope" in reason and n.clear_since is None


def test_speed_boost_2_taper_horizon_and_arc_turn_cap_on_the_lead_in_straight():
    """2026-09-19: 0.85 boost / 0.55 base / 0.40 arcs; 2 s look-ahead; an arc's chain turns at 0.45."""
    from amr_navigation.compiler import arc_pose

    assert ren.VEHICLE_ARC_W_MAX == 0.45
    s1 = CompiledStep(
        "s1",
        STRAIGHT,
        (0.0, 0.0, 0.0),
        (6.0, 0.0, 0.0),
        length_m=6.0,
        samples=[(0.1 * k, 0.0, 0.0) for k in range(61)],
        v_mps=0.85,
    )
    centre, theta = (6.0, 1.0), math.pi / 2
    arc_samples = [arc_pose(centre, 1.0, 0.0, 1.0, theta * k / 30) for k in range(31)]
    s2 = CompiledStep(
        "s2",
        "arc",
        (6.0, 0.0, 0.0),
        arc_samples[-1],
        length_m=theta,
        samples=arc_samples,
        signed_angle_rad=theta,
        v_mps=0.40,
        centre=centre,
        radius_m=1.0,
    )
    s3 = CompiledStep("s3", ROTATE, s2.end, s2.end, signed_angle_rad=1.0)
    n = make_node([s1, s2, s3])
    _clock_stub(n)
    n.route.limits.linear_mps, n.route.limits.w_mps = 0.55, 0.37
    n.stopping_time_s = 2.0  # the node default since 2026-09-19
    n.fsm.start(True, True)
    # the ramp-down into the arc starts (0.85^2 - 0.40^2) / 0.8 + 0.85 x 0.3 = 0.96 m before it
    assert n._taper_dist(0.85, 0.40) == pytest.approx(0.958, abs=1e-3)
    assert n._taper_dist(0.55, 0.40) == pytest.approx(0.343, abs=1e-3)
    assert n._step_speed(s1, (4.9, 0.0, 0.0)) == 0.85 and n._step_speed(s1, (5.1, 0.0, 0.0)) == 0.40
    # look-ahead: 2 s of travel at 0.85 = 1.7 m; the 1.5 m floor at 0.55
    assert n._horizon(s1) == pytest.approx(1.7)
    assert n._horizon(CompiledStep("x", STRAIGHT, (0, 0, 0), (1, 0, 0), length_m=1.0, v_mps=0.55)) == 1.5
    # the straight leading into the arc already carries the arc's turn ceiling (RPP curves early)
    assert n._step_w_max(s1) == pytest.approx(0.45)
    n.fsm.step_done()
    assert n._step_w_max(s2) == pytest.approx(0.45)
    n.fsm.step_done()
    assert n._step_w_max(s3) == pytest.approx(0.37)  # a spin keeps the spin cap
    # a boosted straight with no arc after it keeps the spin cap, and ends at the base speed
    t1 = CompiledStep("t1", STRAIGHT, (0, 0, 0), (6.0, 0, 0), length_m=6.0, v_mps=0.85)
    t2 = CompiledStep("t2", ROTATE, (6.0, 0, 0), (6.0, 0, 1.0), signed_angle_rad=1.0)
    m = make_node([t1, t2])
    _clock_stub(m)
    m.route.limits.linear_mps, m.route.limits.w_mps = 0.55, 0.37
    m.fsm.start(True, True)
    assert m._step_w_max(t1) == pytest.approx(0.37)
    assert m._step_speed(t1, (5.5, 0.0, 0.0)) == 0.55  # tapered: RPP's approach starts from the base
