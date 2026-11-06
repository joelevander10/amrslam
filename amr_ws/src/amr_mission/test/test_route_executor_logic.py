"""R07/R18/R08 wiring in the route executor, without a ROS graph: the node object is
built with __new__ and only the state these methods read is set."""

import math
import threading
from types import SimpleNamespace

import pytest
from amr_navigation.compiler import ROTATE, STRAIGHT, CompiledStep
from test_goal_attempts import Client, Handle, Result

from amr_interfaces.msg import DriveStatus, LocalizationState, MotionPermit, PanelState, WheelStates
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
    n.taper_decel, n.taper_lead_s, n.taper_jerk = 0.4, 0.3, 1.0  # the pinned tests below; see test_taper_*
    n.goals = ga.GoalAttempts(n._lock, n._now, n._on_goal_error)
    n.start_gate_m, n.start_gate_rad = 0.1, math.radians(5)
    n.w_eps, n.wheels_age, n.panel_age, n.loc_age = 0.02, 0.1, 0.2, 1.5
    n.scan_age, n._scan_t = float("inf"), 100.0  # freshness is exercised where a test sets scan_age
    n.centre_drift_m, n.turn_tol, n.wrong_way = 0.05, math.radians(2), math.radians(5)
    n.entry_corr_max = math.radians(10)
    # 0 = the grace period off, so the classic tests still read "this condition is a FAULT";
    # the debounce itself has its own tests (test_prereq_*).
    n.prereq_grace_s = 0.0
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
    n.auto_resume_enabled, n.auto_clear_s, n.auto_resume_estop = True, 2.0, False
    n.safety_window, n.drives_age, n.field_index = 1.0, 0.5, 0
    n.blocked_clear_m = 1.0
    n.resume_corridor_m = 0.30
    n.costmap_clear_after_s, n._costmap_cleared_t = 10.0, None
    # No warning by default (2026-10-08): these tests are about what follows it. The
    # pre-move phase still needs a fresh drive report with torque - see drives().
    n.premove_s, n.premove_timeout_s, n._premove_since = 0.0, 8.0, None
    n._init_hold_state()
    n._reset_step_state()
    n.wheels(0.0, valid=True)
    return n


def _wheels(self, vel, valid=True):
    m = WheelStates()
    m.left_valid = m.right_valid = valid
    m.left_vel_rad_s = m.right_vel_rad_s = vel
    self._on_wheels(m)


ren.RouteExecutor.wheels = _wheels


def _drives(self, torque=True):
    m = DriveStatus()
    m.operational = torque
    m.left_state = m.right_state = "Operation enabled" if torque else "Ready to switch on"
    self._on_drives(m)


ren.RouteExecutor.drives = _drives


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
    assert n.fsm.state == fsm.EXECUTING and n.phase == ren.PHASE_PREMOVE
    n.drives(True)
    n._execute(n.clock[0])  # the warning ends: the drives have torque
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
    n.drives(True)
    n._execute(n.clock[0])  # the pre-move warning ends
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
    # ... until the taper distance before its end - a jerk-limited 0.70 -> 0.40 (2026-11-03):
    # (0.3/0.4 + 0.4/1.0) s x 0.55 m/s + 0.7 x 0.3 s = 0.84 m - where it comes back to the base
    # speed so the mux reaches 0.40 at the boundary and the approach ramp starts from 0.40
    assert n._taper_dist(0.70, 0.40) == pytest.approx(0.8425)
    n._pose = lambda: (5.1, 0.0, 0.0)  # 0.9 m to go: still boosted
    n._publish_permit()
    assert published[-1].v_max == pytest.approx(0.70)
    n._pose = lambda: (5.3, 0.0, 0.0)  # 0.7 m to go: tapered
    n._publish_permit()
    assert published[-1].v_max == pytest.approx(0.40) and speeds[-1].speed_limit == pytest.approx(0.40)
    n._pose = lambda: (0.0, 0.0, 0.0)
    n._publish_permit()
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


def test_arc_step_progress_cross_track_and_path():
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
    # the straight tapers to the arc's speed before the boundary, for the mux's jerk-limited
    # slowdown (2026-11-03): (0.2/0.4 + 0.4/1.0) s x 0.40 m/s mean + 0.5 x 0.3 s = 0.51 m
    assert n._taper_dist(0.5, 0.3) == pytest.approx(0.51)
    assert n._step_speed(s1, (2.4, 0.0, 0.0)) == 0.5 and n._step_speed(s1, (2.6, 0.0, 0.0)) == 0.3
    # crossing the boundary advances the step without a settle: still EXECUTING in the goal phase
    n.phase = ren.PHASE_GOAL
    n.goals.current = object()
    n.goals.result = None
    n.goals.handle = object()
    n._pose = lambda: (3.02, 0.01, 0.0)
    n._execute(n.clock[0])
    assert n.fsm.state == fsm.EXECUTING and n.fsm.step_index == 1 and n.phase == ren.PHASE_GOAL
    assert n.goals.current is not None  # the FollowPath goal was not revoked


def test_speed_boost_2_taper_and_arc_turn_cap_on_the_lead_in_straight():
    """2026-09-19: 0.85 boost / 0.55 base / 0.40 arcs; an arc's chain turns at 0.45."""
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
    n.fsm.start(True, True)
    # the jerk-limited ramp-down into the arc starts (0.45/0.4 + 0.4) s x 0.625 m/s + 0.85 x 0.3
    # = 1.21 m before it; a small step (dv 0.15 <= d^2/j) is triangular: 2 sqrt(0.15) s
    assert n._taper_dist(0.85, 0.40) == pytest.approx(1.208, abs=1e-3)
    assert n._taper_dist(0.55, 0.40) == pytest.approx(0.533, abs=1e-3)
    assert n._step_speed(s1, (4.7, 0.0, 0.0)) == 0.85 and n._step_speed(s1, (5.1, 0.0, 0.0)) == 0.40
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


def test_taper_into_an_arc_starts_early_enough_with_the_shipped_slopes():
    """Owner 2026-11-03: straights 0.60, arcs 0.30, slopes twice as long as the first
    smoothing (mux a_max 0.1 / d_max 0.125 / auto_jerk 0.1). The shipped executor defaults
    (taper_decel 0.125 = mux d_max, taper_jerk 0.1 = mux auto_jerk) put the start 1.82 m
    before the arc; the mux S-curve with the same numbers needs 1.59 m."""
    from amr_base.diff_drive import scurve

    n = make_node([CompiledStep("s1", STRAIGHT, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0), length_m=5.0)])
    n.taper_decel, n.taper_lead_s, n.taper_jerk = 0.125, 0.3, 0.1
    assert n._taper_dist(0.60, 0.30) == pytest.approx(1.82, abs=0.005)
    v, a, x = 0.60, 0.0, 0.0
    while v > 0.30:
        v, a = scurve(v, a, 0.30, 0.1, 0.125, 0.1, 0.02)
        x += v * 0.02
    assert 1.4 < x < 1.82 - 0.6 * 0.3, "reaches 0.30 before the arc, with the lead to spare"


def test_home_leg_progress_chain_overshoot_and_turn_cap():
    from amr_navigation.scurve import home_leg

    # the route ends 2 m behind HOME (the origin, facing +x), 0.2 m left and turned 5 deg left;
    # a straight leads into the S-curve home and the two run as one goal
    yaw0 = math.radians(5.0)
    p0 = (-2.0 - 2.0 * math.cos(yaw0), 0.2 - 2.0 * math.sin(yaw0), yaw0)
    p1 = (-2.0, 0.2, yaw0)
    c, s = math.cos(yaw0), math.sin(yaw0)
    line = [(p0[0] + 0.1 * k * c, p0[1] + 0.1 * k * s, yaw0) for k in range(21)]
    s1 = CompiledStep("s1", STRAIGHT, p0, p1, length_m=2.0, samples=line, v_mps=0.5)
    leg = home_leg(p1, (0.0, 0.0, 0.0))
    home = (0.0, 0.0, 0.0)
    s2 = CompiledStep("s2", "home", p1, home, length_m=leg.length_m, samples=leg.samples, v_mps=0.30, leg=leg)
    n = make_node([s1, s2])
    _clock_stub(n)
    n.goal_overshoot_m = 0.045
    assert [s.id for s in n._chain(0)] == ["s1", "s2"]
    # progress and cross-track on the curve come from its samples
    mid = leg.samples[len(leg.samples) // 2]
    along, cross = n._along_cross(s2, mid)
    assert 0.3 * leg.length_m < along < 0.7 * leg.length_m and cross == pytest.approx(0.0, abs=1e-6)
    assert n._along_cross(s2, (0.1, 0.0, 0.0))[0] == pytest.approx(leg.length_m + 0.1)
    # one path for both; the extra pose sits past HOME along HOME's heading (+x), not the
    # 5 deg the leg started on
    n.fsm.start(True, True)
    p = n._follow_path(s1, 0.0)
    last = p.poses[-1].pose.position
    assert (last.x, last.y) == (pytest.approx(0.045), pytest.approx(0.0, abs=1e-9))
    # a curved home leg in the chain lifts the turn cap to the arc ceiling, as an arc does
    assert n._step_w_max(s1) == pytest.approx(max(0.24, ren.VEHICLE_ARC_W_MAX))


def _line(sid, p0, p1, typ=STRAIGHT):
    n = max(1, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / 0.05))
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    pts = [(p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n, yaw) for k in range(n + 1)]
    return CompiledStep(
        sid,
        typ,
        (p0[0], p0[1], yaw),
        (p1[0], p1[1], yaw),
        length_m=math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
        samples=pts,
        v_mps=0.5,
    )


def test_a_route_that_closes_on_its_start_is_not_one_followpath_goal():
    """2026-11-06: every step chained and the last one (Go home) ending on the route start -
    Nav2's stateful goal checker latched the goal at the start and finished the chain 1.4 m
    into step 0: "endpoint missed: 1.365 m / 1.5 deg". The closing step is its own goal."""
    corners = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]
    steps = [_line(f"s{k}", corners[k], corners[k + 1]) for k in range(4)]
    steps[-1].type = "home"
    assert ren.chain_breaks(steps) == {3}
    n = make_node(steps)
    assert [s.id for s in n._chain(0)] == ["s0", "s1", "s2"]
    assert [s.id for s in n._chain(1)] == ["s1", "s2"]  # mid-chain bookkeeping agrees
    assert [s.id for s in n._chain(3)] == ["s3"]
    # the goal of the first chain ends at step 2: nothing chained after it
    n.fsm.start(True, True)
    for _ in range(2):
        n.fsm.step_done()
    assert n._next_in_chain() is None


def test_an_open_route_and_a_short_approach_stay_one_chain():
    out = [
        _line("a", (0.0, 0.0), (3.0, 0.0)),
        _line("b", (3.0, 0.0), (3.0, 3.0)),
        _line("c", (3.0, 3.0), (6.0, 3.0)),
    ]
    assert ren.chain_breaks(out) == set()
    # coming back to within the guard only over its own last 0.9 m is a normal approach
    hook = [_line("a", (0.0, 0.0), (2.0, 0.0)), _line("b", (2.0, 0.0), (2.0, 0.2))]
    assert ren.chain_breaks(hook) == set()
    # a rotate already ends a chain, so a loop broken by one needs nothing more
    turn = CompiledStep("r", ROTATE, (4.0, 0.0, 0.0), (4.0, 0.0, math.pi / 2), signed_angle_rad=math.pi / 2)
    loop = [
        _line("a", (0.0, 0.0), (4.0, 0.0)),
        turn,
        _line("b", (4.0, 0.0), (4.0, 3.0)),
        _line("c", (4.0, 3.0), (0.0, 3.0)),
        _line("d", (0.0, 3.0), (0.0, 0.0)),
    ]
    assert ren.chain_breaks(loop) == set()  # b..d never returns near d's end (0, 0)


def test_a_slow_zone_slows_before_it_and_speeds_up_after_it_without_a_stop():
    s1, s2, s3 = (
        _line("s1", (0.0, 0.0), (6.0, 0.0)),
        _line("s2", (6.0, 0.0), (8.0, 0.0)),
        _line("s3", (8.0, 0.0), (14.0, 0.0)),
    )
    s1.v_mps, s2.v_mps, s3.v_mps = 0.5, 0.3, 0.5
    n = make_node([s1, s2, s3])
    assert [s.id for s in n._chain(0)] == ["s1", "s2", "s3"]  # one move, no stop
    n.fsm.start(True, True)
    assert n._step_speed(s1, (1.0, 0.0, 0.0)) == 0.5  # far from the zone
    assert n._step_speed(s1, (5.5, 0.0, 0.0)) == 0.3  # tapered down before it
    n.fsm.step_done()
    assert n._step_speed(s2, (7.0, 0.0, 0.0)) == 0.3  # in it
    n.fsm.step_done()
    assert n._step_speed(s3, (8.1, 0.0, 0.0)) == 0.5  # back up as soon as it is left


def test_the_end_of_a_move_is_a_constant_deceleration_stop_when_end_decel_is_set():
    """AMR QR 2026-11-06: the executor brakes for the end of a chain itself (RPP's ramp only
    works inside the local costmap)."""
    s1, s2 = _line("s1", (0.0, 0.0), (6.0, 0.0)), _line("s2", (6.0, 0.0), (8.0, 0.0))
    s1.v_mps = s2.v_mps = 0.8
    n = make_node([s1, s2])
    n.fsm.start(True, True)
    assert n._step_speed(s1, (5.9, 0.0, 0.0)) == 0.8  # off by default: RPP's ramp does it
    n.end_decel, n.end_min_v = 0.2, 0.05
    assert n._step_speed(s1, (1.0, 0.0, 0.0)) == 0.8  # 7 m to go
    assert n._step_speed(s1, (5.9, 0.0, 0.0)) == 0.8  # 2.1 m to go: sqrt(2 x 0.2 x 2.1) > 0.8
    n.fsm.step_done()
    # 1.0 m before the end of s2 (the chain's end): sqrt(2 x 0.2 x 1.0)
    assert n._step_speed(s2, (7.0, 0.0, 0.0)) == pytest.approx(math.sqrt(0.4))
    assert n._step_speed(s2, (8.0, 0.0, 0.0)) == 0.05  # never below the crawl


def test_a_slow_zone_two_steps_ahead_is_tapered_for_in_time():
    s1, s2, s3 = (
        _line("s1", (0.0, 0.0), (6.0, 0.0)),
        _line("s2", (6.0, 0.0), (6.2, 0.0)),
        _line("s3", (6.2, 0.0), (9.0, 0.0)),
    )
    s1.v_mps, s2.v_mps, s3.v_mps = 0.5, 0.5, 0.2
    n = make_node([s1, s2, s3])
    n.fsm.start(True, True)
    # 0.2 m of normal straight is shorter than the 0.5 -> 0.2 taper (0.55 m): it starts on s1,
    # 0.4 m before the slow zone - looking only at the next step it would never start
    assert n._taper_dist(0.5, 0.2) == pytest.approx(0.5525)
    assert n._step_speed(s1, (5.0, 0.0, 0.0)) == 0.5
    assert n._step_speed(s1, (5.8, 0.0, 0.0)) == 0.2
