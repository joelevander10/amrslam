"""Auto-resume after a stop whose cause clears by itself (coding-plan-auto-resume.md, 2026-09-19).

The executor node is built with __new__ like test_route_executor_logic; time, wheels, the drive
report and the nanoScan3 output paths are fed by hand, and _tick runs the real hold logic."""

from types import SimpleNamespace

import pytest
from amr_navigation.compiler import STRAIGHT, CompiledStep
from test_route_executor_logic import _clock_stub, make_node

from amr_interfaces.msg import DriveStatus, LocalizationState, RunState
from amr_mission import goal_attempts as ga
from amr_mission import route_executor_node as ren
from amr_mission import run_fsm as fsm


def node(start=True):
    st = CompiledStep(
        "s1",
        STRAIGHT,
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        length_m=4.0,
        samples=[(0.1 * k, 0.0, 0.0) for k in range(41)],
        v_mps=0.55,
    )
    n = make_node([st])
    _clock_stub(n)
    n.generation, n._lease_instance, n._permit_seq = 1, "sup", 0
    n._permit_pub = SimpleNamespace(publish=lambda _m: None)
    n._pose = lambda: (1.0, 0.0, 0.0)
    n.goal_overshoot_m = 0.045
    n.sent = []

    def send(st, pose):  # a FollowPath goal "accepted": the executor then tracks it
        n.sent.append(st.id)
        n.goals.current, n.goals.result, n.goals.handle = object(), None, object()
        return True

    n._send_follow = send
    if not start:
        return n  # left READY: a mission loaded, waiting for AUTO + Start
    n.fsm.start(True, True)
    send(n.compiled.steps[0], None)
    n.phase = ren.PHASE_GOAL
    return n


def tick(n, dt=0.1, wheels=True, torque=None, field=None):
    """Advance the clock and run one executor tick. torque: True/False feeds a drive report;
    field: True/False feeds an /output_paths sample (status[0])."""
    n.clock[0] += dt
    n._panel_t = n._loc_t = n.clock[0]
    if wheels:
        n.wheels(0.0, valid=True)
    if torque is not None:
        m = DriveStatus()
        m.operational = torque
        m.left_state = m.right_state = "Operation enabled" if torque else "Switch on disabled"
        n._on_drives(m)
    if field is not None:
        n._on_output_paths(SimpleNamespace(status=[field]))
    n._tick()


def run_for(n, seconds, **kw):
    for _ in range(round(seconds / 0.1)):
        tick(n, **kw)


def test_lidar_stop_holds_then_resumes_by_itself_after_two_clear_seconds():
    n = node()
    tick(n, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING
    # the protective field trips, the FX3 drops HWTO: the drives report torque off
    tick(n, field=False)
    tick(n, torque=False, wheels=False)
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "field"
    assert "protective field" in n.fsm.reason
    # no wheel feedback while the torque is off: held, never a fault
    run_for(n, 5.0, wheels=False, torque=False, field=False)
    assert n.fsm.state == fsm.BLOCKED and "waiting" in n.fsm.reason
    # field clear, drives re-armed: waits auto_resume_clear_s, then continues without Start
    tick(n, field=True, torque=True)
    run_for(n, 1.5, torque=True, field=True)
    assert n.fsm.state == fsm.BLOCKED
    run_for(n, 0.8, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING and n.hold_cause == ""
    assert n.fsm.step_index == 0 and n.sent == ["s1", "s1"]  # the same step, a new goal from here


def test_resume_waits_while_the_field_is_violated_or_the_vehicle_is_off_the_segment():
    n = node()
    tick(n, field=False)
    tick(n, torque=False, wheels=False)
    assert n.hold_cause == "field"
    run_for(n, 3.0, torque=True, field=False)  # drives back, field still violated
    assert n.fsm.state == fsm.BLOCKED and "protective field not clear" in n.fsm.reason
    n._pose = lambda: (1.0, 0.5, 0.0)  # pushed 0.5 m sideways during the stop
    run_for(n, 3.0, torque=True, field=True)
    assert n.fsm.state == fsm.BLOCKED and "corridor" in n.fsm.reason


def test_e_stop_button_waits_for_start_without_a_resume_click():
    n = node()
    tick(n, torque=True, field=True)
    tick(n, torque=False, wheels=False)  # torque gone, field clear: the button
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "estop"
    run_for(n, 5.0, torque=True, field=True)
    assert n.fsm.state == fsm.BLOCKED  # never by itself (auto_resume_estop false)
    n._start_edge()
    assert n.fsm.state == fsm.EXECUTING and n.hold_cause == ""


def test_e_stop_resumes_by_itself_only_when_configured():
    n = node()
    n.auto_resume_estop = True
    tick(n, torque=False, wheels=False, field=True)
    assert n.hold_cause == "estop"
    run_for(n, 2.5, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING


def test_wheel_loss_first_is_held_then_classified_by_the_drive_report():
    n = node()
    n.clock[0] += 0.3  # wheel feedback stops before the drive report says why
    n._panel_t = n._loc_t = n.clock[0]
    n._tick()
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "pending"  # stopped at once
    tick(n, wheels=False, torque=False, field=False)
    assert n.hold_cause == "field"
    # without a drive report within the window it was a real feedback loss: FAULT
    n = node()
    n.clock[0] += 0.3
    n._panel_t = n._loc_t = n.clock[0]
    n._tick()
    assert n.hold_cause == "pending"
    run_for(n, 1.2, wheels=False, torque=True)
    assert n.fsm.state == fsm.FAULT and "without a safety stop" in n.fsm.reason


def test_manual_during_a_safety_hold_aborts_the_run():
    n = node()
    tick(n, field=False)
    tick(n, torque=False, wheels=False)
    n._panel.mode_auto = False
    tick(n, wheels=False, torque=False)
    assert n.fsm.state == fsm.IDLE and "manual" in n.fsm.reason


def test_controller_abort_is_retried_three_times_then_faults():
    n = node()
    tick(n, torque=True, field=True)
    for attempt in range(4):
        n.phase = ren.PHASE_GOAL
        n.goals.current, n.goals.result = object(), ga.ABORTED
        tick(n, torque=True, field=True)
        if attempt < 3:
            assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "controller", attempt
            run_for(n, 2.2, torque=True, field=True)
            assert n.fsm.state == fsm.EXECUTING, attempt
    assert n.fsm.state == fsm.FAULT and "aborted" in n.fsm.reason


def test_abort_during_a_safety_stop_is_a_hold_not_a_retry():
    n = node()
    tick(n, torque=True, field=False)  # field violated, the drive report not in yet
    n.goals.current, n.goals.result = object(), ga.ABORTED
    tick(n, torque=True, field=False)
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "field" and n._aborts == 0


def test_controller_hold_resumes_by_itself_and_can_be_switched_off():
    n = node()
    n.goals.current, n.goals.result = object(), ga.ABORTED
    tick(n, torque=True, field=True)
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "controller"
    run_for(n, 2.2, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING and n._aborts == 1
    m = node()
    m.auto_resume_enabled = False
    m.goals.current, m.goals.result = object(), ga.ABORTED
    tick(m, torque=True, field=True)
    run_for(m, 5.0, torque=True, field=True)
    assert m.fsm.state == fsm.FAULT  # no auto-resume: an aborted goal is a fault


def test_run_state_carries_the_hold_cause():
    n = node()
    published = []
    n._state_pub = SimpleNamespace(publish=published.append)
    n.manifest = n.route_obj = None
    n.route = SimpleNamespace(**vars(n.route), route_id="r", revision=1)
    tick(n, field=False)
    tick(n, torque=False, wheels=False)
    n._publish_state()
    m: RunState = published[-1]
    assert m.state == RunState.BLOCKED and m.hold_cause == "field" and m.auto_resume
    n._on_drives(DriveStatus(operational=False, left_state="Switch on disabled", right_state="x"))
    assert n._torque_off(n.clock[0])
    # a feedback-only loss with a drive still enabled is not "torque off"
    n._on_drives(DriveStatus(operational=False, left_state="Operation enabled", right_state="x"))
    assert not n._torque_off(n.clock[0])
    assert pytest.approx(n.safety_window) == 1.0


def test_a_safety_stop_during_another_hold_or_a_pause_is_not_a_fault():
    n = node()
    n.goals.current, n.goals.result = object(), ga.ABORTED
    tick(n, torque=True, field=True)
    assert n.hold_cause == "controller"
    tick(n, field=False)  # someone walks on into the protective field
    tick(n, torque=False, wheels=False, field=False)
    run_for(n, 3.0, torque=False, wheels=False, field=False)
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "field"
    n.goals.result = None
    run_for(n, 2.5, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING
    # an operator Pause, then the E-stop: still PAUSED (Resume + Start), never a fault
    m = node()
    m.fsm.pause()
    run_for(m, 3.0, torque=False, wheels=False, field=True)
    assert m.fsm.state == fsm.PAUSED


def test_the_hold_cause_ends_with_the_hold():
    n = node()
    tick(n, field=False)
    tick(n, torque=False, wheels=False)
    assert n.hold_cause == "field"
    n.fsm.abort("aborted by operator")
    tick(n, torque=True, field=True)
    assert n.hold_cause == ""


# ---- prerequisite debounce (operator decision 2026-09-20) ---------------------------------
# A prerequisite must fail CONTINUOUSLY for prereq_grace_s before it costs the run. The
# localisation state is the lever here because it fails without the clock, which tick()
# advances on its own. NOT_READY is a literal rather than LocalizationState.LOST so the test
# means the same thing under the laptop's stubbed messages, where every constant is 0.
NOT_READY = 99


def test_a_prereq_lapse_shorter_than_the_grace_period_is_not_a_fault():
    n = node()
    n.prereq_grace_s = 0.5
    n._loc.state = NOT_READY
    run_for(n, 0.3, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING  # inside the grace period: the run carries on
    n._loc.state = LocalizationState.READY
    run_for(n, 1.0, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING  # recovered: the timer was cleared, never faulted


def test_a_prereq_that_stays_broken_still_faults():
    n = node()
    n.prereq_grace_s = 0.5
    n._loc.state = NOT_READY
    run_for(n, 0.3, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING
    run_for(n, 0.4, torque=True, field=True)
    assert n.fsm.state == fsm.FAULT and "localisation not READY" in n.fsm.reason


def test_the_grace_period_does_not_survive_a_recovery_in_between():
    """Two 0.3 s lapses with a good tick between them are not one 0.6 s failure."""
    n = node()
    n.prereq_grace_s = 0.5
    for _ in range(4):
        n._loc.state = NOT_READY
        run_for(n, 0.3, torque=True, field=True)
        n._loc.state = LocalizationState.READY
        tick(n, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING


def test_a_momentary_lapse_does_not_unready_a_loaded_mission():
    n = node(start=False)
    assert n.fsm.state == fsm.READY
    n.prereq_grace_s = 0.5
    n._loc.state = NOT_READY
    run_for(n, 0.3, torque=True, field=True)
    assert n.fsm.state == fsm.READY  # the loaded mission is not thrown away for one gap
    n._loc.state = LocalizationState.READY
    run_for(n, 1.0, torque=True, field=True)
    assert n.fsm.state == fsm.READY
    n._loc.state = NOT_READY  # a real loss still unreadies it
    run_for(n, 0.7, torque=True, field=True)
    assert n.fsm.state == fsm.IDLE


def test_losing_wheel_feedback_while_moving_is_not_debounced():
    """The grace period is for faults. A wheel-feedback loss under way stops AT ONCE (a
    recoverable `pending` hold), because waiting half a second to stop is the wrong way round."""
    n = node()
    n.prereq_grace_s = 5.0
    run_for(n, 0.4, wheels=False, torque=True, field=True)
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "pending"


def test_the_grace_timer_does_not_survive_into_the_next_mission():
    """A run that faulted with a broken prerequisite must not hand its timer to the next
    mission, or the first bad tick after loading would fault instantly."""
    n = node()
    n.prereq_grace_s = 0.5
    n._loc.state = NOT_READY
    run_for(n, 1.0, torque=True, field=True)
    assert n.fsm.state == fsm.FAULT
    run_for(n, 1.0, torque=True, field=True)  # ticks in FAULT clear the timer
    assert n._prereq_since is None
    assert n.fsm.ack() and n.fsm.load("m2", len(n.compiled.steps))
    run_for(n, 0.3, torque=True, field=True)  # still not READY, but inside a fresh grace period
    assert n.fsm.state == fsm.READY


# -- the pre-move warning (2026-10-08): drives in standby until a Start --------------------

def started(premove_s=2.0):
    """A run started from READY by the physical Start, with the drives in standby."""
    n = node(start=False)
    n._pose = lambda: (0.0, 0.0, 0.0)  # at the route start
    n.premove_s, n.premove_timeout_s = premove_s, 8.0
    tick(n, torque=False, field=True)
    assert n.fsm.state == fsm.READY and not n.drive_power()
    n._start_edge()
    return n


def state_msg(n):
    out = {}
    n._state_pub = SimpleNamespace(publish=lambda m: out.setdefault("m", m))
    n.manifest = None
    if not hasattr(n.route, "route_id"):
        n.route = SimpleNamespace(**vars(n.route), route_id="r", revision=1)
    n._publish_state()
    return out["m"]


def test_a_start_warns_then_moves_once_the_drives_have_torque():
    n = started()
    assert n.fsm.state == fsm.EXECUTING and n.phase == ren.PHASE_PREMOVE
    m = state_msg(n)
    assert m.premove and m.drive_power
    run_for(n, 1.0, torque=False, field=True)
    assert n.phase == ren.PHASE_PREMOVE and n.fsm.state == fsm.EXECUTING, "torque off here is no stop"
    assert n.sent == [], "no goal during the warning"
    run_for(n, 0.5, torque=True, field=True)
    assert n.phase == ren.PHASE_PREMOVE, "the warning runs its full time"
    run_for(n, 0.8, torque=True, field=True)
    assert n.sent == ["s1"] and not state_msg(n).premove


def test_no_torque_within_the_timeout_holds_for_a_human():
    n = started()
    run_for(n, 8.2, torque=False, field=True)
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "estop" and "did not power up" in n.fsm.reason
    assert not n.drive_power() and n.sent == []


def test_holds_that_wait_for_start_drop_power_and_resume_through_the_warning():
    n = node()
    n.premove_s = 2.0
    tick(n, torque=True, field=True)
    tick(n, torque=False, wheels=False, field=True)          # the E-stop button
    assert n.hold_cause == "estop" and not n.drive_power()
    run_for(n, 1.0, torque=False, field=True)                # standby: torque off on purpose
    n._start_edge()                                           # no "drives have no torque" refusal
    assert n.fsm.state == fsm.EXECUTING and n.phase == ren.PHASE_PREMOVE and n.drive_power()


def test_holds_that_resume_by_themselves_keep_power_and_skip_the_warning():
    n = node()
    n.premove_s = 2.0
    tick(n, torque=True, field=True)
    tick(n, field=False)
    tick(n, torque=False, wheels=False)
    assert n.hold_cause == "field" and n.drive_power()
    run_for(n, 2.5, torque=True, field=True)
    assert n.fsm.state == fsm.EXECUTING and n.phase != ren.PHASE_PREMOVE
