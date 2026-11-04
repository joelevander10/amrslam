"""Route wait steps and PB Continue in the executor (gls-wait-coding-plan §A.4, 2026-10-30).

A wait stops at the previous step's end pose until the operator presses Continue (or Start):
WAITING, drives disarmed, no permit. Nothing that guards a MOVING run applies to it - the
prerequisite faults, the MANUAL takeover, the emergency hold, the /initialpose fault - and the
press re-checks everything instead. Built with __new__ like test_auto_resume."""

import math
from types import SimpleNamespace

from amr_navigation.compiler import STRAIGHT, WAIT, CompiledStep
from test_auto_resume import run_for, tick
from test_route_executor_logic import _clock_stub, make_node

from amr_interfaces.msg import LocalizationState, MotionPermit, PanelState
from amr_mission import route_executor_node as ren
from amr_mission import run_fsm as fsm

WAIT_POSE = (2.0, 0.0, 0.0)


def straight(sid, x0, x1):
    n = round((x1 - x0) / 0.1)
    return CompiledStep(
        sid, STRAIGHT, (x0, 0.0, 0.0), (x1, 0.0, 0.0), length_m=x1 - x0,
        samples=[(x0 + 0.1 * k, 0.0, 0.0) for k in range(n + 1)], v_mps=0.55,
    )


def node(waiting=True):
    """s1 straight 0 -> 2 m, w1 wait at 2 m ("UBC hitch", label x0100y0000), s2 straight 2 -> 4 m.
    waiting=True: s1 is done and one tick has entered the wait."""
    steps = [
        straight("s1", 0.0, 2.0),
        CompiledStep("w1", WAIT, WAIT_POSE, WAIT_POSE, name="UBC hitch", label_id="x0100y0000"),
        straight("s2", 2.0, 4.0),
    ]
    n = make_node(steps)
    _clock_stub(n)
    n.generation, n._lease_instance, n._permit_seq = 1, "sup", 0
    n.permits = []
    n._permit_pub = SimpleNamespace(publish=n.permits.append)
    n.verify_pos, n.verify_yaw = 0.15, math.radians(8.0)
    n.wait_since, n.continue_refused, n.last_initialpose_t = None, "", None
    n._pose = lambda: WAIT_POSE
    n.sent = []

    def send(st, _pose):
        n.sent.append(st.id)
        n.goals.current, n.goals.result, n.goals.handle = object(), None, object()
        return True

    n._send_follow = send
    assert n.fsm.start(True, True)
    assert n.fsm.step_done()  # s1 settled: the bookkeeping is on the wait
    n._reset_step_state()
    if waiting:
        tick(n)
    return n


def panel(start=False, cont=False, emergency=PanelState.EMG_NORMAL, auto=True):
    m = PanelState()
    m.valid, m.mode_auto, m.emergency = True, auto, emergency
    m.estop_ok = emergency == PanelState.EMG_NORMAL
    m.start_edge, m.continue_edge = start, cont
    return m


def test_the_wait_step_stops_disarmed_with_no_permit():
    n = node()
    assert n.fsm.state == fsm.WAITING and n.fsm.step_index == 1
    assert "UBC hitch" in n.fsm.reason and "Continue" in n.fsm.reason
    assert n.sent == [] and not n.drive_power(), "nothing is driven, the drives are disarmed (W2)"
    run_for(n, 1.0)
    assert n.permits[-1].source == MotionPermit.NONE and not n.permits[-1].enabled
    assert n.wait_since is not None


def test_a_wait_breaks_a_chain():
    n = node(waiting=False)
    assert not n._chainable(n.compiled.steps[1])
    assert [s.id for s in n._chain(0)] == ["s1"], "the straight before a wait stops and settles"


def test_continue_releases_the_wait_through_the_premove_warning():
    n = node()
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.EXECUTING and n.phase == ren.PHASE_PREMOVE and n.drive_power()
    assert n.fsm.step_index == 2, "past the wait, on the next step"
    assert n.continue_refused == ""


def test_start_also_releases_the_wait():
    n = node()
    n._on_panel(panel(start=True))
    assert n.fsm.state == fsm.EXECUTING and n.fsm.step_index == 2


def test_continue_is_refused_in_manual_and_the_wait_survives_it():
    n = node()
    n._panel.mode_auto = False
    run_for(n, 2.0)
    assert n.fsm.state == fsm.WAITING, "MANUAL while waiting is not a takeover abort (W6)"
    n._on_panel(panel(cont=True, auto=False))
    assert n.fsm.state == fsm.WAITING and "AUTO" in n.continue_refused
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.EXECUTING


def test_continue_is_refused_off_the_wait_pose():
    n = node()
    n._pose = lambda: (2.25, 0.0, 0.0)  # nudged 0.25 m in MANUAL
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.WAITING and "off the wait pose" in n.continue_refused
    n._pose = lambda: (2.0, 0.0, math.radians(10.0))
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.WAITING and "off the wait pose" in n.continue_refused


def test_a_lapsed_prerequisite_while_waiting_is_not_a_fault():
    n = node()
    n._loc.state = LocalizationState.LOST
    run_for(n, 3.0)
    assert n.fsm.state == fsm.WAITING, "nothing moves: only the Continue gate cares"
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.WAITING and "not READY" in n.continue_refused


def test_an_initial_pose_while_waiting_is_accepted():
    n = node()
    n._on_initialpose(None)
    assert n.fsm.state == fsm.WAITING and n.fault_code == ""


def test_an_emergency_never_ends_a_wait_and_a_press_during_it_is_dropped():
    n = node()
    for state in (PanelState.EMG_ACTIVE, PanelState.EMG_RESET_NEEDED, PanelState.EMG_RESETTING):
        n._on_panel(panel(cont=True, emergency=state))
        n._panel_t = n._now()
        n._tick()
        assert n.fsm.state == fsm.WAITING and n.hold_cause == ""
        assert "E-stop" in n.continue_refused and n.phase != ren.PHASE_PREMOVE
    n._on_panel(panel(emergency=PanelState.EMG_NORMAL))  # released + Reset, no new press
    run_for(n, 1.0)
    assert n.fsm.state == fsm.WAITING, "the press made during the E-stop never fires"
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.EXECUTING and n.fsm.step_index == 2


def test_continue_resumes_an_estop_hold_like_start():
    n = node(waiting=False)
    n.fsm.step_index = 2  # driving s2
    n.phase = ren.PHASE_GOAL
    n._on_panel(panel(emergency=PanelState.EMG_ACTIVE))
    assert n.fsm.state == fsm.BLOCKED and n.hold_cause == "estop"
    n._pose = lambda: (2.5, 0.0, 0.0)
    n._on_panel(panel(cont=True, emergency=PanelState.EMG_ACTIVE))
    assert n.fsm.state == fsm.BLOCKED, "refused while the emergency lasts"
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.EXECUTING and n.phase == ren.PHASE_PREMOVE
    assert n.fsm.step_index == 2


def test_continue_does_not_start_a_new_run():
    n = make_node([straight("s1", 0.0, 2.0)])
    _clock_stub(n)
    n.wait_since, n.continue_refused = None, ""
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.READY and "needs Start" in n.fsm.reason


def test_abort_from_a_wait():
    n = node()
    n._interrupt = lambda _why: None
    res = n._srv_abort(None, SimpleNamespace(success=None, message=""))
    assert res.success and n.fsm.state == fsm.IDLE


def test_continue_at_a_wait_clears_the_local_costmap_before_the_warning():
    """Owner 2026-11-03: whoever walked round the vehicle at the wait left marks beside it that
    the scanner can never clear; Continue drops them before the pre-move warning."""
    n = node()
    calls = []
    n._clear_cli = SimpleNamespace(service_is_ready=lambda: True, call_async=calls.append)
    n._note = lambda *_a: None
    n._on_panel(panel(cont=True))
    assert n.fsm.state == fsm.EXECUTING and n.phase == ren.PHASE_PREMOVE
    assert len(calls) == 1


def test_the_web_continue_releases_only_a_wait_and_only_when_enabled():
    """/amr/continue (the AMR QR's Run page): off by default; on, it is a Continue press at a
    wait - the same gate and pre-move warning - and refuses anything that is not WAITING."""
    from std_srvs.srv import Trigger

    n = node()
    n._on_panel(panel())
    n.web_continue = False
    r = n._srv_continue(None, Trigger.Response())
    assert not r.success and "panel" in r.message and n.fsm.state == fsm.WAITING
    n.web_continue = True
    n._pose = lambda: (2.25, 0.0, 0.0)  # off the wait pose: refused like the button
    r = n._srv_continue(None, Trigger.Response())
    assert not r.success and n.fsm.state == fsm.WAITING and "off the wait pose" in n.continue_refused
    n._pose = lambda: WAIT_POSE
    r = n._srv_continue(None, Trigger.Response())
    assert r.success and n.fsm.state == fsm.EXECUTING and n.phase == ren.PHASE_PREMOVE
    assert n.fsm.step_index == 2
    r = n._srv_continue(None, Trigger.Response())
    assert not r.success and "nothing is waiting" in r.message, "a moving run is not continued again"


def test_the_web_continue_is_refused_in_an_emergency():
    from std_srvs.srv import Trigger

    n = node()
    n.web_continue = True
    n._on_panel(panel(emergency=PanelState.EMG_ACTIVE))
    r = n._srv_continue(None, Trigger.Response())
    assert not r.success and n.fsm.state == fsm.WAITING
