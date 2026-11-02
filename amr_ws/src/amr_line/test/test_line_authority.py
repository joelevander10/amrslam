"""Who may move the vehicle, and what stops it. No ROS, no hardware.

`amr_line.job` is deliberately ROS-free so these can run anywhere, including a
Windows dev box with no rclpy. The engine is real - a stub follower would let
this suite pass while the thing it guards did not exist - but the world around
it is a dataclass, so every check below is about authority and holds rather
than about control.
"""
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_line")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from amr_line import autopilot, runtime  # noqa: E402
from amr_line import job as lj  # noqa: E402

runtime.load_from_profile()

LEASE_LINE = 8


def inputs(**over):
    """A world in which the vehicle is allowed to move. Override to break it."""
    base = dict(
        now=100.0,
        dt=0.02,
        sensor={"tracks": [{"index": 1, "pos_mm": 0, "width": 10}], "has_track": True},
        sensor_age_s=0.005,
        track_ok=True,
        track_cause="",
        panel_valid=True,
        panel_auto=True,
        start_edge=False,
        start_edge_t=None,
        reset_edge=False,
        lease_allowed=LEASE_LINE,
        lease_line=LEASE_LINE,
        authority=("sup-1", 3),
        torque_off=False,
        field_clear=True,
        drives_fresh=True,
    )
    base.update(over)
    return lj.Inputs(**base)


def make():
    return lj.FollowJob(autopilot.LineFollower())


def start(job, t=101.0, **over):
    """One physical Start edge from IDLE (or DONE). There is no software arm."""
    return job.tick(inputs(now=t, start_edge=True, start_edge_t=t - 0.05, **over))


def run(job, t=101.0):
    start(job, t=t)
    assert job.state == lj.RUNNING, job.reason
    return job


# ---------------------------------------------------------------------------
# starting: the physical Start from IDLE, no arm step (2026-10-07)
# ---------------------------------------------------------------------------

def test_start_refuses_without_each_prerequisite():
    """Every one of these alone must be enough to refuse. A prerequisite that
    is checked only in combination is not a prerequisite."""
    for name, broken in (
        ("stale panel", {"panel_valid": False}),
        ("selector in MANUAL", {"panel_auto": False}),
        ("no lease", {"authority": None}),
        ("lease without LEASE_LINE", {"lease_allowed": 1}),
        ("no drive report", {"drives_fresh": False}),
        ("field violated", {"field_clear": False}),
        ("field unknown", {"field_clear": None}),
        ("unusable tape", {"track_ok": False, "track_cause": "track"}),
        ("tape too slow", {"track_ok": False, "track_cause": "rate"}),
    ):
        job = make()
        assert start(job, **broken) == (0.0, 0.0)
        assert job.state == lj.IDLE, f"{name} was allowed to start"
        assert job.reason.startswith("Start refused:"), job.reason
        assert job.refused, "the refusal is handed to the node for an operator event"


def test_a_refused_start_does_not_run_later():
    """The commissioning hazard, kept without an arm: a press refused while the
    field was blocked must not run the vehicle when the field clears."""
    job = make()
    start(job, t=200.0, field_clear=False)
    assert job.state == lj.IDLE
    for t in (200.1, 201.0, 205.0):
        assert job.tick(inputs(now=t)) == (0.0, 0.0)
        assert job.state == lj.IDLE
    assert job.reason.startswith("ready:")


def test_idle_shows_live_readiness_and_never_enters_armed():
    job = make()
    job.tick(inputs(now=100.0, panel_auto=False))
    assert job.state == lj.IDLE and job.reason == "not ready: selector is not in AUTO"
    job.tick(inputs(now=100.1))
    assert job.state == lj.IDLE and job.reason.startswith("ready:")
    run(job, t=100.2)
    assert job.state != lj.ARMED


def test_a_start_from_idle_runs():
    job = run(make())
    assert job.state == lj.RUNNING and job._binding == ("sup-1", 3)


def test_a_start_from_done_runs_a_fresh_run():
    job = run(make())
    job.state = lj.DONE
    job.followed_m = 12.0
    run(job, t=110.0)
    assert job.state == lj.RUNNING and job.followed_m == 0.0


def test_a_start_in_fault_is_ignored_until_reset():
    job = run(make())
    job.tick(inputs(now=102.0, lease_allowed=1))
    assert job.state == lj.FAULT
    start(job, t=103.0)
    assert job.state == lj.FAULT
    job.tick(inputs(now=104.0, reset_edge=True))
    run(job, t=105.0)


# ---------------------------------------------------------------------------
# what stops it
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "broken,cause",
    [
        ({"torque_off": True}, "estop"),
        ({"field_clear": False}, "field"),
        ({"torque_off": True, "field_clear": False}, "field"),
        ({"drives_fresh": False}, "drives"),
    ],
)
def test_a_running_vehicle_stops_at_once(broken, cause):
    """No debounce on anything meaning the vehicle may be moving without the
    torque or the feedback to do it safely. Delaying a stop is the wrong
    direction, so these must act on the FIRST tick."""
    job = run(make())
    left, right = job.tick(inputs(now=102.0, **broken))
    assert (left, right) == (0.0, 0.0)
    assert job.state == lj.HOLD
    assert job.hold_cause == cause


def test_a_field_trip_and_an_estop_are_told_apart():
    """Both are the drives losing torque; the protective field is what
    distinguishes a stop that resumes itself from one that needs a human."""
    a = run(make())
    a.tick(inputs(now=102.0, torque_off=True, field_clear=False))
    b = run(make())
    b.tick(inputs(now=102.0, torque_off=True, field_clear=True))
    assert (a.hold_cause, b.hold_cause) == ("field", "estop")
    assert a.auto_resume() and not b.auto_resume()


def test_losing_the_lease_faults_and_never_auto_resumes():
    """A replaced lease means something above this layer took control. It is
    not a condition that clears - resuming on it would be resuming into a
    vehicle somebody else is now driving."""
    job = run(make())
    left, right = job.tick(inputs(now=102.0, authority=("sup-1", 4)))
    assert (left, right) == (0.0, 0.0)
    assert job.state == lj.FAULT
    assert not job.auto_resume()


def test_one_bad_tick_does_not_latch_anything():
    """prereq_grace_s exists so a single dropped frame is survivable. The tape
    is unusable for one tick, then fine: the run must continue."""
    job = run(make())
    job.tick(inputs(now=102.0, track_ok=False, track_cause="track"))
    assert job.state == lj.RUNNING, "a single bad frame stopped the run"
    job.tick(inputs(now=102.02))
    assert job.state == lj.RUNNING


def test_a_continuous_failure_does_latch():
    """...but a real loss still stops it, half a second later."""
    job = run(make())
    bad = dict(track_ok=False, track_cause="rate")
    job.tick(inputs(now=102.0, **bad))
    assert job.state == lj.RUNNING
    job.tick(inputs(now=102.6, **bad))
    assert job.state == lj.HOLD
    assert job.hold_cause == "rate"


def test_the_grace_timer_is_about_anything_wrong_not_one_message():
    """The reason may change while the timer runs. What is debounced is
    'something is wrong', so a different fault each tick still latches."""
    job = run(make())
    job.tick(inputs(now=102.0, track_ok=False, track_cause="track"))
    job.tick(inputs(now=102.3, panel_valid=False))
    job.tick(inputs(now=102.7, track_ok=False, track_cause="rate"))
    assert job.state == lj.HOLD


def test_the_selector_leaving_auto_ends_the_run_at_once():
    """Acceptance 3.6: MANUAL mid-run stops at once, "authority", FAULT,
    Reset to clear. Not debounced - a flick to MANUAL and back inside the
    grace window used to leave the job RUNNING, entitled to move again the
    moment the mux re-opened, with nobody having pressed Start."""
    job = run(make())
    left, right = job.tick(inputs(now=102.0, panel_auto=False))
    assert (left, right) == (0.0, 0.0)
    assert job.state == lj.FAULT and job.hold_cause == "authority", job.reason
    job.tick(inputs(now=102.2))  # back in AUTO, no Start
    assert job.state == lj.FAULT
    job.tick(inputs(now=102.3, start_edge=True, start_edge_t=102.25))
    assert job.state == lj.FAULT, "a Start alone revived a taken-over run"
    job.tick(inputs(now=102.4, reset_edge=True))
    assert job.state == lj.IDLE


def test_lease_line_withdrawn_ends_the_run_at_once():
    job = run(make())
    job.tick(inputs(now=102.0, lease_allowed=1))
    assert job.state == lj.FAULT and job.hold_cause == "authority"


def test_a_start_under_manual_is_refused_not_faulted():
    """Without an arm there is no armed layer to take over: a Start under
    MANUAL is a refusal, and the job stays IDLE."""
    job = make()
    start(job, panel_auto=False)
    assert job.state == lj.IDLE and "selector is not in AUTO" in job.reason


def test_a_stale_panel_is_still_debounced():
    """Missing evidence is a comms question, not an operator act: one stale
    panel frame must not fault the run (the 2026-09-20 grace rule). A stale
    LEASE is different and already immediate: authority None is not the
    binding the run was accepted under."""
    job = run(make())
    job.tick(inputs(now=102.0, panel_valid=False, panel_auto=False))
    assert job.state == lj.RUNNING
    job.tick(inputs(now=102.1))
    assert job.state == lj.RUNNING


@pytest.mark.parametrize("first", [
    {"drives_fresh": False},
    {"track_ok": False, "track_cause": "rate"},
    {"track_ok": False, "track_cause": "track"},
])
def test_a_torque_loss_during_an_auto_hold_takes_the_safety_cause(first):
    """drives/rate/track -> torque off with the field clear is an E-stop, and
    an E-stop waits for a human. Before: the hold kept its first cause and
    resumed by itself two seconds after everything came back."""
    job = run(make())
    for t in (102.0, 102.6):
        job.tick(inputs(now=t, **first))
    assert job.state == lj.HOLD and job.hold_cause in ("drives", "rate", "track")
    job.tick(inputs(now=103.0, torque_off=True, field_clear=True))
    assert job.hold_cause == "estop" and not job.auto_resume()
    for t in (104.0, 106.0, 110.0):
        job.tick(inputs(now=t))
        assert job.state == lj.HOLD, "resumed without Start after an E-stop"
    job.tick(inputs(now=111.0, start_edge=True, start_edge_t=110.9))
    assert job.state == lj.RUNNING


def test_unknown_field_evidence_is_an_estop_not_a_field_stop():
    """F09: no fresh /output_paths cannot prove a torque loss was only the
    field, so the hold waits for a human; and it never auto-resumes while
    the field is still unknown."""
    job = run(make())
    job.tick(inputs(now=102.0, torque_off=True, field_clear=None))
    assert job.hold_cause == "estop" and not job.auto_resume()
    # a field hold does not resume on unknown evidence either
    job2 = run(make())
    job2.tick(inputs(now=102.0, torque_off=True, field_clear=False))
    assert job2.hold_cause == "field"
    for t in (103.0, 104.0, 106.0):
        job2.tick(inputs(now=t, field_clear=None))
    assert job2.state == lj.HOLD and "unknown" in job2.reason


def test_a_field_hold_keeps_the_fields_terms():
    """A field hold sees torque off until the safety relay restarts; that is
    the field's own recovery, not a new E-stop."""
    job = run(make())
    job.tick(inputs(now=102.0, torque_off=True, field_clear=False))
    assert job.hold_cause == "field"
    job.tick(inputs(now=102.5, torque_off=True, field_clear=True))
    assert job.hold_cause == "field" and job.auto_resume()


def test_reset_wins_over_a_start_in_the_same_tick():
    job = run(make())
    job.tick(inputs(now=102.0, reset_edge=True, start_edge=True, start_edge_t=101.95))
    assert job.state == lj.IDLE


# ---------------------------------------------------------------------------
# resuming
# ---------------------------------------------------------------------------

def test_an_auto_hold_waits_for_the_cause_to_stay_clear():
    """Not 'is it clear now' but 'has it been clear', which is what stops an
    intermittent fault accumulating its way back into motion."""
    job = run(make())
    job.tick(inputs(now=102.0, field_clear=False))
    assert job.state == lj.HOLD
    job.tick(inputs(now=103.0))               # clear, but not for long enough
    assert job.state == lj.HOLD
    job.tick(inputs(now=103.5, field_clear=False))  # dropped out again
    assert job.state == lj.HOLD
    job.tick(inputs(now=104.0))               # the window restarts here
    job.tick(inputs(now=105.0))
    assert job.state == lj.HOLD, "resumed before the clear window elapsed"
    job.tick(inputs(now=106.1))
    assert job.state == lj.RUNNING


def test_an_estop_hold_waits_for_a_human():
    job = run(make())
    job.tick(inputs(now=102.0, torque_off=True, field_clear=True))
    assert job.hold_cause == "estop"
    for t in (103.0, 110.0, 200.0):
        job.tick(inputs(now=t))
        assert job.state == lj.HOLD, "an E-stop hold resumed on its own"
    job.tick(inputs(now=201.0, start_edge=True, start_edge_t=200.9))
    assert job.state == lj.RUNNING


def test_reset_ends_a_run_at_once():
    """Physical Reset while RUNNING -> IDLE, zero wheels, this tick (vehicle
    finding 2026-09-21: Reset was ignored while running)."""
    job = run(make())
    assert job.state == lj.RUNNING
    left, right = job.tick(inputs(now=102.0, reset_edge=True))
    assert job.state == lj.IDLE and (left, right) == (0.0, 0.0) and job.reason == "reset"
    run(job, t=102.1)  # a fresh Start is allowed


def test_reset_clears_a_hold_to_idle():
    job = run(make())
    job.tick(inputs(now=102.0, torque_off=True))
    assert job.state == lj.HOLD
    job.tick(inputs(now=103.0, reset_edge=True))
    assert job.state == lj.IDLE


# ---------------------------------------------------------------------------
# ending
# ---------------------------------------------------------------------------

def test_running_off_the_end_of_the_tape_is_DONE_not_a_fault():
    """The engine owns the line-loss distance budget. When it gives up, the
    tape ended - that is a finished run, not something to be recovered from."""
    job = run(make())
    empty = {"tracks": [], "has_track": False}
    t = 102.0
    for _ in range(600):
        t += 0.02
        job.tick(inputs(now=t, sensor=empty))
        if job.state != lj.RUNNING:
            break
    assert job.state == lj.DONE, f"ended as {lj.STATE_NAMES[job.state]}: {job.reason}"
    assert "track lost" in job.reason


def test_clear_stops_a_running_follow():
    job = run(make())
    assert job.clear() is True
    assert job.state == lj.IDLE
    assert job.tick(inputs(now=103.0)) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# speed: no job-level cap since 2026-10-02
# ---------------------------------------------------------------------------

def test_the_job_has_no_speed_cap_of_its_own():
    """Tracked speeds are the follower's two profile values; the job no longer
    clips them (the 0.30 m/s Increment 1 cap is gone)."""
    job = make()
    assert not hasattr(job, "_cap") and not hasattr(job, "v_max_mps")


# ---------------------------------------------------------------------------
# the wire contract
# ---------------------------------------------------------------------------

def test_state_values_match_the_message():
    """readiness.py holds `line_state in (1, 2, 3)` for ARMED/RUNNING/HOLD and
    mode_fsm refuses to leave LINE while that is true. If these drift, a layer
    swap stops being blocked under a moving vehicle - silently. ARMED is never
    entered since 2026-10-07 but keeps its number on the wire."""
    assert (lj.IDLE, lj.ARMED, lj.RUNNING, lj.HOLD) == (0, 1, 2, 3)
    assert (lj.DONE, lj.FAULT) == (4, 5)
    assert set(lj.STATE_NAMES) == {0, 1, 2, 3, 4, 5}


def test_the_engine_is_real_not_a_stub():
    """This suite is only worth anything if it is guarding the actual engine."""
    assert hasattr(autopilot, "LineFollower")
    assert callable(autopilot.predicted_zeta)
    assert 0.0 < autopilot.predicted_zeta() < 2.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))


# ---------------------------------------------------------------------------
# the pre-move warning (ARMED, 2026-10-08): drives in standby until a Start
# ---------------------------------------------------------------------------

def warned():
    return lj.FollowJob(autopilot.LineFollower(), premove_s=2.0, premove_timeout_s=8.0)


def test_idle_asks_for_no_power_and_a_start_does_not_need_torque():
    job = warned()
    job.tick(inputs(now=100.0, torque_off=True))
    assert job.state == lj.IDLE and not job.wants_power()
    assert "torque" not in job.reason, "standby is the normal idle state, not a readiness problem"
    assert start(job, torque_off=True) == (0.0, 0.0)
    assert job.state == lj.ARMED and job.wants_power(), job.reason


def test_the_warning_runs_its_full_time_and_waits_for_torque():
    job = warned()
    start(job, t=101.0, torque_off=True)
    assert job.tick(inputs(now=101.5)) == (0.0, 0.0)            # torque already, warning still on
    assert job.state == lj.ARMED, "moved before the warning had run"
    job.tick(inputs(now=103.0, torque_off=True))                 # warning done, no torque yet
    assert job.state == lj.ARMED and "power up" in job.reason
    job.tick(inputs(now=103.4))
    assert job.state == lj.RUNNING and job.wants_power()


def test_no_torque_within_the_timeout_cancels_the_start():
    job = warned()
    start(job, t=101.0, torque_off=True)
    job.tick(inputs(now=108.9, torque_off=True))
    assert job.state == lj.ARMED
    job.tick(inputs(now=109.1, torque_off=True))
    assert job.state == lj.IDLE and not job.wants_power()
    assert "did not power up" in job.reason and job.refused


def test_a_field_trip_during_the_warning_cancels_at_once():
    job = warned()
    start(job, t=101.0, torque_off=True)
    job.tick(inputs(now=101.5, field_clear=False))
    assert job.state == lj.IDLE and "field" in job.reason


def test_a_lapsing_prerequisite_during_the_warning_is_debounced_then_cancels():
    job = warned()
    start(job, t=101.0, torque_off=True)
    job.tick(inputs(now=101.2, track_ok=False, track_cause="track"))
    assert job.state == lj.ARMED, "one bad tick cancels nothing"
    job.tick(inputs(now=101.8, track_ok=False, track_cause="track"))
    assert job.state == lj.IDLE and "tape" in job.reason


def test_reset_and_the_selector_end_the_warning():
    job = warned()
    start(job, t=101.0, torque_off=True)
    job.tick(inputs(now=101.5, reset_edge=True))
    assert job.state == lj.IDLE
    job = warned()
    start(job, t=101.0, torque_off=True)
    job.tick(inputs(now=101.5, panel_auto=False))
    assert job.state == lj.FAULT, "an explicit takeover ends it like a run"


def test_a_hold_waiting_for_start_drops_power_and_resumes_through_the_warning():
    job = warned()
    start(job, t=101.0, torque_off=True)
    job.tick(inputs(now=103.1))
    assert job.state == lj.RUNNING
    job.tick(inputs(now=104.0, torque_off=True, field_clear=True))   # E-stop
    assert job.hold_cause == "estop" and not job.wants_power()
    job.tick(inputs(now=110.0, torque_off=True))
    assert job.hold_cause == "estop", "standby is not a second E-stop"
    job.tick(inputs(now=111.0, torque_off=True, start_edge=True, start_edge_t=110.9))
    assert job.state == lj.ARMED and job.wants_power()
    job.tick(inputs(now=112.0))
    assert job.state == lj.ARMED
    job.tick(inputs(now=113.1))
    assert job.state == lj.RUNNING and job.reason == "resumed on Start"


def test_an_auto_hold_keeps_power_and_no_warning():
    job = warned()
    start(job, t=101.0)
    job.tick(inputs(now=103.1))
    job.tick(inputs(now=104.0, torque_off=True, field_clear=False))
    assert job.hold_cause == "field" and job.wants_power(), "the field's own recovery needs the drives"
    for t in (105.0, 106.0, 107.1):
        job.tick(inputs(now=t))
    assert job.state == lj.RUNNING, "auto-resume does not go through ARMED"


def test_a_cancelled_resume_returns_to_its_hold():
    job = warned()
    start(job, t=101.0)
    job.tick(inputs(now=103.1))
    job.tick(inputs(now=104.0, torque_off=True, field_clear=True))
    job.tick(inputs(now=111.0, torque_off=True, start_edge=True, start_edge_t=110.9))
    job.tick(inputs(now=119.5, torque_off=True))
    assert job.state == lj.HOLD and job.hold_cause == "estop" and "did not power up" in job.reason
