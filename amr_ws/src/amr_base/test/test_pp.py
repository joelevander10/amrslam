"""amr_base.pp: profile-position blind moves - geometry, bounds, config check, and the
held-not-latched state machine with every halt trigger (plan: PP blind-run page)."""

import pytest

from amr_base import pp

SEG_STRAIGHT = {"counts": [1_000_000, 1_000_000], "rpm": [1273.0, 1273.0], "duration_s": 3.0}
# arc: inner wheel 60 % of the outer
SEG_ARC = {"counts": [600_000, 1_000_000], "rpm": [763.8, 1273.0], "duration_s": 3.0}
SEG_PIVOT = {"counts": [-400_000, 400_000], "rpm": [-500.0, 500.0], "duration_s": 2.0}


def spec(seg=SEG_STRAIGHT, a=800, d=800):
    return pp.move_spec(seg, a, d)


# ---- geometry ------------------------------------------------------------------------


def test_straight_both_wheels_identical():
    s = spec()
    assert s.left == s.right == pp.WheelMove(1_000_000, 1273, 800, 800)
    assert s.ratio_error == 0.0


def test_arc_scales_velocity_and_both_ramps_by_distance_ratio():
    s = spec(SEG_ARC)
    assert (s.left.accel_rpm_s, s.left.decel_rpm_s) == (480, 480)  # 800 x 0.6
    assert (s.right.accel_rpm_s, s.right.decel_rpm_s) == (800, 800)
    assert s.left.velocity_rpm == 764 and s.right.velocity_rpm == 1273
    # the trapezoids are the same shape in time: v/a is equal on both wheels
    assert abs(s.left.velocity_rpm / s.left.accel_rpm_s - s.right.velocity_rpm / s.right.accel_rpm_s) < 0.01
    assert s.ratio_error < 1e-3


def test_pivot_signs_come_from_counts_not_rpm():
    s = spec(SEG_PIVOT)
    assert s.left.delta_counts == -400_000 and s.right.delta_counts == 400_000
    assert s.left.velocity_rpm == s.right.velocity_rpm == 500


def test_arc_with_a_stationary_inner_wheel():
    s = spec({"counts": [0, 500_000], "rpm": [0.0, 900.0], "duration_s": 2.0})
    assert not s.left.moves and s.left.velocity_rpm == 0
    assert s.right.moves


def test_zero_move_refused():
    with pytest.raises(ValueError):
        spec({"counts": [0, 0], "rpm": [0, 0], "duration_s": 1.0})


def test_absolute_targets_and_int32_refusal():
    assert pp.absolute_targets((10, -10), spec()) == (1_000_010, 999_990)
    with pytest.raises(ValueError, match="INT32"):
        pp.absolute_targets((pp.INT32_MAX - 5, 0), spec())


# ---- bounds at the drive owner -------------------------------------------------------


def bounds(s, **kw):
    args = {"max_rpm": 2546.0, "max_counts": 20_000_000, "max_accel_rpm_s": 800.0}
    args.update(kw)
    return pp.validate_move(s, **args)


def test_validate_accepts_a_sane_move():
    assert bounds(spec()) is None


@pytest.mark.parametrize(
    "kw, s, fragment",
    [
        ({"max_rpm": 1000.0}, spec(), "profile velocity"),
        ({"max_counts": 10}, spec(), "counts exceeds"),
        ({"max_accel_rpm_s": 500.0}, spec(), "acceleration"),
        ({}, pp.MoveSpec(pp.WheelMove(0, 0, 0, 0), pp.WheelMove(0, 0, 0, 0), 1.0), "zero counts"),
        ({}, pp.MoveSpec(pp.WheelMove(5, 10, 10, 10), pp.WheelMove(5, 10, 10, 10), 0.0), "duration"),
    ],
)
def test_validate_refuses(kw, s, fragment):
    assert fragment in (bounds(s, **kw) or "")


# ---- configuration readback -------------------------------------------------------

OBJECTS = {"max_torque_permille": 0x6072, "halt_option": 0x605D}
NODES = {1: "left", 2: "right"}


def test_verify_config_unset_profile_refuses_without_reading():
    reads = []
    out = pp.verify_config(
        lambda n, i: reads.append((n, i)), NODES, {"max_torque_permille": None, "halt_option": 1}, OBJECTS
    )
    assert out and "not configured" in out[0] and reads == []


def test_verify_config_match_mismatch_and_unreadable():
    expect = {"max_torque_permille": 500, "halt_option": 1}
    good = {(1, 0x6072): 500, (2, 0x6072): 500, (1, 0x605D): 1, (2, 0x605D): 1}
    assert pp.verify_config(lambda n, i: good[(n, i)], NODES, expect, OBJECTS) == []
    bad = dict(good)
    bad[(2, 0x6072)] = 1000  # the right drive differs
    bad[(1, 0x605D)] = None
    out = pp.verify_config(lambda n, i: bad[(n, i)], NODES, expect, OBJECTS)
    assert any("right 6072h" in p and "1000" in p for p in out)
    assert any("left 605Dh" in p and "unreadable" in p for p in out)


# ---- the controller ---------------------------------------------------------------


def fb(sw=pp.CW_ENABLED, rpm=0, t=0.0):
    return pp.Feedback(sw, rpm, t)


def req(t, run="r1", hold=True, s=None):
    return pp.Request(run, 3, hold, s or spec(), t)


def tick(c, now, request, gate=None, left=None, right=None):
    return c.tick(pp.Tick(now, request, gate, left or fb(t=now), right or fb(t=now)))


def started(c=None):
    c = c or pp.Controller()
    out = tick(c, 0.0, req(0.0))
    assert out.action == "enter"
    c.entered(True, "", (1_000_000, 1_000_000), 0.0)
    return c


def test_enter_needs_a_fresh_hold_and_runs_a_run_id_once():
    c = pp.Controller()
    assert tick(c, 1.0, req(0.5)).action == "none"  # stale hold
    assert tick(c, 1.0, req(1.0, hold=False)).action == "none"
    assert tick(c, 1.0, req(1.0)).action == "enter"
    c.entered(True, "", (0, 0), 1.0)
    c.state = pp.IDLE  # pretend it finished
    assert tick(c, 1.1, req(1.1)).action == "none"  # same run id: never again
    assert tick(c, 1.1, req(1.1, run="r2")).action == "enter"


def test_gate_at_entry_refuses_and_records():
    c = pp.Controller()
    assert tick(c, 0.0, req(0.0), gate="panel not MANUAL").action == "none"
    assert c.last.outcome == pp.REFUSED and "MANUAL" in c.last.reason


def test_refused_entry_goes_back_to_idle():
    c = pp.Controller()
    tick(c, 0.0, req(0.0))
    c.entered(False, "6061h did not read 1", None, 0.0)
    assert c.state == pp.IDLE and c.last.outcome == pp.REFUSED


def test_nsp_held_until_ack_then_released():
    c = started()
    out = tick(c, 0.02, req(0.02), left=fb(t=0.02), right=fb(t=0.02))
    assert out.controlwords == (pp.CW_START, pp.CW_START)
    ack = pp.CW_ENABLED | pp.SW_SETPOINT_ACK
    out = tick(c, 0.04, req(0.04), left=fb(ack, 100, 0.04), right=fb(pp.CW_ENABLED, 0, 0.04))
    assert out.controlwords == (pp.CW_ENABLED, pp.CW_START)


def test_stale_target_reached_before_ack_does_not_finish():
    c = started()
    tr = pp.CW_ENABLED | pp.SW_TARGET_REACHED  # left over from before the move
    for k in range(20):
        t = 0.02 * (k + 1)
        out = tick(c, t, req(t), left=fb(tr, 0, t), right=fb(tr, 0, t))
        assert out.action == "cw"


def test_done_after_ack_reached_at_rest_and_settled():
    c = started()
    ack, tr = pp.SW_SETPOINT_ACK, pp.SW_TARGET_REACHED
    tick(c, 0.02, req(0.02), left=fb(pp.CW_ENABLED | ack, 50, 0.02), right=fb(pp.CW_ENABLED | ack, 50, 0.02))
    # reached, but still turning: not done
    assert tick(c, 0.04, req(0.04), left=fb(tr, 5, 0.04), right=fb(tr, 5, 0.04)).action == "cw"
    assert tick(c, 0.06, req(0.06), left=fb(tr, 0, 0.06), right=fb(tr, 0, 0.06)).action == "cw"
    out = tick(c, 0.20, req(0.20), left=fb(tr, 0, 0.20), right=fb(tr, 0, 0.20))
    assert out.action == "exit"
    c.exited(True, "", 0.21)
    assert c.state == pp.IDLE and c.last.outcome == pp.DONE


@pytest.mark.parametrize(
    "make, fragment",
    [
        (lambda t: {"request": None}, "stale"),
        (lambda t: {"request": req(t - 0.5)}, "stale"),
        (lambda t: {"request": req(t, hold=False)}, "released"),
        (lambda t: {"request": req(t, run="other")}, "different run"),
        (lambda t: {"request": req(t), "gate": "selector AUTO"}, "selector AUTO"),
        (lambda t: {"request": req(t), "left": fb(t=t - 1.0)}, "left feedback stale"),
        (
            lambda t: {"request": req(t), "right": fb(pp.CW_ENABLED | pp.SW_FOLLOWING_ERROR, 10, t)},
            "following",
        ),
        (lambda t: {"request": req(t), "now": 99.0}, "time limit"),
    ],
)
def test_every_trigger_halts_both_wheels(make, fragment):
    c = started()
    kw = make(0.1)
    now = kw.pop("now", 0.1)
    if now != 0.1 and "request" in kw and kw["request"] is not None:
        kw["request"] = req(now)
    kw.setdefault("left", fb(t=now))
    kw.setdefault("right", fb(t=now))
    out = c.tick(pp.Tick(now, kw["request"], kw.get("gate"), kw["left"], kw["right"]))
    assert out.action == "cw" and out.controlwords == (pp.CW_HALTED, pp.CW_HALTED)
    assert c.state == pp.HALTING and fragment in c.reason


def test_halt_confirmed_at_rest_exits_as_halted():
    c = started()
    tick(c, 0.1, None)
    assert c.state == pp.HALTING
    assert tick(c, 0.2, None, left=fb(rpm=300, t=0.2), right=fb(rpm=0, t=0.2)).controlwords == (
        pp.CW_HALTED,
        pp.CW_HALTED,
    )
    assert tick(c, 0.5, None).action == "exit"
    c.exited(True, "", 0.5)
    assert c.last.outcome == pp.HALTED and "stale" in c.last.reason


def test_unconfirmed_halt_faults():
    c = started()
    tick(c, 0.1, None)
    out = tick(c, 3.2, None, left=fb(rpm=200, t=3.2), right=fb(rpm=0, t=3.2))
    assert out.action == "fault" and c.state == pp.IDLE and c.last.outcome == pp.FAULTED


def test_halting_ignores_stale_feedback_as_rest():
    c = started()
    tick(c, 0.1, None)
    # rpm 0 but from a frame 1 s old is not evidence of rest
    assert tick(c, 1.2, None, left=fb(rpm=0, t=0.2), right=fb(rpm=0, t=1.2)).action == "cw"


def test_failed_return_to_pv_is_a_fault_result():
    c = started()
    tick(c, 0.1, None)
    assert tick(c, 0.2, None).action == "exit"
    c.exited(False, "6061h stayed 1", 0.2)
    assert c.last.outcome == pp.FAULTED and "pv" in c.last.reason


def test_abandon_records_a_fault():
    c = started()
    c.abandon("drive fault", 0.3)
    assert c.state == pp.IDLE and c.last.outcome == pp.FAULTED


# ---- the drive owner's own authority check ------------------------------------------

LEASE_OK = (10.0, 3, pp.LEASE_COMMISSIONING)
PANEL_OK = (10.0, True, False)


def gate(**kw):
    args = {"enabled": True, "lease": LEASE_OK, "move_generation": 3, "panel": PANEL_OK}
    args.update(kw)
    return pp.authority_gate(10.05, **args)


def test_gate_open_with_lease_generation_and_manual_panel():
    assert gate() is None


@pytest.mark.parametrize(
    "kw, fragment",
    [
        ({"enabled": False}, "locked"),
        ({"lease": None}, "lease"),
        ({"lease": (9.0, 3, pp.LEASE_COMMISSIONING)}, "lease"),
        ({"lease": (10.0, 3, 1)}, "does not allow commissioning"),
        ({"move_generation": 4}, "generation"),
        ({"panel": None}, "panel state stale"),
        ({"panel": (9.5, True, False)}, "panel state stale"),
        ({"panel": (10.0, False, False)}, "not valid"),
        ({"panel": (10.0, True, True)}, "AUTO"),
    ],
)
def test_gate_refuses(kw, fragment):
    assert fragment in (gate(**kw) or "")
