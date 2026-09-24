"""AMR QR wheel loop and drive logic: interlocks, loop behaviour, fault latches.

Includes a small closed-loop plant (first-order motor, voltage deadband) so the PI is
checked for convergence, not only for arithmetic - and a negative control: the same plant
with the encoder sign wrong must trip the runaway latch.
"""

import math

import pytest

from amr_base import qr_motor as qm

P = qm.LoopParams(
    gear_ratio=30.0,
    rpm_per_volt=300.0,
    offset_v=0.4,
    v_max=4.0,
    kp=0.3,
    ki=2.0,
    i_max=1.0,
    zero_rad_s=0.05,
    dwell_s=0.1,
    runaway_rad_s=1.0,
    runaway_s=0.3,
    stall_s=2.0,
)


def test_feedforward_line():
    w = 2 * math.pi * 300 / 60 / 30  # 300 motor r/min
    assert qm.feedforward_v(w, P) == pytest.approx(0.4 + 1.0)


def test_rest_brakes_and_never_raises_both_coils():
    wl = qm.WheelLoop(P, invert=False, name="left")
    out = wl.step(0.0, 0.0, 0.0, True)
    assert (out.fwd, out.rev, out.brake, out.volts) == (False, False, True, 0.0)


def test_direction_waits_for_dwell_and_readback():
    wl = qm.WheelLoop(P, invert=False, name="left")
    wl.reset(0.0)
    assert wl.step(0.05, 1.0, 0.0, True).phase == qm.DWELL  # dwell not elapsed
    assert wl.step(0.11, 1.0, 0.0, False).phase == qm.DWELL  # readback still shows a coil high
    out = wl.step(0.12, 1.0, 0.0, True)
    assert out.phase == qm.DRIVE and out.fwd and not out.rev and not out.brake
    # reversal goes through zero first, with both coils low
    out = wl.step(0.14, -1.0, 0.5, False)
    assert (out.fwd, out.rev, out.volts) == (False, False, 0.0)
    assert out.brake is False  # passing through zero is not a stop
    assert wl.step(0.20, -1.0, 0.0, True).phase == qm.DWELL
    out = wl.step(0.25, -1.0, 0.0, True)
    assert out.rev and not out.fwd


def test_invert_selects_the_rev_coil_for_vehicle_forward():
    wr = qm.WheelLoop(P, invert=True, name="right")
    wr.reset(-1.0)
    out = wr.step(0.0, 1.0, 0.0, True)
    assert out.rev and not out.fwd


def test_antiwindup_and_voltage_cap():
    wl = qm.WheelLoop(P, invert=False, name="left")
    wl.reset(-1.0)
    t = 0.0
    for _ in range(200):  # wheel held still under a big command: output saturates
        out = wl.step(t, 20.0, 0.0, True)
        t += 0.02
    assert out.volts == pytest.approx(P.v_max)
    assert abs(wl._integ) <= P.i_max + 1e-9


def plant(steps, target, enc_sign=1.0, invert=False, load=0.0):
    """Motor: speed tends to k*(V-deadband) with tau 0.15 s; wheel speed in vehicle terms."""
    loop = qm.WheelLoop(P, invert=invert, name="left")
    loop.reset(-1.0)
    drv = -1.0 if invert else 1.0
    w_true, t, fault, hist = 0.0, 0.0, None, []
    k = 300.0 * 1.15 * 2 * math.pi / 60 / 30  # the real motor is 15 % stronger than the ff assumes
    for _ in range(steps):
        out = loop.step(t, target, w_true * enc_sign, not (loop.dir != 0))
        v = out.volts
        d = 1 if out.fwd else -1 if out.rev else 0
        w_ss = drv * d * max(0.0, v - 0.45) * k - load
        w_true += (w_ss - w_true) * 0.02 / 0.15
        t += 0.02
        hist.append(w_true)
        if out.fault:
            fault = out.fault
            break
    return hist, fault


def test_closed_loop_converges_despite_model_error():
    hist, fault = plant(150, 2.0)
    assert fault is None
    assert hist[-1] == pytest.approx(2.0, abs=0.05)
    hist, fault = plant(150, -1.5, invert=True)
    assert fault is None and hist[-1] == pytest.approx(-1.5, abs=0.05)


def test_negative_control_wrong_encoder_sign_trips_runaway():
    hist, fault = plant(150, 2.0, enc_sign=-1.0)
    assert fault is not None and "AGAINST" in fault


def test_stall_latch():
    wl = qm.WheelLoop(P, invert=False, name="left")
    wl.reset(-1.0)
    t, fault = 0.0, None
    while t < 5.0 and not fault:
        fault = wl.step(t, 5.0, 0.0, True).fault
        t += 0.02
    assert fault and "stalled" in fault and t < 2.0 + 1.5


def _inp(now, cmd=(0.0, 0.0), left=0.0, right=0.0, io=True, estop=False):
    return qm.Inputs(now, cmd, left, right, True, True, io, estop)


def _logic():
    return qm.DriveLogic(qm.WheelLoop(P, False, "left"), qm.WheelLoop(P, True, "right"))


def test_drive_logic_arms_faults_and_needs_ack():
    d = _logic()
    o = d.tick(_inp(0.0, left=None))
    assert o.state == qm.DISARMED and not o.operational  # no encoders, no arm
    o = d.tick(_inp(0.1))
    assert o.state == qm.ARMED and o.operational and o.event == "armed (targets zero)"
    o = d.tick(_inp(0.2, io=False))
    assert o.state == qm.FAULT and not o.operational and o.left.brake and o.left.volts == 0.0
    o = d.tick(_inp(0.3))
    assert o.state == qm.FAULT  # latched; I/O back is not enough
    assert d.ack()[0]
    assert d.tick(_inp(0.4)).state == qm.ARMED


def test_estop_forces_rest_with_brake_but_keeps_the_arm_state():
    d = _logic()
    d.tick(_inp(0.0))
    o = d.tick(_inp(0.1, cmd=(2.0, 2.0), estop=True))
    assert o.state == qm.ARMED and not o.operational and o.estop
    assert (o.left.volts, o.right.volts) == (0.0, 0.0) and o.left.brake and o.right.brake
    assert "E-STOP active" in o.event


def test_disarm_frees_the_brake():
    d = _logic()
    d.tick(_inp(0.0))
    d.disarm()
    o = d.tick(_inp(0.1))
    assert o.state == qm.DISARMED and not o.left.brake and not o.operational


def test_a_driver_stopped_from_outside_faults_below_the_voltage_cap():
    # a moderate command (well under the cap) and a wheel that does not turn - what an
    # externally disabled analog driver looks like - must still latch
    wl = qm.WheelLoop(P, invert=False, name="left")
    wl.reset(-1.0)
    t, fault, out = 0.0, None, None
    while t < 5.0 and not fault:
        out = wl.step(t, 1.0, 0.0, True)
        fault = out.fault
        t += 0.02
    assert fault and "stalled" in fault
    assert out.volts < P.v_max
    assert P.stall_s <= t < P.stall_s + 0.2
    # a slow creep command (below 4 x zero_rad_s) never trips it
    wl.reset(-1.0)
    assert all(wl.step(i * 0.02, 0.15, 0.0, True).fault is None for i in range(300))
