"""Shared rig for the tape-engine suite. No CAN, no hardware, no motion.

Ported from `gy-demo:tests/helpers.py`. The plant simulation, the sensor frame
shape, the shared FAIL/CHECKS state and `check()` are carried over unchanged -
same maths, same magic numbers - so a number produced here is comparable to a
number produced by the original suite. The only edits are the bootstrap below
and the name `config`, which on gy-demo was the profile loader and here is
`amr_line.runtime`, the plain namespace the engine is handed instead.

The plant simulation lives here because more than one module integrates against
it: it runs the REAL LineFollower against e_dot = v*theta + Ls*omega, including
the 1 mm sensor quantisation, the 50 Hz tick, the transport delay and the wheel
slew limit imposed by 6083h. That slew limit is what makes simulating worth the
trouble - an ideal plant that lets yaw follow the command instantly says
K_RATIO = 100 is fine, and on this vehicle that diverges at 0.8 m/s.

FAIL is shared state, deliberately. Every module appends to the same list so a
runner can report one verdict for the whole suite rather than eight.
"""
import math
import pathlib
import sys

# Repo root, so a check that reads a source file keeps working wherever the
# suite is run from. Two entries are needed in the new layout: the root carries
# `agv_core`, and the package dir carries `amr_line` (the ROS package is not
# installed when the suite runs straight out of the tree).
ROOT = pathlib.Path(__file__).resolve().parents[4]
PKG = pathlib.Path(__file__).resolve().parents[1]
for _d in (str(ROOT), str(PKG)):
    if _d not in sys.path:
        sys.path.insert(0, _d)

from agv_core import kinematics  # noqa: E402

from amr_line import autopilot  # noqa: E402
from amr_line import runtime as config  # noqa: E402

# The engine reads its constants at call time, so the namespace only has to be
# populated before the first tick - but every caller of this module ticks, so
# there is no reason to defer it past import.
if not config.is_loaded():
    config.load_from_profile()

FAIL = []

# Counted so a runner can pin the total. A single-element list rather than an
# int because every test module imports this by value.
CHECKS = [0]


def check(name, cond, detail=""):
    CHECKS[0] += 1
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"   {detail}" if detail else ""))
    if not cond:
        FAIL.append(name)


def sensor(pos_mm, n=1):
    """A TPDO1 decode shaped like canworker._sensor_json()."""
    return {"tracks": [{"index": i + 1, "pos_mm": pos_mm, "width": 10}
                       for i in range(n)],
            "has_track": True}


NO_TRACK = {"tracks": [], "has_track": False}


# ---------------------------------------------------------------------------
# plant simulation
# ---------------------------------------------------------------------------

def simulate(base_rpm, e0_mm=30.0, seconds=12.0, driver_accel=2000.0,
             delay_s=0.020, sub_dt=0.0005, at_speed=True):
    """Closed-loop step response. Returns (overshoot_mm, settle_s, tail_rms_mm).

    at_speed pre-loads the ramp so the disturbance lands while already cruising.
    That is the demanding case and the realistic one: starting from rest lets the
    PID null most of the error before the vehicle has any speed, which flatters
    high gains badly.
    """
    f = autopilot.LineFollower()
    f.reset()

    y, theta = e0_mm / 1000.0, 0.0
    nl_act = nr_act = 0.0
    nl_cmd = nr_cmd = 0.0
    queue = []

    # The follower always ramps toward AUTO_RPM, so the cruise speed under test
    # has to BE AUTO_RPM or the vehicle spends the run decelerating.
    saved_rpm = config.AUTO_RPM
    config.AUTO_RPM = base_rpm
    if at_speed:
        f._v_rpm = base_rpm
        nl_act = nr_act = base_rpm
        nl_cmd = nr_cmd = base_rpm

    try:
        return _integrate(f, y, theta, nl_act, nr_act, nl_cmd, nr_cmd, queue,
                          seconds, driver_accel, delay_s, sub_dt, e0_mm)
    finally:
        config.AUTO_RPM = saved_rpm


def _integrate(f, y, theta, nl_act, nr_act, nl_cmd, nr_cmd, queue,
               seconds, driver_accel, delay_s, sub_dt, meas_mm):
    t = 0.0
    next_tick = next_frame = 0.0
    over = 0.0
    settle = None
    tail = []

    while t < seconds:
        if t >= next_frame:                       # 100 Hz, quantised to 1 mm
            next_frame += 0.01
            # The MLS reports where the LINE is relative to the SENSOR, which is
            # the NEGATIVE of where the sensor is relative to the line. Getting
            # this backwards used to cancel against INVERT_ERROR=False and the
            # sim converged for the wrong reason; with the real polarity now
            # measured on the machine, the model has to match it.
            meas_mm = -round((y + config.SENSOR_LOOKAHEAD_M
                              * math.sin(theta)) * 1000.0)
        if t >= next_tick:                        # 50 Hz control tick
            next_tick += config.DT_NOMINAL_S
            l, r, _ = f.update(sensor(meas_mm), 0.0, config.DT_NOMINAL_S, True)  # noqa: E741  (verbatim from gy-demo; renaming would edit the port)
            queue.append((t + delay_s, l, r))
        while queue and queue[0][0] <= t:
            _, nl_cmd, nr_cmd = queue.pop(0)

        # 6083h slews each wheel; this is the constraint the ideal model misses.
        step = driver_accel * sub_dt
        nl_act += max(-step, min(step, nl_cmd - nl_act))
        nr_act += max(-step, min(step, nr_cmd - nr_act))

        # Forward speed comes from the wheels, not from a constant - otherwise
        # speed_reduction has no effect on the plant and the controller and the
        # vehicle disagree about how fast it is going.
        v, omega = kinematics.wheels_to_body(nl_act, nr_act)
        y += v * math.sin(theta) * sub_dt
        theta += omega * sub_dt
        t += sub_dt

        e = y + config.SENSOR_LOOKAHEAD_M * math.sin(theta)
        over = max(over, -e)
        if abs(e) < 0.002 and settle is None and t > 0.5:
            settle = t
        elif abs(e) >= 0.002:
            settle = None
        if t > seconds - 2.0:
            tail.append(e)

    rms = math.sqrt(sum(x * x for x in tail) / len(tail)) * 1000.0 if tail else 0.0
    return over * 1000.0, settle, rms
