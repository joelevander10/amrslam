"""The follower's speed ramp is an S-curve at both ends (2026-10-07).

Before the taper the acceleration built at the jerk limit but was cut from the
full rate to zero in one tick on arrival. Every speed change the tracked product
makes - start, stop, slow zone, RFID speed toggle - goes through this ramp, and
the follower's rate must stay under the mux's AUTO a_max (0.3 m/s^2) so the mux
passes it through instead of reshaping it into a trapezoid.
"""
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_line")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from agv_core import config as vehicle  # noqa: E402

from amr_line import autopilot, runtime  # noqa: E402

runtime.load_from_profile()
DT = 0.02
MUX_AUTO_A_MAX = 0.3  # base.launch.py a_max, m/s^2


def profile(start_rpm, target_rpm):
    f = autopilot.LineFollower()
    f._v_rpm, f._a_rpm_s = start_rpm, 0.0
    v, a = [start_rpm], [0.0]
    for _ in range(int(10.0 / DT)):
        v.append(f._ramp(target_rpm, DT))
        a.append((v[-1] - v[-2]) / DT)
        if v[-1] == target_rpm and f._a_rpm_s == 0.0:
            break
    return v, a


@pytest.mark.parametrize("start,target", [
    (0.0, "AUTO_RPM"), ("AUTO_RPM", "AUTO_HIGH_RPM"), ("AUTO_HIGH_RPM", "AUTO_RPM"), ("AUTO_RPM", 0.0)])
def test_jerk_limited_at_both_ends(start, target):
    s = getattr(vehicle, start) if isinstance(start, str) else start
    t = getattr(vehicle, target) if isinstance(target, str) else target
    v, a = profile(s, t)
    assert v[-1] == t, "reaches the target"
    assert all(min(s, t) - 1e-6 <= x <= max(s, t) + 1e-6 for x in v), "no overshoot"
    step = vehicle.RAMP_JERK_RPM_S2 * DT
    worst = max(abs(a[k] - a[k - 1]) for k in range(1, len(a)))
    assert worst <= 1.2 * step, f"accel stepped {worst:.0f} r/min/s in one tick (jerk step {step:.0f})"


def test_follower_rate_is_under_the_mux_auto_limit():
    assert vehicle.RAMP_ACCEL_RPM_S * vehicle.MPS_PER_RPM <= MUX_AUTO_A_MAX


def test_measured_station_stop_keeps_its_constant_rate():
    """No taper on a measured stop: the rate was solved for constant deceleration."""
    f = autopilot.LineFollower()
    f._v_rpm, f._a_rpm_s = vehicle.AUTO_RPM, -2000.0
    rate = 2000.0
    before = f._v_rpm
    f._ramp(0.0, DT, accel_limit=rate)
    assert abs((before - f._v_rpm) / DT - rate) < 1e-6


def _ramp_rpm(rate_limit):
    f = autopilot.LineFollower()
    f._v_rpm, f._a_rpm_s = vehicle.AUTO_HIGH_RPM, 0.0
    v = [f._v_rpm]
    for _ in range(int(10.0 / DT)):
        v.append(f._ramp(vehicle.AUTO_RPM, DT, rate_limit=rate_limit))
        if v[-1] == vehicle.AUTO_RPM and f._a_rpm_s == 0.0:
            break
    return v


def test_change_rate_only_ever_slows_the_profile_ramp():
    """high_ramp_s (2026-10-08): None and a faster rate are the profile ramp."""
    assert _ramp_rpm(None) == _ramp_rpm(10 * vehicle.RAMP_ACCEL_RPM_S)
    full = vehicle.AUTO_HIGH_RPM - vehicle.AUTO_RPM
    slow = _ramp_rpm(full / 2.0)
    secs = (len(slow) - 1) * DT
    assert 2.0 <= secs <= 2.6, f"a 2 s change took {secs:.2f} s"
    worst = max(abs((slow[k] - slow[k - 1]) / DT) for k in range(1, len(slow)))
    assert worst <= full / 2.0 + 1e-6


def test_cruise_ceiling_never_raises_the_speed():
    """The U-turn creep is a ceiling: a higher one leaves the zone's cruise alone."""
    sensor = {"tracks": [{"index": 2, "pos_mm": 0, "width": 10}], "has_track": True, "nlcp": 2}
    f = autopilot.LineFollower()
    _, _, d = f.update(sensor, 0.0, DT, True, cruise_rpm=10 * vehicle.AUTO_RPM)
    assert d["speed_target_rpm"] == vehicle.AUTO_RPM
    _, _, d = f.update(sensor, 0.0, DT, True, cruise_rpm=300.0)
    assert d["speed_target_rpm"] == 300.0


def test_an_urgent_drop_has_no_jerk_build_up():
    """A late high-zone exit (2026-10-08): the full rate from the first tick, constant,
    and it lands on NORMAL without overshoot."""
    f = autopilot.LineFollower()
    f._v_rpm, f._a_rpm_s = vehicle.AUTO_HIGH_RPM, 0.0
    rate = 0.95 * vehicle.DECEL_RPM_S
    v1 = f._ramp(vehicle.AUTO_RPM, DT, accel_limit=rate, no_jerk=True)
    assert vehicle.AUTO_HIGH_RPM - v1 == pytest.approx(rate * DT)
    for _ in range(100):
        v = f._ramp(vehicle.AUTO_RPM, DT, accel_limit=rate, no_jerk=True)
    assert v == vehicle.AUTO_RPM and f._a_rpm_s == 0.0
