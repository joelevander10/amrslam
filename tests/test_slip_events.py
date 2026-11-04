"""Slip and snatch candidates (agv_core/slip_events.py) on synthetic 50 Hz series.

The point of each case is the one thing that would make the report lie: a clean
start-up flagged, a snatch missed, a braking vehicle called slipping, a steady
calibration error reported as slip, or an event's localization consequence taken
from the wrong side of it.
"""
import math

from helpers import check

from agv_core import slip_events as se

DT = 0.02


def _times(t_end):
    return [round(i * DT, 6) for i in range(int(round(t_end / DT)) + 1)]


def _run(t_end=4.0, v_max=0.3, ramp_s=2.0, accel_nm=20.0, cruise_nm=8.0, sign=1.0):
    """A start-up: speed ramps to v_max over ramp_s, torque eases from accel_nm to
    cruise_nm over 0.5 s once the ramp ends. Encoder speed = command, no turning."""
    ts = _times(t_end)
    cmd = [(t, sign * v_max * min(t / ramp_s, 1.0)) for t in ts]
    torque = []
    for t in ts:
        if t < ramp_s:
            nm = accel_nm
        else:
            nm = accel_nm + (cruise_nm - accel_nm) * min((t - ramp_s) / 0.5, 1.0)
        torque.append([t, sign * nm, True])
    wheels = [(t, v, 0.0, True) for t, v in cmd]
    gyro = [(t, 0.0) for t in ts]
    return torque, cmd, wheels, gyro


def _set(torque, a, b, nm):
    for row in torque:
        if a - 1e-9 <= row[0] < b - 1e-9:
            row[1] = nm


def test_clean_runs():
    print("\nslip events: clean runs stay quiet")

    torque, cmd, wheels, gyro = _run()
    check("a clean start-up gives no events", se.detect(torque, cmd, wheels, gyro) == [])

    torque, cmd, wheels, gyro = _run(sign=-1.0)
    check("a clean reverse start-up gives no events", se.detect(torque, cmd, wheels, gyro) == [])

    # Braking: the command falls and the torque goes with it, fast. Not a slip.
    ts = _times(3.0)
    cmd = [(t, max(0.3 - 0.3 * t, 0.0)) for t in ts]
    torque = [[t, 20.0 if t < 0.5 else 2.0, True] for t in ts]
    wheels = [(t, v, 0.0, True) for t, v in cmd]
    check("torque falling while the command falls is not a collapse",
          se.collapses(torque, cmd) == [])

    # Standing still, a big torque step (holding against a push) is not a snatch.
    ts = _times(1.0)
    still = [(t, 0.0) for t in ts]
    torque = [[t, 30.0 if t >= 0.5 else 0.0, True] for t in ts]
    check("a torque step while standing still is not a snatch",
          se.snatches(torque, still, [(t, 0.0, 0.0, True) for t in ts]) == [])


def test_snatch():
    print("\nslip events: A, the snatch")

    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 1.06, 35.0)  # +15 N·m for three samples
    ev = se.detect(torque, cmd, wheels, gyro)
    sn = [e for e in ev if e.kind == "snatch"]
    check("a +15 N·m spike while accelerating is one snatch, and only that",
          [e.kind for e in ev] == ["snatch"], str(ev))
    check("...at the step", sn and abs(sn[0].t - 1.0) < 1e-6, str(sn))
    check("...sized by the jump, sign kept", sn and abs(sn[0].size - 15.0) < 1e-6, str(sn))
    check("...and the speed at the time recorded", sn and abs(sn[0].v_mps - 0.15) < 1e-6)

    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 4.1, 4.0)  # -16 N·m, towards zero and staying: not an impact
    check("a sudden fall towards zero is not a snatch", se.snatches(torque, cmd, wheels) == [])

    # Braking, and the trolley runs into the vehicle: braking torque jumps from -10 to -25.
    ts = _times(3.0)
    cmd = [(t, max(0.3 - 0.15 * t, 0.0)) for t in ts]
    torque = [[t, -25.0 if 1.0 <= t < 1.06 else -10.0, True] for t in ts]
    sn = se.snatches(torque, cmd, [(t, v, 0.0, True) for t, v in cmd])
    check("braking torque jumping while stopping is a snatch, signed backwards",
          len(sn) == 1 and abs(sn[0].size + 15.0) < 1e-6, str(sn))

    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 1.06, 25.0)  # +5 N·m: under the threshold
    check("a jump under SNATCH_NM is not a snatch", se.snatches(torque, cmd, wheels) == [])

    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 1.06, 35.0)
    for row in torque:
        if 0.98 < row[0] < 1.08:
            row[2] = False
    check("invalid samples do not make a snatch", se.snatches(torque, cmd, wheels) == [])

    torque, cmd, wheels, gyro = _run()
    sparse = [row for row in torque if abs(row[0] - 1.0) > 0.25]
    _set(sparse, 1.26, 4.1, 40.0)
    check("samples further apart than MAX_GAP_S are not compared",
          se.snatches(sparse, cmd, wheels) == [])


def test_collapse():
    print("\nslip events: B, the collapse, and snatch -> slip")

    # Snatch at 1.0 to 35 N·m, the tyre lets go at 1.1 and torque falls to 10 for good
    # (no regrip here: that would be a second snatch, tested below).
    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 1.1, 35.0)
    _set(torque, 1.1, 4.1, 10.0)
    ev = se.detect(torque, cmd, wheels, gyro)
    kinds = [e.kind for e in ev]
    check("a spike then a fall while accelerating is a snatch then a snatch_slip",
          kinds == ["snatch", "snatch_slip"], str(kinds))
    slip = [e for e in ev if e.kind == "snatch_slip"]
    check("...the slip sized against the peak", slip and abs(slip[0].size - (10.0 - 35.0)) < 1e-6,
          str(slip))
    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 1.1, 35.0)
    _set(torque, 1.1, 1.5, 10.0)  # grips again at 1.5: back to 20
    kinds = [e.kind for e in se.detect(torque, cmd, wheels, gyro)]
    check("the tyre gripping again after the slip is a second snatch",
          kinds == ["snatch", "snatch_slip", "snatch"], str(kinds))

    # The tyre lets go without a snatch: torque at 20 halves within one sample.
    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 1.4, 8.0)
    co = se.collapses(torque, cmd)
    check("a fall to 40 % while the command rises is a plain collapse",
          len(co) == 1 and co[0].kind == "collapse", str(co))

    # Same in reverse: driving torque is negative, the fall is towards zero.
    torque, cmd, wheels, gyro = _run(sign=-1.0)
    _set(torque, 1.0, 1.4, -8.0)
    co = se.collapses(torque, cmd)
    check("a collapse in reverse is caught the same way", len(co) == 1, str(co))

    # The end of a jerk-limited ramp: the command still rises while the acceleration
    # torque eases off over 0.5 s. Torque ends well under 60 % of its level, but slowly.
    ts = _times(4.0)
    cmd = [(t, 0.1 * t) for t in ts]
    torque = [[t, 20.0 - 12.0 * min(max(t - 1.0, 0.0) / 0.5, 1.0), True] for t in ts]
    check("torque easing off over 0.5 s with the command still rising is not a collapse",
          se.collapses(torque, cmd) == [])

    # A fall to 70 %: not enough.
    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 1.4, 14.0)
    check("a fall to 70 % of the peak is not a collapse", se.collapses(torque, cmd) == [])


def test_yaw():
    print("\nslip events: C, wheel vs gyro yaw rate")

    torque, cmd, wheels, gyro = _run()
    wheels = [(t, v, 0.12 if 3.0 <= t < 3.2 else 0.0, ok) for t, v, _w, ok in wheels]
    ev = se.yaw_mismatches(wheels, gyro)
    check("a 0.2 s wheel turn the gyro did not feel is one yaw event", len(ev) == 1, str(ev))
    check("...near its start", ev and 3.0 <= ev[0].t < 3.1, str(ev))
    check("...sized near the full residual", ev and abs(ev[0].size - 0.12) < 0.01, str(ev))

    wheels2 = [(t, v, 0.12 if 3.0 <= t < 3.04 else 0.0, ok) for t, v, _w, ok in wheels]
    check("a two-sample blip is too short to report", se.yaw_mismatches(wheels2, gyro) == [])

    # A steady 5 % over-read on a turning run: calibration, not slip.
    ts = _times(20.0)
    gyro = [(t, 0.3 * math.sin(t)) for t in ts]
    wheels = [(t, 0.2, 1.05 * 0.3 * math.sin(t), True) for t in ts]
    k = se.yaw_gain(wheels, gyro)
    check("a steady wheel/gyro ratio is fitted as the gain", k is not None and abs(k - 1.05) < 0.005,
          str(k))
    check("...and gives no yaw events", se.yaw_mismatches(wheels, gyro) == [])
    check("too little turning gives no gain",
          se.yaw_gain([(t, 0.2, 0.0, True) for t in ts], [(t, 0.0) for t in ts]) is None)

    # Invalid wheel samples are skipped, not read as a zero-rate turn against the gyro.
    gyro = [(t, 0.2) for t in ts]
    wheels = [(t, 0.2, 0.0 if 5.0 <= t < 6.0 else 0.2, not (5.0 <= t < 6.0)) for t in ts]
    check("invalid wheel samples make no yaw event", se.yaw_mismatches(wheels, gyro, gain=1.0) == [])


def test_consequence():
    print("\nslip events: what each event did to localization")

    torque, cmd, wheels, gyro = _run()
    _set(torque, 1.0, 1.06, 35.0)
    ts = _times(4.0)
    # AMCL pulls map->odom 8 cm and 2 deg at 1.5 s; a 3 cm shift BEFORE the event is not its.
    mo = [(t, 0.03 if t >= 0.5 else 0.0, 0.0, 0.0) for t in ts]
    mo = [(t, x + (0.08 if t >= 1.5 else 0.0), 0.0, math.radians(2.0) if t >= 1.5 else 0.0)
          for t, x, _y, _a in mo]
    loc = [(0.0, "READY"), (2.0, "LOST"), (3.6, "READY")]
    ev = se.detect(torque, cmd, wheels, gyro, mo, loc)
    check("the map->odom shift after the event is measured from just before it",
          ev and abs(ev[0].map_odom_m - 0.08) < 1e-6, str(ev))
    check("...with its heading part", ev and abs(ev[0].map_odom_deg - 2.0) < 1e-6)
    check("the worst localization state within AFTER_S is recorded",
          ev and ev[0].loc_worst == "LOST", str(ev))
    late = se.Event(3.7, "snatch", 15.0)
    se._consequence(late, se._S(mo), se._S(loc))
    check("a state that cleared before the event is not blamed on it", late.loc_worst == "READY")

    long_ts = _times(12.0)
    cruise = [(t, 0.3, 0.0, True) for t in long_ts]
    check("a constant map->odom gives a 0 m baseline",
          se.map_odom_baseline([(t, 1.0, 2.0, 0.3) for t in long_ts], cruise) == 0.0)
    check("too short a run gives no baseline",
          se.map_odom_baseline([(t, 1.0, 2.0, 0.3) for t in ts], wheels) is None)
    check("no map->odom gives no baseline", se.map_odom_baseline([], wheels) is None)


def test_top_and_merge():
    print("\nslip events: merging and the top-N list")

    hits = [se.Event(1.0, "snatch", 5.0), se.Event(1.1, "snatch", -9.0), se.Event(3.0, "snatch", 7.0)]
    merged = se._merge(hits)
    check("hits within MERGE_S merge into one event with the largest |size|",
          [(e.t, e.size) for e in merged] == [(1.0, -9.0), (3.0, 7.0)], str(merged))
    check("...lasting to the last hit", abs(merged[0].duration_s - 0.1) < 1e-9)
    check("merging does not modify its inputs", hits[0].size == 5.0 and hits[0].duration_s == 0.0)
    check("top() ranks by |size|", [e.size for e in se.top(hits, 2)] == [-9.0, 7.0])
    # Every sample of a 50 Hz series, three bumps in it: three peaks, not one chained event.
    dense = [se.Event(t, "yaw", 0.01 + sum(h * math.exp(-((t - c) / 0.05) ** 2)
                                            for c, h in ((2.0, 0.3), (5.0, -0.5), (8.0, 0.2))))
             for t in _times(10.0)]
    picked = se.top(dense, 3)
    check("top() on a dense series finds each bump's peak, largest first",
          [round(e.t, 2) for e in picked] == [5.0, 2.0, 8.0], str([(e.t, e.size) for e in picked]))


TESTS = [test_clean_runs, test_snatch, test_collapse, test_yaw, test_consequence,
         test_top_and_merge]


if __name__ == "__main__":
    for fn in TESTS:
        fn()
    import helpers
    print(f"\n{helpers.CHECKS[0]} checks, {len(helpers.FAIL)} failed")
