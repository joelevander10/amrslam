"""Control law: the plant simulation and the guards on the tick.

Ported from `gy-demo:tests/test_control.py` against the new package layout.
Every check, threshold and print is the original's; only the import block
changed. `config` is `amr_line.runtime` (the namespace the engine is handed),
`kinematics` and `motion` come from `agv_core`, and the rig lives in
`harness.py` next to this file rather than in `tests/helpers.py`.

Runs two ways: `python3.11 test_control.py` prints the original PASS/FAIL
report, and under pytest `test_all_control_checks_pass` runs the same list and
asserts the shared FAIL list came back empty.
"""
import os
import sys

# The rig sits beside this file. pytest imports this module as part of the
# `test` package (there is an __init__.py), which puts the PACKAGE root on
# sys.path and not this directory, so a bare `import harness` needs help.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# `ROOT` is unused here but is part of what the original module imported; it is
# kept so a later ported check does not have to re-derive where it lives.
# (agv_core.motion was deleted 2026-10-02 with the dead manual.* speed keys.)
from agv_core import kinematics  # noqa: E402,F401
from harness import FAIL, NO_TRACK, ROOT, check, sensor, simulate  # noqa: E402,F401

from amr_line import autopilot  # noqa: E402
from amr_line import runtime as config  # noqa: E402


def test_step_response():
    print("\nstep response (30 mm), real slew limit + 20 ms transport delay")
    # Each gain pair at the speed it is used at (2026-10-08): the corner pair at NORMAL,
    # the straight pair at HIGH. simulate() pins AUTO_RPM to the speed under test, and
    # at or above AUTO_HIGH_RPM the follower uses K_RATIO/KD - so the pair is set there.
    k, kd = config.K_RATIO, config.KD
    cases = ((config.AUTO_RPM, "NORMAL", k, kd),
             (config.AUTO_HIGH_RPM, "HIGH", config.HIGH_K_RATIO, config.HIGH_KD))
    try:
        for rpm, label, kr, kdd in cases:
            config.K_RATIO, config.KD = kr, kdd
            zeta = autopilot.predicted_zeta()
            print(f"  {label}: K_RATIO={kr} KD={kdd} -> predicted zeta {zeta:.3f}")
            over, settle, rms = simulate(rpm)
            d = (f"overshoot {over:.1f} mm, settle "
                 f"{f'{settle:.2f} s' if settle else 'NEVER'}, tail {rms:.2f} mm")
            check(f"converges at {rpm:.0f} r/min ({label})", settle is not None, d)
            check(f"overshoot bounded at {rpm:.0f} r/min", over < 25.0, d)
            check(f"tail settled at {rpm:.0f} r/min", rms < 3.0, d)
    finally:
        config.K_RATIO, config.KD = k, kd


def test_divergence_is_detectable():
    """The rig must be able to SEE instability, or the passes above mean nothing."""
    print("\nnegative control: an over-high gain must diverge")
    k, kd = config.K_RATIO, config.KD
    try:
        config.K_RATIO, config.KD = 100.0, 10.0
        over, settle, _ = simulate(2546.0, seconds=8.0)
        check("K_RATIO=100 at 0.8 m/s diverges as predicted",
              settle is None or over > 50.0, f"overshoot {over:.0f} mm")
    finally:
        config.K_RATIO, config.KD = k, kd


# ---------------------------------------------------------------------------
# guards
# ---------------------------------------------------------------------------

def test_a_track_swap_cannot_become_a_yaw_spike():
    """A jump in the reported position is not motion, and must not reach the
    derivative as though it were.

    Run 0020, t=31.08: at the merge after the U-turn the followed position
    jumped ~120 mm between two tapes. The slew guard turned that into three
    consecutive 40 mm steps - 2 m/s of apparent lateral velocity, eight times
    what this vehicle can do - and the D term turned THAT into 6.66 rad/s of
    commanded yaw, slamming the wheels to 0 and 1177 r/min. Then the mirror
    image 0.4 s later. The P term was 0.26 of it; the rest was all derivative.
    """
    print("\na track swap must not become a yaw spike")

    def sensor(mm):
        return {"tracks": [{"index": 2, "pos_mm": mm, "width": 10}],
                "has_track": True}

    # The measured shape: settled on one tape, five ticks with the leftmost
    # track off-sensor, then the other tape ~120 mm away.
    seq = [39.0] * 10 + [None] * 5 + [-81.0] * 14

    f = autopilot.LineFollower()
    f.reset()
    # Run 0020's speed (800 r/min, the slow zone then, with the corner gains K 25 / Kd 5 -
    # which is what the follower uses below NORMAL since 2026-10-08). Kp = K_RATIO*v, so
    # this pins the D-term mechanism at the speed the log was taken.
    saved = config.AUTO_RPM
    config.configure(AUTO_RPM=800.0)
    f._v_rpm = 800.0
    peak_omega = 0.0
    peak_d = 0.0
    try:
        for mm in seq:
            # 300 mm is beyond sensor_max_mm, which is how the real frames read.
            _, _, d = f.update(sensor(300.0 if mm is None else mm), 0.0, 0.02,
                               True, "left", False)
            peak_omega = max(peak_omega, abs(d["omega_cmd"]))
            peak_d = max(peak_d, abs(d["d"]))
    finally:
        config.configure(AUTO_RPM=saved)

    check("the 120 mm swap does not produce the logged 6.66 rad/s",
          peak_omega < 2.0, f"peak |omega| {peak_omega:.2f} rad/s")
    check("...and the derivative is no longer what drives it",
          peak_d < 1.5, f"peak |d| {peak_d:.2f}")

    # The mechanism, asserted directly: a rate the vehicle cannot produce is
    # clamped to one it can.
    f2 = autopilot.LineFollower()
    f2.reset()
    f2._last_e_m = 0.0
    v = 0.25
    _, _, _, d_big = f2._pid(0.040, v, 0.02)      # 40 mm in one tick = 2 m/s
    lim = autopilot.LATERAL_RATE_FACTOR * v + 0.01
    check("an impossible lateral rate is clamped to a possible one",
          abs(d_big) <= config.KD * lim + 1e-9,
          f"d {d_big:.3f} vs limit {config.KD * lim:.3f}")

    # Real steering must still get through untouched.
    f3 = autopilot.LineFollower()
    f3.reset()
    f3._last_e_m = 0.0
    slow_mm = v * 0.02 * 0.3 * 1000.0             # 30% of the physical limit
    _, _, _, d_ok = f3._pid(slow_mm / 1000.0, v, 0.02)
    expect = config.KD * (0.02 / (config.TAU_D_S + 0.02)) * (slow_mm / 1000.0 / 0.02)
    check("a genuine correction is not clamped",
          abs(d_ok - expect) < 1e-6, f"{d_ok:.5f} vs {expect:.5f}")

    # And a gap must not be differentiated over one tick's dt either.
    f4 = autopilot.LineFollower()
    f4.reset()
    f4._v_rpm = config.AUTO_RPM
    for _ in range(6):
        f4.update(sensor(10.0), 0.0, 0.02, True)
    for _ in range(5):
        f4.update({"tracks": [], "has_track": False}, 0.0, 0.02, True)
    _, _, d_after = f4.update(sensor(30.0), 0.0, 0.02, True)
    check("the first sample after a gap primes instead of differentiating",
          d_after["d"] == 0.0, f"d {d_after['d']:.4f}")


def test_gains_follow_the_speed():
    """2026-10-08: the gains ride the speed between the corner pair at NORMAL and the
    straight pair at HIGH - no timer, so no gain/speed pair outside the two tuned ends.

    The zone-tag step that run 0020 showed (k_ratio 25 -> 11.3 in one tick at a tag)
    cannot recur: the speed change is a ramp, and the gain moves with it.
    """
    print("\ngains follow the speed")
    on_tape = {"tracks": [{"index": 2, "pos_mm": 5.0, "width": 10}], "has_track": True}
    lo, hi = config.AUTO_RPM, config.AUTO_HIGH_RPM
    seen = []
    for v in (0.0, lo, (lo + hi) / 2.0, hi, hi * 1.2):
        f = autopilot.LineFollower()
        f.reset()
        f._v_rpm = v
        _, _, d = f.update(on_tape, 0.0, 0.02, True)
        seen.append(d["k_used"])
    check("below and at NORMAL: the corner pair", seen[0] == seen[1] == config.K_RATIO, str(seen))
    check("half way: half way", abs(seen[2] - (config.K_RATIO + config.HIGH_K_RATIO) / 2) < 0.2,
          f"{seen[2]:.2f}")
    check("at and above HIGH: the straight pair", abs(seen[3] - config.HIGH_K_RATIO) < 0.05
          and abs(seen[4] - config.HIGH_K_RATIO) < 1e-9, str(seen))
    f = autopilot.LineFollower()
    f.reset()
    f._v_rpm = hi
    _, _, d = f.update(on_tape, 0.0, 0.02, True, high=False)
    check("dropping HIGH does not step the gain: it waits for the speed",
          abs(d["k_used"] - config.HIGH_K_RATIO) < 0.5, f"{d['k_used']:.2f}")


def test_no_derivative_kick_on_reset():
    print("\nreset() must not differentiate a standing error against a fake zero")
    f = autopilot.LineFollower()
    f.reset()
    _, _, d1 = f.update(sensor(40.0), 0.0, 0.02, True)
    check("first tick after reset has zero D term", d1["d"] == 0.0,
          f"d={d1['d']:.4f}")
    _, _, d2 = f.update(sensor(40.0), 0.0, 0.02, True)
    check("second tick with unchanged error still has ~zero D",
          abs(d2["d"]) < 1e-9, f"d={d2['d']:.6f}")


def test_sensor_slew_guard():
    print("\nsensor guards")
    f = autopilot.LineFollower()
    f.reset()
    f.update(sensor(10.0), 0.0, 0.02, True)
    _, _, d = f.update(sensor(95.0), 0.0, 0.02, True)   # 85 mm jump
    check("a glitch jump is clamped, not accepted", d["guard"] == "slew",
          f"guard={d['guard']!r} e_used={d['e_used']}")
    check("clamped to the step limit",
          abs(d["e_used"] - (10.0 + config.SENSOR_MAX_STEP_MM)) < 1e-6,
          f"e_used={d['e_used']}")

    f.reset()
    _, _, d = f.update(sensor(500.0), 0.0, 0.02, True)  # beyond the sensor
    check("an implausible reading is discarded", d["guard"] == "discard"
          and not d["has_track"], f"guard={d['guard']!r}")


def test_conditional_integration():
    print("\nconditional integration (integrate INSIDE the deadband only)")
    ki = config.KI
    try:
        config.KI = 1.0
        f = autopilot.LineFollower()
        f.reset()
        big = config.TI_DEADBAND_MM + 30.0
        for _ in range(50):
            f.update(sensor(big), 0.0, 0.02, True)
        check("integrator frozen outside the deadband", f._integral == 0.0,
              f"integral={f._integral:.4f}")
        small = config.TI_DEADBAND_MM / 2.0
        for _ in range(50):
            f.update(sensor(small), 0.0, 0.02, True)
        # Sign of the accumulation follows INVERT_ERROR; only the fact that it
        # accumulates at all is under test here.
        check("integrator accumulates inside the deadband", f._integral != 0.0,
              f"integral={f._integral:.4f}")
    finally:
        config.KI = ki


def test_line_loss_grace():
    print("\nline loss")
    f = autopilot.LineFollower()
    f.reset()
    for _ in range(100):
        f.update(sensor(0.0), 0.0, 0.02, True)
    cruising = f._v_rpm
    check("reaches cruise before the line drops", cruising > 100.0,
          f"v_base={cruising:.0f}")

    _, _, d = f.update(NO_TRACK, 0.0, 0.02, True)
    check("first missing frame coasts, does not stop", d["state"] == "coast",
          f"state={d['state']}")

    # The budget is a distance, so how long it lasts depends on how fast we are
    # going. Drive until it trips rather than assuming a tick count.
    for _ in range(500):
        _, _, d = f.update(NO_TRACK, 0.0, 0.02, True)
        if d["state"] == "line_lost":
            break
    check("grace expires into a stop", d["state"] == "line_lost",
          f"state={d['state']}")

    for _ in range(300):
        _, _, d = f.update(NO_TRACK, 0.0, 0.02, True)
    check("ramps down to zero after line loss", abs(d["v_base"]) < 1.0,
          f"v_base={d['v_base']:.2f}")


def test_stale_sensor_is_not_line_loss():
    print("\nstale sensor is a different failure from a tape gap")
    f = autopilot.LineFollower()
    f.reset()
    _, _, d = f.update(sensor(0.0), config.SENSOR_TIMEOUT_S + 0.05, 0.02, True)
    check("stale frames stop immediately, no grace",
          d["state"] == "sensor_lost", f"state={d['state']}")
    _, _, d = f.update(sensor(0.0), None, 0.02, True)
    check("never-seen sensor is also a stop", d["state"] == "sensor_lost")


def test_joint_saturation_preserves_ratio():
    print("\nsaturation scales BOTH wheels (turn ratio preserved)")
    f = autopilot.LineFollower()
    f.reset()
    v_rpm, omega = 3900.0, -3.0            # deliberately past the ceiling
    left, right, scale = f._to_wheels(v_rpm, omega)
    check("scale factor applied", scale < 1.0, f"scale={scale:.3f}")
    check("neither wheel exceeds the motor limit",
          max(abs(left), abs(right)) <= config.MOTOR_MAX_RPM + 1e-6,
          f"L={left:.0f} R={right:.0f}")

    # Uniform scaling multiplies v and omega by the same factor, so omega/v -
    # the radius of the arc being driven - must come out unchanged. Compare
    # against the REQUESTED arc, not against some other command.
    want = omega / kinematics.rpm_to_mps(v_rpm)
    v_out, w_out = kinematics.wheels_to_body(left, right)
    check("arc (omega/v) preserved through scaling",
          abs(want - w_out / v_out) < 1e-6,
          f"requested {want:.5f}, got {w_out / v_out:.5f}")


def test_inner_wheel_floor():
    print("\ninner wheel floor limits the DIFFERENTIAL, not one wheel")
    f = autopilot.LineFollower()
    f.reset()
    left, right, _ = f._to_wheels(400.0, -5.0)     # demands a huge differential
    lo = config.INNER_WHEEL_MIN_RPM
    check("inner wheel held at or above the floor",
          min(left, right) >= lo - 1e-6, f"L={left:.1f} R={right:.1f} floor={lo}")
    check("base speed unchanged by the floor",
          abs((left + right) / 2.0 - 400.0) < 1e-6,
          f"mean={(left + right) / 2.0:.2f}")


def test_kinematics_roundtrip():
    print("\nkinematics")
    l, r = kinematics.body_to_wheels(0.25, 0.3)  # noqa: E741  (verbatim from gy-demo; renaming would edit the port)
    v, w = kinematics.wheels_to_body(l, r)
    check("body_to_wheels and wheels_to_body are inverses",
          abs(v - 0.25) < 1e-9 and abs(w - 0.3) < 1e-9, f"v={v:.6f} w={w:.6f}")
    check("2546 r/min is 0.8 m/s",
          abs(kinematics.rpm_to_mps(2546) - 0.8) < 0.001,
          f"{kinematics.rpm_to_mps(2546):.4f} m/s")
    check("yaw accel limit at 6083h=2000 is 2.59 rad/s^2",
          abs(kinematics.max_yaw_accel(2000) - 2.586) < 0.01,
          f"{kinematics.max_yaw_accel(2000):.3f}")


def test_grace_is_a_distance_not_a_time():
    """The regression that started all this.

    line_loss_grace was a TIME, so raising cruise 800 -> 2000 silently grew the
    blind travel from 75 mm to 188 mm and nobody noticed. As a distance the
    budget holds across speeds - the coast lasts a shorter TIME when moving
    faster, but covers the same GROUND, which is the quantity that matters.
    """
    print("\nline-loss grace is budgeted in distance, not time")

    def coast(base_rpm):
        saved = config.AUTO_RPM
        config.AUTO_RPM = base_rpm
        try:
            f = autopilot.LineFollower()
            f.reset()
            for _ in range(600):                  # reach cruise
                f.update(sensor(0.0), 0.0, 0.02, True)
            v = f._v_rpm
            secs, dist = 0.0, 0.0
            for _ in range(2000):
                _, _, d = f.update(NO_TRACK, 0.0, 0.02, True)
                if d["state"] != "coast":
                    break
                secs += 0.02
                dist += kinematics.rpm_to_mps(d["v_base"]) * 0.02
            return v, secs, dist
        finally:
            config.AUTO_RPM = saved

    v_slow, t_slow, d_slow = coast(800.0)
    v_fast, t_fast, d_fast = coast(2000.0)
    check("both speeds reached cruise", v_slow > 700 and v_fast > 1900,
          f"{v_slow:.0f} / {v_fast:.0f} r/min")
    check("coasts the same DISTANCE at either speed",
          abs(d_slow - d_fast) < 0.02,
          f"{d_slow*1000:.0f} mm at 800 vs {d_fast*1000:.0f} mm at 2000")
    check("distance is about the configured budget",
          abs(d_fast - config.LINE_LOSS_GRACE_M) < 0.03,
          f"{d_fast*1000:.0f} mm vs budget {config.LINE_LOSS_GRACE_M*1000:.0f} mm")
    check("coasts for LESS TIME when moving faster", t_fast < t_slow * 0.75,
          f"{t_slow:.2f} s at 800 vs {t_fast:.2f} s at 2000")

    # Standstill backstop: no distance accrues, so only the time ceiling can
    # end it. Drive the follower with the ramp pinned at zero.
    f = autopilot.LineFollower()
    f.reset()
    secs = 0.0
    for _ in range(2000):
        f._v_rpm = 0.0                      # pin: stopped, cannot travel
        _, _, d = f.update(NO_TRACK, 0.0, 0.02, True)
        secs += 0.02
        if d["state"] != "coast":
            break
    check("a stationary AGV still gives up, on the time ceiling",
          d["state"] == "line_lost"
          and secs <= config.LINE_LOSS_GRACE_MAX_S + 0.05,
          f"{d['state']} after {secs:.2f} s "
          f"(ceiling {config.LINE_LOSS_GRACE_MAX_S:.2f} s)")


def test_speed_reduction_is_a_fraction():
    """Speed reduction must shed the same PROPORTION of base at any speed."""
    print("\nspeed reduction scales with base speed")

    def shed(base_rpm, err_mm=20.0):
        f = autopilot.LineFollower()
        f.reset()
        f._v_rpm = base_rpm
        for _ in range(200):                # let the SR low-pass settle
            red = f._reduce_speed(err_mm / 1000.0, 0.02)
        return red

    r800, r2000 = shed(800.0), shed(2000.0)
    check("20 mm error sheds the same fraction at 800 and 2000",
          abs(r800 / 800.0 - r2000 / 2000.0) < 0.005,
          f"{r800/800*100:.1f}% vs {r2000/2000*100:.1f}%")
    check("and that fraction is the configured one",
          abs(r2000 / 2000.0 - 20.0 * config.SR_POS_FRAC) < 0.005,
          f"{r2000/2000*100:.1f}% vs {20.0*config.SR_POS_FRAC*100:.1f}%")
    check("the cap still bounds it", shed(2000.0, err_mm=5000.0)
          <= 2000.0 * config.SR_CAP + 1e-6)


TESTS = [
    test_step_response,
    test_divergence_is_detectable,
    test_a_track_swap_cannot_become_a_yaw_spike,
    test_gains_follow_the_speed,
    test_no_derivative_kick_on_reset,
    test_sensor_slew_guard,
    test_conditional_integration,
    test_line_loss_grace,
    test_stale_sensor_is_not_line_loss,
    test_joint_saturation_preserves_ratio,
    test_inner_wheel_floor,
    test_kinematics_roundtrip,
    test_grace_is_a_distance_not_a_time,
    test_speed_reduction_is_a_fraction,
]


def test_all_control_checks_pass():
    """The pytest verdict: run the whole list, then judge the shared FAIL.

    The ported functions report by printing and appending to `harness.FAIL`
    rather than by raising, which is what lets `python3.11 test_control.py`
    print the original whole-suite report. pytest also collects each of them
    individually - they are `test_*` functions and they pass by not raising -
    so this one exists to turn a failed CHECK into a failed test.
    """
    del FAIL[:]
    for fn in TESTS:
        fn()
    assert FAIL == [], "failed checks:\n  " + "\n  ".join(FAIL)


if __name__ == "__main__":
    for fn in TESTS:
        fn()
    print(f"\n{'ALL PASS' if not FAIL else 'FAILURES: ' + ', '.join(FAIL)}")
    sys.exit(1 if FAIL else 0)
