"""Encoder-only blind run: plan maths, closed loop, odometry and panel rules.

Synthetic encoder only: an ideal plant turns commanded r/min into counts. None
of this says anything about the real encoders - measuring them is what the
page is for.
"""
import math
from unittest.mock import patch

from helpers import check

from agv_core import blindrun, config

MOTOR_CPR = 36000                      # 608Fh default, per motor revolution
CPR = MOTOR_CPR * config.GEAR_RATIO    # per wheel revolution
M_PER_COUNT = math.pi * config.WHEEL_DIA_M / CPR
TOL_COUNTS = config.BLIND_STOP_TOLERANCE_MM / 1000.0 / M_PER_COUNT
HALF = config.TRACK_M / 2.0
SPEED = {"speed_mps": 0.2}


class Plant:
    """Ideal drives: commanded r/min becomes counts, standstill is instant."""

    def __init__(self):
        self.lc = self.rc = 0.0

    def step(self, left, right, dt):
        self.lc += left / 60.0 * MOTOR_CPR * dt
        self.rc += right / 60.0 * MOTOR_CPR * dt

    @property
    def counts(self):
        return round(self.lc), round(self.rc)


def fly(segments, speed=SPEED, dt=0.02, steps=60000):
    planned = blindrun.plan(segments, speed, CPR)
    plant = Plant()
    run = blindrun.BlindRun(planned, plant.counts)
    last, worst = (0.0, 0.0), 0.0
    for _ in range(steps):
        cmd = run.update(plant.counts, dt, last == (0.0, 0.0))
        if run.phase == blindrun.RUNNING:
            seg = run.segment
            if seg["left_m"] and seg["right_m"]:
                worst = max(worst, abs(run.progress[0] / seg["left_m"]
                                       - run.progress[1] / seg["right_m"]))
        if run.phase in (blindrun.DONE, blindrun.ABORTED):
            break
        plant.step(*cmd, dt)
        last = cmd
    return run, worst


def refused(name, segments, speed=SPEED, cpr=CPR, expect=""):
    try:
        blindrun.plan(segments, speed, cpr)
    except ValueError as e:
        check(name, expect in str(e), str(e))
    else:
        check(name, False, "accepted")


def test_plan_maths():
    print("\nblind run plan maths")
    p = blindrun.plan([{"kind": "straight", "distance_m": 1.0}], SPEED, CPR)["segments"][0]
    want = round(CPR / (math.pi * config.WHEEL_DIA_M))
    check("1 m straight is counts_per_wheel_rev / (pi*D) on both wheels",
          p["counts"] == [want, want], f"{p['counts']} vs {want}")
    check("...and commands 1 m with no heading change",
          abs(p["commanded"]["distance_m"] - 1) < 1e-12 and p["commanded"]["heading_deg"] == 0)
    p = blindrun.plan([{"kind": "straight", "distance_m": -0.5}], SPEED, CPR)["segments"][0]
    check("a negative distance reverses both wheels", p["counts"][0] < 0 and p["counts"][1] < 0)

    p = blindrun.plan([{"kind": "pivot", "angle_deg": 90}], SPEED, CPR)["segments"][0]
    # Each wheel rolls an arc of (track/2)*angle; counts are that over pi*D.
    n = round(HALF * math.radians(90) / (math.pi * config.WHEEL_DIA_M) * CPR)
    check("a 90 deg pivot rolls each wheel (track/2)*angle, wheels opposed",
          abs(p["counts"][1] - n) <= 1 and abs(p["counts"][0] + n) <= 1)
    check("...and is +90 deg (counter-clockwise)", abs(p["commanded"]["heading_deg"] - 90) < 1e-9)

    p = blindrun.plan([{"kind": "arc", "radius_m": 1.0, "angle_deg": 90}], SPEED, CPR)["segments"][0]
    check("a left arc runs the right wheel faster by (R+T/2)/(R-T/2)",
          abs(p["right_m"] / p["left_m"] - (1 + HALF) / (1 - HALF)) < 1e-12)
    check("...and ends 1 m forward, 1 m left, heading +90",
          abs(p["commanded"]["dx_m"] - 1) < 1e-9 and abs(p["commanded"]["dy_m"] - 1) < 1e-9
          and abs(p["commanded"]["heading_deg"] - 90) < 1e-9)
    p = blindrun.plan([{"kind": "arc", "radius_m": -1.0, "angle_deg": 90}], SPEED, CPR)["segments"][0]
    check("a negative radius turns right", abs(p["commanded"]["heading_deg"] + 90) < 1e-9
          and abs(p["commanded"]["dy_m"] + 1) < 1e-9)
    p = blindrun.plan([{"kind": "arc", "radius_m": 1.0, "angle_deg": 90}],
                      {"motor_rpm": 500}, CPR)["segments"][0]
    check("the speed names the centre path; the outer wheel runs faster",
          abs(p["dom_rpm"] - 500 * (1 + HALF)) < 1e-9)

    p = blindrun.plan([{"kind": "pulses", "left": 1000, "right": 3000}], SPEED, CPR)["segments"][0]
    check("pulses pass through unchanged, in driver terms", p["counts"] == [1000, 3000])
    check("...and their heading follows from the wheel difference",
          abs(p["commanded"]["heading_deg"]
              - math.degrees(2000 * M_PER_COUNT / config.TRACK_M)) < 1e-9)

    refused("an unknown kind is refused", [{"kind": "spiral"}], expect="kind")
    refused("a move of nothing is refused", [{"kind": "straight", "distance_m": 0}],
            expect="moves nothing")
    refused("a wheel beyond max_distance_m is refused",
            [{"kind": "straight", "distance_m": config.BLIND_MAX_DISTANCE_M + 1}],
            expect="max_distance_m")
    refused("an arc tighter than half the track is refused",
            [{"kind": "arc", "radius_m": HALF / 2, "angle_deg": 90}], expect="half the track")
    refused("a speed that pushes the faster wheel past max_rpm is refused",
            [{"kind": "arc", "radius_m": 0.3, "angle_deg": 45}],
            {"motor_rpm": config.BLIND_MAX_RPM * 0.9}, expect="max_rpm")
    refused("too many segments are refused",
            [{"kind": "pivot", "angle_deg": 10}] * (config.BLIND_MAX_SEGMENTS + 1),
            expect="at most")
    refused("a speed must name its unit", [{"kind": "pivot", "angle_deg": 10}],
            {"fast": 1}, expect="speed")
    refused("a non-finite value is refused", [{"kind": "straight", "distance_m": float("nan")}],
            expect="finite")
    refused("no encoder scale, no plan", [{"kind": "pivot", "angle_deg": 10}], cpr=None,
            expect="encoder scale")


def test_closed_loop_and_odometry():
    print("\nblind run closed loop on an ideal plant")
    run, _ = fly([{"kind": "straight", "distance_m": 1.0}])
    r = run.results[0]
    check("a 1 m straight completes and records its counts at rest",
          run.phase == blindrun.DONE and len(run.results) == 1, f"{run.phase} {run.reason}")
    check("both wheels end within the stop tolerance",
          all(abs(e) <= TOL_COUNTS + 1 for e in r["error_counts"]), str(r["error_counts"]))
    check("encoder distance agrees with the counts it came from",
          abs(r["encoder_distance_m"]
              - (r["final_counts"][0] + r["final_counts"][1]) / 2 * M_PER_COUNT) < 1e-9)

    run, worst = fly([{"kind": "arc", "radius_m": 1.0, "angle_deg": 90}])
    r = run.results[0]
    check("an arc completes with both wheels on target",
          run.phase == blindrun.DONE and all(abs(e) <= TOL_COUNTS + 1 for e in r["error_counts"]),
          str(r["error_counts"]))
    check("...the wheels stay in step the whole way", worst < 0.01, f"{worst:.4f}")
    check("...and the encoder heading is the planned 90 deg",
          abs(r["encoder_heading_deg"] - 90) < 0.5, f"{r['encoder_heading_deg']:.3f}")

    # Each segment may stop up to the tolerance short, and four pivots add that
    # up, so closure is judged at a tight tolerance: what remains is odometry.
    for sign, name in ((1, "counter-clockwise"), (-1, "clockwise")):
        legs = [{"kind": "straight", "distance_m": 1.0},
                {"kind": "pivot", "angle_deg": 90 * sign}] * 4
        with patch.object(config, "BLIND_STOP_TOLERANCE_MM", 0.05):
            run, _ = fly(legs)
        x, y, h = run.pose
        check(f"a 1 m square {name} closes on the start by encoder",
              run.phase == blindrun.DONE and math.hypot(x, y) < 0.002
              and abs(math.remainder(h, 2 * math.pi)) < math.radians(0.1),
              f"{run.phase} x={x:.4f} y={y:.4f} h={math.degrees(h):.3f}")

    # At the default tolerance the pose must still be exactly the composition
    # of the recorded segments: the log and the live pose cannot disagree.
    run, _ = fly([{"kind": "straight", "distance_m": 1.0},
                  {"kind": "pivot", "angle_deg": 90}, {"kind": "straight", "distance_m": 0.5}])
    px = py = ph = 0.0
    for r in run.results:
        px += r["encoder_dx_m"] * math.cos(ph) - r["encoder_dy_m"] * math.sin(ph)
        py += r["encoder_dx_m"] * math.sin(ph) + r["encoder_dy_m"] * math.cos(ph)
        ph += math.radians(r["encoder_heading_deg"])
    check("the live pose is the composition of the recorded segments",
          abs(px - run.pose[0]) < 1e-9 and abs(py - run.pose[1]) < 1e-9
          and abs(ph - run.pose[2]) < 1e-12)

    planned = blindrun.plan([{"kind": "straight", "distance_m": 0.2}], SPEED, CPR)
    plant, run = Plant(), None
    run = blindrun.BlindRun(planned, plant.counts)
    for _ in range(3000):
        cmd = run.update(plant.counts, 0.02, None)
        if run.phase == blindrun.SETTLING:
            break
        plant.step(*cmd, 0.02)
    for _ in range(int(blindrun.SETTLE_LIMIT_S / 0.02) + 2):
        run.update(plant.counts, 0.02, None)
    check("counts are never recorded without a standstill verdict",
          run.results == [] and run.phase == blindrun.ABORTED and "standstill" in run.reason,
          str(run.reason))

    planned = blindrun.plan([{"kind": "straight", "distance_m": 0.5}], SPEED, CPR)
    plant = Plant()
    run = blindrun.BlindRun(planned, plant.counts)
    for _ in range(3000):
        run.update(plant.counts, 0.02, False)
        if run.phase == blindrun.ABORTED:
            break
        plant.step(1000, 1000, 0.02)          # a wheel that ignores the command
    check("running past the target by the margin aborts",
          run.phase == blindrun.ABORTED and "overran" in run.reason, str(run.reason))

    run = blindrun.BlindRun(planned, (0, 0))
    for _ in range(int((planned["segments"][0]["duration_s"] * blindrun.TIME_MARGIN
                        + blindrun.TIME_SLACK_S) / 0.02) + 5):
        run.update((0, 0), 0.02, True)
    check("a frozen counter aborts on the time budget",
          run.phase == blindrun.ABORTED and "time budget" in run.reason, str(run.reason))


# test_controller_panel_rules (canworker), test_blind_page_sets_but_cannot_start
# (app/) and test_run_log_failure_is_bounded (runlog) went with the legacy
# controller at U11 (2026-09-21). The planner and the closed loop stay: the ROS
# commissioning node (amr_base.commissioning) drives the same blindrun.plan().
TESTS = [test_plan_maths, test_closed_loop_and_odometry]
