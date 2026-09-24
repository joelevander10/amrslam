"""Encoder-only blind run: plan maths, closed loop, odometry and panel rules.

Synthetic encoder only: an ideal plant turns commanded r/min into counts. None
of this says anything about the real encoders - measuring them is what the
page is for.
"""
import math
import re
from unittest.mock import patch

from helpers import ROOT, check
import blindrun
import canworker
import config
import events
import panel
import runlog

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
          abs(r["encoder_distance_m"] - (r["final_counts"][0] + r["final_counts"][1]) / 2 * M_PER_COUNT) < 1e-9)

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


class Bench:
    """A controller armed in MANUAL, with the ideal plant behind 6064h."""

    def __init__(self, now):
        self.now = now
        c = self.c = canworker.Controller()
        c._blind_log = runlog.RunLog(enabled=False, columns=runlog.BLIND_COLUMNS,
                                     prefix="blind")
        c._armed, c._mode = True, "manual"
        c._counts_per_wheel_rev = CPR
        self.plant = Plant()
        c._read_positions = lambda: self.plant.counts
        self.last = (0, 0)

    def telemetry(self):
        for n in (config.LEFT, config.RIGHT):
            self.c._telemetry[n]["speed_zero"] = self.last == (0, 0)
            self.c._status_seen[n] = self.now[0]

    def start(self, segments):
        self.c.set_blind_plan({"segments": segments, "speed": SPEED})
        self.c._panel_start(panel.MANUAL)
        self.now[0] += config.AUTO_START_DELAY_S + 0.01
        self.c._pending_start()

    def tick(self, dt=0.02):
        self.now[0] += dt
        self.telemetry()
        target = self.c._blind_tick()
        self.plant.step(*target, dt)
        self.last = target
        return target


def test_controller_panel_rules():
    print("\nblind run on the CAN thread: web sets, panel runs")
    now = [1000.0]
    with patch("canworker.time.monotonic", side_effect=lambda: now[0]), \
            patch("canworker.time.perf_counter", side_effect=lambda: now[0]):
        b = Bench(now)
        c = b.c
        c._counts_per_wheel_rev = None
        try:
            c.set_blind_plan({"segments": [{"kind": "pivot", "angle_deg": 90}], "speed": SPEED})
            check("no plan without an encoder scale", False, "accepted")
        except RuntimeError as e:
            check("no plan without an encoder scale", "encoder scale" in str(e), str(e))
        c._counts_per_wheel_rev = CPR

        c._panel_start(panel.MANUAL)
        check("Start in MANUAL with no plan is ignored", not c._blind_start_at and c._blind is None)

        c.set_blind_plan({"segments": [{"kind": "straight", "distance_m": 0.3}], "speed": SPEED})
        check("setting a plan moves nothing", c._blind is None and c._target == (0, 0))
        c._panel_start(panel.MANUAL)
        check("Start in MANUAL with a plan arms the start delay",
              c._blind_start_at > now[0] and c._blind is None)
        try:
            c.drive("forward")
            check("jogging is refused while a blind start is pending", False, "accepted")
        except RuntimeError as e:
            check("jogging is refused while a blind start is pending", "blind run" in str(e))
        c._pending_start()
        check("nothing moves before the delay", c._blind is None)
        now[0] += config.AUTO_START_DELAY_S + 0.01
        c._pending_start()
        check("the run begins after the delay", c._blind is not None)
        try:
            c.set_blind_plan({"segments": [{"kind": "pivot", "angle_deg": 5}], "speed": SPEED})
            check("the plan cannot be changed mid-run", False, "accepted")
        except RuntimeError as e:
            check("the plan cannot be changed mid-run", "in progress" in str(e))

        for _ in range(5000):
            b.tick()
            if c._blind is None:
                break
        snap = c.snapshot()["blind"]
        check("the run completes and publishes its segment",
              snap["run"]["phase"] == "done" and len(snap["results"]) == 1
              and c._target == (0, 0), str(snap["run"]))
        res = snap["results"][0]
        check("the result row carries blank-for-operator context",
              res["profile"] == config.PROFILE_NAME and res["kind"] == "straight"
              and abs(res["error_left"]) <= TOL_COUNTS + 1)
        check("the plan survives the run for a repeat", snap["plan"] is not None)
        check("every result key the CSV wants is on the row, tape columns gone",
              all(k in res for k in runlog.RESULT_COLUMNS
                  if not k.startswith("measured_") and k != "notes")
              and not any(k.startswith("e_mm") or k == "tape_at_start" for k in res),
              str(sorted(set(runlog.RESULT_COLUMNS) - set(res))))

        b.start([{"kind": "straight", "distance_m": 2.0}])
        for _ in range(20):
            b.tick()
        c._panel_reset(panel.MANUAL)
        snap = c.snapshot()["blind"]
        check("Reset stops a blind run and keeps the plan",
              c._blind is None and snap["run"]["phase"] == "aborted"
              and "Reset" in snap["run"]["reason"] and snap["plan"] is not None,
              str(snap["run"]))
        check("...with the setpoint zeroed", c._target == (0, 0))

        b.start([{"kind": "straight", "distance_m": 2.0}])
        b.tick()
        c.halt()
        b.tick()
        check("web Stop stops a blind run",
              c._blind is None and "web" in c.snapshot()["blind"]["run"]["reason"])

        c.set_blind_plan({"segments": [{"kind": "straight", "distance_m": 2.0}], "speed": SPEED})
        c._panel_start(panel.MANUAL)
        c.halt()
        now[0] += config.AUTO_START_DELAY_S + 0.01
        c._pending_start()
        check("web Stop cancels a pending blind start", c._blind is None and not c._blind_start_at)

        b.start([{"kind": "straight", "distance_m": 2.0}])
        b.tick()
        c._do_disarm = lambda: None
        c._panel_mode_changed(panel.AUTO)
        check("a selector move stops a blind run", c._blind is None)

        c._armed, c._mode = True, "manual"
        b.start([{"kind": "straight", "distance_m": 2.0}])
        b.tick()
        c._read_positions = lambda: None
        now[0] += config.DRIVER_TIMEOUT_S + 0.01
        b.tick()
        check("lost 6064h feedback stops the run",
              c._blind is None and "6064h" in c.snapshot()["blind"]["run"]["reason"])

        c._deadline = now[0]
        c._blind = object()
        c._blind_keepalive()
        check("panel scans keep a blind run's watchdog alive", c._deadline > now[0])
        c._blind = None
    events.clear()


def test_blind_page_sets_but_cannot_start():
    print("\nthe blind-run page sets a plan and cannot start the wheels")
    import server as webapp
    c = webapp.app.test_client()
    body = c.get("/blind").get_data(as_text=True)
    check("/blind renders", 'id="blind-set"' in body and 'id="br-imu-tile"' in body)
    ids = re.findall(r'id="([^"]+)"', body)
    check("no control on the page is a start", not [i for i in ids if "start" in i.lower()], str(ids))
    check("...and it says where Start is", "run it from the panel" in body)
    check("the page does not claim the heartbeat", "CLAIM_HEARTBEAT" not in body)
    js = (ROOT / "app" / "static" / "blind.js").read_text()
    posts = sorted(set(re.findall(r"api\('([^']+)'", js)))
    check("blind.js POSTs only to the plan endpoints",
          posts == ["/api/blind/clear", "/api/blind/plan"], str(posts))
    check("there is no blind start route", c.post("/api/blind/start", json={}).status_code == 404)
    src = (ROOT / "app" / "server.py").read_text()
    check("...defined anywhere", "/api/blind/start" not in src and '"/api/arm"' not in src)
    real = webapp.ctl._counts_per_wheel_rev
    webapp.ctl._counts_per_wheel_rev = None
    try:
        r = c.post("/api/blind/plan", json={"segments": [{"kind": "pivot", "angle_deg": 90}],
                                            "speed": SPEED})
        check("setting a plan without an encoder scale is refused with the reason",
              r.status_code == 409 and "encoder scale" in r.get_json()["error"])
    finally:
        webapp.ctl._counts_per_wheel_rev = real
    check("every page links to it", 'href="/blind"' in c.get("/manual").get_data(as_text=True))


def test_run_log_failure_is_bounded():
    """R32: a disk that refuses writes must not grow the buffer or leak the file.

    _drain() used to fail before clearing the buffer, so every tick added a row
    and retried an ever larger write; close() skipped the actual close when that
    drain raised.
    """
    import errno
    import shutil
    import tempfile
    print("\nrun log: a failing disk is bounded and reported once")

    class FullDisk:
        def __init__(self):
            self.writes, self.closed = 0, False

        def write(self, text):
            self.writes += 1
            raise OSError(errno.ENOSPC, "No space left on device")

        def close(self):
            self.closed = True

    tmp = tempfile.mkdtemp()
    now = [5000.0]
    try:
        with patch.object(runlog, "LOG_DIR", tmp), \
                patch("runlog.time.monotonic", side_effect=lambda: now[0]):
            log = runlog.RunLog(columns=runlog.BLIND_COLUMNS, prefix="blind")
            log.open("test")
            real = log._fh
            disk = log._fh = FullDisk()
            events.clear()
            ticks = runlog.MAX_BUFFER_ROWS + 500
            for _ in range(ticks):
                now[0] += 0.02
                log.write({"phase": "run"})
            check("the buffer stops at its bound",
                  len(log._buf) == runlog.MAX_BUFFER_ROWS, str(len(log._buf)))
            check("...and the overflow is counted, not silently lost",
                  log.dropped == ticks - runlog.MAX_BUFFER_ROWS,
                  str(log.dropped))
            check("a failing write is retried once per flush period, not per "
                  "tick", disk.writes <= ticks * 0.02 / runlog.FLUSH_PERIOD_S + 2,
                  f"{disk.writes} attempts in {ticks} ticks")
            errs = [e for e in events.since(0)[1] if "dropping rows" in e["msg"]]
            check("one actionable error, naming the cause",
                  len(errs) == 1 and "No space left" in errs[0]["msg"],
                  str([e["msg"] for e in errs])[:90])
            log.close()
            check("close() still closes the file when the final drain fails",
                  disk.closed and log._fh is None and not log._buf)
            real.close()
            events.clear()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


TESTS = [test_plan_maths, test_closed_loop_and_odometry, test_controller_panel_rules,
         test_blind_page_sets_but_cannot_start, test_run_log_failure_is_bounded]
