"""The bus thread: arming, locking, frame routing, loop timing."""
import math
import os
import pathlib
import struct
import sys
import threading
import time

from helpers import FAIL, ROOT, check, _FakeRaw

import config
import kinematics
import motion

def test_arm_does_not_deadlock():
    """Arming must not self-deadlock the bus thread.

    sdo_read() drains the RX queue before transmitting, TpdoTap routes pushed
    frames to Controller._on_heartbeat(), and that takes Controller._lock. Any
    bus I/O performed while already holding that lock therefore deadlocks the
    bus thread against itself - and because snapshot() and keepalive() take the
    same lock, every Flask request wedges with it. That is not hypothetical: it
    shipped, and it presented as 409s on both arm and disarm with /api/state
    hanging.
    """
    print("\ncanworker: arming must not deadlock on the sensor stream")
    import canworker
    import events

    def arm_in_thread(objects):
        """_do_arm on its own thread, with its RESULT and its EXCEPTION kept.

        A bare Thread swallows an exception into stderr, so "the thread
        stopped" used to pass for an arm that had died on an SDO timeout.
        """
        ctl = canworker.Controller()
        ctl.bus = canworker.TpdoTap(_FakeRaw(objects=objects),
                                    on_heartbeat=ctl._on_heartbeat,
                                    nodes=(1,))
        ctl._do_preflight = lambda: {"ok": True, "report": []}
        ctl._nmt = lambda cmd, node: None
        out = {}

        def body():
            try:
                out["result"] = ctl._do_arm("manual")
            except BaseException as e:          # noqa: BLE001
                out["error"] = e

        t = threading.Thread(target=body, daemon=True)
        t.start()
        t.join(timeout=10.0)
        return ctl, t, out

    # -- the drives reach Operation enabled: the arm must SUCCEED ------------
    ready = {(0x6041, 0): 0x0027,
             (0x608F, 1): 10000, (0x608F, 2): 1,
             (0x6091, 1): 1, (0x6091, 2): 1}
    ctl, t, out = arm_in_thread(ready)
    check("_do_arm completes (no deadlock)", not t.is_alive(),
          "still blocked after 10 s" if t.is_alive() else "")
    if t.is_alive():
        return
    check("the arm thread raised nothing", "error" not in out,
          repr(out.get("error")))
    check("the arm reports success and the controller is armed in manual",
          (out.get("result") or {}).get("ok") is True
          and ctl._armed and ctl._mode == "manual",
          f"result={out.get('result')!r} armed={ctl._armed} mode={ctl._mode}")
    check("the re-entrant path was actually exercised",
          ctl._nmt_state.get(1) is not None,
          f"node 1 NMT state {ctl._nmt_state.get(1)!r} - set from a heartbeat "
          f"routed mid-arm")

    # -- a drive that never reaches Operation enabled: FAIL, still no deadlock
    bad, tb, outb = arm_in_thread({**ready, (0x6041, 0): 0x0270})
    check("a failing arm also completes (no deadlock)", not tb.is_alive())
    check("...and fails with the reason, left disarmed",
          "did not reach Operation enabled" in str(outb.get("error"))
          and "result" not in outb and not bad._armed,
          f"error={outb.get('error')!r} armed={bad._armed}")

    done = threading.Event()
    threading.Thread(target=lambda: (ctl.snapshot(), done.set()),
                     daemon=True).start()
    check("snapshot() still returns afterwards", done.wait(3.0))

    # No bus call may sit inside a locked section - the RLock is a backstop,
    # not a licence. This is the invariant that actually keeps it fixed.
    import re
    src = (ROOT / "canworker.py").read_text(encoding="utf-8").split("\n")
    inlock, indent, bad = False, 0, []
    for i, line in enumerate(src, 1):
        body = line.strip()
        if body.startswith("with self._lock"):
            inlock, indent = True, len(line) - len(line.lstrip())
            continue
        if inlock:
            cur = len(line) - len(line.lstrip())
            if body and cur <= indent:
                inlock = False
            elif re.search(r"self\._read\(|self\._write\(|sdo_read\("
                           r"|sdo_write\(|self\._nmt\(|bus\.(send|recv)\(",
                           body):
                bad.append(f"{i}: {body}")
    check("no bus I/O inside any locked section", not bad,
          "; ".join(bad) if bad else "")

    # -- the health wiring, end to end on the fake bus -----------------------
    # health.py is unit-tested above; this proves the Controller actually feeds
    # it and acts on it, which a source scan alone cannot show.
    ctl._armed = True
    ctl._target = (800, 800)
    events.clear()

    for nid in config.NODES:                    # both drivers answered once
        ctl._src_node[nid].mark_rx(now=0.0)
    hw = ctl._hw.evaluate(now=0.0)
    check("healthy drivers raise no critical fault", not hw["system_error"])

    hw = ctl._hw.evaluate(now=config.DRIVER_TIMEOUT_S + 1.0)
    check("a silent driver trips the critical tier", hw["system_error"],
          hw["system_detail"])
    target = ctl._apply_health(hw, armed=True, target=(800, 800))
    check("a critical fault zeroes the setpoint", target == (0, 0), str(target))
    check("a critical fault latches, so it needs an acknowledgment",
          ctl._fault is not None, str(ctl._fault))
    check("a critical fault names itself in stop_reason",
          "silent" in (ctl._last_stop_reason or ""), str(ctl._last_stop_reason))

    n_after_first = len(events.since(0)[1])
    for i in range(50):                         # a second of ticks, still dead
        hw2 = ctl._hw.evaluate(now=config.DRIVER_TIMEOUT_S + 2.0 + i * 0.02)
        if hw2["changed"] or hw2["system_edge"] is not None:
            ctl._apply_health(hw2, armed=True, target=(0, 0))
    check("a standing fault does not re-emit every tick",
          len(events.since(0)[1]) == n_after_first,
          f"{len(events.since(0)[1]) - n_after_first} extra event(s)")

    ctl._health = hw
    try:
        ctl._do_arm("manual")
        check("arming is refused while a driver is silent", False, "accepted!")
    except RuntimeError as e:
        check("arming is refused while a driver is silent",
              "not answering" in str(e), str(e)[:60])
    events.clear()


def test_arm_no_longer_touches_the_sensor():
    """The MLS bring-up is gone from the arm path, and must not creep back.

    Arming used to NMT-start node 10 and resolve its TPDO1 layout on EVERY arm,
    manual included, because the manual page showed the tape strip. That cost a
    sensor probe on a path that had nothing to do with the sensor, and it made a
    silent MLS able to refuse an arm.

    The sensor hardware is still installed - the IMU lives inside it - but it is
    read by drivers/canbus/read_imu.py on a bench, not by the arm.
    """
    print("\narming is independent of the MLS")
    src = (ROOT / "canworker.py").read_text(encoding="utf-8")
    check("no sensor bring-up on the arm path",
          "_start_sensor" not in src)
    check("no TPDO1 track decode remains", "decode_tpdo1" not in src)
    check("the arm cannot be refused by a silent sensor",
          "SENSOR_SILENT_MSG" not in src)
    check("the sensor node is still known, for the IMU that lives in it",
          "SENSOR_NODE" in (ROOT / "config.py").read_text(encoding="utf-8"))


def test_loop_health():
    """The loop-health window is what makes a telemetry stall visible without
    hand-parsing a CSV column, so its two timings must stay distinct."""
    print("\nloop health window")
    import canworker

    h = canworker._LoopHealth(window=4)
    out = None
    for i in range(4):
        out = h.tick(i * 0.02, 0.009)
    check("emits once the window fills", out is not None)
    # *** The two numbers must stay distinct. *** work_ms is what the tick SPENT
    # and is what shrinks when SDO leaves the loop; period_ms is what the tick
    # ACHIEVED. Collapsing them hides exactly the case worth seeing: work
    # climbing while the period is still being met by a shrinking pump.
    check("work and period are separate numbers",
          abs(out["work_avg_ms"] - 9.0) < 0.01
          and abs(out["period_avg_ms"] - 20.0) < 0.01,
          f"work {out['work_avg_ms']:.1f} ms, period {out['period_avg_ms']:.1f} ms")
    check("the max is kept, not just the mean - the tail is the problem",
          "work_max_ms" in out and "period_max_ms" in out)
    check("window resets after emitting", h.tick(0.1, 0.009) is None)

    # A tick that overruns shows up in work, not in a smoothed average: runs
    # 0023-0025 held a p50 of 20.1 ms with a max of 37.9, and it is the max that
    # names the telemetry burst.
    h2 = canworker._LoopHealth(window=4)
    for i, spent in enumerate((0.009, 0.031, 0.009, 0.009)):
        out2 = h2.tick(i * 0.02, spent)
    check("a single overrun survives into work_max_ms",
          abs(out2["work_max_ms"] - 31.0) < 0.01, f"{out2['work_max_ms']:.1f} ms")


def test_unsolicited_frames_survive_sdo():
    """EMCY and heartbeat must reach their handlers even mid-SDO.

    This is the whole reason TpdoTap exists and the reason it had to grow.
    sdo_read() opens by draining the RX queue and then keeps only frames
    matching 0x580+node, discarding the rest - so any pushed frame class not
    routed by the tap is silently lost for as long as a transfer is in flight,
    which with a setpoint write most ticks is most of the time.
    """
    import canworker
    from verify_drivers import sdo_read
    print("\nunsolicited frames survive an SDO transfer")

    emcy, beats = [], []

    class _Bus:
        """Replays a fixed frame sequence, pushed frames mixed into the stream."""

        def __init__(self, frames):
            self._frames = list(frames)
            self.sent = []

        def send(self, msg):
            self.sent.append(msg)

        def recv(self, timeout=None):
            # sdo_read() drains with timeout=0 before transmitting. Returning
            # nothing then models an idle bus, so the scripted frames land
            # AFTER the request - which is the case under test.
            if not timeout:
                return None
            return self._frames.pop(0) if self._frames else None

        def shutdown(self):
            pass

    def msg(cob, data):
        return canworker.can.Message(arbitration_id=cob, data=bytes(data),
                                     is_extended_id=False)

    reply = [0x4B, 0x41, 0x60, 0, 0x27, 0x06, 0, 0]     # 6041h = 0x0627
    raw = _Bus([
        msg(0x080 + 1, [0x22, 0xFF, 0x81, 0, 0, 0, 0, 0]),   # EMCY, node 1
        msg(0x700 + 2, [0x05]),                              # heartbeat, node 2
        msg(0x580 + 1, reply),                               # the SDO reply
    ])
    tap = canworker.TpdoTap(raw,
                            on_emcy=lambda n, d: emcy.append((n, d)),
                            on_heartbeat=lambda n, b: beats.append((n, b)),
                            nodes=[1, 2])

    st, val, _, _ = sdo_read(tap, 1, 0x6041, 0, collision_window=0.0)
    check("the SDO reply still gets through", st is True and val is not None)
    check("an EMCY mid-transfer reaches its handler",
          len(emcy) == 1 and emcy[0][0] == 1, str(emcy)[:60])
    check("a heartbeat mid-transfer reaches its handler",
          beats == [(2, 0x05)], str(beats))

    # A handler that throws must not break the bus thread - these run inside
    # somebody else's SDO transfer.
    boom = canworker.TpdoTap(
        _Bus([msg(0x080 + 1, [0] * 8), msg(0x580 + 1, reply)]),
        on_emcy=lambda n, d: 1 / 0,
        on_heartbeat=lambda n, b: None, nodes=[1])
    st, _, _, _ = sdo_read(boom, 1, 0x6041, 0, collision_window=0.0)
    check("a throwing handler cannot break the transfer", st is True)

    # A frame we do not route must be handed back, not swallowed.
    passthru = canworker.TpdoTap(_Bus([msg(0x580 + 1, reply)]), nodes=[])
    check("an unrouted frame is returned to the caller",
          passthru.recv(timeout=0.1) is not None)


def test_bus_thread_survives_open():
    """_run() must get past TpdoTap construction with a bus that opened.

    Nothing else in this file goes through _run(): every test builds the tap
    by hand, so the call site in _run() can drift from TpdoTap.__init__ and
    the suite stays green. That shipped: the MLS sensor path was removed, the
    tap lost its on_pdo argument, and _run() kept passing self._on_pdo - so
    open_bus() succeeded, the thread died on an AttributeError one line later,
    and the UI reported the bus down while can0 carried traffic.

    open_bus is replaced by an idle fake; the thread has to come up, publish
    connected/how, and still be alive when told to stop.
    """
    import canworker
    import events
    print("\ncanworker: the bus thread survives a successful open")

    class _Idle:
        def send(self, m):
            pass

        def recv(self, timeout=None):
            if timeout:
                time.sleep(min(timeout, 0.01))
            return None

        def shutdown(self):
            pass

    real = canworker.open_bus
    canworker.open_bus = lambda *a, **k: (_Idle(), "fake:test")
    try:
        ctl = canworker.Controller()
        t = threading.Thread(target=ctl._run, name="can-test", daemon=True)
        t.start()
        time.sleep(0.5)
        snap = ctl.snapshot()
        check("bus thread is still running", t.is_alive(),
              "died - see the traceback above" if not t.is_alive() else "")
        check("connected is published", snap["connected"] is True,
              f"connected={snap['connected']!r} error={snap['error']!r}")
        check("transport is published", snap["how"] == "fake:test",
              f"how={snap['how']!r}")
        ctl._stop_evt.set()
        t.join(timeout=5.0)
        check("bus thread stops on request", not t.is_alive())
    finally:
        canworker.open_bus = real


def test_horn_follows_commanded_motion():
    """DO high whenever motion is commanded, in either mode, and never else."""
    print("\ncanworker: the horn")
    import canworker

    class FakeDio:
        def __init__(self):
            self.calls = []

        def set_coil(self, ch, value, hold_s):
            self.calls.append((ch, value, hold_s))

    ctl = canworker.Controller()
    fake = FakeDio()
    ctl._dio = fake

    def horn(armed, target):
        fake.calls.clear()
        ctl._update_horn(armed, target)
        check_ch = [c for c in fake.calls if c[0] == config.HORN_DO_CHANNEL]
        return check_ch[-1][1] if check_ch else None

    check("a jog sounds it", horn(True, (600, 600)) is True)
    check("so does a spin, where the wheels oppose each other and the vehicle "
          "does not go anywhere a bystander expects",
          horn(True, (600, -600)) is True)
    check("a single creeping wheel still counts as motion",
          horn(True, (0, 40)) is True)

    check("armed and holding zero is silent - arming is not motion",
          horn(True, (0, 0)) is False)
    check("a disarmed vehicle is silent", horn(False, (0, 0)) is False)
    # The interlock, stated rather than implied. A setpoint left standing from
    # before a disarm must not sound the horn on a vehicle that cannot move.
    check("...even if a setpoint is somehow still standing",
          horn(False, (600, 600)) is False)

    check("the command carries the renewal deadline, so a dead tick drops it",
          fake.calls[-1][2] == config.HORN_HOLD_S, str(fake.calls[-1]))

    # Renewed every tick, not written on the edge - that is what makes the
    # deadline in dio.set_coil() a live watchdog rather than a formality.
    fake.calls.clear()
    for _ in range(5):
        ctl._update_horn(True, (600, 600))
    check("a held jog renews the claim every tick", len(fake.calls) == 5)

    # And the policy has to be ON the tick, not merely available to it.
    src = (ROOT / "canworker.py").read_text()
    body = src[src.index("def _run(self)"):src.index("def _update_horn")]
    check("_run() calls it on every tick, outside any armed-only branch",
          "self._update_horn(armed, target)" in body)

    horn_at = src.index("self._update_horn(armed, target)")
    check("and does so BEFORE the setpoint reaches the wheels",
          horn_at < src.index("self._write_target(target)", horn_at))


def test_imu_poll():
    """One SDO read per poll, spread over the schedule, backing off on a miss.

    The whole point of the round-robin is that no single tick pays for the
    full set, so the test counts requests per poll as well as checking that
    the values land decoded in the snapshot.
    """
    print("\ncanworker: the IMU poll")
    import canworker

    class _Sensor:
        """Answers node 10 SDO uploads from a table; silent for anything else."""

        def __init__(self, table):
            self.table = dict(table)      # (index, sub) -> 16-bit raw
            self.requests = []
            self.pending = []
            self.absent = False

        def send(self, m):
            if m.arbitration_id != 0x600 + config.SENSOR_NODE:
                return
            index = m.data[1] | (m.data[2] << 8)
            sub = m.data[3]
            self.requests.append((index, sub))
            if self.absent:
                return
            raw = self.table[(index, sub)]
            self.pending.append(canworker.can.Message(
                arbitration_id=0x580 + config.SENSOR_NODE,
                data=bytes([0x4B, m.data[1], m.data[2], sub,
                            raw & 0xFF, raw >> 8, 0, 0]),
                is_extended_id=False))

        def recv(self, timeout=None):
            if self.pending:
                return self.pending.pop(0)
            if timeout:
                time.sleep(min(timeout, 0.002))
            return None

        def shutdown(self):
            pass

    # gyro z = +16 LSB = 0.9765625 deg/s; accel z = 2048 LSB = 1 g;
    # gyro x negative to prove the sign is decoded; yaw = -pi.
    raw = {(0x2034, 1): 0x10000 - 32, (0x2034, 2): 0, (0x2034, 3): 16,
           (0x2033, 1): 0, (0x2033, 2): 0, (0x2033, 3): 2048,
           (0x2030, 3): 0x10000 - 31416, (0x2035, 0): 1234}
    sensor = _Sensor(raw)
    ctl = canworker.Controller()
    ctl.bus = canworker.TpdoTap(sensor, nodes=())

    n = len(canworker._IMU_SCHEDULE)
    base = time.monotonic()
    for i in range(n):
        before = len(sensor.requests)
        ctl._poll_imu(base + i)
        check(f"poll {i} is exactly one SDO request", len(sensor.requests) - before == 1,
              str(sensor.requests[before:]))

    imu = ctl.snapshot()["imu"]
    check("gyro z decoded to deg/s", abs(imu["gyro_dps"][2] - 0.9765625) < 1e-9,
          str(imu["gyro_dps"]))
    check("a negative gyro reading keeps its sign",
          abs(imu["gyro_dps"][0] + 1.953125) < 1e-9, str(imu["gyro_dps"]))
    check("accel z decoded to g", abs(imu["accel_g"][2] - 1.0) < 1e-9,
          str(imu["accel_g"]))
    check("yaw decoded to radians", abs(imu["yaw_rad"] + 3.1416) < 1e-9,
          str(imu["yaw_rad"]))
    check("the sensor clock is unsigned", imu["stamp_ms"] == 1234)
    check("every read counted, none missed",
          imu["seen"] == n and imu["misses"] == 0)
    check("fresh", imu["stale"] is False and imu["age_s"] is not None)
    check("the next poll is one poll_period_s out",
          abs(ctl._imu_next - (base + n - 1 + config.IMU_PERIOD_S)) < 1e-9)

    # The sensor goes away. One short timeout, then back off - the control
    # loop must not pay a timeout on every poll for a display value.
    sensor.absent = True
    cursor = ctl._imu_cursor
    t0 = time.perf_counter()
    ctl._poll_imu(base + 100.0)
    took = time.perf_counter() - t0
    check("a miss is a short timeout, not sdo_read's 0.4 s default",
          took < 0.2, f"{took:.3f} s")
    check("a miss backs off retry_period_s",
          abs(ctl._imu_next - (base + 100.0 + config.IMU_RETRY_PERIOD_S)) < 1e-9)
    check("and retries the same object rather than skipping it",
          ctl._imu_cursor == cursor)
    imu = ctl.snapshot()["imu"]
    check("the miss is counted and the last values stand",
          imu["misses"] == 1 and abs(imu["accel_g"][2] - 1.0) < 1e-9)

    # Stale is a function of age, judged at snapshot time.
    ctl._imu["last"] = time.monotonic() - 60.0
    check("a sensor silent for a minute is reported stale",
          ctl.snapshot()["imu"]["stale"] is True)

    # And the poll has to be on the tick.
    src = (ROOT / "canworker.py").read_text()
    body = src[src.index("def _run(self)"):src.index("def _pump(")]
    check("_run() polls it, gated on imu.enabled",
          "config.IMU_ENABLED" in body and "self._poll_imu(now)" in body)


def test_queued_actions_cannot_starve_the_tick():
    """R15: queued web work is rationed, refused while armed, and abandoned
    requests never run.

    _drain_queue() used to run to exhaustion ahead of the panel scan and the
    watchdog, preflight (blocking SDO reads) was accepted while armed, and a
    request whose HTTP wait had timed out still executed later.
    """
    from concurrent.futures import Future
    import canworker
    print("\ncanworker: the action queue is bounded per tick")

    ran = []

    class Ctl(canworker.Controller):
        def _do_preflight(self):
            ran.append("preflight")
            return {"ok": True, "report": []}

        def _do_disarm(self, force=False):
            ran.append("disarm")
            self._armed = False
            return {"ok": True}

    c = Ctl()
    futs = [Future() for _ in range(5)]
    for f in futs:
        c._q.put(("preflight", (), f))
    c._drain_queue()
    check("one queued action runs per tick, however many are waiting",
          ran == ["preflight"] and c._q.qsize() == 4,
          f"ran={ran} left={c._q.qsize()}")

    c._armed = True
    del ran[:]
    c._q.put(("disarm", (), Future()))
    for _ in range(4):
        c._drain_queue()
    check("preflight is refused while armed, without touching the bus",
          "preflight" not in ran
          and all("refused while armed" in str(f.exception(0))
                            for f in futs[1:]),
          f"ran={ran}")
    check("...and refusals do not hold up the disarm queued behind them",
          c._q.qsize() == 0 and not c._armed and "disarm" in ran,
          f"left={c._q.qsize()} armed={c._armed}")

    del ran[:]
    try:
        c.submit("preflight", timeout=0.01)     # no bus thread to answer it
        check("a request nobody answers times out", False, "returned")
    except Exception:                           # noqa: BLE001
        check("a request nobody answers times out", True)
    c._drain_queue()
    check("...and never executes later", ran == [] and c._q.qsize() == 0,
          f"ran={ran}")


TESTS = [
    test_arm_does_not_deadlock,
    test_arm_no_longer_touches_the_sensor,
    test_loop_health,
    test_unsolicited_frames_survive_sdo,
    test_bus_thread_survives_open,
    test_horn_follows_commanded_motion,
    test_imu_poll,
    test_queued_actions_cannot_starve_the_tick,
]
