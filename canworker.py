"""Single-threaded owner of can0. Every CAN frame in this app goes through here.

Flask request handlers never touch the bus. They mutate a setpoint under a lock
(cheap, non-blocking) or push a slow action onto a queue and wait on a Future.
One thread owns the bus because:

  * an SDO transfer is a send/recv PAIR that must not interleave with another
    one - two threads doing SDO at once will read each other's replies;
  * verify_drivers.sdo_read() drains the RX queue before transmitting, so a
    second reader would have its frames eaten out from under it.

Reuses the proven helpers rather than reimplementing CANopen:
  verify_drivers.open_bus / sdo_read / u32     bus discovery + SDO upload
  drive_forward.sdo_write / CW_* / SW_*        SDO download + CiA 402 words
  bus_health.decode_state                      statusword -> state name
  rpdo.pack / configure                        RPDO1 setpoint frames
"""
import os
import queue
import struct
import sys
import threading
import time
from concurrent.futures import Future
from concurrent.futures import TimeoutError as FutureTimeout

# The layer directories are put on sys.path rather than made into packages, so
# every module keeps importing its neighbours by bare name. That is what lets
# drivers/canbus/ still run standalone on a bench: its modules import each other
# as `verify_drivers`, not `drivers.canbus.verify_drivers`, and as a package
# they would resolve twice under two different names.
#
# The cost is that module BASENAMES are one flat namespace across these
# directories. Two modules may never share a name and none may shadow a stdlib
# module; tests/test_layout.py enforces it.
_ROOT = os.path.dirname(os.path.abspath(__file__))
for _d in ("", "core", "drivers", os.path.join("drivers", "canbus")):
    sys.path.insert(0, os.path.join(_ROOT, _d) if _d else _ROOT)

import can  # noqa: E402
from alarms import (decode_emcy, decode_nmt,  # noqa: E402
                    decode_statusword_flags)
from bus_health import decode_state  # noqa: E402
from guard import check as guard_write  # noqa: E402
from drive_forward import (CW_DISABLE_VOLTAGE, CW_ENABLE, CW_SHUTDOWN,  # noqa: E402
                           CW_SWITCH_ON, SW_FAULT, SW_REMOTE,
                           SW_SPEED_IS_ZERO, sdo_write)
import read_imu  # noqa: E402
import blindrun  # noqa: E402
import runlog  # noqa: E402
import rpdo  # noqa: E402
from verify_drivers import open_bus, sdo_read, u32  # noqa: E402

import canmon  # noqa: E402
import ownerlock  # noqa: E402
import config  # noqa: E402
import events  # noqa: E402
import health  # noqa: E402
import motion  # noqa: E402
import dio  # noqa: E402
import panel  # noqa: E402
import rfid  # noqa: E402

# Node IDs, driver ramp rates, watchdogs and loop periods all come from the
# vehicle profile - see config.py's TUNING NOTES for the reasoning behind the
# values, and profiles/<AGV_PROFILE>.json to change them.

# How long to wait before retrying an auto-arm that failed. Not a vehicle
# parameter: it is "slow enough not to hammer the bus". An arm attempt is
# ~20 blocking SDO round-trips, and the common reason for failing one is a
# safety chain holding the drives in ETO - a condition that lasts as long as
# it lasts, so retrying it at tick rate would spend the whole bus thread on
# a question whose answer cannot change quickly.
ARM_RETRY_S = 2.0

# Queued web actions run on the bus thread between the panel scan and the
# watchdog, so they are rationed: at most one that actually does bus work per
# tick. Refused or abandoned requests cost nothing and do not use the slot, but
# a tick still looks at no more than QUEUE_SCAN_PER_TICK of them.
QUEUE_SCAN_PER_TICK = 32

# Queued actions that stall the bus thread on blocking SDO round-trips and have
# no business running while the drives are energised.
_REFUSED_WHILE_ARMED = {"preflight"}


def _battery(mon):
    """Pack voltage for the shared rail: the WORST of the two drives.

    Both amplifiers sit on the same battery, so the two readings are the same
    quantity measured twice. Showing the lower one is the honest summary - and
    showing an average would let a drive reading 38 V hide behind one reading 49.

    canmon already applies the profile's two-sided thresholds, so the state comes
    from there rather than being re-derived against a second copy of the limits.
    """
    worst_v, state, warn_low = None, "ok", None
    rank = {"ok": 0, "warn": 1, "trip": 2}
    for node in (mon.get("nodes") or {}).values():
        bv = node.get("bus_v") or {}
        v = bv.get("value")
        if v is None:
            continue
        warn_low = bv.get("warn_low") if warn_low is None else warn_low
        if worst_v is None or v < worst_v:
            worst_v = v
        if rank.get(bv.get("state"), 0) > rank.get(state, 0):
            state = bv.get("state")
    return {"volts": worst_v, "state": state, "warn_low": warn_low}


class TpdoTap:
    """Bus wrapper that routes every UNSOLICITED frame before the SDO helpers
    can discard it.

    sdo_read() and sdo_write() only ever call .send() and .recv() on the object
    they are given, so passing this in lets both be reused verbatim while the
    pushed traffic still reaches its decoder.

    *** This is the only thing standing between a pushed frame and the bin. ***
    Both SDO helpers open with a "drain anything stale" loop and then keep only
    frames matching 0x580+node, discarding everything else. So any frame class
    not routed HERE is silently lost for as long as an SDO transfer is in
    flight - which, with a setpoint write most ticks and six telemetry reads
    every 200 ms, is most of the time.

    Routed:
        0x080 + n    EMCY - alarms, pushed one per error event
        0x700 + n    heartbeat - the only thing that tells a dead driver apart
                     from an idle one
    Everything else is handed back to the caller, which is how SDO replies get
    home.

    *** This is where pushed telemetry lands when it arrives. *** The 5 Hz
    _poll_telemetry() burst is six blocking SDO reads inside one 20 ms tick and
    is the largest remaining spike in the loop; moving the statusword and actual
    velocity onto drive TPDOs (0x180+n / 0x280+n) is one more entry in the table
    below, which is exactly what this class was shaped for. See
    manuals/codebase-improvement.md section 3.
    """

    def __init__(self, bus, on_emcy=None, on_heartbeat=None, nodes=()):
        self._bus = bus
        self._on_emcy = on_emcy
        self._on_heartbeat = on_heartbeat
        # Built once: a dict lookup per frame, on a path that sees every frame
        # on the bus at 100+ Hz.
        self._route = {}
        for nid in nodes:
            if on_emcy is not None:
                self._route[0x080 + nid] = ("emcy", nid)
            if on_heartbeat is not None:
                self._route[0x700 + nid] = ("hb", nid)

    def send(self, msg):
        self._bus.send(msg)

    def recv(self, timeout=None):
        deadline = None if timeout is None else time.perf_counter() + timeout
        while True:
            remaining = (None if deadline is None
                         else max(0.0, deadline - time.perf_counter()))
            m = self._bus.recv(timeout=remaining)
            if m is None:
                return None
            hit = self._route.get(m.arbitration_id)
            if hit is not None:
                kind, nid = hit
                # A handler must never break the bus thread, and these run
                # inside somebody else's SDO transfer.
                try:
                    if kind == "emcy":
                        self._on_emcy(nid, bytes(m.data))
                    else:
                        self._on_heartbeat(nid, m.data[0] if m.data else 0)
                except Exception:               # noqa: BLE001
                    pass
                continue
            return m

    def shutdown(self):
        self._bus.shutdown()


# The IMU poll, one SDO read per poll_period_s, round-robin over this list. A
# read costs ~4 ms on the 125 kbps bus and a full set would eat a 20 ms tick,
# so it is spread rather than burst - see the imu.* tuning note in config.py.
# (field, axis, index, sub). Axis is the slot in a three-vector, None for the
# scalars. Only what is worth watching before gyro-odometry exists: the
# quaternion and the other two Euler angles are on read_imu.py when wanted.
_IMU_SCHEDULE = (
    ("gyro_dps", 0, read_imu.OBJ_GYRO, 1),
    ("gyro_dps", 1, read_imu.OBJ_GYRO, 2),
    ("gyro_dps", 2, read_imu.OBJ_GYRO, 3),
    ("accel_g", 0, read_imu.OBJ_ACCEL, 1),
    ("accel_g", 1, read_imu.OBJ_ACCEL, 2),
    ("accel_g", 2, read_imu.OBJ_ACCEL, 3),
    ("yaw_rad", None, read_imu.OBJ_EULER, 3),
    ("stamp_ms", None, *read_imu.OBJ_STAMP),
)
_IMU_SCALE = {"gyro_dps": read_imu.GYRO_LSB_DPS,
              "accel_g": read_imu.ACCEL_LSB_G,
              "yaw_rad": read_imu.EULER_LSB_RAD}
# A reply normally lands in ~4 ms. This is the wait before the sensor is
# declared absent for this poll; sdo_read's 0.4 s default is meant for
# commissioning, and blocking the tick that long for a display value is not.
IMU_SDO_TIMEOUT_S = 0.05


class _LoopHealth:
    """Rolling loop-timing window for the bus thread.

    Separates two things that a single "loop time" number confuses:

      work_ms   how long the iteration spent doing things - SDO round-trips,
                telemetry, monitoring. This is the number that grows when the
                bus gets slow, and it is the one to watch when the RPDO1 and
                TPDO migrations land: both exist to shrink it.
      period_ms the interval actually achieved between iterations.

    Measured on tape-following runs 0023-0025, this loop ran a p50 of 20.1 ms
    against a 20 ms budget but a p95 of 28.1 and a max of 37.9, and the tail was
    almost entirely the 5 Hz telemetry burst. That measurement is the reason
    work_ms and period_ms are reported separately rather than as one number -
    see manuals/codebase-improvement.md section 1.

    Pure arithmetic on the bus thread; it never touches the lock or the bus.
    """

    def __init__(self, window=50):
        self.window = window          # ~1 s at 50 Hz
        self._reset()
        self._prev_iter = None

    def _reset(self):
        self._work = []
        self._period = []

    def tick(self, t_iter, spent):
        """Returns a stats dict once per window, else None."""
        if self._prev_iter is not None:
            self._period.append((t_iter - self._prev_iter) * 1000.0)
        self._prev_iter = t_iter
        self._work.append(spent * 1000.0)

        if len(self._work) < self.window:
            return None
        stats = {
            "work_avg_ms": sum(self._work) / len(self._work),
            "work_max_ms": max(self._work),
            "period_avg_ms": (sum(self._period) / len(self._period)
                              if self._period else None),
            "period_max_ms": max(self._period) if self._period else None,
        }
        self._reset()
        return stats


class Controller:
    """Owns the bus thread and the vehicle state machine."""

    def __init__(self):
        # RLock, not Lock. Any bus read can re-enter this class: sdo_read()
        # drains the RX queue, TpdoTap hands pushed frames to _on_emcy() /
        # _on_heartbeat(), and both take this lock. Holding a plain Lock across
        # a bus call
        # therefore self-deadlocks the bus thread, which wedges every Flask
        # request too (snapshot() and keepalive() both take it). Bus I/O is kept
        # out of locked sections as the real fix; this is the backstop.
        self._lock = threading.RLock()
        self._q = queue.Queue()
        self._stop_evt = threading.Event()
        self._thread = None

        self.bus = None
        self.how = None
        self.error = None

        self._armed = False
        self._mode = "idle"          # idle | manual
        self._direction = "stop"
        self._target = (0, 0)
        self._applied = None
        self._deadline = 0.0
        self._last_stop_reason = None

        self._telemetry = {n: {"statusword": None, "state": "-", "rpm": None,
                               "error_reg": None} for n in config.NODES}
        # Last reported fault state per node, so _poll_telemetry emits one event
        # per edge rather than one per 5 Hz poll. This is the driver's OWN fault
        # bit; whether the driver is still answering at all is health.py's job.
        self._fault_seen = {n: False for n in config.NODES}

        # The IMU inside the MLS, polled over SDO for display. Not a health
        # source and not an input to anything - see the imu.* tuning note.
        self._imu = {"gyro_dps": [None, None, None],
                     "accel_g": [None, None, None],
                     "yaw_rad": None, "stamp_ms": None,
                     "seen": 0, "misses": 0, "last": None}
        self._imu_cursor = 0
        self._imu_next = 0.0

        # When each drive's statusword was last read. _drives_stopped() needs
        # a FRESH speed-zero bit, not the last one that happened to be seen.
        self._status_seen = {n: None for n in config.NODES}

        # Encoder-only blind run: the web SETS a plan, PB Start (MANUAL) runs
        # it. See blindrun.py and the blind_run.* tuning note.
        self._counts_per_wheel_rev = None     # 608Fh/6091h, read at arm time
        self._blind_plan = None
        self._blind = None
        self._blind_start_at = 0.0
        self._blind_last = None
        self._blind_results = []
        self._blind_written = 0
        self._blind_tick_at = None
        self._blind_pos_at = 0.0
        self._blind_abort_req = None
        self._blind_log = runlog.RunLog(columns=runlog.BLIND_COLUMNS,
                                        prefix="blind")

        # Hardware health. One table, one place to add the next device - see
        # health.py for why this is not three more if-statements, and for why it
        # is emphatically not the browser watchdog above.
        self._hw = health.HealthMonitor()
        self._src_node = {
            n: self._hw.add(health.HealthSource(f"driver:{n}", critical=True,
                                                detail=f"node {n} "
                                                       f"({config.NODES[n]})"),
                            config.DRIVER_TIMEOUT_S)
            for n in config.NODES}
        self._health = {}

        # Drive monitoring. Diagnostic only - see manuals/can-monitoring-plan.txt
        # and canmon.py. The poller is pure bookkeeping; this class does the
        # actual SDO read, one object per node per tick.
        self._mon = canmon.MonitorPoller(config.NODES)
        self._alarms = {n: None for n in config.NODES}   # latest decoded EMCY
        self._nmt_state = {n: None for n in config.NODES}
        self._emcy_seen = 0

        # Loop health, published for the UI. Answers "is the tick late, and if so
        # is it the controller or the bus?" - work_ms is time spent doing things,
        # period_ms is the interval actually achieved. We diagnosed a 70 ms
        # telemetry stall by hand from a CSV column once; this makes it visible.
        self._loop = {"work_avg_ms": None, "work_max_ms": None,
                      "period_avg_ms": None, "period_max_ms": None,
                      "target_ms": config.LOOP_PERIOD_S * 1000.0}

        # Station tags. Its own thread and its own socket - the control tick
        # only ever reads rfid.snapshot(), never touches the network.
        #
        # The reader survives tape following, but its JOB changes. It used to
        # drive the junction ladder; under SLAM it answers the kidnapped-robot
        # problem - a known tag collapses the pose hypothesis space to one
        # region and lets a localiser converge. It returns IDENTITY, not pose,
        # so anything downstream must treat a read as a wide-covariance seed
        # rather than a correction (hardware-reconciliation.md D-4).
        self._rfid = rfid.RfidLink()
        # Pulled rather than pushed: the link already tracks its own health on
        # its own thread, and copying that verdict beats defining "healthy" for
        # the reader twice. Registered unconditionally - snapshot() reports a
        # None verdict while rfid.enabled is false, which health.py reads as
        # "not in use" and never counts against a mode.
        self._hw.add(health.PullSource("rfid", self._rfid.snapshot),
                     config.RFID_SILENT_WARN_S)

        # Digital I/O, same arrangement: its own thread, its own socket, pulled
        # for health. Non-critical, so a dead module never holds manual
        # jogging hostage to a device manual does not use.
        #
        # Note the shared cable: the RFID reader is daisy-chained through this
        # module, so losing it reports BOTH sources at once. That pairing is
        # the signature of a cable or a power fault rather than two devices
        # failing together, and it is only visible because both are registered.
        self._dio = dio.DioLink()
        self._hw.add(health.PullSource("dio", self._dio.snapshot),
                     config.DIO_SILENT_WARN_S)

        # Operator panel. Scanned from the DI image every tick in _run(), in
        # every state - the panel is what ENTERS a state, so it has to be read
        # while idle.
        self._panel = panel.PanelScan(
            config.PANEL_DI_RESET, config.PANEL_DI_START, config.PANEL_DI_AUTO,
            config.PANEL_DEBOUNCE_SCANS)
        # A latched involuntary stop. Start is refused until Reset clears it, so
        # a vehicle that stopped itself cannot be restarted by somebody who did
        # not see why. A DELIBERATE stop is not a fault and does not latch.
        self._fault = None
        # MANUAL is an armed state (panel.manual_auto_arm): the selector sitting
        # there is the arm command, so arming is a level to be maintained rather
        # than an edge somebody presses. _arm_retry_at backs off a failed attempt
        # - see ARM_RETRY_S - and _arm_fail carries the last reason for the UI.
        self._arm_retry_at = 0.0
        self._arm_fail = None
        self._last_action = None        # (what, source) for the UI
        # Manual authority comes from the panel, so it ends with the panel's
        # image. _panel_valid is the last scan's verdict; _jog_released goes
        # False when that image is lost and stays False until a stop arrives
        # from the browser, so a direction the browser was still re-POSTing
        # through the gap cannot resume on its own when the panel comes back.
        self._panel_valid = False
        self._jog_released = True

    # ---- lifecycle ------------------------------------------------------

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        # Cooperative owner locks (core/ownerlock.py): the ROS drive_node and
        # panel_node take the same two before touching can0 / the DIO island,
        # so neither side can start on top of the other. Raises OwnerBusy.
        self._locks = [ownerlock.acquire("can", "agv_controller"), ownerlock.acquire("dio", "agv_controller")]
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, name="can", daemon=True)
        self._thread.start()
        self._rfid.start()          # no-op while rfid.enabled is false
        self._dio.start()           # likewise while dio.enabled is false

    def shutdown(self):
        self._stop_evt.set()
        self._rfid.stop()
        self._dio.stop()
        if self._thread:
            self._thread.join(timeout=6.0)

    def submit(self, action, *args, timeout=15.0):
        """Run a slow action on the bus thread and wait for it."""
        fut = Future()
        self._q.put((action, args, fut))
        try:
            return fut.result(timeout=timeout)
        except FutureTimeout:
            # Nobody is waiting any more, so it must not run later either. A
            # no-op if the bus thread has already started it.
            fut.cancel()
            raise

    # ---- called from Flask threads --------------------------------------

    def drive(self, direction):
        """Set a manual direction and refresh the watchdog. Non-blocking."""
        if not motion.is_direction(direction):
            raise ValueError(f"unknown direction {direction!r}")
        with self._lock:
            if self._mode != "manual":
                raise RuntimeError("not in manual mode")
            if not self._armed:
                raise RuntimeError("not armed")
            if self._fault:
                # A latched fault zeroed the setpoint; a jog must not replace
                # it. Reset acknowledges, then MANUAL re-arms (_hold_arm_state).
                raise RuntimeError(f"fault latched: {self._fault} - press Reset")
            if self._blind is not None or self._blind_start_at:
                raise RuntimeError("a blind run is in progress - Reset or Stop it first")
            if direction == "stop":
                self._jog_released = True
            elif config.PANEL_ENABLED and not self._panel_valid:
                # A browser keepalive is not manual authority; the panel is.
                raise RuntimeError("panel input lost - jog refused")
            elif not self._jog_released:
                raise RuntimeError("panel input was lost while jogging - "
                                   "release and press again")
            self._direction = direction
            self._target = motion.velocities(direction)
            self._deadline = time.monotonic() + config.MANUAL_WATCHDOG_S
            self._last_stop_reason = None

    def halt(self):
        """Zero the setpoint but stay armed. Also stops a blind run."""
        with self._lock:
            if self._blind is not None or self._blind_start_at:
                # Acted on by the bus thread, which owns the run.
                self._blind_abort_req = "stopped from the web (Stop)"
            self._direction = "stop"
            self._target = (0, 0)
            self._deadline = time.monotonic() + config.MANUAL_WATCHDOG_S
            self._jog_released = True

    def set_blind_plan(self, spec):
        """Validate and store a blind-run plan. Moves nothing: PB Start runs it."""
        spec = spec or {}
        with self._lock:
            busy = self._blind is not None or bool(self._blind_start_at)
            cpr = self._counts_per_wheel_rev
        if busy:
            raise RuntimeError("a blind run is in progress - press Reset to stop it first")
        if cpr is None:
            raise RuntimeError("encoder scale unknown - put the selector in MANUAL so "
                               "the vehicle arms and reads 608Fh/6091h")
        planned = blindrun.plan(spec.get("segments"), spec.get("speed"), cpr)
        with self._lock:
            self._blind_plan = planned
        events.info(f"blind-run plan set from the web: {len(planned['segments'])} "
                    f"segment(s) - press Start with the selector in MANUAL to run it")
        return planned

    def clear_blind_plan(self):
        with self._lock:
            if self._blind is not None or self._blind_start_at:
                raise RuntimeError("a blind run is in progress - press Reset to stop it first")
            had, self._blind_plan = self._blind_plan, None
        if had:
            events.info("blind-run plan cleared from the web")

    def _alarm(self, mon, flags):
        """The one-line verdict every page shows. Caller holds the lock.

        Ranked, because "something is wrong" is useless if it cannot say how
        wrong. The order is the order an operator acts in:

          error  a latched fault, a lost critical device, an EMCY alarm still
                 standing, or a monitored value past its TRIP limit. The vehicle
                 has stopped or should.
          warn   a non-critical device lost (auto unavailable, manual fine), a
                 statusword warning flag, or a value past its WARN limit.
          ok     nothing outstanding.

        Deliberately reports the FIRST reason at each level rather than all of
        them: the rail is one line, and the alarms page carries the full list.
        """
        hw = self._health or {}

        if self._fault:
            return {"level": "error", "active": True, "detail": self._fault,
                    "source": "fault"}
        if hw.get("system_error"):
            return {"level": "error", "active": True,
                    "detail": hw.get("system_detail") or "critical device lost",
                    "source": "health"}
        for nid in config.NODES:
            al = self._alarms.get(nid)
            if al and al.get("level") == "error":
                return {"level": "error", "active": True,
                        "detail": f"node {nid} {al.get('name') or al.get('hex')}",
                        "source": "emcy"}
        for node_id, vals in (mon.get("nodes") or {}).items():
            for key, v in vals.items():
                if v.get("state") == "trip":
                    return {"level": "error", "active": True,
                            "detail": f"node {node_id} {v.get('label')} "
                                      f"{v.get('value')}{v.get('unit')}",
                            "source": "monitor"}

        if hw.get("sensor_error"):
            return {"level": "warn", "active": True,
                    "detail": hw.get("sensor_detail") or "sensor lost",
                    "source": "health"}
        for node_id, vals in (mon.get("nodes") or {}).items():
            for key, v in vals.items():
                if v.get("state") == "warn":
                    return {"level": "warn", "active": True,
                            "detail": f"node {node_id} {v.get('label')} "
                                      f"{v.get('value')}{v.get('unit')}",
                            "source": "monitor"}
        for node_id, fl in (flags or {}).items():
            for f in fl:
                if f.get("level") in ("warn", "error"):
                    return {"level": "warn", "active": True,
                            "detail": f"node {node_id} {f.get('name')}",
                            "source": "statusword"}

        return {"level": "ok", "active": False, "detail": "", "source": None}

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            # Built once and shared: the rail below and the /monitor detail read
            # the same sweep, so they can never disagree about a voltage.
            mon = self._mon.snapshot(config.MONITOR_THRESHOLDS)
            flags = {str(n): decode_statusword_flags(
                         self._telemetry[n].get("statusword"))
                     for n in config.NODES}
            return {
                "connected": self.bus is not None,
                "how": self.how,
                "error": self.error,
                "armed": self._armed,
                "mode": self._mode,
                "direction": self._direction,
                "target": {"left": self._target[0], "right": self._target[1]},
                "watchdog_s": max(0.0, self._deadline - now) if self._armed else 0.0,
                "stop_reason": self._last_stop_reason,
                "nodes": {
                    str(n): dict(self._telemetry[n], label=config.NODES[n]) for n in config.NODES
                },
                "rpm_profile": {"full": config.MANUAL_FULL_RPM,
                                "half": config.MANUAL_HALF_RPM},
                "loop": dict(self._loop),
                # Just the high-water mark. The UI fetches /api/events only when
                # this moves, so a quiet vehicle costs no extra requests.
                "rfid": self._rfid.snapshot(),
                "dio": self._dio.snapshot(),
                "imu": self._imu_snapshot(now),
                "blind": self._blind_snapshot(now),
                "horn": {"enabled": config.HORN_ENABLED,
                         "channel": config.HORN_DO_CHANNEL},
                "panel": {"enabled": config.PANEL_ENABLED,
                          "selector": self._panel.mode(),
                          "fault": self._fault,
                          "auto_arm": config.PANEL_MANUAL_AUTO_ARM,
                          "arm_fail": self._arm_fail,
                          "last_action": self._last_action},
                "health": self._health,
                # The shared rail every page shows. Derived here rather than in
                # the browser so all four pages agree on what "alarm" means -
                # three pages computing it three ways is three chances to have
                # one of them quietly say everything is fine.
                "battery": _battery(mon),
                "alarm": self._alarm(mon, flags),
                "can": {
                    "alarms": {str(n): self._alarms[n] for n in config.NODES},
                    "nmt": {str(n): self._nmt_state[n] for n in config.NODES},
                    "emcy_seen": self._emcy_seen,
                    "heartbeat_ms": config.CAN_HEARTBEAT_MS,
                    "monitor": mon,
                    "flags": flags,
                },
                "event_seq": events.latest_seq(),
            }

    # ---- bus thread ------------------------------------------------------

    def _run(self):
        try:
            raw, how = open_bus(config.CAN_BITRATE, config.CAN_CHANNEL,
                                config.CAN_ADAPTER_SERIAL)
        except Exception as e:
            with self._lock:
                self.error = str(e)
            events.error(f"CAN bus unavailable: {e}")
            return
        self.bus = TpdoTap(raw, self._on_emcy, self._on_heartbeat,
                           nodes=config.NODES)
        with self._lock:
            self.how = how
            self.error = None
        events.info(f"bus up on {how} at {config.CAN_BITRATE // 1000} kbps · "
                    f"profile {config.PROFILE_NAME}")
        # Here, not at arm time: liveness has to work before, during and after
        # arming, and must not depend on having armed. Without this call 1017h
        # keeps its factory default of 0 = OFF, can.heartbeat_ms is inert, and
        # the only evidence a drive is alive is the 5 Hz telemetry poll.
        self._enable_heartbeat()

        t_tel = t_mon = 0.0
        # loop_health, not health: `health` is the hardware-health module. Loop
        # timing and device liveness are different questions.
        loop_health = _LoopHealth()
        self._hw.reset()      # a reopened bus starts with no liveness history
        try:
            while not self._stop_evt.is_set():
                t_iter = time.perf_counter()
                self._drain_queue()
                # A queued action runs on THIS thread, so a long one is a window
                # in which nothing polled and nothing could have marked
                # liveness. Judging it would report every source as lost.
                # reset() makes them "never seen" instead, which evaluate()
                # already treats as not-a-fault.
                if 0 < self._hw.min_timeout() < time.perf_counter() - t_iter:
                    self._hw.reset()

                self._panel_scan()

                now = time.monotonic()
                with self._lock:
                    armed, target, deadline, mode = (
                        self._armed, self._target, self._deadline, self._mode)
                target = self._fault_gate(target)

                if armed and now > deadline and target != (0, 0):
                    reason = "watchdog: no keepalive from the browser"
                    # A manual watchdog stop does NOT latch a fault. The
                    # setpoint is already zero, and going again means HOLDING a
                    # control that re-POSTs every 100 ms with the operator's
                    # hand on it - so there is no "restarted by somebody who did
                    # not see why", which is the hazard _set_fault() exists for.
                    # Latching a dropped packet would cost a walk to the panel,
                    # and under panel.manual_auto_arm it would block re-arming.
                    #
                    # *** When an autonomous mode returns this changes. ***
                    # Latched motion that would otherwise resume on its own is
                    # the opposite case and must latch. Tape following did, and
                    # whatever navigates next has to make that choice again
                    # rather than inherit this one by default.
                    #
                    # Emitted here rather than through _set_fault, which is safe
                    # on this 50 Hz path only because the branch is
                    # self-clearing - it zeroes the target below, which makes
                    # its own condition false on the very next tick.
                    events.warn(f"{reason} - setpoint zeroed, "
                                f"hold again to drive")
                    with self._lock:
                        self._direction = "stop"
                        self._target = target = (0, 0)
                        self._last_stop_reason = reason
                    if self._blind is not None or self._blind_start_at:
                        self._abort_blind(reason)

                # Device liveness, a separate question from the operator
                # watchdog above. Pure arithmetic, no bus I/O, deliberately
                # outside the lock.
                hw = self._hw.evaluate(now)
                if hw["changed"] or hw["system_edge"] is not None:
                    target = self._apply_health(hw, armed, target)
                with self._lock:
                    self._health = hw

                if self._blind is not None:
                    target = (self._blind_tick() if armed and mode == "manual"
                              else self._abort_blind("vehicle is no longer armed in MANUAL"))

                # Horn before wheels. Both are only intents at this point -
                # one crosses to the DIO thread, the other goes out as an SDO -
                # but the order says which is meant to lead, and a warning that
                # follows the motion it warns about is decoration.
                self._update_horn(armed, target)

                if armed and self._target_moved(target):
                    self._write_target(target)

                if now - t_tel >= config.TELEMETRY_PERIOD_S:
                    t_tel = now
                    self._poll_telemetry()
                # Diagnostic monitoring runs whether armed or not: a drive
                # overheating or a battery sagging while parked is exactly what
                # you want to have seen BEFORE the next run. Paced like the
                # other polls - at tick rate it would spend two SDO round-trips
                # of every 20 ms budget on values that move thermally.
                if config.MONITOR_ENABLED and now - t_mon >= config.MONITOR_PERIOD_S:
                    t_mon = now
                    self._poll_monitor()
                # One IMU object per poll, display only. Paces itself, because
                # a miss backs off further than a hit - see _poll_imu.
                if config.IMU_ENABLED and now >= self._imu_next:
                    self._poll_imu(now)

                # Pump only the REMAINDER of the period, not a further fixed
                # 20 ms on top of the work. Telemetry and setpoint writes already
                # spend time on the bus - and absorb pushed frames through the
                # tap while they do - so adding a full period afterwards
                # stretched the tick instead of pacing it. The 1 ms floor
                # guarantees the thread always yields.
                spent = time.perf_counter() - t_iter
                stats = loop_health.tick(t_iter, spent)
                if stats:
                    with self._lock:
                        self._loop.update(stats)
                # A tick that overran the command watchdog is an event, not a statistic
                # (Q21): the vehicle went that long without a steering update or an
                # expiry check. Reported once per overrun, with what it cost.
                if spent > config.DRIVER_TIMEOUT_S / 2.0:
                    events.error(f"control loop overran: {spent * 1000:.0f} ms of work in one tick "
                                 f"(budget {config.LOOP_PERIOD_S * 1000:.0f} ms, driver timeout "
                                 f"{config.DRIVER_TIMEOUT_S * 1000:.0f} ms)")
                self._pump(max(0.001, config.LOOP_PERIOD_S - spent))
        finally:
            try:
                self._do_disarm()
            except Exception:
                pass
            try:
                self.bus.shutdown()
            except Exception:
                pass

    def _pump(self, seconds):
        """Idle for `seconds`, routing any sensor frames that turn up."""
        end = time.perf_counter() + seconds
        while True:
            left = end - time.perf_counter()
            if left <= 0:
                return
            m = self.bus.recv(timeout=left)   # tap consumes TPDO1 internally
            if m is None:
                return

    def _drain_queue(self):
        """Run queued web actions - bounded, so the tick after it always runs.

        It used to run the queue to exhaustion, so a burst of requests (each
        preflight is six blocking SDO reads, 0.4 s apiece against a silent
        drive) held off the panel scan and the watchdog for as long as the
        burst lasted.
        """
        for _ in range(QUEUE_SCAN_PER_TICK):
            try:
                action, args, fut = self._q.get_nowait()
            except queue.Empty:
                return
            if not fut.set_running_or_notify_cancel():
                continue                # its HTTP waiter already timed out
            if action in _REFUSED_WHILE_ARMED:
                with self._lock:
                    armed = self._armed
                if armed:
                    fut.set_exception(RuntimeError(
                        f"{action} refused while armed - it blocks the bus "
                        f"thread; disarm first"))
                    continue
            try:
                fut.set_result(getattr(self, "_do_" + action)(*args))
            except Exception as e:
                fut.set_exception(e)
            return                      # one real action per tick

    # ---- primitives ------------------------------------------------------

    def _mark_alive(self, node):
        """A completed SDO transfer proves that node is on the bus.

        Liveness used to be marked ONLY from _poll_telemetry and the heartbeat,
        so the ~20 round-trips inside _do_arm counted for nothing: the monitor
        declared both drives silent while we were mid-conversation with them,
        and an arm - 700-900 ms of blocking work against a 0.6 s timeout -
        reliably produced a false "driver silent" fault the moment it returned.

        .get() rather than [] on purpose: the MLS is node 10 and has no entry
        here, and its liveness is TPDO1 frames, not SDO replies.
        """
        src = self._src_node.get(node)
        if src is not None:
            src.mark_rx()

    def _read(self, node, index, sub=0, fast=False, timeout=None):
        """fast=True skips sdo_read's post-reply collision window.

        That window is 10 ms of pure dead time per read. Six telemetry reads at
        5 Hz therefore stalled the control loop for ~96 ms every 215 ms, which at
        0.8 m/s is 77 mm travelled with no steering update. Collision detection
        belongs in preflight and discovery, not in a loop that has to keep a
        vehicle on a line - so those paths keep the default.

        timeout overrides sdo_read's 0.4 s default. Used where a NON-answer is an
        expected outcome rather than a fault, so waiting the full default would
        just be dead time on a path that has already lost.
        """
        kw = {} if timeout is None else {"timeout": timeout}
        st, val, _, _ = sdo_read(self.bus, node, index, sub,
                                 collision_window=0.0 if fast else 0.01, **kw)
        if st:
            self._mark_alive(node)
        return u32(val) if st else None

    def _read_i32(self, node, index, sub=0, fast=False, timeout=None):
        kw = {} if timeout is None else {"timeout": timeout}
        st, val, _, _ = sdo_read(self.bus, node, index, sub,
                                 collision_window=0.0 if fast else 0.01, **kw)
        if st:
            self._mark_alive(node)
        return struct.unpack("<i", val.ljust(4, b"\0"))[0] if st else None

    def _write(self, node, index, sub, value, size, what):
        # Section 8 of the monitoring plan: the nav stack is READ-MOSTLY. A
        # denied index raises rather than being silently dropped - a command a
        # caller believed had landed is its own hazard.
        #
        # `sub` is passed because an RPDO mapping entry cannot be judged without
        # it: sub 0 is the entry count, sub 1-8 are object references, and the
        # guard applies the deny-list recursively to the object being mapped.
        guard_write(index, value, sub)
        ok, detail = sdo_write(self.bus, node, index, sub, value, size)
        if not ok:
            raise RuntimeError(f"node {node}: {what} ({index:04X}h) failed: {detail}")
        self._mark_alive(node)

    def _nmt(self, command, node):
        self.bus.send(can.Message(arbitration_id=0x000, data=[command, node],
                                  is_extended_id=False))
        time.sleep(0.05)

    def _update_horn(self, armed, target):
        """Sound the horn and lights while motion is COMMANDED. Manual and auto alike.

        No distinction between the modes, because the hazard does not make one:
        a jogged vehicle and a running one are the same 150 kg, and a horn that
        meant two different things would mean neither.

        It follows the SETPOINT, not measured rpm. That puts the sound at the
        moment motion is asked for rather than once the wheels are already
        turning, which is the whole point of a warning - and it accepts the
        other end, where the horn stops while the drives are still ramping
        down. Ending the warning when the vehicle stops being commanded to move
        is the right way round; what is left is a coast that is already over.

        `armed` is redundant against a zero setpoint today, and is stated
        anyway. It is the interlock, and an interlock that is only implied is
        one a later edit removes without noticing.

        Renewed every tick rather than written on the edge: the deadline is
        what drops the coil if this loop dies, so a horn cannot outlive the
        thread that was sounding it. See dio.set_coil().
        """
        if not config.HORN_ENABLED:
            return
        self._dio.set_coil(config.HORN_DO_CHANNEL,
                           bool(armed) and tuple(target) != (0, 0),
                           config.HORN_HOLD_S)

    def _fault_gate(self, target):
        """The one motion-permission rule, applied at the output.

        drive() refuses a jog while a fault is latched; this is the same rule
        on the tick, so nothing that slipped past acceptance - a request that
        raced the fault edge, a code path added later - can reach the drives.
        Pure arithmetic under the lock, no bus I/O.
        """
        if target == (0, 0):
            return target
        with self._lock:
            if self._fault is None:
                return target
            self._direction = "stop"
            self._target = (0, 0)
            self._last_stop_reason = f"fault latched: {self._fault}"
        return (0, 0)

    def _write_target(self, target):
        """Push the setpoint to both drives. The hot path of the whole loop.

        Two routes, chosen by can.use_rpdo:

          RPDO1  one 6-byte frame per node, unacknowledged. A queue append.
          SDO    a blocking round trip per node, ~1.8 ms each.

        The SDO path is what shipped and stays the default until the RPDO path
        has been through the bench checklist - see config.py's can.use_rpdo. The
        cost of keeping both is one branch on a path that is about to get 3.6 ms
        cheaper, which is not a trade worth agonising over.

        rpdo.pack() puts the controlword through the same deny-list the SDO path
        uses, so bit 7 cannot reach a drive by this route either.
        """
        if config.CAN_USE_RPDO:
            for nid, rpm in zip((config.LEFT, config.RIGHT), target):
                rpdo.send(self.bus, nid, rpdo.CW_OPERATION_ENABLED, int(rpm))
        else:
            for nid, rpm in zip((config.LEFT, config.RIGHT), target):
                self._write(nid, 0x60FF, 0, int(rpm), 4, "target velocity")
        self._applied = target

    def _sdo_write_for_rpdo(self, bus, node, index, sub, value, size):
        """The SDO writer rpdo.configure() drives. Guarded, like every other write.

        Adapts _write()'s raise-on-failure style to the (ok, detail) tuple
        rpdo.configure() understands, so that module stays free of this class's
        conventions - it has to serve the ROS drive node too.
        """
        try:
            self._write(node, index, sub, value, size, "RPDO1 setup")
        except Exception as e:                  # noqa: BLE001
            return False, str(e)
        return True, ""

    def _target_moved(self, target):
        """Deadband the setpoint writes - see config.TARGET_DEADBAND_RPM."""
        if self._applied is None:
            return True
        if target == (0, 0) and self._applied != (0, 0):
            return True          # a stop is never deadbanded away
        return any(abs(a - b) >= config.TARGET_DEADBAND_RPM
                   for a, b in zip(target, self._applied))

    # ---- hardware health -------------------------------------------------



    def _apply_health(self, hw, armed, target):
        """React to a hardware-health TRANSITION. Returns the setpoint to write.

        Only called when something actually changed, so the events below fire
        once per edge - the rule events.py exists to protect, since this sits on
        a 50 Hz path where a per-tick emit would flush the whole ring in about
        four seconds.

        A critical loss is treated exactly like a watchdog trip: zero the
        setpoint outright rather than running the profile down. There is no
        point easing to a stop through a driver that has stopped answering.
        """
        for _name, detail, ok in hw["changed"]:
            if ok:
                events.info(f"{detail} is answering again")
            else:
                events.error(f"{detail} stopped answering")

        if hw["system_edge"] is True:
            reason = f"driver silent - {hw['system_detail']}, stopping"
            self._set_fault(reason)
            with self._lock:
                self._direction = "stop"
                self._target = target = (0, 0)
                self._last_stop_reason = reason
            events.error(reason)
        elif hw["system_edge"] is False:
            events.info("drivers answering again - re-arm to continue")
        return target

    # ---- operator panel --------------------------------------------------

    def _set_fault(self, reason):
        """Latch an INVOLUNTARY stop.

        Deliberate stops - Reset, web STOP, the selector - never come through
        here. Only things the vehicle decided for itself: a failed arm, a silent
        driver, a lost line, a dead DI link, a watchdog trip. Start stays
        refused until Reset acknowledges it, so a vehicle that stopped itself
        cannot be restarted with one press by somebody who did not see why.
        """
        with self._lock:
            first = self._fault is None
            self._fault = reason
        if first:                       # edge only; this is reachable at 50 Hz
            events.error(f"FAULT: {reason} - press Reset to clear")

    def _clear_fault(self):
        with self._lock:
            was, self._fault = self._fault, None
        if was:
            events.info(f"fault cleared: {was}")
        return was

    def _note_action(self, what, source):
        with self._lock:
            self._last_action = {"what": what, "source": source}

    def _panel_scan(self):
        """One panel scan. Called every tick from _run(), in every state."""
        if not config.PANEL_ENABLED:
            return
        snap = self._dio.snapshot()
        intent = self._panel.scan(snap.get("di"), bool(snap.get("comms_ok")))
        if not intent.valid:
            # No trusted image, so no decisions - and a pending start is
            # abandoned rather than carried across the gap. The operator's
            # panel is the thing that just went away; going anyway is the
            # wrong way to resolve that.
            if self._blind is not None or self._blind_start_at:
                self._abort_blind("panel image lost")
            # *** Manual jog loses its authority too. *** DIO is not a critical
            # health source, and drive() used to check only cached mode/armed,
            # so a browser still holding an arrow kept the vehicle moving with
            # the panel gone. Zero now, refuse jogs until the panel is back, and
            # make the held direction be released before it counts again.
            with self._lock:
                self._panel_valid = False
                moving = self._target != (0, 0)
                if moving:
                    self._jog_released = False
                self._direction = "stop"
                self._target = (0, 0)
                if moving:
                    self._last_stop_reason = "panel input lost"
            if moving:
                events.warn("panel input lost - manual jog stopped; release "
                            "and press again once the panel is back")
            return
        with self._lock:
            self._panel_valid = True

        # A blind run is panel-started, so this scan - not a browser - is its
        # evidence of a live control path. Without this the manual watchdog
        # would stop it 0.6 s in, blaming a browser nobody is holding.
        if self._blind is not None and intent.mode == panel.MANUAL:
            self._blind_keepalive()

        # Order matters: a selector move disarms, so evaluate it before the
        # buttons decide what to do about the new mode.
        if intent.mode_changed:
            self._panel_mode_changed(intent.mode)
        if intent.reset:
            self._panel_reset(intent.mode)
        if intent.start:
            self._panel_start(intent.mode)

        # Both of these are LEVELS, not edges: what the selector is resting on
        # decides whether the vehicle should be energised, so they are
        # evaluated every scan rather than only when something is pressed.
        self._pending_start()
        self._hold_arm_state(intent.mode)

    def _panel_start(self, mode):
        """Start runs the blind-run plan in MANUAL. In AUTO it has nothing to run.

        *** The AUTO branch is a stub with a reason. *** Start used to arm and
        then launch a tape-following run; that run is gone and no autonomous
        mode has replaced it yet. The button, its debounced edge in
        core/panel.py, and the anti-tie-down rule that stops a taped-down Start
        acting at power-on are all still correct and still tested - what is
        missing is something to start.

        So it reports rather than silently doing nothing: an operator pressing a
        button that does nothing needs to be told which of the two it is. When
        navigation lands, this is where it hooks in, and the fault gate below is
        the behaviour it must keep - a vehicle that stopped itself is not
        restarted by somebody who did not see why.
        """
        with self._lock:
            fault = self._fault
        if fault:
            events.warn(f"Start ignored - fault latched: {fault}. Press Reset.")
            return
        if mode != panel.AUTO:
            # In MANUAL, Start runs the blind-run plan set on /blind, if any.
            # Jogging stays per-direction from the web pad.
            self._blind_panel_start()
            return
        events.warn("Start ignored - this build has no autonomous mode. "
                    "Tape following is retired; navigation is not here yet.")
        self._note_action("start ignored (no autonomous mode)", "panel")

    def _panel_mode_changed(self, mode):
        events.info(f"selector -> {mode.upper()}")
        if self._blind is not None or self._blind_start_at:
            self._abort_blind(f"selector moved to {mode.upper()}")
        # A move INTO manual re-arms through _hold_arm_state on this same scan,
        # so the disarm below is not a round trip to idle and back - it is the
        # mode change itself, which _do_arm cannot do in place.
        self._arm_retry_at = 0.0
        with self._lock:
            armed = self._armed
        if not armed:
            return
        # Mode is fixed at arm time, so a selector move has to go back to
        # idle. It is a deliberate operator action, so it stops without
        # latching a fault.
        self._do_disarm()
        self._note_action(f"disarmed (selector -> {mode})", "panel")

    def _panel_reset(self, mode):
        """Reset now only ever STOPS and ACKNOWLEDGES.

        It used to arm as well, which is why it was being pressed before every
        jog - and a button pressed by reflex has stopped being a decision.
        Arming is a consequence of the selector sitting in MANUAL, so what is
        left here is to stop and to acknowledge.

        Clearing the fault is what lets MANUAL arm itself again, so Reset is
        still the way back from anything the vehicle latched.
        """
        if self._blind is not None or self._blind_start_at:
            # The plan is kept, so the same move can be run again with Start.
            self._abort_blind("stopped from panel (Reset)")
            self._note_action("blind run stopped", "panel")
            return

        # Stop first, whatever else is true. Reset is the button somebody
        # presses when they want motion to end, and it must not be conditional
        # on which mode happens to be live.
        with self._lock:
            moving = self._target != (0, 0)
            self._direction = "stop"
            self._target = (0, 0)
        if moving:
            self._note_action("stopped", "panel")
            return

        was = self._clear_fault()
        # The backoff is cleared with the fault so the next scan retries at
        # once: somebody has just pressed the button that means "try again".
        self._arm_retry_at = 0.0
        self._note_action("fault cleared" if was else "reset", "panel")




    def _pending_start(self):
        """Turn a delayed Start into an actual run once the delay is up."""
        if not self._blind_start_at:
            return
        with self._lock:
            request = self._blind_abort_req
        if request:
            self._abort_blind(request)
        elif self._panel.mode() == panel.AUTO:
            self._abort_blind("selector is not in MANUAL")
        elif time.monotonic() >= self._blind_start_at:
            self._blind_start_at = 0.0
            self._begin_blind()

    # ---- encoders --------------------------------------------------------

    def _read_positions(self):
        """6064h from both drives, or None - a move must not guess its progress."""
        counts = []
        for nid in (config.LEFT, config.RIGHT):
            pos = self._read_i32(nid, 0x6064, fast=True)
            if pos is None:
                return None
            counts.append(pos)
        return tuple(counts)

    def _read_encoder_scale(self):
        """Position counts per WHEEL revolution, or None if not trustworthy.

        608Fh is the control resolution per motor revolution. 6091h is the gear
        ratio the drive applies itself; only 1:1 (motor-shaft steps) or the
        profile's own gear_ratio agree with scaling by vehicle.gear_ratio.
        """
        scales = set()
        for nid in (config.LEFT, config.RIGHT):
            inc, revs = self._read(nid, 0x608F, 1), self._read(nid, 0x608F, 2)
            gm, gs = self._read(nid, 0x6091, 1), self._read(nid, 0x6091, 2)
            if not (inc and revs and gm and gs):
                return None
            drive_gear = gm / gs
            if not (abs(drive_gear - 1.0) < 1e-9
                    or abs(drive_gear - config.GEAR_RATIO) < 1e-9):
                return None
            scales.add(inc / revs * config.GEAR_RATIO)
        return scales.pop() if len(scales) == 1 else None

    def _drives_stopped(self, now):
        """Both speed-zero bits from fresh statuswords; None if not fresh.

        Caller holds the lock.
        """
        nodes = (config.LEFT, config.RIGHT)
        if not all(self._status_seen[n] is not None
                   and now - self._status_seen[n] <= config.DRIVER_TIMEOUT_S
                   for n in nodes):
            return None
        return all(self._telemetry[n].get("speed_zero", False) for n in nodes)

    # ---- encoder-only blind run -------------------------------------------

    def _blind_panel_start(self):
        """PB Start with the selector in MANUAL: run the plan set on /blind."""
        with self._lock:
            plan, armed, mode = self._blind_plan, self._armed, self._mode
        if self._blind is not None or self._blind_start_at:
            return                      # already going, or already about to
        if plan is None:
            events.info("Start ignored - selector is in MANUAL and no blind-run "
                        "plan is set on /blind")
            return
        if not (armed and mode == "manual"):
            events.warn("Start ignored - a blind run needs the vehicle armed in MANUAL")
            return
        if plan["counts_per_wheel_rev"] != self._counts_per_wheel_rev:
            events.warn("Start ignored - the encoder scale changed since the "
                        "blind-run plan was set; Set it again")
            return
        self._blind_start_at = time.monotonic() + config.AUTO_START_DELAY_S
        events.info(f"START pressed - blind run of {len(plan['segments'])} "
                    f"segment(s) in {config.AUTO_START_DELAY_S:.1f} s")
        self._note_action("blind run START", "panel")

    def _blind_keepalive(self):
        with self._lock:
            if self._mode == "manual" and self._armed:
                self._deadline = time.monotonic() + config.MANUAL_WATCHDOG_S

    def _begin_blind(self):
        with self._lock:
            plan, armed, mode, fault = (self._blind_plan, self._armed,
                                        self._mode, self._fault)
        if plan is None or not (armed and mode == "manual") or fault:
            events.warn("blind start abandoned - the plan, MANUAL arming or a "
                        "fault changed during the start delay")
            return
        counts = self._read_positions()
        if counts is None:
            self._set_fault("blind run refused: encoder position (6064h) unreadable")
            return
        now = time.monotonic()
        with self._lock:
            self._blind = blindrun.BlindRun(plan, counts)
            self._blind_last = None
            self._blind_written = 0
            self._blind_tick_at = None
            self._blind_pos_at = now
            self._blind_abort_req = None
            self._deadline = now + config.MANUAL_WATCHDOG_S
            self._last_stop_reason = None
        self._blind_log.open(
            f"profile={config.PROFILE_NAME} blind run: {len(plan['segments'])} "
            f"segment(s) at {plan['ref_rpm']:.0f} r/min reference, "
            f"{plan['counts_per_wheel_rev']:.0f} counts per wheel turn")
        events.info("blind run started - encoder only; the IMU yaw rate is "
                    "recorded, never steered on")

    def _blind_tick(self):
        """One tick of a blind run. Returns the setpoint to write."""
        tick = time.perf_counter()
        dt = (tick - self._blind_tick_at) if self._blind_tick_at else config.LOOP_PERIOD_S
        self._blind_tick_at = tick
        now = time.monotonic()
        with self._lock:
            fault, request = self._fault, self._blind_abort_req
            self._blind_abort_req = None
        if request:
            return self._abort_blind(request)
        if fault:
            return self._abort_blind(f"fault latched: {fault}")
        if self._drives_ready() is False:
            return self._abort_blind("drives left Operation enabled")
        counts = self._read_positions()
        if counts is None:
            if now - self._blind_pos_at > config.DRIVER_TIMEOUT_S:
                return self._abort_blind("encoder position (6064h) unreadable")
            return self._target
        self._blind_pos_at = now

        with self._lock:
            stopped = self._drives_stopped(now)
            left, right = self._blind.update(counts, dt, stopped)
            run = self._blind
        self._emit_blind_results(run)
        if run.phase == blindrun.ABORTED:
            return self._abort_blind(run.reason)

        snap = run.snapshot()
        target = (int(round(left)), int(round(right)))
        with self._lock:
            self._target = target
            self._direction = "blind" if target != (0, 0) else "stop"
            tel = {"rpm_l": self._telemetry[config.LEFT]["rpm"],
                   "rpm_r": self._telemetry[config.RIGHT]["rpm"],
                   "gyro_z_dps": self._imu["gyro_dps"][2]}
        self._blind_log.write({
            "dt": dt, "phase": snap["phase"], "segment": snap["segment"],
            "tgt_l_m": snap["target_m"][0], "tgt_r_m": snap["target_m"][1],
            "prog_l_m": snap["progress_m"][0], "prog_r_m": snap["progress_m"][1],
            "cnt_l": counts[0], "cnt_r": counts[1], "n_l": left, "n_r": right,
            "x_m": snap["pose"]["x_m"], "y_m": snap["pose"]["y_m"],
            "heading_deg": snap["pose"]["heading_deg"], "speed_mps": snap["speed_mps"],
            "loop_ms": dt * 1000.0,
        }, tel)
        if run.phase == blindrun.DONE:
            self._finish_blind()
            return (0, 0)
        return target

    def _emit_blind_results(self, run):
        """Publish segments completed since the last tick: page, CSV, event."""
        new = run.results[self._blind_written:]
        if not new:
            return
        self._blind_written = len(run.results)
        for r in new:
            row = dict(r, time=time.strftime("%Y-%m-%d %H:%M:%S"),
                       profile=config.PROFILE_NAME,
                       counts_per_wheel_rev=run.plan["counts_per_wheel_rev"],
                       log_dir=self._blind_log.dir,
                       spec=";".join(f"{k}={v}" for k, v in r["spec"].items()),
                       speed_motor_rpm=run.plan["ref_rpm"],
                       target_left=r["target_counts"][0], target_right=r["target_counts"][1],
                       final_left=r["final_counts"][0], final_right=r["final_counts"][1],
                       error_left=r["error_counts"][0], error_right=r["error_counts"][1])
            with self._lock:
                self._blind_results.append(row)
                del self._blind_results[:-blindrun.RESULTS_KEPT]
            if self._blind_log.enabled:
                err = runlog.append_result(row)
                if err:
                    events.warn(f"blind result not written to CSV: {err}")
            events.info(f"blind segment {r['segment']} ({r['kind']}): encoder "
                        f"{r['encoder_distance_m']:.4f} m, "
                        f"{r['encoder_heading_deg']:.2f} deg; count error "
                        f"L {r['error_counts'][0]:+d} R {r['error_counts'][1]:+d}")

    def _abort_blind(self, reason):
        """Stop a blind run, or cancel one about to start. Zeroes outright."""
        with self._lock:
            run, pending = self._blind, bool(self._blind_start_at)
            self._blind_start_at = 0.0
            self._blind = None
            self._blind_abort_req = None
            if run is not None:
                run.abort(reason)
                self._blind_last = run.snapshot()
            if run is not None or pending:
                self._target = (0, 0)
                self._direction = "stop"
                self._last_stop_reason = f"blind run stopped: {reason}"
        if run is not None:
            self._blind_log.close()
            events.warn(f"blind run stopped - {reason}")
        elif pending:
            events.warn(f"blind run start cancelled - {reason}")
        return (0, 0)

    def _finish_blind(self):
        with self._lock:
            run, self._blind = self._blind, None
            self._blind_last = run.snapshot()
            self._target = (0, 0)
            self._direction = "stop"
        self._blind_log.close()
        events.info(f"blind run complete - {len(run.results)} segment(s) recorded")

    def _blind_snapshot(self, now):
        """Caller holds the lock."""
        run = self._blind
        return {"plan": self._blind_plan,
                "active": run is not None,
                "starting_in": (max(0.0, self._blind_start_at - now)
                                if self._blind_start_at else None),
                "run": run.snapshot() if run is not None else self._blind_last,
                "results": list(self._blind_results),
                "counts_per_wheel_rev": self._counts_per_wheel_rev,
                "log_dir": self._blind_log.dir}

    def _hold_arm_state(self, mode):
        """Keep the vehicle energised iff the selector says it should be.

        MANUAL is an armed state and AUTO is a disarmed one, so this is a level
        held every scan rather than an edge somebody presses:

          MANUAL  arm, and re-arm if anything de-energised the drives
          AUTO    disarm - there is nothing autonomous to be armed for

        Blocking work - an arm is ~20 SDO round-trips - but it runs on the bus
        thread from the panel scan, which is where every other arm has always
        happened.
        """
        if not config.PANEL_MANUAL_AUTO_ARM:
            return
        with self._lock:
            armed, cur, fault = self._armed, self._mode, self._fault

        if mode == panel.AUTO:
            # Nothing autonomous exists to be armed for, so AUTO is the resting
            # state. When navigation lands this is where it has to decide to
            # stay armed instead.
            if armed:
                self._do_disarm()
                self._note_action("disarmed (auto is idle)", "panel")
            return

        # MANUAL from here on.
        if fault:
            return                      # a latched fault is cleared by a human
        now = time.monotonic()
        if now < self._arm_retry_at:
            return

        if armed and cur == "manual":
            # Armed in software is not the same as torque at the wheels. If the
            # safety chain took the drives out from under us they are sitting in
            # ETO, and re-arming is how the vehicle comes back on its own once
            # the chain is restored - see _drives_ready().
            if self._drives_ready() is False:
                self._arm_retry_at = now + ARM_RETRY_S
                events.warn("drives are no longer enabled - re-arming")
                self._do_disarm()
                self._try_arm("manual")
            return

        if armed:
            self._do_disarm()           # armed in the other mode
        self._try_arm("manual")

    def _drives_ready(self):
        """Are both drives in Operation enabled? None while it cannot be known.

        Read from the 5 Hz telemetry rather than asking the bus, so this costs
        nothing on a path that runs every tick. None - not False - when a
        statusword has not been read yet, because "unknown" must not be allowed
        to trigger a re-arm.
        """
        with self._lock:
            words = [self._telemetry[n].get("statusword") for n in config.NODES]
        if any(w is None for w in words):
            return None
        return all((w & 0x6F) == 0x27 for w in words)

    def _try_arm(self, mode):
        """One auto-arm attempt, with backoff. Never latches a fault.

        A failed arm here is usually the safety chain holding the drives in ETO,
        which is a condition rather than a mistake: latching it would demand a
        Reset for something no operator did, and this whole change exists to
        stop asking for that. So it backs off and tries again, and the vehicle
        re-arms by itself once the chain is restored.
        """
        self._arm_retry_at = time.monotonic() + ARM_RETRY_S
        try:
            self._do_arm(mode)
            if self._arm_fail:
                self._arm_fail = None
            self._note_action(f"armed in {mode}", "panel")
        except Exception as e:          # noqa: BLE001 - a condition, not a fault
            why = str(e)
            if why != self._arm_fail:   # edge only; this retries every 2 s
                self._arm_fail = why
                events.warn(f"cannot arm in {mode}: {why} - retrying")







    def _on_emcy(self, node, data):
        """An alarm was PUSHED by a drive. Routed here by TpdoTap.

        EMCY is itself an event, not a poll, so emitting one line per frame does
        not violate the events.py rule - the drive only sends on a change. A
        repeated identical alarm would come from a drive genuinely re-raising it.
        """
        a = decode_emcy(data)
        label = config.NODES.get(node, node)
        with self._lock:
            self._emcy_seen += 1
            self._alarms[node] = dict(a, node=node, label=label,
                                      t=time.time())
        if a["cleared"]:
            events.info(f"node {node} ({label}) alarms cleared")
            return
        msg = f"node {node} ({label}) ALARM {a['hex']} - {a['name']}"
        if a["note"]:
            msg += f": {a['note']}"
        (events.error if a["level"] == "error" else events.warn)(msg)

    def _on_heartbeat(self, node, state_byte):
        """A drive's producer heartbeat (1017h) landed. Routed by TpdoTap.

        This is the liveness signal health.py actually wants: without it a
        driver that has stopped responding looks identical to one that is idle,
        which is why the plan calls enabling 1017h a MUST.
        """
        src = self._src_node.get(node)
        if src is not None:
            src.mark_rx()
        with self._lock:
            self._nmt_state[node] = decode_nmt(state_byte)

    def _poll_monitor(self):
        """One monitored object per node, round-robin. See canmon.py.

        Deliberately NOT a burst: _poll_telemetry() below already spends most of
        a 20 ms tick, and a late tick is a steering update the vehicle does not
        get. Bounded at two SDO round-trips however long the table grows.
        """
        nxt = self._mon.next_object()
        if nxt is None:
            return
        index, key, ctype = nxt
        for nid in config.NODES:
            if not self._src_node[nid].health(time.monotonic(), config.DRIVER_TIMEOUT_S)["ok"]:
                self._mon.store(nid, key, None)  # silent node: no optional reads (Q21)
                continue
            st, val, _, _ = sdo_read(self.bus, nid, index, 0,
                                     collision_window=0.0,
                                     timeout=self.TELEMETRY_SDO_TIMEOUT_S)
            self._mon.store(nid, key, canmon._decode(ctype, val) if st else None)

    # Telemetry reads are bounded (review Q21): an SDO reply on a healthy bus arrives in
    # ~2 ms, so a 50 ms wait is already 25x that; the 0.4 s default was a preflight
    # figure. Six of those in one burst could stall the control loop for 2.4 s with
    # the drives silent - longer than the command watchdog it is meant to feed.
    TELEMETRY_SDO_TIMEOUT_S = 0.05

    def _poll_telemetry(self):
        for nid in config.NODES:
            src = self._src_node[nid]
            if not src.health(time.monotonic(), config.DRIVER_TIMEOUT_S)["ok"]:
                # A node that has stopped answering is not polled three more times
                # per period: liveness is already lost, and each unanswered read is
                # dead time for the loop. One probe read keeps the door open.
                sw = self._read(nid, 0x6041, fast=True, timeout=self.TELEMETRY_SDO_TIMEOUT_S)
                rpm = err = None
            else:
                sw = self._read(nid, 0x6041, fast=True, timeout=self.TELEMETRY_SDO_TIMEOUT_S)
                rpm = self._read_i32(nid, 0x606C, fast=True, timeout=self.TELEMETRY_SDO_TIMEOUT_S)
                err = self._read(nid, 0x1001, fast=True, timeout=self.TELEMETRY_SDO_TIMEOUT_S)
            # Any answer at all proves the driver is still on the bus. Without
            # this the stale values below simply persist and a driver that has
            # stopped replying looks healthy for as long as the process runs.
            if sw is not None or rpm is not None or err is not None:
                self._src_node[nid].mark_rx()
            with self._lock:
                t = self._telemetry[nid]
                if sw is not None:
                    self._status_seen[nid] = time.monotonic()
                    t["statusword"] = sw & 0xFFFF
                    t["state"] = decode_state(sw)
                    t["fault"] = bool(sw & SW_FAULT)
                    t["remote"] = bool(sw & SW_REMOTE)
                    t["speed_zero"] = bool(sw & SW_SPEED_IS_ZERO)
                if rpm is not None:
                    t["rpm"] = rpm
                if err is not None:
                    t["error_reg"] = err & 0xFF
                label = config.NODES[nid]
                faulted = bool(t.get("fault")) or bool(t.get("error_reg"))
                sw_now = t.get("statusword") or 0
                err_now = t.get("error_reg") or 0
            # Edge only. This runs at 5 Hz, so reporting a standing fault every
            # poll would bury every other event within a minute.
            if faulted != self._fault_seen.get(nid, False):
                self._fault_seen[nid] = faulted
                if faulted:
                    events.error(f"node {nid} ({label}) FAULT - statusword "
                                 f"0x{sw_now:04X}, error reg 0x{err_now:02X}")
                else:
                    events.info(f"node {nid} ({label}) fault cleared")


    def _poll_imu(self, now):
        """Read the next object in _IMU_SCHEDULE. One SDO round-trip, no more.

        A hit advances the cursor and schedules the next read one poll_period_s
        out. A miss does NOT advance - the same object is retried - and backs
        off retry_period_s, so an absent sensor costs one short timeout every
        couple of seconds rather than one on every poll. The values already
        held are left standing and dated by `last`, which is what the page
        uses to grey them.
        """
        field, axis, index, sub = _IMU_SCHEDULE[self._imu_cursor]
        raw = self._read(config.SENSOR_NODE, index, sub, fast=True,
                         timeout=IMU_SDO_TIMEOUT_S)
        if raw is None:
            with self._lock:
                self._imu["misses"] += 1
            self._imu_next = now + config.IMU_RETRY_PERIOD_S
            return
        self._imu_cursor = (self._imu_cursor + 1) % len(_IMU_SCHEDULE)
        self._imu_next = now + config.IMU_PERIOD_S
        # 16-bit objects come back as the low half of a u32; the stamp is the
        # one unsigned quantity.
        value = (raw & 0xFFFF if field == "stamp_ms"
                 else read_imu.s16(raw & 0xFFFF) * _IMU_SCALE[field])
        with self._lock:
            if axis is None:
                self._imu[field] = value
            else:
                self._imu[field][axis] = value
            self._imu["seen"] += 1
            self._imu["last"] = now

    def _imu_snapshot(self, now):
        """Caller holds the lock. Lists are copied: this goes out as JSON from
        another thread while the bus thread keeps writing into them."""
        i = self._imu
        last = i["last"]
        return {
            "enabled": config.IMU_ENABLED,
            "node": config.SENSOR_NODE,
            "gyro_dps": list(i["gyro_dps"]),
            "accel_g": list(i["accel_g"]),
            "yaw_rad": i["yaw_rad"],
            "stamp_ms": i["stamp_ms"],
            "seen": i["seen"],
            "misses": i["misses"],
            "age_s": None if last is None else now - last,
            # A full picture is eight reads; anything older than a couple of
            # sweeps is a sensor that has stopped answering, not a slow one.
            "stale": last is None
                     or now - last > 2 * len(_IMU_SCHEDULE) * config.IMU_PERIOD_S
                                     + config.IMU_RETRY_PERIOD_S,
        }

    # ---- queued actions --------------------------------------------------

    def _do_preflight(self):
        report = []
        ok = True
        for nid, label in config.NODES.items():
            st, _, note, _ = sdo_read(self.bus, nid, 0x1000, 0)
            if st is None:
                report.append(f"node {nid} ({label}): not responding")
                ok = False
                continue
            if "COLLISION" in note:
                report.append(f"node {nid} ({label}): {note}")
                ok = False
                continue
            err = self._read(nid, 0x1001)
            sw = self._read(nid, 0x6041)
            if err is None or sw is None:
                report.append(f"node {nid} ({label}): diagnostics unreadable")
                ok = False
                continue
            report.append(f"node {nid} ({label}): error reg 0x{err:02X}, "
                          f"statusword 0x{sw:04X} ({decode_state(sw)})")
            if err:
                report.append(f"  node {nid}: driver reports a fault - clear it first")
                ok = False
            if not sw & SW_REMOTE:
                report.append(f"  node {nid}: Remote bit clear - controlword ignored "
                              f"(S-ON active, or MEXE02 has the driver)")
                ok = False
            if sw & SW_FAULT:
                report.append(f"  node {nid}: FAULT state")
                ok = False
        return {"ok": ok, "report": report}


    def _enable_heartbeat(self):
        """Turn on the drives' producer heartbeat (1017h). Never fatal.

        The driver default is 0 = OFF, and with it off a drive that has stopped
        responding is indistinguishable from one that is idle. 1017h is a
        standard CANopen object and is on the permitted-write list; it cannot
        influence motion.

        A drive that refuses the write is logged and left alone - health.py
        still has the telemetry-reply fallback, just with coarser resolution.
        """
        if not config.CAN_HEARTBEAT_MS:
            return
        for nid in config.NODES:
            try:
                self._write(nid, 0x1017, 0, config.CAN_HEARTBEAT_MS, 2,
                            "producer heartbeat time")
            except Exception as e:                  # noqa: BLE001
                events.warn(f"node {nid} ({config.NODES[nid]}) would not accept "
                            f"a heartbeat interval ({e}) - liveness falls back "
                            f"to telemetry replies")

    def _do_arm(self, mode):
        """Energise the drives. `mode` is "manual" today - see _hold_arm_state.

        *** Arming excites the motor; it does not free it. *** Reaching CiA 402
        "Operation enabled" excites the motor (opman_can:1391) and the velocity
        loop then holds zero - a servo lock, not a free shaft - and a
        non-excited brake motor has the brake clamped instead (opman_fun:2021).
        Only the FREE input releases it, and FREE (403Eh bit 6) is on the write
        deny-list. So a disarmed vehicle cannot be pushed either.
        """
        # A driver that is not answering blocks every mode, dry run included -
        # the preflight below would fail anyway, but this says why in the terms
        # the event log has already been using.
        with self._lock:
            hw = self._health
        if hw.get("system_error"):
            raise RuntimeError(f"driver not answering: {hw['system_detail']}")

        pre = self._do_preflight()
        if not pre["ok"]:
            raise RuntimeError("preflight failed: " + "; ".join(pre["report"]))

        # PDO mapping belongs in PRE-OPERATIONAL (CiA 301), so this runs before
        # the NMT start below rather than beside the 6060h/6083h writes further
        # down. The sequence disables the PDO before remapping it, so a drive
        # that tolerates a live remap is not relied on to.
        if config.CAN_USE_RPDO:
            for nid in config.NODES:
                rpdo.configure(self.bus, nid, self._sdo_write_for_rpdo)

        # From the first NMT start onward a drive may be enabled, and _armed
        # is only set at the very end. Any exit before then - a statusword
        # that never reached Operation enabled, a write that timed out on the
        # SECOND drive after the first was already excited - has to run the
        # de-energise sequence for BOTH drives, so force=True: _do_disarm()
        # would otherwise see _armed False and return without touching them.
        try:
            self._arm_drives(mode)
        except BaseException:
            self._do_disarm(force=True)
            raise

        # The blind run plans in wheel counts, so the scale is read here, once
        # per arm, rather than trusted from a previous session or the profile.
        self._counts_per_wheel_rev = self._read_encoder_scale()
        if self._counts_per_wheel_rev is None:
            events.warn("encoder scale (608Fh/6091h) unreadable or inconsistent "
                        "- blind runs unavailable this arm")
        else:
            events.info(f"encoder scale {self._counts_per_wheel_rev:.0f} "
                        f"counts per wheel turn (blind run)")

        self._applied = None
        with self._lock:
            self._armed = True
            self._mode = mode
            self._direction = "stop"
            self._target = (0, 0)
            self._last_stop_reason = None
            self._deadline = time.monotonic() + config.MANUAL_WATCHDOG_S
        report = list(pre["report"])
        events.info(f"armed in {mode} mode")
        return {"ok": True, "report": report}

    def _arm_drives(self, mode):
        """NMT start, mode and ramps, then the 402 enable sequence, per drive."""
        for nid in config.NODES:
            self._nmt(0x01, nid)

        ramp = config.RAMP[mode]
        for nid in config.NODES:
            self._write(nid, 0x6060, 0, 3, 1, "modes of operation = pv")
            self._write(nid, 0x6083, 0, ramp["accel"], 4, "profile acceleration")
            self._write(nid, 0x6084, 0, ramp["decel"], 4, "profile deceleration")
            self._write(nid, 0x60FF, 0, 0, 4, "target velocity = 0")
            for cw, name in ((CW_SHUTDOWN, "Shutdown"),
                             (CW_SWITCH_ON, "Switch On"),
                             (CW_ENABLE, "Enable Operation")):
                self._write(nid, 0x6040, 0, cw, 2, name)
                time.sleep(0.05)
            sw = self._read(nid, 0x6041) or 0
            if (sw & 0x6F) != 0x27:
                raise RuntimeError(f"node {nid} did not reach Operation enabled "
                                   f"(statusword 0x{sw:04X}, {decode_state(sw)})")

    def _do_disarm(self, force=False):
        """Zero the setpoint, wait for the ramp, then de-energise. Never raises.

        force=True runs the de-energise sequence even though software never
        reached armed - the rollback of an arm that failed part-way, with one
        drive already in Operation enabled.

        A blind run, active or counting down to start, dies here too. The
        loop would abort an active one on its next tick, but under MANUAL
        auto-arm the panel scan re-arms on that same tick, so a run that was
        not cancelled HERE resumed without a new Start.
        """
        with self._lock:
            was_armed = self._armed
            self._armed = False
            self._direction = "stop"
            self._target = (0, 0)
        if not was_armed and not force:
            return {"ok": True}
        self._abort_blind("disarmed")
        if was_armed:
            self._clear_fault()         # web disarm is an acknowledgement too
            events.info("disarmed")
        else:
            events.warn("arm did not complete - de-energising both drives")

        for nid in config.NODES:
            try:
                sdo_write(self.bus, nid, 0x60FF, 0, 0, 4)
            except Exception:
                pass
        end = time.time() + 4.0
        while time.time() < end:
            try:
                if all((self._read(n, 0x6041) or 0) & SW_SPEED_IS_ZERO for n in config.NODES):
                    break
            except Exception:
                break
            time.sleep(0.05)
        for nid in config.NODES:
            for cw in (CW_SHUTDOWN, CW_DISABLE_VOLTAGE):
                try:
                    sdo_write(self.bus, nid, 0x6040, 0, cw, 2)
                except Exception:
                    pass
        try:
            self._nmt(0x80, 0)            # everything back to Pre-operational
        except Exception:
            pass
        self._applied = None
        with self._lock:
            self._mode = "idle"
        return {"ok": True}



