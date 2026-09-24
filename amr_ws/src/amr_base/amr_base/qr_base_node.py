"""qr_base_node: the ONE owner of the AMR QR base hardware (profile platform qr_analog).

Replaces drive_node + panel_node on this platform with the SAME ROS interface, so the mux,
odometry, EKF, IMU bias, executor, supervisor and web run unchanged:

    /cmd_wheel_vel (WheelVelocities)  -> wheel loop -> CKDA08ETH voltages + CK5162E coils
    CANopen encoders 0x6004 (TPDO/SDO)-> /wheel_states (WheelStates)
    WitMotion serial 0x52/0x53        -> /imu/data_raw (sensor_msgs/Imu, yaw rate)
    CK5162E inputs (E-stop/START/STOP)-> /amr/panel_state (virtual selector, qr_panel)
                                       -> /amr/io (IoImage, 5 Hz)
    arm / fault / E-stop state        -> /drives/status (DriveStatus, 10 Hz)
    /diagnostics (1 Hz), /amr/events (transitions only)

Services (std_srvs/Trigger): /drives/arm, /drives/disarm, /drives/ack_fault.

Threads - each device has exactly one:
    dio      drivers/dio.DioLink: Modbus scan of the CK5162E, expiring coil claims
    can      encoders (qr_encoders.EncoderPair over canopen.Router)
    imu      serial read + WitParser
    control  50 Hz: panel image, command gate, DriveLogic, analog write, coil claims,
             publishing. The ONLY thread that decides an output.
rclpy spins on the main thread (subscriptions, services).

Independent gates before any voltage (same as drive_node): a /cmd_wheel_vel older than
cmd_timeout_s is zero; with require_supervisor a fresh ControlLease whose generation
matches the command is needed (gating.drive_gate). The mux already requires the panel
and DriveStatus.operational; this node additionally forces rest on E-stop, on a stale
DIO image, stale encoders or a failed analog write (latched FAULT while armed).

Exit (SIGINT): 0 V on both channels, then every coil claim driven low by DioLink.stop()
(which waits for the readback) - direction inputs off, brake released, as the QR
controller's shutdown_system() left it.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from amr_base import canopen, gating, qr_motor
from amr_base.agv_repo import config
from amr_base.legacy_guard import refuse_if_legacy_running
from amr_base.qr_analog import AnalogOut
from amr_base.qr_encoders import EncoderPair
from amr_base.qr_panel import VirtualSelector
from amr_base.wit_imu import NO_DATA, WitParser, YawRateSource
from amr_interfaces.msg import (
    ControlLease,
    DriveStatus,
    Event,
    IoImage,
    PanelState,
    WheelStates,
    WheelVelocities,
)

import dio  # noqa: E402  (repo module: drivers/dio.py)
import ownerlock  # noqa: E402
from verify_drivers import open_bus  # noqa: E402

SENSOR_DATA = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)
RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)
BIG = 1e6
IO_FRESH_S = 0.3  # a DIO image or analog write older than this is not a live I/O link


def loop_params() -> qr_motor.LoopParams:
    return qr_motor.LoopParams(
        gear_ratio=config.GEAR_RATIO,
        rpm_per_volt=config.QR_FF_MOTOR_RPM_PER_VOLT,
        offset_v=config.QR_FF_OFFSET_V,
        v_max=config.QR_V_MAX_V,
        kp=config.QR_KP_V_PER_RAD_S,
        ki=config.QR_KI_V_PER_RAD,
        i_max=config.QR_I_MAX_V,
        zero_rad_s=config.QR_ZERO_RAD_S,
        dwell_s=config.QR_DIR_DWELL_S,
        runaway_rad_s=config.QR_RUNAWAY_RAD_S,
        runaway_s=config.QR_RUNAWAY_S,
        stall_s=config.QR_STALL_S,
        brake_on_stop=config.QR_BRAKE_ON_STOP,
    )


def int32(x: int) -> int:
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x >= 1 << 31 else x


class QrBaseNode(Node):
    def __init__(self) -> None:
        super().__init__("qr_base_node")
        if config.PLATFORM != config.PLATFORM_QR:
            raise RuntimeError(
                f"qr_base_node needs a platform qr_analog profile; {config.PROFILE_NAME} is "
                f"{config.PLATFORM} (set AGV_PROFILE, e.g. amr-qr-01)"
            )
        dp = self.declare_parameter
        dp("rate_hz", 1.0 / config.LOOP_PERIOD_S)
        dp("cmd_timeout_s", 0.2)
        dp("auto_arm", True)
        dp("require_supervisor", False)
        dp("lease_timeout_s", 0.3)
        dp("imu_enabled", bool(config.IMU_ENABLED))
        dp("gyro_sign", config.QR_IMU_GYRO_SIGN)  # profile qr_base.imu_gyro_sign; CCW spin = positive
        dp("gyro_var", 1e-5)  # (rad/s)^2, placeholder until measured parked (imu_bias logs sigma)
        dp("imu_frame", "imu_frame")
        dp("can_retry_s", 2.0)
        p = self.get_parameter
        self.period = 1.0 / float(p("rate_hz").value)
        self.cmd_timeout = float(p("cmd_timeout_s").value)
        self.imu_enabled = bool(p("imu_enabled").value)
        self.gyro_sign = float(p("gyro_sign").value)
        self.gyro_var = float(p("gyro_var").value)
        self.imu_frame = str(p("imu_frame").value)
        self.can_retry = float(p("can_retry_s").value)
        self._gate = gating.Params(
            require_supervisor=bool(p("require_supervisor").value),
            lease_timeout_s=float(p("lease_timeout_s").value),
        )

        lp = loop_params()
        self.logic = qr_motor.DriveLogic(
            qr_motor.WheelLoop(lp, config.INVERT_LEFT, "left"),
            qr_motor.WheelLoop(lp, config.INVERT_RIGHT, "right"),
            auto_arm=bool(p("auto_arm").value),
        )
        self.selector = VirtualSelector(
            config.PANEL_DI_START,
            config.PANEL_DI_RESET,
            config.QR_DI_ESTOP,
            config.QR_ESTOP_ACTIVE_LOW,
            config.PANEL_DEBOUNCE_SCANS,
            config.QR_AUTO_HOLD_S,
            (
                (
                    config.PENDANT_DI_FWD,
                    config.PENDANT_DI_RVS,
                    config.PENDANT_DI_LEFT,
                    config.PENDANT_DI_RIGHT,
                )
                if config.PENDANT_ENABLED
                else None
            ),
        )
        self.ao = AnalogOut(
            config.QR_AO_IP,
            config.QR_AO_PORT,
            config.QR_AO_DEVICE_ID,
            config.QR_AO_TIMEOUT_S,
            config.QR_AO_REGISTER_BASE,
            config.QR_AO_COUNTS_PER_VOLT,
            config.QR_AO_FULL_SCALE_V,
            config.QR_AO_CH_LEFT,
            config.QR_AO_CH_RIGHT,
        )
        self.coils = {
            "left": (config.QR_DO_LEFT_FWD, config.QR_DO_LEFT_REV, config.QR_DO_LEFT_BRK),
            "right": (config.QR_DO_RIGHT_FWD, config.QR_DO_RIGHT_REV, config.QR_DO_RIGHT_BRK),
        }

        self._lock = threading.Lock()
        self._cmd: tuple[float, float, float] | None = None  # (t_mono, wl, wr)
        self._cmd_gen = 0
        self._bad_cmds = 0
        self._lease: gating.Lease | None = None
        self._gate_reason: str | None = None
        self._requests: list[tuple[str, threading.Event, dict]] = []
        self._stop = threading.Event()
        self._enc_lock = threading.Lock()
        self._enc: dict[str, tuple | None] = {"left": None, "right": None}  # (t, counts, rad, rad_s)
        self._can_state = {
            "state": "starting",
            "detail": "",
            "sources": "-/-",
            "sdo_reads": 0,
            "sdo_timeouts": 0,
        }
        self._imu_state = {"mode": NO_DATA, "samples": 0, "bad": 0, "t": None, "yaw_deg": None, "detail": ""}
        self._event_seq = 0
        self._last_out: qr_motor.Output | None = None

        self._pub_wheels = self.create_publisher(WheelStates, "/wheel_states", SENSOR_DATA)
        self._pub_status = self.create_publisher(DriveStatus, "/drives/status", RELIABLE_1)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self._pub_event = self.create_publisher(Event, "/amr/events", 50)
        self._pub_imu = self.create_publisher(Imu, "/imu/data_raw", SENSOR_DATA)
        self._pub_panel = self.create_publisher(PanelState, "/amr/panel_state", 10)
        self._pub_io = self.create_publisher(IoImage, "/amr/io", 5)
        self.create_subscription(WheelVelocities, "/cmd_wheel_vel", self._on_cmd, RELIABLE_1)
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, RELIABLE_1)
        self.create_service(Trigger, "/drives/arm", lambda q, r: self._request("arm", r))
        self.create_service(Trigger, "/drives/disarm", lambda q, r: self._request("disarm", r))
        self.create_service(Trigger, "/drives/ack_fault", lambda q, r: self._request("ack", r))

        # Ownership before any I/O (plan §9.2): one process per device.
        self._locks = [ownerlock.acquire("dio", "qr_base_node"), ownerlock.acquire("can", "qr_base_node")]
        if self.imu_enabled:
            self._locks.append(ownerlock.acquire("imu", "qr_base_node"))
        self.dio = dio.DioLink()
        self.dio.start()
        self._threads = [threading.Thread(target=self._can_run, name="can", daemon=True)]
        if self.imu_enabled:
            self._threads.append(threading.Thread(target=self._imu_run, name="imu", daemon=True))
        self._control = threading.Thread(target=self._control_run, name="control", daemon=True)
        self._threads.append(self._control)
        for t in self._threads:
            t.start()
        self.create_timer(0.5, self._check_threads)
        self.get_logger().info(
            f"profile {config.PROFILE_NAME}: DIO {config.DIO_IP}, AO {config.QR_AO_IP} "
            f"(L ch{config.QR_AO_CH_LEFT} R ch{config.QR_AO_CH_RIGHT}), encoders "
            f"{config.LEFT}/{config.RIGHT} on {config.CAN_CHANNEL} ({config.QR_ENC_MODE}), "
            f"IMU {config.QR_IMU_PORT if self.imu_enabled else 'disabled'}; "
            f"E-stop DI{config.QR_DI_ESTOP}, START DI{config.PANEL_DI_START}, STOP DI{config.PANEL_DI_RESET}"
        )

    def _check_threads(self) -> None:
        if not self._control.is_alive() and not self._stop.is_set():
            self.get_logger().fatal("control thread ended")
            raise SystemExit(1)

    # ---- rclpy thread ----

    def _on_lease(self, msg: ControlLease) -> None:
        with self._lock:
            cur = self._lease
            if cur is not None and (cur.instance, cur.generation) == (msg.instance, int(msg.generation)):
                if int(msg.seq) <= cur.seq:
                    return
            self._lease = gating.Lease(
                time.monotonic(), msg.instance, int(msg.generation), int(msg.seq), int(msg.allowed)
            )

    def _on_cmd(self, msg: WheelVelocities) -> None:
        wl, wr = float(msg.left_rad_s), float(msg.right_rad_s)
        ok = math.isfinite(wl) and math.isfinite(wr)
        with self._lock:
            self._cmd = (time.monotonic(), wl, wr) if ok else None  # R03: nonfinite = nothing
            self._cmd_gen = int(msg.generation)
            if not ok:
                self._bad_cmds += 1
        if not ok:
            self.get_logger().warn(f"/cmd_wheel_vel not finite ({wl}, {wr}): zero", throttle_duration_sec=1.0)

    def _request(self, what: str, res):
        done, box = threading.Event(), {}
        with self._lock:
            self._requests.append((what, done, box))
        if not done.wait(5.0):
            res.success, res.message = False, f"{what}: control thread did not answer"
            return res
        res.success, res.message = box.get("ok", False), box.get("msg", "")
        return res

    # ---- can thread (encoders) ----

    def _can_run(self) -> None:
        while not self._stop.is_set():
            try:
                raw, how = open_bus(config.CAN_BITRATE, config.CAN_CHANNEL, config.CAN_ADAPTER_SERIAL)
            except Exception as e:  # noqa: BLE001 - retried
                self._can_state.update(state="no bus", detail=str(e))
                self.get_logger().error(f"CAN unavailable: {e}", throttle_duration_sec=10.0)
                self._stop.wait(self.can_retry)
                continue
            router = canopen.Router(raw)
            try:
                encs = EncoderPair(
                    router,
                    config.LEFT,
                    config.RIGHT,
                    config.QR_ENC_COUNTS_PER_REV,
                    config.QR_ENC_RANGE_COUNTS,
                    config.QR_ENC_INVERT_LEFT,
                    config.QR_ENC_INVERT_RIGHT,
                    config.QR_ENC_MODE,
                    config.QR_ENC_EVENT_MS,
                    log=self.get_logger().info,
                )
                encs.start()
                self._can_state.update(state="up", detail=how)
                self.get_logger().info(f"encoders on {how}, mode {config.QR_ENC_MODE}")
                self._can_loop(router, encs)
            except Exception as e:  # noqa: BLE001
                self._can_state.update(state="bus error", detail=repr(e))
                self.get_logger().error(f"encoder bus error: {e!r}")
            finally:
                try:
                    router.shutdown()
                except Exception:  # noqa: BLE001
                    pass
            with self._enc_lock:
                self._enc = {"left": None, "right": None}
            self._stop.wait(self.can_retry)

    def _can_loop(self, router, encs: EncoderPair) -> None:
        poll_period = config.QR_ENC_EVENT_MS / 1000.0
        t_poll = 0.0
        while not self._stop.is_set():
            router.pump(0.004)
            now = time.monotonic()
            if now - t_poll >= poll_period:
                t_poll = now
                encs.poll(now)
            snap = {}
            for name, enc in (("left", encs.left), ("right", encs.right)):
                snap[name] = None if enc.counts is None else (enc.t, enc.counts, enc.rad, enc.rad_s)
                for old, new, why in encs.drain_range_changes(enc):
                    self.get_logger().warning(
                        f"encoder {name} (node {enc.node}): wrap range {old} -> {new} ({why})"
                    )
            with self._enc_lock:
                self._enc = snap
            self._can_state.update(
                sources=encs.source_summary(), sdo_reads=encs.sdo_reads, sdo_timeouts=encs.sdo_timeouts
            )

    # ---- imu thread ----

    def _imu_run(self) -> None:
        import serial  # noqa: PLC0415 - pyserial, only on this platform

        src = YawRateSource(sign=self.gyro_sign)
        while not self._stop.is_set():
            try:
                port = serial.Serial(config.QR_IMU_PORT, config.QR_IMU_BAUD, timeout=0.05)
            except Exception as e:  # noqa: BLE001 - retried
                self._imu_state["detail"] = str(e)
                self.get_logger().error(f"IMU unavailable: {e}", throttle_duration_sec=10.0)
                self._stop.wait(2.0)
                continue
            parser = WitParser()
            self._imu_state["detail"] = f"{config.QR_IMU_PORT} @ {config.QR_IMU_BAUD}"
            try:
                port.reset_input_buffer()
                while not self._stop.is_set():
                    data = port.read(max(1, port.in_waiting))
                    if not data:
                        continue
                    t = time.monotonic()
                    for f in parser.feed(data, t):
                        s = src.offer(f)
                        if s is not None:
                            self._publish_imu(s.wz_rad_s)
                            self._imu_state.update(mode=s.mode, t=s.t)
                    self._imu_state.update(samples=src.samples, bad=parser.bad_checksum, yaw_deg=src.yaw_deg)
            except Exception as e:  # noqa: BLE001
                self._imu_state["detail"] = f"read error: {e}"
                self.get_logger().error(f"IMU read error: {e}")
            finally:
                try:
                    port.close()
                except Exception:  # noqa: BLE001
                    pass
            self._stop.wait(2.0)

    def _publish_imu(self, wz: float) -> None:
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.imu_frame
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.z = wz
        m.angular_velocity_covariance = [BIG, 0.0, 0.0, 0.0, BIG, 0.0, 0.0, 0.0, self.gyro_var]
        m.linear_acceleration_covariance[0] = -1.0
        self._safe_publish(self._pub_imu, m)

    # ---- control thread ----

    def _control_run(self) -> None:
        t_status = t_diag = 0.0
        io_n = 0
        try:
            while not self._stop.is_set() and rclpy.ok():
                t0 = time.perf_counter()
                now = time.monotonic()
                self._serve_requests()
                snap = self.dio.snapshot()
                frame = self.selector.tick(snap, now)
                if frame.changed:
                    self.get_logger().info(frame.changed)
                    lvl = (
                        Event.WARN
                        if ("LOST" in frame.changed or "E-STOP ACTIVE" in frame.changed)
                        else Event.INFO
                    )
                    self.event(lvl, "PANEL", frame.changed)
                self._publish_panel(frame)
                io_n = (io_n + 1) % 10
                if io_n == 0:
                    self._publish_io(snap)

                out = self.logic.tick(self._inputs(now, snap, frame))
                if out.event:
                    lvl = Event.ERROR if "FAULT" in out.event else Event.INFO
                    # one severity per call site: rclpy refuses to change it between calls
                    if lvl == Event.ERROR:
                        self.get_logger().error(out.event)
                    else:
                        self.get_logger().info(out.event)
                    self.event(lvl, "DRIVES", out.event)
                self._apply(out)
                self._last_out = out
                self._publish_wheels()
                if now - t_status >= 0.1:
                    t_status = now
                    self._publish_status(out)
                if now - t_diag >= 1.0:
                    t_diag = now
                    self._publish_diagnostics(out, snap)
                spent = time.perf_counter() - t0
                self._stop.wait(max(0.001, self.period - spent))
        finally:
            self._shutdown_outputs()

    def _serve_requests(self) -> None:
        with self._lock:
            reqs, self._requests = self._requests, []
        for what, done, box in reqs:
            try:
                ok, msg = {"arm": self.logic.arm, "disarm": self.logic.disarm, "ack": self.logic.ack}[what]()
                box["ok"], box["msg"] = ok, msg
                self.event(Event.INFO, "DRIVES", f"{what}: {msg}")
            finally:
                done.set()

    def _inputs(self, now: float, snap: dict, frame) -> qr_motor.Inputs:
        with self._lock:
            cmd, gen, lease = self._cmd, self._cmd_gen, self._lease
        reason = gating.drive_gate(now, lease, gen, self._gate)
        if reason != self._gate_reason:
            self._gate_reason = reason
            if reason is not None:
                self.get_logger().warn(f"setpoint gated to zero: {reason}")
        wheels = None
        if reason is None and cmd is not None and now - cmd[0] <= self.cmd_timeout:
            wheels = (cmd[1], cmd[2])

        with self._enc_lock:
            enc = dict(self._enc)
        speeds = {}
        for name in ("left", "right"):
            e = enc.get(name)
            fresh = e is not None and now - e[0] <= config.QR_ENC_TIMEOUT_S and e[3] is not None
            speeds[name] = e[3] if fresh else None

        dio_age = snap.get("rx_age_s")
        dio_ok = bool(snap.get("comms_ok")) and dio_age is not None and dio_age <= IO_FRESH_S
        ao_ok = self.ao.t_ok is not None and now - self.ao.t_ok <= IO_FRESH_S
        do = snap.get("do") or []

        def coils_low(name: str) -> bool | None:
            fwd, rev, _brk = self.coils[name]
            if not dio_ok or max(fwd, rev) >= len(do):
                return None
            return not do[fwd] and not do[rev]

        return qr_motor.Inputs(
            now=now,
            cmd=wheels,
            left_rad_s=speeds["left"],
            right_rad_s=speeds["right"],
            left_coils_low=coils_low("left"),
            right_coils_low=coils_low("right"),
            io_ok=dio_ok and ao_ok,
            # No trusted panel image = E-stop state unknown = treated as pressed.
            estop=frame.estop or not frame.valid,
        )

    def _apply(self, out: qr_motor.Output) -> None:
        hold = config.QR_COIL_HOLD_S
        # FWD and REV of a wheel are never claimed high together: WheelLoop only raises
        # a direction after the READBACK shows both low for dir_dwell_s, so the order in
        # which DioLink writes the claims within a scan cannot overlap them.
        for name, w in (("left", out.left), ("right", out.right)):
            fwd, rev, brk = self.coils[name]
            self.dio.set_coil(fwd, w.fwd, hold)
            self.dio.set_coil(rev, w.rev, hold)
            self.dio.set_coil(brk, w.brake, hold)
        self.ao.write(out.left.volts, out.right.volts)

    def _shutdown_outputs(self) -> None:
        """0 V, then every claim low (direction off, brake released) - QR shutdown_system()."""
        try:
            self.ao.zero()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.dio.stop()  # drives claims low and waits for the readback
        except Exception:  # noqa: BLE001
            pass
        try:
            self.ao.close()
        except Exception:  # noqa: BLE001
            pass

    # ---- publishing ----

    def _safe_publish(self, pub, msg) -> None:
        if self._stop.is_set() or not rclpy.ok():
            return
        try:
            pub.publish(msg)
        except Exception:  # noqa: BLE001 - context going down under SIGINT
            self._stop.set()

    def event(self, level: int, code: str, text: str) -> None:
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source, m.level, m.code, m.text = "qr_base_node", level, code, text
        self._event_seq += 1
        m.seq = self._event_seq
        self._safe_publish(self._pub_event, m)

    def _publish_panel(self, frame) -> None:
        m = PanelState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.valid = frame.valid
        m.mode_auto = frame.mode_auto
        m.start_edge = frame.start_edge
        m.reset_edge = frame.reset_edge
        m.seq = frame.seq
        m.pendant_fwd, m.pendant_rvs, m.pendant_left, m.pendant_right = frame.pendant
        self._safe_publish(self._pub_panel, m)

    def _publish_io(self, snap: dict) -> None:
        m = IoImage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.comms_ok = bool(snap.get("comms_ok"))
        age = snap.get("rx_age_s")
        m.rx_age_s = -1.0 if age is None else float(age)
        m.di_names = list(snap.get("di_names", []))
        m.di = [bool(x) for x in snap.get("di", [])]
        m.do_names = list(snap.get("do_names", []))
        m.do_readback = [bool(x) for x in snap.get("do", [])]
        wanted = snap.get("commanded", {})
        m.do_requested = [bool(wanted.get(str(i), False)) for i in range(len(m.do_names))]
        m.scans, m.errors, m.writes = (
            int(snap.get("scans", 0)),
            int(snap.get("errors", 0)),
            int(snap.get("writes", 0)),
        )
        m.detail = str(snap.get("detail", ""))
        self._safe_publish(self._pub_io, m)

    def _publish_wheels(self) -> None:
        now = time.monotonic()
        with self._enc_lock:
            enc = dict(self._enc)
        el, er = enc.get("left"), enc.get("right")
        if el is None and er is None:
            return  # nothing has ever been read on this bus connection
        m = WheelStates()
        newest = max(e[0] for e in (el, er) if e is not None)
        # Stamped at the newest encoder sample, not at publish time: odometry divides
        # position increments by stamp differences.
        m.header.stamp = (self.get_clock().now() - Duration(seconds=max(0.0, now - newest))).to_msg()
        cpr = config.QR_ENC_COUNTS_PER_REV
        counts = {}
        for name, e, inv in (("left", el, config.INVERT_LEFT), ("right", er, config.INVERT_RIGHT)):
            valid = e is not None and now - e[0] <= config.QR_ENC_TIMEOUT_S and e[3] is not None
            pos = e[2] if e is not None else 0.0
            vel = e[3] if (e is not None and e[3] is not None) else 0.0
            setattr(m, f"{name}_pos_rad", float(pos))
            setattr(m, f"{name}_vel_rad_s", float(vel) if valid else 0.0)
            setattr(m, f"{name}_valid", bool(valid))
            # raw counters in DRIVER terms, the convention commissioning undoes with invert_*
            counts[name] = None if e is None else int32(-e[1] if inv else e[1])
        if counts["left"] is not None and counts["right"] is not None:
            m.left_counts, m.right_counts = counts["left"], counts["right"]
            m.counts_valid = bool(m.left_valid and m.right_valid)
        m.counts_per_wheel_rev = float(cpr)
        self._safe_publish(self._pub_wheels, m)

    @staticmethod
    def _wheel_text(w: qr_motor.WheelOut) -> str:
        if w.phase == qr_motor.DRIVE:
            return f"drive {'fwd' if w.fwd else 'rev'} {w.volts:.2f} V"
        return f"{w.phase}{' (brake)' if w.brake else ''}"

    def _publish_status(self, out: qr_motor.Output) -> None:
        m = DriveStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        if out.estop:
            m.left_state = m.right_state = "E-STOP"
        elif out.state == qr_motor.FAULT:
            m.left_state = m.right_state = f"FAULT: {out.fault_reason}"
        elif out.state == qr_motor.DISARMED:
            m.left_state = m.right_state = "disarmed"
        else:
            m.left_state, m.right_state = self._wheel_text(out.left), self._wheel_text(out.right)
        m.operational = bool(out.operational)
        self._safe_publish(self._pub_status, m)

    def _publish_diagnostics(self, out: qr_motor.Output, snap: dict) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()
        level = DiagnosticStatus.OK
        if out.state == qr_motor.FAULT:
            level = DiagnosticStatus.ERROR
        elif out.estop or out.state != qr_motor.ARMED:
            level = DiagnosticStatus.WARN
        drv = DiagnosticStatus(name="qr/drives", hardware_id=config.PROFILE_NAME, level=level)
        drv.message = "E-STOP" if out.estop else out.state
        drv.values = [
            KeyValue(key="state", value=out.state),
            KeyValue(key="fault_reason", value=out.fault_reason),
            KeyValue(key="estop", value=str(out.estop)),
            KeyValue(key="left", value=self._wheel_text(out.left)),
            KeyValue(key="right", value=self._wheel_text(out.right)),
            KeyValue(key="setpoint_gate", value=self._gate_reason or "open"),
            KeyValue(key="nonfinite_cmds", value=str(self._bad_cmds)),
            KeyValue(key="require_supervisor", value=str(self._gate.require_supervisor)),
        ]
        arr.status.append(drv)
        ao = DiagnosticStatus(name="qr/analog", hardware_id=config.QR_AO_IP)
        ao.level = DiagnosticStatus.OK if self.ao.ok else DiagnosticStatus.ERROR
        ao.message = self.ao.detail
        ao.values = [
            KeyValue(key="volts_left", value=f"{self.ao.volts[0]:.3f}"),
            KeyValue(key="volts_right", value=f"{self.ao.volts[1]:.3f}"),
            KeyValue(key="writes", value=str(self.ao.writes)),
            KeyValue(key="errors", value=str(self.ao.errors)),
        ]
        arr.status.append(ao)
        dio_st = DiagnosticStatus(name="qr/dio", hardware_id=config.DIO_IP)
        dio_st.level = DiagnosticStatus.OK if snap.get("comms_ok") else DiagnosticStatus.ERROR
        dio_st.message = str(snap.get("detail", ""))
        dio_st.values = [
            KeyValue(key=k, value=str(snap.get(k))) for k in ("rx_age_s", "scans", "errors", "writes")
        ]
        arr.status.append(dio_st)
        with self._enc_lock:
            enc = dict(self._enc)
        cs = self._can_state
        enc_st = DiagnosticStatus(name="qr/encoders", hardware_id=config.CAN_CHANNEL)
        ages = {n: (None if enc.get(n) is None else now - enc[n][0]) for n in ("left", "right")}
        fresh = all(a is not None and a <= config.QR_ENC_TIMEOUT_S for a in ages.values())
        enc_st.level = DiagnosticStatus.OK if fresh else DiagnosticStatus.ERROR
        enc_st.message = f"{cs['state']} {cs['detail']}"
        enc_st.values = (
            [
                KeyValue(key="sources", value=cs["sources"]),
                KeyValue(key="sdo_reads", value=str(cs["sdo_reads"])),
                KeyValue(key="sdo_timeouts", value=str(cs["sdo_timeouts"])),
            ]
            + [KeyValue(key=f"{n}_age_s", value="" if a is None else f"{a:.3f}") for n, a in ages.items()]
            + [
                KeyValue(key=f"{n}_counts", value="" if enc.get(n) is None else str(enc[n][1]))
                for n in ("left", "right")
            ]
        )
        arr.status.append(enc_st)
        im = self._imu_state
        imu_st = DiagnosticStatus(name="qr/imu", hardware_id=config.QR_IMU_PORT)
        age = None if im["t"] is None else now - im["t"]
        if not self.imu_enabled:
            imu_st.level, imu_st.message = DiagnosticStatus.OK, "disabled"
        elif age is None or age > config.QR_IMU_TIMEOUT_S:
            imu_st.level, imu_st.message = DiagnosticStatus.ERROR, f"no yaw rate ({im['detail']})"
        elif im["mode"] != "gyro":
            imu_st.level = DiagnosticStatus.WARN
            imu_st.message = "yaw rate differentiated from 0x53 angle: configure the unit to send 0x52"
        else:
            imu_st.level, imu_st.message = DiagnosticStatus.OK, "gyro"
        imu_st.values = [
            KeyValue(key="mode", value=str(im["mode"])),
            KeyValue(key="samples", value=str(im["samples"])),
            KeyValue(key="bad_checksum", value=str(im["bad"])),
            KeyValue(key="yaw_deg", value="" if im["yaw_deg"] is None else f"{im['yaw_deg']:.2f}"),
            KeyValue(key="gyro_sign", value=str(self.gyro_sign)),
        ]
        arr.status.append(imu_st)
        self._safe_publish(self._pub_diag, arr)

    def stop(self) -> None:
        self._stop.set()
        self._control.join(timeout=5.0)  # its finally: zero volts, coils low


def main(args=None) -> None:
    refuse_if_legacy_running("qr_base_node")
    rclpy.init(args=args)
    node = QrBaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.stop()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
