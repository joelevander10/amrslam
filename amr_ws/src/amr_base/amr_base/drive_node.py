"""drive_node (T9 + T10): the ONE owner of can0 - both BLV-R drives and the MLS gyro.

    /cmd_wheel_vel  (WheelVelocities, mux, 50 Hz)  ->  RPDO1 to both drives
    TPDO1/TPDO2 from both drives                   ->  /wheel_states  (WheelStates)
    MLS 2034h:3 (TPDO or SDO poll)                 ->  /imu/data_raw  (sensor_msgs/Imu)
    MLS TPDO1 track (or SDO poll, read-only)       ->  /amr/line_track (LineTrack)
    MLS TPDO1 marker code (profile mls.*)          ->  /amr/line_marker (MarkerEvent)
    MLS event flag -> 2080h cause, 2070h temp      ->  /amr/events MLS_FIELD_WEAK / MLS_TEMP
    CiA-402 state, alarms, liveness                ->  /drives/status (DriveStatus, 10 Hz)

    /amr/commissioning_pp (PpMove, held 50 Hz)     ->  profile-position blind move
    /drives/pp_status (PpStatus, 10 Hz)            <-  its state and last result

Services (std_srvs/Trigger): /drives/arm, /drives/disarm, /drives/ack_fault.

Drive power (2026-10-08). The link and the power stage are separate levels:
STANDBY (CANopen up, TPDOs flowing, power stage off) whenever the link is wanted,
ARMED (Operation enabled) only while amr_base.arm_policy wants power - under MANUAL
while the Manual Arm input (DI08) is high, under AUTO while the line or the
trackless run asks for it (LineState / RunState.drive_power). /drives/arm and
/drives/disarm set the LINK (standby or fully disarmed), not the power.

Profile position (amr_base/pp.py) is LOCKED by the profile (pp.enabled false)
until the decision to run it on this motor is recorded (pp.vendor_ref). While a pp move is active the
bus thread sends the pp controller's controlwords instead of /cmd_wheel_vel, and
halts on its own authority check (lease with COMMISSIONING and the move's
generation, a fresh MANUAL panel - required even without require_supervisor), on
a stale hold, a following error or the time limit.

One bus thread does everything on the wire, in this order every tick:
service requests, arm policy, setpoint, PC heartbeat, IMU poll, feedback
publish, then pumps the bus for the rest of the period (TPDOs, EMCY and
heartbeats are decoded inside that pump). rclpy spins on the main thread.

Independent command watchdog: a /cmd_wheel_vel older than cmd_timeout_s is a
zero setpoint. Independent supervisor gate (unified plan §4.3 item 7): with
`require_supervisor` a fresh ControlLease whose generation matches the wheel
command's is needed for any nonzero setpoint - so a mux that keeps publishing
after its own control-plane subscription stalled cannot move the vehicle.
Independent PC-loss response on the drive side: see
amr_base.canopen (1016h). Never runs beside another can0 owner (ownerlock)
and refuses to start if the bus cannot be opened.
"""

from __future__ import annotations

import math
import queue
import threading
import time

import rclpy
from agv_core import (
    alarms,  # noqa: E402
    canmon,  # noqa: E402
    config,
    ownerlock,  # noqa: E402
)
from agv_core.drivers.canbus.verify_drivers import open_bus  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from amr_base import arm_policy, canopen, gating, pp
from amr_base.marker_events import MarkerDetector
from amr_base.mls_imu import ImuSample, MlsImu
from amr_base.mls_track import ERROR, WARN, MlsTrack, RateLimit, TrackSample
from amr_interfaces.msg import (
    ControlLease,
    DriveStatus,
    Event,
    LineState,
    LineTrack,
    MarkerEvent,
    PanelState,
    PpMove,
    PpStatus,
    RunState,
    WheelStates,
    WheelVelocities,
)

SENSOR_DATA = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)
RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
# /amr/line_marker (mls-marker-plan 5.4): every marker is a message, none may be dropped
# behind a newer one, and a replayed old one must never look new.
MARKER_QOS = QoSProfile(
    depth=50, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)
BIG = 1e6


def marker_expectation(polarity_lock: bool, teach_lock: bool) -> dict:
    """Profile mls.* -> the MLS objects and values the sensor must hold for markers to be
    trusted (read-only check in MlsTrack). Markers are always on (2026-10-23). The same
    values the bench tool writes (read_mls.MARKER_SETTINGS)."""
    want = {(0x2028, 1): 1, (0x2028, 2): 1, (0x2028, 3): 1}
    if teach_lock:
        want[(0x2029, 0)] = 1
    if polarity_lock:
        want[(0x202D, 5)] = 1
    return want


def _fault_code(reason: str) -> str:
    """canopen.decide()'s fault reason -> the catalogue code the operator sees."""
    r = (reason or "").lower()
    if "silent" in r:
        return "DRIVE_SILENT"
    if "alarm" in r:
        return "DRIVE_ALARM"
    return "DRIVE_FAULT"


TRACK_UI_PERIOD_S = 0.2  # /amr/line_track_ui, the display copy
FIELD_EVENT_MIN_S = 2.0  # MLS_FIELD_WEAK: at most one operator event this often (mls-diagnostics-plan M1)
WHEELS_UI_PERIOD_S = 0.2  # /wheel_states_ui, the display copy (Home speed tile, 2026-10-08)

class DriveNode(Node):
    def __init__(self) -> None:
        super().__init__("drive_node")
        dp = self.declare_parameter
        dp("rate_hz", 1.0 / config.LOOP_PERIOD_S)
        dp("cmd_timeout_s", 0.2)
        dp("feedback_hz", 50.0)  # TPDO event timer; 100 Hz once the 125 kbps bus is proven to carry it
        dp("driver_timeout_s", config.DRIVER_TIMEOUT_S)
        dp("auto_arm", True)  # bring the link up (STANDBY) at start; power follows arm_policy
        dp("power_panel_timeout_s", 1.0)  # arm_policy: a panel image older than this = no power
        dp("power_ask_timeout_s", 2.5)  # arm_policy: a LineState/RunState request older than this
        dp("arm_retry_s", 2.0)
        dp("pc_node_id", 100)
        dp("pc_heartbeat_ms", 100)
        dp("pc_loss_ms", 500)  # 1016h on the drives; 0 disables the drive-side response
        dp("ramp", "auto")  # profile drivers.ramp.<name>
        dp("imu_enabled", config.IMU_ENABLED)
        dp("imu_poll_hz", 50.0)
        dp("imu_stamp_every", 10)
        dp("gyro_sign", 1.0)  # VERIFY on the vehicle: CCW spin must read positive
        dp("gyro_var", 1e-6)  # (rad/s)^2; measured sigma 0.029 deg/s and LSB 0.061 deg/s
        dp("imu_frame", "imu_frame")
        # MLS track reading (line-follow plan §1.1), read-only: tpdo | sdo | auto | off.
        # auto listens for TPDO1 and polls by SDO (monitor rate) while none arrive.
        dp("mls_track_mode", "auto")
        dp("mls_track_sdo_hz", 10.0)
        dp("mls_track_variant", config.MLS_VARIANT)  # 2006h:01 as commissioned (profile mls.variant)
        dp("require_supervisor", False)  # production: True (unified plan §4.2)
        dp("lease_timeout_s", 0.3)
        # Diagnostic monitoring (unified plan §7.1): ONE bounded SDO read per slot on the
        # bus thread, below command/feedback/heartbeat work, slower while moving.
        dp("monitor_enabled", bool(config.MONITOR_ENABLED))
        dp("monitor_period_still_s", 0.25)
        dp("monitor_period_moving_s", 1.0)
        dp("pp_hold_timeout_s", 0.2)  # a pp move halts this long after its last hold
        dp("pp_panel_timeout_s", 0.2)
        p = self.get_parameter
        self.period = 1.0 / p("rate_hz").value
        self.cmd_timeout = p("cmd_timeout_s").value
        self.feedback_ms = max(1, int(round(1000.0 / p("feedback_hz").value)))
        self.driver_timeout = p("driver_timeout_s").value
        self.want_link = bool(p("auto_arm").value)
        self._power_params = arm_policy.Params(
            panel_timeout_s=float(p("power_panel_timeout_s").value),
            ask_timeout_s=float(p("power_ask_timeout_s").value),
        )
        self._power: tuple[bool, str] = (False, "starting")
        self._power_panel: arm_policy.Panel | None = None
        self._asks: dict[str, arm_policy.Ask] = {}
        self.arm_retry = p("arm_retry_s").value
        self.pc_node = int(p("pc_node_id").value)
        self.pc_hb_s = p("pc_heartbeat_ms").value / 1000.0
        self.pc_loss_ms = int(p("pc_loss_ms").value)
        self.ramp = config.RAMP[p("ramp").value]
        self.imu_enabled = bool(p("imu_enabled").value)
        self.imu_frame = p("imu_frame").value
        self.gyro_var = p("gyro_var").value

        self.nodes = {config.LEFT: "left", config.RIGHT: "right"}
        self._lock = threading.Lock()
        self._cmd: tuple[float, float, float] | None = None  # (t_mono, wl, wr)
        self._cmd_gen = 0
        self._bad_cmds = 0
        self._lease: gating.Lease | None = None
        self._gate = gating.Params(
            require_supervisor=bool(p("require_supervisor").value),
            lease_timeout_s=float(p("lease_timeout_s").value),
        )
        self._gate_reason: str | None = None
        self._mon = canmon.MonitorPoller(config.NODES) if p("monitor_enabled").value else None
        self._mon_periods = (
            float(p("monitor_period_still_s").value),
            float(p("monitor_period_moving_s").value),
        )
        self._cursor = canopen.MonitorCursor(self._mon, config.NODES) if self._mon is not None else None
        self._bus_stats = {"sdo_timeouts": 0, "monitor_reads": 0}
        self._event_seq = 0
        self._operational = False
        self._edges: dict[str, tuple[str, int, str]] = {}  # slot -> (code, level, detail)
        self._field_events = RateLimit(FIELD_EVENT_MIN_S)
        self._requests: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._status_snapshot: dict = {"state": "starting", "reason": "", "mode": "off"}
        self._pp = pp.Controller(
            hold_timeout_s=float(p("pp_hold_timeout_s").value),
            feedback_timeout_s=2.5 * self.feedback_ms / 1000.0,
            stop_confirm_s=canopen.DriveLink.STOP_CONFIRM_S,
        )
        self._pp_panel_timeout = float(p("pp_panel_timeout_s").value)
        self._pp_req: pp.Request | None = None
        self._pp_bad = ""  # why the last PpMove was not accepted as a request
        self._panel: tuple[float, bool, bool] | None = None  # (t_mono, valid, mode_auto)
        self._track: MlsTrack | None = None  # built on the bus thread
        # /amr/line_track is best-effort, so a dropped sample is invisible
        # without this. Touched only on the bus thread, the sole publisher.
        self._track_seq = 0

        self._pub_wheels = self.create_publisher(WheelStates, "/wheel_states", SENSOR_DATA)
        # 5 Hz copy for displays, as /amr/line_track_ui. Nothing in control reads it.
        self._pub_wheels_ui = self.create_publisher(WheelStates, "/wheel_states_ui", SENSOR_DATA)
        self._wheels_ui_t = 0.0
        self._pub_status = self.create_publisher(DriveStatus, "/drives/status", RELIABLE_1)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self._pub_event = self.create_publisher(Event, "/amr/events", 50)
        self._pub_imu = self.create_publisher(Imu, "/imu/data_raw", SENSOR_DATA)
        self._pub_pp = self.create_publisher(PpStatus, "/drives/pp_status", RELIABLE_1)
        self._pub_track = self.create_publisher(LineTrack, "/amr/line_track", SENSOR_DATA)
        # A 5 Hz copy for displays (CPU, 2026-10-02): the web shows it at 2 Hz and was
        # deserialising all 100 samples a second in Python. Control stays on the full topic.
        self._pub_track_ui = self.create_publisher(LineTrack, "/amr/line_track_ui", SENSOR_DATA)
        self._track_ui_t = 0.0
        # Marker codes (mls-marker-plan): events made on the bus thread, a 1 Hz status heartbeat.
        self._pub_marker = self.create_publisher(MarkerEvent, "/amr/line_marker", MARKER_QOS)
        self._markers = MarkerDetector(config.MLS_MARKER_CODES)
        self._marker_gen_seen = 0
        self.create_timer(1.0, self._marker_heartbeat)
        self.create_subscription(PpMove, "/amr/commissioning_pp", self._on_pp, RELIABLE_1)
        self.create_subscription(PanelState, "/amr/panel_state", self._on_panel, 10)
        self.create_subscription(LineState, "/amr/line_state", lambda m: self._on_ask("line", m), LATCHED)
        self.create_subscription(RunState, "/amr/run_state", lambda m: self._on_ask("run", m), LATCHED)
        self.create_subscription(WheelVelocities, "/cmd_wheel_vel", self._on_cmd, RELIABLE_1)
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, RELIABLE_1)
        self.create_service(Trigger, "/drives/arm", lambda q, r: self._request("arm", r))
        self.create_service(Trigger, "/drives/disarm", lambda q, r: self._request("disarm", r))
        self.create_service(Trigger, "/drives/ack_fault", lambda q, r: self._request("ack", r))

        self._thread = threading.Thread(target=self._run, name="can", daemon=True)
        self._thread.start()
        # A dead bus thread must take the process down, not leave a node that
        # answers services and publishes nothing.
        self.create_timer(0.5, self._check_bus_thread)

    def _check_bus_thread(self) -> None:
        if not self._thread.is_alive() and not self._stop.is_set():
            self.get_logger().fatal(f"bus thread ended: {self._status_snapshot}")
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
            # R03: a nonfinite command REPLACES the last one with nothing (zero at
            # the next tick); keeping the previous nonzero target would let a bad
            # sample extend motion until the watchdog.
            self._cmd = (time.monotonic(), wl, wr) if ok else None
            self._cmd_gen = int(msg.generation)
            if not ok:
                self._bad_cmds += 1
        if not ok:
            self.get_logger().warn(
                f"/cmd_wheel_vel not finite ({wl}, {wr}): setpoint zero", throttle_duration_sec=1.0
            )

    def _on_panel(self, msg: PanelState) -> None:
        t = time.monotonic()
        with self._lock:
            self._panel = (t, bool(msg.valid), bool(msg.mode_auto))
            self._power_panel = arm_policy.Panel(
                t, bool(msg.valid), bool(msg.mode_auto), bool(msg.manual_arm)
            )

    def _on_ask(self, source: str, msg) -> None:
        """A layer's drive_power, timed by its own stamp: a latched message from a layer that
        has since died arrives fresh but is not (arm_policy.Ask)."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp if stamp > 0 else BIG
        with self._lock:
            self._asks[source] = arm_policy.Ask(source, time.monotonic() - age, bool(msg.drive_power))

    def _on_pp(self, msg: PpMove) -> None:
        """Every hold replaces the last. A malformed one counts as no hold at all."""
        try:
            spec = pp.MoveSpec(
                pp.WheelMove(
                    int(msg.left_delta_counts),
                    int(msg.left_velocity_rpm),
                    int(msg.left_accel_rpm_s),
                    int(msg.left_decel_rpm_s),
                ),
                pp.WheelMove(
                    int(msg.right_delta_counts),
                    int(msg.right_velocity_rpm),
                    int(msg.right_accel_rpm_s),
                    int(msg.right_decel_rpm_s),
                ),
                float(msg.duration_s),
            )
            req = pp.Request(str(msg.run_id), int(msg.generation), bool(msg.hold), spec, time.monotonic())
            bad = ""
        except (TypeError, ValueError) as e:
            req, bad = None, f"malformed PpMove: {e}"
        with self._lock:
            self._pp_req, self._pp_bad = req, bad

    def _request(self, what: str, res):
        done = threading.Event()
        box: dict = {}
        self._requests.put((what, done, box))
        if not done.wait(15.0):
            res.success, res.message = False, f"{what}: bus thread did not answer"
            return res
        res.success, res.message = box.get("ok", False), box.get("msg", "")
        return res

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=8.0)

    # ---- bus thread ----

    def _log(self, s: str) -> None:
        self.get_logger().info(s)

    def _run(self) -> None:
        try:
            self._can_lock = ownerlock.acquire("can", "drive_node")  # before any bus I/O (plan §9.2)
        except ownerlock.OwnerBusy as e:
            self.get_logger().error(f"refusing can0: {e}")
            self._status_snapshot = {"state": "no bus", "reason": str(e), "mode": "off"}
            return
        try:
            raw, how = open_bus(config.CAN_BITRATE, config.CAN_CHANNEL, config.CAN_ADAPTER_SERIAL)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"CAN bus unavailable: {e} (another CAN owner running?)")
            self._status_snapshot = {"state": "no bus", "reason": str(e), "mode": "off"}
            return
        router = canopen.Router(raw)
        link = canopen.DriveLink(router, self.nodes, self.ramp, log=self._log)
        if self.pc_loss_ms:  # heartbeat kept alive inside blocking arm/disarm sequences too
            link.pc_node, link.pc_heartbeat_s = self.pc_node, self.pc_hb_s
        self._log(f"bus up on {how} at {config.CAN_BITRATE // 1000} kbps, profile {config.PROFILE_NAME}")
        link.enable_heartbeat(config.CAN_HEARTBEAT_MS)
        imu = None
        if self.imu_enabled:
            imu = MlsImu(
                link,
                config.SENSOR_NODE,
                self._publish_imu,
                sign=self.get_parameter("gyro_sign").value,
                poll_hz=self.get_parameter("imu_poll_hz").value,
                stamp_every=self.get_parameter("imu_stamp_every").value,
                log=self._log,
            )
            imu.start()
            # polled between the SDOs of blocking arm/engage/release sequences too
            link.idle_hook = lambda: imu.poll(time.monotonic())
        track = MlsTrack(
            link,
            config.SENSOR_NODE,
            self._publish_track,
            mode=str(self.get_parameter("mls_track_mode").value),
            sdo_hz=float(self.get_parameter("mls_track_sdo_hz").value),
            expected_variant=int(self.get_parameter("mls_track_variant").value),
            markers=marker_expectation(config.MLS_POLARITY_LOCK, config.MLS_TEACH_LOCK),
            on_event=self._on_mls_event,
            event_clear=config.MLS_EVENT_CLEAR,
            log=self._log,
        )
        try:
            track.start()
        except Exception as e:  # noqa: BLE001 - a sensor problem must not keep the drives down
            self.get_logger().error(f"MLS track reading unavailable: {e!r}")
            track.mode = "off"
        self._track = track
        if not self.pc_loss_ms:
            self.get_logger().warn("pc_loss_ms=0: the drives have NO independent response to losing this PC")

        try:
            self._loop(link, router, imu)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"bus thread error: {e!r}")
            self._status_snapshot = {"state": "bus error", "reason": repr(e), "mode": "off"}
        finally:
            try:
                link.disarm()
            except Exception:  # noqa: BLE001
                pass
            try:
                router.shutdown()
            except Exception:  # noqa: BLE001
                pass
            self._log("bus closed, drives de-energised")

    def _loop(self, link, router, imu) -> None:
        retry_at = 0.0
        t_tick = t_hb = t_status = t_mon = t_diag = 0.0
        t_wheels = None
        last_pub = {n: (None, None) for n in self.nodes}
        positions = {n: canopen.WheelPosition() for n in self.nodes}
        max_age = 2.5 * self.feedback_ms / 1000.0
        while not self._stop.is_set() and rclpy.ok():
            t0 = time.perf_counter()
            now = time.monotonic()
            self._serve_requests(link)

            # Link/power policy: the pure tables decide, this thread acts. Silent = no
            # frames at all, OR a required TPDO stopped while heartbeats and SDO
            # replies still arrive (R07) - in STANDBY too, where the counts still flow.
            silent = []
            if link.state in (canopen.ARMED, canopen.STANDBY):
                silent = sorted(
                    set(link.silent_nodes(now, self.driver_timeout))
                    | set(link.missing_feedback(now, self.driver_timeout))
                )
            self._update_power(now)
            d = canopen.decide(
                link.state,
                self.want_link,
                self._power[0],
                now,
                retry_at,
                silent,
                link.dropped_out(),
                link.faulted_nodes(),
                link.cleanup_owed,
            )
            if d.action == "prepare":
                retry_at = now + self.arm_retry
                try:
                    link.prepare(
                        self.feedback_ms,
                        self.pc_node if self.pc_loss_ms else None,
                        self.pc_loss_ms,
                        config.GEAR_RATIO,
                        config.INVERT_LEFT,
                        config.INVERT_RIGHT,
                    )
                    self._log("drives in standby (linked, power stage off)")
                    self._edge("arm", None)
                except Exception as e:  # noqa: BLE001 - a condition, retried
                    self.get_logger().warn(
                        f"cannot link the drives: {e} - retrying in {self.arm_retry:.0f} s"
                    )
                    self._edge("arm", ("DRIVE_FAULT", 2, f"{e}"))
            elif d.action == "engage":
                retry_at = now + self.arm_retry
                try:
                    link.engage()
                    self._log(f"drives powered (targets zero): {self._power[1]}")
                    self._edge("arm", None)
                except Exception as e:  # noqa: BLE001 - a condition, retried
                    self.get_logger().warn(
                        f"cannot power the drives: {e} - retrying in {self.arm_retry:.0f} s"
                    )
                    # "Switch on disabled" on every node is the safety chain holding STO,
                    # not a broken drive: the operator's answer is the cabinet's Reset
                    # button (2026-09-22). Edge-triggered - engaging retries every 2 s.
                    self._edge("arm", ("SAFETY_RESET_NEEDED", 1, f"{e}")
                               if "Switch on disabled" in str(e) else ("DRIVE_FAULT", 2, f"{e}"))
            elif d.action == "release":
                if link.release():
                    self._log(f"drives in standby (power off): {d.reason}; {self._power[1]}")
                else:
                    self.get_logger().error(f"FAULT: {link.fault_reason}")
                    self._edge("fault", ("DRIVE_FAULT", 2, link.fault_reason or "power-off incomplete"))
                retry_at = now + self.arm_retry
            elif d.action == "disarm":
                self.get_logger().warn(f"disarming: {d.reason}")
                if not link.disarm():
                    self.get_logger().error(f"disarm incomplete: {'; '.join(link.cleanup_failures)}")
                retry_at = now + self.arm_retry
            elif d.action == "cleanup":
                # Q02: an owed teardown is retried on the arm backoff; arming waits for it.
                if link.disarm(force=True):
                    self._log("owed drive cleanup completed")
                else:
                    self.get_logger().error(f"drive cleanup still owed: {'; '.join(link.cleanup_failures)}")
                retry_at = now + self.arm_retry
            elif d.action == "fault":
                self.get_logger().error(f"FAULT: {d.reason}")
                link.fault(d.reason)
                self._edge("fault", (_fault_code(d.reason), 2, d.reason))
            if link.state == canopen.FAULT:
                link.fault_tick(now, max_age)
                # R04 fallback: a stop we could not deliver or confirm must not be
                # covered by a heartbeat that says this controller is healthy.
                # Withholding it lets the drives' own 1016h reaction (8130h,
                # quick stop; needs a power cycle) take over.
                if link.stop_unconfirmed and link.pc_guard_set and not link.heartbeat_withheld:
                    link.heartbeat_withheld = True
                    self.get_logger().error(
                        f"fault stop unconfirmed ({'; '.join(link.stop_unconfirmed)}): "
                        "withholding the PC heartbeat so the drives trip 1016h"
                    )
                    self._edge("stop", ("DRIVE_STOP_UNCONFIRMED", 2, "; ".join(link.stop_unconfirmed)))

            # Setpoint at rate_hz; the command watchdog is independent of the mux.
            if self._pp.active and link.state != canopen.ARMED:
                # The drives were lost under a pp move (fault/disarm). Their stop frames
                # already carry Halt (DriveLink.in_pp); record it and stand down.
                self._pp.abandon(f"drives {link.state} {link.fault_reason or ''}".strip(), now)
                self.event(2, "PP_ABANDONED", self._pp.last.reason)
            if now - t_tick >= self.period and link.state == canopen.ARMED:
                t_tick = now
                if not self._pp_step(link, time.monotonic()):
                    link.send_target(*self._target(time.monotonic(), link.scale))

            if self.pc_loss_ms and now - t_hb >= self.pc_hb_s and not link.heartbeat_withheld:
                t_hb = now
                link.send_pc_heartbeat(self.pc_node)

            link.idle()  # the IMU poll (link.idle_hook), guarded against re-entry
            if self._track is not None:
                self._track.poll(now)

            # Feedback: a WheelStates per complete new pair - a new TPDO1 AND a new
            # TPDO2 from both drives, not just one member changing (R07). Once
            # started, a missing pair still publishes every max_age, marked invalid,
            # so consumers see the loss rather than silence.
            is_new = {
                n: link.telemetry[n].t_status not in (None, last_pub[n][0])
                and link.telemetry[n].t_position not in (None, last_pub[n][1])
                for n in self.nodes
            }
            if all(is_new.values()) or (t_wheels is not None and now - t_wheels >= max_age):
                t_wheels = now
                for n in self.nodes:
                    last_pub[n] = (link.telemetry[n].t_status, link.telemetry[n].t_position)
                self._publish_wheels(link, now, max_age, is_new, positions)

            if now - t_status >= 0.1:
                t_status = now
                self._publish_status(link, imu)
                self._publish_pp(link, now)

            if self._mon is not None and link.state != canopen.DISARMED:
                moving = link.applied is not None and link.applied != (0, 0)
                if now - t_mon >= self._mon_periods[1 if moving else 0]:
                    t_mon = now
                    self._monitor_slot(link)
            if now - t_diag >= 1.0:
                t_diag = now
                self._publish_diagnostics(link, imu)

            spent = time.perf_counter() - t0
            router.pump(max(0.001, self.period - spent))

    def _serve_requests(self, link) -> None:
        while True:
            try:
                what, done, box = self._requests.get_nowait()
            except queue.Empty:
                return
            try:
                if what == "arm":
                    self.want_link = True
                    box["ok"] = True
                    box["msg"] = f"link requested; power follows the panel ({self._power[1]})"
                elif what == "disarm":
                    self.want_link = False
                    box["ok"], box["msg"] = self._disarm_result(link, "disarmed")
                elif what == "ack":
                    if link.state != canopen.FAULT:
                        box["ok"], box["msg"] = False, f"no fault latched (state {link.state})"
                    else:
                        reason = link.fault_reason
                        box["ok"], box["msg"] = self._disarm_result(
                            link, f"fault acknowledged ({reason}); relinking if wanted"
                        )
            finally:
                done.set()

    @staticmethod
    def _disarm_result(link, ok_msg: str) -> tuple[bool, str]:
        """Truthful service result (review Q02): a teardown that did not finish is not
        "disarmed"; the loop keeps retrying it and will not arm until it is done."""
        if link.disarm():
            return True, ok_msg
        return False, "disarm incomplete, cleanup owed and retrying: " + "; ".join(link.cleanup_failures)

    def _update_power(self, now: float) -> None:
        with self._lock:
            panel, asks = self._power_panel, tuple(self._asks.values())
        power = arm_policy.want_power(now, panel, asks, self._power_params)
        if power != self._power:
            self._log(f"drive power {'wanted' if power[0] else 'not wanted'}: {power[1]}")
            self._power = power

    def _target(self, now: float, scale: canopen.WheelScale) -> tuple[int, int]:
        # `now` is sampled by the caller immediately before this call (review Q03): the
        # loop's tick timestamp predates request servicing and blocking SDO work, and a
        # command that expired during that work must not be transmitted.
        with self._lock:
            cmd, gen, lease = self._cmd, self._cmd_gen, self._lease
        reason = gating.drive_gate(now, lease, gen, self._gate)
        if reason != self._gate_reason:
            self._gate_reason = reason
            if reason is not None:
                self.get_logger().warn(f"setpoint gated to zero: {reason}")
        if reason is not None:
            return 0, 0
        return canopen.target_rpm(cmd, now, self.cmd_timeout, scale, config.MOTOR_MAX_RPM)

    # ---- profile position ----

    def _pp_gate(self, link, now: float, generation: int) -> str | None:
        with self._lock:
            lease, panel = self._lease, self._panel
        why = pp.authority_gate(
            now,
            enabled=bool(config.PP_ENABLED),
            lease=None if lease is None else (lease.t_recv, lease.generation, lease.allowed),
            move_generation=generation,
            panel=panel,
            lease_timeout_s=self._gate.lease_timeout_s,
            panel_timeout_s=self._pp_panel_timeout,
        )
        if why:
            return why
        if link.faulted_nodes():
            return f"drive fault on node(s) {link.faulted_nodes()}"
        return None

    def _pp_max_counts(self, link) -> int:
        """blind_run.max_distance_m of wheel travel, in counts (0 = scale unknown: refuse)."""
        cpr = link.scale.counts_per_wheel_rev if link.scale else None
        if not cpr:
            return 0
        return int(cpr * config.BLIND_MAX_DISTANCE_M / (math.pi * config.WHEEL_DIA_M))

    def _pp_step(self, link, now: float) -> bool:
        """One pp tick. -> True when pp owns this tick's RPDO (no velocity setpoint)."""
        with self._lock:
            req = self._pp_req
        if req is None and not self._pp.active:
            return False
        tl, tr = link.telemetry[config.LEFT], link.telemetry[config.RIGHT]
        gen = req.generation if req is not None else -1
        out = self._pp.tick(
            pp.Tick(
                now,
                req,
                self._pp_gate(link, now, gen),
                pp.Feedback(tl.statusword, tl.rpm, tl.t_status),
                pp.Feedback(tr.statusword, tr.rpm, tr.t_status),
            )
        )
        if out.action == "enter":
            why = pp.validate_move(
                out.spec,
                max_rpm=config.PP_MAX_SPEED_MPS * config.RPM_PER_MPS,
                max_counts=self._pp_max_counts(link),
                max_accel_rpm_s=float(config.BLIND_ACCEL_RPM_S),
            )
            targets = None
            if why is None:
                try:
                    targets = link.pp_enter(
                        out.spec, config.PP_EXPECT, config.PP_EXPECT_OBJECTS, 2.5 * self.feedback_ms / 1000.0
                    )
                except (RuntimeError, ValueError) as e:
                    why = str(e)
            self._pp.entered(why is None, why or "", targets, time.monotonic())
            if why:
                self.get_logger().warn(f"pp move {self._pp.run_id} refused: {why}")
                self.event(1, "PP_REFUSED", why)
                return False
            # Audit R02: re-read authority NOW - setup blocked for several SDO transactions.
            with self._lock:
                latest = self._pp_req
            t = time.monotonic()
            veto = self._pp.veto_start(latest, self._pp_gate(link, t, self._pp.generation), t)
            if veto is not None:
                self.get_logger().warn(f"pp move {self._pp.run_id} not started: {veto.reason}")
                self.event(1, "PP_VETOED", f"{self._pp.run_id}: {veto.reason}")
                link.send_controlwords(*veto.controlwords)
                return True
            self._log(f"pp move {self._pp.run_id} started: targets {targets}")
            self.event(0, "PP_START", f"{self._pp.run_id} targets {targets}")
            link.send_controlwords(pp.CW_START, pp.CW_START)
            return True
        if out.action == "cw":
            link.send_controlwords(*out.controlwords)
            return True
        if out.action == "exit":
            try:
                link.pp_exit()
                self._pp.exited(True, "", time.monotonic())
            except RuntimeError as e:
                self._pp.exited(False, str(e), time.monotonic())
                link.fault(f"pp: return to pv failed ({e})")
            r = self._pp.last
            level = 0 if r.outcome == pp.DONE else 1 if r.outcome == pp.HALTED else 2
            self._log(f"pp move {r.run_id} {r.outcome}: {r.reason}")
            self.event(level, f"PP_{r.outcome.upper()}", f"{r.run_id}: {r.reason}")
            return True
        if out.action == "fault":
            link.fault(out.reason)
            # A halt nobody can confirm at rest: let the drives' own 1016h take over.
            if link.pc_guard_set:
                link.heartbeat_withheld = True
            self.get_logger().error(out.reason)
            self.event(2, "PP_FAULTED", out.reason)
            return True
        return self._pp.active

    def _pp_availability(self, link) -> tuple[bool, str]:
        if not config.PP_ENABLED:
            return False, "pp locked in the profile (pp.enabled false)"
        unset = sorted(k for k, v in config.PP_EXPECT.items() if v is None)
        if unset:
            return False, f"pp.expect not configured: {', '.join(unset)}"
        if link.state != canopen.ARMED:
            return False, f"drives {link.state}"
        if self._pp.active:
            return False, f"a pp move is {self._pp.state}"
        return True, ""

    def _publish_pp(self, link, now: float) -> None:
        m = PpStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.available, why = self._pp_availability(link)
        last = self._pp.last
        m.reason = why or (last.reason if last else "") or self._pp_bad
        m.state, m.run_id = self._pp.state, self._pp.run_id
        m.outcome = last.outcome if last and last.run_id == self._pp.run_id and not self._pp.active else ""
        if self._pp.targets:
            m.left_target, m.right_target = (int(t) for t in self._pp.targets)
        tl, tr = link.telemetry[config.LEFT], link.telemetry[config.RIGHT]
        m.left_actual, m.right_actual = int(tl.position or 0), int(tr.position or 0)
        m.left_target_reached = bool((tl.statusword or 0) & pp.SW_TARGET_REACHED)
        m.right_target_reached = bool((tr.statusword or 0) & pp.SW_TARGET_REACHED)
        m.left_following_error = bool((tl.statusword or 0) & pp.SW_FOLLOWING_ERROR)
        m.right_following_error = bool((tr.statusword or 0) & pp.SW_FOLLOWING_ERROR)
        m.started_age_s = (now - self._pp.t_start) if (self._pp.active and self._pp.t_start) else -1.0
        self._safe_publish(self._pub_pp, m)

    def _safe_publish(self, pub, msg) -> None:
        # SIGINT invalidates the context while this thread is mid-iteration;
        # a publish then raises RCLError. Stop quietly rather than log an error.
        if self._stop.is_set() or not rclpy.ok():
            return
        try:
            pub.publish(msg)
        except Exception:  # noqa: BLE001
            self._stop.set()

    def _publish_wheels(self, link, now: float, max_age: float, is_new: dict, positions: dict) -> None:
        m = WheelStates()
        m.header.stamp = self.get_clock().now().to_msg()
        scale = link.scale
        # Continuous positions from wrap-safe count deltas (R06), see canopen.wheel_feedback.
        fb = canopen.wheel_feedback(
            link, now, max_age, ((config.LEFT, True), (config.RIGHT, False)), is_new, positions
        )
        m.left_pos_rad, m.left_vel_rad_s, m.left_valid = fb[config.LEFT]
        m.right_pos_rad, m.right_vel_rad_s, m.right_valid = fb[config.RIGHT]
        tl, tr = link.telemetry[config.LEFT], link.telemetry[config.RIGHT]
        if tl.position is not None and tr.position is not None:
            m.left_counts, m.right_counts = int(tl.position), int(tr.position)
            m.counts_valid = bool(m.left_valid and m.right_valid)
        m.counts_per_wheel_rev = float(scale.counts_per_wheel_rev or 0.0) if scale else 0.0
        self._safe_publish(self._pub_wheels, m)
        if now - self._wheels_ui_t >= WHEELS_UI_PERIOD_S:
            self._wheels_ui_t = now
            self._safe_publish(self._pub_wheels_ui, m)

    def _publish_status(self, link, imu) -> None:
        m = DriveStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        tl, tr = link.telemetry[config.LEFT], link.telemetry[config.RIGHT]
        m.left_statusword, m.right_statusword = tl.statusword or 0, tr.statusword or 0
        m.left_error_code = (tl.alarm or {}).get("code", tl.error_register or 0) & 0xFFFF
        m.right_error_code = (tr.alarm or {}).get("code", tr.error_register or 0) & 0xFFFF
        m.left_state, m.right_state = tl.state, tr.state
        # Cached statuswords are not evidence: operational also needs fresh TPDO1
        # and TPDO2 from both drives (R07).
        now, max_age = time.monotonic(), 2.5 * self.feedback_ms / 1000.0
        fresh = all(all(link.feedback_fresh(n, now, max_age)) for n in (config.LEFT, config.RIGHT))
        m.operational = bool(
            link.state == canopen.ARMED and fresh and tl.operation_enabled and tr.operation_enabled
        )
        m.link_state = link.state
        m.power_wanted, m.power_reason = bool(self._power[0]), self._power[1]
        self._safe_publish(self._pub_status, m)
        # Latched CANopen alarms and the operational edge, as operator events. Both are
        # edge-guarded by _edge, so this 10 Hz path emits only on a change.
        alarmed = [
            f"node {n} {(t.alarm or {}).get('hex', '')} {(t.alarm or {}).get('name', '')}".strip()
            for n, t in sorted(link.telemetry.items())
            if t.alarm
        ]
        self._edge("alarm", ("DRIVE_ALARM", 2, "; ".join(alarmed)) if alarmed else None)
        if m.operational != self._operational:
            self._operational = bool(m.operational)
            self.event(0, "MODE_CHANGE", "drives are powered" if m.operational else "drives are not powered")
        snap = {"state": link.state, "reason": link.fault_reason or "", "mode": imu.mode if imu else "off"}
        if snap != self._status_snapshot:
            self._status_snapshot = snap
            why = f" ({snap['reason']})" if snap["reason"] else ""
            self._log(f"drives {snap['state']}{why}, imu {snap['mode']}")

    def _monitor_slot(self, link) -> None:
        """One object, one node, one SDO round trip (~2-4 ms); see canopen.MonitorCursor."""
        self._cursor.step(lambda nid, idx: link.read(nid, idx, 0, timeout=0.05), canmon._decode)
        self._bus_stats["monitor_reads"] = self._cursor.reads
        self._bus_stats["sdo_timeouts"] = self._cursor.timeouts

    def _edge(self, slot: str, what: tuple[str, int, str] | None) -> None:
        """Emit an operator event only when `slot` CHANGES (agv_core/events: edges,
        never per tick). `what` is (code, level, detail); None means the slot cleared,
        which emits a cleared event if something was standing."""
        prev = self._edges.get(slot)
        if what is None:
            if prev is not None:
                self._edges.pop(slot, None)
                self.event(0, prev[0], f"cleared: {alarms.get(prev[0]).title.lower()}")
            return
        if prev is not None and prev[0] == what[0] and prev[2] == what[2]:
            return
        self._edges[slot] = what
        self.event(what[1], what[0], f"{alarms.get(what[0]).title}: {what[2]}")

    def event(self, level: int, code: str, text: str) -> None:
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source, m.level, m.code, m.text = "drive_node", level, code, text
        self._event_seq += 1
        m.seq = self._event_seq
        self._safe_publish(self._pub_event, m)

    def _publish_diagnostics(self, link, imu) -> None:
        """Owner-published snapshot (unified plan §7.1): drive states, alarms, the
        monitored analogue values with warn/trip applied from the profile, bus
        counters and IMU acquisition. GET requests on the web read this; they
        never cause bus traffic."""
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        mon = self._mon.snapshot(config.MONITOR_THRESHOLDS)["nodes"] if self._mon is not None else {}
        for nid, label in config.NODES.items():
            t = link.telemetry[nid]
            st = DiagnosticStatus()
            st.name = f"drive/{label}"
            st.hardware_id = f"node {nid}"
            worst = DiagnosticStatus.OK
            kv = [
                KeyValue(key="state", value=t.state),
                KeyValue(key="statusword", value=f"0x{(t.statusword or 0):04X}"),
                KeyValue(key="error_register", value=f"0x{(t.error_register or 0):02X}"),
                KeyValue(key="rpm", value=str(t.rpm if t.rpm is not None else "")),
                KeyValue(key="position_counts", value=str(t.position if t.position is not None else "")),
                KeyValue(key="nmt", value=t.nmt or ""),
                KeyValue(
                    key="tpdo1_age_s",
                    value="" if t.t_status is None else f"{time.monotonic() - t.t_status:.3f}",
                ),
                KeyValue(
                    key="tpdo2_age_s",
                    value="" if t.t_position is None else f"{time.monotonic() - t.t_position:.3f}",
                ),
            ]
            if t.alarm:
                kv.append(
                    KeyValue(key="alarm", value=f"0x{t.alarm.get('code', 0):04X} {t.alarm.get('text', '')}")
                )
                worst = DiagnosticStatus.ERROR
            for key, v in (mon.get(str(nid)) or {}).items():
                val = "" if v.get("value") is None else f"{v['value']}"
                kv.append(KeyValue(key=f"{key} [{v.get('unit', '')}]", value=val))
                lvl = v.get("state")
                if lvl == "trip":
                    worst = DiagnosticStatus.ERROR
                elif lvl == "warn" and worst == DiagnosticStatus.OK:
                    worst = DiagnosticStatus.WARN
            st.level = worst
            st.message = t.state if not t.alarm else "alarm"
            st.values = kv
            arr.status.append(st)
        bus = DiagnosticStatus(name="can/bus", hardware_id=config.CAN_CHANNEL, level=DiagnosticStatus.OK)
        bus.message = f"{link.state}"
        bus.values = [
            KeyValue(key="link_state", value=link.state),
            KeyValue(key="power_wanted", value=str(self._power[0])),
            KeyValue(key="power_reason", value=self._power[1]),
            KeyValue(key="fault_reason", value=link.fault_reason or ""),
            KeyValue(key="monitor_reads", value=str(self._bus_stats["monitor_reads"])),
            KeyValue(key="sdo_timeouts", value=str(self._bus_stats["sdo_timeouts"])),
            KeyValue(key="require_supervisor", value=str(self._gate.require_supervisor)),
            KeyValue(key="feedback_hz", value=f"{1000.0 / self.feedback_ms:.0f}"),
            KeyValue(key="pc_loss_ms", value=str(self.pc_loss_ms)),
            KeyValue(key="setpoint_gate", value=self._gate_reason or "open"),
            KeyValue(key="nonfinite_cmds", value=str(self._bad_cmds)),
            KeyValue(key="stop_unconfirmed", value="; ".join(link.stop_unconfirmed)),
            KeyValue(key="heartbeat_withheld", value=str(link.heartbeat_withheld)),
            KeyValue(key="cleanup_failures", value="; ".join(link.cleanup_failures)),
            KeyValue(key="pp_state", value=self._pp.state),
            KeyValue(key="pp_in_mode", value=str(link.in_pp)),
            KeyValue(
                key="pp_last",
                value=f"{self._pp.last.run_id} {self._pp.last.outcome}: {self._pp.last.reason}"
                if self._pp.last
                else "",
            ),
        ]
        arr.status.append(bus)
        if imu is not None:
            im = DiagnosticStatus(name="imu/mls", hardware_id="node 10", level=DiagnosticStatus.OK)
            im.message = imu.mode
            im.values = [
                KeyValue(key="mode", value=imu.mode),
                KeyValue(key="samples", value=str(imu.samples)),
                KeyValue(key="misses", value=str(imu.misses)),
                KeyValue(key="rejected", value=str(imu.rejected)),
                KeyValue(key="last_reject", value=imu.last_reject),
                KeyValue(key="restarts", value=str(imu.restarts)),
                KeyValue(key="gyro_sign", value=str(self.get_parameter("gyro_sign").value)),
            ]
            arr.status.append(im)
        if self._track is None:
            self._safe_publish(self._pub_diag, arr)
            return
        temp = self._track.temp
        if temp is not None and temp["temp_c"] is not None:
            detail = f"{temp['temp_c']} degC, the sensor's range {temp['lo_c']}..{temp['hi_c']} degC"
            self._edge("temperature", ("MLS_TEMP", 1, detail) if temp["out"] else None)
        level, message, kv = self._track.diagnostic(time.monotonic())
        tr = DiagnosticStatus(name="mls/track", hardware_id=f"node {config.SENSOR_NODE}")
        levels = {ERROR: DiagnosticStatus.ERROR, WARN: DiagnosticStatus.WARN}
        tr.level = levels.get(level, DiagnosticStatus.OK)
        tr.message = message
        kv += [
            ("marker_events", str(self._markers.seq)),
            ("marker_generation", str(self._markers.generation)),
            ("marker_rejects", str(self._markers.rejects)),
            ("marker_aborts", str(self._markers.aborts)),
        ]
        tr.values = [KeyValue(key=k, value=v) for k, v in kv]
        arr.status.append(tr)
        self._safe_publish(self._pub_diag, arr)

    def _on_mls_event(self, ev: dict) -> None:
        """Bus thread: a magnetic-field event under a track (mls_track, E5 gate passed).
        History only (plan E1): one INFO event, never a standing alarm, never motion."""
        held = self._field_events.allow(ev["t_mono"])
        if held is None:
            return
        where = "LCP2 --" if ev["lcp2_mm"] is None else f"LCP2 {ev['lcp2_mm']:+d} mm"
        more = f"; +{held} more since the last report" if held else ""
        text = f"{ev['field_text']} ({where}, track level {ev['track_level']}/7{more})"
        self.event(0, "MLS_FIELD_WEAK", text)

    def _publish_track(self, s: TrackSample) -> None:
        r = s.reading
        m = LineTrack()
        m.stamp = self.get_clock().now().to_msg()
        # stamp is publish time; t_mono is when the frame was DECODED. In SDO
        # mode those differ by up to a poll period, and a consumer looking only
        # at stamp cannot tell. The follower gates on this, so it is computed
        # here rather than inferred downstream.
        m.sample_age_s = max(0.0, time.monotonic() - s.t_mono)
        self._track_seq = (self._track_seq + 1) & 0xFFFFFFFF
        m.seq = self._track_seq
        m.lcp_mm = [max(-32768, min(32767, int(p))) for p in s.lcp_mm]
        m.valid = [(i + 1) in r["valid"] for i in range(3)]
        m.nlcp = int(r["nlcp"])
        m.nlcp_label = r["nlcp_label"]
        st = r["status"]
        m.line_good, m.track_level, m.polarity = st["line_good"], int(st["track_level"]), st["polarity"]
        m.event_flag = st["event_flag"]
        m.marker, m.marker_intro = int(r["marker"]["code"]), r["marker"]["intro"]
        m.source = s.source
        self._safe_publish(self._pub_track, m)
        self._marker_sample(s)
        now = time.monotonic()
        if now - self._track_ui_t >= TRACK_UI_PERIOD_S:
            self._track_ui_t = now
            self._safe_publish(self._pub_track_ui, m)

    def _marker_sample(self, s: TrackSample) -> None:
        """Bus thread, every decoded MLS sample: the detector, and one message per marker."""
        track = getattr(self, "_track", None)
        if track is None:
            return  # still inside start(): its snapshot sample predates any check
        if track.start_count != self._marker_gen_seen:
            self._marker_gen_seen = track.start_count
            self._markers.new_generation()
        ev = self._markers.feed(s.t_mono, s.reading, s.source, track.markers_ok)
        if ev is None:
            return
        m = self._marker_msg(track, heartbeat=False)
        m.seq, m.code, m.raw, m.direction = ev.seq, ev.code, ev.raw, ev.direction
        m.lcp2_mm = max(-32768, min(32767, ev.lcp2_mm))
        m.nlcp, m.line_good = ev.nlcp, ev.line_good
        self._safe_publish(self._pub_marker, m)

    def _marker_msg(self, track, heartbeat: bool) -> MarkerEvent:
        m = MarkerEvent()
        m.stamp = self.get_clock().now().to_msg()
        m.heartbeat = heartbeat
        m.generation = self._markers.generation
        m.seq = self._markers.seq
        m.markers_ok = bool(track is not None and track.markers_ok)
        m.status = track.marker_status_text() if track is not None else "no sensor"
        m.rejects, m.aborts = self._markers.rejects, self._markers.aborts
        return m

    def _marker_heartbeat(self) -> None:
        """1 Hz: the marker stream's status, so "no marker passed" differs from "no stream".
        Executor thread; reads counters the bus thread writes (ints, a stale read is harmless)."""
        self._safe_publish(self._pub_marker, self._marker_msg(getattr(self, "_track", None), heartbeat=True))

    def _publish_imu(self, s: ImuSample) -> None:
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.imu_frame
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.z = s.wz_rad_s
        m.angular_velocity_covariance = [BIG, 0.0, 0.0, 0.0, BIG, 0.0, 0.0, 0.0, self.gyro_var]
        m.linear_acceleration_covariance[0] = -1.0
        self._safe_publish(self._pub_imu, m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.stop()  # disarms on the bus thread before the context goes away
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
