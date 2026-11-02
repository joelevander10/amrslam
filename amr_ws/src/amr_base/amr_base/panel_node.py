"""panel_node (T12): the real operator panel and the horn, over the DIO island.

    DIO discrete inputs (Modbus TCP, drivers/dio.DioLink thread)
        -> /amr/panel_state (PanelState, 50 Hz): debounced Reset/Start edges,
           AUTO/MANUAL selector, valid=false while comms are down or before
           the first baseline; jog pendant direction levels
    /cmd_wheel_vel + /drives/status
        -> horn-and-lights coil (DO HORN_DO_CHANNEL), renewed every tick,
           expiring on its own (config.HORN_HOLD_S) if this node dies

The panel is the ONLY thing that can authorise motion on the ROS side (the mux
takes authority from /amr/panel_state, the executor needs a Start edge under
AUTO). This node reports what the operator did; it decides nothing about
motion. It has no services and no parameters that could fake an edge.

Never runs beside another DIO owner: two writers on the same coil (ownerlock).
"""

from __future__ import annotations

import time

import rclpy
from agv_core import (
    config,
    ownerlock,  # repo module: core/ownerlock.py
)
from agv_core.drivers import dio  # repo module: drivers/dio.py
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data

from amr_base import panel_io
from amr_interfaces.msg import (
    DriveStatus,
    Event,
    IoImage,
    LineState,
    MuxState,
    PanelState,
    RunState,
    WheelStates,
    WheelVelocities,
)

WHEELS_FRESH_S = 0.3
MUX_FRESH_S = 0.5  # /amr/mux_state is 10 Hz
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
TURNING_RAD_S = 0.01  # wheel speed above which a motor counts as turning (feedback noise floor)

RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)


class PanelNode(Node):
    def __init__(self, link=None) -> None:
        super().__init__("panel_node")
        self.declare_parameter("rate_hz", 50.0)  # PANEL_DEBOUNCE_SCANS was tuned at canworker's 50 Hz
        self.declare_parameter("cmd_timeout_s", 0.2)  # same as drive_node: a stale command is zero
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)

        self.link = link or dio.DioLink()
        self.adapter = panel_io.PanelAdapter()
        self._pub = self.create_publisher(PanelState, "/amr/panel_state", 10)
        # 10 Hz copy for displays (CPU, 2026-10-02): the web shows the selector at 2 Hz.
        # Edges ride only the full topic - nothing that acts on Start/Reset reads this one.
        self._pub_ui = self.create_publisher(PanelState, "/amr/panel_state_ui", 10)
        self._ui_t = 0.0
        self._pub_io = self.create_publisher(IoImage, "/amr/io", 5)
        self._pub_event = self.create_publisher(Event, "/amr/events", 50)
        self._event_seq = 0
        self._io_every = 10  # 50 Hz tick -> 5 Hz image (unified plan §7.1)
        self._io_n = 0
        self.create_subscription(WheelVelocities, "/cmd_wheel_vel", self._on_cmd, RELIABLE_1)
        self.create_subscription(DriveStatus, "/drives/status", self._on_drives, RELIABLE_1)
        # dio.motion_on outputs follow the measured wheels (drive_node feedback).
        self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, qos_profile_sensor_data)
        self._turning = False
        self._wheels_t: float | None = None
        # dio.alarm_on: the run states and the scanner fields as the mux sees them.
        self.create_subscription(MuxState, "/amr/mux_state", self._on_mux, RELIABLE_1)
        self.create_subscription(LineState, "/amr/line_state", self._on_line, LATCHED)
        self.create_subscription(RunState, "/amr/run_state", self._on_run, LATCHED)
        self._mux: MuxState | None = None
        self._mux_t: float | None = None
        self._line_state: int | None = None
        self._run_state: int | None = None
        self._run_premove = False
        self._alarm = False
        self._cmd = (0.0, 0.0)
        self._cmd_t: float | None = None
        self._armed = False
        self._armed_t: float | None = None  # a stale "armed" is not armed (unified plan §7.1)
        self._horn = None

        if not config.DIO_ENABLED:
            self.get_logger().warn("dio.enabled is false in the profile: no panel, no horn, valid=false")
        if not config.PANEL_ENABLED:
            self.get_logger().warn("panel.enabled is false in the profile: publishing valid=false forever")
        self._dio_lock = ownerlock.acquire("dio", "panel_node")  # before any Modbus I/O (plan §9.2)
        self.link.start()
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info(
            f"panel on DIO {config.DIO_IP}:{config.DIO_PORT}: reset DI{config.PANEL_DI_RESET}, "
            f"start DI{config.PANEL_DI_START}, selector DI{config.PANEL_DI_AUTO} "
            f"(ON = {'AUTO' if config.PANEL_AUTO_WHEN_ON else 'MANUAL'}), "
            f"manual arm DI{config.PANEL_DI_MANUAL_ARM}; "
            f"horn {f'DO{config.HORN_DO_CHANNEL}' if config.HORN_ENABLED else 'disabled'}; "
            + (
                f"pendant fwd DI{config.PENDANT_DI_FWD}, "
                f"rvs DI{config.PENDANT_DI_RVS}, left DI{config.PENDANT_DI_LEFT}, "
                f"right DI{config.PENDANT_DI_RIGHT}"
                if config.PENDANT_ENABLED
                else "pendant disabled"
            )
        )

    # -- inputs --

    def _on_cmd(self, m: WheelVelocities) -> None:
        self._cmd = (float(m.left_rad_s), float(m.right_rad_s))
        self._cmd_t = time.monotonic()

    def _on_mux(self, m: MuxState) -> None:
        self._mux, self._mux_t = m, time.monotonic()

    def _on_line(self, m: LineState) -> None:
        self._line_state = int(m.state)

    def _on_run(self, m: RunState) -> None:
        self._run_state = int(m.state)
        self._run_premove = bool(m.premove)

    def _on_wheels(self, m: WheelStates) -> None:
        self._turning = (m.left_valid and abs(m.left_vel_rad_s) > TURNING_RAD_S) or (
            m.right_valid and abs(m.right_vel_rad_s) > TURNING_RAD_S
        )
        self._wheels_t = time.monotonic()

    def _on_drives(self, m: DriveStatus) -> None:
        self._armed = bool(m.operational)
        self._armed_t = time.monotonic()

    # -- tick --

    def _tick(self) -> None:
        snap = self.link.snapshot()
        if not config.PANEL_ENABLED:
            snap = dict(snap, comms_ok=False)
        frame = self.adapter.tick(snap)
        if frame.changed:
            self.get_logger().info(frame.changed)
            lvl = Event.WARN if "LOST" in frame.changed or "invalid" in frame.changed else Event.INFO
            self._event(lvl, "PANEL", frame.changed)
        self._io_n = (self._io_n + 1) % self._io_every
        if self._io_n == 0:
            self._publish_io(snap)
        m = PanelState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.valid = frame.valid
        m.mode_auto = frame.mode_auto
        m.start_edge = frame.start_edge
        m.reset_edge = frame.reset_edge
        m.seq = frame.seq
        m.pendant_fwd, m.pendant_rvs, m.pendant_left, m.pendant_right = frame.pendant
        m.manual_arm = frame.manual_arm
        self._pub.publish(m)
        if time.monotonic() - self._ui_t >= 0.1:
            self._ui_t = time.monotonic()
            self._pub_ui.publish(m)

        # dio.motion_on (DO00, DO08): on while the motors turn, off when they do not, in any
        # mode. Feedback older than WHEELS_FRESH_S counts as not turning.
        fresh = self._wheels_t is not None and time.monotonic() - self._wheels_t <= WHEELS_FRESH_S

        # dio.alarm_on (DO01): AUTO run active and a scanner field occupied, or the
        # pre-move warning after a Start.
        mux = self._mux
        mux_fresh = mux is not None and time.monotonic() - self._mux_t <= MUX_FRESH_S
        alarm = panel_io.alarm_wanted(
            frame.valid and frame.mode_auto, self._line_state, self._run_state,
            mux_fresh and mux.field_fresh,
            bool(mux.protective_clear) if mux_fresh else True,
            bool(mux.warning_active) if mux_fresh else False,
            run_premove=self._run_premove,
        )
        for ch in config.DIO_ALARM_ON:
            self.link.set_coil(ch, alarm, config.HORN_HOLD_S)
        # the movement horn (dio.movement_horn, DO00) gives way to the alarm horn
        outs = panel_io.motion_outputs(self._turning and fresh, alarm, config.DIO_MOTION_ON, config.DIO_MOVEMENT_HORN)
        for ch, on in outs.items():
            self.link.set_coil(ch, on, config.HORN_HOLD_S)
        if alarm != self._alarm:
            self._alarm = alarm
            self.get_logger().info(f"alarm horn {'on' if alarm else 'off'}")

        if config.HORN_ENABLED:
            now = time.monotonic()
            age = None if self._cmd_t is None else now - self._cmd_t
            armed = self._armed and self._armed_t is not None and now - self._armed_t <= 0.3
            want = panel_io.horn_wanted(armed, *self._cmd, age, self.cmd_timeout)
            self.link.set_coil(config.HORN_DO_CHANNEL, want, config.HORN_HOLD_S)
            if want != self._horn:
                self._horn = want
                self.get_logger().debug(f"horn {'on' if want else 'off'}")

    def _event(self, level: int, code: str, text: str) -> None:
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source, m.level, m.code, m.text = "panel_node", level, code, text
        self._event_seq += 1
        m.seq = self._event_seq
        self._pub_event.publish(m)

    def _publish_io(self, snap: dict) -> None:
        """The owner's full I/O image (unified plan §7): requested vs readback kept apart."""
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
        self._pub_io.publish(m)

    def close(self) -> None:
        try:
            self.link.stop()  # quiesces the coil low before the socket closes
        except Exception:  # noqa: BLE001
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PanelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
