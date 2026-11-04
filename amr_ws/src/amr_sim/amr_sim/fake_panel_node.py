"""fake_panel_node: the operator panel in simulation (spec §8, T8).

Publishes /amr/panel_state at 20 Hz with the PanelScan semantics (debounced
image, rising edges on exactly one message). Services drive it from tests or a
terminal:
    /sim/panel/set_mode     std_srvs/SetBool   data=true -> AUTO, false -> MANUAL
    /sim/panel/press_start  std_srvs/Trigger
    /sim/panel/press_reset  std_srvs/Trigger
    /sim/panel/set_valid    std_srvs/SetBool   false = DIO link lost (stale/invalid image)
    /sim/panel/set_estop    std_srvs/SetBool   true = E-stop pressed (DI08 LOW), false = released
The emergency fields run the real amr_base/emergency.py: press, release, then press_reset
clears it (the sim drives are not observable, so a Reset with DI08 HIGH is taken at once).
Jog pendant levels are bool parameters read every tick, so a terminal can drive
without a browser:  ros2 param set /fake_panel pendant_fwd true  (also
pendant_rvs / pendant_left / pendant_right).
The real adapter (T12) wraps core/panel.PanelScan over drivers/dio.DioLink and
publishes the same message.
"""

import time

import rclpy
from agv_core import panel as panel_core  # repo module: core/panel.py
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from amr_base import emergency
from amr_interfaces.msg import PanelState

PENDANT_PARAMS = ("pendant_fwd", "pendant_rvs", "pendant_left", "pendant_right")


class FakePanel(Node):
    def __init__(self) -> None:
        super().__init__("fake_panel")
        self.declare_parameter("auto", False)
        self.declare_parameter("rate_hz", 20.0)
        for name in PENDANT_PARAMS:
            self.declare_parameter(name, False)
        self.auto = bool(self.get_parameter("auto").value)
        self.valid = True
        self.estop = False  # E-stop pressed: DI08 LOW
        self.emergency = emergency.Emergency(3.0)
        self.seq = 0
        self._start_pending = False
        self._reset_pending = False
        self._continue_pending = False
        self._dest_pending = 0
        self._pub = self.create_publisher(PanelState, "/amr/panel_state", 10)
        # 10 Hz copy for displays (CPU, 2026-10-02): the web shows the selector at 2 Hz.
        # Edges ride only the full topic - nothing that acts on Start/Reset reads this one.
        self._pub_ui = self.create_publisher(PanelState, "/amr/panel_state_ui", 10)
        self._ui_t = 0.0
        self.create_service(SetBool, "/sim/panel/set_mode", self._set_mode)
        self.create_service(SetBool, "/sim/panel/set_valid", self._set_valid)
        self.create_service(SetBool, "/sim/panel/set_estop", self._set_estop)
        self.create_service(Trigger, "/sim/panel/press_start", self._press_start)
        self.create_service(Trigger, "/sim/panel/press_reset", self._press_reset)
        self.create_service(Trigger, "/sim/panel/press_continue", self._press_continue)
        # destination buttons, 1-based like PanelState.dest_edge (/sim/panel/press_dest_1 = UBC)
        for k in range(1, 4):
            self.create_service(Trigger, f"/sim/panel/press_dest_{k}", self._press_dest(k))
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._tick)
        self.get_logger().info(f"panel: {'AUTO' if self.auto else 'MANUAL'}")

    def _set_mode(self, req, res):
        self.auto = bool(req.data)
        res.success, res.message = True, f"selector -> {'AUTO' if self.auto else 'MANUAL'}"
        self.get_logger().info(res.message)
        return res

    def _set_valid(self, req, res):
        self.valid = bool(req.data)
        res.success, res.message = True, f"panel image {'valid' if self.valid else 'INVALID'}"
        return res

    def _set_estop(self, req, res):
        self.estop = bool(req.data)
        res.success, res.message = True, f"E-stop {'PRESSED' if self.estop else 'released'}"
        self.get_logger().info(res.message)
        return res

    def _press_start(self, _req, res):
        self._start_pending = True
        res.success, res.message = True, "Start pressed"
        self.get_logger().info(res.message)
        return res

    def _press_continue(self, _req, res):
        self._continue_pending = True
        res.success, res.message = True, "Continue pressed"
        self.get_logger().info(res.message)
        return res

    def _press_dest(self, k: int):
        def press(_req, res):
            self._dest_pending = k
            res.success, res.message = True, f"destination {k} pressed"
            self.get_logger().info(res.message)
            return res
        return press

    def _press_reset(self, _req, res):
        self._reset_pending = True
        res.success, res.message = True, "Reset pressed"
        return res

    def _tick(self) -> None:
        if not self.valid:
            return  # a dead DIO link publishes nothing: consumers see staleness
        m = PanelState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.valid = True
        m.mode_auto = self.auto
        m.start_edge, self._start_pending = self._start_pending, False
        m.reset_edge, self._reset_pending = self._reset_pending, False
        m.continue_edge, self._continue_pending = self._continue_pending, False
        m.dest_edge, self._dest_pending = self._dest_pending, 0
        m.estop_ok = not self.estop
        note = self.emergency.step(time.monotonic(), emergency.Inputs(True, m.estop_ok, m.reset_edge, None))
        if note:
            self.get_logger().info(note)
        m.emergency = self.emergency.state
        self.seq += 1
        m.seq = self.seq
        levels = [bool(self.get_parameter(n).value) for n in PENDANT_PARAMS]
        m.pendant_fwd, m.pendant_rvs, m.pendant_left, m.pendant_right = panel_core.pendant_intent(*levels)
        self._pub.publish(m)
        if time.monotonic() - self._ui_t >= 0.1:
            self._ui_t = time.monotonic()
            self._pub_ui.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakePanel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
