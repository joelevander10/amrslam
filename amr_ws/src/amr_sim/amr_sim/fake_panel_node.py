"""fake_panel_node: the operator panel in simulation (spec §8, T8).

Publishes /amr/panel_state at 20 Hz with the PanelScan semantics (debounced
image, rising edges on exactly one message). Services drive it from tests or a
terminal:
    /sim/panel/set_mode     std_srvs/SetBool   data=true -> AUTO, false -> MANUAL
    /sim/panel/press_start  std_srvs/Trigger
    /sim/panel/press_reset  std_srvs/Trigger
    /sim/panel/set_valid    std_srvs/SetBool   false = DIO link lost (stale/invalid image)
Jog pendant levels are bool parameters read every tick, so a terminal can drive
without a browser:  ros2 param set /fake_panel pendant_fwd true  (also
pendant_rvs / pendant_left / pendant_right).
The real adapter (T12) wraps core/panel.PanelScan over drivers/dio.DioLink and
publishes the same message.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from amr_base import agv_repo  # noqa: F401 - puts the agv_can repo on sys.path
from amr_interfaces.msg import PanelState

import panel as panel_core  # repo module: core/panel.py

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
        self.seq = 0
        self._start_pending = False
        self._reset_pending = False
        self._pub = self.create_publisher(PanelState, "/amr/panel_state", 10)
        self.create_service(SetBool, "/sim/panel/set_mode", self._set_mode)
        self.create_service(SetBool, "/sim/panel/set_valid", self._set_valid)
        self.create_service(Trigger, "/sim/panel/press_start", self._press_start)
        self.create_service(Trigger, "/sim/panel/press_reset", self._press_reset)
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

    def _press_start(self, _req, res):
        self._start_pending = True
        res.success, res.message = True, "Start pressed"
        self.get_logger().info(res.message)
        return res

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
        self.seq += 1
        m.seq = self.seq
        levels = [bool(self.get_parameter(n).value) for n in PENDANT_PARAMS]
        m.pendant_fwd, m.pendant_rvs, m.pendant_left, m.pendant_right = panel_core.pendant_intent(*levels)
        self._pub.publish(m)


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
