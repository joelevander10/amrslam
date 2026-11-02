"""tape_synth_node: a straight strip of magnetic tape, for the LINE layer in sim.

Re-hosts the plant from the tape engine's test rig (`amr_line/test/harness.py`,
itself verbatim from gy-demo): the vehicle's lateral offset y and heading theta
integrate e_dot = v*sin(theta), theta_dot = omega from the wheel speeds the
fake base actually reaches - not from the commanded ones - so the drivers' slew
limit still shapes the loop exactly as it does in the offline step-response
tests. What the sensor reports is the NEGATIVE of the sensor's offset from the
tape, quantised to 1 mm at 100 Hz, which is the polarity measured on the
machine on 2026-08-31 and the one the engine's INVERT_ERROR is tuned against.

The tape is `tape_length_m` long and then stops. Past its end the sensor
reports no track, so a follower that runs off the end must finish DONE on its
own line-loss distance budget - that is the swap-out test, and it is the
reason this node exists: a LINE layer that can be entered, run, ended and left
without a vehicle.

    /wheel_states    WheelStates (in)   the fake base's wheel speeds
    /amr/line_track  LineTrack          100 Hz, tpdo-sourced, seq-numbered

Parameters: y0_mm (initial offset, default 30 = the rig's step), tape_length_m
(default 20), rate_hz (100). Nothing here reads the engine's gains.
"""

from __future__ import annotations

import math
import time

import rclpy
from agv_core import config
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from amr_interfaces.msg import LineTrack, WheelStates
from amr_sim.tape_plant import TapePlant

SENSOR_DATA = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)
WHEEL_RAD_S_PER_MOTOR_RPM = 2.0 * math.pi / 60.0 / config.GEAR_RATIO


class TapeSynth(Node):
    def __init__(self) -> None:
        super().__init__("tape_synth")
        self.declare_parameter("y0_mm", 30.0)
        self.declare_parameter("tape_length_m", 20.0)
        self.declare_parameter("rate_hz", 100.0)
        self.plant = TapePlant(
            float(self.get_parameter("y0_mm").value) / 1000.0,
            float(self.get_parameter("tape_length_m").value),
            config.SENSOR_LOOKAHEAD_M,
        )
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self._left = self._right = 0.0
        self._seq = 0
        self._t_last = time.monotonic()
        self._pub = self.create_publisher(LineTrack, "/amr/line_track", SENSOR_DATA)
        self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, SENSOR_DATA)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"tape: {self.plant.tape_length_m} m straight, sensor starts "
            f"{self.plant.y * 1000:.0f} mm off it"
        )

    def _on_wheels(self, m: WheelStates) -> None:
        # rad/s at the wheel -> motor r/min, the engine's unit
        self._left = m.left_vel_rad_s / WHEEL_RAD_S_PER_MOTOR_RPM
        self._right = m.right_vel_rad_s / WHEEL_RAD_S_PER_MOTOR_RPM

    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._t_last = now - self._t_last, now
        self.plant.step(self._left, self._right, min(dt, 5 * self.dt))
        mm = self.plant.reading_mm()

        m = LineTrack()
        m.stamp = self.get_clock().now().to_msg()
        m.source = "tpdo"
        m.sample_age_s = 0.0
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        m.seq = self._seq
        m.lcp_mm = [0, 0, 0]
        m.valid = [False, False, False]
        if mm is None:
            m.nlcp, m.nlcp_label, m.line_good = 0, "none", False
        else:
            # One tape -> LCP2 is the track (MLS table 17).
            m.lcp_mm[1] = int(mm)
            m.valid[1] = True
            m.nlcp, m.nlcp_label, m.line_good = 2, "one track", True
        m.track_level = 7 if mm is not None else 0
        m.polarity = "north"
        self._pub.publish(m)


def main() -> None:
    rclpy.init()
    node = TapeSynth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
