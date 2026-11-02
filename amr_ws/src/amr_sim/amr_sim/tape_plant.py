"""The synthetic tape's maths: ROS-free, so a test can drive it tick by tick.

Split out of tape_synth_node.py for the same reason wheel_model.py is: the node
imports rclpy, and a plant that cannot be imported without rclpy cannot be
checked against the engine on a dev box.
"""

from __future__ import annotations

import math

from agv_core import kinematics


class TapePlant:
    """The maths, ROS-free so a test can drive it tick by tick."""

    def __init__(self, y0_m: float, tape_length_m: float, lookahead_m: float):
        self.y = float(y0_m)
        self.theta = 0.0
        self.s = 0.0  # distance along the tape
        self.tape_length_m = float(tape_length_m)
        self.lookahead_m = float(lookahead_m)

    def step(self, left_rpm: float, right_rpm: float, dt: float) -> None:
        v, omega = kinematics.wheels_to_body(left_rpm, right_rpm)
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.s += v * math.cos(self.theta) * dt

    @property
    def on_tape(self) -> bool:
        return 0.0 <= self.s <= self.tape_length_m

    def reading_mm(self) -> int | None:
        """What the MLS reports: line relative to sensor, 1 mm, or None off the end."""
        if not self.on_tape:
            return None
        e = self.y + self.lookahead_m * math.sin(self.theta)
        mm = -round(e * 1000.0)
        # The sensor window is +/-100 mm; beyond it there is simply no track.
        return mm if abs(mm) <= 100 else None
