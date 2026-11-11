#!/usr/bin/env python3
"""straight_run.py - why does the AMR not drive straight, and does it count distance right? (2026-11-09)

One straight drive answers both:
  * DRIFT (it pulls left / right): the gyro's heading change against the encoders'.
      - encoders say straight, gyro says it turned  -> one wheel is bigger than the profile
        thinks: new qr_base.wheel_scale_left / _right are printed (the 2026-11-05 method);
      - encoders ALSO say it turned                 -> the speed loops did not hold the two
        wheels equal (feedforward per wheel / PI), not the trim: send the output to Claude.
  * DISTANCE (localisation corrections "ahead -" along the path): the encoders' distance
    against a tape measure -> a new vehicle.wheel_dia_m.

How:
  1. Service running, MANUAL, a straight clear run of 8 m or more. Put a tape mark on the
     floor at the AMR's FRONT edge, AMR square to the run.
  2.  source ~/agv_can/amr_ws/install/setup.bash
      source ~/agv_can/amr_ws/env/vehicle.sh
      python3 ~/agv_can/tools/straight_run.py
  3. Stand still until it says GO (2 s gyro zero). Hold W only (no A / D, no steering) for
     8 m or more at the keypad speed, then let go. Do not correct the drift: let it drift.
  4. When it stops it prints the result and asks for the tape distance (front edge to front
     edge along the run; Enter skips the distance part).
Do it twice (or three times) and compare: the numbers should agree within ~0.2 %.
"""

import math
import statistics
import sys

import rclpy
from amr_interfaces.msg import WheelStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

ZERO_S = 2.0  # gyro bias, standing still
MOVING_RAD_S = 0.3  # wheel speed that counts as driving
STOPPED_S = 1.0  # stopped this long after the drive: done
MIN_DIST_M = 3.0
TRACK_GEOM_M = 0.37652  # wheel centre to wheel centre (vehicle.amr-qr-01.yaml track_width)


class Run(Node):
    def __init__(self) -> None:
        super().__init__("straight_run")
        self.bias_samples: list[float] = []
        self.bias: float | None = None
        self.yaw = 0.0
        self.t_imu: float | None = None
        self.start: tuple[float, float] | None = None  # wheel positions at the start of the drive
        self.last: WheelStates | None = None
        self.moved = False
        self.still_since: float | None = None
        self.done = False
        self.t0 = None
        self.create_subscription(Imu, "/imu/data_raw", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(WheelStates, "/wheel_states", self.on_wheels, qos_profile_sensor_data)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, m: Imu) -> None:
        t, wz = self.now(), m.angular_velocity.z
        if self.t0 is None:
            self.t0 = t
        if self.bias is None:
            if (
                self.last is not None
                and max(abs(self.last.left_vel_rad_s), abs(self.last.right_vel_rad_s)) > 0.05
            ):
                self.bias_samples.clear()  # moving: not a zero
                self.t0 = t
                return
            self.bias_samples.append(wz)
            if t - self.t0 >= ZERO_S and len(self.bias_samples) >= 20:
                self.bias = statistics.mean(self.bias_samples)
                print(f"gyro zero {math.degrees(self.bias):+.3f} deg/s ({len(self.bias_samples)} samples)")
                print("GO: hold W only, 8 m or more, then let go")
            self.t_imu = t
            return
        if self.start is not None and not self.done and self.t_imu is not None:
            self.yaw += (wz - self.bias) * (t - self.t_imu)
        self.t_imu = t

    def on_wheels(self, m: WheelStates) -> None:
        if not (m.left_valid and m.right_valid):
            return
        self.last = m
        if self.bias is None or self.done:
            return
        moving = max(abs(m.left_vel_rad_s), abs(m.right_vel_rad_s)) > MOVING_RAD_S
        t = self.now()
        if self.start is None:
            if moving:
                self.start = (m.left_pos_rad, m.right_pos_rad)
                self.yaw = 0.0
                print("driving ...")
            return
        if moving:
            self.moved, self.still_since = True, None
        elif self.moved:
            self.still_since = self.still_since or t
            if t - self.still_since >= STOPPED_S:
                self.done = True


def main() -> int:
    try:
        from agv_core import config  # noqa: PLC0415

        r_wheel = config.WHEEL_DIA_M / 2.0
        sl, sr = config.QR_WHEEL_SCALE_LEFT, config.QR_WHEEL_SCALE_RIGHT
        dia = config.WHEEL_DIA_M
    except Exception as e:  # noqa: BLE001
        print(f"cannot read the profile ({e}); source env/vehicle.sh first")
        return 1
    rclpy.init()
    node = Run()
    print("stand still: measuring the gyro zero ...")
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node.start is None:
            print("stopped before driving")
            return 1
    m = node.last
    if node.start is None or m is None:
        print("no drive seen (is the service running? /wheel_states and /imu/data_raw)")
        return 1
    d_l = (m.left_pos_rad - node.start[0]) * r_wheel  # already x the current wheel_scale_*
    d_r = (m.right_pos_rad - node.start[1]) * r_wheel
    dist = (d_l + d_r) / 2.0
    b = TRACK_GEOM_M
    enc_yaw = (d_r - d_l) / b
    gyro_yaw = node.yaw
    node.destroy_node()
    rclpy.shutdown()
    if dist < MIN_DIST_M:
        print(f"only {dist:.2f} m: drive {MIN_DIST_M:g} m or more (8 m is better)")
        return 1

    def side(a: float) -> str:
        return "LEFT" if a > 0 else "RIGHT"

    print()
    print(f"distance (encoders)   {dist:.3f} m   left {d_l:.3f}  right {d_r:.3f}")
    print(
        f"heading, gyro         {math.degrees(gyro_yaw):+.2f} deg  (the AMR really turned {side(gyro_yaw)})"
    )
    print(f"heading, encoders     {math.degrees(enc_yaw):+.2f} deg")
    drift = math.sin(gyro_yaw / 2.0) * dist  # sideways at the end, constant curvature
    print(f"sideways drift        about {abs(drift) * 100:.0f} cm to the {side(gyro_yaw)} over {dist:.1f} m")
    print()
    if abs(math.degrees(gyro_yaw)) < 0.5:
        print("DRIFT: straight within 0.5 deg - the wheel trim is fine")
    elif abs(enc_yaw) > 0.5 * abs(gyro_yaw) and enc_yaw * gyro_yaw > 0:
        print("DRIFT: the ENCODERS also saw the turn: the speed loops did not keep the wheels equal.")
        print("       Not a trim problem - send this whole output to Claude.")
    else:
        dlt = (b * gyro_yaw - (d_r - d_l)) / (d_l + d_r)
        nl, nr = sl * (1.0 - dlt), sr * (1.0 + dlt)
        print(f"DRIFT: wheel trim. Now  wheel_scale_left {sl:.4f}  wheel_scale_right {sr:.4f}")
        print(f"                  New  wheel_scale_left {nl:.4f}  wheel_scale_right {nr:.4f}")
        if not (0.95 <= nl <= 1.05 and 0.95 <= nr <= 1.05):
            print("       outside 0.95..1.05: look for a mechanical cause first (tyre, brake dragging)")
        else:
            print(
                "       put the New values in profiles/amr-qr-01.json (qr_base), rebuild, restart, retest"
            )
    print()
    try:
        tape = input("tape distance in m (Enter to skip): ").strip()
    except EOFError:
        tape = ""
    if tape:
        try:
            real = float(tape.replace(",", "."))
        except ValueError:
            print("not a number")
            return 0
        # the encoders' straight distance vs the tape (the arc is a few mm longer: ignored)
        k = real / dist
        print(f"DISTANCE: the encoders read {100 * (1 / k - 1):+.2f} % against the tape")
        print(
            f"          wheel_dia_m {dia:.4f} -> {dia * k:.4f}  (vehicle in profiles/amr-qr-01.json)"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
