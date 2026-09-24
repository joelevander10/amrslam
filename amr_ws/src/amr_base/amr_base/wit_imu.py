"""WitMotion serial IMU (the AMR QR's IMU on qr_base.imu_port), framework-free.

Frame: 11 bytes, 0x55 | type | 8 data bytes | checksum (sum of the first 10, low byte).

    0x51 acceleration      ax ay az (int16 / 32768 * 16 g), temperature
    0x52 angular velocity  wx wy wz (int16 / 32768 * 2000 deg/s)
    0x53 angle             roll pitch yaw (int16 / 32768 * 180 deg)

The QR controller (trackless_module.py) parsed 0x53 only and used the fused yaw.
The ROS stack takes YAW RATE only (ekf.yaml: vyaw from /imu/data, bias removed by
imu_bias_node), because an absolute yaw from a consumer 6-axis IMU drifts; so:

    gyro       : wz from 0x52 - the preferred source.
    angle_diff : no 0x52 seen for gyro_absent_s -> wz = d(yaw)/dt between consecutive
                 0x53 frames (wrap-safe). Works, but the unit's own filter is in the
                 loop, so the diagnostics flag it and the fix is configuring the unit to
                 send 0x52 (see the commissioning notes).

A frame with a bad checksum is dropped and the parser resynchronises on the next 0x55.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

HEADER = 0x55
FRAME_LEN = 11
ACC, GYRO, ANGLE = 0x51, 0x52, 0x53
GYRO_FULL_SCALE_DPS = 2000.0
ANGLE_FULL_SCALE_DEG = 180.0
ACC_FULL_SCALE_G = 16.0

GYRO_MODE, ANGLE_DIFF_MODE, NO_DATA = "gyro", "angle_diff", "none"


@dataclass(frozen=True)
class Frame:
    kind: int
    values: tuple[float, float, float]
    t: float


class WitParser:
    """Byte stream -> checked frames. Pure (the caller supplies the receive time)."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames = 0
        self.bad_checksum = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes, t: float) -> list[Frame]:
        self._buf.extend(data)
        out: list[Frame] = []
        while True:
            i = self._buf.find(bytes([HEADER]))
            if i < 0:
                self.skipped_bytes += len(self._buf)
                self._buf.clear()
                return out
            if i:
                self.skipped_bytes += i
                del self._buf[:i]
            if len(self._buf) < FRAME_LEN:
                return out
            frame = bytes(self._buf[:FRAME_LEN])
            if sum(frame[:10]) & 0xFF != frame[10]:
                self.bad_checksum += 1
                del self._buf[:1]  # resync on the next header byte
                continue
            del self._buf[:FRAME_LEN]
            f = decode(frame, t)
            if f is not None:
                self.frames += 1
                out.append(f)


def decode(frame: bytes, t: float) -> Frame | None:
    kind = frame[1]
    a, b, c = struct.unpack_from("<hhh", frame, 2)
    if kind == GYRO:
        k = GYRO_FULL_SCALE_DPS / 32768.0
    elif kind == ANGLE:
        k = ANGLE_FULL_SCALE_DEG / 32768.0
    elif kind == ACC:
        k = ACC_FULL_SCALE_G / 32768.0
    else:
        return None  # magnetometer, quaternion, ...: not used
    return Frame(kind, (a * k, b * k, c * k), t)


@dataclass(frozen=True)
class YawRate:
    t: float
    wz_rad_s: float
    mode: str
    yaw_deg: float | None


class YawRateSource:
    """Frames -> yaw-rate samples, choosing gyro or angle differentiation. Pure."""

    def __init__(self, sign: float = 1.0, gyro_absent_s: float = 1.0, max_diff_dt_s: float = 0.5) -> None:
        self.sign = 1.0 if sign >= 0 else -1.0
        self.gyro_absent_s = gyro_absent_s
        self.max_diff_dt_s = max_diff_dt_s
        self.t_gyro: float | None = None
        self._last_angle: tuple[float, float] | None = None  # (t, yaw_deg)
        self.yaw_deg: float | None = None
        self.mode = NO_DATA
        self.samples = 0

    def offer(self, f: Frame) -> YawRate | None:
        if f.kind == GYRO:
            self.t_gyro = f.t
            self.mode = GYRO_MODE
            self.samples += 1
            return YawRate(f.t, self.sign * math.radians(f.values[2]), GYRO_MODE, self.yaw_deg)
        if f.kind != ANGLE:
            return None
        yaw = f.values[2]
        self.yaw_deg = yaw
        prev = self._last_angle
        if prev is not None and f.t <= prev[0]:
            # Several frames out of one serial read share a receive time: keep the older
            # reference, so the rotation between them is not lost from the next difference.
            return None
        self._last_angle = (f.t, yaw)
        if self.t_gyro is not None and f.t - self.t_gyro <= self.gyro_absent_s:
            return None  # the gyro is live; the angle is informational
        if prev is None:
            return None
        dt = f.t - prev[0]
        if dt > self.max_diff_dt_s:
            return None
        d = (yaw - prev[1] + 180.0) % 360.0 - 180.0
        self.mode = ANGLE_DIFF_MODE
        self.samples += 1
        return YawRate(f.t, self.sign * math.radians(d / dt), ANGLE_DIFF_MODE, yaw)
