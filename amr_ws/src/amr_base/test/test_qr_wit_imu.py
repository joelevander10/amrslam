"""WitMotion parser and the yaw-rate source (AMR QR IMU)."""

import math
import struct

import pytest

from amr_base import wit_imu
from amr_base.wit_imu import ANGLE, ANGLE_DIFF_MODE, GYRO, GYRO_MODE, WitParser, YawRateSource


def frame(kind: int, a: int, b: int, c: int, extra: int = 0) -> bytes:
    body = bytes([0x55, kind]) + struct.pack("<hhhh", a, b, c, extra)
    return body + bytes([sum(body) & 0xFF])


def gyro_z(dps: float) -> bytes:
    return frame(GYRO, 0, 0, int(round(dps / 2000.0 * 32768)))


def yaw(deg: float) -> bytes:
    return frame(ANGLE, 0, 0, int(round(deg / 180.0 * 32768)))


def test_parser_resyncs_and_rejects_bad_checksum():
    p = WitParser()
    bad = bytearray(gyro_z(10.0))
    bad[10] ^= 0xFF
    stream = b"\x00\x13" + bytes(bad) + gyro_z(20.0)
    out = p.feed(stream[:7], 0.0) + p.feed(stream[7:], 0.1)  # split across reads
    assert [f.kind for f in out] == [GYRO]
    assert out[0].values[2] == pytest.approx(20.0, abs=0.07)
    assert p.bad_checksum == 1


def test_yaw_matches_the_qr_controller_decoding():
    # trackless_module: raw = (hi<<8|lo)/32768*180, minus 360 at >= 180: a signed int16
    p = WitParser()
    (f,) = p.feed(frame(ANGLE, 0, 0, -16384), 0.0)
    assert f.values[2] == pytest.approx(-90.0)


def test_gyro_is_preferred_and_signed():
    src = YawRateSource(sign=-1.0)
    p = WitParser()
    out = [src.offer(f) for f in p.feed(gyro_z(30.0) + yaw(5.0), 0.0)]
    assert out[0].mode == GYRO_MODE
    assert out[0].wz_rad_s == pytest.approx(-math.radians(30.0), rel=1e-3)
    assert out[1] is None  # angle is informational while the gyro is live


def test_angle_difference_fallback_is_wrap_safe_and_survives_batched_frames():
    src = YawRateSource()
    p = WitParser()
    samples = []
    for t, deg in ((0.0, 178.0), (0.1, -178.0)):  # +4 deg across the +-180 wrap in 0.1 s
        samples += [src.offer(f) for f in p.feed(yaw(deg), t)]
    s = samples[-1]
    assert s.mode == ANGLE_DIFF_MODE
    assert s.wz_rad_s == pytest.approx(math.radians(40.0), rel=0.02)
    # two frames in one read share a receive time: the rotation is carried to the next diff
    src2, p2 = YawRateSource(), WitParser()
    src2.offer(p2.feed(yaw(0.0), 0.0)[0])
    batch = [src2.offer(f) for f in p2.feed(yaw(1.0) + yaw(2.0), 0.1)]
    assert batch[1] is None
    nxt = src2.offer(p2.feed(yaw(3.0), 0.2)[0])
    integrated = (batch[0].wz_rad_s + nxt.wz_rad_s) * 0.1
    assert integrated == pytest.approx(math.radians(3.0), rel=0.02)  # 0 -> 3 deg, nothing lost


def test_configuration_frames():
    assert wit_imu.UNLOCK == bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
    assert wit_imu.SAVE == bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])
    rate50 = wit_imu.command(wit_imu.REG_RRATE, wit_imu.RATE_CODES[50])
    baud115k = wit_imu.command(wit_imu.REG_BAUD, wit_imu.BAUD_CODES[115200])
    assert rate50 == bytes([0xFF, 0xAA, 0x03, 0x08, 0x00])
    assert baud115k == bytes([0xFF, 0xAA, 0x04, 0x06, 0x00])
    assert wit_imu.command(wit_imu.REG_RSW, 0x0E) == bytes([0xFF, 0xAA, 0x02, 0x0E, 0x00])
    # why the AMR QR runs at 10 Hz: three frames at 9600 baud cannot go past ~29 Hz
    assert wit_imu.frame_bytes_per_s(50) > 9600
    assert wit_imu.frame_bytes_per_s(100) < 0.8 * 115200
