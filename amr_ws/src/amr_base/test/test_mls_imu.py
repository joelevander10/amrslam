"""amr_base.mls_imu: mapping-driven TPDO decode, SDO polling fallback, stamp rules."""

import math
import struct

import pytest

from amr_base import mls_imu
from amr_base.mls_imu import MlsImu, StampTracker, decode_mapped, gyro_z_rad_s, parse_mapping

GYRO = 0x2034
STAMP = 0x2035


def test_parse_and_decode_by_mapping():
    entries = [
        (GYRO << 16) | (1 << 8) | 16,
        (GYRO << 16) | (2 << 8) | 16,
        (GYRO << 16) | (3 << 8) | 16,
        (STAMP << 16) | 16,
    ]
    m = parse_mapping(entries + [0])
    assert [(e.index, e.sub, e.bits) for e in m] == [
        (GYRO, 1, 16),
        (GYRO, 2, 16),
        (GYRO, 3, 16),
        (STAMP, 0, 16),
    ]
    data = struct.pack("<hhhH", 5, -6, -1000, 65000)
    f = decode_mapped(data, m)
    assert f[(GYRO, 3)] == 0xFC18 and f[(STAMP, 0)] == 65000
    assert gyro_z_rad_s(f[(GYRO, 3)]) == pytest.approx(-1000 * 125 * 2**-11 * math.pi / 180)
    assert gyro_z_rad_s(f[(GYRO, 3)], sign=-1.0) > 0


def test_stamp_tracker_rejects_duplicates_and_backwards_but_allows_wrap():
    t = StampTracker(gap_s=1.0)
    assert t.accept(65500, 0.00)
    assert t.accept(65530, 0.01) and t.delta_ms == 30
    assert t.accept(10, 0.02) and t.delta_ms == 16  # wrap 65536
    assert not t.accept(10, 0.03) and t.rejected == 1  # duplicate
    assert not t.accept(5, 0.04) and t.rejected == 2  # backwards
    assert t.accept(None, 0.05)  # polled without a stamp this round
    assert t.accept(100, 5.0) and t.resets == 1  # after a gap: chain restarts, no delta trusted


class FakeLink:
    def __init__(self, objects):
        self.objects = objects
        self.router = self
        self.routes = {}

    def read(self, node, index, sub=0, timeout=0.4):
        return self.objects.get((index, sub))

    def add(self, cob, handler):
        self.routes[cob] = handler


class Frame:
    def __init__(self, data):
        self.data = data


def test_tpdo_mode_when_the_sensor_reports_an_enabled_yaw_rate_pdo():
    objs = {
        (0x2006, 2): 1,
        (mls_imu.TPDO_GYRO_COMM, 1): 0x28A,
        (mls_imu.TPDO_GYRO_MAP, 0): 2,
        (mls_imu.TPDO_GYRO_MAP, 1): (GYRO << 16) | (3 << 8) | 16,
        (mls_imu.TPDO_GYRO_MAP, 2): (STAMP << 16) | 16,
    }
    got = []
    link = FakeLink(objs)
    imu = MlsImu(link, 10, got.append, log=lambda s: None)
    assert imu.start() == "tpdo" and 0x28A in link.routes
    link.routes[0x28A](Frame(struct.pack("<hH", 164, 1000)))  # 164 LSB = 10.01 deg/s
    link.routes[0x28A](Frame(struct.pack("<hH", 164, 1000)))  # duplicate stamp -> dropped
    assert len(got) == 1 and got[0].wz_rad_s == pytest.approx(math.radians(164 * 125 * 2**-11))
    assert got[0].stamp_ms == 1000


def test_sdo_mode_when_the_pdo_is_disabled():
    objs = {(0x2006, 2): 1, (mls_imu.TPDO_GYRO_COMM, 1): 0x80000000, (GYRO, 3): 0xFFFF, (STAMP, 0): 7}
    got = []
    imu = MlsImu(FakeLink(objs), 10, got.append, poll_hz=50.0, stamp_every=2, log=lambda s: None)
    assert imu.start() == "sdo"
    imu.poll(0.0)
    imu.poll(0.01)  # too early
    imu.poll(0.02)
    assert len(got) == 2
    assert got[0].wz_rad_s == pytest.approx(-math.radians(125 * 2**-11))
    assert got[0].stamp_ms is None and got[1].stamp_ms == 7


def test_sdo_mode_backs_off_when_the_sensor_is_absent():
    objs = {(0x2006, 2): 1, (mls_imu.TPDO_GYRO_COMM, 1): 0}
    imu = MlsImu(FakeLink(objs), 10, lambda s: None, log=lambda s: None)
    assert imu.start() == "sdo"
    for k in range(5):
        imu.poll(k * 0.02)
    assert imu.misses == 5  # single misses are retried at the next poll
    imu.poll(0.5)
    assert imu.misses == 5  # after 5 in a row: not retried before 2 s
    imu.poll(2.2)
    assert imu.misses == 6
