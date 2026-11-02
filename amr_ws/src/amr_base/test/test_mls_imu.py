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


# ---- F04: malformed PDOs never become a measurement --------------------------


def _tpdo_objs(bits=16):
    return {
        (0x2006, 2): 1,
        (mls_imu.TPDO_GYRO_COMM, 1): 0x28A,
        (mls_imu.TPDO_GYRO_MAP, 0): 2,
        (mls_imu.TPDO_GYRO_MAP, 1): (GYRO << 16) | (3 << 8) | bits,
        (mls_imu.TPDO_GYRO_MAP, 2): (STAMP << 16) | 16,
    }


@pytest.mark.parametrize("data", [b"", b"\x01", b"\x01\x02", b"\x01\x02\x03"])
def test_a_short_frame_is_rejected_not_zero_padded(data):
    """An empty frame used to decode as yaw rate 0.0 with a fresh receipt
    time - a fabricated measurement that kept the IMU 'fresh' downstream."""
    got = []
    link = FakeLink(_tpdo_objs())
    imu = MlsImu(link, 10, got.append, log=lambda s: None)
    assert imu.start() == "tpdo"
    link.routes[0x28A](Frame(data))
    assert got == [] and imu.samples == 0
    assert imu.rejected == 1 and "mapping needs 4" in imu.last_reject
    assert imu.tracker._last_ms is None, "a rejected frame touched the stamp chain"
    # a whole frame still works, and the value is the signed one
    link.routes[0x28A](Frame(struct.pack("<hH", -164, 1000)))
    assert len(got) == 1 and got[0].wz_rad_s < 0 and imu.samples == 1


def test_decode_mapped_needs_every_mapped_byte():
    m = parse_mapping([(GYRO << 16) | (3 << 8) | 16, (STAMP << 16) | 16])
    with pytest.raises(ValueError):
        mls_imu.decode_mapped(b"\x00\x00\x00", m)
    assert mls_imu.decode_mapped(b"\x00\x00\x00\x00", m)[(GYRO, 3)] == 0
    with pytest.raises(ValueError):
        mls_imu.mapping_bits(parse_mapping([(GYRO << 16) | (3 << 8) | 12]))
    with pytest.raises(ValueError):
        mls_imu.mapping_bits(parse_mapping([(GYRO << 16) | (3 << 8) | 64, (STAMP << 16) | 16]))


def test_a_mapping_with_the_wrong_gyro_width_falls_back_to_polling():
    link = FakeLink(_tpdo_objs(bits=32))
    imu = MlsImu(link, 10, lambda s: None, log=lambda s: None)
    assert imu.start() == "sdo" and not link.routes


# ---- F08: a sensor absent at boot is picked up later -----------------------


def test_an_absent_sensor_is_rediscovered_with_one_short_probe_per_period():
    link = FakeLink({})
    reads = []
    real = link.read
    link.read = lambda node, index, sub=0, timeout=0.4: (
        reads.append((index, sub, timeout)),
        real(node, index, sub),
    )[1]
    got = []
    imu = MlsImu(link, 10, got.append, log=lambda s: None)
    assert imu.start(now=0.0) == "off"
    imu.poll(1.0)
    assert len(reads) == 1, "polled while off before the rediscovery period"
    imu.poll(5.0)
    assert reads[-1] == (0x2006, 2, MlsImu.PROBE_TIMEOUT_S) and imu.mode == "off"
    imu.poll(7.0)
    assert len(reads) == 2, "probed more than once per period"
    # the sensor comes up
    link.objects.update(_tpdo_objs())
    imu.poll(10.0)
    assert imu.mode == "tpdo" and imu.restarts == 1 and 0x28A in link.routes
    link.routes[0x28A](Frame(struct.pack("<hH", 164, 1000)))
    assert len(got) == 1
