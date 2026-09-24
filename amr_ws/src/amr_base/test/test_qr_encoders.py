"""AMR QR encoders: unwrapping, speed, TPDO/SDO acquisition against a scripted bus."""

import math
import struct

import can
import pytest

from amr_base import canopen
from amr_base.qr_encoders import (
    FALLBACK_RANGE,
    SDO,
    TPDO,
    EncoderPair,
    SpeedEstimator,
    Unwrapper,
    WheelEncoder,
    range_from_6002,
)

RANGE = 1 << 24
CPR = 8192.0


def test_unwrap_forward_and_back_through_the_24_bit_wrap():
    u = Unwrapper(RANGE)
    assert u.update(RANGE - 100) == 0
    assert u.update(RANGE - 1) == 99
    assert u.update(50) == 150  # across the wrap: +51, not -16.7 M
    assert u.update(RANGE - 10) == 90  # and back
    assert u.rejected == 0


def test_unwrap_rejects_out_of_range_and_half_range_steps():
    u = Unwrapper(RANGE)
    assert u.update(RANGE) is None  # out of range
    assert u.update(0) == 0
    assert u.update(RANGE // 2) is None  # ambiguous direction
    assert u.rejected == 2
    # chain continues from the rejected reading as the new reference
    assert u.update(RANGE // 2 + 10) == 10


def test_speed_estimator_window():
    s = SpeedEstimator(window_s=0.06)
    assert s.counts_per_s() is None
    for i in range(10):
        s.add(i * 0.02, i * 100)
    assert s.counts_per_s() == pytest.approx(5000.0)
    s.add(0.18, 900)  # a duplicate timestamp is ignored
    assert s.counts_per_s() == pytest.approx(5000.0)


def test_wheel_encoder_units_and_invert():
    e = WheelEncoder(1, CPR, RANGE, invert=True)
    e.feed(1000, 0.0, TPDO)
    e.feed((1000 - 8192) % RANGE, 1.0, TPDO)  # one wheel turn backwards in raw terms (wrapped)...
    assert e.counts == 8192  # ...is one turn FORWARD for an inverted wheel
    assert e.rad == pytest.approx(2 * math.pi)
    assert e.rad_s == pytest.approx(2 * math.pi)
    assert e.fresh(1.1, 0.2) and not e.fresh(1.3, 0.2)


class ScriptedBus:
    """Answers SDO downloads (1800h) and uploads (6004h) per node; can queue TPDOs."""

    def __init__(self, positions=None, fail_config=(), objects=None):
        self.positions = dict(positions or {})
        self.objects = dict(objects or {})  # (node, index) -> u32, answered to SDO uploads
        self.fail_config = set(fail_config)
        self.rx = []
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)
        cob = msg.arbitration_id
        if 0x600 < cob < 0x680:
            node = cob - 0x600
            d = bytes(msg.data)
            idx, sub = d[1] | (d[2] << 8), d[3]
            if d[0] & 0xE0 == 0x20:  # download
                cs = 0x80 if node in self.fail_config else 0x60
                self.rx.append(can.Message(arbitration_id=0x580 + node, data=bytes([cs]) + d[1:4] + bytes(4)))
            elif d[0] == 0x40 and idx == 0x6004 and node in self.positions:
                val = struct.pack("<I", self.positions[node])
                self.rx.append(can.Message(arbitration_id=0x580 + node, data=bytes([0x43]) + d[1:4] + val))
            elif d[0] == 0x40 and (node, idx) in self.objects:
                val = struct.pack("<I", self.objects[(node, idx)])
                self.rx.append(can.Message(arbitration_id=0x580 + node, data=bytes([0x43]) + d[1:4] + val))
            _ = sub

    def recv(self, timeout=None):
        return self.rx.pop(0) if self.rx else None

    def shutdown(self):
        pass


def _pair(bus, mode, clock):
    return EncoderPair(
        canopen.Router(bus), 1, 2, CPR, RANGE, False, True, mode, 20, log=lambda s: None, clock=clock
    )


def test_tpdo_mode_configures_event_timer_and_inhibit_then_starts_nodes():
    bus = ScriptedBus()
    t = [0.0]
    pair = _pair(bus, TPDO, lambda: t[0])
    pair.start()
    writes = [(m.arbitration_id, bytes(m.data)) for m in bus.sent if 0x600 < m.arbitration_id < 0x680]
    # per node: 1800h:03 inhibit = 200 x 100 us, 1800h:05 event = 20 ms
    assert (0x601, bytes([0x2B, 0x00, 0x18, 0x03, 200, 0, 0, 0])) in writes
    assert (0x601, bytes([0x2B, 0x00, 0x18, 0x05, 20, 0, 0, 0])) in writes
    assert (0x602, bytes([0x2B, 0x00, 0x18, 0x05, 20, 0, 0, 0])) in writes
    nmt = [bytes(m.data) for m in bus.sent if m.arbitration_id == 0]
    assert nmt == [bytes([1, 1]), bytes([1, 2])]
    # a TPDO1 frame lands on the right wheel, inverted
    bus.rx.append(can.Message(arbitration_id=0x182, data=struct.pack("<I", 5000)))
    bus.rx.append(can.Message(arbitration_id=0x182, data=struct.pack("<I", 4000)))
    pair.router.pump(0.01)
    assert pair.right.counts == 1000 and pair.right.source == TPDO
    assert pair.left.counts is None


def test_tpdo_mode_raises_if_configuration_fails_but_auto_falls_back_to_sdo():
    t = [0.0]
    with pytest.raises(RuntimeError):
        _pair(ScriptedBus(fail_config={1}), TPDO, lambda: t[0]).start()
    bus = ScriptedBus(positions={1: 100, 2: 200}, fail_config={1, 2})
    pair = _pair(bus, "auto", lambda: t[0])
    pair.start()
    pair.poll(0.0)
    assert pair.left.counts == 0 and pair.left.source == SDO
    assert pair.sdo_reads == 2


def test_auto_mode_polls_only_the_wheel_without_recent_tpdo():
    bus = ScriptedBus(positions={1: 10, 2: 20})
    t = [1.0]
    pair = _pair(bus, "auto", lambda: t[0])
    pair.start()
    bus.rx.append(can.Message(arbitration_id=0x181, data=struct.pack("<I", 10)))
    pair.router.pump(0.01)
    pair.poll(1.0)
    assert pair.sdo_reads == 1  # only the right wheel (no TPDO seen from node 2)
    assert pair.right.source == SDO
    pair.poll(1.2)  # left TPDO now 0.2 s old (> 3 periods): SDO for both
    assert pair.sdo_reads == 3


def test_range_from_6002_reads_all_ones_as_a_power_of_two():
    assert range_from_6002(0x00FFFFFF) == 1 << 24  # EDS default style: largest position
    assert range_from_6002(1 << 25) == 1 << 25  # CiA 406 style: number of steps
    assert range_from_6002(100_000_000) == 100_000_000
    assert range_from_6002(None) is None and range_from_6002(0) is None


def test_auto_range_per_encoder_from_6002_and_32_bit_fallback():
    # the AMR QR on 2026-09-24: left raw 14 013 020, right raw 27 601 874 - past 24 bits
    bus = ScriptedBus(objects={(1, 0x6002): 0x1FFFFFF, (1, 0x6001): 8192, (2, 0x6001): 8192})
    t = [0.0]
    pair = EncoderPair(
        canopen.Router(bus), 1, 2, CPR, 0, False, False, "tpdo", 20, log=lambda s: None, clock=lambda: t[0]
    )
    pair.start()
    assert pair.left.range_counts == 1 << 25
    assert pair.right.range_counts == FALLBACK_RANGE  # node 2 did not answer 6002h
    assert pair.info[1] == {"units_per_rev": 8192, "range_6002": 0x1FFFFFF, "range": 1 << 25}
    for raw in (27_601_874, 27_601_874 + 8192):
        bus.rx.append(can.Message(arbitration_id=0x182, data=struct.pack("<I", raw)))
    pair.router.pump(0.01)
    assert pair.right.counts == 8192 and pair.right.unwrap.rejected == 0
    # left wraps at 2^25 correctly
    for raw in ((1 << 25) - 100, 50):
        bus.rx.append(can.Message(arbitration_id=0x181, data=struct.pack("<I", raw)))
    pair.router.pump(0.01)
    assert pair.left.counts == 150
