"""The gyro inside the SICK MLS (CAN node 10), for /imu/data_raw (T10).

Two acquisition modes, chosen at start by reading the sensor:

  TPDO   if the yaw-rate TPDO (1806h) has a COB-ID and is enabled, its
         frames are decoded by the MAPPING THE SENSOR REPORTS (1A06h), not by
         an assumed layout. Enabling that TPDO is a commissioning write
         (`read_imu.py tpdo gyro --go` + sensor restart); this module only
         ever reads.
  SDO    otherwise, one 2034h:3 read per poll (yaw rate, the only axis the
         EKF fuses) and a 2035h stamp read every `stamp_every` polls.
         ~4 ms of bus time per sample at 125 kbps.

Scaling and the 16-bit millisecond stamp are drivers/canbus/read_imu.py's.
The stamp is only used to reject out-of-order/duplicate samples and to
measure the delivered rate; the ROS header stamp is the receive time, because
the sensor clock and the PC clock are not related (spec §2.4).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import amr_base.agv_repo  # noqa: F401  (puts the repo layer dirs on sys.path)

import read_imu  # repo module

OBJ_GYRO_Z = (read_imu.OBJ_GYRO, 3)
OBJ_STAMP = read_imu.OBJ_STAMP
TPDO_GYRO_COMM = read_imu.TPDO_SLOTS["gyro"][0]  # 1806h
TPDO_GYRO_MAP = TPDO_GYRO_COMM + 0x200  # 1A06h
DPS_TO_RAD_S = math.pi / 180.0


@dataclass(frozen=True)
class MapEntry:
    index: int
    sub: int
    bits: int


def parse_mapping(entries: list[int]) -> list[MapEntry]:
    """1A06h:1..n raw u32 entries -> ordered fields (index, sub, bit length)."""
    out = []
    for e in entries:
        if e == 0:
            continue
        out.append(MapEntry((e >> 16) & 0xFFFF, (e >> 8) & 0xFF, e & 0xFF))
    return out


def decode_mapped(data: bytes, mapping: list[MapEntry]) -> dict[tuple[int, int], int]:
    """Slice a PDO payload by its mapping. Values are UNSIGNED; scale later."""
    out, bit = {}, 0
    raw = int.from_bytes(bytes(data).ljust(8, b"\0"), "little")
    for m in mapping:
        if m.bits % 8:
            raise ValueError(
                f"mapping entry {m.index:04X}h:{m.sub:02X} is {m.bits} bits; only byte-aligned handled"
            )
        out[(m.index, m.sub)] = (raw >> bit) & ((1 << m.bits) - 1)
        bit += m.bits
    return out


def gyro_z_rad_s(raw16: int, sign: float = 1.0) -> float:
    return read_imu.s16(raw16 & 0xFFFF) * read_imu.GYRO_LSB_DPS * DPS_TO_RAD_S * sign


class StampTracker:
    """Unwraps the u16 millisecond stamp and flags samples that go backwards.

    A wrap and a 65 s gap are indistinguishable (read_imu.stamp_delta_ms), so
    `accept` also requires the local receive interval to be short; a sample
    that arrives after a long silence resets the chain rather than being
    trusted as a continuation (spec §2.4: discard the first delta after a gap).
    """

    def __init__(self, gap_s: float = 1.0):
        self.gap_s = gap_s
        self._last_ms: int | None = None
        self._last_t: float | None = None
        self.delta_ms: int | None = None
        self.rejected = 0
        self.resets = 0

    def accept(self, stamp_ms: int | None, t_recv: float) -> bool:
        if stamp_ms is None:
            return True  # polled without a stamp this round
        if self._last_t is not None and t_recv - self._last_t > self.gap_s:
            self.resets += 1
            self._last_ms = None
        if self._last_ms is not None:
            d = read_imu.stamp_delta_ms(stamp_ms, self._last_ms)
            if d == 0 or d > 32768:  # duplicate, or "backwards" read as a huge forward wrap
                self.rejected += 1
                return False
            self.delta_ms = d
        self._last_ms, self._last_t = stamp_ms, t_recv
        return True


@dataclass
class ImuSample:
    t_mono: float
    wz_rad_s: float
    stamp_ms: int | None


class MlsImu:
    """Acquisition state for one MLS. Bus-thread only; `on_sample` is called
    from within the bus loop with each accepted sample."""

    def __init__(
        self,
        link,
        node: int,
        on_sample,
        sign: float = 1.0,
        poll_hz: float = 50.0,
        stamp_every: int = 10,
        log=print,
    ):
        self.link, self.node, self.on_sample, self.sign, self.log = link, node, on_sample, sign, log
        self.poll_period = 1.0 / poll_hz
        self.stamp_every = max(1, int(stamp_every))
        self.mode = "off"
        self.mapping: list[MapEntry] = []
        self.cob_id: int | None = None
        self.tracker = StampTracker()
        self.samples = 0
        self.misses = 0
        self._next_poll = 0.0
        self._polls = 0
        self._consecutive = 0

    # One late reply (a busy bus) is retried at the next poll; only a sensor
    # that keeps not answering is backed off.
    MISSES_BEFORE_BACKOFF = 5

    # -- start --

    def start(self) -> str:
        enabled = self.link.read(self.node, *read_imu.OBJ_ENABLE)
        if enabled is None:
            self.log(f"MLS node {self.node} did not answer 2006h:02 - IMU off")
            self.mode = "off"
            return self.mode
        if enabled == 0:
            self.log("MLS 2006h:02 MEMS = 0: the IMU is disabled in the sensor configuration")
        raw = self.link.read(self.node, TPDO_GYRO_COMM, 1)
        cob = None if raw is None else raw & 0x7FF
        if raw is not None and not (raw & read_imu.COB_DISABLED) and cob:
            count = self.link.read(self.node, TPDO_GYRO_MAP, 0) or 0
            entries = [self.link.read(self.node, TPDO_GYRO_MAP, i) or 0 for i in range(1, count + 1)]
            self.mapping = parse_mapping(entries)
            if (OBJ_GYRO_Z[0], OBJ_GYRO_Z[1]) in {(m.index, m.sub) for m in self.mapping}:
                self.cob_id = cob
                self.link.router.add(cob, self._on_tpdo)
                self.mode = "tpdo"
                self.log(
                    f"MLS yaw rate on TPDO 0x{cob:03X}: "
                    + ", ".join(f"{m.index:04X}h:{m.sub:02X}/{m.bits}" for m in self.mapping)
                )
                return self.mode
            self.log(f"MLS TPDO 0x{cob:03X} is enabled but does not carry 2034h:03; polling instead")
        self.mode = "sdo"
        self.log(f"MLS yaw rate by SDO polling at {1.0 / self.poll_period:.0f} Hz (1806h disabled)")
        return self.mode

    # -- TPDO path --

    def _on_tpdo(self, m) -> None:
        fields = decode_mapped(m.data, self.mapping)
        t = time.monotonic()
        stamp = fields.get(OBJ_STAMP)
        if not self.tracker.accept(stamp, t):
            return
        self.samples += 1
        self.on_sample(ImuSample(t, gyro_z_rad_s(fields[OBJ_GYRO_Z], self.sign), stamp))

    # -- SDO path --

    def poll(self, now: float) -> None:
        if self.mode != "sdo" or now < self._next_poll:
            return
        self._next_poll = now + self.poll_period
        raw = self.link.read(self.node, *OBJ_GYRO_Z, timeout=0.05)
        if raw is None:
            self.misses += 1
            self._consecutive += 1
            if self._consecutive >= self.MISSES_BEFORE_BACKOFF:
                self._next_poll = now + 2.0  # an absent sensor costs one short timeout every 2 s
            return
        self._consecutive = 0
        t = time.monotonic()
        stamp = None
        self._polls += 1
        if self._polls % self.stamp_every == 0:
            s = self.link.read(self.node, *OBJ_STAMP, timeout=0.05)
            stamp = None if s is None else s & 0xFFFF
        if not self.tracker.accept(stamp, t):
            return
        self.samples += 1
        self.on_sample(ImuSample(t, gyro_z_rad_s(raw, self.sign), stamp))
