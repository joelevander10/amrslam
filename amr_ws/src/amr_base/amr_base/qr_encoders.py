"""AMR QR wheel encoders: two CiA 406 absolute encoders on the wheel axles (qr_analog platform).

No rclpy here; the bus is injected (canopen.Router over python-can), so everything is
testable against a scripted bus.

    node can.left_node / can.right_node, object 6004h Position value (u32, 24-bit range)
    TPDO1 0x180+n: 6004h mapped (EDS default 1A00h:1 = 0x60040020), transmission type 254

Two acquisition paths, chosen by qr_base.enc_mode:

    tpdo : at start, 1800h:05 event timer = enc_event_ms and 1800h:03 inhibit = the same
           period (inhibit equal to the event timer: a changing value must not push the
           frame faster than the period - drive_node learnt this the hard way on 2026-09-16),
           then NMT start. The encoder pushes the position every period.
    sdo  : 6004h read by SDO from the owning thread, both nodes every poll. What the QR
           controller did (trackless_module.py), ~2 ms per read at 125 kbps.
    auto : tpdo, and an SDO poll for any node whose TPDO has not been seen for 3 periods.

Units and signs are applied ONCE, here:
    * the position is unwrapped (delta modulo the wrap range, trusted only below half a
      range) - the QR controller subtracted raw values, which jumps by a whole range at the
      wrap. The range is read from each encoder's 6002h at start when qr_base.enc_range_counts
      is 0 (auto): on the AMR QR the two units report different ranges (the right one sat
      at 27.6 M, beyond the 24 bits the EDS default suggests);
    * qr_base.enc_invert_* flips a wheel so that vehicle-forward is positive;
    * counts -> wheel radians with enc_counts_per_rev (on the axle: no gearbox term).
"""

from __future__ import annotations

import math
import struct
import time
from collections import deque
from dataclasses import dataclass, field

import can

import amr_base.agv_repo  # noqa: F401  (puts the repo layer dirs on sys.path)

import guard  # repo module: every write passes the same deny-list the drives use
from drive_forward import sdo_write  # noqa: E402
from verify_drivers import sdo_read, u32  # noqa: E402

POSITION = 0x6004
UNITS_PER_REV = 0x6001  # CiA 406 measuring units per revolution
TOTAL_RANGE = 0x6002  # CiA 406 total measuring range (where the position wraps)
FALLBACK_RANGE = 1 << 32  # a plain 32-bit counter
TPDO1_COMM = 0x1800
NMT_START = 0x01
SUB_INHIBIT, SUB_EVENT = 3, 5

TPDO, SDO = "tpdo", "sdo"


class Unwrapper:
    """Raw modulo-range position -> continuous counts. Pure."""

    def __init__(self, range_counts: int) -> None:
        if range_counts < 2:
            raise ValueError("range_counts must be >= 2")
        self.range = int(range_counts)
        self.last_raw: int | None = None
        self.total = 0
        self.rejected = 0

    def reset(self) -> None:
        self.last_raw = None
        self.total = 0

    def update(self, raw: int) -> int | None:
        """Continuous count for this raw reading, or None if the reading is not usable.

        The first reading is the baseline (continuous 0). A step of half a range or more
        cannot be told apart from a step the other way round, so it is rejected and the
        chain restarts from this reading (the caller sees None once and a new baseline).
        """
        if not 0 <= raw < self.range:
            self.rejected += 1
            return None
        if self.last_raw is None:
            self.last_raw = raw
            return self.total
        d = (raw - self.last_raw) % self.range
        if d > self.range // 2:
            d -= self.range
        if abs(d) >= self.range // 2:
            self.rejected += 1
            self.last_raw = raw
            return None
        self.last_raw = raw
        self.total += d
        return self.total


class SpeedEstimator:
    """Wheel speed from timestamped continuous counts over a short window. Pure."""

    def __init__(self, window_s: float = 0.06, max_samples: int = 32) -> None:
        self.window_s = window_s
        self._s: deque[tuple[float, int]] = deque(maxlen=max_samples)

    def reset(self) -> None:
        self._s.clear()

    def add(self, t: float, counts: int) -> None:
        if self._s and t <= self._s[-1][0]:
            return  # a reordered or duplicated sample adds nothing
        self._s.append((t, counts))
        while len(self._s) > 2 and self._s[-1][0] - self._s[1][0] >= self.window_s:
            self._s.popleft()

    def counts_per_s(self) -> float | None:
        if len(self._s) < 2:
            return None
        (t0, c0), (t1, c1) = self._s[0], self._s[-1]
        dt = t1 - t0
        return (c1 - c0) / dt if dt > 0 else None


@dataclass
class WheelEncoder:
    """One wheel: raw frames in, vehicle-signed continuous counts / radians / speed out."""

    node: int
    counts_per_rev: float
    range_counts: int
    invert: bool = False
    window_s: float = 0.06
    unwrap: Unwrapper = field(init=False)
    speed: SpeedEstimator = field(init=False)
    t: float | None = None  # monotonic time of the last accepted reading
    t_tpdo: float | None = None  # ... that arrived by TPDO
    counts: int | None = None  # continuous, vehicle-signed
    raw: int | None = None
    source: str = ""
    frames: int = 0

    def __post_init__(self) -> None:
        self.unwrap = Unwrapper(self.range_counts)
        self.speed = SpeedEstimator(self.window_s)

    def set_range(self, range_counts: int) -> None:
        """New wrap range: the continuous count restarts from the next reading."""
        self.range_counts = int(range_counts)
        self.unwrap = Unwrapper(self.range_counts)
        self.speed.reset()
        self.counts = None

    def feed(self, raw: int, t: float, source: str) -> None:
        c = self.unwrap.update(int(raw))
        self.raw = int(raw)
        if c is None:
            # unusable reading: restart speed estimation from the next good one
            self.speed.reset()
            self.counts = None
            return
        c = -c if self.invert else c
        self.counts, self.t, self.source = c, t, source
        self.frames += 1
        if source == TPDO:
            self.t_tpdo = t
        self.speed.add(t, c)

    def fresh(self, now: float, timeout: float) -> bool:
        return self.t is not None and self.counts is not None and now - self.t <= timeout

    @property
    def rad(self) -> float | None:
        return None if self.counts is None else self.counts * 2.0 * math.pi / self.counts_per_rev

    @property
    def rad_s(self) -> float | None:
        cps = self.speed.counts_per_s()
        return None if cps is None else cps * 2.0 * math.pi / self.counts_per_rev


def range_from_6002(value: int | None) -> int | None:
    """Wrap range from a 6002h reading, or None if the reading is unusable.

    CiA 406 defines 6002h as the NUMBER of measuring steps (positions 0..value-1), but
    many units report the largest position instead (the EDS default 0x00FFFFFF). An
    all-ones value is therefore read as a power-of-two range; guessing wrong costs one
    count at the wrap, nothing else.
    """
    if value is None or value < 2:
        return None
    return value + 1 if (value + 1) & value == 0 else value


def decode_position(data: bytes) -> int | None:
    """TPDO1 / SDO payload -> raw position (u32 little-endian), or None if too short."""
    if len(data) < 4:
        return None
    return struct.unpack_from("<I", bytes(data), 0)[0]


class EncoderPair:
    """Both wheel encoders on one bus, owned by one thread (the caller's)."""

    def __init__(
        self,
        router,
        left_node: int,
        right_node: int,
        counts_per_rev: float,
        range_counts: int,
        invert_left: bool,
        invert_right: bool,
        mode: str = "auto",
        event_ms: int = 20,
        sdo_timeout_s: float = 0.02,
        log=print,
        clock=time.monotonic,
    ) -> None:
        if mode not in ("auto", TPDO, SDO):
            raise ValueError(f"enc_mode {mode!r}")
        self.router = router
        self.mode = mode
        self.event_ms = int(event_ms)
        self.sdo_timeout = sdo_timeout_s
        self.log = log
        self.clock = clock
        self.auto_range = int(range_counts) == 0
        self.counts_per_rev = counts_per_rev
        start_range = FALLBACK_RANGE if self.auto_range else int(range_counts)
        self.left = WheelEncoder(left_node, counts_per_rev, start_range, invert_left)
        self.right = WheelEncoder(right_node, counts_per_rev, start_range, invert_right)
        self.info: dict[int, dict] = {}  # node -> {"units_per_rev", "range_6002", "range"}
        self.sdo_reads = 0
        self.sdo_timeouts = 0
        self.tpdo_configured: dict[int, bool] = {}
        self.status = "starting"
        for enc in (self.left, self.right):
            router.add(0x180 + enc.node, self._tpdo_handler(enc))

    @property
    def wheels(self) -> tuple[WheelEncoder, WheelEncoder]:
        return self.left, self.right

    def _tpdo_handler(self, enc: WheelEncoder):
        def handle(msg) -> None:
            raw = decode_position(msg.data)
            if raw is not None:
                enc.feed(raw, self.clock(), TPDO)

        return handle

    # ---- setup ----

    def start(self) -> None:
        """Read each encoder's range, configure TPDO1 (tpdo/auto), start them.

        Never raises in auto mode."""
        for enc in self.wheels:
            self._identify(enc)
        for enc in self.wheels:
            ok = True
            if self.mode in ("auto", TPDO):
                ok = self._configure_tpdo(enc.node)
                self.tpdo_configured[enc.node] = ok
            if not ok and self.mode == TPDO:
                raise RuntimeError(f"encoder node {enc.node}: TPDO1 configuration failed")
        # NMT start both (TPDOs only flow in Operational; SDO works in pre-op too)
        for enc in self.wheels:
            self.router.send(
                can.Message(arbitration_id=0x000, data=[NMT_START, enc.node], is_extended_id=False)
            )
        self.status = self.mode

    def _read_u32(self, node: int, index: int) -> int | None:
        status, payload, _note, _ms = sdo_read(self.router, node, index, 0, timeout=0.3, collision_window=0)
        return u32(payload) if status is True and payload is not None else None

    def _identify(self, enc: WheelEncoder) -> None:
        """6001h / 6002h of one encoder; with auto range, 6002h sets the wrap."""
        upr = self._read_u32(enc.node, UNITS_PER_REV)
        tmr = self._read_u32(enc.node, TOTAL_RANGE)
        used = enc.range_counts
        if self.auto_range:
            r = range_from_6002(tmr)
            if r is None or r < 2 * self.counts_per_rev:
                self.log(
                    f"encoder node {enc.node}: 6002h unreadable or too small ({tmr}); "
                    f"unwrapping as a 32-bit counter"
                )
                r = FALLBACK_RANGE
            enc.set_range(r)
            used = r
        elif tmr is not None and range_from_6002(tmr) != used:
            self.log(
                f"encoder node {enc.node}: 6002h says range {range_from_6002(tmr)} but the profile "
                f"sets enc_range_counts {used} - set it to 0 (auto) unless you know better"
            )
        if upr is not None and abs(upr - self.counts_per_rev) > 1:
            self.log(
                f"encoder node {enc.node}: 6001h = {upr} units/rev, profile enc_counts_per_rev "
                f"{self.counts_per_rev:g} (the profile value is what odometry uses)"
            )
        self.info[enc.node] = {"units_per_rev": upr, "range_6002": tmr, "range": used}
        self.log(f"encoder node {enc.node}: 6001h {upr}, 6002h {tmr} -> wrap range {used}")

    def _configure_tpdo(self, node: int) -> bool:
        period_100us = self.event_ms * 10
        for sub, value, what in (
            (SUB_INHIBIT, period_100us, "inhibit time"),
            (SUB_EVENT, self.event_ms, "event timer"),
        ):
            guard.check(TPDO1_COMM, value, sub)  # comm params are admitted by rule
            ok, detail = sdo_write(self.router, node, TPDO1_COMM, sub, value, 2, timeout=0.3)
            if not ok:
                self.log(f"encoder node {node}: 1800h:{sub:02X} {what} <- {value} failed ({detail})")
                return False
        self.log(f"encoder node {node}: TPDO1 every {self.event_ms} ms")
        return True

    # ---- per tick ----

    def poll(self, now: float | None = None) -> None:
        """SDO-read any wheel that needs it (sdo mode, or auto without a recent TPDO)."""
        now = self.clock() if now is None else now
        stale_after = 3.0 * self.event_ms / 1000.0
        for enc in self.wheels:
            if self.mode == TPDO:
                continue
            if self.mode == "auto" and enc.t_tpdo is not None and now - enc.t_tpdo <= stale_after:
                continue
            self.sdo_reads += 1
            status, payload, _note, _ms = sdo_read(
                self.router, enc.node, POSITION, 0, timeout=self.sdo_timeout, collision_window=0
            )
            if status is True and payload is not None:
                enc.feed(u32(payload), self.clock(), SDO)
            elif status is None:
                self.sdo_timeouts += 1

    def source_summary(self) -> str:
        return "/".join(enc.source or "-" for enc in self.wheels)
