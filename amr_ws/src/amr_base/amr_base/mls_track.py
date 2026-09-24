"""The track (line) reading of the SICK MLS (CAN node 10), for /amr/line_track.

Line-follow plan §1.1. Read-only: nothing here writes an object or sends NMT.
Decoding is drivers/canbus/read_mls.py's (TPDO1 layout, Standard/Combi, #LCP).

Acquisition, chosen by `mode`:

  tpdo  listen on TPDO1 (0x180+node). Frames only flow while the sensor is
        Operational; in Pre-operational this mode sees nothing (stale).
  sdo   read the TPDO1 fields over SDO (2021h:01..04, 2022h), ONE read per
        bus tick so the drive setpoint is never held up; a sample completes
        every five reads (10 Hz at most on a 50 Hz loop). Good for the
        Monitor, too slow and too skewed for control.
  auto  tpdo, falling back to sdo polling when no TPDO1 frame arrived within
        `tpdo_wait_s`; the first TPDO1 frame switches back to tpdo.
  off   nothing.

Whatever the mode, start() reads the variant (2006h:01), the TPDO1 COB-ID
(1800h:01) and one live SDO sample, so a restart alone shows whether a tape is
under the sensor. Every sample carries its `source`; the line follower accepts
only fresh ones, which the SDO path cannot deliver at control rate.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import amr_base.agv_repo  # noqa: F401  (puts the repo layer dirs on sys.path)

import read_mls  # repo module

MODES = ("tpdo", "sdo", "auto", "off")
SDO_ITEMS = (
    (read_mls.OBJ_LCP, 1),
    (read_mls.OBJ_LCP, 2),
    (read_mls.OBJ_LCP, 3),
    read_mls.OBJ_NLCP,
    read_mls.OBJ_STATUS,
)

OK, WARN, ERROR = "ok", "warn", "error"


@dataclass
class TrackSample:
    t_mono: float
    reading: dict  # read_mls.decode_tpdo1's dict
    source: str  # "tpdo" | "sdo"

    @property
    def lcp_mm(self) -> list[int]:
        return [p for p, _w in self.reading["lcp"]]

    @property
    def nlcp(self) -> int:
        return self.reading["nlcp"]


@dataclass
class _SdoCycle:
    index: int = 0
    values: list = field(default_factory=list)


class MlsTrack:
    """Acquisition state for one MLS. Bus-thread only; `on_sample` is called
    from within the bus loop with every decoded sample."""

    MISSES_BEFORE_BACKOFF = 5

    def __init__(
        self,
        link,
        node: int,
        on_sample,
        mode: str = "auto",
        sdo_hz: float = 10.0,
        tpdo_wait_s: float = 1.0,
        stale_tpdo_s: float = 0.1,
        expected_variant: int | None = 0,
        clock=time.monotonic,
        log=print,
    ):
        if mode not in MODES:
            raise ValueError(f"mls track mode {mode!r} not in {MODES}")
        self.link, self.node, self.on_sample, self.log, self.clock = link, node, on_sample, log, clock
        self.want = mode
        self.mode = "off"
        self.sdo_period = 1.0 / max(0.1, sdo_hz)
        self.tpdo_wait_s = tpdo_wait_s
        self.stale_tpdo_s = stale_tpdo_s
        self.expected_variant = expected_variant
        self.cob_id = read_mls.TPDO1_COB + node
        self.variant: int | None = None
        self.combi = False
        self.tpdo_enabled: bool | None = None
        self.last: TrackSample | None = None
        self.samples = 0
        self.tpdo_frames = 0
        self.misses = 0
        self.stale_periods = 0
        self.variant_mismatches = 0
        self._stale = True
        self._t_start = 0.0
        self._next_cycle = 0.0
        self._cycle = _SdoCycle()
        self._consecutive = 0

    # -- start --

    def start(self) -> str:
        self._t_start = self.clock()
        if self.want == "off":
            self.mode = "off"
            return self.mode
        v = self.link.read(self.node, *read_mls.OBJ_VARIANT)
        if v is None:
            self.log(f"MLS node {self.node} did not answer 2006h:01 - track reading off")
            self.mode = "off"
            return self.mode
        self.variant, self.combi = v, v in read_mls.COMBI_VARIANTS
        if self.expected_variant is not None and v != self.expected_variant:
            self.variant_mismatches += 1
            self.log(
                f"MLS 2006h:01 variant {v} ({read_mls.VARIANT_NAMES.get(v, '?')}), "
                f"commissioned {self.expected_variant}: decoding as reported"
            )
        raw = self.link.read(self.node, read_mls.TPDO1_COMM, 1)
        self.tpdo_enabled = raw is not None and not (raw & read_mls.COB_DISABLED)
        if self.tpdo_enabled and (raw & 0x7FF) != self.cob_id:
            self.log(f"MLS TPDO1 COB-ID 0x{raw & 0x7FF:03X}, not 0x{self.cob_id:03X}: listening there")
            self.cob_id = raw & 0x7FF
        if self.want in ("tpdo", "auto") and self.tpdo_enabled:
            self.link.router.add(self.cob_id, self._on_tpdo)
        self._snapshot()
        if self.want == "sdo" or (self.want == "auto" and not self.tpdo_enabled):
            self.mode = "sdo"
        elif self.tpdo_enabled:
            self.mode = "tpdo"  # auto: falls back in poll() if nothing arrives
        else:
            self.mode = "off"
            self.log("MLS TPDO1 (1800h:01) is disabled - track reading off (mode tpdo)")
        self.log(
            f"MLS track: variant {v} ({'Combi' if self.combi else 'Standard'}), TPDO1 "
            f"{f'0x{self.cob_id:03X}' if self.tpdo_enabled else 'disabled'}, mode {self.mode}"
        )
        return self.mode

    def _snapshot(self) -> None:
        vals = [self.link.read(self.node, i, s) for i, s in SDO_ITEMS]
        if any(x is None for x in vals):
            self.log("MLS track: start snapshot incomplete (SDO)")
            return
        r = read_mls.decode_sdo(vals[:3], vals[3], vals[4], self.combi)
        self.log(f"MLS track at start (SDO): #LCP {r['nlcp']} {r['nlcp_label']}; {read_mls.fmt_reading(r)}")
        self._emit(r, "sdo", self.clock())

    # -- samples --

    def _emit(self, reading: dict, source: str, t: float) -> None:
        self.last = TrackSample(t, reading, source)
        self.samples += 1
        self._stale = False
        self.on_sample(self.last)

    def _on_tpdo(self, m) -> None:
        r = read_mls.decode_tpdo1(bytes(m.data), self.combi)
        if r is None:
            return
        self.tpdo_frames += 1
        if self.mode != "tpdo":
            self.log(f"MLS TPDO1 frames on 0x{self.cob_id:03X}: track reading by TPDO")
            self.mode = "tpdo"
            self._cycle = _SdoCycle()
        self._emit(r, "tpdo", self.clock())

    def stale_after(self) -> float:
        return self.stale_tpdo_s if self.mode == "tpdo" else 3.0 * self.sdo_period

    def age(self, now: float) -> float | None:
        return None if self.last is None else now - self.last.t_mono

    def poll(self, now: float) -> None:
        """Once per bus tick: stale bookkeeping, auto fallback, at most ONE SDO read."""
        a = self.age(now)
        if (a is None or a > self.stale_after()) and not self._stale:
            self._stale = True
            self.stale_periods += 1
        if (
            self.mode == "tpdo"
            and self.want == "auto"
            and self.tpdo_frames == 0
            and now - self._t_start >= self.tpdo_wait_s
        ):
            self.log(
                f"MLS: no TPDO1 on 0x{self.cob_id:03X} in {self.tpdo_wait_s:.1f} s "
                "(Pre-operational?): polling by SDO"
            )
            self.mode = "sdo"
        if self.mode != "sdo":
            return
        c = self._cycle
        if c.index == 0:
            if now < self._next_cycle:
                return
            self._next_cycle = now + self.sdo_period
        index, sub = SDO_ITEMS[c.index]
        v = self.link.read(self.node, index, sub, timeout=0.05)
        if v is None:
            self.misses += 1
            self._consecutive += 1
            self._cycle = _SdoCycle()
            if self._consecutive >= self.MISSES_BEFORE_BACKOFF:
                self._next_cycle = now + 2.0
            return
        self._consecutive = 0
        c.values.append(v)
        c.index += 1
        if c.index == len(SDO_ITEMS):
            self._cycle = _SdoCycle()
            vals = c.values
            self._emit(read_mls.decode_sdo(vals[:3], vals[3], vals[4], self.combi), "sdo", self.clock())

    # -- diagnostics (pure; drive_node wraps it in a DiagnosticStatus) --

    def _variant_text(self) -> str:
        if self.variant is None:
            return ""
        return f"{self.variant} {read_mls.VARIANT_NAMES.get(self.variant, '?')}"

    def diagnostic(self, now: float) -> tuple[str, str, list[tuple[str, str]]]:
        a = self.age(now)
        s = self.last
        kv = [
            ("mode", self.mode),
            ("source", s.source if s else ""),
            ("nlcp", f"{s.nlcp} {s.reading['nlcp_label']}" if s else ""),
        ]
        for i in range(3):
            kv.append((f"lcp{i + 1}_mm", str(s.lcp_mm[i]) if s and (i + 1) in s.reading["valid"] else ""))
        st = s.reading["status"] if s else {}
        kv += [
            ("line_good", str(st.get("line_good", ""))),
            ("track_level", str(st.get("track_level", ""))),
            ("polarity", st.get("polarity", "")),
            ("marker", str(s.reading["marker"]["code"]) if s else ""),
            ("age_ms", "" if a is None else f"{a * 1000.0:.0f}"),
            ("samples", str(self.samples)),
            ("tpdo_frames", str(self.tpdo_frames)),
            ("stale_periods", str(self.stale_periods)),
            ("sdo_misses", str(self.misses)),
            ("variant", self._variant_text()),
            ("variant_mismatches", str(self.variant_mismatches)),
        ]
        if self.mode == "off":
            return ERROR, "off", kv
        if a is None or a > self.stale_after():
            return ERROR, "stale", kv
        if s.nlcp == 0:
            return WARN, "no track", kv
        return OK, s.reading["nlcp_label"], kv
