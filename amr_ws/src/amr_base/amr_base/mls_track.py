"""The track (line) reading of the SICK MLS (CAN node 10), for /amr/line_track.

Line-follow plan §1.1. Writes no configuration to the sensor; it sends NMT Start
addressed to the MLS node, so TPDO1 flows (see _nmt_start), and - only when the
profile sets mls.event_clear - the reset of the latched event flag (2080h:01,
through guard.check_sensor; manuals/mls-diagnostics-plan.md E2).
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
under the sensor.

Marker configuration (mls-marker-plan 5.2, 2026-10-09). When the profile turns
markers on, the sensor's marker objects (2028h:01-03, 2029h, 202Dh:05) are READ
and compared with what the profile expects - never written. Not in start(): start()
also runs from _rediscover() inside poll() while the drives may be moving, and its
reads block up to 0.4 s each. The check is a queue drained ONE read per poll, with
the short probe timeout, and only in tpdo mode (sdo mode already spends its read).
It runs after every start, again whenever the TPDO stream returns after a gap (a
sensor reboot never reaches _rediscover), and every MARKER_RECHECK_S.
`markers_ok` is true only when the check passed and the stream is TPDO.

SDO jobs (mls-diagnostics-plan E4). The marker check, the event cause and the
temperature share ONE queue: one SDO operation per poll, TPDO mode only, the short
probe timeout. A job that times out is dropped and retried later; it never blocks
the others for longer than that one read.

Event detection (mls-diagnostics-plan M1/M2, manual 8.8). A rising event flag
(TPDO1 status bit 7) queues the cause reads 2080h:02/:04/:05. A magnetic-field
event is reported through `on_event` only if a track was under the sensor on the
sample before (E5: leaving the tape is not a weak tape); otherwise it is counted
as `events_no_track`. The flag latches, so with mls.event_clear set the job then
writes 2080h:01; a flag still high CLEAR_MIN_S after a clear is a new event. A
failed or refused clear turns the clearing off for the rest of the run. Without
the clear only the first event after the flag was last reset is seen.
The temperature (2070h:01 and its thresholds :06/:07) is read every TEMP_PERIOD_S
and at once after a temperature event; `temp` holds the last decode.

Every sample carries its `source`; the line follower accepts only fresh ones,
which the SDO path cannot deliver at control rate.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from agv_core.drivers.canbus import read_mls  # repo module

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


class RateLimit:
    """At most one report per `min_s`; the ones in between are counted into the next.
    Pure and clock-fed: allow(now) -> None (hold it back) or the number held back
    since the last report."""

    def __init__(self, min_s: float):
        self.min_s, self._t, self._held = min_s, None, 0

    def allow(self, now: float) -> int | None:
        if self._t is not None and now - self._t < self.min_s:
            self._held += 1
            return None
        held, self._t, self._held = self._held, now, 0
        return held


@dataclass
class _Job:
    """One SDO job: read `keys` in order, then `done(vals, now)`; then, if set, the
    write (index, sub, value, size) and `wrote(ok, detail, now)`."""

    purpose: str
    keys: list
    done: object
    write: tuple | None = None
    wrote: object = None
    vals: dict = field(default_factory=dict)
    failed: object = None  # failed(key, now) when a read gets no answer


@dataclass
class _SdoCycle:
    index: int = 0
    values: list = field(default_factory=list)


class MlsTrack:
    """Acquisition state for one MLS. Bus-thread only; `on_sample` is called
    from within the bus loop with every decoded sample."""

    MISSES_BEFORE_BACKOFF = 5
    # A sensor that did not answer at start (or whose TPDO1 configuration
    # could not be read) is probed again this often with ONE short SDO
    # timeout, so a late-booting MLS is acquired without restarting the bus
    # owner and without starving the wheel loop.
    REDISCOVER_S = 5.0
    PROBE_TIMEOUT_S = 0.05
    MARKER_RECHECK_S = 60.0
    MARKER_RETRY_S = 5.0
    TEMP_FIRST_S = 10.0  # the first temperature read, after start (the marker check goes first)
    TEMP_PERIOD_S = 60.0
    EVENT_RETRY_S = 5.0  # a flag whose cause could not be read is tried again this often
    CLEAR_MIN_S = 1.0  # a flag still high this long after a clear is a new event

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
        markers: dict | None = None,
        on_event=None,
        event_clear: int | None = None,
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
        self.nmt_starts = 0
        self.restarts = 0  # rediscoveries after an incomplete start (F08)
        self._rediscover_at: float | None = None
        self._next_nmt = 0.0
        self._last_tpdo_t: float | None = None
        self.variant_mismatches = 0
        self.start_count = 0  # every start() is a new marker generation
        # marker configuration check: {(index, sub): wanted value} or None (markers off)
        self.marker_expect = dict(markers) if markers else None
        self.marker_status = "off" if not markers else "checking"
        self.marker_checks = 0
        self._jobs: list[_Job] = []
        self._mnext: float | None = None
        self._mrearm = False
        # event detection and temperature (mls-diagnostics-plan)
        self.on_event = on_event
        self.event_clear = event_clear if event_clear in (0, 1) else None
        self.clear_state = "off" if self.event_clear is None else "on"
        self.events_field = 0
        self.events_no_track = 0
        self.events_temp = 0
        self.events_other = 0  # flag set, source neither field nor temperature
        self.events_unread = 0
        self.clears = 0
        self.temp: dict | None = None
        self.temp_unread = 0
        self.track_level_min: int | None = None
        self._temp_next: float | None = None
        self._ev_prev_flag: bool | None = None
        self._ev_prev_nlcp: int | None = None
        self._ev_retry_at: float | None = None
        self._ev_cleared_at: float | None = None
        self._stale = True
        self._t_start = 0.0
        self._next_cycle = 0.0
        self._cycle = _SdoCycle()
        self._consecutive = 0

    # -- start --

    def start(self) -> str:
        self._t_start = self.clock()
        self._rediscover_at = None
        self.start_count += 1
        if self.marker_expect:
            self._arm_marker_check(self._t_start, reset=True)
        self._ev_prev_flag = None  # a (re)started sensor: a flag already up is an event
        self._temp_next = self._t_start + self.TEMP_FIRST_S
        if self.want == "off":
            self.mode = "off"
            return self.mode
        v = self.link.read(self.node, *read_mls.OBJ_VARIANT)
        if v is None:
            self.log(f"MLS node {self.node} did not answer 2006h:01 - track reading off, probing again")
            self.mode = "off"
            self._rediscover_at = self._t_start + self.REDISCOVER_S
            return self.mode
        self.variant, self.combi = v, v in read_mls.COMBI_VARIANTS
        if self.expected_variant is not None and v != self.expected_variant:
            self.variant_mismatches += 1
            self.log(
                f"MLS 2006h:01 variant {v} ({read_mls.VARIANT_NAMES.get(v, '?')}), "
                f"commissioned {self.expected_variant}: decoding as reported"
            )
        raw = self.link.read(self.node, read_mls.TPDO1_COMM, 1)
        if raw is None:
            # One failed read must not pin the reader to SDO for the rest of
            # the run: try the whole discovery again later.
            self.log("MLS 1800h:01 did not answer: TPDO1 unknown, rediscovering later")
            self._rediscover_at = self._t_start + self.REDISCOVER_S
        self.tpdo_enabled = raw is not None and not (raw & read_mls.COB_DISABLED)
        if self.tpdo_enabled and (raw & 0x7FF) != self.cob_id:
            self.log(f"MLS TPDO1 COB-ID 0x{raw & 0x7FF:03X}, not 0x{self.cob_id:03X}: listening there")
            self.cob_id = raw & 0x7FF
        if self.want in ("tpdo", "auto") and self.tpdo_enabled:
            self.link.router.add(self.cob_id, self._on_tpdo)
            self._nmt_start(self._t_start)
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
        self._last_tpdo_t = self.clock()
        if self._mrearm:
            # The stream is back after a gap: the sensor may have rebooted (to other
            # settings). Untrusted until re-read.
            self._mrearm = False
            if self.marker_expect:
                self._arm_marker_check(self._last_tpdo_t, reset=True)
            self._ev_prev_flag = None
        if self.mode != "tpdo":
            self.log(f"MLS TPDO1 frames on 0x{self.cob_id:03X}: track reading by TPDO")
            self.mode = "tpdo"
            self._cycle = _SdoCycle()
        self._event_watch(r, self._last_tpdo_t)
        self._emit(r, "tpdo", self.clock())

    def _nmt_start(self, now: float) -> None:
        """NMT Start addressed to the MLS only. TPDO1 flows in Operational and
        nothing else on this bus puts node 10 there: the sensor boots
        Operational but the drive link's Pre-operational at arm used to take
        it down (2026-09-21). The drives are not addressed. Rate-limited so a
        sensor that will not start is asked every 2 s, not every bus tick."""
        if now < self._next_nmt:
            return
        self._next_nmt = now + 2.0
        self.nmt_starts += 1
        self.link.nmt(0x01, self.node)

    def stale_after(self) -> float:
        return self.stale_tpdo_s if self.mode == "tpdo" else 3.0 * self.sdo_period

    def age(self, now: float) -> float | None:
        return None if self.last is None else now - self.last.t_mono

    def _rediscover(self, now: float) -> bool:
        """Probe an absent/partly-read sensor; a full start() when it answers."""
        if self._rediscover_at is None or now < self._rediscover_at or self.want == "off":
            return False
        self._rediscover_at = now + self.REDISCOVER_S
        if self.link.read(self.node, *read_mls.OBJ_VARIANT, timeout=self.PROBE_TIMEOUT_S) is None:
            return True
        self.restarts += 1
        self.start()
        return True

    def poll(self, now: float) -> None:
        """Once per bus tick: stale bookkeeping, auto fallback, at most ONE SDO read."""
        if self._rediscover(now):
            return
        a = self.age(now)
        if (a is None or a > self.stale_after()) and not self._stale:
            self._stale = True
            self.stale_periods += 1
        want_tpdo = self.want in ("tpdo", "auto") and self.tpdo_enabled and self.mode != "off"
        no_stream = self._last_tpdo_t is None or now - self._last_tpdo_t > self.tpdo_wait_s
        if want_tpdo and no_stream and now - self._t_start >= self.tpdo_wait_s:
            # The stream never came, or died (the sensor was put back to
            # Pre-operational): ask node 10 to start, again and again, and in
            # auto mode poll by SDO meanwhile so the reading is not lost. The
            # first TPDO1 frame switches back (see _on_tpdo).
            self._nmt_start(now)
            self._mrearm = True
            if self.mode == "tpdo" and self.want == "auto":
                self.log(
                    f"MLS: no TPDO1 on 0x{self.cob_id:03X} for {self.tpdo_wait_s:.1f} s "
                    "(Pre-operational?): polling by SDO"
                )
                self.mode = "sdo"
        if self.mode == "tpdo":
            self._sdo_poll(now)
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

    # -- SDO jobs: one queue, one operation per poll (E4) --

    def _pending(self, purpose: str) -> bool:
        return any(j.purpose == purpose for j in self._jobs)

    def _schedule(self, now: float) -> None:
        recheck = self._mnext is not None and now >= self._mnext
        if self.marker_expect and recheck and not self._pending("marker"):
            self._arm_marker_check(now, reset=False)  # periodic: keep the verdict meanwhile
        if self._temp_next is not None and now >= self._temp_next and not self._pending("temp"):
            self._temp_next = now + self.TEMP_PERIOD_S
            self._jobs.append(_Job("temp", list(read_mls.TEMP_READS), self._temp_done,
                                   failed=self._temp_failed))

    def _sdo_poll(self, now: float) -> None:
        """At most ONE short SDO operation: the next read (or write) of the first job."""
        self._schedule(now)
        if not self._jobs:
            return
        job = self._jobs[0]
        if job.keys:
            key = job.keys[0]
            v = self.link.read(self.node, *key, timeout=self.PROBE_TIMEOUT_S)
            if v is None:
                self._jobs.pop(0)
                if job.failed:
                    job.failed(key, now)
                return
            job.vals[key] = v
            job.keys.pop(0)
            if not job.keys:
                job.done(job.vals, now)
                if job.write is None:
                    self._jobs.pop(0)
            return
        self._jobs.pop(0)
        index, sub, value, size = job.write
        try:
            ok, detail = self.link.write_sensor(self.node, index, sub, value, size,
                                                timeout=self.PROBE_TIMEOUT_S)
        except Exception as e:  # noqa: BLE001 - ForbiddenWrite, or a link with no sensor write
            ok, detail = False, repr(e)
        job.wrote(ok, detail, now)

    # -- marker configuration (read-only) --

    def _arm_marker_check(self, now: float, reset: bool) -> None:
        self._jobs = [j for j in self._jobs if j.purpose != "marker"]
        self._jobs.append(_Job("marker", list(self.marker_expect), self._marker_done,
                               failed=self._marker_failed))
        self._mnext = None
        if reset:
            self.marker_status = "checking"

    def _marker_failed(self, key, now: float) -> None:
        self._mnext = now + self.MARKER_RETRY_S
        new = f"unverified: {key[0]:04X}h:{key[1]:02X} did not answer"
        if new != self.marker_status:
            self.log(f"MLS markers {new}; retrying in {self.MARKER_RETRY_S:.0f} s")
        self.marker_status = new

    def _marker_done(self, vals: dict, now: float) -> None:
        self.marker_checks += 1
        self._mnext = now + self.MARKER_RECHECK_S
        diffs = [
            f"{i:04X}h:{sb:02X} = {vals.get((i, sb))} (want {w})"
            for (i, sb), w in self.marker_expect.items()
            if vals.get((i, sb)) != w
        ]
        new = "ok" if not diffs else "misconfigured: " + "; ".join(diffs)
        if new != self.marker_status:
            if diffs:
                self.log(f"MLS markers {new} - no marker events. Fix with the service stopped: "
                         "python3 -m agv_core.drivers.canbus.read_mls set-markers --go")
            else:
                self.log("MLS markers: sensor configured as the profile expects")
        self.marker_status = new

    # -- event detection (mls-diagnostics-plan M1/M2) --

    def _event_watch(self, r: dict, now: float) -> None:
        """Every TPDO1 frame: queue the cause reads on a new event (see the module doc)."""
        st = r["status"]
        flag, nlcp = bool(st["event_flag"]), int(r["nlcp"])
        if nlcp:
            lvl = int(st["track_level"])
            self.track_level_min = lvl if self.track_level_min is None else min(self.track_level_min, lvl)
        prev_nlcp = nlcp if self._ev_prev_nlcp is None else self._ev_prev_nlcp
        new = False
        if flag and not self._pending("event"):
            if not self._ev_prev_flag:
                new = True  # rising, or already up when the stream (re)started
            elif self._ev_retry_at is not None and now >= self._ev_retry_at:
                new = True  # the last cause read got no answer
            elif self._ev_cleared_at is not None and now - self._ev_cleared_at >= self.CLEAR_MIN_S:
                new = True  # up again after a reset
        if new:
            self._ev_retry_at = self._ev_cleared_at = None
            lcp2 = r["lcp"][1][0] if 2 in r["valid"] else None
            ctx = {"t_mono": now, "track_before": prev_nlcp > 0, "nlcp": nlcp, "lcp2_mm": lcp2,
                   "track_level": int(st["track_level"])}
            write = None if self.event_clear is None else (*read_mls.EVENT_RESET, self.event_clear, 1)
            self._jobs.append(_Job(
                "event", list(read_mls.EVENT_READS),
                lambda vals, t, ctx=ctx: self._event_done(vals, t, ctx),
                write=write, wrote=self._clear_done, failed=self._event_failed,
            ))
        self._ev_prev_flag, self._ev_prev_nlcp = flag, nlcp

    def _event_failed(self, key, now: float) -> None:
        self.events_unread += 1
        self._ev_retry_at = now + self.EVENT_RETRY_S

    def _event_done(self, vals: dict, now: float, ctx: dict) -> None:
        ev = read_mls.decode_event(*(vals.get(k) for k in read_mls.EVENT_READS))
        if ev["temp"]:
            self.events_temp += 1
            self._temp_next = now  # read the temperature now, not in a minute
        if ev["field"]:
            if ctx["track_before"]:
                self.events_field += 1
                if self.on_event is not None:
                    self.on_event(dict(ev, **ctx))
            else:
                self.events_no_track += 1
        if not ev["field"] and not ev["temp"]:
            self.events_other += 1

    def _clear_done(self, ok: bool, detail: str, now: float) -> None:
        if ok:
            self.clears += 1
            self._ev_cleared_at = now
            return
        self.log(f"MLS event-flag reset 2080h:01 = {self.event_clear} failed ({detail}): "
                 "not clearing again this run, only the first event after this one is seen")
        self.event_clear = None
        self.clear_state = "failed"

    # -- temperature --

    def _temp_done(self, vals: dict, now: float) -> None:
        self.temp = read_mls.decode_temp(*(vals.get(k) for k in read_mls.TEMP_READS))

    def _temp_failed(self, key, now: float) -> None:
        self.temp_unread += 1

    @property
    def markers_ok(self) -> bool:
        return self.marker_status == "ok" and self.mode == "tpdo"

    def marker_status_text(self) -> str:
        """What /amr/line_marker's heartbeat says."""
        if not self.marker_expect:
            return "off"
        if self.mode == "sdo":
            return "no stream (sdo)"
        if self.mode == "off":
            return "no sensor"
        return self.marker_status

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
            ("nmt_starts", str(self.nmt_starts)),
            ("restarts", str(self.restarts)),
            ("sdo_misses", str(self.misses)),
            ("variant", self._variant_text()),
            ("variant_mismatches", str(self.variant_mismatches)),
            ("markers", self.marker_status_text()),
            ("marker_checks", str(self.marker_checks)),
            ("events_field", str(self.events_field)),
            ("events_no_track", str(self.events_no_track)),
            ("events_temp", str(self.events_temp)),
            ("events_unread", str(self.events_unread)),
            ("event_clear", self.clear_state if self.event_clear is None else f"on ({self.event_clear})"),
            ("event_clears", str(self.clears)),
            ("track_level_min", "" if self.track_level_min is None else str(self.track_level_min)),
            ("temp_c", "" if not self.temp or self.temp["temp_c"] is None else str(self.temp["temp_c"])),
            ("temp_range_c", "" if not self.temp else f"{self.temp['lo_c']}..{self.temp['hi_c']}"),
        ]
        if self.mode == "off":
            return ERROR, "off", kv
        if a is None or a > self.stale_after():
            return ERROR, "stale", kv
        if s.nlcp == 0:
            return WARN, "no track", kv
        return OK, s.reading["nlcp_label"], kv
