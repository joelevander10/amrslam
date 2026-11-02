"""MLS marker codes -> discrete marker events (manuals/mls-marker-plan.md, 2026-10-09).

Pure and clock-free: drive_node feeds every decoded MLS sample from the bus thread,
at TPDO rate, and publishes what comes back on /amr/line_marker. Done there and not
in the line follower because, with FailSafe on, the sensor may show a code for a
single 10 ms frame - the follower's 50 Hz tick and the best-effort /amr/line_track
could both miss it.

The SENSOR decodes the code (2028h standard mode). Nothing here reconstructs one
from track positions, and nothing guesses: when the sensor is not configured as the
profile expects (`ok` False), no sample produces an event.

  event   the reported code changes from 0 to c, or from c to another c' != 0. A code
          that stays is one event. FailSafe makes the reported code final, so one
          frame is enough - no debounce.
  reject  the same edge with a code outside mls.marker_codes: counted, never an event.
  abort   the "reading code" status bit (manual p. 38) went 1 -> 0 and no code was
          reported in between nor within ABORT_GRACE_FRAMES after (FailSafe may report
          the code a frame after the bit drops - the manual does not say): a marker the
          sensor started to read and could not decode. The early sign of a worn,
          off-centre or too-fast marker.

Only TPDO samples count. The SDO fallback (~10 Hz) samples carry the marker bits too
(2021h:04), but at that rate a one-frame code is missed silently, so they are ignored.
"""

from __future__ import annotations

from dataclasses import dataclass

ABORT_GRACE_FRAMES = 5  # 50 ms of TPDO1 at the 10 ms event timer


@dataclass(frozen=True)
class MarkerEvent:
    seq: int
    generation: int
    code: int
    raw: int
    direction: int  # 0 until the bench settles how the MLS reports it (plan D6)
    lcp2_mm: int
    nlcp: int
    line_good: bool
    t_mono: float


class MarkerDetector:
    """Edges of the MLS marker field. Not thread-safe: one feeder (the bus thread)."""

    def __init__(self, codes) -> None:
        self.codes = frozenset(int(c) for c in codes)
        self.seq = 0
        self.generation = 0
        self.rejects = 0
        self.aborts = 0
        self._prev_code: int | None = None  # None = no continuity (start, gap, not ok)
        self._reading = False
        self._coded = False  # a code was reported during the current reading episode
        self._pending = 0  # frames left before an uncoded episode counts as an abort

    def new_generation(self) -> None:
        """The stream restarted (bus owner, MLS rediscovered): continuity is broken."""
        self.generation += 1
        self.seq = 0
        self.break_continuity()

    def break_continuity(self) -> None:
        self._prev_code = None
        self._reading = False
        self._coded = False
        self._pending = 0

    def feed(self, t_mono: float, reading: dict, source: str, ok: bool) -> MarkerEvent | None:
        """One decoded sample (read_mls.decode_tpdo1's dict). Returns an event or None."""
        if not ok or source != "tpdo":
            # Not trusted: forget the edge state, so a code still showing when trust
            # returns is not taken for a fresh marker.
            self.break_continuity()
            return None
        mk, st = reading["marker"], reading["status"]
        code = int(mk["code"])
        reading_code = bool(st.get("reading_code", False))

        if code:
            self._coded, self._pending = True, 0
        if reading_code and not self._reading:
            # A new reading episode. One still waiting for its code never got one.
            if self._pending:
                self.aborts += 1
                self._pending = 0
            self._coded = bool(code)
        elif self._reading and not reading_code and not self._coded:
            self._pending = ABORT_GRACE_FRAMES
        elif self._pending and not code:
            self._pending -= 1
            if not self._pending:
                self.aborts += 1
        self._reading = reading_code

        prev, self._prev_code = self._prev_code, code
        if prev is None or code == 0 or code == prev:
            # prev None: the first trusted sample. A code already showing then is
            # mid-marker of unknown history - not reported.
            return None
        if code not in self.codes:
            self.rejects += 1
            return None
        self.seq += 1
        lcp2 = reading["lcp"][1][0] if 2 in reading["valid"] else 0
        return MarkerEvent(
            seq=self.seq,
            generation=self.generation,
            code=code,
            raw=int(mk.get("raw", 0)),
            direction=0,
            lcp2_mm=int(lcp2),
            nlcp=int(reading["nlcp"]),
            line_good=bool(st["line_good"]),
            t_mono=t_mono,
        )
