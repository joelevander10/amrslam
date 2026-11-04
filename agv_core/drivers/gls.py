"""SICK GLS621 floor-label camera: CoLa A telegrams in, one aggregated reading per image out.

The camera (192.168.4.1 on enp3s0) streams results on TCP 2112 and diagnostics on 2111,
CoLa A (ASCII) only. With the host template as saved on 2026-10-30

    [iec _GoodReadCond][f][$BC];[$XMCL];[$YMCL];[$AN2P];[$FAMS][e][e]

each decoded Data Matrix code of a GLS6 multi-code label arrives as one STX..ETX telegram

    A2x0000y0000;11;9;0.593822;946687420249
    ^^ position of the code in the label (row letter, column digit)
      ^^^^^^^^^^ the label ID: a NAME, never a coordinate
                 ^^ ^ XMCL / YMCL in device counts, ^ angle in degrees, ^ $FAMS (ms)

and every code read from the same image carries the same $FAMS, so the timestamp groups
telegrams into frames (manuals/gls6-qr/gls6-system.md §3.1, gls-wait-coding-plan §B.1).

ONE TELEGRAM MAY CARRY SEVERAL CODES. Measured on the vehicle 2026-10-30: about 3 % of the
telegrams hold two results back to back with no separator at all - the first one's $FAMS
runs straight into the next one's code position:

    C2x0000y0000;7;-33;0.584781;946692806114D2x0000y0000;5;-33;0.590747;946692806114

so parse() returns a LIST of code reads, and a parser that split on ';' into five fields
would silently drop those codes.

Pure: no config, no ROS, no socket, no clock of its own. split() frames bytes, parse()
classifies one telegram, FrameAggregator turns code reads into frames, and to_camera_pose()
converts a frame with caller-supplied scale and signs. Units and signs are NOT known yet
(bench tests, gls6-system.md §7): nothing here assumes millimetres or a direction.

The device sends nothing on a no-read. Absence of telegrams is the only no-read signal, so
a consumer needs its own staleness rule; FrameAggregator.flush() closes a frame on a gap.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

STX, ETX = 0x02, 0x03
# Longest telegram kept while waiting for its ETX. A result is ~45 bytes and the longest
# reply we read (OFtplstr1) under 100; anything far beyond that is a lost ETX, not a telegram.
MAX_TELEGRAM = 2048
REPLY_KINDS = ("sRA", "sWA", "sAN", "sEA", "sFA", "sMA")
# One result: position, label ID, XMCL, YMCL, angle, $FAMS. The label is matched lazily up to
# its ';', and $FAMS greedily up to the next code's position letter (see the module docstring).
RESULT_RE = re.compile(r"([A-Z][0-9])([A-Za-z0-9]+?);(-?\d+);(-?\d+);(-?\d+(?:\.\d+)?);(\d+)")


def split(buf: bytes) -> tuple[list[bytes], bytes, int]:
    """Complete telegrams (payload, STX/ETX stripped) in `buf`, the unfinished rest to keep
    for the next read, and the number of junk bytes dropped (outside any STX..ETX, or an
    overlong telegram whose ETX was lost). A second STX before an ETX restarts the telegram:
    the first one lost its ETX."""
    out: list[bytes] = []
    junk, i, n = 0, 0, len(buf)
    while i < n:
        s = buf.find(STX, i)
        if s < 0:
            junk += n - i
            return out, b"", junk
        junk += s - i
        e = buf.find(ETX, s + 1)
        nxt = buf.find(STX, s + 1)
        if e < 0:
            if nxt >= 0:  # a telegram whose ETX never came, followed by a new one
                junk += nxt - s
                i = nxt
                continue
            rest = buf[s:]
            if len(rest) > MAX_TELEGRAM:
                return out, b"", junk + len(rest)
            return out, rest, junk
        if 0 <= nxt < e:
            junk += nxt - s
            i = nxt
            continue
        out.append(buf[s + 1 : e])
        i = e + 1
    return out, b"", junk


@dataclass(frozen=True)
class CodeRead:
    """One decoded code of a multi-code label, raw device units."""

    pos: str  # "A1".."D4": where the code sits in the label
    label_id: str  # "x0000y0000"
    x: int  # XMCL, device counts
    y: int  # YMCL, device counts
    angle_deg: float  # $AN2P
    ts_ms: int  # $FAMS: identical for every code of one image


@dataclass(frozen=True)
class Reply:
    """A CoLa A answer to a command: sRA (read), sWA (write), sAN (method), sFA (error)..."""

    kind: str
    name: str
    value: str  # everything after the name, verbatim


def parse(payload: bytes) -> list[CodeRead] | Reply | None:
    """Classify one telegram: the code reads it carries (one or more, back to back), a CoLa
    reply, or None for anything else (a heartbeat, a malformed or partly garbled result).
    All or nothing: a telegram with any byte that is not part of a well-formed result is
    None, never a partial list - the caller counts it and moves on."""
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError:
        return None
    head, _, tail = text.partition(" ")
    if head in REPLY_KINDS:
        name, _, value = tail.partition(" ")
        return Reply(head, name, value)
    reads, i = [], 0
    for m in RESULT_RE.finditer(text):
        if m.start() != i:
            return None
        pos, label, xs, ys, angs, tss = m.groups()
        reads.append(CodeRead(pos, label, int(xs), int(ys), float(angs), int(tss)))
        i = m.end()
    if not reads or i != len(text):
        return None
    return reads


def _median(v: list[float]) -> float:
    s = sorted(v)
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


def wrap_deg(a: float) -> float:
    """Into (-180, 180]."""
    a = math.fmod(a, 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def circular_median_deg(angles: list[float]) -> float:
    """Median of angles that may straddle 0/360: deviations from the circular mean are
    medianed, then added back. Result in [0, 360)."""
    c = sum(math.cos(math.radians(a)) for a in angles)
    s = sum(math.sin(math.radians(a)) for a in angles)
    mean = math.degrees(math.atan2(s, c))
    return (mean + _median([wrap_deg(a - mean) for a in angles])) % 360.0


@dataclass(frozen=True)
class Frame:
    """Every code read from one image, aggregated. Raw device units; see to_camera_pose()."""

    ts_ms: int
    label_id: str  # "" when the codes disagree (valid False)
    n_codes: int  # codes used, after outlier rejection
    n_outliers: int
    x: float  # median XMCL, counts
    y: float  # median YMCL, counts
    angle_deg: float  # circular median, [0, 360)
    spread_xy: float  # largest distance of a used code from (x, y), counts
    spread_deg: float  # largest angular distance of a used code from angle_deg
    positions: tuple[str, ...]  # the code positions used, sorted
    valid: bool
    reason: str = ""  # why not valid


class FrameAggregator:
    """Groups CodeReads by $FAMS. add() returns the frame a NEW timestamp closes (a camera
    image is complete once the next one starts); flush() closes the open frame after a gap,
    because the last image before a no-read has no successor to close it.

    outlier_xy / outlier_deg (optional): a code further than this from the frame's median is
    dropped before the medians are taken again. Off by default: the tolerance is in device
    counts, and what a count is has not been measured yet (gls6-system.md §7)."""

    def __init__(self, min_codes: int = 1, outlier_xy: float | None = None, outlier_deg: float | None = None):
        self.min_codes = max(1, int(min_codes))
        self.outlier_xy, self.outlier_deg = outlier_xy, outlier_deg
        self._reads: list[CodeRead] = []
        self._last_t: float | None = None

    def add(self, read: CodeRead, t: float) -> Frame | None:
        closed = None
        if self._reads and read.ts_ms != self._reads[0].ts_ms:
            closed = self._close()
        self._reads.append(read)
        self._last_t = t
        return closed

    def flush(self, t: float, gap_s: float) -> Frame | None:
        """Close the open frame if nothing arrived for gap_s (pass gap_s=0 to force it)."""
        if self._reads and self._last_t is not None and t - self._last_t >= gap_s:
            return self._close()
        return None

    def _close(self) -> Frame:
        reads, self._reads = self._reads, []
        return aggregate(reads, self.min_codes, self.outlier_xy, self.outlier_deg)


def aggregate(
    reads: list[CodeRead],
    min_codes: int = 1,
    outlier_xy: float | None = None,
    outlier_deg: float | None = None,
) -> Frame:
    """One frame from the code reads of one image (all the same ts_ms)."""
    ts = reads[0].ts_ms
    labels = {r.label_id for r in reads}
    if len(labels) != 1:
        # Two labels in one image is not a position: refuse it rather than average them.
        return Frame(ts, "", 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, (), False, f"mixed labels {sorted(labels)}")
    used = list(reads)
    mx, my = _median([r.x for r in used]), _median([r.y for r in used])
    ma = circular_median_deg([r.angle_deg for r in used])
    if outlier_xy is not None or outlier_deg is not None:
        keep = [
            r
            for r in used
            if (outlier_xy is None or math.hypot(r.x - mx, r.y - my) <= outlier_xy)
            and (outlier_deg is None or abs(wrap_deg(r.angle_deg - ma)) <= outlier_deg)
        ]
        if keep:
            used = keep
            mx, my = _median([r.x for r in used]), _median([r.y for r in used])
            ma = circular_median_deg([r.angle_deg for r in used])
    spread_xy = max(math.hypot(r.x - mx, r.y - my) for r in used)
    spread_deg = max(abs(wrap_deg(r.angle_deg - ma)) for r in used)
    valid = len(used) >= min_codes
    return Frame(
        ts,
        labels.pop(),
        len(used),
        len(reads) - len(used),
        mx,
        my,
        ma,
        spread_xy,
        spread_deg,
        tuple(sorted(r.pos for r in used)),
        valid,
        "" if valid else f"{len(used)} code(s), need {min_codes}",
    )


def to_camera_pose(
    frame: Frame,
    m_per_count: float,
    x_sign: int = 1,
    y_sign: int = 1,
    angle_sign: int = 1,
    angle_offset_rad: float = 0.0,
    swap_xy: bool = False,
) -> tuple[float, float, float]:
    """(x_m, y_m, yaw_rad) of the camera in the label frame, T_LC, from a frame's raw values.
    Every argument comes from the bench tests (gls6-system.md §7) via the profile; there is
    deliberately no default scale. swap_xy takes the label frame's x from the device's Y
    (and y from X) before the signs: on MCQ4-4 the device's Y runs along the vehicle's
    fore-aft axis (bench 2026-10-30). Swap, signs and angle offset together express any of
    the 8 axis arrangements, mirrored ones included. yaw is wrapped into (-pi, pi]."""
    if not (math.isfinite(m_per_count) and m_per_count > 0.0):
        raise ValueError(f"m_per_count must be a positive number, got {m_per_count!r}")
    for name, s in (("x_sign", x_sign), ("y_sign", y_sign), ("angle_sign", angle_sign)):
        if s not in (1, -1):
            raise ValueError(f"{name} must be 1 or -1, got {s!r}")
    yaw = math.radians(wrap_deg(angle_sign * frame.angle_deg)) + angle_offset_rad
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    a, b = (frame.y, frame.x) if swap_xy else (frame.x, frame.y)
    return x_sign * a * m_per_count, y_sign * b * m_per_count, yaw


def command(text: str) -> bytes:
    """A CoLa A command telegram, e.g. command("sRN OFtplstr1")."""
    return bytes([STX]) + text.encode("ascii") + bytes([ETX])
