"""Wheel slip and hitch-snatch candidates in one run's logged series (towing-load Phase 1).

*** OFFLINE, DIAGNOSTIC ONLY. *** tools/bag_report.py calls this on a recorded bag.
Nothing here runs on the vehicle or decides whether it may move.

WHY NOT THE ENCODERS
--------------------
The drives hold the commanded speed through a 1:30 gearhead (the motor's inertia
x 900 at the wheel), so a sudden pull at the hitch barely slows the wheel - the
tyre slips instead. The encoders keep reporting the commanded speed, and so do
the drives' own deviation alarms (FF10, 4075h): the motor did what it was told.
A dip in encoder speed is therefore NOT used to confirm anything here.

THE THREE DETECTORS
-------------------
  A  snatch      the size of the summed wheel torque GROWS by more than SNATCH_NM
                 between two consecutive samples (20 ms at feedback_hz 50) while
                 moving: a loose pin taking up its slack on pull-away (driving torque
                 up), the trolley running into the vehicle as it stops (braking torque
                 up), or a caster swivelling round. A sudden fall towards zero is not
                 an impact; that is B's business. The tyre gripping again after a
                 collapse is a real jolt and shows as a snatch right after it.
  B  collapse    torque falls by more than COLLAPSE_FRAC of its peak over the last
                 COLLAPSE_WINDOW_S, AND below (1 - COLLAPSE_FRAC) of its level before
                 that (median over BASELINE_S), while the commanded speed is still
                 rising: the tyre breaking loose. The second condition is what keeps
                 a snatch that simply ends - torque back to where it was - from
                 reading as a slip. Within SNATCH_SLIP_S after a snatch it is
                 labelled "snatch_slip". Static friction letting go at start-up
                 (casters, bearings) has the same shape; the no-trolley baseline
                 runs show how big that is on this vehicle.
  C  yaw         wheel yaw rate and gyro yaw rate disagree by more than YAW_RAD_S
                 after smoothing: one wheel slipping more than the other, or a
                 sideways shove. Both wheels slipping equally is invisible to it;
                 B covers that case.

A STEADY wheel/gyro disagreement is a calibration error (track width, a wheel
diameter), not slip. yaw_gain() fits it once per run and C works on what is left.

Every threshold below is a FIRST GUESS, to be tuned on the Phase 2 towing runs.
The report also prints the run's largest torque jumps and yaw-rate residuals
regardless of thresholds, so the first runs say where the numbers belong.

SERIES
------
Plain tuples, time in seconds from the start of the bag, ascending:
    torque    (t, sum_nm, valid)              both wheels' torque, wheel side
    cmd       (t, v_mps)                      commanded body speed
    wheels    (t, v_mps, yaw_rate, valid)     encoder body speed and yaw rate
    gyro      (t, yaw_rate)                   bias-removed gyro
    map_odom  (t, x, y, yaw)                  AMCL's map->odom
    loc       (t, state_name)                 localization state
"""

from __future__ import annotations

import math
from bisect import bisect_left, bisect_right
from dataclasses import dataclass

SNATCH_NM = 10.0  # N·m between consecutive samples, both wheels summed (100 % rated = 76 N·m)
MOVING_MPS = 0.01  # commanded or encoder speed above which the vehicle counts as moving
COLLAPSE_FRAC = 0.4  # torque at most 60 % of its recent peak ...
COLLAPSE_WINDOW_S = 0.1  # ... within this long ...
COLLAPSE_MIN_PEAK_NM = 5.0  # ... from a peak at least this big
BASELINE_S = 0.4  # the torque level before the window: median over this long before it
RISING_MPS = 0.001  # commanded speed grew by this over COLLAPSE_WINDOW_S (0.01 m/s^2)
SNATCH_SLIP_S = 0.3  # a collapse this soon after a snatch is the snatch slipping
YAW_RAD_S = 0.05  # wheel-gyro yaw-rate residual (2.9 deg/s)
YAW_SMOOTH_S = 0.1  # trailing mean on both yaw rates: absorbs the two streams' timing skew
YAW_MIN_S = 0.06  # residual must persist this long (4 samples at 50 Hz)
GAIN_MIN_RAD_S = 0.05  # samples turning faster than this fit the steady gain
GAIN_MIN_SAMPLES = 25
MAX_GAP_S = 0.1  # samples further apart than this are not consecutive
MERGE_S = 0.3  # detections of one kind closer than this are one event
AFTER_S = 2.0  # how long after an event its localization consequence is looked for
TOP_N = 5

# Higher is worse; anything unknown ranks with LOST rather than hiding.
LOC_RANK = {"READY": 0, "CHECKING": 1, "UNLOCALIZED": 2, "LOST": 3}


@dataclass
class Event:
    t: float
    kind: str  # "snatch", "collapse", "snatch_slip", "yaw"
    size: float  # N·m (snatch, collapse) or rad/s (yaw), signed
    duration_s: float = 0.0
    v_mps: float | None = None
    cmd_mps: float | None = None
    map_odom_m: float | None = None  # largest map->odom shift within AFTER_S
    map_odom_deg: float | None = None
    loc_worst: str = ""


class _S:
    """A series with its time column indexed once: every lookup below is a bisect, so a
    20-minute bag (60 000 samples a stream) stays linear."""

    def __init__(self, rows):
        self.rows = list(rows)
        self.ts = [row[0] for row in self.rows]

    def __bool__(self):
        return bool(self.rows)

    def at(self, t, col):
        """row[col] of the last row at or before t (the first row if t is earlier)."""
        if not self.rows:
            return None
        return self.rows[max(bisect_right(self.ts, t) - 1, 0)][col]

    def between(self, a, b):
        """Rows with a < t <= b."""
        return self.rows[bisect_right(self.ts, a):bisect_right(self.ts, b)]

    def nearest_gap(self, t):
        j = bisect_right(self.ts, t)
        return min((abs(self.ts[k] - t) for k in (j - 1, j) if 0 <= k < len(self.ts)), default=None)


class _Mean:
    """Trailing mean of one column over a time span, by prefix sums."""

    def __init__(self, s: _S, col):
        self.s, self.cum = s, [0.0]
        for row in s.rows:
            self.cum.append(self.cum[-1] + row[col])

    def __call__(self, t, span):
        lo, hi = bisect_left(self.s.ts, t - span), bisect_right(self.s.ts, t)
        return (self.cum[hi] - self.cum[lo]) / (hi - lo) if hi > lo else None


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _merge(hits, merge_s=MERGE_S):
    """Collapse hits of one kind closer than merge_s into one event: it starts at the first
    hit, lasts to the last, and carries the largest |size|. The inputs are not modified."""
    out = []
    for e in sorted(hits, key=lambda e: e.t):
        if out and e.t - (out[-1].t + out[-1].duration_s) <= merge_s:
            first = out[-1]
            end = max(first.t + first.duration_s, e.t + e.duration_s)
            if abs(e.size) > abs(first.size):
                first.size = e.size
            first.duration_s = end - first.t
        else:
            out.append(Event(e.t, e.kind, e.size, e.duration_s))
    return out


def _moving(t, cmd: _S, wheels: _S) -> bool:
    c, v = cmd.at(t, 1), wheels.at(t, 1)
    return (c is not None and abs(c) >= MOVING_MPS) or (v is not None and abs(v) >= MOVING_MPS)


def torque_jumps(torque, cmd, wheels):
    """Every consecutive-sample torque change that grew |torque| while moving, as unmerged
    snatch candidates. Sized by the signed change: + pulls forward, - pushes back."""
    cmd, wheels = _S(cmd), _S(wheels)
    out = []
    for prev, cur in zip(torque, torque[1:], strict=False):
        if not (prev[2] and cur[2]) or cur[0] - prev[0] > MAX_GAP_S:
            continue
        if abs(cur[1]) > abs(prev[1]) and _moving(cur[0], cmd, wheels):
            out.append(Event(cur[0], "snatch", cur[1] - prev[1]))
    return out


def snatches(torque, cmd, wheels):
    return _merge(e for e in torque_jumps(torque, cmd, wheels) if abs(e.size) >= SNATCH_NM)


def collapses(torque, cmd):
    """Torque falling fast, and below where it was before, while the command still asks
    for more speed.

    The sign of the commanded speed says which way "driving" torque points, so reverse
    is handled; a vehicle braking (command falling) is never a candidate. Sized against
    the recent peak.
    """
    tq, cmd = _S(torque), _S(cmd)
    out = []
    for i, (t, nm, valid) in enumerate(tq.rows):
        c_now, c_then = cmd.at(t, 1), cmd.at(t - COLLAPSE_WINDOW_S, 1)
        if not valid or c_now is None or abs(c_now) < MOVING_MPS:
            continue
        if abs(c_now) - abs(c_then) < RISING_MPS:
            continue
        sign = 1.0 if c_now > 0 else -1.0
        lo = bisect_left(tq.ts, t - COLLAPSE_WINDOW_S)
        window = [sign * row[1] for row in tq.rows[lo:i] if row[2]]
        before = sorted(sign * row[1] for row in tq.rows[bisect_left(tq.ts, t - COLLAPSE_WINDOW_S
                                                                     - BASELINE_S):lo] if row[2])
        if not window or not before:
            continue
        peak, level = max(window), before[len(before) // 2]
        floor = (1.0 - COLLAPSE_FRAC) * min(peak, level)
        if peak >= COLLAPSE_MIN_PEAK_NM and sign * nm <= floor:
            out.append(Event(t, "collapse", sign * nm - peak))
    return _merge(out)


def _yaw_pairs(wheels, gyro):
    """(t, smoothed wheel yaw rate, smoothed gyro yaw rate) on valid wheel samples with a
    gyro sample close by."""
    good, gyro = _S(row for row in wheels if row[3]), _S(gyro)
    wmean, gmean = _Mean(good, 2), _Mean(gyro, 1)
    out = []
    for t, *_ in good.rows:
        gap = gyro.nearest_gap(t)
        if gap is None or gap > MAX_GAP_S:
            continue
        wg = gmean(t, YAW_SMOOTH_S)
        out.append((t, wmean(t, YAW_SMOOTH_S), gyro.at(t, 1) if wg is None else wg))
    return out


def yaw_gain(wheels, gyro, pairs=None) -> float | None:
    """Least-squares k in wheel_yaw = k * gyro_yaw over the turning samples, or None
    if the run turned too little to say. k > 1: the encoders over-read turns, so the
    true track width is about TRACK_M * k (wheel diameters assumed right)."""
    pairs = _yaw_pairs(wheels, gyro) if pairs is None else pairs
    turning = [(we, wg) for _t, we, wg in pairs if abs(wg) >= GAIN_MIN_RAD_S]
    if len(turning) < GAIN_MIN_SAMPLES:
        return None
    return sum(we * wg for we, wg in turning) / sum(wg * wg for _we, wg in turning)


def yaw_residuals(wheels, gyro, gain=None):
    """(t, residual) per paired sample, after the steady gain is taken out."""
    pairs = _yaw_pairs(wheels, gyro)
    k = gain if gain is not None else (yaw_gain(wheels, gyro, pairs) or 1.0)
    return [(t, we - k * wg) for t, we, wg in pairs]


def yaw_mismatches(wheels, gyro, gain=None):
    out, run = [], []

    def close():
        if run and run[-1][0] - run[0][0] >= YAW_MIN_S - 1e-9:
            r = max((row[1] for row in run), key=abs)
            out.append(Event(run[0][0], "yaw", r, duration_s=run[-1][0] - run[0][0]))
        run.clear()

    for t, r in yaw_residuals(wheels, gyro, gain):
        if run and (abs(r) < YAW_RAD_S or t - run[-1][0] > MAX_GAP_S):
            close()
        if abs(r) >= YAW_RAD_S:
            run.append((t, r))
    close()
    return _merge(out)


def _consequence(e: Event, map_odom: _S, loc: _S) -> None:
    """Largest map->odom shift within AFTER_S of the event, against its value just before;
    and the worst localization state from the event to AFTER_S past its end."""
    end = e.t + e.duration_s + AFTER_S
    after = map_odom.between(e.t, end)
    if after and map_odom.ts[0] <= e.t:
        x0, y0, a0 = (map_odom.at(e.t, c) for c in (1, 2, 3))
        e.map_odom_m = max(math.hypot(x - x0, y - y0) for _t, x, y, _a in after)
        e.map_odom_deg = math.degrees(max(abs(_wrap(a - a0)) for _t, _x, _y, a in after))
    seen = [s for _t, s in loc.between(e.t, end)]
    if loc and loc.ts[0] <= e.t:
        seen.append(loc.at(e.t, 1))
    if seen:
        e.loc_worst = max(seen, key=lambda s: LOC_RANK.get(s, len(LOC_RANK)))


def detect(torque, cmd, wheels, gyro, map_odom=(), loc=()):
    """All events of the run, in time order, each with its localization consequence."""
    sn = snatches(torque, cmd, wheels)
    co = collapses(torque, cmd)
    for e in co:
        if any(0.0 <= e.t - s.t <= SNATCH_SLIP_S for s in sn):
            e.kind = "snatch_slip"
    events = sorted(sn + co + yaw_mismatches(wheels, gyro), key=lambda e: e.t)
    cs, ws, mo, lo = _S(cmd), _S(wheels), _S(map_odom), _S(loc)
    for e in events:
        e.v_mps, e.cmd_mps = ws.at(e.t, 1), cs.at(e.t, 1)
        _consequence(e, mo, lo)
    return events


def top(events, n=TOP_N):
    """The n largest |size|, no two within MERGE_S of each other - for the first runs,
    where thresholds are guesses. Peak-picking rather than _merge(): on a dense 50 Hz
    series every sample is within MERGE_S of the next, and merging would chain the
    whole run into one event."""
    picked = []
    for e in sorted(events, key=lambda e: -abs(e.size)):
        if all(abs(e.t - p.t) > MERGE_S for p in picked):
            picked.append(e)
            if len(picked) == n:
                break
    return picked


def map_odom_baseline(map_odom, wheels, every_s=1.0):
    """95th percentile of the AFTER_S map->odom shift at moving instants every every_s:
    what an event's shift is compared against. None without enough data."""
    mo, ws, none = _S(map_odom), _S(wheels), _S(())
    if not mo or not ws:
        return None
    shifts, t = [], mo.ts[0]
    while t <= mo.ts[-1] - AFTER_S:
        v = ws.at(t, 1)
        if v is not None and abs(v) >= MOVING_MPS:
            e = Event(t, "", 0.0)
            _consequence(e, mo, none)
            if e.map_odom_m is not None:
                shifts.append(e.map_odom_m)
        t += every_s
    if len(shifts) < 5:
        return None
    shifts.sort()
    return shifts[min(len(shifts) - 1, int(0.95 * len(shifts)))]
