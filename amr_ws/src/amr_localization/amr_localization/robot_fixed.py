"""Robot-fixed return filter: a return that turns WITH the vehicle is not in the world. Pure.

A wall, a rack or a person keeps its place in the world, so when the vehicle turns, the beam
that sees it changes. A return that stays on the same beam at the same range while the
vehicle turns is fixed to the vehicle: its own body, a bracket or cable in the scanner's
view, or a scratch on the scanner window (AMR QR, 2026-09-24). This filter learns such
returns while the vehicle runs and sets them to NaN ("no reading").

Why ROTATION is the evidence, not distance driven: driving straight along a wall keeps
every beam that sees the wall at a constant range too (a beam at angle a sees a parallel
wall at d / sin a), so "unchanged while driving" would erase corridor walls. Turning
changes the range of every world object off the rotation axis; nothing in the world turns
with the vehicle.

Per beam: the reading's anchor range and the SPAN of vehicle heading (max - min of the
unwrapped yaw) seen while the reading stayed within tolerance of it - a span, not a sum, so
heading jitter while driving straight never adds up. The span needed (90 deg) is about 3x
what a smooth wall can fake: a wall seen near its normal keeps its range within the
tolerance for at most ~30 deg of turning. Enough rotation -> the beam is flagged at that range; afterwards
only readings near that range are dropped (a real object farther or nearer on the same
beam still passes). A flag lapses when its range has not been seen for unflag_s.

What was learnt is saved (to_dict / load_dict, ~/.amr/robot_fixed.yaml on the robot) and
reloaded at start for the same scan geometry, so the vehicle does not have to turn 90 deg
after every restart before its phantoms stop counting as obstacles.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

NAN = float("nan")


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class RobotFixedFilter:
    SLOTS = 2  # a flickering phantom alternates with the world behind it: track both

    def __init__(
        self,
        need_rot_rad: float = math.radians(90.0),
        tol_abs_m: float = 0.03,
        tol_rel: float = 0.01,
        max_gap_scans: int = 5,
        unflag_s: float = 120.0,
        max_step_rad: float = math.radians(20.0),
    ) -> None:
        self.need_rot = need_rot_rad
        self.tol_abs, self.tol_rel = tol_abs_m, tol_rel
        self.max_gap = max_gap_scans
        self.unflag_s = unflag_s
        self.max_step = max_step_rad  # a bigger heading step between scans is a relocalisation
        self.n = 0
        self._yaw: float | None = None
        self._uyaw = 0.0  # unwrapped heading
        # per beam, per slot: [anchor range, min heading, max heading, scans since last match]
        self._slots: list[list[list[float]]] = []
        self.flagged: dict[int, float] = {}  # beam -> range
        self._seen: dict[int, float] = {}  # beam -> last time its flagged range was seen

    def _resize(self, n: int) -> None:
        self.n = n
        self._slots = [[] for _ in range(n)]
        self.flagged, self._seen = {}, {}

    def _near(self, r: float, ref: float) -> bool:
        return abs(r - ref) <= self.tol_abs + self.tol_rel * ref

    def update(
        self, ranges: Sequence[float], range_min: float, range_max: float, yaw: float, now: float
    ) -> tuple[list[float], int]:
        """(filtered ranges, number removed). yaw: the scanner's heading in odom at the scan."""
        r = list(ranges)
        if len(r) != self.n:
            self._resize(len(r))
        if self._yaw is not None:
            step = _wrap(yaw - self._yaw)
            if abs(step) > self.max_step:
                self._slots = [[] for _ in range(self.n)]  # not a turn: start over (flags kept)
            else:
                self._uyaw += step
        self._yaw = yaw
        u = self._uyaw
        out, removed = list(r), 0
        for i, x in enumerate(r):
            valid = math.isfinite(x) and range_min <= x <= range_max
            slots = self._slots[i]
            hit = None
            if valid:
                for sl in slots:
                    if self._near(x, sl[0]):
                        hit = sl
                        break
            for sl in slots:
                sl[3] = 0 if sl is hit else sl[3] + 1
            if hit is not None:
                hit[1], hit[2] = min(hit[1], u), max(hit[2], u)
            slots[:] = [sl for sl in slots if sl[3] <= self.max_gap]
            if valid and hit is None:
                if len(slots) >= self.SLOTS:
                    slots.remove(max(slots, key=lambda sl: sl[3]))  # replace the stalest
                slots.append([x, u, u, 0])
            if i not in self.flagged:
                for sl in slots:
                    if sl[2] - sl[1] >= self.need_rot:
                        self.flagged[i] = sl[0]
                        self._seen[i] = now
                        break
            f = self.flagged.get(i)
            if f is not None:
                if valid and self._near(x, f):
                    out[i] = NAN
                    removed += 1
                    self._seen[i] = now
                elif now - self._seen[i] > self.unflag_s:
                    del self.flagged[i], self._seen[i]
        return out, removed

    def to_dict(self, angle_min: float, angle_inc: float) -> dict:
        return {
            "beams": self.n,
            "angle_min": round(float(angle_min), 6),
            "angle_increment": round(float(angle_inc), 9),
            "flags": {int(i): round(float(r), 3) for i, r in sorted(self.flagged.items())},
        }

    def load_dict(self, d: dict | None, n: int, angle_min: float, angle_inc: float, now: float) -> int:
        """Flags from to_dict() if they were learnt on the same scan geometry; returns how many."""
        if not d or int(d.get("beams", -1)) != n:
            return 0
        if abs(float(d.get("angle_min", 1e9)) - angle_min) > 1e-4:
            return 0
        if abs(float(d.get("angle_increment", 1e9)) - angle_inc) > 1e-7:
            return 0
        if len(self._slots) != n:
            self._resize(n)
        for i, r in (d.get("flags") or {}).items():
            i, r = int(i), float(r)
            if 0 <= i < n and math.isfinite(r) and r > 0:
                self.flagged[i] = r
                self._seen[i] = now
        return len(self.flagged)

    def sectors(self, angle_min: float, angle_inc: float) -> list[tuple[float, float, float]]:
        """Flagged beams as (from_deg, to_deg, max_range_m) runs, for the log."""
        out: list[tuple[float, float, float]] = []
        for i in sorted(self.flagged):
            deg = math.degrees(angle_min + i * angle_inc)
            if out and i - out[-1][3] <= 2:
                a, _, rmax, _ = out[-1]
                out[-1] = (a, deg, max(rmax, self.flagged[i]), i)
            else:
                out.append((deg, deg, self.flagged[i], i))
        return [(a, b, r) for a, b, r, _ in out]
