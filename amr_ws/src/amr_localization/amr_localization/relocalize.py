"""Automatic relocalisation after a loss during a run (owner request 2026-11-09). Pure.

The route executor stops the run (a BLOCKED hold, cause "loc") when localisation drops from
READY for a reason the map and the scan can fix (a jump, a scan that stopped fitting, AMCL's
covariance growing) and asks the localization monitor to relocalise. The monitor then does
what the operator would do on the Run page:

  1. sets the initial pose where the AMR stands now (AMCL's current estimate), with a spread
     wide enough for AMCL to find the right place nearby (INITIAL_SIGMA_*);
  2. lets AMCL converge in place (readiness asks for no-motion updates while CHECKING);
  3. confirms by itself - in place of the operator's Confirm - only once readiness itself says
     it could be confirmed (converged, settled, nothing through walls) AND
       * at least `fit_min` of the scan lies on mapped walls, and
       * the converged pose is within `max_shift_m` / `max_turn_rad` of where it started
         (AMCL found the AMR close to where it thought it was, not somewhere else).
  Anything else within `timeout_s` - or a converged pose too far away - is a failure: the
  monitor gives up, localisation stays CHECKING, and the executor FAULTs the run for the
  operator, as before this existed.

A sensor that went quiet (LOC_STREAM_STALE) or a scan check that stopped (LOC_GATE_FAILED)
is never relocalised: that is hardware, and a new pose fixes nothing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

RECOVERABLE = frozenset({"LOC_JUMP", "LOC_SCAN_MISMATCH", "LOC_COV_GREW", "LOC_LOST"})
INITIAL_SIGMA_XY_M = 0.25
INITIAL_SIGMA_YAW_RAD = math.radians(8.0)


@dataclass(frozen=True)
class Limits:
    max_shift_m: float = 0.50
    max_turn_rad: float = math.radians(15.0)
    fit_min: float = 0.70
    timeout_s: float = 30.0


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class AutoReloc:
    """One attempt at a time. start() -> the pose to publish as /initialpose; tick() decides."""

    def __init__(self, limits: Limits | None = None) -> None:
        self.limits = limits or Limits()
        self.pose0: tuple[float, float, float] | None = None
        self.t0: float | None = None
        self.last = ""  # the last outcome, for the operator

    @property
    def active(self) -> bool:
        return self.pose0 is not None

    def start(self, t: float, pose, state_lost: bool, code: str) -> str | None:
        """Begin an attempt at `pose` (map frame x, y, yaw). Returns why it is refused, or None."""
        if not state_lost:
            return "localisation is not LOST"
        if code not in RECOVERABLE:
            return f"{code or 'this loss'} is not fixed by a new pose (a sensor or the scan check)"
        if pose is None:
            return "no pose estimate to start from"
        self.pose0, self.t0, self.last = tuple(pose), t, ""
        return None

    def tick(self, t: float, pose, checking: bool, can_confirm: bool, scan_match) -> str | None:
        """While an attempt is active: "confirm" when the monitor should confirm now, "fail: ..."
        when it is over without, None to keep waiting."""
        if self.pose0 is None:
            return None
        if t - self.t0 > self.limits.timeout_s:
            return self._end(f"fail: not converged within {self.limits.timeout_s:g} s")
        if not checking or not can_confirm or pose is None:
            return None
        d = math.hypot(pose[0] - self.pose0[0], pose[1] - self.pose0[1])
        a = abs(_wrap(pose[2] - self.pose0[2]))
        if d > self.limits.max_shift_m or a > self.limits.max_turn_rad:
            lim = self.limits
            return self._end(
                f"fail: converged {d:.2f} m / {math.degrees(a):.0f} deg from where it stopped "
                f"(allowed {lim.max_shift_m:.2f} m / {math.degrees(lim.max_turn_rad):.0f} deg)"
            )
        if scan_match is None or scan_match < self.limits.fit_min:
            return None  # converged but the scan fits too little yet: keep waiting (timeout)
        return self._end(
            f"confirm: {d:.2f} m / {math.degrees(a):.1f} deg from where it stopped, "
            f"{100 * scan_match:.0f} % of the scan on the map"
        )

    def cancel(self, why: str = "cancelled") -> None:
        if self.pose0 is not None:
            self._end(f"fail: {why}")

    def _end(self, outcome: str) -> str:
        self.pose0, self.t0, self.last = None, None, outcome
        return outcome
