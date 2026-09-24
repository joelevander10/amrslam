"""Preset survey moves (2026-09-19): "straight X m" and "spin 45/90/135/180 deg", press once.

Pure, clock-fed, no ROS. survey_move_node feeds odometry and stamps and publishes what
`step` returns as a browser-style ManualCommand, so the mux's MANUAL rules (selector MANUAL,
manual lease, S-curve, survey spin cap, 0.2 s command lifetime, pendant outranks) all hold.

Why presets: slow, smooth, repeatable motion is what the scan matcher copes with best, and a
move of known size is a yardstick for the map. After every move `slam_correction` measures
how much SLAM moved the vehicle's map pose relative to odometry during it. With the gyro
carrying the heading (EKF weights it ~1000:1 over the wheels, bias ~0.02 deg/s), a heading
correction of a couple of degrees over one short move means the scan matcher took a wrong
alignment: the operator is warned at once, where a rotated map was otherwise found only at
the end of the survey.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

STRAIGHT, ROTATE = "straight", "rotate"
ROTATE_STEPS_DEG = (45.0, 90.0, 135.0, 180.0)
STRAIGHT_MIN_M, STRAIGHT_MAX_M = 0.05, 10.0


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MoveError(ValueError):
    pass


@dataclass(frozen=True)
class Profile:
    v_mps: float = 0.30  # operator decision 2026-09-19
    w_rad_s: float = 0.20
    decel_mps2: float = 0.15  # our own slow-down, gentler than the mux's so it is followed
    decel_rad_s2: float = 0.15
    v_min: float = 0.03  # creep speed for the last centimetres
    w_min: float = 0.03
    tol_m: float = 0.01
    tol_rad: float = math.radians(0.5)
    heading_gain: float = 1.5  # straight: rad/s per rad of heading error, held to the start heading
    heading_w_max: float = 0.10
    stall_s: float = 3.0  # commanding motion with no progress this long -> abort
    time_factor: float = 2.0  # abort after time_factor x nominal duration + time_margin_s
    time_margin_s: float = 5.0


@dataclass
class Move:
    kind: str
    target: float  # signed: m (+ forward) or rad (+ ccw)

    @staticmethod
    def parse(kind: str, value: float) -> Move:
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            raise MoveError(f"value must be a finite number, got {value!r}")
        if kind == STRAIGHT:
            if not STRAIGHT_MIN_M <= abs(value) <= STRAIGHT_MAX_M:
                raise MoveError(f"distance must be {STRAIGHT_MIN_M:g}..{STRAIGHT_MAX_M:g} m, got {value:g}")
            return Move(STRAIGHT, float(value))
        if kind == ROTATE:
            if abs(value) not in ROTATE_STEPS_DEG:
                raise MoveError(f"spin must be one of {ROTATE_STEPS_DEG} deg, got {value:g}")
            return Move(ROTATE, math.radians(value))
        raise MoveError(f"kind must be {STRAIGHT!r} or {ROTATE!r}, got {kind!r}")

    def label(self) -> str:
        if self.kind == STRAIGHT:
            return f"{'forward' if self.target > 0 else 'reverse'} {abs(self.target):.2f} m"
        return f"spin {'left' if self.target > 0 else 'right'} {math.degrees(abs(self.target)):.0f} deg"


def _ramp(remaining: float, vmax: float, decel: float, vmin: float) -> float:
    return max(vmin, min(vmax, math.sqrt(2.0 * decel * max(0.0, remaining))))


class MoveController:
    """One move from `start(pose, now)`; `step(pose, now)` -> (v, w, done, reason). Poses are
    odometry (x, y, yaw): the EKF, gyro-carried heading. `done` with reason "" is arrival."""

    def __init__(self, move: Move, profile: Profile | None = None) -> None:
        self.move, self.p = move, profile or Profile()
        self.progress = 0.0  # signed like the target
        self._start: tuple[float, float, float] | None = None
        self._yaw_prev = 0.0
        self._t0 = 0.0
        self._best = 0.0
        self._best_t = 0.0

    @property
    def nominal_s(self) -> float:
        if self.move.kind == STRAIGHT:
            return abs(self.move.target) / self.p.v_mps + 2.0
        return abs(self.move.target) / self.p.w_rad_s + 2.0

    def start(self, pose: tuple[float, float, float], now: float) -> None:
        self._start, self._yaw_prev, self._t0 = pose, pose[2], now
        self._best, self._best_t, self.progress = 0.0, now, 0.0

    def _update(self, pose) -> None:
        x0, y0, yaw0 = self._start
        if self.move.kind == STRAIGHT:
            self.progress = (pose[0] - x0) * math.cos(yaw0) + (pose[1] - y0) * math.sin(yaw0)
        else:
            self.progress += wrap(pose[2] - self._yaw_prev)  # unwrapped: 180 deg is not ambiguous
            self._yaw_prev = pose[2]

    def step(self, pose: tuple[float, float, float], now: float) -> tuple[float, float, bool, str]:
        self._update(pose)
        sign = 1.0 if self.move.target > 0 else -1.0
        remaining = sign * (self.move.target - self.progress)  # > 0 while short of the target
        tol = self.p.tol_m if self.move.kind == STRAIGHT else self.p.tol_rad
        if remaining <= tol:
            return 0.0, 0.0, True, ""
        done_frac = sign * self.progress
        if done_frac > self._best + 0.2 * tol:
            self._best, self._best_t = done_frac, now
        elif now - self._best_t > self.p.stall_s:
            return 0.0, 0.0, True, f"no progress for {self.p.stall_s:.0f} s"
        if now - self._t0 > self.p.time_factor * self.nominal_s + self.p.time_margin_s:
            return 0.0, 0.0, True, "took too long"
        if self.move.kind == STRAIGHT:
            v = sign * _ramp(remaining, self.p.v_mps, self.p.decel_mps2, self.p.v_min)
            err = wrap(pose[2] - self._start[2])
            w = max(-self.p.heading_w_max, min(self.p.heading_w_max, -self.p.heading_gain * err))
            return v, w, False, ""
        return 0.0, sign * _ramp(remaining, self.p.w_rad_s, self.p.decel_rad_s2, self.p.w_min), False, ""

    def achieved(self) -> float:
        return self.progress


def _apply(frame, pose):
    fx, fy, fyaw = frame
    px, py, pyaw = pose
    c, s = math.cos(fyaw), math.sin(fyaw)
    return fx + c * px - s * py, fy + s * px + c * py, wrap(fyaw + pyaw)


def slam_correction(map_odom_before, map_odom_after, odom_base) -> tuple[float, float]:
    """How far SLAM moved the vehicle's map pose relative to odometry between two map->odom
    samples, measured AT the vehicle (odom_base = base in odom now): (metres, radians)."""
    a = _apply(map_odom_before, odom_base)
    b = _apply(map_odom_after, odom_base)
    return math.hypot(b[0] - a[0], b[1] - a[1]), abs(wrap(b[2] - a[2]))


def check_verdict(move: Move, corr_m: float, corr_rad: float, max_deg: float = 2.0, max_frac: float = 0.03):
    """(ok, text). Translation allowance: 3 % of a straight (wheel scale), at least 0.10 m."""
    lim_m = max(0.10, max_frac * abs(move.target)) if move.kind == STRAIGHT else 0.10
    ok = corr_rad <= math.radians(max_deg) and corr_m <= lim_m
    text = f"SLAM corrected {corr_m * 100:.0f} cm / {math.degrees(corr_rad):.1f} deg over this move"
    if not ok:
        text += " - the map may have slipped here: back up and drive this stretch again, slowly"
    return ok, text
