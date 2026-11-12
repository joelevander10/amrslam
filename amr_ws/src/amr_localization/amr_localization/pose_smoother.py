"""Smooth the map->odom correction the controller steers by (owner request 2026-11-12). Pure.

Why: AMCL moves map->odom at every filter update (update_min_d, every 5 cm). Where most of
the scan is blanked - the dynamic areas, trolleys on both sides - few beams hold the pose,
and each update shifts it a few cm and a fraction of a degree, back and forth. The
controller steers by map->base_footprint, takes every shift for being off the line and
steers back: the AMR wobbles although its wheels drove straight.

So AMCL no longer broadcasts map->odom (tf_broadcast false, launch smooth_pose:=true).
pose_smoother_node works out AMCL's map->odom from /amcl_pose ("raw", also published for
the localization monitor, which judges AMCL by it) and broadcasts a smoothed one.

The smoothing acts on the ROBOT's pose, not on map->odom's numbers: a heading change of
map->odom turns about the odom origin, which can lie 100 m away. Every tick:

  target  = raw map->odom      o odom->base   (where AMCL says the AMR is)
  current = smoothed map->odom o odom->base   (where the controller thinks it is)
  current moves towards target by a first-order lag (tau_s), at most
    position  max_rate_m_s * dt  + per_m * metres driven  + per_rad_m * radians turned
    heading   max_rate_rad_s * dt + per_m_rad * metres driven + per_rad * radians turned
  smoothed map->odom = current o inverse(odom->base)

So a correction is worked in over a second or two of driving instead of in one 5 cm step,
and AMCL's update-to-update jitter mostly cancels before it reaches the steering.

It takes the raw pose at once ("snaps") for the first pose, after /initialpose (until the
next AMCL pose), while the AMR stands still (nothing steers then, and the executor's
endpoint checks see AMCL's pose as before), and when raw and smoothed are further apart
than snap_m / snap_rad: that is a loss, and the localization monitor stops the run anyway.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Pose = tuple[float, float, float]  # x, y, yaw


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def compose(a: Pose, b: Pose) -> Pose:
    """a o b: b expressed in a's parent frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a: Pose) -> Pose:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


@dataclass(frozen=True)
class Params:
    tau_s: float = 1.0
    max_rate_m_s: float = 0.03
    per_m: float = 0.05
    per_rad_m: float = 0.02
    max_rate_rad_s: float = math.radians(0.5)
    per_m_rad: float = math.radians(1.0)
    per_rad: float = 0.05
    snap_m: float = 0.50
    snap_rad: float = math.radians(10.0)


class Smoother:
    def __init__(self, params: Params | None = None) -> None:
        self.p = params or Params()
        self.raw: Pose | None = None  # AMCL's map->odom
        self.out: Pose | None = None  # the smoothed map->odom
        self._snap = True

    def set_raw(self, raw: Pose, snap: bool = False) -> None:
        self.raw = tuple(raw)
        if snap:
            self._snap = True

    def gap(self, odom_base: Pose) -> tuple[float, float] | None:
        """(metres, radians) between where AMCL and the smoothed pose put the AMR."""
        if self.raw is None or self.out is None:
            return None
        t, c = compose(self.raw, odom_base), compose(self.out, odom_base)
        return math.hypot(t[0] - c[0], t[1] - c[1]), abs(wrap(t[2] - c[2]))

    def step(
        self, odom_base: Pose, dt: float, travel_m: float, turned_rad: float, still: bool
    ) -> Pose | None:
        """The smoothed map->odom after this tick (None until AMCL gave a pose)."""
        if self.raw is None:
            return None
        if self.out is None or self._snap or still:
            self.out, self._snap = self.raw, False
            return self.out
        p = self.p
        target = compose(self.raw, odom_base)
        cur = compose(self.out, odom_base)
        dx, dy, da = target[0] - cur[0], target[1] - cur[1], wrap(target[2] - cur[2])
        d = math.hypot(dx, dy)
        if d > p.snap_m or abs(da) > p.snap_rad:
            self.out = self.raw
            return self.out
        dt, travel_m, turned_rad = max(dt, 0.0), abs(travel_m), abs(turned_rad)
        k = 1.0 - math.exp(-dt / p.tau_s) if p.tau_s > 0 else 1.0
        cap_d = p.max_rate_m_s * dt + p.per_m * travel_m + p.per_rad_m * turned_rad
        cap_a = p.max_rate_rad_s * dt + p.per_m_rad * travel_m + p.per_rad * turned_rad
        step_d = min(k * d, cap_d)
        step_a = math.copysign(min(k * abs(da), cap_a), da)
        sx, sy = (dx / d * step_d, dy / d * step_d) if d > 1e-9 else (0.0, 0.0)
        moved = (cur[0] + sx, cur[1] + sy, wrap(cur[2] + step_a))
        self.out = compose(moved, inverse(odom_base))
        return self.out


class StillDetector:
    """Standing still: odometry moved less than v_m_s / w_rad_s for still_s on end."""

    def __init__(
        self, v_m_s: float = 0.01, w_rad_s: float = math.radians(0.5), still_s: float = 0.15
    ) -> None:
        self.v, self.w, self.still_s = v_m_s, w_rad_s, still_s
        self._since: float | None = None
        self._ref: tuple[float, Pose] | None = None  # (t, odom_base) the motion is measured from

    def update(self, t: float, odom_base: Pose) -> bool:
        if self._ref is None:
            self._ref = (t, odom_base)
            return False
        t0, o0 = self._ref
        dt = t - t0
        w = max(dt, 0.1)  # judge over at least 0.1 s: 20 ms of odometry is mostly noise
        moving = (
            math.hypot(odom_base[0] - o0[0], odom_base[1] - o0[1]) > self.v * w
            or abs(wrap(odom_base[2] - o0[2])) > self.w * w
        )
        if moving:  # seen at once
            self._ref, self._since = (t, odom_base), None
            return False
        if dt < 0.1:  # quiet so far: decide once the window is long enough
            return self._since is not None and t - self._since >= self.still_s
        self._ref = (t, odom_base)
        if self._since is None:
            self._since = t0
        return t - self._since >= self.still_s
