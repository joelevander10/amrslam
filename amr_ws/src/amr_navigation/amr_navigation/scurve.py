"""The home leg (route step `home`, 2026-11-03): from wherever the step before it ends, a
forward path to the route's start pose (HOME) that arrives on HOME's heading.

Worked in HOME's frame, x along HOME's heading and y to its left. The vehicle starts
`run_up` metres behind HOME, `offset` to the side, its heading `e` off HOME's. The path is
a quintic Hermite lateral profile over the run-up, t = 0 at the start and 1 at HOME:

    y(t) = offset * H0(t) + run_up * tan(e) * H1(t)
    H0 = 1 - 10t^3 + 15t^4 - 6t^5      y = 1 at t = 0; y' = y'' = 0 at both ends; 0 at t = 1
    H1 = t - 6t^3 + 8t^4 - 3t^5        y' = 1 at t = 0; everything else 0 at both ends

It leaves along the vehicle's own heading with no curvature, so there is no steering step
after a straight, and arrives exactly at HOME, tangent to HOME's heading, with no curvature
again. With e = 0 the tightest radius is about run_up^2 / (5.77 * offset): at the 1.0 m
minimum, 0.17 m sideways fits in a 1 m run-up, 0.69 m in 2 m.

On HOME's line (the offset, and the heading error's drift over the run-up, both within
STRAIGHT_TOL_M) the leg is a straight.

Pure, no ROS. The editor (amr_web/static/editor.js homeLeg) mirrors it for its preview;
the compiler's result is the one that is validated and driven.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from amr_navigation.route import (
    HOME_MAX_HEADING_DEG,
    HOME_MAX_RUNUP_M,
    HOME_MAX_SLOPE_DEG,
    HOME_MIN_RADIUS_M,
    RouteError,
)

STRAIGHT_TOL_M = 0.001  # = compiler.COLLINEAR_TOL_M (the compiler imports this module)
MIN_RUNUP_M = 0.05  # = compiler.MIN_LENGTH_M
SHAPE_POINTS = 400  # where the slope and curvature bounds are checked
SUGGEST_STEP_M = 0.05  # the run-up a refusal suggests is searched in these steps


@dataclass(frozen=True)
class HomeLeg:
    samples: list[tuple[float, float, float]]  # x, y, yaw from the start pose to HOME
    length_m: float  # along the path
    run_up_m: float  # how far behind HOME it starts, along HOME's heading
    offset_m: float  # how far left (+) of HOME's line it starts
    heading_err_rad: float  # the start heading minus HOME's, + = turned left
    min_radius_m: float  # tightest radius on the path; inf for a straight
    straight: bool

    def describe(self) -> str:
        if self.straight:
            return f"straight {self.length_m:.2f} m to HOME"
        side = "left" if self.offset_m > 0 else "right"
        return (
            f"S-curve {self.length_m:.2f} m to HOME: {abs(self.offset_m) * 1000:.0f} mm {side}, "
            f"{math.degrees(self.heading_err_rad):+.1f} deg, tightest R {self.min_radius_m:.2f} m"
        )


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _profile(t: float, offset: float, run_up: float, slope0: float) -> tuple[float, float, float]:
    """y, dy/dx and d2y/dx2 of the path at t (x = run_up * t from the start)."""
    t2, t3, t4, t5 = t * t, t**3, t**4, t**5
    a, b = offset, run_up * slope0
    y = a * (1 - 10 * t3 + 15 * t4 - 6 * t5) + b * (t - 6 * t3 + 8 * t4 - 3 * t5)
    dy = a * (-30 * t2 + 60 * t3 - 30 * t4) + b * (1 - 18 * t2 + 32 * t3 - 15 * t4)
    ddy = a * (-60 * t + 180 * t2 - 120 * t3) + b * (-36 * t + 96 * t2 - 60 * t3)
    return y, dy / run_up, ddy / (run_up * run_up)


def _shape(offset: float, run_up: float, slope0: float) -> tuple[float, float]:
    """(steepest |dy/dx|, tightest radius) along the path."""
    steep, kmax = 0.0, 0.0
    for k in range(SHAPE_POINTS + 1):
        _, dy, ddy = _profile(k / SHAPE_POINTS, offset, run_up, slope0)
        steep = max(steep, abs(dy))
        kmax = max(kmax, abs(ddy) / (1.0 + dy * dy) ** 1.5)
    return steep, (1.0 / kmax if kmax > 0.0 else math.inf)


def _fits(offset: float, run_up: float, slope0: float) -> bool:
    steep, rmin = _shape(offset, run_up, slope0)
    return steep <= math.tan(math.radians(HOME_MAX_SLOPE_DEG)) and rmin >= HOME_MIN_RADIUS_M


def _suggest(offset: float, run_up: float, slope0: float) -> str:
    """What to change when the curve does not fit: the shortest run-up that would."""
    d = run_up
    while d <= HOME_MAX_RUNUP_M + 1e-9:
        if _fits(offset, d, slope0):
            return f"end the route at least {d:.2f} m behind HOME"
        d += SUGGEST_STEP_M
    return f"it does not fit within {HOME_MAX_RUNUP_M:.2f} m: end the route closer to HOME's line"


def home_leg(
    start: tuple[float, float, float], home: tuple[float, float, float], spacing: float = 0.05
) -> HomeLeg:
    """The home leg from `start` (x, y, yaw) to `home`. Raises RouteError (no step id: the
    compiler adds it) when the start is not behind HOME, too far back, turned too far, or
    the curve would be steeper or tighter than the bounds."""
    x0, y0, yaw0 = start
    xh, yh, yawh = home
    c, s = math.cos(yawh), math.sin(yawh)
    dx, dy = x0 - xh, y0 - yh
    along = dx * c + dy * s
    offset = -dx * s + dy * c
    err = _wrap(yaw0 - yawh)
    run_up = -along
    if run_up < MIN_RUNUP_M:
        raise RouteError(
            f"the step before ends {along:+.2f} m along HOME's heading, not behind HOME: "
            "the home leg only drives forward"
        )
    if run_up > HOME_MAX_RUNUP_M + 1e-9:
        raise RouteError(
            f"the step before ends {run_up:.2f} m behind HOME; the home leg drives at most "
            f"{HOME_MAX_RUNUP_M:.2f} m: end the route closer to HOME"
        )
    if abs(err) > math.radians(HOME_MAX_HEADING_DEG) + 1e-12:
        raise RouteError(
            f"the vehicle faces {math.degrees(err):+.1f} deg off HOME's heading; the home leg "
            f"allows {HOME_MAX_HEADING_DEG:g} deg: add a turn before it"
        )
    slope0 = math.tan(err)
    if abs(offset) <= STRAIGHT_TOL_M and abs(run_up * slope0) <= STRAIGHT_TOL_M:
        n = max(1, int(math.ceil(run_up / spacing)))
        samples = [(xh - c * run_up * (1 - k / n), yh - s * run_up * (1 - k / n), yawh) for k in range(n + 1)]
        return HomeLeg(samples, run_up, run_up, offset, err, math.inf, True)
    steep, rmin = _shape(offset, run_up, slope0)
    what = (
        f"{abs(offset) * 1000:.0f} mm sideways and {math.degrees(err):+.1f} deg "
        f"over a {run_up * 1000:.0f} mm run-up"
    )
    if steep > math.tan(math.radians(HOME_MAX_SLOPE_DEG)):
        raise RouteError(
            f"{what}: the S-curve would point {math.degrees(math.atan(steep)):.0f} deg off HOME's "
            f"line (at most {HOME_MAX_SLOPE_DEG:g}); {_suggest(offset, run_up, slope0)}"
        )
    if rmin < HOME_MIN_RADIUS_M:
        raise RouteError(
            f"{what} needs a {rmin:.2f} m radius, tighter than {HOME_MIN_RADIUS_M:.2f} m; "
            f"{_suggest(offset, run_up, slope0)}"
        )
    # x steps short enough that no two samples are farther apart than `spacing` along the path
    n = max(2, int(math.ceil(run_up / (spacing * math.cos(math.atan(steep))))))
    samples = []
    for k in range(n + 1):
        t = k / n
        y, dydx, _ = _profile(t, offset, run_up, slope0)
        xl = -run_up * (1.0 - t)
        samples.append((xh + c * xl - s * y, yh + s * xl + c * y, _wrap(yawh + math.atan(dydx))))
    samples[0], samples[-1] = (x0, y0, yaw0), (xh, yh, yawh)
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(samples, samples[1:], strict=False))
    return HomeLeg(samples, length, run_up, offset, err, rmin, False)


def along_cross(samples, pose) -> tuple[float, float]:
    """Progress along a sampled path (metres from its first sample) and the signed lateral
    offset of `pose` (+ = left of travel), from the nearest segment. The first and last
    segments are extended, so a pose before the start reads negative and an overshoot reads
    past the length, as on a straight."""
    best: tuple[float, float, float] | None = None
    s0, last = 0.0, len(samples) - 2
    for i in range(last + 1):
        ax, ay = samples[i][0], samples[i][1]
        sx, sy = samples[i + 1][0] - ax, samples[i + 1][1] - ay
        seg = math.hypot(sx, sy)
        if seg <= 1e-12:
            continue
        tx, ty = sx / seg, sy / seg
        px, py = pose[0] - ax, pose[1] - ay
        a = px * tx + py * ty
        a = max(a, 0.0) if i > 0 else a
        a = min(a, seg) if i < last else a
        d = math.hypot(px - tx * a, py - ty * a)
        if best is None or d < best[0]:
            best = (d, s0 + a, -px * ty + py * tx)
        s0 += seg
    return (0.0, 0.0) if best is None else (best[1], best[2])
