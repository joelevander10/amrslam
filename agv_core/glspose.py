"""SE(2) pose maths for the GLS floor labels (gls-wait-coding-plan §C.2, §D).

Poses are (x, y, yaw) tuples. The frames:

    M  map            B  base_footprint        C  gls_frame (the camera's floor point)
    L  a floor label  (its +X along the label's printed X axis)

    T_BC   the camera mount, from the profile (gls.mount_*)
    T_LC   the camera in the label frame: one GlsReading's (x_m, y_m, yaw_rad)
    T_ML   a label in the map: T_MB (+) T_BC (+) T_LC^-1      (recording, §C.2)
    T_MB   the vehicle in the map from a label: T_ML (+) T_LC (+) T_BC^-1   (a fix, §D)

Free of ROS and of config: the callers pass the mount and the readings in.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap(a: float) -> float:
    """Into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b) -> tuple[float, float, float]:
    """a (+) b: b expressed in a's frame, taken into a's parent."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def inverse(a) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2])


def label_in_map(t_mb, t_bc, t_lc) -> tuple[float, float, float]:
    """T_ML from the vehicle's map pose, the mount and the camera-in-label reading."""
    return compose(compose(t_mb, t_bc), inverse(t_lc))


def base_in_map(t_ml, t_bc, t_lc) -> tuple[float, float, float]:
    """T_MB from a label's map pose, the mount and the camera-in-label reading."""
    return compose(compose(t_ml, t_lc), inverse(t_bc))


def circular_mean(angles: list[float]) -> float:
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


@dataclass(frozen=True)
class Average:
    pose: tuple[float, float, float]
    n: int
    spread_xy_m: float  # largest distance of one pose from the mean
    spread_yaw_rad: float  # largest |yaw - mean yaw|


def average(poses: list) -> Average:
    """Mean of poses that are all estimates of ONE pose (yaw by circular mean), with the
    worst deviation as the spread. Raises ValueError on an empty list."""
    if not poses:
        raise ValueError("no poses to average")
    mx = sum(p[0] for p in poses) / len(poses)
    my = sum(p[1] for p in poses) / len(poses)
    myaw = circular_mean([p[2] for p in poses])
    return Average(
        (mx, my, myaw),
        len(poses),
        max(math.hypot(p[0] - mx, p[1] - my) for p in poses),
        max(abs(wrap(p[2] - myaw)) for p in poses),
    )


@dataclass(frozen=True)
class LabelObservation:
    label_id: str
    t_lc: Average  # the camera in the label frame, averaged over the frames used
    n_codes_min: int  # fewest codes in any frame used


def observe(readings: list, n: int, min_codes: int, spread_xy_m: float, spread_yaw_rad: float):
    """The last `n` readings as ONE label observation, or the reason they are not one.

    `readings` are GlsReading-like (valid, calibrated, label_id, x_m, y_m, yaw_rad,
    n_codes), oldest first; the caller has already kept only those taken while the
    vehicle stood still. Returns (LabelObservation, None) or (None, reason)."""
    if len(readings) < n:
        return None, f"{len(readings)} of {n} label frames: hold still over the label and retry"
    used = readings[-n:]
    if not all(r.valid for r in used):
        return None, "label not seen in every frame"
    if not all(r.calibrated for r in used):
        return None, "the floor-label camera is not calibrated (gls.xy_scale_m_per_count is 0)"
    ids = {r.label_id for r in used}
    if len(ids) != 1:
        return None, f"more than one label seen: {', '.join(sorted(ids))}"
    fewest = min(int(r.n_codes) for r in used)
    if fewest < min_codes:
        return None, f"only {fewest} codes read in a frame (need {min_codes})"
    avg = average([(r.x_m, r.y_m, r.yaw_rad) for r in used])
    if avg.spread_xy_m > spread_xy_m or avg.spread_yaw_rad > spread_yaw_rad:
        return None, (
            f"label frames disagree: {avg.spread_xy_m * 1000:.1f} mm / "
            f"{math.degrees(avg.spread_yaw_rad):.2f} deg (limits {spread_xy_m * 1000:.1f} mm / "
            f"{math.degrees(spread_yaw_rad):.2f} deg)"
        )
    return LabelObservation(ids.pop(), avg, fewest), None
