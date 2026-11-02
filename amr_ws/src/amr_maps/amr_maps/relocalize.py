"""Scan-to-map relocalisation before a survey continues on a saved map. Pure numpy.

slam_toolbox continues a saved pose graph with START_AT_GIVEN_POSE: it matches the first
scan against the nearest saved node only inside its sequential search window
(correlation_search_space_dimension 0.5 m -> +/-0.25 m, coarse_search_angle_offset
+/-20 deg). A pose dragged on the map by a finger, or "where the last survey ended" after
someone nudged the vehicle, is not that good. This searches a wider window around the hint
for the pose where the live scan best fits the saved map, and says how sure it is, BEFORE
slam_toolbox is handed the pose: a wrong start pose would weld the new scans onto the old
graph in the wrong place, and nothing downstream could undo that.

Score of a pose: the mean over scan points of a proximity field that is 1 on an occupied
cell and falls linearly to 0 at `tol_m` from the nearest one. Points that land in free,
unknown or off-map cells score 0, so a scan that mostly sees unmapped space scores low by
construction: the operator must start where the saved map is.

Search: every translation on the grid lattice (integer cell shifts, so one gather per
angle) and every angle step inside the window, then a finer angle pass around the best.
`runner_up` is the best score at least `sep_m` / `sep_deg` away from the winner; when it
comes close to the winner the match is ambiguous (a featureless corridor, a repeated rack
row) and refused rather than guessed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from amr_maps.grid import Grid

OCCUPIED = 65


@dataclass(frozen=True)
class MatchParams:
    window_m: float = 1.5  # +/- around the hint
    window_deg: float = 30.0
    step_deg: float = 1.0  # coarse angle step; translation steps are one grid cell
    fine_step_deg: float = 0.1
    tol_m: float = 0.15  # proximity reaches 0 here
    max_points: int = 240  # scan points used (evenly thinned)
    min_points: int = 40
    min_score: float = 0.5  # placeholder: settled on the vehicle
    ambiguity_ratio: float = 0.9  # runner_up >= ratio * best -> ambiguous
    sep_m: float = 0.5
    sep_deg: float = 10.0


@dataclass(frozen=True)
class MatchResult:
    ok: bool
    reason: str  # empty when ok
    x: float
    y: float
    yaw: float
    score: float
    runner_up: float
    hint_score: float  # the score at the hint itself, for the operator's message
    points: int

    def text(self) -> str:
        return (
            f"scan match {self.score:.2f} at ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.1f} deg), "
            f"runner-up {self.runner_up:.2f}, at the hint {self.hint_score:.2f}, {self.points} points"
        )


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def scan_points(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_pose: tuple[float, float, float],
) -> np.ndarray:
    """Valid returns as (N, 2) points in the vehicle frame (base_footprint)."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(r.size)
    keep = np.isfinite(r) & (r >= range_min) & (r < range_max)
    r, a = r[keep], a[keep]
    lx, ly, lyaw = laser_pose
    c, s = math.cos(lyaw), math.sin(lyaw)
    px, py = r * np.cos(a), r * np.sin(a)
    return np.stack([lx + c * px - s * py, ly + s * px + c * py], axis=1)


def proximity(grid: Grid, tol_m: float) -> np.ndarray:
    """float32 field: 1 on occupied cells, linear to 0 at tol_m (Chebyshev rings)."""
    occ = grid.data >= OCCUPIED
    k = max(1, int(round(tol_m / grid.meta.resolution)))
    field = occ.astype(np.float32)
    reach = occ.copy()
    h, w = occ.shape
    for ring in range(1, k + 1):
        p = np.pad(reach, 1)
        grown = np.zeros_like(reach)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                grown |= p[1 + dr : 1 + dr + h, 1 + dc : 1 + dc + w]
        new = grown & ~reach
        field[new] = 1.0 - ring / (k + 1)
        reach = grown
    return field


def _box3(s: np.ndarray) -> np.ndarray:
    """3x3 mean over the translation axes of [angle, dy, dx], edges replicated."""
    p = np.pad(s, ((0, 0), (1, 1), (1, 1)), mode="edge")
    _, h, w = s.shape
    out = np.zeros_like(s)
    for dr in (0, 1, 2):
        for dc in (0, 1, 2):
            out += p[:, dr : dr + h, dc : dc + w]
    return out / 9.0


def _local_maxima(s: np.ndarray) -> np.ndarray:
    """Cells >= all 26 neighbours (3-D), the window's outside counting as -inf."""
    p = np.pad(s, 1, constant_values=-np.inf)
    a, h, w = s.shape
    peak = np.ones(s.shape, dtype=bool)
    for da in (-1, 0, 1):
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if da or dr or dc:
                    peak &= s >= p[1 + da : 1 + da + a, 1 + dr : 1 + dr + h, 1 + dc : 1 + dc + w]
    return peak


def _thin(points: np.ndarray, n: int) -> np.ndarray:
    if len(points) <= n:
        return points
    return points[np.linspace(0, len(points) - 1, n).round().astype(int)]


class _Scorer:
    """Scores of all lattice translations within +/-half cells, for one rotation."""

    def __init__(self, grid: Grid, field: np.ndarray, half: int) -> None:
        self.res = grid.meta.resolution
        self.ox, self.oy = grid.meta.origin_x, grid.meta.origin_y
        self.half = half
        self.pad = np.pad(field, half)  # out of the map scores 0
        self.h, self.w = field.shape
        self.offs = np.arange(-half, half + 1)

    def surface(self, pts: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
        """[2*half+1 (dy), 2*half+1 (dx)] mean proximity, centred on (x, y)."""
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * pts[:, 0] - s * pts[:, 1]
        wy = y + s * pts[:, 0] + c * pts[:, 1]
        col = np.floor((wx - self.ox) / self.res).astype(np.int64) + self.half
        row = np.floor((wy - self.oy) / self.res).astype(np.int64) + self.half
        # points that would leave the padded field at some shift score 0 there: clamp them
        # onto the zero border instead of indexing out of range
        rows = row[:, None] + self.offs[None, :]  # [N, dy]
        cols = col[:, None] + self.offs[None, :]  # [N, dx]
        rows_ok = (rows >= 0) & (rows < self.pad.shape[0])
        cols_ok = (cols >= 0) & (cols < self.pad.shape[1])
        rows = np.clip(rows, 0, self.pad.shape[0] - 1)
        cols = np.clip(cols, 0, self.pad.shape[1] - 1)
        vals = self.pad[rows[:, :, None], cols[:, None, :]]  # [N, dy, dx]
        vals = vals * (rows_ok[:, :, None] & cols_ok[:, None, :])
        return vals.mean(axis=0)


def match(
    grid: Grid,
    points: np.ndarray,
    hint: tuple[float, float, float],
    params: MatchParams | None = None,
    field: np.ndarray | None = None,
) -> MatchResult:
    """Best pose near `hint` for scan `points` (vehicle frame) on `grid`."""
    p = params or MatchParams()
    hx, hy, hyaw = (float(v) for v in hint)
    if not all(math.isfinite(v) for v in (hx, hy, hyaw)):
        return MatchResult(False, "the pose hint is not a finite pose", hx, hy, hyaw, 0.0, 0.0, 0.0, 0)
    pts = _thin(np.asarray(points, dtype=np.float64).reshape(-1, 2), p.max_points)
    if len(pts) < p.min_points:
        return MatchResult(
            False, f"only {len(pts)} scan returns (need {p.min_points}): is the scanner seeing anything?",
            hx, hy, hyaw, 0.0, 0.0, 0.0, len(pts),
        )
    field = proximity(grid, p.tol_m) if field is None else field
    res = grid.meta.resolution
    half = max(1, int(round(p.window_m / res)))
    scorer = _Scorer(grid, field, half)
    n_ang = int(round(p.window_deg / p.step_deg))
    angles = hyaw + np.radians(p.step_deg) * np.arange(-n_ang, n_ang + 1)
    surfaces = np.stack([scorer.surface(pts, hx, hy, a) for a in angles])  # [A, dy, dx]
    hint_score = float(surfaces[n_ang, half, half])

    ai, dyi, dxi = np.unravel_index(int(np.argmax(surfaces)), surfaces.shape)
    best = float(surfaces[ai, dyi, dxi])
    # runner-up: the best OTHER PEAK (a local maximum) clearly separated from the winner in
    # position or heading. Not merely the best distant pose: sliding a scan 0.5 m along a
    # long wall keeps most points on it, so the winner's own shoulder scores high without
    # being a second place the vehicle could be. A flat ridge (corridor) is all ties, and
    # ties count as maxima, so it is still caught.
    # Peaks are found on a 3x3-cell box-smoothed surface: cell quantization puts ripples on a
    # sloping ridge that would otherwise count as maxima.
    dpos = np.hypot(*np.meshgrid((scorer.offs - (dyi - half)) * res, (scorer.offs - (dxi - half)) * res,
                                 indexing="ij"))
    dang = np.abs(np.degrees(angles - angles[ai]))
    far = (dpos[None, :, :] > p.sep_m) | (dang[:, None, None] > p.sep_deg)
    peaks = far & _local_maxima(_box3(surfaces))
    runner_up = float(surfaces[peaks].max()) if peaks.any() else 0.0  # raw scores compared

    # fine angle pass around the winner, translations +/-2 cells
    bx, by = hx + (dxi - half) * res, hy + (dyi - half) * res
    fine_half = 2
    fine = _Scorer(grid, field, fine_half)
    n_fine = int(round(p.step_deg / p.fine_step_deg))
    f_angles = angles[ai] + np.radians(p.fine_step_deg) * np.arange(-n_fine, n_fine + 1)
    f_surf = np.stack([fine.surface(pts, bx, by, a) for a in f_angles])
    fa, fdy, fdx = np.unravel_index(int(np.argmax(f_surf)), f_surf.shape)
    if f_surf[fa, fdy, fdx] > best:
        best = float(f_surf[fa, fdy, fdx])
        bx, by = bx + (fdx - fine_half) * res, by + (fdy - fine_half) * res
        byaw = float(f_angles[fa])
    else:
        byaw = float(angles[ai])
    byaw = wrap(byaw)

    reason = ""
    if best < p.min_score:
        reason = (
            f"the scan fits the saved map poorly here (score {best:.2f} < {p.min_score:.2f}): "
            "check the pose hint, or start further inside the mapped area"
        )
    elif runner_up >= p.ambiguity_ratio * best:
        reason = (
            f"ambiguous: another pose {p.sep_m:g} m / {p.sep_deg:g} deg or more away fits almost as well "
            f"({runner_up:.2f} vs {best:.2f}); start where walls run in two directions, or give a closer hint"
        )
    return MatchResult(not reason, reason, bx, by, byaw, best, runner_up, hint_score, len(pts))
