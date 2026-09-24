"""Isolated-return (speckle) filter for a LaserScan's ranges. Pure.

A lone return in open space - dust, a mixed pixel at an edge, a stray reflection - has no
neighbouring beam at a similar range; every real surface the scanner resolves (a wall, a
rack leg, a pallet) is hit by at least two adjacent beams. A return with fewer than
`min_neighbours` similar ranges within `window` beams either side becomes NaN, the ROS
"no reading" value that slam_toolbox, AMCL and the costmap skip (it neither marks a hit
nor clears space along the beam).

"Similar" is |r_j - r_i| <= abs_m + rel * r_i: the tolerance grows with range, because
adjacent beams on a surface seen at an angle differ more far away.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

NAN = float("nan")


def despeckle(
    ranges: Sequence[float],
    range_min: float,
    range_max: float,
    window: int = 2,
    min_neighbours: int = 1,
    abs_m: float = 0.05,
    rel: float = 0.03,
) -> tuple[list[float], int]:
    """(filtered ranges, number removed). Out-of-range and non-finite readings pass unchanged
    and never count as a neighbour."""
    r = list(ranges)
    n = len(r)

    def valid(x: float) -> bool:
        return math.isfinite(x) and range_min <= x <= range_max

    ok = [valid(x) for x in r]
    out = list(r)
    removed = 0
    for i in range(n):
        if not ok[i]:
            continue
        ri = r[i]
        tol = abs_m + rel * ri
        count = 0
        for j in range(max(0, i - window), min(n, i + window + 1)):
            if j != i and ok[j] and abs(r[j] - ri) <= tol:
                count += 1
                if count >= min_neighbours:
                    break
        if count < min_neighbours:
            out[i] = NAN
            removed += 1
    return out, removed
