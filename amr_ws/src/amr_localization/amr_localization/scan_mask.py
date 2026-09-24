"""Angle mask for one scanner's known phantom returns (scratched optics cover). Pure.

A scratch on the nanoScan3's window scatters the beams that cross it: at those angles the
scanner reports a return that is not there, at a short range, and it moves with the robot.
A mask is a list of sectors (from_deg, to_deg, max_range_m): a reading inside a sector
and closer than its max_range becomes NaN ("no reading" - skipped by slam_toolbox, AMCL
and the costmap). Farther returns in the same sector are kept, so a real wall behind the
phantom still counts.

The mask belongs to the physical scanner, not to the code: it lives on the robot
(~/.amr/scan_mask.yaml), is learnt there by `scan_mask_learn` with the robot standing in
open space, and must be learnt again after the optics cover is replaced.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

NAN = float("nan")


@dataclass(frozen=True)
class Sector:
    from_deg: float
    to_deg: float
    max_range_m: float

    def as_list(self) -> list[float]:
        return [round(self.from_deg, 2), round(self.to_deg, 2), round(self.max_range_m, 3)]


def apply_mask(
    ranges: Sequence[float], angle_min: float, angle_inc: float, sectors: Sequence[Sector]
) -> tuple[list[float], int]:
    """(masked ranges, number removed)."""
    out = list(ranges)
    if not sectors or angle_inc == 0.0:
        return out, 0
    removed = 0
    n = len(out)
    for s in sectors:
        a0, a1 = math.radians(min(s.from_deg, s.to_deg)), math.radians(max(s.from_deg, s.to_deg))
        i0 = math.ceil((a0 - angle_min) / angle_inc - 1e-9)
        i1 = math.floor((a1 - angle_min) / angle_inc + 1e-9)
        if angle_inc < 0:
            i0, i1 = i1, i0
        for i in range(max(0, i0), min(n - 1, i1) + 1):
            r = out[i]
            if math.isfinite(r) and r < s.max_range_m:
                out[i] = NAN
                removed += 1
    return out, removed


def learn(
    scans: Sequence[Sequence[float]],
    angle_min: float,
    angle_inc: float,
    range_min: float,
    clear_m: float,
    min_fraction: float = 0.2,
    margin_m: float = 0.10,
    pad_beams: int = 2,
    join_gap: int = 3,
) -> list[Sector]:
    """Sectors from scans taken with NOTHING real within clear_m of the scanner.

    A beam that returned closer than clear_m in at least min_fraction of the scans is a
    phantom. Neighbouring phantom beams (gaps up to join_gap) form one sector, padded by
    pad_beams either side; its max_range is the farthest phantom reading plus margin_m,
    never beyond clear_m."""
    if not scans:
        return []
    n = min(len(s) for s in scans)
    hits = [0] * n
    far = [0.0] * n
    for s in scans:
        for i in range(n):
            r = s[i]
            if math.isfinite(r) and range_min <= r < clear_m:
                hits[i] += 1
                far[i] = max(far[i], r)
    bad = [i for i in range(n) if hits[i] >= min_fraction * len(scans)]
    groups: list[list[int]] = []
    for i in bad:
        if groups and i - groups[-1][-1] <= join_gap:
            groups[-1].append(i)
        else:
            groups.append([i])
    sectors = []
    for g in groups:
        i0, i1 = max(0, g[0] - pad_beams), min(n - 1, g[-1] + pad_beams)
        a0 = math.degrees(angle_min + i0 * angle_inc)
        a1 = math.degrees(angle_min + i1 * angle_inc)
        rmax = min(clear_m, max(far[i] for i in g) + margin_m)
        sectors.append(Sector(min(a0, a1), max(a0, a1), rmax))
    return sectors


def to_yaml_dict(sectors: Sequence[Sector], note: str = "") -> dict:
    return {"note": note, "sectors": [s.as_list() for s in sectors]}


def from_yaml_dict(d: dict | None) -> list[Sector]:
    out = []
    for row in (d or {}).get("sectors") or []:
        a, b, r = (float(x) for x in row)
        if not (math.isfinite(a) and math.isfinite(b) and math.isfinite(r)) or r <= 0:
            raise ValueError(f"bad scan mask sector {row!r}")
        out.append(Sector(a, b, r))
    return out


def load(path: str) -> list[Sector]:
    import yaml  # noqa: PLC0415

    with open(path) as f:
        return from_yaml_dict(yaml.safe_load(f))
