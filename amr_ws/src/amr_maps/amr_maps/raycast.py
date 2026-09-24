"""Raycasting a 2-D laser against an occupancy grid. Pure numpy.

All beams are marched together: sample points at `step` spacing along every
ray, look up the grid, take the first occupied sample per beam. Accuracy is
±step/2 in range, which is why step defaults to half a cell. Cost is
beams x (max_range / step) lookups per scan; at 550 beams, 20 m and 0.025 m
that is 440 k lookups, a few ms in numpy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from amr_maps.grid import Grid


@dataclass(frozen=True)
class ScanGeometry:
    angle_min: float  # rad, in the laser frame
    angle_max: float
    beams: int
    range_min: float
    range_max: float

    @property
    def angle_increment(self) -> float:
        return (self.angle_max - self.angle_min) / (self.beams - 1)

    @property
    def angles(self) -> np.ndarray:
        return np.linspace(self.angle_min, self.angle_max, self.beams)


# nanoScan3: 275 deg, from -47.5 deg (behind-right) to +227.5 deg, i.e. the
# 85 deg dead sector faces backwards. Expressed symmetric about the forward
# axis: -137.5 .. +137.5 deg. The wire count is 1652 at 1/6 deg; the sim
# default is coarser and configurable (spec §8: "configurable scan geometry").
# On this vehicle a wall behind the mount limits the usable window to about
# +/-95 deg (operator, 2026-09-16): that is what the sim and the driver use.
NANOSCAN3_FOV = math.radians(275.0)
MOUNTED_FOV = math.radians(190.0)


def nanoscan3(beams: int = 551, range_max: float = 30.0) -> ScanGeometry:
    """The device's full geometry."""
    return ScanGeometry(-NANOSCAN3_FOV / 2, NANOSCAN3_FOV / 2, beams, 0.05, range_max)


def mounted(beams: int = 381, range_max: float = 30.0) -> ScanGeometry:
    """The usable window on this vehicle: 190 deg at 0.5 deg."""
    return ScanGeometry(-MOUNTED_FOV / 2, MOUNTED_FOV / 2, beams, 0.05, range_max)


def cast(
    grid: Grid, x: float, y: float, yaw: float, geom: ScanGeometry, step: float | None = None
) -> np.ndarray:
    """Ranges (m) for a laser at world pose (x, y, yaw). inf where nothing is hit."""
    step = step or grid.meta.resolution / 2.0
    n_samples = int(math.ceil(geom.range_max / step)) + 1
    dists = np.arange(n_samples) * step  # [S]
    angles = yaw + geom.angles  # [B]
    px = x + np.outer(np.cos(angles), dists)  # [B, S]
    py = y + np.outer(np.sin(angles), dists)
    cols = np.floor((px - grid.meta.origin_x) / grid.meta.resolution).astype(np.int64)
    rows = np.floor((py - grid.meta.origin_y) / grid.meta.resolution).astype(np.int64)
    inside = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
    occ = np.zeros(px.shape, dtype=bool)
    occ[inside] = grid.data[rows[inside], cols[inside]] >= 65
    # Leaving the grid counts as a hit at the boundary? No: outside is unknown,
    # the beam just reports no return (inf), like a real scanner into open air.
    hit = occ.any(axis=1)
    first = np.where(hit, occ.argmax(axis=1), n_samples - 1)
    ranges = dists[first].astype(np.float64)
    ranges[~hit] = np.inf
    ranges[ranges < geom.range_min] = geom.range_min
    return ranges
