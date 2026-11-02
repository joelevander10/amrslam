"""Operator map edits: paint cells and mark dynamic areas (dynamic-mapping plan, Increment 1).

Pure numpy; no files, no bundles (amr_mission.map_bundle.derive_edit writes the result
as a NEW revision). An edit is an ordered list of polygon operations in map metres:

    {"op": "paint", "value": "free" | "unknown" | "occupied", "polygon": [[x, y], ...]}
    {"op": "dynamic",   "polygon": [...]}   mark: the scan there may change (trolleys, forklifts)
    {"op": "undynamic", "polygon": [...]}   clear a mark
    {"op": "despeckle", "polygon": [...], "max_cells": 4}
                                            remove NOISE: inside the polygon, every occupied
                                            blob of at most max_cells cells (8-connected,
                                            counted over the whole map) becomes free. A wall
                                            or rack is a long connected run and stays; the
                                            lone dots a scanner leaves in open floor go.

Operations apply in order, so a later one wins where polygons overlap, and the list
replays: the same parent grid and the same ops always give the same result (edits.json
keeps them for that). A cell belongs to a polygon when its CENTRE lies inside
(grid.rasterize_polygon, the rule footprint sweeps use too).

Painting "free" is an operator assertion ("I know this is floor"); painting "unknown" is
the safe erase: unknown blocks route approval unless it lies in a dynamic area.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from amr_maps.grid import Grid, rasterize_polygon, require_axis_aligned

PAINT_VALUES = {"free": 0, "unknown": -1, "occupied": 100}
OPS = ("paint", "dynamic", "undynamic", "despeckle")
OCCUPIED = 65  # the map's occupied threshold (map_server occupied_thresh 0.65)
DESPECKLE_DEFAULT_CELLS = 4  # 4 cells of 5 cm: a blob up to ~10 x 10 cm
DESPECKLE_MAX_CELLS = 50
MAX_OPS = 500
MAX_VERTICES = 64
MAX_ABS_COORD_M = 10_000.0


class EditError(ValueError):
    pass


@dataclass
class EditResult:
    grid: Grid  # the edited occupancy (a copy; the parent is never modified)
    dynamic: np.ndarray  # bool [rows, cols]: dynamic-area cells
    cells: list[int]  # map cells each op covered, in op order (0 = the polygon missed the map)

    @property
    def dynamic_cells(self) -> int:
        return int(self.dynamic.sum())


def normalize_ops(raw) -> list[dict]:
    """Checked, canonical copies of `raw` (JSON-like). Raises EditError naming the op."""
    if not isinstance(raw, list):
        raise EditError("ops must be a list")
    if not raw:
        raise EditError("no edits")
    if len(raw) > MAX_OPS:
        raise EditError(f"at most {MAX_OPS} edits at once, got {len(raw)}")
    out = []
    for i, d in enumerate(raw):
        where = f"edit {i + 1}"
        if not isinstance(d, dict):
            raise EditError(f"{where}: must be an object")
        op = d.get("op")
        if op not in OPS:
            raise EditError(f"{where}: op must be one of {OPS}, got {op!r}"[:200])
        poly = d.get("polygon")
        if not isinstance(poly, list) or not 3 <= len(poly) <= MAX_VERTICES:
            raise EditError(f"{where}: polygon needs 3..{MAX_VERTICES} [x, y] points")
        pts = []
        for p in poly:
            if not (isinstance(p, (list, tuple)) and len(p) == 2):
                raise EditError(f"{where}: polygon points are [x, y]")
            xy = []
            for v in p:
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                    raise EditError(f"{where}: polygon coordinates must be finite numbers")
                if abs(v) > MAX_ABS_COORD_M:
                    raise EditError(f"{where}: polygon coordinate {v:g} beyond {MAX_ABS_COORD_M:g} m")
                xy.append(float(v))
            pts.append(xy)
        item = {"op": op, "polygon": pts}
        if op == "despeckle":
            n = d.get("max_cells", DESPECKLE_DEFAULT_CELLS)
            if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= DESPECKLE_MAX_CELLS:
                raise EditError(f"{where}: max_cells must be an integer 1..{DESPECKLE_MAX_CELLS}")
            item["max_cells"] = n
        if op == "paint":
            value = d.get("value")
            if value not in PAINT_VALUES:
                raise EditError(
                    f"{where}: paint value must be one of {tuple(PAINT_VALUES)}, got {value!r}"[:200]
                )
            item["value"] = value
        out.append(item)
    return out


def rectangle(x0: float, y0: float, x1: float, y1: float) -> list[list[float]]:
    """The polygon of an axis-aligned rectangle given two opposite corners."""
    xa, xb, ya, yb = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
    return [[xa, ya], [xb, ya], [xb, yb], [xa, yb]]


def apply(grid: Grid, dynamic: np.ndarray | None, ops: list[dict]) -> EditResult:
    """Apply normalized `ops` in order to copies of `grid` and the `dynamic` mask."""
    require_axis_aligned(grid.meta)
    data = grid.data.copy()
    dyn = np.zeros(data.shape, dtype=bool) if dynamic is None else dynamic.astype(bool).copy()
    if dyn.shape != data.shape:
        raise EditError(f"dynamic mask shape {dyn.shape} != map {data.shape}")
    cells = []
    for d in ops:
        m = np.zeros(data.shape, dtype=bool)
        rasterize_polygon(grid, np.asarray(d["polygon"], dtype=np.float64), m)
        cells.append(int(m.sum()))
        if d["op"] == "paint":
            data[m] = PAINT_VALUES[d["value"]]
        elif d["op"] == "despeckle":
            noise = small_blobs(data >= OCCUPIED, d["max_cells"]) & m
            data[noise] = PAINT_VALUES["free"]
        elif d["op"] == "dynamic":
            dyn |= m
        else:
            dyn &= ~m
    return EditResult(Grid(data, grid.meta), dyn, cells)


def small_blobs(occ: np.ndarray, max_cells: int) -> np.ndarray:
    """Cells of occupied 8-connected components with at most max_cells cells."""
    out = np.zeros(occ.shape, dtype=bool)
    seen = np.zeros(occ.shape, dtype=bool)
    rows, cols = occ.shape
    for r0, c0 in zip(*np.nonzero(occ), strict=True):
        if seen[r0, c0]:
            continue
        comp, stack = [], [(int(r0), int(c0))]
        seen[r0, c0] = True
        while stack:
            r, c = stack.pop()
            comp.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < rows and 0 <= cc < cols and occ[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        if len(comp) <= max_cells:
            for r, c in comp:
                out[r, c] = True
    return out


def mask_grid(grid: Grid, mask: np.ndarray) -> Grid:
    """A mask as a grid in the map's geometry (100 = marked, 0 = not), for gridio.write."""
    data = np.zeros(grid.data.shape, dtype=np.int8)
    data[mask] = 100
    return Grid(data, grid.meta)
