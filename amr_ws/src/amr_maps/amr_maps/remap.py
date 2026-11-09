"""Re-map area (2026-11-09): replace part of a saved map with what the latest survey saw. Pure.

A revision made by Continue mapping carries `survey_scan` (amr_maps.survey_scan): the new
drive's own scans as a grid on the map's geometry. Re-map area takes boxes the operator drew
and, inside them, sets every cell the new drive saw (free or occupied) to what it saw; cells
it never saw keep the old map. Outside the boxes nothing changes.

Before that, check_alignment asks whether the new scan lines up with the old map AROUND the
boxes, where the floor did not change: of the old map's occupied cells in a band around the
boxes that the new drive saw, how many it also saw occupied (within one cell), and whether a
small shift would line them up better. A survey that slipped in a stretch with little fixed
structure would otherwise paste that stretch in at the wrong place.
"""

from __future__ import annotations

import math

import numpy as np

from amr_maps import edit
from amr_maps.grid import Grid, rasterize_polygon, require_axis_aligned

BAND_M = 1.0  # the ring around the boxes the alignment is checked in
MIN_CHECK_CELLS = 30  # fewer old occupied cells seen there: nothing to check against
AGREE_MIN = 0.6
SHIFT_CELLS = 4  # also try shifts of up to this many cells (20 cm at 5 cm)
SHIFT_GAIN = 0.15  # a shift that agrees this much better means the scan is displaced
MAX_BOXES = 20


def normalize_boxes(raw) -> list[list[list[float]]]:
    """Checked polygons (edit.normalize_ops's coordinate rules). Raises edit.EditError."""
    if not isinstance(raw, list) or not raw:
        raise edit.EditError("draw at least one re-map box")
    if len(raw) > MAX_BOXES:
        raise edit.EditError(f"at most {MAX_BOXES} re-map boxes")
    ops = edit.normalize_ops([{"op": "dynamic", "polygon": p} for p in raw])
    return [o["polygon"] for o in ops]


def box_mask(grid: Grid, boxes) -> np.ndarray:
    require_axis_aligned(grid.meta)
    m = np.zeros(grid.data.shape, dtype=bool)
    for p in boxes:
        rasterize_polygon(grid, np.asarray(p, dtype=np.float64), m)
    return m


def _dilate(a: np.ndarray, cells: int) -> np.ndarray:
    if cells <= 0:
        return a.copy()
    p = np.pad(a, cells)
    out = np.zeros_like(a)
    h, w = a.shape
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            out |= p[cells + dr : cells + dr + h, cells + dc : cells + dc + w]
    return out


def _shift(a: np.ndarray, dr: int, dc: int) -> np.ndarray:
    out = np.zeros_like(a)
    h, w = a.shape
    rs, rd = (slice(0, h - dr), slice(dr, h)) if dr >= 0 else (slice(-dr, h), slice(0, h + dr))
    cs, cd = (slice(0, w - dc), slice(dc, w)) if dc >= 0 else (slice(-dc, w), slice(0, w + dc))
    out[rd, cd] = a[rs, cs]
    return out


def check_alignment(grid: Grid, scan: Grid, boxes) -> dict:
    """{"ok": True | False | None (nothing to check), "agree", "cells", "shift_m", "text"}."""
    if scan.data.shape != grid.data.shape:
        return {
            "ok": False,
            "agree": 0.0,
            "cells": 0,
            "shift_m": 0.0,
            "text": "the survey scan has another size than this map",
        }
    res = grid.meta.resolution
    inside = box_mask(grid, boxes)
    band = _dilate(inside, max(1, int(round(BAND_M / res)))) & ~inside
    rows, cols = np.nonzero(band)
    if rows.size == 0:
        return {"ok": None, "agree": 0.0, "cells": 0, "shift_m": 0.0, "text": "the boxes are outside the map"}
    pad = SHIFT_CELLS + 1
    r0, r1 = max(0, rows.min() - pad), min(grid.height, rows.max() + pad + 1)
    c0, c1 = max(0, cols.min() - pad), min(grid.width, cols.max() + pad + 1)
    old_occ = grid.data[r0:r1, c0:c1] >= edit.OCCUPIED
    new_occ = scan.data[r0:r1, c0:c1] >= edit.OCCUPIED
    seen = scan.data[r0:r1, c0:c1] >= 0
    ref = old_occ & band[r0:r1, c0:c1] & _dilate(seen, 1)
    n = int(ref.sum())
    if n < MIN_CHECK_CELLS:
        return {
            "ok": None,
            "agree": 0.0,
            "cells": n,
            "shift_m": 0.0,
            "text": f"only {n} fixed map cells around the boxes were seen again: cannot check the "
            "alignment - make the boxes reach machines or walls that did not change",
        }

    def agree(dr: int, dc: int) -> float:
        return float((ref & _dilate(_shift(new_occ, dr, dc), 1)).sum()) / n

    a0 = agree(0, 0)
    best = (a0, 0, 0)
    for dr in range(-SHIFT_CELLS, SHIFT_CELLS + 1):
        for dc in range(-SHIFT_CELLS, SHIFT_CELLS + 1):
            if dr or dc:
                a = agree(dr, dc)
                if a > best[0]:
                    best = (a, dr, dc)
    shift_m = math.hypot(best[1], best[2]) * res
    if best[0] - a0 >= SHIFT_GAIN:
        return {
            "ok": False,
            "agree": a0,
            "cells": n,
            "shift_m": shift_m,
            "text": f"the new scan is displaced about {100 * shift_m:.0f} cm against the map around "
            f"the boxes ({100 * a0:.0f} % agree as it is, {100 * best[0]:.0f} % if moved): "
            "drive the stretch again more slowly, with fixed structure in view",
        }
    if a0 < AGREE_MIN:
        return {
            "ok": False,
            "agree": a0,
            "cells": n,
            "shift_m": 0.0,
            "text": f"only {100 * a0:.0f} % of the fixed structure around the boxes was seen again "
            f"in place (needs {100 * AGREE_MIN:.0f} %): either it changed too (make the boxes bigger) "
            "or the survey slipped",
        }
    return {
        "ok": True,
        "agree": a0,
        "cells": n,
        "shift_m": 0.0,
        "text": f"lines up: {100 * a0:.0f} % of {n} fixed map cells around the boxes seen again in place",
    }


def apply(grid: Grid, scan: Grid, boxes) -> tuple[Grid, int]:
    """(the re-mapped grid, cells that changed). Inside the boxes, what the new drive saw wins."""
    if scan.data.shape != grid.data.shape:
        raise edit.EditError("the survey scan has another size than this map")
    m = box_mask(grid, boxes) & (scan.data >= 0)
    out = grid.data.copy()
    out[m] = np.where(scan.data[m] >= edit.OCCUPIED, 100, 0).astype(out.dtype)
    return Grid(out, grid.meta), int((out != grid.data).sum())
