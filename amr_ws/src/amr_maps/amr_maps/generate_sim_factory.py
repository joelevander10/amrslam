"""Generate the sim_factory world: 30 x 20 m, perimeter wall, four rack rows.

World origin (0, 0) is the survey start mark, placed in the cross-aisle at the
west end so a rectangular loop round rack row B closes back onto it. Axis: x
east, y north. Everything is deterministic; regenerate with
`ros2 run amr_maps generate_sim_factory [out_dir]`.
"""

from __future__ import annotations

import os
import sys

import numpy as np

from amr_maps.grid import Grid, GridMeta, write

RES = 0.05
W_M, H_M = 30.0, 20.0
ORIGIN_X, ORIGIN_Y = -3.0, -10.0  # start mark (0,0) sits 3 m inside the west wall, mid-height


def _fill(data: np.ndarray, meta: GridMeta, x0: float, y0: float, x1: float, y1: float, value: int) -> None:
    c0 = int(round((x0 - meta.origin_x) / meta.resolution))
    c1 = int(round((x1 - meta.origin_x) / meta.resolution))
    r0 = int(round((y0 - meta.origin_y) / meta.resolution))
    r1 = int(round((y1 - meta.origin_y) / meta.resolution))
    data[r0:r1, c0:c1] = value


def build() -> Grid:
    meta = GridMeta(resolution=RES, origin_x=ORIGIN_X, origin_y=ORIGIN_Y)
    data = np.zeros((int(H_M / RES), int(W_M / RES)), dtype=np.int8)  # all free
    x_min, y_min = ORIGIN_X, ORIGIN_Y
    x_max, y_max = ORIGIN_X + W_M, ORIGIN_Y + H_M
    t = 0.2  # wall thickness
    _fill(data, meta, x_min, y_min, x_max, y_min + t, 100)
    _fill(data, meta, x_min, y_max - t, x_max, y_max, 100)
    _fill(data, meta, x_min, y_min, x_min + t, y_max, 100)
    _fill(data, meta, x_max - t, y_min, x_max, y_max, 100)
    # Four rack rows, 16 m long, 1.2 m deep, 3.6 m aisles between them. The
    # start mark (0, 0) is in the aisle between rows B and C, at its west end,
    # so a loop east along the aisle, north past the rack ends, west along the
    # next aisle and south again closes onto the mark.
    for yc in (-7.2, -2.4, 2.4, 7.2):
        _fill(data, meta, 3.0, yc - 0.6, 19.0, yc + 0.6, 100)
    # A machine block at the east end and two columns, so the map has
    # features that are not all axis-aligned rack faces.
    _fill(data, meta, 22.0, -8.0, 25.0, -5.0, 100)
    _fill(data, meta, 22.0, 5.0, 24.0, 7.0, 100)
    for cx, cy in ((21.0, -3.0), (10.0, -9.0), (10.0, 9.0)):
        _fill(data, meta, cx - 0.2, cy - 0.2, cx + 0.2, cy + 0.2, 100)
    return Grid(data, meta)


def main(argv: list[str] | None = None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    out_dir = argv[0] if argv else os.path.join(os.path.dirname(__file__), "..", "worlds", "sim_factory")
    os.makedirs(out_dir, exist_ok=True)
    pgm, yml = write(build(), os.path.join(out_dir, "world"))
    print(f"wrote {pgm} and {yml}")


if __name__ == "__main__":
    main()
