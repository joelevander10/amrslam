"""Operator map edits (dynamic-mapping plan §1.3): ordered ops, the dynamic mask, bounds."""

import numpy as np
import pytest

from amr_maps import edit
from amr_maps import grid as gridio


def small() -> gridio.Grid:
    # 4 m x 2 m at 0.1 m, origin (-1, -1): row r, col c has its centre at (-0.95 + 0.1 c, -0.95 + 0.1 r)
    d = np.zeros((20, 40), dtype=np.int8)
    d[8:12, 18:22] = 100  # a "trolley" around (1.0, 0.0)
    return gridio.Grid(d, gridio.GridMeta(0.1, -1.0, -1.0))


TROLLEY = edit.rectangle(0.75, -0.25, 1.25, 0.25)


def test_rectangle_is_axis_aligned_whatever_the_drag_direction():
    assert edit.rectangle(2.0, 1.0, 0.0, -1.0) == [[0.0, -1.0], [2.0, -1.0], [2.0, 1.0], [0.0, 1.0]]


def test_dynamic_marks_cells_and_leaves_the_map_untouched():
    g = small()
    res = edit.apply(g, None, edit.normalize_ops([{"op": "dynamic", "polygon": TROLLEY}]))
    assert np.array_equal(res.grid.data, g.data)  # marking is not painting
    assert res.dynamic_cells == 25 and res.cells == [25]  # 5 x 5 centres inside 0.5 m x 0.5 m
    assert res.dynamic[g.world_to_cell(1.0, 0.0)] and not res.dynamic[g.world_to_cell(0.0, 0.0)]
    assert g.data[g.world_to_cell(1.0, 0.0)] == 100  # the parent grid is never modified


def test_ops_apply_in_order_later_wins():
    g = small()
    big = edit.rectangle(0.0, -0.5, 2.0, 0.5)
    ops = edit.normalize_ops(
        [
            {"op": "dynamic", "polygon": big},
            {"op": "undynamic", "polygon": TROLLEY},
            {"op": "paint", "value": "unknown", "polygon": TROLLEY},
            {"op": "paint", "value": "free", "polygon": edit.rectangle(0.75, -0.25, 1.0, 0.25)},
        ]
    )
    res = edit.apply(g, None, ops)
    assert not res.dynamic[g.world_to_cell(1.0, 0.0)]  # cleared by the second op
    assert res.dynamic[g.world_to_cell(0.3, 0.0)]
    assert res.grid.data[g.world_to_cell(1.1, 0.0)] == -1  # erased to unknown
    assert res.grid.data[g.world_to_cell(0.85, 0.0)] == 0  # then painted free
    # replay: the same parent and ops give the same result
    again = edit.apply(g, None, ops)
    assert np.array_equal(again.grid.data, res.grid.data) and np.array_equal(again.dynamic, res.dynamic)


def test_parent_mask_is_inherited_and_shape_checked():
    g = small()
    parent = np.zeros(g.data.shape, dtype=bool)
    parent[0, 0] = True
    res = edit.apply(g, parent, edit.normalize_ops([{"op": "dynamic", "polygon": TROLLEY}]))
    assert res.dynamic[0, 0] and res.dynamic_cells == 26
    with pytest.raises(edit.EditError, match="shape"):
        edit.apply(g, np.zeros((3, 3), dtype=bool), [])


def test_polygon_off_the_map_covers_nothing():
    res = edit.apply(
        small(), None, edit.normalize_ops([{"op": "dynamic", "polygon": edit.rectangle(50, 50, 51, 51)}])
    )
    assert res.cells == [0] and res.dynamic_cells == 0


@pytest.mark.parametrize(
    "raw, match",
    [
        ([], "no edits"),
        ("x", "must be a list"),
        ([{"op": "erase", "polygon": TROLLEY}], "op must be one of"),
        ([{"op": "dynamic", "polygon": [[0, 0], [1, 1]]}], "3..64"),
        ([{"op": "dynamic", "polygon": [[0, 0], [1, float("nan")], [1, 0]]}], "finite"),
        ([{"op": "dynamic", "polygon": [[0, 0], [1e6, 0], [1, 1]]}], "beyond"),
        ([{"op": "dynamic", "polygon": [[0, 0], [True, 0], [1, 1]]}], "finite"),
        ([{"op": "paint", "value": "wall", "polygon": TROLLEY}], "paint value"),
        ([{"op": "dynamic", "polygon": TROLLEY}] * (edit.MAX_OPS + 1), "at most"),
    ],
)
def test_malformed_ops_are_refused(raw, match):
    with pytest.raises(edit.EditError, match=match):
        edit.normalize_ops(raw)


def test_mask_grid_round_trips_through_pgm(tmp_path):
    g = small()
    res = edit.apply(g, None, edit.normalize_ops([{"op": "dynamic", "polygon": TROLLEY}]))
    gridio.write(edit.mask_grid(g, res.dynamic), str(tmp_path / "dynamic"))
    back = gridio.read(str(tmp_path / "dynamic.yaml"))
    assert gridio.mask_misalignment(g, back) is None
    assert np.array_equal(back.data >= 65, res.dynamic)
