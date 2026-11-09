"""Re-map area (2026-11-09): the survey's own scans as a grid, and replacing boxes with it."""

import math

import numpy as np

from amr_maps import edit, remap
from amr_maps import grid as gridio
from amr_maps import survey_scan as ss

META = gridio.GridMeta(0.05, -1.0, -3.0)
W, H = 260, 120  # 13 m x 6 m: x -1..12, y -3..3


def corridor_frames(walls=(-2.0, 2.0), gap=None, n_beams=541):
    """A laser driving x 0..10 along y = 0 between two walls; `gap` = (x0, x1) on the upper wall:
    there the beams go on to a back wall at y = +2.8."""
    amin, inc = -math.radians(135), math.radians(270) / (n_beams - 1)
    ang = amin + inc * np.arange(n_beams)
    frames = []
    for i in range(51):
        x = 0.2 * i
        s = np.sin(ang)
        r = np.full(n_beams, np.inf)
        up, dn = s > 1e-3, s < -1e-3
        r[up] = walls[1] / s[up]
        r[dn] = walls[0] / s[dn]
        if gap is not None:
            hx = x + r * np.cos(ang)
            through = up & (hx >= gap[0]) & (hx <= gap[1])
            r[through] = 2.8 / s[through]
        frames.append(ss.Keyframe(x, 0.0, 0.0, amin, inc, r.astype(np.float32)))
    return frames


def cell(g, x, y):
    return g.data[g.world_to_cell(x, y)]


def test_rasterize_sees_walls_occupied_and_the_floor_free():
    g = ss.rasterize(META, W, H, corridor_frames())
    assert cell(g, 5.0, 2.02) == 100 and cell(g, 5.0, -1.98) == 100
    assert cell(g, 5.0, 0.0) == 0 and cell(g, 5.0, 1.0) == 0
    assert cell(g, 5.0, 2.6) == -1  # behind the wall: never seen


def test_a_removed_pallet_row_is_free_in_the_new_scan():
    g = ss.rasterize(META, W, H, corridor_frames(gap=(4.0, 7.0)))
    assert cell(g, 5.5, 2.02) == 0  # the wall/pallets that stood there: looked through, free
    assert cell(g, 5.5, 2.82) == 100  # what is behind it now


def test_keyframes_round_trip_as_arrays():
    f = corridor_frames()[:3]
    back = ss.from_arrays(**ss.to_arrays(f))
    assert len(back) == 3 and back[1].x == f[1].x and np.array_equal(back[2].ranges, f[2].ranges)


def old_map():
    d = np.zeros((H, W), dtype=np.int8)
    g = gridio.Grid(d, META)
    for x in np.arange(-0.9, 12.0, 0.05):
        d[g.world_to_cell(x, 2.02)] = 100
        d[g.world_to_cell(x, -1.98)] = 100
    return g


def test_remap_replaces_only_inside_the_boxes_what_the_new_drive_saw():
    old = old_map()
    scan = ss.rasterize(META, W, H, corridor_frames(gap=(4.0, 7.0)))
    box = edit.rectangle(3.5, 1.5, 7.5, 3.0)
    chk = remap.check_alignment(old, scan, [box])
    assert chk["ok"] is True, chk["text"]
    new, changed = remap.apply(old, scan, [box])
    assert changed > 0
    assert cell(new, 5.5, 2.02) == 0 and cell(new, 5.5, 2.82) == 100  # inside: as the new drive saw
    assert cell(new, 2.0, 2.02) == 100  # outside: the old map
    gap_outside = remap.apply(old, scan, [edit.rectangle(0.0, -1.0, 1.0, 1.0)])[0]
    assert cell(gap_outside, 5.5, 2.02) == 100  # the gap is not in that box: unchanged


def test_a_displaced_survey_is_refused():
    old = old_map()
    frames = corridor_frames(gap=(4.0, 7.0))
    for f in frames:
        f.y += 0.15  # the survey slipped 15 cm sideways
    scan = ss.rasterize(META, W, H, frames)
    chk = remap.check_alignment(old, scan, [edit.rectangle(3.5, 1.5, 7.5, 3.0)])
    assert chk["ok"] is False and "displaced" in chk["text"], chk


def test_nothing_fixed_around_the_box_cannot_be_checked():
    old = gridio.Grid(np.zeros((H, W), dtype=np.int8), META)
    scan = ss.rasterize(META, W, H, corridor_frames())
    chk = remap.check_alignment(old, scan, [edit.rectangle(3.5, -1.0, 7.5, 1.0)])
    assert chk["ok"] is None and "cannot check" in chk["text"]


def test_boxes_are_checked():
    import pytest  # noqa: PLC0415

    with pytest.raises(edit.EditError):
        remap.normalize_boxes([])
    with pytest.raises(edit.EditError):
        remap.normalize_boxes([[[0, 0], [1, 0]]])
    assert remap.normalize_boxes([edit.rectangle(0, 0, 1, 1)]) == [
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
    ]
