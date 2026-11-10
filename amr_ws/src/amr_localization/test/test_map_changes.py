"""Where the map no longer fits, learnt during runs (map_changes.py, 2026-11-09)."""

import math

import numpy as np

from amr_localization import map_changes as mc

UNMAPPED, THROUGH, ON_MAP = mc.UNMAPPED, mc.THROUGH, mc.ON_MAP


def scan(cls, x, y, ex=None, ey=None):
    n = len(cls)
    ex = np.full(n, np.nan) if ex is None else np.asarray(ex, float)
    ey = np.full(n, np.nan) if ey is None else np.asarray(ey, float)
    return np.asarray(x, float), np.asarray(y, float), np.asarray(cls), ex, ey


def drive(cm, n, frame, x0=0.0):
    for i in range(n):
        cm.add((x0 + 0.25 * i, 0.0, 0.0), *frame(i), now=100.0)


def test_counts_once_per_place_not_per_time():
    cm = mc.ChangeMap(200, 100, 0.05, 0.0, -2.5)  # 10 m x 5 m
    f = scan([UNMAPPED], [5.0], [1.0])
    assert cm.add((0.0, 0.0, 0.0), *f)
    assert not cm.add((0.05, 0.0, 0.0), *f)  # parked: not again
    assert cm.add((0.0, 0.0, math.radians(15)), *f)  # turned: again
    assert cm.new.sum() == 2.0 and cm.scans == 2


def test_gone_is_counted_where_the_map_expected_the_wall():
    cm = mc.ChangeMap(200, 100, 0.05, 0.0, -2.5)
    drive(cm, 10, lambda i: scan([THROUGH], [5.0], [2.4], [5.0], [1.0]))
    b = cm.blocks()
    assert len(b) == 1 and abs(b[0][0] - 5.1) < 1e-6 and abs(b[0][1] - 1.0) < 1e-6 and b[0][2] == 10.0


def test_regions_say_what_to_do():
    cm = mc.ChangeMap(400, 100, 0.05, 0.0, -2.5)  # 20 m x 5 m

    # x 2..4: mapped trolleys always gone; x 8..10: there half the time; x 14..16: new objects
    def frame(i):
        xs = np.arange(2.0, 4.0, 0.2)
        gone_always = scan([THROUGH] * len(xs), xs, np.full(len(xs), 2.4), xs, np.full(len(xs), 1.0))
        half = np.arange(8.0, 10.0, 0.2)
        cls = [THROUGH if i % 2 else ON_MAP] * len(half)
        varies = scan(
            cls,
            half,
            np.full(len(half), 1.0) if i % 2 == 0 else np.full(len(half), 2.4),
            half,
            np.full(len(half), 1.0),
        )
        newx = np.arange(14.0, 16.0, 0.2)
        new = scan([UNMAPPED] * len(newx), newx, np.full(len(newx), -1.0))
        return [np.concatenate(p) for p in zip(gone_always, varies, new, strict=True)]

    drive(cm, 20, frame)
    regs = {r["kind"]: r for r in cm.regions()}
    assert set(regs) == {"gone", "varies", "new"}
    assert "Re-map area" in regs["gone"]["advice"] and "Dynamic area" in regs["varies"]["advice"]
    g = regs["gone"]["box"]
    assert g[0] <= 2.0 and g[2] >= 3.8 and g[1] <= 1.0 <= g[3]


def test_fades_and_saves(tmp_path):
    cm = mc.ChangeMap(200, 100, 0.05, 0.0, -2.5)
    drive(cm, 8, lambda i: scan([UNMAPPED], [5.0], [1.0]))
    cm.t_fade = 0.0
    cm.fade(now=mc.HALF_LIFE_S)
    assert abs(cm.new.sum() - 4.0) < 1e-4
    path = mc.path_for(str(tmp_path), "hall", 3)
    cm.save(path)
    back = mc.load(path)
    assert back is not None and back.same_geometry(200, 100, 0.05, 0.0, -2.5) and back.scans == 8
    assert back.new.sum() <= 4.0 + 1e-4  # faded further to now, never grown
    assert mc.load(str(tmp_path / "nope.npz")) is None


def test_a_trolley_gone_from_in_front_of_a_machine_counts_with_passed():
    """2026-11-10: the beam goes past the gone trolley and ends ON the machine behind it - an
    ON_MAP endpoint, not THROUGH. scan_consistency's `passed` still says the trolley is gone."""
    cm = mc.ChangeMap(200, 100, 0.05, 0.0, -2.5)
    f = scan([ON_MAP], [5.0], [2.0], [5.0], [1.0])  # ends on the machine at y 2.0; trolley expected at y 1.0
    drive(cm, 5, lambda i: (*f,))
    assert cm.gone.sum() == 0  # without `passed`: missed
    cm = mc.ChangeMap(200, 100, 0.05, 0.0, -2.5)
    for i in range(5):
        cm.add((0.25 * i, 0.0, 0.0), *f, now=100.0, passed=[True])
    assert cm.gone.sum() == 5.0 and cm.hit.sum() == 5.0
