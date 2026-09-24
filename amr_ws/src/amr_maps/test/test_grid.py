import contextlib
import signal

import numpy as np
import pytest
import yaml
from amr_maps.generate_sim_factory import ORIGIN_X, ORIGIN_Y, build

from amr_maps import grid


def test_pgm_round_trip(tmp_path):
    g = build()
    stem = str(tmp_path / "w")
    grid.write(g, stem)
    back = grid.read(stem + ".yaml")
    assert back.data.shape == g.data.shape
    assert np.array_equal(back.data, g.data)
    assert back.meta.resolution == g.meta.resolution
    assert (back.meta.origin_x, back.meta.origin_y) == (ORIGIN_X, ORIGIN_Y)


def test_row_zero_is_bottom(tmp_path):
    g = grid.Grid(np.zeros((4, 3), dtype=np.int8), grid.GridMeta(0.1, 0.0, 0.0))
    g.data[0, 0] = 100  # bottom-left cell occupied
    raw = grid.to_pgm_bytes(g)
    pixels = raw[len(b"P5\n3 4\n255\n") :]
    assert pixels[-3] == 0  # last row of the image = bottom row of the grid
    assert pixels[0] == 254


def test_unknown_survives(tmp_path):
    g = grid.Grid(np.full((2, 2), -1, dtype=np.int8), grid.GridMeta(0.1, 0.0, 0.0))
    g.data[1, 1] = 0
    back = grid.from_pgm_bytes(grid.to_pgm_bytes(g), g.meta)
    assert back.data[0, 0] == -1 and back.data[1, 1] == 0


def test_world_to_cell_and_back():
    g = build()
    r, c = g.world_to_cell(0.0, 0.0)
    x, y = g.cell_to_world(r, c)
    assert abs(x) <= g.meta.resolution and abs(y) <= g.meta.resolution
    assert g.data[r, c] == 0, "the start mark must be free space"


def test_world_has_walls_and_racks():
    g = build()
    assert g.data[0, :].min() == 100 and g.data[-1, :].max() == 100
    r, c = g.world_to_cell(10.0, 2.4)
    assert g.data[r, c] == 100  # rack row C
    r, c = g.world_to_cell(10.0, 0.0)
    assert g.data[r, c] == 0  # aisle between B and C, the survey aisle
    with pytest.raises(ValueError):
        grid.from_pgm_bytes(b"P2\n1 1\n255\n0", g.meta)


@pytest.mark.parametrize("yaw", [0.0, 0.3, -1.2])
@pytest.mark.parametrize("origin", [(0.0, 0.0), (-3.0, -10.0), (12.5, 4.25)])
@pytest.mark.parametrize("res", [0.05, 0.1, 0.025])
def test_pixel_world_round_trip(yaw, origin, res):
    meta = grid.GridMeta(res, origin[0], origin[1], origin_yaw=yaw)
    h = 400
    for x, y in ((0.0, 0.0), (5.3, -2.1), (-7.7, 9.9)):
        u, v = grid.world_to_pixel(meta, h, x, y)
        bx, by = grid.pixel_to_world(meta, h, u, v)
        assert (bx, by) == pytest.approx((x, y), abs=1e-9)


def test_pixel_conventions():
    meta = grid.GridMeta(0.05, -3.0, -10.0)
    h = 400
    # the grid origin corner is the bottom-left pixel corner of the image
    assert grid.world_to_pixel(meta, h, -3.0, -10.0) == pytest.approx((0.0, 400.0))
    # one cell east, one cell north -> one pixel right, one pixel up (smaller v)
    assert grid.world_to_pixel(meta, h, -2.95, -9.95) == pytest.approx((1.0, 399.0))
    # the centre of cell (row 0, col 0) is at pixel (0.5, 399.5)
    g = build()
    cx, cy = g.cell_to_world(0, 0)
    assert grid.world_to_pixel(g.meta, g.height, cx, cy) == pytest.approx((0.5, 399.5))


# ---- R24: bounded PGM header parsing; R19: yaw-free origins only ------------------


@contextlib.contextmanager
def hard_timeout(seconds: float):
    """SIGALRM fails the test instead of hanging the run (pytest-timeout is not loaded)."""

    def fire(signum, frame):
        raise TimeoutError(f"parser did not finish within {seconds} s")

    old = signal.signal(signal.SIGALRM, fire)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)


META = grid.GridMeta(0.05, 0.0, 0.0)
GOOD = b"P5\n# made by test\n3 2\n255\n" + bytes([254, 0, 205, 254, 254, 0])


def test_r24_eof_at_every_header_position_errors_promptly():
    header_len = GOOD.index(b"255\n") + 4
    with hard_timeout(5.0):
        for cut in range(header_len):
            with pytest.raises(grid.GridError):
                grid.from_pgm_bytes(GOOD[:cut], META)
        for bad in (
            b"",
            b"P5",
            b"P5 ",
            b"P5\n# no newline",
            b"P5\n3 2\n255",  # EOF right after maxval, no separator
            b"P5\n3 2\n255\n" + b"\x00" * 5,  # short payload
            b"P5\n0 2\n255\n",
            b"P5\n-3 2\n255\n" + b"\x00" * 6,
            b"P5\nx 2\n255\n" + b"\x00" * 6,
            b"P5\n3 2\n0\n" + b"\x00" * 6,
            b"P5\n3 2\n65535\n" + b"\x00" * 12,
            b"P5\n100000 100000\n255\n",  # oversized: refused before allocating
            b"P6\n3 2\n255\n" + b"\x00" * 18,
            b"P55\n3 2\n255\n" + b"\x00" * 6,
            b"P5" + b"9" * 1_000_000,  # no whitespace anywhere
        ):
            with pytest.raises(grid.GridError):
                grid.from_pgm_bytes(bad, META)


def test_r24_supported_header_forms_still_parse():
    with hard_timeout(5.0):
        for raw in (
            GOOD,
            b"P5 3 2 255 " + GOOD[-6:],
            b"P5\r\n3\t2\r\n#c\n255\n" + GOOD[-6:],
            b"P5\n3#c\n 2\n255\n" + GOOD[-6:],
        ):
            g = grid.from_pgm_bytes(raw, META)
            assert g.data.shape == (2, 3)
            assert g.data[1].tolist() == [0, 100, -1] and g.data[0].tolist() == [0, 0, 100]


def test_r24_bad_map_yaml_is_a_grid_error(tmp_path):
    g = grid.Grid(np.zeros((2, 3), dtype=np.int8), META)
    stem = str(tmp_path / "m")
    grid.write(g, stem)
    base = yaml.safe_load(open(stem + ".yaml"))
    (tmp_path / "trunc.pgm").write_bytes(b"P5\n3 ")
    for name, patch in (
        ("trunc", {"image": "trunc.pgm"}),
        ("res0", {"resolution": 0.0}),
        ("resnan", {"resolution": float("nan")}),
        ("noorigin", {"origin": None}),
        ("shortorigin", {"origin": [1.0]}),
        ("thresh", {"free_thresh": 0.9}),
    ):
        doc = {k: v for k, v in {**base, **patch}.items() if v is not None}
        (tmp_path / f"{name}.yaml").write_text(yaml.safe_dump(doc))
        with hard_timeout(5.0), pytest.raises(grid.GridError):
            grid.read(str(tmp_path / f"{name}.yaml"))
    (tmp_path / "garbage.yaml").write_text("origin: [1, 2\n: :")
    with pytest.raises(grid.GridError):
        grid.read(str(tmp_path / "garbage.yaml"))


def test_r19_rotated_origin_is_refused_at_load(tmp_path):
    stem = str(tmp_path / "rot")
    grid.write(
        grid.Grid(np.zeros((2, 3), dtype=np.int8), grid.GridMeta(0.05, 1.0, 2.0, origin_yaw=0.3)), stem
    )
    with pytest.raises(grid.GridError, match="yaw"):
        grid.read(stem + ".yaml")
    # what slam_toolbox / map_saver and our own writer produce: yaw 0 (float or int) still loads
    stem0 = str(tmp_path / "flat")
    grid.write(grid.Grid(np.zeros((2, 3), dtype=np.int8), grid.GridMeta(0.05, -1.5, 2.0)), stem0)
    assert grid.read(stem0 + ".yaml").meta.origin_yaw == 0.0
    text = open(stem0 + ".yaml").read()
    (tmp_path / "ints.yaml").write_text(
        text.replace("- 0.0\n", "- 0\n").replace("image: flat.pgm", "image: " + stem0 + ".pgm")
    )
    assert grid.read(str(tmp_path / "ints.yaml")).width == 3
