"""Speckle filter: lone returns go, surfaces stay, out-of-range readings untouched."""

import math

from amr_localization.speckle import despeckle


def test_lone_return_in_open_space_is_removed():
    inf = float("inf")
    ranges = [inf, inf, 3.2, inf, inf]  # one hit, nothing around it
    out, n = despeckle(ranges, 0.1, 40.0)
    assert n == 1 and math.isnan(out[2]) and out[0] == inf


def test_wall_and_two_beam_post_are_kept():
    wall = [5.00, 5.01, 5.02, 5.03, 5.04]
    post = [20.0, 20.0, 2.00, 2.02, 20.0, 20.0]  # a thin post hit by two beams
    for ranges in (wall, post):
        out, n = despeckle(ranges, 0.1, 40.0)
        assert n == 0 and out == ranges


def test_speckle_in_front_of_a_wall_is_removed_but_not_the_wall():
    ranges = [8.0, 8.0, 8.0, 3.0, 8.0, 8.0, 8.0]  # a stray return far in front of a wall
    out, n = despeckle(ranges, 0.1, 40.0)
    assert n == 1 and math.isnan(out[3])
    assert [x for i, x in enumerate(out) if i != 3] == [8.0] * 6


def test_tolerance_grows_with_range():
    near = [1.0, 1.10]  # 10 cm apart at 1 m: not the same surface (tol 0.08)
    far = [15.0, 15.4]  # 40 cm apart at 15 m: adjacent beams on a slanted wall (tol 0.50)
    assert despeckle(near, 0.1, 40.0, window=1)[1] == 2
    assert despeckle(far, 0.1, 40.0, window=1)[1] == 0


def test_invalid_readings_pass_and_never_vouch():
    nan = float("nan")
    ranges = [0.05, 2.0, nan, 50.0]  # below min, a lone hit, NaN, above max
    out, n = despeckle(ranges, 0.1, 40.0)
    assert n == 1 and math.isnan(out[1])
    assert out[0] == 0.05 and math.isnan(out[2]) and out[3] == 50.0
