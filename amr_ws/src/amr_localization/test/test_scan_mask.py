"""Scan mask: phantom beams learnt from open-space scans, masked only below their range."""

import math

import pytest
from amr_localization.scan_mask import Sector, apply_mask, from_yaml_dict, learn, to_yaml_dict

INF = float("inf")
A0, INC = math.radians(-90.0), math.radians(1.0)  # 181 beams, -90..+90 deg


def scan(phantoms: dict[int, float]) -> list[float]:
    r = [INF] * 181
    for i, v in phantoms.items():
        r[i] = v
    return r


def test_learn_finds_the_scratch_and_ignores_a_one_off():
    scans = [scan({100: 0.30, 101: 0.32, 102: 0.31}) for _ in range(30)]
    scans[5][40] = 0.5  # one stray return in one scan: not a phantom
    sectors = learn(scans, A0, INC, 0.1, clear_m=1.0)
    assert len(sectors) == 1
    s = sectors[0]
    assert s.from_deg == pytest.approx(8.0) and s.to_deg == pytest.approx(14.0)  # beams 98..104
    assert s.max_range_m == pytest.approx(0.42)  # farthest phantom 0.32 + 0.10


def test_mask_drops_phantoms_but_keeps_a_real_wall_behind_them():
    sectors = [Sector(8.0, 14.0, 0.42)]
    ranges = scan({100: 0.31, 101: 3.0, 20: 0.31})  # phantom, a wall seen through the gap, outside
    out, n = apply_mask(ranges, A0, INC, sectors)
    assert n == 1 and math.isnan(out[100])
    assert out[101] == 3.0 and out[20] == 0.31


def test_mask_handles_a_reversed_scan():
    # an upside-down scanner publishes angle_min > angle_max with a negative increment
    ranges = scan({80: 0.3})  # beam 80 of a reversed scan: +90 - 80 = +10 deg
    out, n = apply_mask(ranges, math.radians(90.0), -INC, [Sector(8.0, 12.0, 0.5)])
    assert n == 1 and math.isnan(out[80])


def test_yaml_round_trip_and_bad_rows():
    d = to_yaml_dict([Sector(8.0, 14.0, 0.42)], "t")
    assert from_yaml_dict(d) == [Sector(8.0, 14.0, 0.42)]
    assert from_yaml_dict(None) == [] and from_yaml_dict({"sectors": None}) == []
    with pytest.raises(ValueError):
        from_yaml_dict({"sectors": [[1, 2, -1]]})
