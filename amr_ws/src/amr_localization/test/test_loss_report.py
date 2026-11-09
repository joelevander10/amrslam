"""Why localisation was lost (2026-11-09): the report built from the last seconds."""

import math

import numpy as np

from amr_localization import loss_report as lr
from amr_localization import readiness as rd

AGES = {"scan": 0.05, "wheels": 0.02, "imu": 0.05, "tf": 0.05}


def _driving(rec, v=0.7, t_end=10.0, yaw=0.0, unmapped=0.05, match=0.85):
    t = 0.0
    while t <= t_end + 1e-9:
        x = v * t
        rec.add(
            lr.Sample(
                t,
                (x * math.cos(yaw), x * math.sin(yaw), yaw),
                dict(AGES),
                (0.01, 0.01, 0.001),
                match,
                0.02,
                unmapped,
            )
        )
        t += 0.1


def test_a_sideways_jump_while_driving_straight_points_at_the_lidar_or_the_map(tmp_path):
    rec = lr.Recorder()
    _driving(rec, yaw=math.pi / 2)  # driving north at 0.7 m/s
    # the correction: 0.48 m to the robot's left (west, -x) and 0.1 m back
    jump = rd.Jump(0.49, math.radians(1.4), 10.0, -0.48, -0.10, math.pi / 2, math.radians(1.4))
    rec.scan = {
        "t": 9.9,
        "laser": (0.0, 7.0, math.pi / 2),
        "x": np.array([0.0, 1.0, 2.0]),
        "y": np.array([8.0, 8.0, 8.0]),
        "cls": np.array([1, 2, 3], dtype=np.int8),
    }
    rep = lr.build(rec, 10.0, "LOC_JUMP", "localisation corrected by 0.49 m / 1.4 deg", jump)
    assert rep["jump"] == {"dist_m": 0.49, "ahead_m": -0.1, "left_m": 0.48, "turn_deg": 1.4}
    assert rep["speed_mps"] == 0.7 and abs(rep["turn_rad_s"]) < 1e-6 and rep["travelled_m"] > 6.9
    assert any("sideways" in h and "lidar" in h for h in rep["hints"])
    assert any("0.70 m/s" in h for h in rep["hints"])
    assert rep["scan_points"]["map"] == [[0.0, 8.0]] and rep["scan_points"]["unmapped"] == [[1.0, 8.0]]
    name = lr.write(str(tmp_path), rep)
    assert lr.load(str(tmp_path), name)["jump"]["left_m"] == 0.48
    listed = lr.list_reports(str(tmp_path))
    assert listed[0]["name"] == name and "scan_points" not in listed[0]
    text = lr.summary(rep, name)
    assert "left +0.48 m" in text and f"(report {name})" in text and "Likely: " in text


def test_unmapped_objects_and_a_quiet_sensor_are_named():
    rec = lr.Recorder()
    _driving(rec, unmapped=0.4, match=0.5)
    ages = dict(AGES, scan=0.9)
    rec.add(lr.Sample(10.1, (7.07, 0.0, 0.0), ages, (0.01, 0.01, 0.001), 0.5, 0.02, 0.4))
    rep = lr.build(rec, 10.1, "LOC_STALE", "scan stale")
    assert rep["hints"][0].startswith("scan went quiet (0.90 s old): check the lidar")
    assert any("not on the map" in h and "dynamic" in h for h in rep["hints"])
    assert not any("does not fit here" in h for h in rep["hints"])  # explained by the objects


def test_reports_are_kept_to_the_newest_ones(tmp_path):
    rep = lr.build(lr.Recorder(), 0.0, "LOC_LOST", "covariance")
    names = [lr.write(str(tmp_path), rep, keep=3) for _ in range(5)]
    assert [r["name"] for r in lr.list_reports(str(tmp_path))] == names[-3:][::-1]
    assert lr.load(str(tmp_path), "../x") is None
