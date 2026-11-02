"""tools/p0_report.py's arithmetic on synthetic laps (no bag, no ROS).

    python3 -m pytest -q tools/test_p0_report.py

A lap: 30 m east along y=0, a pivot, 30 m back. Code-3 strips at x = 5, 15, 25. AMCL
reports with 0.1 s latency and noise; the wheels read 1 % long.
"""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p0_report as pr  # noqa: E402

STRIPS = (5.0, 15.0, 25.0)


def lap(name, seed, along_sigma=0.01, scale=1.01, latency=0.1, jump_at=None):
    rnd = random.Random(seed)
    v, dt = 0.5, 0.02
    wheels, poses, events = [], [], []
    t, travel = 0.0, 0.0
    # out: x from 0 to 30, then a 5 s pivot, then back from 30 to 0
    legs = [(0.0, 30.0, 1.0), (30.0, 0.0, -1.0)]
    pivot = None
    for k, (x0, x1, sgn) in enumerate(legs):
        x = x0
        next_pose = t
        while (x1 - x) * sgn > 0:
            x += sgn * v * dt
            t += dt
            travel += v * dt * scale
            wheels.append((t, travel))
            for s in STRIPS:
                if (x - sgn * v * dt - s) * (x - s) <= 0:
                    events.append((t, "code3"))
            if t >= next_pose:
                ex = x + rnd.gauss(0, along_sigma)
                if jump_at is not None and k == 0 and abs(x - jump_at) < 0.03:
                    ex += 0.5
                poses.append((t + latency, t, ex, rnd.gauss(0, 0.005)))
                next_pose += 0.1
        if k == 0:
            pivot = (t, t + 5.0)
            t += 5.0
            wheels.append((t, travel))
    return {"name": name, "wheels": wheels, "poses": poses, "events": events, "pivots": [pivot]}


def test_repeatable_laps_pass():
    res = pr.analyse([lap(f"r{i}", i) for i in range(5)])
    for d in ("out", "back"):
        m = res[d]
        assert len(m["M1"]) == 3 and all(n == 5 for n in m["M1_n"].values())
        assert all(v < 0.05 for v in m["M1"].values()), m["M1"]
        assert m["M2"] < 0.05
        assert statistics_mean(m["M6"]) == pytest.approx(0.01, abs=0.003)
    assert res["all"]["M3"] == [] and res["all"]["M4"] == pytest.approx(0.1, abs=1e-6)
    text = "\n".join(pr.report(res))
    assert "FAIL" not in text.replace("M5", "")


def test_noisy_amcl_fails_m1_and_a_jump_counts():
    res = pr.analyse([lap(f"r{i}", i, along_sigma=0.06) for i in range(5)])
    assert any(v > 0.05 for v in res["out"]["M1"].values())
    res = pr.analyse([lap("j", 1, jump_at=12.0)])
    assert len(res["all"]["M3"]) >= 1


def test_travel_at_interpolates():
    w = [(0.0, 0.0), (1.0, 1.0), (2.0, 4.0)]
    assert pr.travel_at(w, 1.5) == pytest.approx(2.5) and pr.travel_at(w, 9) == 4.0
    assert math.isnan(pr._two_sigma([1.0]))


def statistics_mean(xs):
    return sum(xs) / len(xs)
