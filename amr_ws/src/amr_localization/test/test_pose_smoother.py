"""Smoothed map->odom (pose_smoother.py, owner 2026-11-12): the wobble in dynamic areas."""

import math
import random

from amr_localization import pose_smoother as ps


def robot(sm: ps.Smoother, ob):
    return ps.compose(sm.out, ob)


def test_compose_and_inverse_round_trip():
    a, b = (3.0, -2.0, 0.7), (1.5, 0.4, -2.0)
    c = ps.compose(a, ps.compose(ps.inverse(a), b))
    assert all(abs(u - v) < 1e-12 for u, v in zip(c, b, strict=True))


def test_first_pose_and_standing_still_take_amcl_at_once():
    sm = ps.Smoother()
    assert sm.step((0.0, 0.0, 0.0), 0.02, 0.0, 0.0, False) is None  # no AMCL pose yet
    sm.set_raw((1.0, 2.0, 0.1))
    assert sm.step((5.0, 0.0, 0.0), 0.02, 0.016, 0.0, False) == (1.0, 2.0, 0.1)
    sm.set_raw((1.2, 2.0, 0.1))
    assert sm.step((5.0, 0.0, 0.0), 0.02, 0.0, 0.0, True) == (1.2, 2.0, 0.1)  # still: snap


def test_a_correction_is_worked_in_over_a_couple_of_metres_not_one_step():
    sm, ob = ps.Smoother(), (0.0, 0.0, 0.0)
    sm.set_raw((0.0, 0.0, 0.0))
    sm.step(ob, 0.02, 0.0, 0.0, False)
    sm.set_raw((-0.18, 0.0, 0.0))  # a typical aisle correction: 0.18 m back
    x, t, first = 0.0, 0.0, None
    while True:  # 0.8 m/s along x, 50 Hz
        x += 0.016
        t += 0.02
        sm.step((x, 0.0, 0.0), 0.02, 0.016, 0.0, False)
        d, _ = sm.gap((x, 0.0, 0.0))
        if first is None:
            first = 0.18 - d
        if d < 0.01:
            break
    assert first < 0.003  # the first tick moves at most a few mm (was 0.18 m at once)
    assert 1.0 < t < 6.0  # done within a few seconds of driving


def test_a_heading_correction_turns_about_the_robot_not_the_odom_origin():
    # 100 m from the odom origin a 1 deg change of map->odom would throw the pose 1.7 m
    sm, ob = ps.Smoother(), (100.0, 0.0, 0.0)
    sm.set_raw((0.0, 0.0, 0.0))
    sm.step(ob, 0.02, 0.0, 0.0, False)
    target = (100.0, 0.0, math.radians(1.0))  # AMCL: same place, 1 deg turned
    sm.set_raw(ps.compose(target, ps.inverse(ob)))
    for _ in range(20):
        sm.step(ob, 0.02, 0.016, 0.0, False)
        r = robot(sm, ob)
        assert math.hypot(r[0] - 100.0, r[1]) < 1e-6  # the position never wanders
    assert 0.0 < robot(sm, ob)[2] < math.radians(1.0)


def test_jitter_is_damped_before_it_reaches_the_steering():
    rnd = random.Random(1)
    sm = ps.Smoother()
    sm.set_raw((0.0, 0.0, 0.0))
    x, worst_raw, worst_out = 0.0, 0.0, 0.0
    sm.step((x, 0.0, 0.0), 0.02, 0.0, 0.0, False)
    for i in range(1000):  # 20 s at 0.8 m/s; AMCL updates every 3rd tick (5 cm) with +-3 cm / +-0.5 deg
        x += 0.016
        if i % 3 == 0:  # AMCL's robot pose jitters about the true one (x, 0, 0)
            noisy = (x, rnd.uniform(-0.03, 0.03), math.radians(rnd.uniform(-0.5, 0.5)))
            sm.set_raw(ps.compose(noisy, ps.inverse((x, 0.0, 0.0))))
        sm.step((x, 0.0, 0.0), 0.02, 0.016, 0.0, False)
        if i > 100:
            worst_raw = max(worst_raw, abs(ps.compose(sm.raw, (x, 0.0, 0.0))[1]))
            worst_out = max(worst_out, abs(robot(sm, (x, 0.0, 0.0))[1]))
    assert worst_out < 0.4 * worst_raw


def test_a_loss_sized_gap_snaps():
    sm, ob = ps.Smoother(), (0.0, 0.0, 0.0)
    sm.set_raw((0.0, 0.0, 0.0))
    sm.step(ob, 0.02, 0.0, 0.0, False)
    sm.set_raw((0.6, 0.0, 0.0))
    assert sm.step(ob, 0.02, 0.016, 0.0, False) == (0.6, 0.0, 0.0)
    sm.set_raw((0.6, 0.0, 0.0), snap=True)  # after /initialpose: at once even when small
    sm.out = (0.5, 0.0, 0.0)
    assert sm.step(ob, 0.02, 0.016, 0.0, False) == (0.6, 0.0, 0.0)


def test_still_detector_needs_odometry_quiet_for_still_s():
    st = ps.StillDetector(still_s=0.15)
    o = (1.0, 2.0, 0.3)
    flags = [st.update(i * 0.02, o) for i in range(30)]
    assert not any(flags[:7]) and all(flags[8:])  # quiet since t 0: still from 0.15 s on
    assert not st.update(0.64, (1.2, 2.0, 0.3))  # moved 0.2 m in 0.1 s
    assert not st.update(0.70, (1.2, 2.0, 0.3))
