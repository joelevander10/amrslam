"""SE(2) maths for the GLS floor labels and the one-observation rule (agv_core/glspose.py,
gls-wait-coding-plan §C.2). Pure: no ROS, no config, no network."""

import math
from types import SimpleNamespace

from helpers import check

from agv_core import glspose as gp


def close(a, b, tol=1e-9):
    return all(
        abs(gp.wrap(x - y)) <= tol if i == 2 else abs(x - y) <= tol
        for i, (x, y) in enumerate(zip(a, b, strict=True))
    )


def test_compose_inverse():
    print("\nglspose: compose / inverse")
    check(
        "wrap keeps (-pi, pi]",
        abs(gp.wrap(3 * math.pi) - math.pi) < 1e-12
        and abs(gp.wrap(-0.5) + 0.5) < 1e-12
        and abs(gp.wrap(2 * math.pi)) < 1e-12,
    )
    ok = True
    for yaw in (0.0, 0.7, math.pi / 2, -2.5, math.pi, -math.pi + 1e-6):
        a = (1.2, -0.4, yaw)
        b = (0.3, 0.8, 1.1 - yaw)
        ok &= close(gp.compose(a, gp.inverse(a)), (0.0, 0.0, 0.0))
        ok &= close(gp.compose(gp.inverse(a), a), (0.0, 0.0, 0.0))
        ok &= close(gp.compose(gp.compose(a, b), gp.inverse(b)), a)
        ok &= close(gp.inverse(gp.inverse(a)), a)
    check("a (+) a^-1 = identity, (a (+) b) (+) b^-1 = a, at six yaws incl. +-pi", ok)
    check(
        "compose rotates the second pose by the first's yaw",
        close(gp.compose((1.0, 2.0, math.pi / 2), (1.0, 0.0, 0.0)), (1.0, 3.0, math.pi / 2)),
    )


def test_label_in_map():
    print("\nglspose: label_in_map / base_in_map")
    ok = True
    for t_mb, t_bc, t_lc in (
        ((3.0, -1.0, 0.4), (0.35, 0.05, 0.02), (0.004, -0.007, 0.3)),
        ((-7.5, 2.2, math.pi), (-0.2, 0.0, -math.pi / 2), (0.0, 0.01, -3.0)),
        ((0.0, 0.0, -2.9), (0.0, 0.0, 0.0), (0.012, 0.012, 1.0)),
    ):
        t_ml = gp.label_in_map(t_mb, t_bc, t_lc)
        ok &= close(gp.base_in_map(t_ml, t_bc, t_lc), t_mb, 1e-12)
    check("base_in_map(label_in_map(T_MB)) recovers T_MB exactly", ok)
    # By hand: base at (1, 2) facing +y; camera 0.3 m ahead, 0.1 m left, turned +90 deg -> the
    # camera at (0.9, 2.3) facing -x. The camera reads (0.01, -0.02) at +90 deg in the label
    # frame, so the label faces +y and its origin is the camera minus R(90 deg)(0.01, -0.02)
    # = (0.9 - 0.02, 2.3 - 0.01).
    t_ml = gp.label_in_map((1.0, 2.0, math.pi / 2), (0.3, 0.1, math.pi / 2), (0.01, -0.02, math.pi / 2))
    check(
        "hand-worked T_ML with a turned mount: (0.88, 2.29, +90 deg)",
        close(t_ml, (0.88, 2.29, math.pi / 2), 1e-12),
        str(t_ml),
    )


def test_average():
    print("\nglspose: circular mean and average")
    m = gp.circular_mean([math.pi - 0.1, -math.pi + 0.1])
    check("the circular mean across +-pi is pi, not 0", abs(abs(m) - math.pi) < 1e-12, str(m))
    m = gp.circular_mean([math.pi - 0.1, -math.pi + 0.3])
    check(
        "an uneven pair across +-pi averages to -pi + 0.1", abs(gp.wrap(m - (math.pi + 0.1))) < 1e-12, str(m)
    )
    a = gp.average([(0.0, 0.0, 0.0), (0.002, 0.0, 0.0), (0.001, 0.003, 0.02)])
    check(
        "average: mean xy and the worst distance from it",
        close(a.pose[:2], (0.001, 0.001), 1e-12) and abs(a.spread_xy_m - 0.002) < 1e-12 and a.n == 3,
        str(a),
    )
    check(
        "average: yaw spread is the worst |yaw - mean|",
        abs(a.spread_yaw_rad - (0.02 - a.pose[2])) < 1e-12 and abs(a.pose[2] - 0.02 / 3) < 1e-5,
        str(a),
    )
    a = gp.average([(0.0, 0.0, math.pi - 0.01), (0.0, 0.0, -math.pi + 0.01)])
    check("a yaw spread across +-pi is 0.01, not 2 pi", abs(a.spread_yaw_rad - 0.01) < 1e-9, str(a))
    try:
        gp.average([])
        raised = False
    except ValueError:
        raised = True
    check("averaging nothing is a ValueError", raised)


def reading(x=0.001, y=-0.002, yaw=0.05, label="x0100y0000", n_codes=5, valid=True, calibrated=True):
    return SimpleNamespace(
        valid=valid, calibrated=calibrated, label_id=label, x_m=x, y_m=y, yaw_rad=yaw, n_codes=n_codes
    )


def test_observe():
    print("\nglspose: one label observation")
    lim = dict(n=4, min_codes=3, spread_xy_m=0.002, spread_yaw_rad=0.01)

    def obs(rs):
        return gp.observe(rs, **lim)

    good = [reading(x=0.001 + 0.0005 * k) for k in range(4)]
    o, why = obs(good)
    check(
        "four agreeing frames are one observation",
        why is None
        and o.label_id == "x0100y0000"
        and o.t_lc.n == 4
        and o.n_codes_min == 5
        and abs(o.t_lc.pose[0] - 0.00175) < 1e-12,
        why or str(o),
    )
    o, why = obs(good[:3])
    check("too few frames are refused", o is None and "3 of 4" in why, why)
    o, why = obs(good[:3] + [reading(valid=False)])
    check("one frame without the label is refused", o is None and "not seen" in why, why)
    o, why = obs(good[:3] + [reading(calibrated=False)])
    check("an uncalibrated camera is refused", o is None and "not calibrated" in why, why)
    o, why = obs(good[:3] + [reading(label="x0200y0000")])
    check(
        "two labels in the frames are refused, both named",
        o is None and "x0100y0000" in why and "x0200y0000" in why,
        why,
    )
    o, why = obs(good[:3] + [reading(n_codes=2)])
    check("a frame under min_codes is refused", o is None and "only 2 codes" in why, why)
    o, why = obs(good[:3] + [reading(x=0.001 + 0.005)])
    check("an xy spread over the limit is refused", o is None and "disagree" in why, why)
    o, why = obs(good[:3] + [reading(yaw=0.05 + 0.04)])
    check("a yaw spread over the limit is refused", o is None and "disagree" in why, why)
    # only the LAST n count: an earlier frame of another label, a gap, a 2-code frame and an
    # outlier are all outside the window
    early = [reading(label="x0900y0900"), reading(valid=False), reading(n_codes=1), reading(x=0.5, yaw=2.0)]
    o, why = obs(early + good)
    check(
        "only the last n frames count",
        why is None
        and o.t_lc.n == 4
        and o.n_codes_min == 5
        and close(o.t_lc.pose, gp.average([(r.x_m, r.y_m, r.yaw_rad) for r in good]).pose, 1e-12),
        why or str(o),
    )
    o, why = obs(good + early[:1])
    check("... and a bad newest frame is inside the window", o is None and "more than one" in why, why)


TESTS = [
    test_compose_inverse,
    test_label_in_map,
    test_average,
    test_observe,
]
