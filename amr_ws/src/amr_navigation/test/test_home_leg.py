"""The home leg (route step `home`, 2026-11-03): scurve.home_leg geometry, its bounds, the
compiler's placement rule, the route file, validation's footprint sweep and the path
projection the executor measures progress with."""

import math

import numpy as np
import pytest
from amr_maps.grid import Grid, GridMeta
from amr_navigation.compiler import HOME, compile_route
from amr_navigation.route import (
    HOME_MAX_HEADING_DEG,
    HOME_MAX_RUNUP_M,
    HOME_MIN_RADIUS_M,
    Limits,
    MapRef,
    Route,
    RouteError,
    StartPose,
    Step,
)
from amr_navigation.scurve import along_cross, home_leg
from amr_navigation.validate import validate

from amr_navigation import footprint as fpmod

FP = fpmod.Footprint(((-0.5, -0.35), (1.1, -0.35), (1.1, 0.35), (-0.5, 0.35)), 0.20)
HOME_POSE = (2.0, 1.0, math.radians(30.0))  # anywhere, any heading: the leg works in HOME's frame


class Manifest:
    map_id, revision, sha256 = "m", 1, "abc"


def behind(run_up, offset=0.0, err_deg=0.0, home=HOME_POSE):
    """A pose run_up behind HOME along its heading, offset to its left, turned err_deg."""
    x, y, yaw = home
    c, s = math.cos(yaw), math.sin(yaw)
    return (x - c * run_up - s * offset, y - s * run_up + c * offset, yaw + math.radians(err_deg))


def radii(samples):
    """Radius through every three consecutive samples (inf where they are collinear)."""
    out = []
    for a, b, c in zip(samples, samples[1:], samples[2:], strict=False):
        ab, bc, ca = (math.dist(a[:2], b[:2]), math.dist(b[:2], c[:2]), math.dist(c[:2], a[:2]))
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        out.append(math.inf if area2 < 1e-12 else ab * bc * ca / (2.0 * area2))
    return out


def test_on_homes_line_the_leg_is_a_straight():
    leg = home_leg(behind(1.2), HOME_POSE)
    assert leg.straight and leg.min_radius_m == math.inf
    assert leg.length_m == pytest.approx(1.2) and leg.run_up_m == pytest.approx(1.2)
    assert leg.samples[-1] == pytest.approx(HOME_POSE)
    assert all(abs(p[2] - HOME_POSE[2]) < 1e-12 for p in leg.samples)
    # within a millimetre of the line still counts as on it
    assert home_leg(behind(1.2, offset=0.0008), HOME_POSE).straight


@pytest.mark.parametrize("offset", [0.05, -0.05, 0.15, -0.15])
@pytest.mark.parametrize("err_deg", [0.0, 5.0, -5.0])
def test_s_curve_leaves_on_the_vehicle_heading_and_arrives_on_homes(offset, err_deg):
    start = behind(2.0, offset, err_deg)
    leg = home_leg(start, HOME_POSE)
    assert not leg.straight
    assert leg.offset_m == pytest.approx(offset) and math.degrees(leg.heading_err_rad) == pytest.approx(
        err_deg
    )
    assert leg.samples[0] == pytest.approx(start) and leg.samples[-1] == pytest.approx(HOME_POSE)
    # no curvature at either end: the heading barely changes over the first and last sample
    # (a constant-curvature start at the 1 m bound would turn 0.05 rad there)
    assert abs(leg.samples[1][2] - start[2]) < 0.005
    assert abs(leg.samples[-2][2] - HOME_POSE[2]) < 0.005
    # samples no farther apart than the spacing; the drawn radius never below the bound
    gaps = [math.dist(a[:2], b[:2]) for a, b in zip(leg.samples, leg.samples[1:], strict=False)]
    assert max(gaps) <= 0.05 + 1e-9
    assert leg.length_m == pytest.approx(sum(gaps)) and leg.length_m > leg.run_up_m
    assert leg.min_radius_m >= HOME_MIN_RADIUS_M and min(radii(leg.samples)) >= HOME_MIN_RADIUS_M * 0.98


def test_tightest_radius_matches_the_closed_form_for_a_pure_offset():
    # e = 0: peak curvature 5.7735 * offset / run_up^2 (small-slope), so R ~ run_up^2 / (5.77 offset)
    leg = home_leg(behind(2.0, 0.10), HOME_POSE)
    assert leg.min_radius_m == pytest.approx(2.0**2 / (5.7735 * 0.10), rel=0.02)
    # and the description an operator reads
    assert leg.describe().startswith("S-curve") and "100 mm left" in leg.describe()


@pytest.mark.parametrize(
    "start, match",
    [
        (behind(-0.5), "not behind HOME"),  # in front of HOME
        (behind(0.02), "not behind HOME"),  # practically on it
        (behind(HOME_MAX_RUNUP_M + 0.1), "at most 5.00 m"),
        (behind(2.0, 0.0, HOME_MAX_HEADING_DEG + 1.0), "off HOME's heading"),
        (behind(1.0, 0.25), r"needs a 0\.\d\d m radius.*end the route at least 1\.\d\d m behind HOME"),
        (behind(0.6, 0.40), "would point .* off HOME's line"),
        (behind(5.0, 3.0), "does not fit within 5.00 m"),
    ],
)
def test_refusals_say_why_and_what_to_change(start, match):
    with pytest.raises(RouteError, match=match):
        home_leg(start, HOME_POSE)


def test_suggested_run_up_is_enough():
    with pytest.raises(RouteError) as e:
        home_leg(behind(1.0, 0.40), HOME_POSE)
    need = float(str(e.value).split("at least ")[1].split(" m")[0])
    leg = home_leg(behind(need, 0.40), HOME_POSE)
    assert leg.min_radius_m >= HOME_MIN_RADIUS_M


# ---- route file and compiler ----------------------------------------------------------


def make_route(steps, start=(0.0, 0.0, 0.0), repeat=1):
    return Route("r1", MapRef("m", 1, "abc"), StartPose(*start), steps, Limits(), repeat_count=repeat)


def straight(sid, x, y):
    return Step(sid, "straight", to=(x, y))


def rotate(sid, direction, angle):
    return Step(sid, "rotate", direction=direction, angle_deg=angle)


def out_and_back(lateral=0.0, steps_after=()):
    """Out 3 m along +x, U-turn, back to 1.5 m behind HOME... which is behind it only after a
    second U-turn: so out along +x, CCW 180, back past HOME to -2.0, CCW 180, then home."""
    return make_route(
        [
            straight("s1", 3.0, 0.0),
            rotate("s2", "ccw", 180),
            straight("s3", -1.5, 0.0),
            rotate("s4", "ccw", 180),
            *([straight("s5", -1.0, 0.0)] if lateral == 0.0 else []),
            Step("s9", "home"),
            *steps_after,
        ]
    )


def test_home_step_round_trips_and_closes_the_route_exactly():
    r = out_and_back()
    assert Route.loads(r.dumps()).to_dict() == r.to_dict()
    assert r.to_dict()["steps"][-1] == {"id": "s9", "type": "home"}
    c = compile_route(r)
    st = c.steps[-1]
    assert st.type == HOME and st.leg.straight and st.length_m == pytest.approx(1.0)
    assert c.end == (0.0, 0.0, 0.0) and c.closes
    assert st.v_mps == pytest.approx(Limits().linear_mps)  # a straight home leg runs at the trackless speed
    assert c.total_length_m == pytest.approx(3.0 + 4.5 + 0.5 + 1.0)


def test_a_curved_home_leg_runs_at_arc_speed_and_repeats_close():
    # the U-turns leave the vehicle 0.2 m left of HOME's line: an S-curve back onto it
    r = make_route(
        [
            straight("s1", 3.0, 0.0),
            rotate("s2", "ccw", 90),
            straight("s3", 3.0, 0.2),
            rotate("s4", "ccw", 90),
            straight("s5", -2.0, 0.2),
            rotate("s6", "ccw", 180),
            Step("s7", "home"),
        ],
        repeat=3,
    )
    c = compile_route(r)
    st = c.steps[-1]
    assert (
        not st.leg.straight
        and st.leg.offset_m == pytest.approx(0.2)
        and st.leg.run_up_m == pytest.approx(2.0)
    )
    assert st.v_mps == pytest.approx(0.30) and c.closes
    assert validate(r, Manifest, Grid(np.zeros((200, 200), dtype=np.int8), GridMeta(0.05, -5.0, -5.0)), FP).ok


@pytest.mark.parametrize(
    "steps, match",
    [
        ([straight("s1", 2.0, 0.0), Step("s2", "home"), straight("s3", 3.0, 0.0)], "must be the last step"),
        ([Step("s1", "home")], "not behind HOME"),  # the start pose is HOME itself
        ([straight("s1", 2.0, 0.0), Step("s2", "home")], "not behind HOME"),  # HOME is behind the vehicle
    ],
)
def test_home_step_placement(steps, match):
    with pytest.raises(RouteError, match=match) as e:
        compile_route(make_route(steps))
    assert e.value.step_id is not None


def test_a_wait_may_come_right_before_the_home_leg():
    # route-itg-02's ending: a release wait, then back to HOME
    r = out_and_back()
    r.steps.insert(len(r.steps) - 1, Step("w1", "wait", name="ITG release"))
    assert compile_route(r).closes


# ---- validation: the footprint swept along the leg ------------------------------------


def test_an_obstacle_beside_the_curve_blocks_the_home_leg_only():
    # 10 m x 10 m free map, HOME at the origin facing +x. A loop up, back and down the left side
    # ends 3 m behind HOME, 0.3 m left of its line; nothing but the home leg passes near (-1.3, 0.75)
    g = Grid(np.zeros((200, 200), dtype=np.int8), GridMeta(0.05, -5.0, -5.0))
    r = make_route(
        [
            rotate("s1", "ccw", 90),
            straight("s2", 0.0, 2.0),
            rotate("s3", "ccw", 90),
            straight("s4", -3.0, 2.0),
            rotate("s5", "ccw", 90),
            straight("s6", -3.0, 0.3),
            rotate("s7", "ccw", 90),
            Step("s8", "home"),
        ]
    )
    v = validate(r, Manifest, g, FP)
    assert v.ok and v.compiled.steps[-1].leg.offset_m == pytest.approx(0.3), [i.to_dict() for i in v.issues]
    # a post the S-curve's footprint sweeps (its left side, early in the curve)
    post = g.data.copy()
    col, row = int((-1.3 + 5.0) / 0.05), int((0.75 + 5.0) / 0.05)
    post[row - 1 : row + 1, col - 1 : col + 1] = 100
    v = validate(r, Manifest, Grid(post, g.meta), FP)
    assert not v.ok and [(i.code, i.step_id) for i in v.errors] == [("clearance", "s8")]
    assert "home leg sweep" in v.errors[0].message


def test_a_home_leg_leaving_the_map_is_refused():
    g = Grid(np.zeros((60, 60), dtype=np.int8), GridMeta(0.05, -1.5, -1.5))
    r = make_route(
        [rotate("s1", "ccw", 180), straight("s2", -1.0, 0.0), rotate("s3", "ccw", 180), Step("s4", "home")]
    )
    v = validate(r, Manifest, g, FP)
    assert not v.ok and any(i.step_id == "s4" and "outside the map" in i.message for i in v.errors)


# ---- the executor's progress on the leg ------------------------------------------------


def test_along_cross_on_the_samples():
    leg = home_leg(behind(2.0, 0.2), HOME_POSE)
    s = leg.samples
    assert along_cross(s, s[0]) == pytest.approx((0.0, 0.0), abs=1e-9)
    along, cross = along_cross(s, s[-1])
    assert along == pytest.approx(leg.length_m) and cross == pytest.approx(0.0, abs=1e-9)
    mid = s[len(s) // 2]
    along, cross = along_cross(s, mid)
    assert 0.3 * leg.length_m < along < 0.7 * leg.length_m and cross == pytest.approx(0.0, abs=1e-6)
    # 5 cm to the left of travel at the middle reads +0.05
    yaw = mid[2]
    along2, cross2 = along_cross(s, (mid[0] - 0.05 * math.sin(yaw), mid[1] + 0.05 * math.cos(yaw), yaw))
    assert along2 == pytest.approx(along, abs=0.01) and cross2 == pytest.approx(0.05, abs=0.002)
    # past HOME along its heading: progress past the length (the executor's overshoot check)
    x, y, h = HOME_POSE
    over, _ = along_cross(s, (x + 0.1 * math.cos(h), y + 0.1 * math.sin(h), h))
    assert over == pytest.approx(leg.length_m + 0.1)
    # behind the start: negative
    back, _ = along_cross(s, behind(2.1, 0.2))
    assert back < 0.0
