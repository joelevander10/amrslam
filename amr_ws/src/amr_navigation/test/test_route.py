import math
import os

import numpy as np
import pytest
from amr_maps.generate_sim_factory import build
from amr_maps.grid import Grid, GridMeta
from amr_navigation.compiler import compile_route
from amr_navigation.route import Limits, MapRef, Route, RouteError, StartPose, Step
from amr_navigation.validate import validate

from amr_navigation import footprint as fpmod
from amr_navigation import store

FP = fpmod.Footprint(((-0.5, -0.35), (1.1, -0.35), (1.1, 0.35), (-0.5, 0.35)), 0.20)


class Manifest:  # the fields validate() reads from amr_mission.map_bundle.Manifest
    map_id, revision, sha256 = "sim_factory", 1, "abc123"


def route(steps, start=(0.0, 0.0, 0.0), repeat=1, sha="abc123") -> Route:
    return Route("r1", MapRef("sim_factory", 1, sha), StartPose(*start), steps, Limits(), repeat_count=repeat)


def straight(sid, x, y):
    return Step(sid, "straight", to=(x, y))


def rotate(sid, direction, angle):
    return Step(sid, "rotate", direction=direction, angle_deg=angle)


# ---- compiler ----------------------------------------------------------------


@pytest.mark.parametrize("direction", ["cw", "ccw"])
@pytest.mark.parametrize("angle", [45, 90, 180, 270])
def test_all_eight_turns_keep_full_magnitude(direction, angle):
    c = compile_route(route([rotate("t", direction, angle)]))
    st = c.steps[0]
    expected = math.radians(angle) * (1 if direction == "ccw" else -1)
    assert st.signed_angle_rad == pytest.approx(expected)
    assert st.end[2] == pytest.approx(math.atan2(math.sin(expected), math.cos(expected)))
    assert st.time_allowance_s > abs(expected) / Limits().angular_rad_s  # the route's own cap


def test_cw_270_is_not_ccw_90():
    a = compile_route(route([rotate("t", "cw", 270)])).steps[0]
    b = compile_route(route([rotate("t", "ccw", 90)])).steps[0]
    assert a.end[2] == pytest.approx(b.end[2])  # same final heading
    assert a.signed_angle_rad == pytest.approx(-3 * math.pi / 2)
    assert b.signed_angle_rad == pytest.approx(math.pi / 2)


def test_spec_example_ends_facing_west():
    r = route(
        [
            straight("s1", 3.0, 0.0),
            rotate("s2", "ccw", 90),
            straight("s3", 3.0, 2.0),
            rotate("s4", "cw", 270),
            straight("s5", 1.0, 2.0),
        ],
    )
    c = compile_route(r)
    assert c.steps[1].end[2] == pytest.approx(math.pi / 2)
    assert abs(c.steps[3].end[2]) == pytest.approx(math.pi)  # north -> west via the long clockwise sweep
    assert c.end[:2] == pytest.approx((1.0, 2.0))
    assert c.total_length_m == pytest.approx(7.0)
    assert len(c.steps[0].samples) == 61 and c.steps[0].samples[-1][:2] == pytest.approx((3.0, 0.0))


def test_bend_is_rejected_not_rounded():
    with pytest.raises(RouteError) as e:
        compile_route(route([straight("s1", 3.0, 0.02)]))
    assert e.value.step_id == "s1" and "turn" in str(e.value)


def test_backwards_and_zero_length_rejected():
    with pytest.raises(RouteError, match="forward"):
        compile_route(route([straight("s1", -1.0, 0.0)]))
    with pytest.raises(RouteError, match="forward"):
        compile_route(route([straight("s1", 0.01, 0.0)]))


def test_bad_angles_and_directions_rejected():
    with pytest.raises(RouteError, match="angle_deg"):
        compile_route(route([rotate("t", "cw", 60)]))
    with pytest.raises(RouteError, match="direction"):
        compile_route(route([rotate("t", "left", 90)]))
    with pytest.raises(RouteError, match="no steps"):
        compile_route(route([]))
    with pytest.raises(RouteError, match="duplicate"):
        compile_route(route([rotate("t", "cw", 90), rotate("t", "cw", 90)]))


def test_arbitrary_start_heading_then_explicit_turns():
    r = route(
        [straight("s1", 2 * math.cos(0.7), 2 * math.sin(0.7)), rotate("s2", "ccw", 45)],
        start=(0.0, 0.0, math.degrees(0.7)),
    )
    c = compile_route(r)
    assert c.steps[0].length_m == pytest.approx(2.0)
    assert c.end[2] == pytest.approx(0.7 + math.pi / 4)


def test_closure_detection():
    sq = [
        straight("a", 2.0, 0.0),
        rotate("b", "ccw", 90),
        straight("c", 2.0, 2.0),
        rotate("d", "ccw", 90),
        straight("e", 0.0, 2.0),
        rotate("f", "ccw", 90),
        straight("g", 0.0, 0.0),
        rotate("h", "ccw", 90),
    ]
    assert compile_route(route(sq)).closes
    assert not compile_route(route(sq[:-1])).closes


# ---- yaml round trip --------------------------------------------------------------


def test_yaml_round_trip():
    r = route([straight("s1", 3.0, 0.0), rotate("s2", "cw", 270)], repeat=2)
    back = Route.loads(r.dumps())
    assert back.to_dict() == r.to_dict()
    assert back.steps[1].angle_deg == 270 and back.steps[1].direction == "cw"
    with pytest.raises(RouteError, match="schema_version"):
        Route.loads("schema_version: 7\n")


# ---- footprint sweeps and validation ----------------------------------------------


def test_footprint_rasterize_and_reach():
    g = Grid(np.zeros((100, 100), dtype=np.int8), GridMeta(0.05, -2.5, -2.5))
    mask = fpmod.swept_line(g, fpmod.Footprint(FP.polygon, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    area = mask.sum() * 0.05 * 0.05
    assert area == pytest.approx(1.6 * 0.7, rel=0.05)
    assert FP.reach_m == pytest.approx(math.hypot(1.1, 0.35))
    disc = fpmod.swept_rotation(g, fpmod.Footprint(FP.polygon, 0.0), (0.0, 0.0))
    assert disc.sum() * 0.0025 == pytest.approx(math.pi * FP.reach_m**2, rel=0.05)
    # the exact sweep of a turn: a 360 is the disc, a 90 is a quadrant of it plus the body
    full = fpmod.swept_rotation(g, fpmod.Footprint(FP.polygon, 0.0), (0.0, 0.0), 0.0, 2 * math.pi)
    assert (full & ~disc).sum() == 0 and full.sum() > 0.97 * disc.sum()
    quarter = fpmod.swept_rotation(g, fpmod.Footprint(FP.polygon, 0.0), (0.0, 0.0), 0.0, math.pi / 2)
    assert (quarter & ~disc).sum() == 0 and 0.3 * disc.sum() < quarter.sum() < 0.6 * disc.sum()
    assert FP.as_costmap_string().startswith("[[-0.500, -0.350]")


def test_rotation_sweep_is_the_turn_actually_made():
    # A wall 0.9 m behind the axle: standing there and a 90 ccw are clear (the nose swings
    # ahead and left, the tail reaches 0.61 m), a 180 sweeps the nose through it. The old
    # reach disc (1.35 m with margin) refused all three.
    g = small_grid()
    r0, c0 = g.world_to_cell(-0.95, 0.0)
    g.data[r0 - 10 : r0 + 10, c0 - 1 : c0 + 1] = 100
    ok = validate(route([rotate("s1", "ccw", 90)]), Manifest, g, FP)
    assert ok.ok, [i.to_dict() for i in ok.issues]
    bad = validate(route([rotate("s1", "ccw", 180)]), Manifest, g, FP)
    assert [i.step_id for i in bad.issues if i.code == "clearance"] == ["s1"]
    # the bounds check follows the same sweep: tail 0.5 m from the east edge, facing west
    fp0 = fpmod.Footprint(FP.polygon, 0.0)
    edge = 2.5 - 0.5 - 0.001
    assert fpmod.rotation_outside(g, fp0, (edge, 0.0))  # disc of any turn
    assert not fpmod.rotation_outside(g, fp0, (edge, 0.0), math.pi, 0.0)
    assert fpmod.rotation_outside(g, fp0, (edge, 0.0), math.pi, math.pi)


def test_valid_route_in_the_aisle():
    g = build()
    r = route([straight("s1", 15.0, 0.0), rotate("s2", "ccw", 180), straight("s3", 2.0, 0.0)])
    v = validate(r, Manifest, g, FP)
    assert v.ok, [i.to_dict() for i in v.issues]
    assert v.compiled.total_length_m == pytest.approx(28.0)


def test_line_through_a_rack_is_rejected():
    g = build()
    r = route(
        [straight("s1", 5.0, 0.0), rotate("s2", "ccw", 90), straight("s3", 5.0, 4.0)]
    )  # north into rack row C
    v = validate(r, Manifest, g, FP)
    assert not v.ok
    codes = {(i.code, i.step_id) for i in v.issues}
    assert ("clearance", "s3") in codes


def test_rotation_sweep_next_to_a_wall_is_rejected():
    g = build()
    # Drive to 0.8 m from the west wall's inner face (x = -2.8) facing it: the 1.1 m nose sweeps into it.
    r = route([straight("s1", -2.0, 0.0), rotate("s2", "cw", 180)], start=(0.0, 0.0, 180.0))
    v = validate(r, Manifest, g, FP)
    assert not v.ok
    assert any(i.code == "clearance" and i.step_id == "s2" for i in v.issues)


def test_unknown_cells_block_and_keepout_blocks():
    g = build()
    g.data[
        g.world_to_cell(8.0, 0.0)[0] - 3 : g.world_to_cell(8.0, 0.0)[0] + 3,
        g.world_to_cell(8.0, 0.0)[1] - 3 : g.world_to_cell(8.0, 0.0)[1] + 3,
    ] = -1
    v = validate(route([straight("s1", 12.0, 0.0)]), Manifest, g, FP)
    assert any("unknown" in i.message and i.step_id == "s1" for i in v.issues)
    g2 = build()
    ko = Grid(np.zeros(g2.data.shape, dtype=np.int8), g2.meta)
    r, c = g2.world_to_cell(6.0, 0.0)
    ko.data[r - 2 : r + 2, c - 2 : c + 2] = 100
    v = validate(route([straight("s1", 12.0, 0.0)]), Manifest, g2, FP, keepout=ko)
    assert any("keepout" in i.message for i in v.issues)


def test_map_reference_and_repeat_rules():
    g = build()
    v = validate(route([straight("s1", 5.0, 0.0)], sha="other"), Manifest, g, FP)
    assert any(i.code == "map_hash" for i in v.issues)
    r = route([straight("s1", 5.0, 0.0)])
    r.map.revision = 2
    assert any(i.code == "map_ref" for i in validate(r, Manifest, g, FP).issues)
    v = validate(route([straight("s1", 5.0, 0.0)], repeat=3), Manifest, g, FP)
    assert any(i.code == "repeat" for i in v.issues)
    assert any(
        i.code == "repeat"
        for i in validate(route([straight("s1", 5.0, 0.0)], repeat=0), Manifest, g, FP).issues
    )


# ---- stores -----------------------------------------------------------------------


def test_route_store_revisions(tmp_path):
    r = route([straight("s1", 5.0, 0.0)])
    rev, path, sha = store.save_route(str(tmp_path), r)
    assert rev == 1 and path.endswith("rev1.yaml") and len(sha) == 64
    rev2, _, sha2 = store.save_route(str(tmp_path), route([straight("s1", 6.0, 0.0)]))
    assert rev2 == 2 and sha2 != sha
    assert store.list_routes(str(tmp_path), "sim_factory") == {"r1": [1, 2]}
    back, back_sha = store.load_route(str(tmp_path), "sim_factory", "r1", 1)
    assert back.revision == 1 and back_sha == sha
    p = store.save_mission(str(tmp_path), "m1", "sim_factory", 1, "abc123", "r1", 2, sha2)
    m = store.load_mission(str(tmp_path), "m1")
    assert m["route"] == {"id": "r1", "revision": 2, "sha256": sha2} and p.endswith("m1.yaml")
    assert [x["mission_id"] for x in store.list_missions(str(tmp_path))] == ["m1"]
    with pytest.raises(store.StoreError):
        store.save_mission(str(tmp_path), "../x", "sim_factory", 1, "a", "r1", 1, "b")


# ---- R10: map bounds are never "clear" -------------------------------------------------

SMALL = GridMeta(0.05, -2.5, -2.5)  # 5 m x 5 m, x and y in [-2.5, 2.5]


def small_grid():
    return Grid(np.zeros((100, 100), dtype=np.int8), SMALL)


def test_r10_dilation_does_not_wrap_across_edges():
    mask = np.zeros((20, 30), dtype=bool)
    mask[10, 0] = True  # left edge
    mask[0, 15] = True  # bottom edge
    out = fpmod.dilate(mask, 3)
    assert not out[:, -5:].any(), "left-edge cell appeared on the right edge"
    assert not out[-5:, :].any(), "bottom-edge cell appeared on the top edge"
    assert out[10, 3] and out[3, 15] and out[13, 0] and not out[10, 4]


def test_r10_margin_smaller_than_a_cell_still_dilates():
    assert fpmod.margin_cells(fpmod.Footprint(FP.polygon, 0.01), 0.05) == 1
    assert fpmod.margin_cells(fpmod.Footprint(FP.polygon, 0.20), 0.05) == 4  # 0.2/0.05 = 4.000000000000001
    assert fpmod.margin_cells(fpmod.Footprint(FP.polygon, 0.0), 0.05) == 0


@pytest.mark.parametrize(
    "steps,start,step_id",
    [
        ([straight("s1", 41.0, 40.0)], (40.0, 40.0, 0.0), None),  # wholly outside: empty in-map mask
        ([straight("s1", 2.0, 0.0)], (0.0, 0.0, 0.0), "s1"),  # nose overhangs the east edge
        ([rotate("s1", "ccw", 90), straight("s2", 0.0, 2.2)], (0.0, 0.0, 0.0), "s2"),  # north edge
        ([straight("s1", -2.0, 0.0)], (0.0, 0.0, 180.0), "s1"),  # west edge
        ([rotate("s1", "cw", 90), straight("s2", 0.0, -2.0)], (0.0, 0.0, 0.0), "s2"),  # south edge
    ],
)
def test_r10_routes_leaving_the_map_are_rejected(steps, start, step_id):
    v = validate(route(steps, start=start), Manifest, small_grid(), FP)
    assert not v.ok
    out = [i for i in v.issues if i.code == "clearance" and "outside the map" in i.message]
    assert out, [i.to_dict() for i in v.issues]
    if step_id:
        assert any(i.step_id == step_id for i in out)
    else:
        assert any(i.step_id is None for i in out)  # the start pose itself


def test_r10_rotation_and_corner_overhang():
    g = small_grid()
    fp0 = fpmod.Footprint(FP.polygon, 0.0)
    assert not fpmod.rotation_outside(g, fp0, (0.0, 0.0))
    for corner in ((2.0, 2.0), (-2.0, 2.0), (2.0, -2.0), (-2.0, -2.0)):
        assert fpmod.rotation_outside(g, fp0, corner)
    # a line whose rotated footprint pokes past the corner by a sliver of margin only
    reach = math.hypot(1.1, 0.35)
    edge = 2.5 - reach - 0.001
    assert not fpmod.rotation_outside(g, fp0, (edge, 0.0))
    assert fpmod.rotation_outside(g, fpmod.Footprint(FP.polygon, 0.01), (edge, 0.0))
    yaw = math.radians(45)
    assert fpmod.line_outside(g, fp0, (1.5, 1.5, yaw), (1.6, 1.6, yaw))
    assert not fpmod.line_outside(g, fp0, (-0.5, -0.5, yaw), (0.0, 0.0, yaw))
    c = fpmod.check(g, np.zeros(g.data.shape, dtype=bool), outside=True)
    assert c.occupied == c.unknown == c.keepout == c.cells == 0 and not c.clear


def test_r10_route_inside_small_map_still_valid():
    v = validate(route([straight("s1", 0.5, 0.0)], start=(-0.5, 0.0, 0.0)), Manifest, small_grid(), FP)
    assert v.ok, [i.to_dict() for i in v.issues]


# ---- R19: yaw-free maps only; keepout must be aligned ---------------------------------


def test_r19_rotated_grid_is_a_validation_issue():
    g = Grid(np.zeros((100, 100), dtype=np.int8), GridMeta(0.05, -2.5, -2.5, origin_yaw=0.2))
    v = validate(route([straight("s1", 0.5, 0.0)], start=(-0.5, 0.0, 0.0)), Manifest, g, FP)
    assert not v.ok and any(i.code == "map_geometry" for i in v.issues)


@pytest.mark.parametrize(
    "ko_meta,shape",
    [
        (GridMeta(0.05, -2.5, -2.5), (100, 99)),
        (GridMeta(0.10, -2.5, -2.5), (100, 100)),
        (GridMeta(0.05, -2.45, -2.5), (100, 100)),
        (GridMeta(0.05, -2.5, -2.5, origin_yaw=0.1), (100, 100)),
    ],
)
def test_r19_misaligned_keepout_is_rejected(ko_meta, shape):
    ko = Grid(np.zeros(shape, dtype=np.int8), ko_meta)
    v = validate(route([straight("s1", 0.5, 0.0)], start=(-0.5, 0.0, 0.0)), Manifest, small_grid(), FP, ko)
    assert not v.ok and any(i.code == "keepout" and "aligned" in i.message for i in v.issues)


# ---- R21: strict schema --------------------------------------------------------------


def good_dict():
    return route([straight("s1", 3.0, 0.0), rotate("s2", "cw", 270)], repeat=2).to_dict()


def _mut(path, value):
    d = good_dict()
    cur = d
    for k in path[:-1]:
        cur = cur[k]
    if value is KeyError:
        del cur[path[-1]]
    else:
        cur[path[-1]] = value
    return d


@pytest.mark.parametrize(
    "path,value",
    [
        (("repeat_count",), 2.5),
        (("repeat_count",), "2"),
        (("repeat_count",), True),
        (("repeat_count",), -1),
        (("repeat_count",), 10**9),
        (("repeat_count",), 101),  # amr_mission run_fsm MAX_PASSES = 100
        (("revision",), 1.5),
        (("schema_version",), "1"),
        (("limits", "linear_mps"), 0),
        (("limits", "linear_mps"), float("nan")),
        (("limits", "linear_mps"), float("inf")),
        (("limits", "linear_mps"), "fast"),
        (("limits", "angular_rad_s"), -0.3),
        (("limits", "warp"), 1.0),
        (("limits",), [1, 2]),
        (("start",), KeyError),
        (("start", "yaw_deg"), KeyError),
        (("start", "x_m"), None),
        (("start", "x_m"), 1e300),
        (("frame_id",), "odom"),
        (("steps",), {"a": 1}),
        (("steps",), [1]),
        (("steps",), [{"id": "s1", "type": "straight"}]),
        (("steps",), [{"id": "s1", "type": "straight", "to": {"x_m": 1.0}}]),
        (("steps",), [{"id": "s1", "type": "straight", "to": {"x_m": "1", "y_m": 0}}]),
        (("steps",), [{"id": "s1", "type": "rotate", "direction": "cw", "angle_deg": 90.5}]),
        (("steps",), [{"id": "s1", "type": "rotate", "direction": "cw", "angle_deg": "90"}]),
        (("steps",), [{"id": 7, "type": "rotate", "direction": "cw", "angle_deg": 90}]),
        (("steps",), [{"id": "s1", "type": "rotate", "direction": "up", "angle_deg": 90}]),
        (("steps",), [{"id": "s1", "type": "rotate", "angle_deg": 90}]),
        (("steps",), [{"id": "s", "type": "rotate", "direction": "cw", "angle_deg": 90}] * 501),
        (("route_id",), ["r"]),
        (("map",), "m"),
    ],
)
def test_r21_malformed_routes_raise_route_error(path, value):
    with pytest.raises(RouteError):
        Route.from_dict(_mut(path, value))


def test_r21_well_formed_inputs_still_accepted():
    back = Route.from_dict(good_dict())
    assert back.repeat_count == 2 and back.steps[1].angle_deg == 270
    d = _mut(("limits",), {"linear_mps": 0.5})  # JSON number from the browser
    d["start"] = {"x_m": 0, "y_m": 0, "yaw_deg": 0}
    d["steps"][1]["angle_deg"] = 270  # int from JSON; stored revisions carry 270.0
    assert Route.from_dict(d).limits.linear_mps == 0.5
    assert Route.from_dict(_mut(("limits",), KeyError)).limits == Limits()
    assert Route.from_dict(_mut(("repeat_count",), 0)).repeat_count == 0  # validate() reports it
    assert Route.from_dict(_mut(("repeat_count",), 100)).repeat_count == 100


def test_r21_validate_bounds_repeat_count_of_constructed_routes():
    g = build()
    for bad in (101, 2.5, True):
        r = route([straight("s1", 5.0, 0.0)])
        r.repeat_count = bad
        assert any(i.code == "repeat" for i in validate(r, Manifest, g, FP).issues), bad


def test_r21_compiler_refuses_zero_speed_and_unbounded_work():
    r = route([straight("s1", 3.0, 0.0)])
    r.limits.linear_mps = 0.0
    with pytest.raises(RouteError, match="linear_mps"):
        compile_route(r)
    r = route([straight("s1", 9000.0, 0.0), rotate("t", "ccw", 180), straight("s2", -9000.0, 0.0)])
    with pytest.raises(RouteError, match="too long"):
        compile_route(r)


# ---- R22/R23: immutable, serialised store --------------------------------------------


def test_r22_mission_id_conflict_and_idempotent_retry(tmp_path):
    d = str(tmp_path)
    p = store.save_mission(d, "m", "mapA", 1, "shaA", "r1", 1, "rsha")
    before = open(p, "rb").read()
    assert store.save_mission(d, "m", "mapA", 1, "shaA", "r1", 1, "rsha") == p  # same refs: ok
    with pytest.raises(store.StoreConflict):
        store.save_mission(d, "m", "mapB", 1, "shaB", "r1", 1, "rsha2")
    assert open(p, "rb").read() == before
    assert [x["map"]["id"] for x in store.list_missions(d)] == ["mapA"]


def test_r23_concurrent_route_writers_get_distinct_immutable_revisions(tmp_path):
    import hashlib
    import threading

    d = str(tmp_path)
    n = 12
    barrier = threading.Barrier(n)
    results, errors = [], []

    def writer(k):
        try:
            r = route([straight("s1", 1.0 + k, 0.0)])
            barrier.wait(timeout=10)
            results.append((k, *store.save_route(d, r)))
        except Exception as e:  # noqa: BLE001
            errors.append(e)

    threads = [threading.Thread(target=writer, args=(k,)) for k in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=30)
    assert not errors and len(results) == n
    assert sorted(rev for _, rev, _, _ in results) == list(range(1, n + 1))
    for k, rev, path, sha in results:
        data = open(path, "rb").read()
        assert hashlib.sha256(data).hexdigest() == sha
        assert (
            Route.loads(data.decode()).steps[0].to == (1.0 + k, 0.0) and f"revision: {rev}" in data.decode()
        )
    leftovers = [f for f in os.listdir(os.path.dirname(results[0][2])) if f.startswith(".tmp-")]
    assert leftovers == []


def test_r23_concurrent_mission_writers_one_winner(tmp_path):
    import threading

    d = str(tmp_path)
    n = 8
    barrier = threading.Barrier(n)
    ok, conflicts = [], []

    def writer(k):
        barrier.wait(timeout=10)
        try:
            ok.append((k, store.save_mission(d, "m", f"map{k}", 1, "s", "r", 1, "rs")))
        except store.StoreConflict:
            conflicts.append(k)

    threads = [threading.Thread(target=writer, args=(k,)) for k in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=30)
    assert len(ok) == 1 and len(conflicts) == n - 1
    assert store.load_mission(d, "m")["map"]["id"] == f"map{ok[0][0]}"


def test_r23_existing_revision_is_never_replaced(tmp_path):
    d = str(tmp_path)
    _, path, _ = store.save_route(d, route([straight("s1", 1.0, 0.0)]))
    with pytest.raises(store.StoreConflict):
        store._commit_new(path, b"other")
    assert b"other" not in open(path, "rb").read()


# ---- autonomous defaults 0.55 / 0.37 / arcs 0.40 (2026-09-19) ----


def test_limits_default_to_055_and_037_and_explicit_values_survive():
    d = good_dict()
    del d["limits"]
    r = Route.from_dict(d)
    assert (r.limits.linear_mps, r.limits.angular_rad_s, r.limits.arc_linear_mps) == (0.55, 0.37, 0.40)
    assert r.limits.long_linear_mps is None  # no boost unless the file says so
    c = compile_route(r)
    assert c.steps[1].time_allowance_s > math.radians(270) / 0.37
    d = good_dict()
    d["limits"] = {"linear_mps": 0.15, "angular_rad_s": 0.10}
    r = Route.from_dict(d)
    assert (r.limits.linear_mps, r.limits.angular_rad_s) == (0.15, 0.10)
    assert compile_route(r).steps[1].time_allowance_s > math.radians(270) / 0.10
    assert r.to_dict()["limits"]["linear_mps"] == 0.15  # never rewritten


def test_vehicle_ceilings_and_long_straight_boost():
    from amr_navigation.compiler import step_speed
    from amr_navigation.route import BASE_V_MAX, VEHICLE_V_MAX, VEHICLE_W_MAX

    # linear above the vehicle ceiling is refused; angular above it is CLAMPED (old files carry 0.30)
    d = good_dict()
    d["limits"] = {"linear_mps": VEHICLE_V_MAX + 0.01}
    with pytest.raises(RouteError, match="linear_mps"):
        Route.from_dict(d)
    # the BASE speed is bounded lower than the ceiling: only the boost goes near 0.85 (2026-09-19)
    assert (BASE_V_MAX, VEHICLE_V_MAX, VEHICLE_W_MAX) == (0.60, 0.85, 0.37)
    d["limits"] = {"linear_mps": 0.61}
    with pytest.raises(RouteError, match="linear_mps"):
        Route.from_dict(d)
    d["limits"] = {"linear_mps": 0.60, "long_linear_mps": 0.85, "arc_linear_mps": 0.60}
    Route.from_dict(d)
    d["limits"] = {"arc_linear_mps": 0.61}
    with pytest.raises(RouteError, match="arc_linear_mps"):
        Route.from_dict(d)
    d["limits"] = {"long_linear_mps": VEHICLE_V_MAX + 0.01}
    with pytest.raises(RouteError, match="long_linear_mps"):
        Route.from_dict(d)
    d["limits"] = {"angular_rad_s": VEHICLE_W_MAX + 0.1}
    r = Route.from_dict(d)
    assert r.limits.angular_rad_s == VEHICLE_W_MAX + 0.1 and r.limits.w_mps == VEHICLE_W_MAX
    assert r.to_dict()["limits"]["angular_rad_s"] == VEHICLE_W_MAX + 0.1  # the file is never rewritten
    # a boost slower than the base speed is a typo
    d["limits"] = {"linear_mps": 0.5, "long_linear_mps": 0.4}
    with pytest.raises(RouteError, match="long_linear_mps"):
        Route.from_dict(d)
    # null = off, and survives a round trip
    d["limits"] = {"linear_mps": 0.5, "long_linear_mps": None}
    r = Route.from_dict(d)
    assert r.limits.long_linear_mps is None and r.to_dict()["limits"]["long_linear_mps"] is None
    # per-step speed: strictly LONGER than the threshold boosts, equal does not
    d["limits"] = {"linear_mps": 0.5, "long_linear_mps": 0.7, "long_min_length_m": 4.0}
    lim = Route.from_dict(d).limits
    assert step_speed(lim, 4.0) == 0.5 and step_speed(lim, 4.01) == 0.7 and step_speed(lim, 2.0) == 0.5
    d["steps"] = [
        {"id": "s1", "type": "straight", "to": {"x_m": 3.0, "y_m": 0.0}},
        {"id": "s2", "type": "straight", "to": {"x_m": 9.0, "y_m": 0.0}},
    ]
    c = compile_route(Route.from_dict(d))
    assert [s.v_mps for s in c.steps] == [0.5, 0.7]
    assert c.steps[1].duration_est_s == pytest.approx(6.0 / 0.7)


def test_reverse_step_is_bounded_slow_and_backs_along_the_heading():
    from amr_navigation.route import REVERSE, REVERSE_MAX_M

    d = good_dict()
    d["limits"] = {"linear_mps": 0.5, "long_linear_mps": 0.7, "long_min_length_m": 1.0}
    d["steps"] = [
        {"id": "s1", "type": "straight", "to": {"x_m": 3.0, "y_m": 0.0}},
        {"id": "s2", "type": "reverse", "distance_m": 1.5},
    ]
    r = Route.from_dict(d)
    assert r.steps[1].type == REVERSE
    assert r.to_dict()["steps"][1] == {"id": "s2", "type": "reverse", "distance_m": 1.5}
    c = compile_route(r)
    st = c.steps[1]
    assert st.reverse and st.start == pytest.approx((3.0, 0.0, 0.0))
    assert st.end == pytest.approx((1.5, 0.0, 0.0))
    assert st.length_m == 1.5 and st.samples[0] == pytest.approx((3.0, 0.0, 0.0))
    assert st.samples[-1] == pytest.approx((1.5, 0.0, 0.0)) and all(s[2] == 0.0 for s in st.samples)
    assert st.v_mps == pytest.approx(0.25)  # half the BASE speed, never the boost (1.5 m > 1.0 m threshold)
    assert st.travel_yaw == pytest.approx(math.pi)
    assert c.total_length_m == pytest.approx(4.5) and c.end == pytest.approx((1.5, 0.0, 0.0))
    # bounded to REVERSE_MAX_M, and never zero
    for bad in (REVERSE_MAX_M + 0.01, 0.0, -1.0):
        d["steps"][1]["distance_m"] = bad
        with pytest.raises(RouteError, match="distance_m"):
            Route.from_dict(d)
    d["steps"][1]["distance_m"] = REVERSE_MAX_M
    assert compile_route(Route.from_dict(d)).steps[1].length_m == REVERSE_MAX_M


def test_arc_step_bounds_geometry_and_speed():
    from amr_navigation.compiler import arc_speed
    from amr_navigation.route import ARC, ARC_MIN_RADIUS_M

    from amr_navigation import footprint as fpmod

    d = good_dict()
    d["limits"] = {"linear_mps": 0.5, "long_linear_mps": 0.7, "long_min_length_m": 1.0, "angular_rad_s": 0.34}
    arc = {"id": "s1", "type": "arc", "direction": "ccw", "angle_deg": 90, "radius_m": 1.0}
    d["steps"] = [arc]
    r = Route.from_dict(d)
    assert r.steps[0].type == ARC and r.to_dict()["steps"][0] == {**arc, "angle_deg": 90.0}
    for bad in (
        {"angle_deg": 44},
        {"angle_deg": 181},
        {"radius_m": ARC_MIN_RADIUS_M - 0.01},
        {"direction": "left"},
    ):
        d["steps"] = [{**arc, **bad}]
        with pytest.raises(RouteError):
            Route.from_dict(d)
    # ccw 90 deg, R 1 from the origin heading +x: centre (0, 1), end (1, 1) heading +y
    d["steps"] = [arc]
    c = compile_route(Route.from_dict(d))
    st = c.steps[0]
    assert st.centre == pytest.approx((0.0, 1.0)) and st.end == pytest.approx((1.0, 1.0, math.pi / 2))
    assert st.length_m == pytest.approx(math.pi / 2) and st.signed_angle_rad == pytest.approx(math.pi / 2)
    assert st.samples[0] == pytest.approx((0.0, 0.0, 0.0)) and st.samples[-1] == pytest.approx(st.end)
    mid = st.samples[len(st.samples) // 2]
    assert math.hypot(mid[0] - 0.0, mid[1] - 1.0) == pytest.approx(1.0) and 0 < mid[2] < math.pi / 2
    assert c.total_turn_rad == pytest.approx(math.pi / 2) and c.total_length_m == pytest.approx(math.pi / 2)
    # speed (2026-09-19): arc_linear_mps (0.40), never the boost, never above linear_mps, or less
    # where v/R would pass 90 % of the ARC turn ceiling 0.45 (R 0.5 would ask 0.2025)
    assert r.limits.arc_linear_mps == 0.40
    assert st.v_mps == pytest.approx(0.40) and st.v_mps == arc_speed(r.limits, 1.0)
    assert arc_speed(r.limits, 5.0) == pytest.approx(0.40) and arc_speed(r.limits, 0.5) == pytest.approx(
        0.2025
    )
    from amr_navigation.route import Limits

    slow = Limits(linear_mps=0.30)  # a slow route: the arc never outruns its straights
    assert arc_speed(slow, 3.0) == pytest.approx(0.30)
    assert Limits(linear_mps=0.30).arc_linear_mps == 0.40  # stored as is, applied as the lower
    # cw mirrors; 180 deg ends across the diameter
    d["steps"] = [{**arc, "direction": "cw", "angle_deg": 180}]
    st = compile_route(Route.from_dict(d)).steps[0]
    assert st.centre == pytest.approx((0.0, -1.0))
    assert st.end[0] == pytest.approx(0.0) and st.end[1] == pytest.approx(-2.0)
    assert abs(abs(st.end[2]) - math.pi) < 1e-9
    # four 90 deg arcs close a loop
    d["steps"] = [{**arc, "id": f"s{i}"} for i in range(4)]
    assert compile_route(Route.from_dict(d)).closes
    # the sweep covers the footprint at the start, the middle and the end of the arc
    import numpy as np
    from amr_maps.grid import Grid, GridMeta

    g = Grid(np.zeros((120, 120), dtype=np.int8), GridMeta(0.05, -3.0, -3.0))
    fp = fpmod.Footprint(polygon=[[-0.3, -0.2], [0.5, -0.2], [0.5, 0.2], [-0.3, 0.2]], margin_m=0.0)
    d["steps"] = [arc]
    st = compile_route(Route.from_dict(d)).steps[0]
    m = fpmod.swept_arc(g, fp, st.centre, st.radius_m, st.start[2], st.signed_angle_rad)
    for x, y, _ in (st.samples[0], st.samples[len(st.samples) // 2], st.samples[-1]):
        r_, c_ = g.world_to_cell(x, y)
        assert m[r_, c_]
    assert not fpmod.arc_outside(g, fp, st.centre, st.radius_m, st.start[2], st.signed_angle_rad)
    assert fpmod.arc_outside(g, fp, (2.5, 1.0), 1.0, 0.0, math.pi / 2)  # runs off the +x edge


# ---- dynamic-mapping plan §1.2/§1.3: mapped clutter in a dynamic area is provisional -----------


def _trolley(g, x=8.0, y=0.0, half=3):
    r, c = g.world_to_cell(x, y)
    g.data[r - half : r + half, c - half : c + half] = 100
    return r, c


def _dynamic_around(g, r, c, half=6):
    d = Grid(np.zeros(g.data.shape, dtype=np.int8), g.meta)
    d.data[r - half : r + half, c - half : c + half] = 100
    return d


def test_mapped_trolley_in_a_dynamic_area_validates_with_one_info_issue():
    g = build()
    r, c = _trolley(g)
    rt = route([straight("s1", 12.0, 0.0)])
    blocked = validate(rt, Manifest, g, FP)
    assert not blocked.ok and any(i.step_id == "s1" and "occupied" in i.message for i in blocked.issues)
    v = validate(rt, Manifest, g, FP, dynamic=_dynamic_around(g, r, c))
    assert v.ok, [i.to_dict() for i in v.issues]
    assert [(i.code, i.step_id, i.severity) for i in v.issues] == [("provisional", "s1", "info")]
    assert "dynamic area" in v.issues[0].message and v.issues[0].to_dict()["severity"] == "info"


def test_dynamic_area_elsewhere_does_not_help_and_keepout_still_blocks():
    g = build()
    _trolley(g)
    r2, c2 = g.world_to_cell(2.0, -6.0)  # far from the line
    assert not validate(
        route([straight("s1", 12.0, 0.0)]), Manifest, g, FP, dynamic=_dynamic_around(g, r2, c2)
    ).ok
    g2 = build()
    r, c = g2.world_to_cell(6.0, 0.0)
    ko = Grid(np.zeros(g2.data.shape, dtype=np.int8), g2.meta)
    ko.data[r - 2 : r + 2, c - 2 : c + 2] = 100
    v = validate(
        route([straight("s1", 12.0, 0.0)]), Manifest, g2, FP, keepout=ko, dynamic=_dynamic_around(g2, r, c)
    )
    assert not v.ok and any("keepout" in i.message for i in v.issues)


def test_unknown_inside_a_dynamic_area_is_provisional_outside_it_blocks():
    g = build()
    r, c = g.world_to_cell(8.0, 0.0)
    g.data[r - 3 : r + 3, c - 3 : c + 3] = -1  # the floor a trolley hid during the survey
    rt = route([straight("s1", 12.0, 0.0)])
    assert any("unknown" in i.message for i in validate(rt, Manifest, g, FP).issues)
    v = validate(rt, Manifest, g, FP, dynamic=_dynamic_around(g, r, c))
    assert v.ok and v.issues[0].code == "provisional"


def test_dynamic_area_only_partly_covering_the_trolley_still_blocks():
    g = build()
    r, c = _trolley(g, half=4)
    v = validate(
        route([straight("s1", 12.0, 0.0)]), Manifest, g, FP, dynamic=_dynamic_around(g, r, c, half=2)
    )
    assert not v.ok  # the cells outside the mark are ordinary occupied cells


def test_misaligned_dynamic_mask_is_an_error_and_never_indexed():
    g = small_grid()
    d = Grid(np.zeros((3, 3), dtype=np.int8), g.meta)
    v = validate(route([straight("s1", 0.5, 0.0)], start=(-0.5, 0.0, 0.0)), Manifest, g, FP, dynamic=d)
    assert not v.ok and any(i.code == "dynamic" and "aligned" in i.message for i in v.issues)


def test_rotation_and_arc_sweeps_honour_the_dynamic_area():
    g = build()
    r, c = _trolley(g, x=5.0, y=0.9, half=2)  # beside the turn at x = 5
    rt = route([straight("s1", 5.0, 0.0), rotate("s2", "ccw", 90)])
    assert not validate(rt, Manifest, g, FP).ok
    v = validate(rt, Manifest, g, FP, dynamic=_dynamic_around(g, r, c, half=4))
    assert v.ok and [i.step_id for i in v.issues if i.code == "provisional"] == ["s2"]
