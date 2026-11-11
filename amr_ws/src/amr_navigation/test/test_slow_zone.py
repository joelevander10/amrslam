"""Speed waypoints (2026-11-06): a straight's own lower speed, and the route speed per vehicle."""

import pytest
from amr_navigation.compiler import compile_route

from amr_navigation import route as rt


def _route(steps):
    return rt.Route.from_dict(
        {
            "schema_version": rt.SCHEMA_VERSION,
            "route_id": "r",
            "map": {"id": "m", "revision": 1, "sha256": "x"},
            "start": {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0},
            "steps": steps,
        }
    )


def test_a_slow_straight_between_two_normal_ones():
    r = _route(
        [
            {"id": "s1", "type": "straight", "to": {"x_m": 4.0, "y_m": 0.0}},
            {"id": "s2", "type": "straight", "to": {"x_m": 6.0, "y_m": 0.0}, "v_mps": 0.3},
            {"id": "s3", "type": "straight", "to": {"x_m": 10.0, "y_m": 0.0}},
        ]
    )
    c = compile_route(r)
    assert [s.v_mps for s in c.steps] == [rt.VEHICLE_V_MAX, 0.3, rt.VEHICLE_V_MAX]
    # saved back with its speed, the others without one
    d = r.to_dict()["steps"]
    assert "v_mps" not in d[0] and d[1]["v_mps"] == 0.3


def test_a_step_speed_never_raises_the_route_speed():
    r = _route([{"id": "s1", "type": "straight", "to": {"x_m": 2.0, "y_m": 0.0}, "v_mps": 1.9}])
    assert compile_route(r).steps[0].v_mps == rt.VEHICLE_V_MAX


@pytest.mark.parametrize("v", [0.05, 2.5, "fast", True])
def test_a_bad_step_speed_is_refused_naming_the_step(v):
    with pytest.raises(rt.RouteError) as e:
        _route([{"id": "s1", "type": "straight", "to": {"x_m": 2.0, "y_m": 0.0}, "v_mps": v}])
    assert e.value.step_id == "s1"


def test_rpp_runs_at_the_route_speed_and_its_ramp_fits_in_the_local_costmap():
    """2026-11-06: a 5.2 m ramp in a 5 x 5 m costmap held 0.80 m/s straights at 0.38 m/s."""
    import pathlib

    import yaml

    assert rt.rpp_overrides(0.60, qr=False) == {}  # nav2_params.yaml as written (gvievo-01)
    o = rt.rpp_overrides(0.80, qr=True)
    assert o["FollowPath.desired_linear_vel"] == 0.80
    cfg = pathlib.Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    lc = yaml.safe_load(cfg.read_text())["local_costmap"]["local_costmap"]["ros__parameters"]
    half = max(lc["width"], lc["height"]) / 2.0
    assert half == rt.LOCAL_COSTMAP_HALF_M
    assert o["FollowPath.approach_velocity_scaling_dist"] < half
    # the executor's stop profile and taper stay within what the mux brakes
    assert rt.QR_END_DECEL < rt.QR_AUTO_D_MAX


def test_gvievo_keeps_060():
    assert rt.route_speed() == rt.ROUTE_V_MPS_DEFAULT == 0.60  # the test profile is agv-01


def test_a_faster_route_speed_never_makes_the_unscanned_reverse_faster(monkeypatch):
    from amr_navigation import compiler

    monkeypatch.setattr(rt.Limits, "linear_mps", property(lambda self: 0.80))
    r = _route([{"id": "s1", "type": "reverse", "distance_m": 1.0}])
    assert compiler.compile_route(r).steps[0].v_mps == rt.REVERSE_V_MAX_MPS == 0.30


def test_an_arc_and_the_home_leg_take_their_own_speed_within_the_turn_rate_limit():
    """2026-11-11: a curve's speed set per step; empty = ARC_V_MPS; never above the route speed
    nor v/R over ARC_YAW_RATE_RATIO x VEHICLE_ARC_W_MAX."""
    arc = {"id": "s2", "type": "arc", "direction": "ccw", "angle_deg": 90.0, "radius_m": 2.0}
    r = _route([{"id": "s1", "type": "straight", "to": {"x_m": 4.0, "y_m": 0.0}}, arc])
    assert compile_route(r).steps[1].v_mps == rt.ARC_V_MPS
    r = _route([{"id": "s1", "type": "straight", "to": {"x_m": 4.0, "y_m": 0.0}}, {**arc, "v_mps": 0.5}])
    assert compile_route(r).steps[1].v_mps == 0.5
    assert r.to_dict()["steps"][1]["v_mps"] == 0.5
    cap = rt.ARC_YAW_RATE_RATIO * rt.VEHICLE_ARC_W_MAX * 1.0  # R 1 m: 0.405 m/s
    r = _route([{**arc, "radius_m": 1.0, "v_mps": 0.8}])
    assert abs(compile_route(r).steps[0].v_mps - min(cap, rt.VEHICLE_V_MAX)) < 1e-9
    r = _route([{**arc, "v_mps": 0.2}])
    assert compile_route(r).steps[0].v_mps == 0.2  # slower than the default too
    with pytest.raises(rt.RouteError):
        _route([{**arc, "v_mps": 0.01}])
    # the home leg: S-curve back to the start, its own speed
    home = [
        {"id": "s1", "type": "straight", "to": {"x_m": 4.0, "y_m": 0.0}},
        {"id": "s2", "type": "rotate", "direction": "ccw", "angle_deg": 90},
        {"id": "s3", "type": "straight", "to": {"x_m": 4.0, "y_m": 0.5}},
        {"id": "s4", "type": "rotate", "direction": "ccw", "angle_deg": 90},
        {"id": "s6", "type": "straight", "to": {"x_m": -4.0, "y_m": 0.5}},
        {"id": "s7", "type": "rotate", "direction": "ccw", "angle_deg": 180},
        {"id": "s5", "type": "home", "v_mps": 0.25},
    ]
    r = _route(home)
    c = compile_route(r)
    assert c.steps[-1].v_mps == 0.25 and r.to_dict()["steps"][-1] == {
        "id": "s5",
        "type": "home",
        "v_mps": 0.25,
    }
    assert _route(home[:-1] + [{"id": "s5", "type": "home"}]).to_dict()["steps"][-1] == {
        "id": "s5",
        "type": "home",
    }
