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


def test_rpp_follows_a_faster_route_speed_and_brakes_no_harder_than_the_mux():
    assert rt.rpp_overrides(0.60) == {}  # nav2_params.yaml as written (gvievo-01)
    o = rt.rpp_overrides(0.80)
    assert o["FollowPath.desired_linear_vel"] == 0.80
    d = o["FollowPath.approach_velocity_scaling_dist"]
    assert d == 5.2 and 0.80**2 / d <= rt.MUX_AUTO_D_MAX


def test_gvievo_keeps_060():
    assert rt.route_speed() == rt.ROUTE_V_MPS_DEFAULT == 0.60  # the test profile is agv-01


def test_a_faster_route_speed_never_makes_the_unscanned_reverse_faster(monkeypatch):
    from amr_navigation import compiler

    monkeypatch.setattr(rt.Limits, "linear_mps", property(lambda self: 0.80))
    r = _route([{"id": "s1", "type": "reverse", "distance_m": 1.0}])
    assert compiler.compile_route(r).steps[0].v_mps == rt.REVERSE_V_MAX_MPS == 0.30
