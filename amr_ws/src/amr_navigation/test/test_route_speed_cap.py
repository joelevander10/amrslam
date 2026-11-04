"""Route speeds are capped at 95 % of the vehicle's top speed (route._top_speed_cap): the AMR
QR's wheels top out near 0.28 m/s, and a 0.60 m/s plan there would be clipped by the mux
wheel limit, turning the S-curve's smooth approach to cruise into a corner."""

import importlib

from agv_core import config

from amr_navigation import route


def test_gvievo_keeps_its_route_speed():
    assert config.MAX_SPEED_MPS > 1.0 and route.VEHICLE_V_MAX == route.ROUTE_V_MPS == 0.60


def test_a_slow_vehicle_plans_at_what_its_wheels_can_do(monkeypatch):
    monkeypatch.setattr(config, "MAX_SPEED_MPS", 0.2827)
    try:
        r = importlib.reload(route)
        assert r.VEHICLE_V_MAX == 0.27
        assert r.Limits().linear_mps == 0.27
    finally:
        monkeypatch.undo()
        importlib.reload(route)
    assert route.VEHICLE_V_MAX == 0.60
