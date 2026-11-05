"""Preset survey moves run at 0.40 m/s, capped like the routes at 95 % of the vehicle's top
speed: the AMR QR (about 0.28 m/s) plans them at 0.27 so the mux does not clip them."""

from agv_core import config

from amr_mission import survey_move as sm


def test_gvievo_keeps_0_40():
    assert sm.survey_speed() == sm.SURVEY_V_MPS == 0.40


def test_a_slow_vehicle_plans_its_preset_moves_at_what_its_wheels_can_do(monkeypatch):
    monkeypatch.setattr(config, "MAX_SPEED_MPS", 0.2827)
    assert sm.survey_speed() == 0.27
