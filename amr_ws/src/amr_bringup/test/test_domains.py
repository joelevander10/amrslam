"""The vehicle and a simulation can never share a ROS graph (spec §9)."""

import os

import pytest

from amr_bringup import domains


@pytest.fixture
def env(monkeypatch):
    def set_(v):
        if v is None:
            monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
        else:
            monkeypatch.setenv("ROS_DOMAIN_ID", str(v))

    return set_


def test_hardware_needs_exactly_the_vehicle_domain(env):
    env(domains.VEHICLE)
    assert domains.require_vehicle_domain("x") == domains.VEHICLE
    for wrong in (None, 0, domains.SIM, 61):
        env(wrong)
        with pytest.raises(RuntimeError, match="vehicle"):
            domains.require_vehicle_domain("x")


def test_simulation_refuses_the_vehicle_domain_and_the_unset_default(env):
    env(domains.SIM)
    assert domains.refuse_vehicle_domain("x") == domains.SIM
    for d in domains.TEST_RANGE:
        env(d)
        assert domains.refuse_vehicle_domain("x") == d
    env(domains.VEHICLE)
    with pytest.raises(RuntimeError, match="simulation"):
        domains.refuse_vehicle_domain("x")
    env(None)
    with pytest.raises(RuntimeError, match="unset"):
        domains.refuse_vehicle_domain("x")


def test_the_pinned_test_domains_are_neither_vehicle_nor_sim():
    assert domains.VEHICLE not in domains.TEST_RANGE
    assert domains.SIM not in domains.TEST_RANGE
    assert domains.VEHICLE != domains.SIM


def test_garbage_is_an_error(env):
    env("ten")
    with pytest.raises(RuntimeError, match="not an integer"):
        domains.current()


def test_env_files_match_the_constants():
    root = os.path.join(os.path.dirname(__file__), "..", "..", "..", "env")
    veh = open(os.path.join(root, "vehicle.sh")).read()
    sim = open(os.path.join(root, "sim.sh")).read()
    assert f"ROS_DOMAIN_ID={domains.VEHICLE}\n" in veh
    assert f"ROS_DOMAIN_ID={domains.SIM}\n" in sim
    assert "cyclonedds-local.xml" in veh and "cyclonedds-local.xml" in sim
