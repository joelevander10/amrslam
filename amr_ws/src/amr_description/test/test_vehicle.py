"""Per-vehicle geometry files (amr_description.vehicle) and their agreement with each other."""

import json
import os
import re

import pytest
import yaml

from amr_description import vehicle

SHARE = os.path.join(os.path.dirname(__file__), "..")  # the source tree has the share layout
REPO = os.path.abspath(os.path.join(SHARE, "..", "..", ".."))
URDF = os.path.join(SHARE, "urdf", "amr.urdf.xacro")


def _xacro_arg_names() -> set:
    return set(re.findall(r'<xacro:arg name="([a-z_]+)"', open(URDF).read()))


def test_agv01_keeps_its_legacy_files():
    assert vehicle.config_path("footprint", "agv-01", SHARE).endswith(
        os.path.join("config", "footprint.yaml")
    )
    assert vehicle.config_path("vehicle", "agv-01", SHARE) is None
    assert vehicle.xacro_args("agv-01", SHARE) == {}


def test_other_profiles_need_their_own_files():
    with pytest.raises(FileNotFoundError, match="no footprint file"):
        vehicle.config_path("footprint", "agv-99", SHARE)
    with pytest.raises(ValueError):
        vehicle.config_path("chassis", "agv-01", SHARE)


def test_profile_from_environment(monkeypatch):
    monkeypatch.delenv("AGV_PROFILE", raising=False)
    assert vehicle.profile_name() == "agv-01"
    monkeypatch.setenv("AGV_PROFILE", "amr-qr-01")
    assert vehicle.profile_name() == "amr-qr-01"


@pytest.mark.parametrize("profile", ["amr-qr-01"])
def test_vehicle_file_matches_urdf_profile_and_footprint(profile):
    args = vehicle.xacro_args(profile, SHARE)
    assert set(args) <= _xacro_arg_names()
    prof = json.load(open(os.path.join(REPO, "profiles", f"{profile}.json")))
    assert float(args["track_width"]) == pytest.approx(prof["vehicle"]["track_m"])
    assert float(args["wheel_radius"]) == pytest.approx(prof["vehicle"]["wheel_dia_m"] / 2)
    fp = yaml.safe_load(open(vehicle.config_path("footprint", profile, SHARE)))
    xs = [p[0] for p in fp["polygon"]]
    ys = [p[1] for p in fp["polygon"]]
    length, cx = float(args["chassis_length"]), float(args["chassis_x"])
    # the drawn chassis box and the collision polygon describe the same body
    assert min(xs) == pytest.approx(cx - length / 2, abs=1e-6)
    assert max(xs) == pytest.approx(cx + length / 2, abs=1e-6)
    assert max(ys) - min(ys) == pytest.approx(float(args["chassis_width"]))
    # the scanner sits inside the body outline, not in front of it
    assert float(args["laser_x"]) < max(xs)


def test_amr_qr_specified_values():
    args = vehicle.xacro_args("amr-qr-01", SHARE)
    assert float(args["laser_x"]) == pytest.approx(0.81)
    assert float(args["track_width"]) == pytest.approx(0.37652)
    assert float(args["chassis_length"]) == pytest.approx(1.18)
    assert float(args["chassis_width"]) == pytest.approx(0.675)
    assert float(args["imu_x"]) == 0.0
