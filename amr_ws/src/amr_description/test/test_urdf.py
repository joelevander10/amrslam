"""The xacro must process, and its frames must be the spec §2.3 strings."""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

URDF = os.path.join(os.path.dirname(__file__), "..", "urdf", "amr.urdf.xacro")

# spec §2.3 minus floor_cam_frame (no camera, reconciliation D-4)
REQUIRED_LINKS = {"base_footprint", "base_link", "laser_frame", "imu_frame"}
VISUAL_ONLY_LINKS = {"wheel_left", "wheel_right"}


@pytest.fixture(scope="module")
def urdf() -> ET.Element:
    out = subprocess.run(["xacro", URDF], check=True, capture_output=True, text=True)
    return ET.fromstring(out.stdout)


def _joints(root: ET.Element) -> dict:
    return {j.find("child").get("link"): j.find("parent").get("link") for j in root.iter("joint")}


def test_links_present(urdf):
    links = {link.get("name") for link in urdf.iter("link")}
    assert REQUIRED_LINKS <= links
    assert VISUAL_ONLY_LINKS <= links
    assert "floor_cam_frame" not in links


def test_tree_shape(urdf):
    parent_of = _joints(urdf)
    assert parent_of["base_link"] == "base_footprint"
    for child in ("laser_frame", "imu_frame", "wheel_left", "wheel_right"):
        assert parent_of[child] == "base_link"


def test_all_joints_fixed(urdf):
    # No joint_states publisher exists; a non-fixed joint would leave a TF gap.
    assert {j.get("type") for j in urdf.iter("joint")} == {"fixed"}


def test_base_link_height_is_wheel_radius(urdf):
    for j in urdf.iter("joint"):
        if j.find("child").get("link") == "base_link":
            z = float(j.find("origin").get("xyz").split()[2])
            assert z == pytest.approx(0.09)


def test_measured_defaults(urdf):
    # Measured on the vehicle 2026-09-15; scan plane 0.110 m above the floor.
    for j in urdf.iter("joint"):
        child = j.find("child").get("link")
        xyz = [float(v) for v in j.find("origin").get("xyz").split()]
        if child == "laser_frame":
            assert xyz[0] == pytest.approx(0.964)
            assert xyz[2] + 0.09 == pytest.approx(0.110)
        if child == "imu_frame":
            assert xyz[0] == pytest.approx(0.092)
        if child == "wheel_left":
            assert xyz[1] == pytest.approx(0.487 / 2)


def test_args_override():
    out = subprocess.run(
        ["xacro", URDF, "laser_x:=0.42", "laser_yaw:=0.1"],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(out.stdout)
    for j in root.iter("joint"):
        if j.find("child").get("link") == "laser_frame":
            o = j.find("origin")
            assert o.get("xyz").split()[0] == "0.42"
            assert o.get("rpy").split()[2] == "0.1"
