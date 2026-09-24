"""mapping.launch.py: standalone survey stack = base + web + mapping_layer (+ Foxglove).

sim:=true   base.launch.py real:=false with scan_synth against the sim_factory world
sim:=false  base.launch.py real:=true (drive_node on can0, panel, nanoScan3)

Diagnostics and simulation-test entry point only (review Q18): without the
supervisor the web pages have no lease or mode, so their jog/survey controls are
refused; use the ROS services. Interactive operation is amr.service (§3.1).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_bringup.launch_helpers import default_world, foxglove, include


def _base(context):
    cfg = LaunchConfiguration
    if cfg("sim").perform(context).lower() == "true":
        return [
            include(
                "amr_bringup",
                "base.launch.py",
                real="false",
                slip_noise_std=cfg("slip_noise_std"),
                scan_synth="true",
                world_yaml=cfg("world_yaml"),
                clutter_count=cfg("clutter_count"),
            )
        ]
    return [include("amr_bringup", "base.launch.py", real="true", lidar="true")]


def generate_launch_description() -> LaunchDescription:
    cfg = LaunchConfiguration
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="true"),
            DeclareLaunchArgument("slip_noise_std", default_value="0.02"),
            DeclareLaunchArgument("world_yaml", default_value=default_world()),
            DeclareLaunchArgument("clutter_count", default_value="0"),
            DeclareLaunchArgument("maps_dir", default_value=os.path.expanduser("~/amr_maps")),
            DeclareLaunchArgument("foxglove", default_value="false"),
            DeclareLaunchArgument("web", default_value="true", description="operator pages on :5001"),
            OpaqueFunction(function=_base),
            Node(
                package="amr_web",
                executable="web_node",
                name="amr_web",
                output="screen",
                parameters=[{"maps_dir": cfg("maps_dir")}],
                condition=IfCondition(cfg("web")),
            ),
            include("amr_bringup", "mapping_layer.launch.py", maps_dir=cfg("maps_dir")),
            foxglove(IfCondition(cfg("foxglove"))),
        ]
    )
