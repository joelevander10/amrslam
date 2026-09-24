"""sim.launch.py: base.launch.py real:=false (+ Foxglove). No hardware is touched.

Drive it with `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from amr_bringup.launch_helpers import foxglove, include


def generate_launch_description() -> LaunchDescription:
    cfg = LaunchConfiguration
    return LaunchDescription(
        [
            DeclareLaunchArgument("slip_noise_std", default_value="0.0"),
            DeclareLaunchArgument("panel_auto", default_value="false"),
            DeclareLaunchArgument("scan_synth", default_value="false"),
            DeclareLaunchArgument("clutter_count", default_value="0"),
            DeclareLaunchArgument("foxglove", default_value="false"),
            include(
                "amr_bringup",
                "base.launch.py",
                real="false",
                slip_noise_std=cfg("slip_noise_std"),
                panel_auto=cfg("panel_auto"),
                scan_synth=cfg("scan_synth"),
                clutter_count=cfg("clutter_count"),
            ),
            foxglove(IfCondition(cfg("foxglove"))),
        ]
    )
