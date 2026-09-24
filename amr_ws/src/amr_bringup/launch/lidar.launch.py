"""lidar.launch.py: standalone scanner commissioning - nanoScan3 + URDF TF (+ Foxglove).

Not part of a supervised stack (base.launch.py includes scanner.launch.py
itself). Add `foxglove:=true` for the bridge on :8765; URDF args pass through.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from amr_bringup.launch_helpers import foxglove, include


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "foxglove", default_value="false", description="Also start foxglove_bridge"
            ),
            include("amr_bringup", "scanner.launch.py"),
            include("amr_description", "description.launch.py"),
            foxglove(IfCondition(LaunchConfiguration("foxglove"))),
        ]
    )
