"""lidar.launch.py: standalone scanner commissioning - nanoScan3 + URDF TF.

Not part of a supervised stack (base.launch.py includes scanner.launch.py
itself). URDF args pass through. To look at the scan, run the normal service and use
the web live view.
"""

from launch import LaunchDescription

from amr_bringup.launch_helpers import include


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            include("amr_bringup", "scanner.launch.py"),
            include("amr_description", "description.launch.py"),
        ]
    )
