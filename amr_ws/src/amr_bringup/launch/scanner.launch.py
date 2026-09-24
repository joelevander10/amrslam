"""scanner.launch.py: the nanoScan3 driver and nothing else (unified plan U1).

No URDF here - base.launch.py owns the one robot_state_publisher. The parameter file
is the vehicle's (AGV_PROFILE): nanoscan3.yaml for gvievo-01, nanoscan3.<profile>.yaml
otherwise. For a
standalone scanner + TF session use lidar.launch.py. ROS owns UDP 6060.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def scanner_params() -> str:
    """nanoscan3.yaml for gvievo-01 (agv-01), nanoscan3.<AGV_PROFILE>.yaml for any other vehicle."""
    profile = os.environ.get("AGV_PROFILE") or "agv-01"
    name = "nanoscan3.yaml" if profile == "agv-01" else f"nanoscan3.{profile}.yaml"
    path = os.path.join(get_package_share_directory("amr_bringup"), "config", name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"AGV_PROFILE={profile}: no scanner config {path}")
    return path


def generate_launch_description() -> LaunchDescription:
    params = scanner_params()
    return LaunchDescription(
        [
            Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name="sick_safetyscanners2_node",
                output="screen",
                emulate_tty=True,
                parameters=[params],
            ),
        ]
    )
