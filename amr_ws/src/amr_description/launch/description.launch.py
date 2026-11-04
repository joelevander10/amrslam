"""robot_state_publisher from amr.urdf.xacro.

Every xacro arg is exposed as a launch argument so measured values can be
passed without editing the URDF, e.g. `laser_x:=0.42 laser_yaw:=0.01`.

The defaults are the vehicle's (AGV_PROFILE): the xacro's own for agv-01, and
config/vehicle.<profile>.yaml for any other vehicle (amr_description.vehicle).
The GLS camera's mount is NOT a second copy: its defaults are read from the profile
(gls.mount_x_m / mount_y_m / mount_yaw_rad), so TF shows what label_fix will use.
"""

import os

from agv_core import config as vehicle_config
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name, default, description). Defaults mirror the xacro; MEASURE = placeholder.
XACRO_ARGS = [
    ("wheel_radius", "0.09", "Wheel radius [m], profile wheel_dia_m / 2"),
    ("wheel_width", "0.05", "Wheel width [m] (visual only)"),
    ("track_width", "0.487", "Wheel track [m], measured"),
    ("chassis_length", "1.00", "Chassis box length [m] (MEASURE)"),
    ("chassis_width", "0.60", "Chassis box width [m] (MEASURE)"),
    ("chassis_height", "0.30", "Chassis box height [m] (MEASURE)"),
    ("chassis_x", "0.00", "Chassis centre ahead of axle [m] (MEASURE)"),
    ("chassis_z", "0.05", "Chassis underside above axle [m] (MEASURE)"),
    ("laser_x", "0.964", "nanoScan3 x from base_link [m]"),
    ("laser_y", "0.00", "nanoScan3 y from base_link [m] (MEASURE, assumed centreline)"),
    ("laser_z", "0.020", "nanoScan3 z from base_link [m]: 0.110 floor - 0.090 axle"),
    ("laser_yaw", "0.0", "nanoScan3 yaw [rad] (MEASURE, calibrate by scan alignment)"),
    ("imu_x", "0.092", "MLS IMU x from base_link [m]"),
    ("imu_y", "0.0", "MLS IMU y from base_link [m] (VERIFY)"),
    ("imu_z", "0.0", "MLS IMU z from base_link [m] (VERIFY)"),
    ("gls_x", str(vehicle_config.GLS_MOUNT_X_M), "GLS621 lens axis on the floor, x [m]: gls.mount_x_m"),
    ("gls_y", str(vehicle_config.GLS_MOUNT_Y_M), "GLS621 lens axis on the floor, y [m]: gls.mount_y_m"),
    ("gls_yaw", str(vehicle_config.GLS_MOUNT_YAW_RAD), "GLS621 housing yaw [rad]: gls.mount_yaw_rad"),
]


def generate_launch_description() -> LaunchDescription:
    from amr_description import vehicle  # noqa: PLC0415

    overrides = vehicle.xacro_args()
    unknown = set(overrides) - {n for n, _, _ in XACRO_ARGS}
    if unknown:
        raise ValueError(
            f"vehicle file for {vehicle.profile_name()} has unknown xacro args {sorted(unknown)}"
        )
    defaults = [(n, overrides.get(n, d), desc) for n, d, desc in XACRO_ARGS]
    xacro_file = os.path.join(get_package_share_directory("amr_description"), "urdf", "amr.urdf.xacro")

    cmd = [FindExecutable(name="xacro"), " ", xacro_file]
    for name, _, _ in XACRO_ARGS:
        cmd += [" ", f"{name}:=", LaunchConfiguration(name)]

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in defaults]
        + [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (bag replay, sim)",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(Command(cmd), value_type=str),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
