"""drivers.launch.py: base.launch.py real:=true (+ Foxglove) - the bench entry point.

Hardware Layer 1 plus the estimation chain: drive_node on can0, panel_node on
the DIO island (panel:=fake is a SIMULATED panel, bench only - never with the
vehicle on the floor and people nearby), nanoScan3, mux/odom/bias/EKF, URDF.
drive_node and panel_node take the can/dio owner locks (agv_core.ownerlock), so a
second owner refuses to start. Needs ROS_DOMAIN_ID=10 (env/vehicle.sh).

On a platform qr_analog profile (AGV_PROFILE=amr-qr-01) base.launch.py starts qr_base_node
instead of drive_node + panel_node (it owns drives, encoders, IMU and panel); pc_loss_ms,
feedback_hz and gyro_sign do not apply there and panel:=fake is refused.
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
            DeclareLaunchArgument(
                "lidar", default_value="true", description="also start the nanoScan3 driver"
            ),
            DeclareLaunchArgument("foxglove", default_value="false"),
            DeclareLaunchArgument(
                "pc_loss_ms", default_value="500", description="1016h on the drives; 0 = off"
            ),
            DeclareLaunchArgument(
                "feedback_hz", default_value="50.0", description="drive TPDO event timer rate"
            ),
            DeclareLaunchArgument("gyro_sign", default_value="1.0", description="+1 if CCW reads positive"),
            DeclareLaunchArgument(
                "panel", default_value="real", description="real (DIO island) | fake (SIMULATED, bench only)"
            ),
            include(
                "amr_bringup",
                "base.launch.py",
                real="true",
                lidar=cfg("lidar"),
                pc_loss_ms=cfg("pc_loss_ms"),
                feedback_hz=cfg("feedback_hz"),
                gyro_sign=cfg("gyro_sign"),
                panel=cfg("panel"),
            ),
            foxglove(IfCondition(cfg("foxglove"))),
        ]
    )
