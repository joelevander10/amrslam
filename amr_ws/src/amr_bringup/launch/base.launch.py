"""base.launch.py: the persistent base layer (unified plan §3.1, U1).

    supervised:=true   mux and drive_node require the supervisor's ControlLease
                       and the keyboard teleop input is off (production);
                       false = unsupervised bench (wrappers), no lease needed
    real:=true    drive_node (can0), panel_node (DIO) or fake panel, nanoScan3
                  - or, for a platform qr_analog profile (AGV_PROFILE=amr-qr-01),
                  qr_base_node (analog drives, CANopen encoders, WitMotion IMU and
                  the panel, one owner) + nanoScan3
    real:=false   fake_base, fake_imu, fake_panel, optional scan_synth (the profile's
                  geometry, so a QR sim is a QR-sized vehicle)
    both          exactly one robot_state_publisher, cmd_mux, diff_drive_odom,
                  imu_bias, scan_gate, ekf_local

No web app, no Foxglove, no SLAM, no AMCL: those are the supervisor's other
groups. Every node listed as REQUIRED ends the launch when it exits, so a
dead node cannot hide behind a live `ros2 launch` parent.

Domain guard: real:=true needs ROS_DOMAIN_ID=10; real:=false refuses it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_bringup import domains
from amr_bringup.launch_helpers import default_world, include, required


def _qr_base(cfg, context, gate) -> list:
    """AMR QR hardware: one node owns drives, encoders, IMU and panel (no panel_node)."""
    if cfg("panel").perform(context) != "real":
        raise RuntimeError(
            "panel:=fake is not available on a qr_analog vehicle: qr_base_node owns the DIO "
            "island and publishes the panel itself (a second publisher would fight it)"
        )
    return required(
        Node(
            package="amr_base",
            executable="qr_base_node",
            name="qr_base_node",
            output="screen",
            emulate_tty=True,
            # gyro sign comes from the profile (qr_base.imu_gyro_sign), not this launch's gyro_sign
            parameters=[gate],
        ),
        "qr_base_node",
    )


def _laser_pose() -> dict:
    """scan_synth's laser offset from this vehicle's geometry file ({} = its gvievo-01 defaults)."""
    from amr_description import vehicle  # noqa: PLC0415

    args = vehicle.xacro_args()
    return {k: float(args[k]) for k in ("laser_x", "laser_y", "laser_yaw") if k in args}


def _compose(context):
    real = LaunchConfiguration("real").perform(context).lower() == "true"
    what = f"base.launch.py real:={str(real).lower()}"
    domains.require_vehicle_domain(what) if real else domains.refuse_vehicle_domain(what)
    ekf_yaml = os.path.join(get_package_share_directory("amr_localization"), "config", "ekf.yaml")
    cfg = LaunchConfiguration
    supervised = cfg("supervised").perform(context).lower() == "true"
    gate = {"require_supervisor": supervised}

    from amr_base.agv_repo import config  # noqa: PLC0415 - the vehicle profile (AGV_PROFILE)

    actions = [include("amr_description", "description.launch.py")]
    if real and config.PLATFORM == config.PLATFORM_QR:
        actions += _qr_base(cfg, context, gate)
        if cfg("lidar").perform(context).lower() == "true":
            actions.append(include("amr_bringup", "scanner.launch.py"))
    elif real:
        actions += required(
            Node(
                package="amr_base",
                executable="drive_node",
                name="drive_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "pc_loss_ms": cfg("pc_loss_ms"),
                        "feedback_hz": cfg("feedback_hz"),
                        "gyro_sign": cfg("gyro_sign"),
                        **gate,
                    }
                ],
            ),
            "drive_node",
        )
        actions += required(
            Node(
                package="amr_base",
                executable="panel_node",
                name="panel_node",
                output="screen",
                condition=LaunchConfigurationEquals("panel", "real"),
            ),
            "panel_node",
        )
        actions.append(
            Node(
                package="amr_sim",
                executable="fake_panel_node",
                name="fake_panel",
                output="screen",
                parameters=[{"auto": False}],
                condition=LaunchConfigurationEquals("panel", "fake"),
            )
        )
        if cfg("lidar").perform(context).lower() == "true":
            actions.append(include("amr_bringup", "scanner.launch.py"))
    else:
        actions += required(
            Node(
                package="amr_sim",
                executable="fake_base_node",
                name="fake_base",
                output="screen",
                parameters=[{"slip_noise_std": cfg("slip_noise_std"), **gate}],
            ),
            "fake_base",
        )
        actions.append(Node(package="amr_sim", executable="fake_imu_node", name="fake_imu", output="screen"))
        actions.append(
            Node(
                package="amr_sim",
                executable="fake_panel_node",
                name="fake_panel",
                output="screen",
                parameters=[{"auto": cfg("panel_auto")}],
            )
        )
        actions.append(
            Node(
                package="amr_sim",
                executable="scan_synth_node",
                name="scan_synth",
                output="screen",
                parameters=[
                    {"world_yaml": cfg("world_yaml"), "clutter_count": cfg("clutter_count"), **_laser_pose()}
                ],
                condition=IfCondition(cfg("scan_synth")),
            )
        )
    actions += required(
        Node(
            package="amr_base",
            executable="cmd_mux_kinematics_node",
            name="cmd_mux_kinematics",
            output="screen",
            # Gentle autonomous starts on the vehicle (3.3 s to 0.5 m/s, 0.85 s to 0.34 rad/s); stops
            # at the hardware-class 0.5 m/s^2 / 1.0 rad/s^2, stated explicitly: leaving delta_max at
            # its "same as alpha_max" default made every Spin coast 2.6 deg past its target
            # (0.19 rad/s stopped at 0.4 rad/s^2, vehicle 2026-09-17).
            # Manual (pendant / browser jog, 2026-09-18): 0.5 m/s, S-curve to 0.3 m/s^2 with
            # 1.0 m/s^3 (1.9 s to full speed); driving + turning arcs with the slow wheel at 75 %
            # of the fast one; a spin in place stays at 0.3 rad/s.
            parameters=[
                {
                    **gate,
                    "teleop_enabled": not supervised,
                    "a_max": 0.15,
                    "alpha_max": 0.4,
                    "d_max": 0.5,
                    "delta_max": 1.0,
                    "manual_a_max": 0.3,
                    "manual_jerk": 1.0,
                    "pendant_v_m_s": 0.5,
                    "pendant_w_rad_s": 0.39,
                    "pendant_turn_ratio": 0.75,
                    "survey_w_max_rad_s": 0.27,  # manual spin cap while surveying (pendant + jog)
                }
            ],
        ),
        "cmd_mux_kinematics",
    )
    actions += required(
        Node(package="amr_base", executable="diff_drive_odom_node", name="diff_drive_odom", output="screen"),
        "diff_drive_odom",
    )
    actions.append(
        Node(package="amr_localization", executable="imu_bias_node", name="imu_bias", output="screen")
    )
    # /scan_gated for slam_toolbox and AMCL: scans released only once their odom
    # transform exists, so their tf2 MessageFilters take the synchronous path
    # (the asynchronous one hung slam_toolbox on the vehicle, 2026-09-17).
    actions += required(
        Node(
            package="amr_localization",
            executable="scan_gate_node",
            name="scan_gate",
            output="screen",
            # the AMR QR's nanoScan3 leaves lone returns in open space (2026-09-24 surveys)
            parameters=[
                {
                    "despeckle": config.PLATFORM == config.PLATFORM_QR,
                    # phantom beams of this scanner's scratched window, learnt on the robot
                    # (ros2 run amr_localization scan_mask_learn --write); absent = no mask
                    "mask_file": os.path.join(
                        os.environ.get("AMR_STATE_DIR", os.path.expanduser("~/.amr")), "scan_mask.yaml"
                    )
                    if config.PLATFORM == config.PLATFORM_QR
                    else "",
                }
            ],
        ),
        "scan_gate",
    )
    if supervised:  # the /blind replacement (unified plan §7.2); an exclusive IDLE substate
        actions.append(
            Node(
                package="amr_base",
                executable="commissioning_node",
                name="commissioning_node",
                output="screen",
                parameters=[{"state_dir": os.environ.get("AMR_STATE_DIR", os.path.expanduser("~/.amr"))}],
            )
        )
    actions += required(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            output="screen",
            parameters=[ekf_yaml],
        ),
        "ekf_local",
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("real", default_value="false", description="true = vehicle hardware"),
            DeclareLaunchArgument(
                "supervised", default_value="false", description="true = ControlLease required (production)"
            ),
            # hardware
            DeclareLaunchArgument(
                "lidar", default_value="true", description="real: also start the nanoScan3"
            ),
            DeclareLaunchArgument("pc_loss_ms", default_value="500"),
            DeclareLaunchArgument("feedback_hz", default_value="50.0"),
            DeclareLaunchArgument("gyro_sign", default_value="1.0"),
            DeclareLaunchArgument(
                "panel", default_value="real", description="real: DIO island | fake: SIMULATED, bench only"
            ),
            # simulation
            DeclareLaunchArgument("slip_noise_std", default_value="0.0"),
            DeclareLaunchArgument("panel_auto", default_value="false"),
            DeclareLaunchArgument("scan_synth", default_value="false", description="sim: raycast the world"),
            DeclareLaunchArgument("world_yaml", default_value=default_world()),
            DeclareLaunchArgument("clutter_count", default_value="0"),
            OpaqueFunction(function=_compose),
        ]
    )
