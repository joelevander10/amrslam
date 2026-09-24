"""mapping_layer.launch.py: live SLAM + survey coordinator, nothing else (unified plan U1).

One survey = one fresh instance of this launch: slam_toolbox has no reset, and
the coordinator's graph reference dies with the process. Both nodes are
REQUIRED - either exiting ends the layer, which the supervisor reads as a
fault unless it asked for the stop.

slam_toolbox after Humble (Jazzy, 2.8) is a LIFECYCLE node: started as a plain Node it
sits unconfigured, publishes no map->odom, and the supervisor faults the layer after
30 s ("new layer not ready"; AMR QR on Jazzy, 2026-09-24). There it is launched as a
LifecycleNode and driven configure -> activate here, like slam_toolbox's own launch files.

    generation:=N   stamped into MappingState so consumers can drop old layers
    internal:=true  remap the coordinator's services under /amr/internal/survey/*
                    (the supervisor owns the public /amr/survey/* API)
"""

import os
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_bringup.launch_helpers import required

SURVEY_SERVICES = ("start", "returned", "save", "abort")


def slam_param_files() -> list:
    """slam_mapping.yaml, then slam_mapping.<AGV_PROFILE>.yaml if the vehicle has overrides."""
    cfg = os.path.join(get_package_share_directory("amr_bringup"), "config")
    files = [os.path.join(cfg, "slam_mapping.yaml")]
    profile = os.environ.get("AGV_PROFILE") or "agv-01"
    overlay = os.path.join(cfg, f"slam_mapping.{profile}.yaml")
    if profile != "agv-01" and os.path.isfile(overlay):
        files.append(overlay)
    return files


def slam_is_lifecycle() -> bool:
    """True when the installed slam_toolbox is a lifecycle node (2.7 and later: Iron, Jazzy)."""
    try:
        xml = os.path.join(get_package_share_directory("slam_toolbox"), "package.xml")
        version = ElementTree.parse(xml).getroot().findtext("version") or ""
        major, minor = (int(x) for x in version.split(".")[:2])
        return (major, minor) >= (2, 7)
    except Exception:  # noqa: BLE001 - not installed (static tests): go by the distro
        return os.environ.get("ROS_DISTRO", "humble") != "humble"


def _slam(slam_params: list) -> list:
    if not slam_is_lifecycle():
        return required(
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=slam_params,
            ),
            "slam_toolbox",
        )
    from launch.actions import EmitEvent, LogInfo, RegisterEventHandler  # noqa: PLC0415
    from launch.events import matches_action  # noqa: PLC0415
    from launch_ros.actions import LifecycleNode  # noqa: PLC0415
    from launch_ros.event_handlers import OnStateTransition  # noqa: PLC0415
    from launch_ros.events.lifecycle import ChangeState  # noqa: PLC0415
    from lifecycle_msgs.msg import Transition  # noqa: PLC0415

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[*slam_params, {"use_lifecycle_manager": False}],
    )

    def transition(t: int) -> EmitEvent:
        return EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam), transition_id=t))

    return required(slam, "slam_toolbox") + [
        transition(Transition.TRANSITION_CONFIGURE),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    LogInfo(msg="[mapping_layer] slam_toolbox configured, activating"),
                    transition(Transition.TRANSITION_ACTIVATE),
                ],
            )
        ),
    ]


def _compose(context):
    internal = LaunchConfiguration("internal").perform(context).lower() == "true"
    remaps = [(f"/amr/survey/{s}", f"/amr/internal/survey/{s}") for s in SURVEY_SERVICES] if internal else []
    actions = _slam(slam_param_files())
    actions += required(
        Node(
            package="amr_mission",
            executable="mapping_session_node",
            name="mapping_session",
            output="screen",
            parameters=[
                {
                    "maps_dir": LaunchConfiguration("maps_dir"),
                    "generation": LaunchConfiguration("generation"),
                }
            ],
            remappings=remaps,
        ),
        "mapping_session",
    )
    # preset survey moves (press once, MANUAL authority): only exists while surveying. NOT
    # required: a helper that dies stops its move (0.2 s command lifetime) but not the survey
    actions.append(
        Node(
            package="amr_mission",
            executable="survey_move_node",
            name="survey_move",
            output="screen",
            respawn=True,  # a helper that exits comes back; a move never resumes by itself
            respawn_delay=2.0,
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("maps_dir", default_value=os.path.expanduser("~/amr_maps")),
            DeclareLaunchArgument("generation", default_value="0"),
            DeclareLaunchArgument("internal", default_value="false"),
            OpaqueFunction(function=_compose),
        ]
    )
