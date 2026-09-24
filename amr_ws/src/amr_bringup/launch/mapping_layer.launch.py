"""mapping_layer.launch.py: live SLAM + survey coordinator, nothing else (unified plan U1).

One survey = one fresh instance of this launch: slam_toolbox (Humble) has no
reset, and the coordinator's graph reference dies with the process. Both nodes
are REQUIRED - either exiting ends the layer, which the supervisor reads as a
fault unless it asked for the stop.

    generation:=N   stamped into MappingState so consumers can drop old layers
    internal:=true  remap the coordinator's services under /amr/internal/survey/*
                    (the supervisor owns the public /amr/survey/* API)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_bringup.launch_helpers import required

SURVEY_SERVICES = ("start", "returned", "save", "abort")


def _compose(context):
    internal = LaunchConfiguration("internal").perform(context).lower() == "true"
    remaps = [(f"/amr/survey/{s}", f"/amr/internal/survey/{s}") for s in SURVEY_SERVICES] if internal else []
    slam_yaml = os.path.join(get_package_share_directory("amr_bringup"), "config", "slam_mapping.yaml")
    actions = required(
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_yaml],
        ),
        "slam_toolbox",
    )
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
