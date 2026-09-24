"""Shared launch construction (unified plan §3.2, §3.3).

`required(node)` returns the node plus an exit handler that shuts the whole
launch down when that node exits - for ANY exit code. A ROS node that dies
would otherwise leave its `ros2 launch` parent alive and looking healthy;
the supervisor treats any unrequested layer termination as a fault, so a
launch must end when a required member ends. During an orderly shutdown the
handler fires too and is harmless (the launch is already stopping).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def required(node: Node, what: str) -> list:
    return [
        node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[Shutdown(reason=f"required process {what} exited")],
            )
        ),
    ]


def include(package: str, launch_file: str, **launch_arguments) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", launch_file)),
        launch_arguments={k: v for k, v in launch_arguments.items()}.items(),
    )


def foxglove(condition) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"
            )
        ),
        condition=condition,
    )


def default_world() -> str:
    return os.path.join(get_package_share_directory("amr_maps"), "worlds", "sim_factory", "world.yaml")
