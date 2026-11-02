"""p0_localization.launch.py: AMCL beside a running LINE layer, for measurement only
(tracked plan 3, section 5, tool T1; runbook section 2).

    ros2 launch amr_bringup p0_localization.launch.py map_id:=site103 revision:=N \\
        params:=default|lean initial:=start|none

map_server, AMCL and the localization lifecycle manager on one verified map revision,
in the vehicle's domain (source amr_ws/env/vehicle.sh first). initial:=start publishes
the manifest's survey-start pose once, so park the vehicle on the survey start mark.

Publishes only /amcl_pose, /particle_cloud and map->odom, which nothing in LINE reads.
Commands nothing. Refused when the profile already runs tape localization in the LINE
layer (tape_loc.enabled): two AMCLs would fight over map->odom.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from amr_bringup import tape_loc


def _compose(context):
    from agv_core import config as vehicle  # noqa: PLC0415 - launch-time import

    vehicle.load()
    if vehicle.TAPE_LOC_ENABLED:
        raise RuntimeError("tape_loc.enabled is true: the LINE layer runs its own AMCL. "
                           "P0 is measured with it off.")
    cfg = LaunchConfiguration
    maps_dir = os.path.expanduser(cfg("maps_dir").perform(context))
    map_yaml, revision, manifest = tape_loc.verified_map(
        maps_dir, cfg("map_id").perform(context), cfg("revision").perform(context))
    params = cfg("params").perform(context)
    actions = tape_loc.group(map_yaml, params_set=params, monitor=True, require=True)
    initial = cfg("initial").perform(context)
    if initial == "start":
        actions.append(tape_loc.initial_pose_cmd(manifest.start, delay_s=8.0))
    elif initial != "none":
        raise RuntimeError(f"initial:={initial}: expected start or none")
    print(f"[p0_localization] {manifest.map_id} rev{revision} bundle {manifest.sha256[:12]}, "
          f"AMCL params {params}, initial {initial}")
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("maps_dir", default_value=os.environ.get("AMR_MAPS_DIR", "~/amr_maps")),
            DeclareLaunchArgument("map_id"),
            DeclareLaunchArgument("revision", default_value="latest"),
            DeclareLaunchArgument("params", default_value="default"),
            DeclareLaunchArgument("initial", default_value="start"),
            OpaqueFunction(function=_compose),
        ]
    )
