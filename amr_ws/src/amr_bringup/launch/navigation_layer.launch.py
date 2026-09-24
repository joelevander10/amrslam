"""navigation_layer.launch.py: one exact map revision -> map_server, AMCL,
localization monitor, controller, behaviors, route executor (unified plan U1).

The bundle <maps_dir>/<map_id>/rev<revision>/ is verified (hashes) before
anything starts. No base, no web, no Foxglove, no SLAM. A map change is a
new instance of this launch; nothing here is reused across maps.

    revision:=N        exact revision; "latest" is resolved ONCE here for the
                       standalone wrappers - the supervisor always passes a number
    generation:=N      stamped into RunState/LocalizationState/MotionPermit
    autostart:=true    lifecycle managers bring Nav2 up on their own (wrappers);
                       the supervisor passes false and drives them explicitly
    blank_dynamic:=false
                       AMCL localises against a derived map with the revision's
                       dynamic areas set to unknown (dynamic-mapping plan §1.2),
                       served by a second map_server on /map_loc; /map (web view,
                       localization monitor) stays the reviewed map. Opt-in until the
                       plain-vs-blanked AMCL comparison has been measured on the
                       vehicle (plan Increment 4). Env default: AMR_LOC_BLANK_DYNAMIC.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_base import gating
from amr_bringup.launch_helpers import required


def spin_plugin(distro: str | None = None) -> str:
    """Nav2 behavior plugin name for Spin. Humble/Iron use "pkg/Class", Jazzy and newer
    "pkg::Class" (docs.nav2.org, behavior server configuration). nav2_params.yaml keeps the
    Humble spelling for gvievo-01; this overrides it on the distro actually running."""
    distro = (distro or os.environ.get("ROS_DISTRO") or "humble").lower()
    return "nav2_behaviors/Spin" if distro in ("humble", "iron") else "nav2_behaviors::Spin"


def _costmap_footprint_file(footprint: str, generation: int) -> str:
    """Node-scoped ROS params YAML carrying the selected footprint to the local costmap."""
    state_dir = os.environ.get("AMR_STATE_DIR", os.path.expanduser("~/.amr"))
    os.makedirs(state_dir, exist_ok=True)
    path = os.path.join(state_dir, f"costmap_footprint_gen{generation}.yaml")
    doc = {"local_costmap": {"local_costmap": {"ros__parameters": {"footprint": footprint}}}}
    with open(path, "w") as fh:
        yaml.safe_dump(doc, fh)
    return path


def _localization_map_file(map_yaml: str, dynamic_yaml: str, generation: int, min_occupied: int) -> str:
    """AMCL's map: the reviewed map with dynamic-area cells set to unknown, so a trolley that
    moved leaves no landmark to pull the pose towards. Refuses a map with too little fixed
    structure left to localise against."""
    from amr_maps import grid as gridio  # noqa: PLC0415 - launch-time import

    grid = gridio.read(map_yaml)
    dyn = gridio.read(dynamic_yaml)
    problem = gridio.mask_misalignment(grid, dyn)
    if problem:
        raise RuntimeError(f"dynamic mask not aligned with the map: {problem}")
    mask = dyn.data >= 65
    occ = grid.data >= 65
    blanked = int((occ & mask).sum())
    remaining = int((occ & ~mask).sum())
    print(
        f"[navigation_layer] AMCL map: {blanked} dynamic cells blanked of {int(occ.sum())} occupied, "
        f"{remaining} remain"
    )
    if remaining < min_occupied:
        raise RuntimeError(
            f"only {remaining} occupied cells remain outside dynamic areas (< {min_occupied}): "
            "nothing fixed to localise against; shrink the dynamic areas or set blank_dynamic:=false"
        )
    data = grid.data.copy()
    data[mask] = -1
    state_dir = os.environ.get("AMR_STATE_DIR", os.path.expanduser("~/.amr"))
    os.makedirs(state_dir, exist_ok=True)
    _pgm, yml = gridio.write(
        gridio.Grid(data, grid.meta), os.path.join(state_dir, f"loc_map_gen{generation}")
    )
    return yml


def _compose(context):
    from amr_mission import map_bundle as mb  # noqa: PLC0415 - launch-time import
    from amr_navigation import footprint as fpmod  # noqa: PLC0415

    cfg = LaunchConfiguration
    maps_dir = os.path.expanduser(cfg("maps_dir").perform(context))
    map_id = cfg("map_id").perform(context)
    revision = cfg("revision").perform(context)
    if revision in ("", "latest"):
        revs = mb.list_revisions(maps_dir, map_id)
        if not revs:
            raise RuntimeError(f"no revisions of '{map_id}' under {maps_dir}")
        revision = revs[-1]
    revision = int(revision)
    rev_dir = mb.revision_dir(maps_dir, map_id, revision)
    manifest = mb.verify(rev_dir)  # raises BundleError on any mismatch
    map_yaml = os.path.join(rev_dir, "map.yaml")
    autostart = cfg("autostart").perform(context).lower() == "true"
    generation = int(cfg("generation").perform(context))
    # Generation-private Nav2 outputs (unified plan §4.3 item 4): the mux of the
    # matching generation subscribes here; a replaced layer's controller keeps
    # publishing on ITS own topic and can never look fresh to the new mux.
    cmd_vel = gating.nav_topic("/cmd_vel", generation)
    cmd_vel_rotate = gating.nav_topic("/cmd_vel_rotate", generation)

    loc = get_package_share_directory("amr_localization")
    params = os.path.join(loc, "config", "amcl.yaml")
    nav2 = os.path.join(get_package_share_directory("amr_navigation"), "config", "nav2_params.yaml")
    footprint_yaml = cfg("footprint_yaml").perform(context) or fpmod.default_path()
    footprint = fpmod.load(footprint_yaml).as_costmap_string()
    # The costmap is a separate node (local_costmap/local_costmap) inside the controller
    # process: an inline {"local_costmap.local_costmap.footprint": ...} dict would set a
    # parameter of that literal name on controller_server (review Q20). Node-scoped
    # parameters reach it only through a params file, so write one per launch.
    costmap_params = _costmap_footprint_file(footprint, generation)
    active_map = {
        "active_map_id": map_id,
        "active_map_revision": revision,
        "active_map_sha256": manifest.sha256,
        "generation": generation,
    }

    dynamic_yaml = os.path.join(rev_dir, "dynamic.yaml") if "dynamic.yaml" in manifest.files else ""
    blank = cfg("blank_dynamic").perform(context).lower() == "true" and bool(dynamic_yaml)

    actions = required(
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params, {"yaml_filename": map_yaml}],
        ),
        "map_server",
    )
    lifecycle_loc = {"autostart": autostart}
    amcl_remap = []
    if blank:
        loc_yaml = _localization_map_file(
            map_yaml, dynamic_yaml, generation, int(cfg("loc_min_occupied_cells").perform(context))
        )
        actions += required(
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server_loc",
                output="screen",
                parameters=[params, {"yaml_filename": loc_yaml, "topic_name": "map_loc", "frame_id": "map"}],
            ),
            "map_server_loc",
        )
        amcl_remap = [("map", "map_loc")]
        lifecycle_loc["node_names"] = ["map_server", "map_server_loc", "amcl"]
    actions += required(
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params],
            remappings=amcl_remap,
        ),
        "amcl",
    )
    actions.append(
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_localization",
                    output="screen",
                    parameters=[params, lifecycle_loc],
                )
            ],
        )
    )
    actions += required(
        Node(
            package="amr_localization",
            executable="localization_monitor_node",
            name="localization_monitor",
            output="screen",
            parameters=[{"generation": generation, "dynamic_yaml": dynamic_yaml}],
        ),
        "localization_monitor",
    )
    actions += required(
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2, costmap_params],  # the file overrides nav2_params.yaml's footprint
            remappings=[("cmd_vel", cmd_vel)],
        ),
        "controller_server",
    )
    actions += required(
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2, {"spin.plugin": spin_plugin()}],
            remappings=[("cmd_vel", cmd_vel_rotate)],
        ),
        "behavior_server",
    )
    actions.append(
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[nav2, {"autostart": autostart}],
                )
            ],
        )
    )
    actions += required(
        Node(
            package="amr_mission",
            executable="route_executor_node",
            name="route_executor",
            output="screen",
            parameters=[{"maps_dir": maps_dir, "footprint_yaml": footprint_yaml, **active_map}],
        ),
        "route_executor",
    )
    print(
        f"[navigation_layer] map {map_id} rev{revision} bundle {manifest.sha256[:12]} ({map_yaml})"
        + (f", dynamic areas{' (AMCL blanked)' if blank else ''}" if dynamic_yaml else "")
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("maps_dir", default_value=os.path.expanduser("~/amr_maps")),
            DeclareLaunchArgument("map_id", default_value="sim_factory"),
            DeclareLaunchArgument("revision", default_value="latest"),
            DeclareLaunchArgument("footprint_yaml", default_value=""),
            DeclareLaunchArgument("generation", default_value="0"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "blank_dynamic", default_value=os.environ.get("AMR_LOC_BLANK_DYNAMIC", "false")
            ),
            DeclareLaunchArgument("loc_min_occupied_cells", default_value="500"),
            OpaqueFunction(function=_compose),
        ]
    )
