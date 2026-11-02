"""line_layer.launch.py: the tape follower, and nothing else (dual-product plan, Increment 1).

One node. Increment 1 follows straight tape and stops; there is no mission
store, no RFID station handling and no branch selection yet, so nothing has to
be handed to the layer to start it - which is why `_ready_line` waits only for
the layer's own LineState on the current generation and makes no RPC.

The node is REQUIRED: if the follower exits, the layer is gone, and the
supervisor must read that as a fault rather than leave LINE mode standing with
nothing driving. This matters more here than for a survey helper - the mux
keeps applying the last command only until its freshness window closes, so a
dead follower means a vehicle that coasts to a stop with the layer still
nominally up.

    generation:=N   stamped into LineState so consumers can drop old layers;
                    also selects the generation-private /amr/line_cmd topic
                    the mux subscribes to for this layer

Tape localization (tracked plan 3, P1): with the profile's tape_loc.enabled, the layer
also runs map_server, AMCL and the readiness monitor (amr_bringup.tape_loc) on the map
of the active pointer ($AMR_STATE_DIR/line_graph.json, set from the Teach page). They
are REQUIRED like the follower: AMCL exiting ends the layer (LAYER_EXITED, Recover on
the Status page). No pointer, or a bundle that does not verify: the group is skipped
and the follower is told why - line following itself is never blocked by it.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_base import gating
from amr_bringup import tape_loc
from amr_bringup.launch_helpers import required


def _localization(generation: int) -> tuple[list, dict]:
    """(launch actions, follower parameters) for tape localization."""
    from agv_core import config as vehicle  # noqa: PLC0415 - launch-time import

    from amr_line import tape_graph_store as store  # noqa: PLC0415

    vehicle.load()
    maps_dir = os.path.expanduser(os.environ.get("AMR_MAPS_DIR", "~/amr_maps"))
    state_dir = os.path.expanduser(os.environ.get("AMR_STATE_DIR", "~/.amr"))
    params = {"maps_dir": maps_dir, "state_dir": state_dir, "loc_status": ""}
    if not vehicle.TAPE_LOC_ENABLED:
        params["loc_status"] = "off: tape_loc.enabled is false"
        return [], params
    sel = store.read_active(state_dir)
    if sel is None:
        params["loc_status"] = "off: no map selected (Teach page: map <id> <revision>, then restart LINE)"
        return [], params
    try:
        map_yaml, revision, _manifest = tape_loc.verified_map(maps_dir, sel["map_id"], sel["map_revision"])
    except Exception as e:  # noqa: BLE001 - any bundle problem is a reason, never a dead layer
        params["loc_status"] = f"off: map {sel['map_id']} rev{sel['map_revision']} refused: {e}"
        print(f"[line_layer] tape localization {params['loc_status']}")
        return [], params
    g = sel.get("graph_revision")
    params.update(loc_map_id=sel["map_id"], loc_map_revision=revision,
                  loc_graph_revision=-1 if g is None else int(g))
    what = f"graph g{g}" if g is not None else "teaching, no graph"
    print(f"[line_layer] tape localization on {sel['map_id']} rev{revision} ({what}), "
          f"AMCL {vehicle.TAPE_LOC_AMCL_PARAMS}")
    return tape_loc.group(map_yaml, params_set=vehicle.TAPE_LOC_AMCL_PARAMS, generation=generation,
                          monitor=True), params


def _compose(context):
    generation = int(LaunchConfiguration("generation").perform(context))
    real = LaunchConfiguration("real").perform(context).lower() == "true"
    # The follower publishes on plain /amr/line_cmd; the mux of THIS generation
    # listens on the generation-private topic (unified plan §4.3 item 4), like
    # Nav2's cmd_vel in navigation_layer.launch.py. Without this remap the mux
    # sees "line: no fresh command" for the whole run (found in sim, 2026-09-21).
    line_cmd = gating.nav_topic("/amr/line_cmd", generation)
    loc_actions, loc_params = _localization(generation)
    return loc_actions + required(
        Node(
            package="amr_line",
            executable="line_follow_node",
            name="line_follow",
            output="screen",
            parameters=[
                {
                    "generation": generation,
                    # The sim has no scanner: the field is assumed clear there
                    # and ONLY there. On the vehicle a stale /output_paths is
                    # "unknown" and the layer refuses to arm.
                    "field_source": "scanner" if real else "assume_clear",
                    **loc_params,
                }
            ],
            remappings=[("/amr/line_cmd", line_cmd)],
        ),
        "line_follow",
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("generation", default_value="0"),
            DeclareLaunchArgument("real", default_value="true"),
            OpaqueFunction(function=_compose),
        ]
    )
