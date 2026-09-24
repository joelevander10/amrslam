"""Test/launch fixture: publish the sim world itself as a map bundle.

`nav.launch.py sim:=true` needs a saved revision to localise against. Running
the two-minute T4 survey for every test is wasteful, and the T4 survey test
already proves a surveyed bundle matches the world. So the world grid is
written as `<maps_dir>/<map_id>/rev<N>/` through the same staging/verify/
publish path a real save uses; the manifest says so in `review.note`.
"""

from __future__ import annotations

import os
import sys

import numpy as np
from ament_index_python.packages import get_package_share_directory

from amr_maps import grid as gridio
from amr_mission import map_bundle as mb


def default_world_yaml() -> str:
    return os.path.join(get_package_share_directory("amr_maps"), "worlds", "sim_factory", "world.yaml")


def surfaces_only(grid: gridio.Grid) -> gridio.Grid:
    """Occupied interiors -> unknown, as a lidar-built map has them.

    A world file is solid: walls four cells thick, racks filled. A SLAM map only
    ever sees surfaces, and nav2's likelihood field scores a beam endpoint
    anywhere INSIDE an occupied block as a perfect hit, so a solid map biases
    AMCL toward the obstacle ahead by a few centimetres along a corridor.
    Keeping only occupied cells with a non-occupied 8-neighbour removes that.
    """
    occ = grid.data >= 65
    interior = occ.copy()
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            interior &= np.roll(np.roll(occ, dr, axis=0), dc, axis=1)
    data = grid.data.copy()
    data[interior] = -1
    return gridio.Grid(data, grid.meta)


def write_world_as_bundle(maps_dir: str, map_id: str = "sim_factory", world_yaml: str | None = None) -> str:
    """Create the next revision of `map_id` from the world. Returns the revision dir."""
    world = surfaces_only(gridio.read(world_yaml or default_world_yaml()))
    revision = mb.next_revision(maps_dir, map_id)
    os.makedirs(os.path.join(maps_dir, map_id), exist_ok=True)
    stage = mb.staging_dir(maps_dir, map_id, revision)
    manifest = mb.Manifest(
        map_id=map_id,
        revision=revision,
        created=mb.now_iso(),
        frame_id="map",
        resolution=world.meta.resolution,
        origin=[world.meta.origin_x, world.meta.origin_y, world.meta.origin_yaw],
        width=world.width,
        height=world.height,
        start={"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "description": "world origin mark"},
        review={
            "dx_m": 0.0,
            "dy_m": 0.0,
            "dyaw_rad": 0.0,
            "note": "fixture: sim world surfaces written as map",
        },
        software={"source": "amr_mission.fixtures.write_world_as_bundle"},
    )
    mb.stage_bundle(stage, world, None, manifest, required_posegraph=False)
    mb.verify(stage)
    return mb.publish(stage, maps_dir, map_id, revision)


def main(argv: list[str] | None = None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    maps_dir = os.path.expanduser(argv[0] if argv else "~/amr_maps")
    map_id = argv[1] if len(argv) > 1 else "sim_factory"
    print(write_world_as_bundle(maps_dir, map_id))


if __name__ == "__main__":
    main()
