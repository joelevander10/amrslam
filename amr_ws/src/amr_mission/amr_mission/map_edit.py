"""map_edit: derive a new map revision with dynamic areas / cleanup, from the shell.

The Maps -> Edit areas page does the same through the web app; this is for the bench and
for scripted setups (dynamic-mapping plan §"Order of work"). Rectangles are map metres:

    ros2 run amr_mission map_edit --map tool-center-00 --rev 1 \\
        --dynamic-rect 4.0 -1.5 6.5 -0.4 --note "trolley bay by rack C"
    ros2 run amr_mission map_edit --map tool-center-00 --rev 2 --ops edits.json

`--ops` takes a JSON list in the amr_maps.edit format (or an edits.json, whose "ops" are
replayed). Rectangle options are applied after it, in the order given per kind:
--dynamic-rect, --undynamic-rect, --unknown-rect, --free-rect.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from amr_maps import edit
from amr_mission import map_bundle as mb


def build_ops(args: argparse.Namespace) -> list[dict]:
    ops: list[dict] = []
    if args.ops:
        with open(args.ops) as fh:
            doc = json.load(fh)
        ops.extend(doc["ops"] if isinstance(doc, dict) else doc)
    for flag, make in (
        ("dynamic_rect", lambda p: {"op": "dynamic", "polygon": p}),
        ("undynamic_rect", lambda p: {"op": "undynamic", "polygon": p}),
        ("unknown_rect", lambda p: {"op": "paint", "value": "unknown", "polygon": p}),
        ("free_rect", lambda p: {"op": "paint", "value": "free", "polygon": p}),
    ):
        for x0, y0, x1, y1 in getattr(args, flag) or []:
            ops.append(make(edit.rectangle(x0, y0, x1, y1)))
    return ops


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--maps-dir", default=os.environ.get("AMR_MAPS_DIR", "~/amr_maps"))
    ap.add_argument("--map", required=True, help="map id")
    ap.add_argument("--rev", required=True, type=int, help="parent revision (never modified)")
    ap.add_argument("--ops", help="JSON list of edit ops, or an edits.json to replay")
    rect = {"nargs": 4, "type": float, "action": "append", "metavar": ("X0", "Y0", "X1", "Y1")}
    ap.add_argument("--dynamic-rect", help="mark a dynamic area", **rect)
    ap.add_argument("--undynamic-rect", help="clear a dynamic mark", **rect)
    ap.add_argument("--unknown-rect", help="paint unknown (safe erase)", **rect)
    ap.add_argument("--free-rect", help="paint free (operator assertion)", **rect)
    ap.add_argument("--note", default="")
    args = ap.parse_args(argv)
    try:
        path, rev, res = mb.derive_edit(
            os.path.expanduser(args.maps_dir), args.map, args.rev, build_ops(args), args.note
        )
    except (edit.EditError, mb.BundleError, OSError, ValueError, KeyError) as e:
        print(f"map_edit: refused: {e}", file=sys.stderr)
        return 1
    print(
        f"{args.map} rev{rev} from rev{args.rev}: {len(res.cells)} edits, {res.dynamic_cells} dynamic cells"
    )
    print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
