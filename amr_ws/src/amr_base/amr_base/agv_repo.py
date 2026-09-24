"""Put the agv_can repository on sys.path so ROS nodes import its modules.

Reconciliation D-8: the ROS layer reuses `config`, `kinematics`, `guard`,
`dio`, `rfid` as libraries, with the same bare-import convention app/server.py
uses (layer directories on sys.path, not packages). Nodes import the repo
modules through this file so import ordering can never break the path setup:

    from amr_base.agv_repo import config, kinematics

The root is found by walking up from this file (which lives inside the repo at
amr_ws/src/amr_base/...) until a directory holds both config.py and profiles/.
AGV_CAN_ROOT overrides the search, for an installed workspace outside the repo.
"""

import os
import sys

_LAYER_DIRS = ("", "core", "drivers", os.path.join("drivers", "canbus"))


def find_root() -> str:
    env = os.environ.get("AGV_CAN_ROOT")
    if env:
        return env
    here = os.path.dirname(os.path.realpath(__file__))
    while True:
        if os.path.isfile(os.path.join(here, "config.py")) and os.path.isdir(os.path.join(here, "profiles")):
            return here
        parent = os.path.dirname(here)
        if parent == here:
            raise RuntimeError(
                f"agv_can repository root not found above {os.path.realpath(__file__)}; set AGV_CAN_ROOT"
            )
        here = parent


def add_to_path() -> str:
    root = find_root()
    for d in _LAYER_DIRS:
        p = os.path.join(root, d) if d else root
        if p not in sys.path:
            sys.path.insert(0, p)
    return root


add_to_path()

import config  # noqa: E402
import kinematics  # noqa: E402

__all__ = ["add_to_path", "find_root", "config", "kinematics"]
