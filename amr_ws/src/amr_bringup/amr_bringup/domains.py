"""ROS domain scope: the vehicle and any simulation must never share a graph.

Spec §9: "Separate simulation/hardware ROS domains and choose DDS interface
scope before the first physical ROS drive launch." The hazard is concrete: a
sim.launch.py on the same domain as a live drive_node puts a fake base and the
real drives on the same /cmd_wheel_vel, /wheel_states and /tf.

    VEHICLE   the one domain drive_node / panel / lidar may run in
    SIM       the default for simulation on this host
    61..68    pinned by the launch tests, one each, so they cannot see each other

Launch files call require_vehicle_domain() / refuse_vehicle_domain() at
description-generation time, so a wrong shell fails before any process starts.
The DDS side is amr_bringup/config/cyclonedds-local.xml (loopback only): nothing
off this host can join either graph; Foxglove goes through its bridge.
"""

from __future__ import annotations

import os

VEHICLE = 10
SIM = 20
TEST_RANGE = range(61, 69)  # 68: test_unified_sim (P7)

ENV_HINT = "source ~/agv_can/amr_ws/env/vehicle.sh (hardware) or env/sim.sh (simulation)"


def current() -> int | None:
    v = os.environ.get("ROS_DOMAIN_ID", "").strip()
    if not v:
        return None
    try:
        return int(v)
    except ValueError as e:
        raise RuntimeError(f"ROS_DOMAIN_ID={v!r} is not an integer") from e


def require_vehicle_domain(what: str) -> int:
    """Hardware launches: exactly VEHICLE, and it must be set explicitly."""
    d = current()
    if d != VEHICLE:
        raise RuntimeError(
            f"{what} runs only in ROS_DOMAIN_ID={VEHICLE} (the vehicle); "
            f"this shell has {d if d is not None else 'nothing set (default 0)'}. {ENV_HINT}"
        )
    return d


def refuse_vehicle_domain(what: str) -> int:
    """Simulation launches: anything but VEHICLE, and not the unset default either."""
    d = current()
    if d is None:
        raise RuntimeError(
            f"{what} refuses to run with ROS_DOMAIN_ID unset: default 0 is shared by every "
            f"unconfigured ROS host on the LAN. {ENV_HINT}"
        )
    if d == VEHICLE:
        raise RuntimeError(
            f"{what} is a simulation and must not run in the vehicle domain {VEHICLE}; "
            f"use ROS_DOMAIN_ID={SIM}. {ENV_HINT}"
        )
    return d
