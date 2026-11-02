"""scanner.launch.py: the nanoScan3 driver and nothing else (unified plan U1).

No URDF here - base.launch.py owns the one robot_state_publisher. The parameter file
is the vehicle's (AGV_PROFILE): nanoscan3.yaml for gvievo-01, nanoscan3.<profile>.yaml
otherwise. host_ip "auto" in it is replaced here by this PC's address on the route to
sensor_ip (the address the scanner must send its UDP data to). For a
standalone scanner + TF session use lidar.launch.py. ROS owns UDP 6060.

The driver aborts (exit -6) if the scanner is unreachable when it starts - a
cable out at boot left it dead until the next service restart (2026-10-02). It
respawns instead, so plugging the cable back in recovers the data link by itself.
The STOP path (OSSD pair into the FX3) does not depend on this node.
"""

import os
import socket

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def scanner_params() -> str:
    """nanoscan3.yaml for gvievo-01 (agv-01), nanoscan3.<AGV_PROFILE>.yaml for any other vehicle."""
    profile = os.environ.get("AGV_PROFILE") or "agv-01"
    name = "nanoscan3.yaml" if profile == "agv-01" else f"nanoscan3.{profile}.yaml"
    path = os.path.join(get_package_share_directory("amr_bringup"), "config", name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"AGV_PROFILE={profile}: no scanner config {path}")
    return path


def local_ip_towards(ip: str) -> str:
    """This host's source address for packets to ip (no packet is sent: UDP connect only)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((ip, 2122))
        return s.getsockname()[0]


def host_ip_override(params_path: str) -> dict:
    """{"host_ip": <address>} when the file says host_ip "auto", else {}.

    No route to the scanner at all (interface down at boot) is not fatal - the driver
    respawns, as above - so the profile's lidar.host_ip is used instead, with a warning."""
    with open(params_path) as f:
        p = (yaml.safe_load(f) or {}).get("sick_safetyscanners2_node", {}).get("ros__parameters", {})
    if str(p.get("host_ip", "")).lower() != "auto":
        return {}
    sensor = str(p.get("sensor_ip", ""))
    try:
        host = local_ip_towards(sensor)
        if host.startswith("127.") or host == sensor:
            raise OSError(f"got {host}")
    except OSError as e:
        from agv_core import config  # noqa: PLC0415 - the vehicle profile (AGV_PROFILE)

        host = config.LIDAR_HOST_IP
        print(f"[scanner.launch] host_ip auto: no usable route to {sensor} ({e}); using lidar.host_ip {host}")
        return {"host_ip": host}
    print(f"[scanner.launch] host_ip auto -> {host} (scanner {sensor})")
    return {"host_ip": host}


def generate_launch_description() -> LaunchDescription:
    params = scanner_params()
    override = host_ip_override(params)
    return LaunchDescription(
        [
            Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name="sick_safetyscanners2_node",
                output="screen",
                emulate_tty=True,
                parameters=[params, override] if override else [params],
                respawn=True,
                respawn_delay=5.0,
            ),
        ]
    )
