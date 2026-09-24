"""scanner.launch.py: the nanoScan3 driver and nothing else (unified plan U1).

No URDF here - base.launch.py owns the one robot_state_publisher. The parameter file
is the vehicle's (AGV_PROFILE): nanoscan3.yaml for gvievo-01, nanoscan3.<profile>.yaml
otherwise. For a
standalone scanner + TF session use lidar.launch.py. ROS owns UDP 6060.

host_ip "auto" in the parameter file is replaced here by this PC's address on the route to
sensor_ip (the address the scanner must send its UDP data to).
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
    """{"host_ip": <address>} when the file says host_ip "auto", else {}."""
    with open(params_path) as f:
        p = (yaml.safe_load(f) or {}).get("sick_safetyscanners2_node", {}).get("ros__parameters", {})
    if str(p.get("host_ip", "")).lower() != "auto":
        return {}
    sensor = str(p.get("sensor_ip", ""))
    try:
        host = local_ip_towards(sensor)
    except OSError as e:
        raise RuntimeError(f"host_ip auto: no route to the scanner {sensor} ({e})") from e
    if host.startswith("127.") or host == sensor:
        raise RuntimeError(f"host_ip auto: got {host} for scanner {sensor} - set host_ip by hand")
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
            ),
        ]
    )
