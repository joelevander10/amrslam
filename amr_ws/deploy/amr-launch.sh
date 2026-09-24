#!/bin/bash
# systemd ExecStart wrapper: a unit cannot `source`, and every ROS process on
# the vehicle must come up in the same overlay, domain and DDS scope as an
# interactive shell (amr_ws/README.md "ROS domain / DDS scope"), or custom
# amr_interfaces messages will not decode and nodes will not discover each other.
set -eo pipefail  # no -u: the ROS setup scripts read unset variables
WS="${AMR_WS:-$HOME/agv_can/amr_ws}"
# ROS distro: AMR_ROS_DISTRO if set, else the one installed (Jazzy on Ubuntu 24.04,
# e.g. the AMR QR PC; Humble on Ubuntu 22.04, gvievo-01).
ROS_DISTRO_DIR="/opt/ros/${AMR_ROS_DISTRO:-$(ls /opt/ros 2>/dev/null | grep -E '^(jazzy|humble)$' | sort -r | head -1)}"
[[ -f "$ROS_DISTRO_DIR/setup.bash" ]] || { echo "no ROS 2 under $ROS_DISTRO_DIR" >&2; exit 1; }
# shellcheck disable=SC1091
source "$ROS_DISTRO_DIR/setup.bash"
source "$WS/install/setup.bash"
source "$WS/env/vehicle.sh"
export PYTHONUNBUFFERED=1
exec ros2 launch "$@"
