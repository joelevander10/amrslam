#!/bin/bash
# systemd entry point for the unified service. The supervisor starts web and
# hardware in IDLE; mapping/navigation are selected later by the operator.
set -eo pipefail

AMR_WORKSPACE="${AMR_WS:-$HOME/agv_can/amr_ws}"
# ROS distro: AMR_ROS_DISTRO if set, else the one installed (Jazzy on Ubuntu 24.04,
# e.g. the AMR QR PC; Humble on Ubuntu 22.04, gvievo-01).
ROS_DISTRO_DIR="/opt/ros/${AMR_ROS_DISTRO:-$(ls /opt/ros 2>/dev/null | grep -E '^(jazzy|humble)$' | sort -r | head -1)}"
[[ -f "$ROS_DISTRO_DIR/setup.bash" ]] || { echo "no ROS 2 under $ROS_DISTRO_DIR" >&2; exit 1; }
# shellcheck disable=SC1091
source "$ROS_DISTRO_DIR/setup.bash"
source "$AMR_WORKSPACE/install/setup.bash"
source "$AMR_WORKSPACE/env/vehicle.sh"

export AMR_STATE_DIR="${AMR_STATE_DIR:-$HOME/.amr}"
export PYTHONUNBUFFERED=1

exec python3 -m amr_bringup.supervisor_node --ros-args \
  -p real:=true \
  -p maps_dir:="${AMR_MAPS_DIR:-$HOME/amr_maps}" \
  -p state_dir:="$AMR_STATE_DIR" \
  -p web:="${AMR_WEB:-true}" \
  -p foxglove:="${AMR_FOXGLOVE:-true}" \
  -p lidar:="${AMR_LIDAR:-true}"
