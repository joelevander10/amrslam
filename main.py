#!/usr/bin/env python3
"""Entry point for the AGV controller.

    python3 main.py                 # 0.0.0.0:5000, reachable from the LAN
    python3 main.py --port 5001
    AGV_PROFILE=agv-02 python3 main.py

*** THIS MOVES HARDWARE, and it has no authentication. *** Anyone who can reach
the port can drive the AGV. Keep it on a trusted network.

Thin on purpose. Everything real lives in app/server.py; this exists so the
systemd unit's ExecStart names a path at the repo root that never moves again,
however the tree below it is reorganised.
"""
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
for _d in ("", "core", "drivers", os.path.join("drivers", "canbus"), "app"):
    sys.path.insert(0, os.path.join(_ROOT, _d) if _d else _ROOT)

import config  # noqa: E402

# The legacy controller speaks CiA 402 to BLV-R drives. On an AMR QR profile
# (platform qr_analog) can.left_node/right_node are ENCODERS and the motors are
# analog - arming them from here would write controlwords to the wrong devices.
# That vehicle runs the ROS stack only (amr_base qr_base_node).
if config.PLATFORM != config.PLATFORM_BLVR:
    sys.exit(f"main.py is the BLV-R controller; profile {config.PROFILE_NAME} is platform "
             f"{config.PLATFORM} - use the ROS stack (amr.service / base.launch.py)")

from server import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main() or 0)
