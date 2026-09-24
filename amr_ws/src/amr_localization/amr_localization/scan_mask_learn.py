"""scan_mask_learn: find the scanner's phantom beams (scratched window) and save a mask.

    ros2 run amr_localization scan_mask_learn            # 10 s, clear 1.0 m, print only
    ros2 run amr_localization scan_mask_learn --write    # ... and save ~/.amr/scan_mask.yaml

Park the robot in OPEN SPACE: nothing - no wall, person, pallet - within --clear metres
of the scanner in its whole field of view. Everything the scanner then reports inside
that distance is a phantom. Restart the stack after --write (scan_gate loads the mask at
start). Learn again after replacing the optics cover; delete the file to drop the mask.
"""

from __future__ import annotations

import argparse
import datetime
import os
import sys
import time

from amr_localization.scan_mask import learn, to_yaml_dict

DEFAULT_PATH = os.path.join(os.environ.get("AMR_STATE_DIR", os.path.expanduser("~/.amr")), "scan_mask.yaml")


def main() -> None:  # pragma: no cover - needs ROS and a scanner
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--clear", type=float, default=1.0, help="metres around the scanner known to be empty")
    ap.add_argument("--fraction", type=float, default=0.2, help="share of scans a phantom beam must fire in")
    ap.add_argument("--write", action="store_true", help=f"save the mask to {DEFAULT_PATH}")
    ap.add_argument("--file", default=DEFAULT_PATH)
    args = ap.parse_args()

    import rclpy  # noqa: PLC0415
    from rclpy.qos import qos_profile_sensor_data  # noqa: PLC0415
    from sensor_msgs.msg import LaserScan  # noqa: PLC0415

    rclpy.init()
    node = rclpy.create_node("scan_mask_learn")
    got: list = []
    node.create_subscription(LaserScan, "/scan", got.append, qos_profile_sensor_data)
    print(f"recording /scan for {args.seconds:g} s - nothing real may be within {args.clear:g} m ...")
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    if not got:
        sys.exit("no /scan received - is the scanner running?")
    m = got[0]
    sectors = learn(
        [list(s.ranges) for s in got], m.angle_min, m.angle_increment, m.range_min, args.clear, args.fraction
    )
    print(f"{len(got)} scans, {len(m.ranges)} beams, {len(sectors)} phantom sector(s)")
    print("(0 deg = straight ahead, + = left)")
    for s in sectors:
        print(
            f"  {s.from_deg:+7.2f} .. {s.to_deg:+7.2f} deg   drop readings closer than {s.max_range_m:.2f} m"
        )
    if len(sectors) > 8:
        print(
            "MANY sectors: something real was probably within --clear. Move to open space or lower --clear."
        )
    if not args.write:
        print("dry run: add --write to save")
        return
    import yaml  # noqa: PLC0415

    os.makedirs(os.path.dirname(args.file), exist_ok=True)
    note = f"learnt {datetime.datetime.now():%Y-%m-%d %H:%M}, clear {args.clear:g} m, {len(got)} scans"
    with open(args.file, "w") as f:
        yaml.safe_dump(to_yaml_dict(sectors, note), f, sort_keys=False)
    print(f"saved {args.file} - restart the stack to apply")


if __name__ == "__main__":
    main()
