#!/usr/bin/env python3
"""Run the whole offline suite. No CAN, no hardware, no motion.

    python3 tests/run_all.py

Splitting one 1,465-line file into eight had a failure mode worth guarding: a
module that quietly stops being imported costs coverage without producing a
symptom - the run still ends in "all checks passed", just with fewer checks in
it. So this runner pins BOTH the module list and the total number of checks. A
dropped module, or a test that silently stopped asserting, fails the run.

Raise EXPECTED_CHECKS deliberately when checks are added; never lower it to make
a run go green.
"""
import importlib
import os
import pathlib
import sys
import threading

# The suite is written against the gvievo-01 profile ("the real profile loads" means
# agv-01, and the mutation tests start from its JSON). The AMR QR's shell and amr.env
# export AGV_PROFILE=amr-qr-01, which would turn many of these checks red for no reason.
# Pin it here, before config is imported; test_qr_profile loads amr-qr-01 explicitly.
os.environ["AGV_PROFILE"] = "agv-01"

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import helpers  # noqa: E402

MODULES = [
    "test_config",
    "test_invariants",
    "test_canmon",
    "test_lss",
    "test_imu",
    "test_rpdo",
    "test_rfid",
    "test_dio",
    "test_lidar",
    "test_panel",
    "test_blindrun",
    "test_layout",
    "test_mls",
    "test_mission",
    "test_qr_profile",
]

# 869 + 24 (2026-09-19): test_mls added with the restored read_mls.py decoders
# (line-follow plan §1.1). Nothing removed or moved.
# 893 + 5 (2026-09-20): the agv_core package. test_layout stopped pinning the
# flat sys.path namespace (4 checks) and started pinning the package boundary
# instead (9), split across one extra test function: the library is a package,
# it imports neither ROS nor Flask, and no module in it imports a sibling bare.
# 898 - 367 (2026-09-21, U11): the legacy controller is gone. Removed, legacy
# only: test_canworker (70), test_web (129), test_health (32) as modules, and
# the Controller/app/runlog cases inside test_panel (37), test_blindrun (33),
# test_layout (33), test_invariants (21), test_lidar (6, the lidar_scan bench
# tool), test_dio (4, the legacy health monitor), test_rpdo (2). Moved, shared
# library: the guard-bypass checks in test_canmon (3) and the RPDO wiring
# checks in test_rpdo (4) now read amr_base/canopen.py, the bus owner that
# replaced canworker. Nothing added. Per-module counts before/after are in
# manuals/vehicle-reports/2026-09-21-restructure-and-line-layer.md.
# + 8 (2026-10-02): test_mls calibration summary (read_mls calibrate).
# - 3 + 1 (2026-10-02): dead speed params removed (manual.*, motion.py, MANUAL_HALF_RPM);
# one check that the retired manual section is refused.
# + 17 (2026-10-02): test_mission, the tape-mission validator ported from gy-demo.
# + 9 (2026-10-08): mission schema v2, the RFID tag table. test_mission 20 -> 32 (v1 route,
# stop and profile-toggle checks replaced by the tag-table rules and the site file);
# test_config 7 -> 4 (the profile toggle-tag rules became refusals of the retired keys,
# plus the U-turn window).
# + 18 (2026-10-08): tracked-speed-plan-1. test_config + 8 (the slow-zone keys refused,
# HIGH ratio / motor limit / curve guard / high ramp bounds); test_mission + 10 (the
# high_zone block: budget, entry window, refusals; the toggle check became its refusal).
# + 31 (2026-10-09): MLS markers (manuals/mls-marker-plan.md). test_canmon + 4 (guard's
# line-sensor list, read_mls writes only through it); test_mls + 15 (raw marker field,
# set-markers, marker watch, Hall profile, no-track gate, element count, baseline); test_config + 12 (the mls
# block and its refusals).
# Counted on top of the uncommitted RFID work in this tree (671).
# + 3 (2026-10-09): speed zones on MLS markers, RFID stops only. test_mission high zone
#   10 -> 13 (RFID zone row refused, v2.1 block refused, code not laid / text / same code
#   refused, zone with no RFID row; the zone-tag ignore window refusal is gone).
# + 19 (2026-10-17): tracked plan 3, tape-graph localization. test_config + 6 (the tape_loc block:
#   off, station limit within +-10 cm, refusals); test_mission + 13 (site-3 schema: the site-103
#   file, its legs and listing, 8 refusals).
# + 1 (2026-10-18): the straight-section test mission (plan 3 section 7).
# + 14 (2026-10-18): MLS diagnostics (manuals/mls-diagnostics-plan.md M0). test_mls + 11 (event
#   source/cause and temperature decoding, the reset on the guard list and the thresholds off it);
#   test_canmon + 1 (DriveLink.write_sensor goes through check_sensor; the list check now names
#   2080h:01); test_config + 2 (mls.event_clear off, and its refusal).
# + 7 (2026-10-18): the marker run (plan 3 section 7.5): test_mission, the straight's mission and
#   6 refusals.
# + 5 (2026-10-23): the four basic jobs (empty, basic-right, basic-high-left, basic-high-right)
#   and the mission picker's note (test_mission).
# + 30 (AMR QR port, 2026-11-02): test_qr_profile - the platform key and the AMR QR
#   profile (qr_base section, platform refusals, each qr_base bound).
EXPECTED_CHECKS = 782


# An exception on a helper thread only prints a traceback by default - the
# check that started the thread can still pass. Record every one so the run
# fails instead.
THREAD_ERRORS = []


def _thread_excepthook(args):
    THREAD_ERRORS.append(f"{args.thread.name if args.thread else '?'}: "
                         f"{args.exc_type.__name__}: {args.exc_value}")
    _default_excepthook(args)


_default_excepthook = threading.excepthook


def main():
    threading.excepthook = _thread_excepthook
    print(f"profile {helpers.config.PROFILE_NAME}: "
          f"can {helpers.config.CAN_BITRATE // 1000} kbps "
          f"use_rpdo={helpers.config.CAN_USE_RPDO} "
          f"6083h={helpers.config.ACCEL_RPM_S}")

    ran = 0
    for name in MODULES:
        mod = importlib.import_module(name)
        for fn in mod.TESTS:
            fn()
            ran += 1

    total = helpers.CHECKS[0]
    print()
    print(f"{ran} test function(s), {total} check(s)")

    problems = list(helpers.FAIL)
    problems += [f"uncaught exception in thread {e}" for e in THREAD_ERRORS]
    if total != EXPECTED_CHECKS:
        problems.append(
            f"expected {EXPECTED_CHECKS} checks, ran {total} - a module or a "
            f"check went missing (or update EXPECTED_CHECKS deliberately)")

    if problems:
        print(f"{len(problems)} FAILED: " + ", ".join(problems))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
