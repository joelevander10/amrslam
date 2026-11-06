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
    "test_dio",
    "test_lidar",
    "test_panel",
    "test_blindrun",
    "test_gls",
    "test_gls_link",
    "test_glspose",
    "test_slip_events",
    "test_layout",
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
# + 8 (2026-10-27): test_invariants C05, segmented SDO uploads. sdo_read finishes or aborts the
#   transfer instead of returning the length as the value and leaving the MLS's server open.
# - 138 (2026-10-27): tracked removal P3 (RFID driver, MLS track decoder, line-sensor guard).
#   test_rfid (75) and test_mls (58) gone as modules; test_canmon - 5 (the line-sensor list
#   checks, read_mls and DriveLink.write_sensor). Nothing added.
# - 99 (2026-10-27): tracked removal P5 (profile, missions). test_mission (72) gone as a
#   module with agv_core.mission and missions/; test_config 125 -> 98: - 33 (the mls and
#   tape_loc blocks, the partial-autopilot check, the autopilot leftovers and limits) + 6
#   (refusals of a leftover autopilot, rfid, mls or tape_loc section, the tracked key and
#   vehicle.sensor_lookahead_m).
# + 7 (2026-10-30): emergency suite (manuals/emergency-suite-plan.md). test_config + 7 (DI08 is
#   panel.di_estop_ok; refused on a panel button, the arm, a pendant channel, off the module;
#   estop_reset_timeout_s outside 0.5..30 s).
# + 10 (2026-10-30): route wait step + PB Continue (manuals/gls6-qr/gls-wait-coding-plan.md
#   §A.2). test_config + 5 (DI13 is panel.di_continue; refused on Start, on the E-stop input,
#   on a pendant channel, off the module); test_panel + 5 (test_continue: one edge per
#   press, not a Start, no edge at boot, none across a reconnect, none without the channel).
# - 38 (2026-10-30): the commissioning page, agv_core/blindrun.py and profile position
#   removed (plan noble-waddling-sutton). test_blindrun (32) deleted; test_config - 6
#   (blind_run.max_rpm/max_speed_mps and the six pp lock checks replaced by two refusals
#   of the retired sections; the rejected-profile check now mutates timing instead).
# + 28 (2026-10-30): test_gls added (gls-wait-coding-plan P0): CoLa A framing (5), parsing
#   incl. two results in one telegram as seen on the vehicle (5), frame aggregation (11),
#   counts -> T_LC conversion (3), the read-only probe and its "gls" owner lock (4).
# + 75 (2026-10-30): P2 of gls-wait-coding-plan (§B.1/§B.3), the GLS621 link and its profile
#   section. test_gls_link + 45 (new: value_matches and the string length prefix, connect
#   readback and frame delivery incl. split / joined telegrams, mismatch and verify timeout
#   drop every frame, keepalive cadence and comms_ok, reconnect / peer close / socket error,
#   connect backoff doubling to the maximum, counters and the 64-frame queue, stop() ends the
#   thread, only sRN ever sent); test_config + 30 (the gls block: enabled, scale 0 = not
#   measured, template has [$XMCL]/[$YMCL]; 26 refusals, one per rule, and its bounds accepted).
# + 32 (2026-10-30): P3 of gls-wait-coding-plan (§C), label poses bound to the map. test_glspose
#   + 21 (new: wrap / compose / inverse round trips at six yaws, label_in_map <-> base_in_map, a
#   hand-worked T_ML with a turned mount, circular mean and spreads across +-pi, every observe()
#   refusal and only the last n frames counting); test_config + 11 (gls.fix_frames,
#   fix_spread_xy_m, fix_spread_yaw_rad, stationary_s: shipped values, 8 bound refusals, a
#   fractional fix_frames refused, the bounds accepted).
# + 4 (2026-10-30): gls.swap_xy (the device's Y is the vehicle's fore-aft, bench 2026-10-30).
#   test_gls + 2 (to_camera_pose swaps X/Y, the signs apply after the swap); test_config + 2
#   (ships true, a non-boolean refused).
# + 3 (2026-11-02): test_panel, the DI08 / DI09 input filters (estop_low_scans below 1 and
#   above 0.2 s refused, manual_arm_scans below 1 refused).
# + 33 (2026-11-02): the blind run is back (PV only; pp stays removed). test_blindrun (32)
#   restored as it was; test_config + 1 (the retired-section refusal for blind_run gives way
#   to the max_rpm and max_speed_mps refusals).
# + 19 (2026-11-03): destination buttons (panel.destinations: DI10 UBC, DI11 ITG, DI12 PISTON).
#   test_config + 11 (shipped channels; refused on Continue, on another destination, on a pendant
#   channel, off the module, a duplicate or empty name, a route id with a slash, an unknown key,
#   the key missing); test_panel + 8 (test_destinations: one edge per press, none at boot or
#   across a reconnect, debounce, two at once or a second while one is held select nothing).
# + 40 (2026-11-03): test_slip_events (towing-load Phase 1, agv_core/slip_events.py): clean
#   runs quiet (4), snatch (9), collapse, snatch -> slip and regrip (7), wheel vs gyro yaw (8),
#   the localization consequence and baseline (7), merging and top-N (5).
# + 31 (AMR QR port, 2026-11-04): test_qr_profile - the platform key and the AMR QR
#   profile (qr_base section, platform refusals, each qr_base bound).
# + 1 (2026-11-05): test_qr_profile, qr_base.wheel_scale_* bounds (the straight-line trim).
EXPECTED_CHECKS = 764


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
