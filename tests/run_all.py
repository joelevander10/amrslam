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
# agv-01, and the mutation tests start from its JSON). A shell on another vehicle
# exports its own AGV_PROFILE (the AMR QR's ~/.bashrc sets amr-qr-01), which would
# turn 17 of these checks red for no reason. Pin it here, before config is imported;
# test_qr_profile loads amr-qr-01 explicitly where it needs it.
os.environ["AGV_PROFILE"] = "agv-01"

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import helpers  # noqa: E402

MODULES = [
    "test_config",
    "test_canworker",
    "test_invariants",
    "test_health",
    "test_canmon",
    "test_lss",
    "test_imu",
    "test_rpdo",
    "test_rfid",
    "test_dio",
    "test_lidar",
    "test_panel",
    "test_blindrun",
    "test_web",
    "test_layout",
    "test_mls",
    "test_qr_profile",
]

# 869 + 24 (2026-09-19): test_mls added with the restored read_mls.py decoders
# (line-follow plan §1.1). Nothing removed or moved.
# 893 + 30 (2026-09-23): test_qr_profile - the platform key and the AMR QR profile
# (amr-qr-01, platform qr_analog). Nothing removed or moved.
EXPECTED_CHECKS = 923


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
          f"jog {helpers.config.MANUAL_FULL_RPM} r/min "
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
