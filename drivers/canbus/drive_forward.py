#!/usr/bin/env python3
"""Drive both BLVD-KRD motors forward in Profile Velocity mode.

*** THIS MOVES HARDWARE. *** Refuses to run without --go.

Speed, ramp rate and duration are the TARGET_RPM / ACCEL_RPM_S / RUN_SECONDS
constants below; run without --go to have the actual values printed back at you
rather than trusting a number written in this docstring.

Object references (BLV-R CANopen edition, opman_can/blvr_canopen.md):
  6040h Controlword          6041h Statusword        :1412 / :1428
  6060h Modes of operation   3 = pv                  :4556
  6083h Profile acceleration (r/min)/s               :1634
  6084h Profile deceleration (r/min)/s               :1635
  60FFh Target velocity      r/min, signed           :1637
  606Ch Velocity actual value r/min, ro              :1619

Sequence per CiA 402: NMT Start -> Shutdown(06) -> Switch On(07) ->
Enable Operation(0F) with target 0, then ramp by writing 60FFh.
"""
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can  # noqa: E402
from bus_health import decode_state  # noqa: E402
from verify_drivers import (BAD, OK, WARN, claim_bus, open_bus,  # noqa: E402
                            sdo_read, u32)

NODES = {1: "left", 2: "right"}
TARGET_RPM = 160        # 60FFh, signed: positive = forward
ACCEL_RPM_S = 400       # 6083h
DECEL_RPM_S = 400       # 6084h
RUN_SECONDS = 2.2
MAX_RPM_SANITY = 4000    # refuse obviously wrong setpoints

# Controlword bit 13 (PVCM): 0 = motion extension, 1 = normal (opman_can:1667).
# MOTION EXTENSION: this vehicle's motor is a BLMR6400SKM-GFV-B, 400 W on a 1:30
# gearhead, and the Function Edition says that combination must use motion
# extension - in normal mode a hard decel while demand and actual velocity differ
# can damage the motor (opman_fun:2951, 3121). It is also what the RPDO stream
# already sends (rpdo.CW_OPERATION_ENABLED = 0x000F), so the arm sequence and
# every later setpoint frame now agree.
PVCM_MOTION_EXTENSION = 0
CW_ENABLE = 0x000F | (PVCM_MOTION_EXTENSION << 13)   # bit12=0 => continuous velocity
CW_SWITCH_ON = 0x0007
CW_SHUTDOWN = 0x0006
CW_DISABLE_VOLTAGE = 0x0000
CW_QUICK_STOP = 0x0002

SW_FAULT = 1 << 3
SW_SPEED_IS_ZERO = 1 << 12
SW_REMOTE = 1 << 9


def sdo_write(bus, node, index, sub, value, size, timeout=0.5):
    """Expedited SDO download. Returns (ok, detail)."""
    cs = {1: 0x2F, 2: 0x2B, 3: 0x27, 4: 0x23}[size]
    payload = struct.pack("<i" if value < 0 else "<I", value)[:size]
    while bus.recv(timeout=0) is not None:
        pass
    bus.send(can.Message(
        arbitration_id=0x600 + node,
        data=bytes([cs, index & 0xFF, (index >> 8) & 0xFF, sub]) + payload.ljust(4, b"\0"),
        is_extended_id=False))

    # Same rule as sdo_read: only a full frame echoing our (index, sub) is
    # this transaction's answer. A late reply to an earlier request, or a
    # short frame, is skipped rather than reported as the result.
    mux = bytes([index & 0xFF, (index >> 8) & 0xFF, sub])
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if m is None:
            break
        if m.arbitration_id != 0x580 + node:
            continue
        data = bytes(m.data)
        if len(data) != 8 or data[1:4] != mux:
            continue
        if data[0] == 0x60:
            return True, "ok"
        if data[0] == 0x80:
            return False, f"abort 0x{struct.unpack_from('<I', data, 4)[0]:08X}"
        return False, f"unexpected 0x{data[0]:02X}"
    return False, "timeout"


def w(bus, node, index, sub, value, size, what):
    ok, detail = sdo_write(bus, node, index, sub, value, size)
    if not ok:
        raise RuntimeError(f"node {node}: writing {what} ({index:04X}h) failed: {detail}")
    return ok


def statusword(bus, node):
    st, val, _, _ = sdo_read(bus, node, 0x6041, 0)
    return u32(val) & 0xFFFF if st else None


def actual_rpm(bus, node):
    st, val, _, _ = sdo_read(bus, node, 0x606C, 0)
    if not st:
        return None
    return struct.unpack("<i", val.ljust(4, b"\0"))[0]


def nmt(bus, command, node=0):
    """0x01 start, 0x02 stop, 0x80 pre-operational. node 0 = broadcast."""
    bus.send(can.Message(arbitration_id=0x000, data=[command, node],
                         is_extended_id=False))
    time.sleep(0.05)


def coast_to_stop(bus, quiet=False):
    """Best-effort: zero the setpoint, let the ramp run, then de-energise.

    Must never raise - this runs from the finally block.
    """
    for nid in NODES:
        try:
            sdo_write(bus, nid, 0x60FF, 0, 0, 4)
        except Exception:
            pass
    deadline = time.time() + 3.0
    while time.time() < deadline:
        try:
            if all((statusword(bus, n) or 0) & SW_SPEED_IS_ZERO for n in NODES):
                break
        except Exception:
            break
        time.sleep(0.05)
    for nid in NODES:
        for cw in (CW_SHUTDOWN, CW_DISABLE_VOLTAGE):
            try:
                sdo_write(bus, nid, 0x6040, 0, cw, 2)
            except Exception:
                pass
    if not quiet:
        print("    motors de-energised.")


def preflight(bus):
    print("[1] preflight")
    ok = True
    for nid, label in NODES.items():
        st, val, note, _ = sdo_read(bus, nid, 0x1000, 0)
        if st is None:
            print(f"    node {nid} ({label}): {BAD} not responding")
            ok = False
            continue
        if "COLLISION" in note:
            print(f"    node {nid} ({label}): {BAD} {note}")
            ok = False
            continue
        err = sdo_read(bus, nid, 0x1001, 0)[1]
        err = u32(err) if err else None
        sw = statusword(bus, nid)
        print(f"    node {nid} ({label}): error reg 0x{err:02X}, "
              f"statusword 0x{sw:04X} ({decode_state(sw)})")
        if err:
            print(f"      {BAD} driver reports a fault - clear it before driving")
            ok = False
        if not sw & SW_REMOTE:
            print(f"      {BAD} Remote bit clear - controlword will be ignored "
                  f"(S-ON active, or MEXE02 has the driver)")
            ok = False
        if sw & SW_FAULT:
            print(f"      {BAD} FAULT state")
            ok = False
    return ok


def arm(bus):
    print("\n[2] arming (target velocity held at 0)")
    nmt(bus, 0x01)  # broadcast Start Remote Node -> NMT Operational
    print("    NMT: all nodes -> Operational")
    for nid, label in NODES.items():
        w(bus, nid, 0x6060, 0, 3, 1, "modes of operation = pv")
        w(bus, nid, 0x6083, 0, ACCEL_RPM_S, 4, "profile acceleration")
        w(bus, nid, 0x6084, 0, DECEL_RPM_S, 4, "profile deceleration")
        w(bus, nid, 0x60FF, 0, 0, 4, "target velocity = 0")
        for cw, name in ((CW_SHUTDOWN, "Shutdown"),
                         (CW_SWITCH_ON, "Switch On"),
                         (CW_ENABLE, "Enable Operation")):
            w(bus, nid, 0x6040, 0, cw, 2, name)
            time.sleep(0.05)
        sw = statusword(bus, nid)
        state = decode_state(sw)
        flag = OK if (sw & 0x6F) == 0x27 else BAD
        print(f"    node {nid} ({label}): 0x{sw:04X} ({state}) {flag}")
        if (sw & 0x6F) != 0x27:
            raise RuntimeError(f"node {nid} did not reach Operation enabled")


def run(bus):
    print(f"\n[3] ramping to {TARGET_RPM} r/min at {ACCEL_RPM_S} (r/min)/s, "
          f"holding {RUN_SECONDS:.0f} s")
    for nid in NODES:
        w(bus, nid, 0x60FF, 0, TARGET_RPM, 4, "target velocity")

    t0 = time.time()
    while (elapsed := time.time() - t0) < RUN_SECONDS:
        row = []
        for nid, label in NODES.items():
            sw, rpm = statusword(bus, nid), actual_rpm(bus, nid)
            if sw is None or rpm is None:
                raise RuntimeError(f"node {nid} stopped responding mid-run")
            if sw & SW_FAULT:
                raise RuntimeError(f"node {nid} entered FAULT (statusword 0x{sw:04X})")
            row.append(f"{label} {rpm:>6} rpm")
        print(f"    t={elapsed:5.1f}s   " + "   ".join(row))
        time.sleep(1.0)
    print(f"    hold complete ({RUN_SECONDS:.0f} s)")


def main():
    if "--go" not in sys.argv:
        print(__doc__)
        print(f"PLAN: nodes {list(NODES)} -> {TARGET_RPM:+} r/min, "
              f"accel {ACCEL_RPM_S} / decel {DECEL_RPM_S} (r/min)/s, "
              f"hold {RUN_SECONDS:.1f} s.")
        print("\nBoth setpoints are POSITIVE, which on THIS AGV is forward travel:")
        print("the wheels are opposite-handed, so equal positive velocities drive")
        print("it straight. See motion.py for the same convention in table")
        print(f"form. {WARN}: re-verify on blocks after swapping a motor or driver -")
        print("if the two wheels turn opposite ways you will get a spin, not travel.")
        print("\nRe-run with --go to actually drive.")
        return 0

    if abs(TARGET_RPM) > MAX_RPM_SANITY:
        print(f"{BAD}: TARGET_RPM {TARGET_RPM} exceeds sanity limit")
        return 2

    lock = claim_bus("drive_forward")
    if lock is None:
        return 2
    try:
        bus, how = open_bus()
    except Exception as e:
        print(f"{BAD}: {e}")
        return 2
    print(f"connected via {how}\n")

    rc = 0
    armed = False
    try:
        if not preflight(bus):
            print(f"\n{BAD}: preflight failed, not driving.")
            return 1
        arm(bus)
        armed = True
        run(bus)
        print(f"\n[4] {OK}: run completed normally")
    except KeyboardInterrupt:
        print(f"\n\n{WARN}: interrupted - stopping motors")
        rc = 1
    except Exception as e:
        print(f"\n{BAD}: {e}")
        rc = 1
    finally:
        if armed:
            print("\n[stop] ramping down")
            coast_to_stop(bus)
        try:
            nmt(bus, 0x80)  # back to Pre-operational
        except Exception:
            pass
        bus.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
