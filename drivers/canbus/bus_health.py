#!/usr/bin/env python3
"""Bus integrity + driver diagnostics for the AGV CANopen bus.

Read-only. Motors will not move.

Rationale: slcan gives no visibility of CAN error state (no ERROR-PASSIVE,
no BUS-OFF, no berr-counter), so bad termination shows up only as flaky
transfers. Hammering SDO reads and measuring the failure rate is the
practical substitute for a bus-health readout.
"""
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_drivers import (BAD, OK, WARN, claim_bus, open_bus,  # noqa: E402
                            sdo_read, u32)

NODES = {1: "left", 2: "right"}
SOAK_N = 400

# CiA 402 statusword (6041h) state decode: (mask, value) -> name
CIA402_STATES = [
    (0x4F, 0x00, "Not ready to switch on"),
    (0x4F, 0x40, "Switch on disabled"),
    (0x6F, 0x21, "Ready to switch on"),
    (0x6F, 0x23, "Switched on"),
    (0x6F, 0x27, "Operation enabled"),
    (0x6F, 0x07, "Quick stop active"),
    (0x4F, 0x0F, "Fault reaction active"),
    (0x4F, 0x08, "FAULT"),
]
MODE_BITS = {0: "pp (profile position)", 1: "vl (velocity)", 2: "pv (profile velocity)",
             3: "tq (torque)", 5: "hm (homing)", 6: "ip (interpolated)",
             7: "csp", 8: "csv", 9: "cst"}
MODE_NAMES = {0: "no mode", 1: "pp", 2: "vl", 3: "pv", 4: "tq", 6: "hm", 7: "ip"}


def decode_state(sw):
    for mask, val, name in CIA402_STATES:
        if sw & mask == val:
            return name
    return f"unknown (0x{sw:04X})"


def read(bus, node, idx, sub=0):
    st, val, _, _ = sdo_read(bus, node, idx, sub)
    return u32(val) if st else None


def main():
    lock = claim_bus("bus_health")
    if lock is None:
        return 2
    try:
        bus, how = open_bus()
    except Exception as e:
        print(f"{BAD}: {e}")
        return 2
    print(f"connected via {how}\n")

    rc = 0
    try:
        # ---- diagnostics ------------------------------------------------
        print("[1] driver diagnostics")
        for nid, label in NODES.items():
            print(f"  node {nid} ({label}):")

            err = read(bus, nid, 0x1001)
            if err is None:
                print(f"    error register   {BAD} no response")
                rc = 1
            else:
                flag = OK if err == 0 else BAD
                print(f"    error register   0x{err:02X}  {flag}"
                      + ("" if err == 0 else "  <-- driver is reporting a fault"))
                if err:
                    rc = 1

            sw = read(bus, nid, 0x6041)
            if sw is not None:
                print(f"    statusword       0x{sw & 0xFFFF:04X}  ({decode_state(sw)})")

            mode = read(bus, nid, 0x6061)
            if mode is not None:
                m = mode & 0xFF
                m = m - 256 if m > 127 else m
                print(f"    current mode     {m} ({MODE_NAMES.get(m, '?')})")

            modes = read(bus, nid, 0x6502)
            if modes is not None:
                names = [n for b, n in MODE_BITS.items() if modes & (1 << b)]
                print(f"    supported modes  0x{modes:08X} -> {', '.join(names)}")

            hb = read(bus, nid, 0x1017)
            if hb is not None:
                print(f"    heartbeat 1017h  {hb} ms" + ("  (disabled)" if hb == 0 else ""))

        # ---- soak -------------------------------------------------------
        print(f"\n[2] bus integrity soak: {SOAK_N} SDO reads, alternating nodes")
        lat, fails = [], 0
        t0 = time.time()
        for i in range(SOAK_N):
            nid = 1 if i % 2 == 0 else 2
            st, _, note, lat_ms = sdo_read(bus, nid, 0x1000, 0, timeout=0.3)
            if st is True and "COLLISION" not in note:
                lat.append(lat_ms)
            else:
                fails += 1
        elapsed = time.time() - t0

        ok_n = SOAK_N - fails
        rate = 100.0 * ok_n / SOAK_N
        print(f"    completed        {ok_n}/{SOAK_N}  ({rate:.2f}%) in {elapsed:.1f}s")
        if lat:
            lat.sort()
            print(f"    latency  min     {lat[0]:.2f} ms")
            print(f"             median  {statistics.median(lat):.2f} ms")
            print(f"             p95     {lat[int(len(lat) * 0.95)]:.2f} ms")
            print(f"             max     {lat[-1]:.2f} ms")

        print("\n[3] verdict")
        if fails == 0:
            print(f"    {OK}: {SOAK_N}/{SOAK_N} transfers, zero errors. "
                  f"Termination and wiring look sound.")
        elif rate >= 99.0:
            print(f"    {WARN}: {fails} failure(s). Not clean - suspect termination "
                  f"or a marginal connection.")
            rc = 1
        else:
            print(f"    {BAD}: {fails} failures ({100 - rate:.1f}%). "
                  f"Check termination (exactly two 120 ohm, at the physical ends).")
            rc = 1
    finally:
        bus.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
