#!/usr/bin/env python3
"""Verify the full AGV bus: both BLVD-KRD drivers and the SICK MLS sensor.

Read-only by default. Motors will not move.

  --nmt-start   also send NMT Start Remote Node (broadcast) so devices leave
                Pre-operational and begin transmitting PDOs. Safe: the drives
                need a controlword to energise, which this never sends.
  --listen N    passive listen seconds (default 5)

Scans the full 1..127 Node-ID range rather than 1..16, so a device that was
previously configured to an unexpected address still shows up.
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can  # noqa: E402
from verify_drivers import (BAD, OK, WARN, claim_bus, open_bus,  # noqa: E402
                            sdo_read, u32)

EXPECTED = {
    1:  ("left driver",  "BLVD-KRD", 0x00020192),
    2:  ("right driver", "BLVD-KRD", 0x00020192),
    10: ("SICK MLS",     "MLSE",     None),   # expected device type unknown
}
SCAN_RANGE = range(1, 128)


def classify(cob):
    """Name a COB-ID by CANopen predefined connection set."""
    base, node = cob & 0x780, cob & 0x7F
    names = {
        0x000: "NMT",      0x080: "SYNC/EMCY", 0x180: "TPDO1", 0x200: "RPDO1",
        0x280: "TPDO2",    0x300: "RPDO2",     0x380: "TPDO3", 0x400: "RPDO3",
        0x480: "TPDO4",    0x500: "RPDO4",     0x580: "SDO tx", 0x600: "SDO rx",
        0x700: "HB/bootup",
    }
    if cob == 0x000:
        return "NMT broadcast", None
    if cob == 0x080:
        return "SYNC", None
    if base in names:
        return names[base], node
    return f"0x{cob:03X}", None


def passive_listen(bus, seconds):
    print(f"\n[1] passive listen, {seconds:.0f} s")
    seen, t_end = {}, time.time() + seconds
    while time.time() < t_end:
        m = bus.recv(timeout=max(0.0, t_end - time.time()))
        if m is None:
            break
        e = seen.setdefault(m.arbitration_id, {"n": 0, "last": None})
        e["n"] += 1
        e["last"] = bytes(m.data)
    if not seen:
        print("    (silent)")
        print("    note: CANopen devices do not send PDOs in Pre-operational,")
        print("          and the drivers have heartbeat disabled (1017h = 0).")
        return seen
    for cob, e in sorted(seen.items()):
        kind, node = classify(cob)
        who = f" node {node}" if node else ""
        rate = e["n"] / seconds
        print(f"    0x{cob:03X}  {kind:<12}{who:<8} x{e['n']:<5} "
              f"({rate:5.1f}/s)  last={e['last'].hex(' ')}")
    return seen


def scan(bus, timeout):
    print(f"\n[2] scanning Node-IDs 1..127 (SDO read of 1000h)")
    found = {}
    for nid in SCAN_RANGE:
        st, val, note, lat = sdo_read(bus, nid, 0x1000, 0, timeout=timeout)
        if st is None:
            continue
        dt = u32(val) if st else None
        found[nid] = (dt, note, lat)
        tag = f"device type 0x{dt:08X}" if st else f"abort 0x{val:08X}"
        star = "" if nid in EXPECTED else f"  {WARN} UNEXPECTED"
        print(f"    node {nid:>3}: {tag}   ({lat:.2f} ms) [{note}]{star}")
    if not found:
        print(f"    {BAD}: no device answered on any address 1..127")
    return found


def identity(bus, found):
    print("\n[3] identity")
    for nid in sorted(found):
        label = EXPECTED.get(nid, ("unknown device", "?", None))[0]
        print(f"  node {nid} ({label}):")
        dt, note, _ = found[nid]
        exp = EXPECTED.get(nid, (None, None, None))[2]
        if dt is not None:
            mark = "" if exp is None else f"  {OK if dt == exp else WARN} (expected 0x{exp:08X})"
            print(f"    device type   0x{dt:08X}{mark}")
        if "COLLISION" in note:
            print(f"    {BAD}: {note}  <-- two devices share this Node-ID")
        for sub, name in ((1, "vendor ID"), (2, "product code"),
                          (3, "revision"), (4, "serial number")):
            st, val, _, _ = sdo_read(bus, nid, 0x1018, sub)
            if st:
                print(f"    {name:<13} 0x{u32(val):08X}")
            elif st is False:
                print(f"    {name:<13} -")          # sub-index absent is normal
        # device name is optional (1008h) but useful for identifying the sensor
        st, val, _, _ = sdo_read(bus, nid, 0x1008, 0)
        if st and val:
            print(f"    device name   {val.decode('ascii', 'replace').strip()}")


def bus_dead_help():
    print(f"\n    {BAD} Nothing on the bus responds.")
    print("""
    Adding a node changes the physical topology, so check that first:

    1. TERMINATION - power everything OFF, measure resistance across
       CAN_H and CAN_L. Expect ~60 ohm (two 120 ohm in parallel).
         ~40 ohm  -> three terminators fitted, remove one
         ~120 ohm -> only one terminator
         ~0  ohm  -> short between CAN_H and CAN_L
       Exactly two, at the two physical ends of the bus, never on a stub.

    2. POLARITY - CAN_H and CAN_L swapped on the new drop kills the whole bus,
       not just the new node. BLVD-KRD CN4: pin 5 = CAN_L, 6 = CAN_H, 7 = CAN_GND.

    3. POWER - confirm the drivers still have 24 V at NET-VIN (CN4 pin 20).
       An overloaded supply browning out when the sensor was added would look
       exactly like this.

    4. BISECT - unplug the sensor and re-run. If nodes 1 and 2 come back, the
       fault is in the sensor drop. If they do not, something else was disturbed.

    Note: on slcan there is no bus error state, so a wiring fault looks
    identical to 'nothing is transmitting'. Silence is not diagnostic here.""")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nmt-start", action="store_true",
                    help="send NMT Start so devices leave Pre-operational")
    ap.add_argument("--listen", type=float, default=5.0)
    ap.add_argument("--scan-timeout", type=float, default=0.1)
    args = ap.parse_args()

    lock = claim_bus("verify_bus")
    if lock is None:
        return 2
    try:
        bus, how = open_bus()
    except Exception as e:
        print(f"{BAD}: {e}")
        return 2
    print(f"connected via {how}")

    try:
        passive_listen(bus, args.listen)
        found = scan(bus, args.scan_timeout)

        if not found:
            bus_dead_help()
            return 1

        identity(bus, found)

        if args.nmt_start:
            print("\n[3b] NMT Start (broadcast) - devices -> Operational")
            bus.send(can.Message(arbitration_id=0x000, data=[0x01, 0x00],
                                 is_extended_id=False))
            time.sleep(0.2)
            passive_listen(bus, args.listen)

        print("\n[4] verdict")
        missing = [n for n in EXPECTED if n not in found]
        extra = [n for n in found if n not in EXPECTED]
        for nid, (label, _, _) in EXPECTED.items():
            mark = OK if nid in found else BAD
            print(f"    node {nid:>3}  {label:<14} {mark}")
        if extra:
            print(f"    {WARN}: unexpected Node-IDs {extra}")
        if not missing and not extra:
            print(f"\n    {OK}: all three nodes present, no collisions.")
            return 0
        if missing:
            print(f"\n    {BAD}: missing {missing}")
            if missing == [10]:
                print("      -> sensor not answering. Check its 24 V (L+), the drop"
                      "\n         wiring, and that it is still at factory 125 kbps.")
        return 1
    finally:
        bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
