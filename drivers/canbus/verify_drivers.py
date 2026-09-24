#!/usr/bin/env python3
"""Verify both BLVD-KRD drivers are alive on can0 at 125 kbps with distinct Node-IDs.

Read-only: issues SDO uploads (reads) and NMT nothing. Motors will not move.

Heartbeat (1017h) defaults to 0 on these drivers, so a passive listen is
expected to be silent. Liveness must be proven with SDO reads.
"""
import os
import struct
import sys
import time

import can

CHANNEL = "can0"
ADAPTER_VID, ADAPTER_PID = 0x16D0, 0x117E
ADAPTER_SERIAL = "2087327F5548"  # None to accept any matching CANable2
BITRATE = 125000
EXPECTED = {1: "left driver", 2: "right driver"}
DEVICE_TYPE_EXPECTED = 0x00020192  # opman_can/blvr_canopen.md:2618
SCAN_RANGE = range(1, 17)  # ID-SEL can only produce 1..16

OK, BAD, WARN = "\033[32mOK\033[0m", "\033[31mFAIL\033[0m", "\033[33mWARN\033[0m"


def sdo_read(bus, node, index, sub, timeout=0.4, collision_window=0.01):
    """Expedited SDO upload. Returns (status, payload, note, latency_ms).

    status: True=data, False=SDO abort, None=timeout
    Collects *all* responses in the window so we can detect two devices
    answering on the same Node-ID.

    collision_window is how long to keep listening AFTER the first reply, purely
    to catch a second responder. That check is worth 10 ms during discovery and
    worthless in a control loop, where it is pure dead time on every read - pass
    0 to return as soon as the first reply lands. See canworker._read(fast=True).
    """
    # Drain anything stale so it can't be mistaken for our reply.
    while bus.recv(timeout=0) is not None:
        pass

    bus.send(can.Message(
        arbitration_id=0x600 + node,
        data=[0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0],
        is_extended_id=False,
    ))

    # Once the first reply lands, linger only briefly to catch a second
    # responder - waiting the full timeout every time would make the measured
    # latency meaningless and the whole scan glacial.
    # A reply counts only if it echoes OUR multiplexer (index, sub) in a full
    # 8-byte frame. The drain above does not rule out a late reply to an
    # EARLIER request landing now, and one accepted on node ID + command byte
    # alone would be read as this object's value. Anything else from the
    # node is skipped, not returned.
    mux = bytes([index & 0xFF, (index >> 8) & 0xFF, sub])
    replies, first_at, stale = [], None, 0
    t0 = time.perf_counter()
    deadline = t0 + timeout
    while time.perf_counter() < deadline:
        m = bus.recv(timeout=max(0.0, deadline - time.perf_counter()))
        if m is None:
            break
        if m.arbitration_id != 0x580 + node:
            continue
        data = bytes(m.data)
        if len(data) != 8 or data[1:4] != mux:
            stale += 1
            continue
        replies.append(data)
        if first_at is None:
            first_at = time.perf_counter()
            deadline = min(deadline, first_at + collision_window)

    if not replies:
        return None, None, ("timeout" if not stale
                            else f"timeout ({stale} reply(s) for another object)"), None

    latency_ms = (first_at - t0) * 1000.0
    note = "ok" if len(replies) == 1 else f"{len(replies)} REPLIES - ID COLLISION?"
    cs = replies[0][0]
    if cs == 0x80:
        return False, struct.unpack_from("<I", replies[0], 4)[0], note, latency_ms
    if cs & 0xE0 == 0x40:
        size = 4 - ((cs >> 2) & 0x03) if cs & 0x02 else 4
        return True, replies[0][4:4 + size], note, latency_ms
    return False, None, f"unexpected SCS 0x{cs:02X}", latency_ms


def u32(b):
    return struct.unpack("<I", b.ljust(4, b"\0"))[0]


def socketcan_ready(iface):
    """True if the interface exists and IFF_UP is set."""
    try:
        with open(f"/sys/class/net/{iface}/flags") as f:
            return bool(int(f.read().strip(), 16) & 0x1)
    except OSError:
        return False


def find_adapter(serial=None):
    """Locate the CANable by USB VID/PID/serial - immune to ttyACM renumbering.

    serial defaults to this module's ADAPTER_SERIAL. Empty string or None means
    accept any matching CANable2, which is what a bench with one adapter wants.
    """
    serial = ADAPTER_SERIAL if serial is None else (serial or None)
    from serial.tools import list_ports
    return [p for p in list_ports.comports()
            if p.vid == ADAPTER_VID and p.pid == ADAPTER_PID
            and (serial is None or p.serial_number == serial)]


def claim_bus(who):
    """Take the CAN owner lock the controller and the ROS drive node hold.

    Returns the lock - keep a reference for as long as the bus is in use - or
    None after printing who owns can0, in which case the caller must exit
    WITHOUT opening the bus. socketcan admits any number of openers, so this
    lock is the only thing that stops a bench tool's SDO requests or controlword
    writes interleaving with a running controller's.

    Imported lazily so the scripts stay standalone: core/ownerlock.py needs no
    vehicle profile, only its own directory on the path.
    """
    core = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), "core")
    if core not in sys.path:
        sys.path.insert(0, core)
    import ownerlock
    try:
        return ownerlock.acquire("can", who)
    except ownerlock.OwnerBusy as e:
        print(f"{BAD}: {e} - stop it first (e.g. systemctl stop agv_controller "
              f"or amr). Nothing was sent.")
    except OSError as e:
        print(f"{BAD}: cannot take the CAN owner lock ({e}). Nothing was sent.")
    return None


def open_bus(bitrate=None, channel=None, adapter_serial=None):
    """Prefer SocketCAN; fall back to slcan on the auto-discovered port.

    Every argument defaults to this module's own constant, so the scripts here
    stay standalone and never import the vehicle profile. canworker passes
    config.CAN_BITRATE / CAN_CHANNEL / CAN_ADAPTER_SERIAL, so the running system
    has a single source for all three and a second vehicle with a different
    adapter is a profile edit rather than a code edit.

    adapter_serial="" means accept any matching CANable2.
    """
    bitrate = BITRATE if bitrate is None else bitrate
    channel = CHANNEL if channel is None else channel
    if socketcan_ready(channel):
        return can.Bus(interface="socketcan", channel=channel), f"socketcan:{channel}"

    ports = find_adapter(adapter_serial)
    if not ports:
        want = ADAPTER_SERIAL if adapter_serial is None else adapter_serial
        raise RuntimeError(
            f"no CANable ({ADAPTER_VID:#06x}:{ADAPTER_PID:#06x}"
            f"{'/' + want if want else ''}) found on USB, "
            f"and {channel} is not up")
    if len(ports) > 1:
        raise RuntimeError(f"ambiguous: {len(ports)} matching adapters "
                           f"({[p.device for p in ports]}) - set "
                           f"can.adapter_serial in the profile")
    dev = ports[0].device
    return can.Bus(interface="slcan", channel=dev, bitrate=bitrate), f"slcan:{dev}"


def main():
    lock = claim_bus("verify_drivers")
    if lock is None:
        return 2
    try:
        bus, how = open_bus()
    except Exception as e:
        print(f"{BAD}: {e}")
        return 2
    print(f"connected via {how}")

    rc = 0
    try:
        # --- 1. passive listen -------------------------------------------
        print("\n[1] passive listen, 2 s (silence is EXPECTED - heartbeat 1017h defaults to 0)")
        seen, t_end = {}, time.time() + 2.0
        while time.time() < t_end:
            m = bus.recv(timeout=max(0.0, t_end - time.time()))
            if m is None:
                break
            seen[m.arbitration_id] = seen.get(m.arbitration_id, 0) + 1
        if seen:
            for cid, n in sorted(seen.items()):
                print(f"    0x{cid:03X}  x{n}")
        else:
            print("    (silent)")

        # --- 2. node scan -------------------------------------------------
        print(f"\n[2] scanning Node-IDs {SCAN_RANGE.start}..{SCAN_RANGE.stop - 1} via SDO read of 1000h")
        found = []
        for nid in SCAN_RANGE:
            st, val, note, _ = sdo_read(bus, nid, 0x1000, 0, timeout=0.25)
            if st is None:
                continue
            found.append(nid)
            tag = f"0x{u32(val):08X}" if st else f"abort 0x{val:08X}"
            print(f"    node {nid:>3}: responded  device type {tag}   [{note}]")
        if not found:
            print(f"    {BAD}: nothing answered. Bitrate mismatch, wiring, or termination.")
            return 1
        print(f"    found: {found}")

        # --- 3. identity of the expected nodes ---------------------------
        print("\n[3] identity check")
        for nid, label in EXPECTED.items():
            print(f"  node {nid} ({label}):")
            if nid not in found:
                print(f"    {BAD}: no response")
                rc = 1
                continue
            st, val, note, _ = sdo_read(bus, nid, 0x1000, 0)
            if st:
                dt = u32(val)
                good = dt == DEVICE_TYPE_EXPECTED
                print(f"    device type   0x{dt:08X}  "
                      f"{OK if good else WARN} (expected 0x{DEVICE_TYPE_EXPECTED:08X})")
                if not good:
                    rc = 1
            if "COLLISION" in note:
                print(f"    {BAD}: {note}  <-- two devices on this Node-ID")
                rc = 1
            for sub, name in ((1, "vendor ID"), (2, "product code"),
                              (3, "revision"), (4, "serial number")):
                st, val, _, _ = sdo_read(bus, nid, 0x1018, sub)
                if st:
                    print(f"    {name:<13} 0x{u32(val):08X}")
                elif st is False:
                    print(f"    {name:<13} abort 0x{val:08X}")
                else:
                    print(f"    {name:<13} timeout")

        # --- 4. verdict ---------------------------------------------------
        print("\n[4] verdict")
        unexpected = [n for n in found if n not in EXPECTED]
        if sorted(found) == sorted(EXPECTED):
            print(f"    {OK}: exactly nodes {sorted(EXPECTED)} present, no collisions.")
        else:
            if unexpected:
                print(f"    {WARN}: unexpected Node-IDs present: {unexpected}")
            missing = [n for n in EXPECTED if n not in found]
            if missing:
                print(f"    {BAD}: missing {missing}")
                if missing == [2]:
                    print("           -> right driver did not take ID-SEL0. Check the IN0 strap;")
                    print("              if it booted as node 1 both drivers share an address.")
            rc = 1
    finally:
        bus.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
