#!/usr/bin/env python3
"""CiA 305 Layer Setting Services: read and change a node's bitrate on the wire.

*** THIS CAN MAKE THE BUS UNREACHABLE. *** Every write refuses to run without
--go. Reads (`scan`, `identify`) are safe, need no mode change, and leave every
node exactly as they found it.

WHY THIS EXISTS
---------------
The SLAM plan's T0 is a 125 kbps -> 1 Mbps migration, and the three devices on
can0 change bitrate by three DIFFERENT mechanisms
(manuals/slam-generalized-plan/hardware-reconciliation.md section 5):

    BLV-R x2    MEXE02 support software only - no DIP switch, and NOT LSS
    SICK MLS    LSS (CiA 305), or SICK's own configurator
    PC (can0)   ip link / the vehicle profile

Only the middle one is scriptable, and it is the one that has to move first: the
drives are configured over the same bus MEXE02 talks on, so they go last. This
script is the MLS half, written so the step is repeatable and auditable rather
than a one-off click in a Windows configurator.

*** THE HALF-COMPLETED MIGRATION IS THE HAZARD. *** A bus whose nodes disagree
about bitrate is a bus where nothing talks to anything, and the recovery for a
device with no DIP switch is a bench with a second adapter. So the order is
fixed and this script enforces what it can:

    1. drives POWERED DOWN. Verify the MLS alone at the new rate.
    2. then the drives, by MEXE02, at the new rate.
    3. then the PC (can.bitrate in the profile, and `ip link`).

Between 1 and 2 the PC still has to reach the MLS, so the PC moves twice: once
to meet the sensor after step 1, and once for real. `--verify` exists for that
check and reopens the bus itself.

PROTOCOL, AND WHAT IS ASSUMED VERSUS CONFIRMED
----------------------------------------------
LSS is two fixed COB-IDs and a one-byte command specifier - there is no node id,
which is the whole point: it addresses a node that may not have a usable one yet.

    0x7E5   master -> slave
    0x7E4   slave  -> master

*** CONFIRMED here: nothing. *** The framing below is CiA 305 as specified, and
CiA 305 is a standard rather than a guess, but no byte of it has been exercised
against THIS sensor. The reconciliation document lists MLS LSS support as
reported, not verified. So:

  * `scan` and `identify` are the first things to run, and they are read-only.
    If the sensor answers them, it speaks LSS and the rest of this file is
    worth trusting. If it does not, stop and use SICK's configurator - do not
    start guessing at command specifiers with a 150 kg vehicle's only line
    sensor.
  * every write path prints the frames it is about to send before sending them.

SWITCH MODE GLOBAL ADDRESSES EVERY NODE ON THE BUS
--------------------------------------------------
`switch_mode_global(CONFIGURATION)` puts EVERY LSS-capable node into
configuration mode at once, and a subsequent bitrate write then lands on all of
them. That is occasionally what you want and is never what you want by
accident, which is why the default here is the SELECTIVE form: address one node
by its identity (vendor, product, revision, serial), read out of it first with
`identify`. The global form is available behind an explicit --global flag and
says so loudly.

Deliberately dependency-free of `config`, like the rest of canbus/ - every
vehicle-specific value is a parameter defaulting to a module constant, the way
open_bus() already does it. That is what keeps this runnable from a laptop on a
bench, which is exactly where it will be used.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can  # noqa: E402
from verify_drivers import BAD, OK, WARN, claim_bus, open_bus  # noqa: E402

# CiA 305, table 1. The master transmits on 0x7E5 and listens on 0x7E4.
LSS_RX = 0x7E5          # master -> slave
LSS_TX = 0x7E4          # slave  -> master

# Command specifiers. Only the ones this script uses are named; an unnamed cs is
# deliberately not reachable from here.
CS_SWITCH_GLOBAL = 0x04
CS_SWITCH_SEL_VENDOR = 0x40
CS_SWITCH_SEL_PRODUCT = 0x41
CS_SWITCH_SEL_REVISION = 0x42
CS_SWITCH_SEL_SERIAL = 0x43
CS_SWITCH_SEL_ACK = 0x44
CS_CONFIGURE_BIT_TIMING = 0x13
CS_ACTIVATE_BIT_TIMING = 0x15
CS_STORE_CONFIGURATION = 0x17
CS_INQUIRE_VENDOR = 0x5A
CS_INQUIRE_PRODUCT = 0x5B
CS_INQUIRE_REVISION = 0x5C
CS_INQUIRE_SERIAL = 0x5D

MODE_WAITING = 0
MODE_CONFIGURATION = 1

# CiA 301 bit timing table 0. The INDEX is what goes on the wire, not the rate -
# writing "1000000" would be a plausible-looking way to select 800 kbit/s.
BIT_TIMING_TABLE = {
    1000000: 0,
    800000: 1,
    500000: 2,
    250000: 3,
    125000: 4,
    # index 5 is reserved in CiA 301; 100 kbit/s is listed by some vendors and
    # is deliberately NOT offered here rather than being assumed.
    50000: 6,
    20000: 7,
    10000: 8,
}

# Defaults, as module constants so this file never imports the vehicle profile.
BITRATE = 125000
SENSOR_NODE = 10
# How long to wait for an LSS reply. LSS is answered by firmware, not by an
# application task, so this is generous rather than tuned.
REPLY_TIMEOUT_S = 0.5
# CiA 305 switch delay: each node waits this long, twice, around the change, so
# every node moves at about the same moment. Sent as a u16 of milliseconds.
SWITCH_DELAY_MS = 50


def _frame(cs, payload=b""):
    """One LSS request. Always 8 bytes - CiA 305 pads with zeros."""
    data = bytes([cs]) + bytes(payload)
    return can.Message(arbitration_id=LSS_RX,
                       data=data.ljust(8, b"\0")[:8],
                       is_extended_id=False)


def _u32(value):
    return int(value).to_bytes(4, "little")


def _send(bus, cs, payload=b"", echo=False):
    msg = _frame(cs, payload)
    if echo:
        print(f"      -> {msg.arbitration_id:03X}  "
              + " ".join(f"{b:02X}" for b in msg.data))
    bus.send(msg)


def _recv(bus, want_cs=None, timeout=REPLY_TIMEOUT_S):
    """The next LSS reply, or None. Anything that is not on 0x7E4 is ignored.

    The bus carries the MLS stream and drive traffic while this runs, so
    filtering by arbitration id is not an optimisation - it is the difference
    between reading a reply and reading a sensor frame.
    """
    end = time.perf_counter() + timeout
    while True:
        left = end - time.perf_counter()
        if left <= 0:
            return None
        m = bus.recv(timeout=left)
        if m is None or m.arbitration_id != LSS_TX:
            continue
        if want_cs is not None and (not m.data or m.data[0] != want_cs):
            continue
        return bytes(m.data)


# ---- read-only ------------------------------------------------------------

def inquire(bus, echo=False):
    """The node's LSS address, read one field at a time. Changes nothing.

    Only meaningful with exactly ONE node in configuration mode, or with one
    node on the bus - the inquire commands are broadcast and several answers
    would collide. That is why `identify` powers the drives down rather than
    trying to be clever about it.

    Returns a dict of the four fields, with None for any that went unanswered.
    """
    out = {}
    for cs, name in ((CS_INQUIRE_VENDOR, "vendor_id"),
                     (CS_INQUIRE_PRODUCT, "product_code"),
                     (CS_INQUIRE_REVISION, "revision"),
                     (CS_INQUIRE_SERIAL, "serial")):
        _send(bus, cs, echo=echo)
        reply = _recv(bus, want_cs=cs)
        out[name] = (int.from_bytes(reply[1:5], "little")
                     if reply is not None else None)
    return out


def listen(bus, seconds=2.0):
    """Any LSS traffic already on the bus. Purely diagnostic.

    A node answering LSS unprompted would mean somebody else is configuring it,
    which is worth knowing before joining in.
    """
    seen, end = [], time.perf_counter() + seconds
    while time.perf_counter() < end:
        m = bus.recv(timeout=max(0.0, end - time.perf_counter()))
        if m is not None and m.arbitration_id in (LSS_RX, LSS_TX):
            seen.append((m.arbitration_id, bytes(m.data)))
    return seen


# ---- mode -----------------------------------------------------------------

def switch_mode_global(bus, mode, echo=False):
    """Put EVERY LSS-capable node on the bus into `mode`. Unacknowledged.

    There is no reply to this by design, so it cannot be verified directly -
    which is exactly why the selective form below is the default.
    """
    _send(bus, CS_SWITCH_GLOBAL, bytes([mode]), echo=echo)
    time.sleep(0.05)


def switch_mode_selective(bus, address, echo=False):
    """Put ONE node - the one matching `address` - into configuration mode.

    address is the four-field dict inquire() returns. Returns True if the node
    acknowledged, which is the only positive confirmation LSS offers that the
    thing about to be reconfigured is the thing intended.
    """
    for cs, key in ((CS_SWITCH_SEL_VENDOR, "vendor_id"),
                    (CS_SWITCH_SEL_PRODUCT, "product_code"),
                    (CS_SWITCH_SEL_REVISION, "revision"),
                    (CS_SWITCH_SEL_SERIAL, "serial")):
        value = address.get(key)
        if value is None:
            raise ValueError(f"selective switch needs {key}; run `identify` first")
        _send(bus, cs, _u32(value), echo=echo)
    return _recv(bus, want_cs=CS_SWITCH_SEL_ACK) is not None


# ---- writes ---------------------------------------------------------------

def configure_bit_timing(bus, bitrate, echo=False):
    """Stage a new bitrate on a node already in configuration mode.

    Staged, NOT applied: nothing changes on the wire until activate_bit_timing()
    and, for it to survive a power cycle, store_configuration(). Those are three
    separate commands in CiA 305 and they are kept separate here, because the
    window between them is the only chance to abort.

    Returns (ok, error_code). error_code 1 means the node does not support the
    requested rate, 2 means it rejected the request for a reason of its own.
    """
    if bitrate not in BIT_TIMING_TABLE:
        raise ValueError(
            f"{bitrate} is not in CiA 301 bit timing table 0 "
            f"({sorted(BIT_TIMING_TABLE)})")
    index = BIT_TIMING_TABLE[bitrate]
    _send(bus, CS_CONFIGURE_BIT_TIMING, bytes([0, index]), echo=echo)
    reply = _recv(bus, want_cs=CS_CONFIGURE_BIT_TIMING)
    if reply is None:
        return False, None
    return reply[1] == 0, reply[1]


def activate_bit_timing(bus, delay_ms=SWITCH_DELAY_MS, echo=False):
    """Apply the staged bitrate, after `delay_ms`, on every node in config mode.

    Unacknowledged, and it cannot be otherwise: the node's answer would have to
    come back at a bitrate one side has already left. After this call the PC is
    talking at the old rate to a node that is not, so the bus must be reopened -
    see verify().
    """
    _send(bus, CS_ACTIVATE_BIT_TIMING, int(delay_ms).to_bytes(2, "little"),
          echo=echo)
    # Both halves of the switch delay, plus slack. Returning before the node has
    # finished switching would invite a caller to send into the gap.
    time.sleep(2.0 * delay_ms / 1000.0 + 0.1)


def store_configuration(bus, echo=False):
    """Make the staged configuration survive a power cycle.

    *** Without this the sensor reverts to its old bitrate on the next power
    cycle, and a bus that worked on the bench comes up dead in the vehicle. ***
    Returns (ok, error_code).
    """
    _send(bus, CS_STORE_CONFIGURATION, echo=echo)
    reply = _recv(bus, want_cs=CS_STORE_CONFIGURATION)
    if reply is None:
        return False, None
    return reply[1] == 0, reply[1]


# ---- CLI ------------------------------------------------------------------

def _cmd_scan(args):
    """Read-only: is anything speaking LSS, and what is it?"""
    lock = claim_bus("lss scan")
    if lock is None:
        return 2
    bus, how = open_bus(args.bitrate, args.channel)
    print(f"connected via {how} at {args.bitrate // 1000} kbps\n")
    try:
        print(f"[1] passive listen for LSS traffic, 2 s "
              f"(silence is EXPECTED - LSS is master-driven)")
        for cob, data in listen(bus, 2.0):
            print(f"    {cob:03X}  " + " ".join(f"{b:02X}" for b in data))

        print("\n[2] inquire LSS address (broadcast - ONE node only)")
        addr = inquire(bus, echo=args.verbose)
        answered = [k for k, v in addr.items() if v is not None]
        for key, value in addr.items():
            print(f"    {key:14s} "
                  + (f"0x{value:08X} ({value})" if value is not None
                     else f"{WARN} no reply"))
        if not answered:
            print(f"\n{WARN} nothing answered LSS at {args.bitrate // 1000} kbps.")
            print("      Either no node here speaks LSS, or the bus is not at")
            print("      this bitrate. Try the other rate before concluding.")
            return 1
        if len(answered) < 4:
            print(f"\n{WARN} only {len(answered)} of 4 fields answered - a"
                  f" partial identity cannot address a selective switch.")
            return 1
        print(f"\n{OK} the node speaks LSS. Record this address; `set` needs it.")
        return 0
    finally:
        bus.shutdown()


def _cmd_set(args):
    """Stage, activate and store a new bitrate on one node."""
    if args.to not in BIT_TIMING_TABLE:
        print(f"{BAD}: {args.to} is not in CiA 301 bit timing table 0. "
              f"Choose from {sorted(BIT_TIMING_TABLE)}.")
        return 2
    if not args.go:
        print(f"{WARN}: `set` changes the bitrate of a device that has no DIP")
        print("      switch. Re-run with --go. Nothing has been changed.")
        return 2

    print(f"{WARN} preconditions, and they are not checkable from here:")
    print("      * the control service must be stopped - it owns can0")
    print("      * the BLV-R drives should be POWERED DOWN. They do not speak")
    print("        LSS, so a global switch cannot reach them, but a bus with")
    print("        one node moved and two not is the failure this ordering")
    print("        exists to avoid - see the module docstring.")
    print("      * you will need to reopen the bus at the new rate afterwards")
    print()

    lock = claim_bus("lss set")
    if lock is None:
        return 2
    bus, how = open_bus(args.bitrate, args.channel)
    print(f"connected via {how} at {args.bitrate // 1000} kbps\n")
    try:
        if args.use_global:
            print(f"{WARN} --global: EVERY LSS-capable node on this bus will be")
            print("      switched, not just one.")
            switch_mode_global(bus, MODE_CONFIGURATION, echo=args.verbose)
            print("    switch mode global -> configuration (unacknowledged)")
        else:
            print("[1] read the LSS address")
            addr = inquire(bus, echo=args.verbose)
            if any(v is None for v in addr.values()):
                print(f"{BAD}: incomplete LSS address {addr} - cannot address a")
                print("      selective switch. Run `scan` first.")
                return 1
            print(f"    {addr}")
            print("\n[2] selective switch -> configuration mode")
            if not switch_mode_selective(bus, addr, echo=args.verbose):
                print(f"{BAD}: the node did not acknowledge the selective switch.")
                print("      Nothing has been staged; the bus is unchanged.")
                return 1
            print(f"    {OK} acknowledged")

        print(f"\n[3] stage bit timing -> {args.to // 1000} kbps "
              f"(table 0 index {BIT_TIMING_TABLE[args.to]})")
        ok, err = configure_bit_timing(bus, args.to, echo=args.verbose)
        if not ok:
            print(f"{BAD}: the node refused the rate (error {err}). Nothing has")
            print("      been activated; the bus is still at the old bitrate.")
            return 1
        print(f"    {OK} staged")

        print("\n[4] store, so it survives a power cycle")
        ok, err = store_configuration(bus, echo=args.verbose)
        if not ok:
            print(f"{WARN} store refused (error {err}). The rate can still be")
            print("      activated, but it will revert on the next power cycle.")
            if not args.force:
                print("      Stopping. Re-run with --force to activate anyway.")
                return 1
        else:
            print(f"    {OK} stored")

        print(f"\n[5] activate, after {SWITCH_DELAY_MS} ms")
        print(f"    {WARN} after this the PC is at {args.bitrate // 1000} kbps and")
        print(f"      the node is at {args.to // 1000}. They cannot talk until the")
        print("      PC follows. That is expected, and it is not a failure.")
        activate_bit_timing(bus, echo=args.verbose)
        print(f"    {OK} sent")
    finally:
        bus.shutdown()

    print(f"\nNext: bring can0 up at {args.to} and confirm with")
    print(f"      lss.py verify --bitrate {args.to}")
    return 0


def _cmd_verify(args):
    """Reopen at a given rate and see whether the node is there. Read-only."""
    lock = claim_bus("lss verify")
    if lock is None:
        return 2
    try:
        bus, how = open_bus(args.bitrate, args.channel)
    except Exception as e:
        print(f"{BAD}: {e}")
        return 2
    print(f"connected via {how} at {args.bitrate // 1000} kbps\n")
    try:
        addr = inquire(bus, echo=args.verbose)
        if all(v is None for v in addr.values()):
            print(f"{BAD} nothing answered at {args.bitrate // 1000} kbps.")
            return 1
        print(f"{OK} the node answers at {args.bitrate // 1000} kbps: {addr}")
        return 0
    finally:
        bus.shutdown()


def main(argv=None):
    p = argparse.ArgumentParser(
        description="CiA 305 LSS: read or change a node's bitrate.",
        epilog="Reads are safe. `set` writes and needs --go.")
    p.add_argument("--bitrate", type=int, default=BITRATE,
                   help=f"the rate to OPEN the bus at (default {BITRATE})")
    p.add_argument("--channel", default=None, help="SocketCAN channel")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="echo every frame sent")
    sub = p.add_subparsers(dest="mode", required=True)

    sub.add_parser("scan", help="read-only: does anything speak LSS, and what")
    s = sub.add_parser("set", help="change the bitrate (writes; needs --go)")
    s.add_argument("--to", type=int, required=True,
                   help=f"new bitrate, one of {sorted(BIT_TIMING_TABLE)}")
    s.add_argument("--go", action="store_true", help="actually write")
    s.add_argument("--global", dest="use_global", action="store_true",
                   help="switch EVERY LSS node, not one by identity")
    s.add_argument("--force", action="store_true",
                   help="activate even if the store was refused")
    sub.add_parser("verify", help="read-only: is the node at --bitrate now")

    args = p.parse_args(argv)
    try:
        if args.mode == "scan":
            return _cmd_scan(args)
        if args.mode == "set":
            return _cmd_set(args)
        if args.mode == "verify":
            return _cmd_verify(args)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    except Exception as e:                      # noqa: BLE001 - a bench tool
        print(f"{BAD}: {type(e).__name__}: {e}")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
