#!/usr/bin/env python3
"""RPDO1: send the controlword and the setpoint as one frame instead of two SDOs.

*** THIS IS THE LARGEST AVAILABLE WIN ON THE CONTROL TICK. *** 60FFh is written
by a BLOCKING SDO transfer every tick, twice - once per drive - at about 1.8 ms
each. The 50 Hz loop measures 20.4 ms average against a 20 ms budget and peaks
near 31 ms, so ~3.6 ms of every tick is spent waiting for two acknowledgements
that carry no information. An RPDO is a bare CAN frame: it is not acknowledged,
there is nothing to wait for, and the write becomes a queue append.

  manuals/potential-ros-migration.md section 9 ranks this first of everything
  worth doing, above any framework change
  manuals/slam-generalized-plan/hardware-reconciliation.md D-8 calls it a
  prerequisite for the ROS drive node, not an optimisation

WHY THIS IS A SEPARATE MODULE AND NOT A METHOD ON THE CONTROLLER
----------------------------------------------------------------
The ROS drive node needs exactly this and nothing else around it. Keeping the
packing and the configuration sequence here - standalone, no config import, no
controller state - is what lets that node import it rather than reimplement it,
which is the shape D-8 settles on for the whole migration.

THE SAFETY ARGUMENT, WHICH IS NOT THE OBVIOUS ONE
-------------------------------------------------
An SDO write is acknowledged and an RPDO is not, so the naive reading is that
this trades safety for speed. It does not, and it is worth being precise about
why:

  * *** Nothing was checking the acknowledgement anyway *** on the per-tick
    path. _write_target() calls _write(), which raises on a failed transfer -
    but a failed setpoint write on one tick is corrected by the next tick 20 ms
    later, and the vehicle has never depended on that acknowledgement.
  * The thing that actually stops the vehicle when a drive goes quiet is
    core/health.py's table, fed by the producer heartbeat (1017h) and by
    telemetry replies. Both survive this change untouched. A drive that stops
    receiving RPDOs is detected by exactly the same mechanism as before.
  * The drive's own CANopen heartbeat CONSUMER is what stops it if the PC dies.
    That is a drive-side setting, unaffected either way.
  * A late tick, on the other hand, IS a steering update the vehicle does not
    get. Removing 3.6 ms from a 20 ms budget that is already overrunning is a
    safety improvement, not a cost.

The one property genuinely lost is "this specific frame arrived". It was never
used.

MAPPING AN RPDO IS A PRIVILEGED ACT
-----------------------------------
The mapping written below is checked by drivers/canbus/guard.py, and that is not
ceremony: an RPDO mapping is a write path, so mapping 403Eh into one would let a
plain CAN frame release the holding brake on both drive wheels with no SDO write
anywhere. guard.check() applies the deny-list recursively to the object each
entry names, which is why the mapping constants below are built from the same
indices the deny-list already permits rather than from raw literals.

Deliberately free of `config`, like the rest of canbus/.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can  # noqa: E402
from guard import check as guard_check  # noqa: E402

# CiA 301 default: RPDO1 is 0x200 + node.
RPDO1_COB_BASE = 0x200

# Communication and mapping parameter for RPDO1. RPDO n is 0x1400 + (n-1) and
# 0x1600 + (n-1); spelled out rather than computed because only RPDO1 is used.
RPDO1_COMM = 0x1400
RPDO1_MAP = 0x1600

# Bit 31 of a COB-ID entry means "this PDO does not exist". A PDO is reconfigured
# by disabling it, rewriting it, and enabling it again - writing a mapping while
# the PDO is live is undefined behaviour in CiA 301, not merely bad manners.
COB_DISABLED = 1 << 31

# Transmission type 255 = asynchronous, event-driven: the drive acts on the
# frame when it arrives. Not SYNC-driven, because there is no SYNC producer on
# this bus and adding one would put a frame on the wire 50 times a second to
# schedule a frame we are already sending 50 times a second.
TRANSMISSION_ASYNC = 255

# Mapping entries: index << 16 | subindex << 8 | bit length.
#   6040h controlword      u16   16 bits
#   60FFh target velocity  i32   32 bits
CONTROLWORD_INDEX = 0x6040
TARGET_VELOCITY_INDEX = 0x60FF
MAP_CONTROLWORD = (CONTROLWORD_INDEX << 16) | 0x0010
MAP_TARGET_VELOCITY = (TARGET_VELOCITY_INDEX << 16) | 0x0020

# The payload the mapping above describes: u16 then i32, little-endian, 6 bytes.
# *** The order is the MAPPING order, not a convenience. *** Swap the two and
# the drive reads the low half of the velocity as a controlword, which is a
# plausible-looking value rather than an error.
_PAYLOAD = struct.Struct("<Hi")
PAYLOAD_LEN = _PAYLOAD.size             # 6

# CiA 402 controlword for "Operation enabled", which is what every tick sends
# once the arm sequence has completed. Named here so the tick does not carry a
# magic 0x000F beside a velocity.
CW_OPERATION_ENABLED = 0x000F


def cob_id(node, base=RPDO1_COB_BASE):
    return base + node


def pack(controlword, velocity_rpm):
    """(controlword, r/min) -> the 6 payload bytes. Pure; no bus, no state.

    60FFh on the BLV-R is signed r/min directly - there is no DEC unit and no
    encoder-resolution term, which is the Kinco scaling layer the generic SLAM
    plan specifies and this vehicle does not need (hardware-reconciliation D-1).

    Raises on a value that will not fit rather than truncating: a velocity that
    silently wraps is a wheel command nobody chose.

    *** The controlword goes through the deny-list on every frame. *** Moving it
    onto a PDO gives bit 7 - Fault reset, which is 40C0h by another name - a
    second route to the drive that the SDO write path's guard never sees. And
    this route fires 50 times a second, so a controlword assembled with that bit
    set would auto-reset an alarm continuously: the automatic restart ISO 3691-4
    prohibits, arrived at by accident. guard owns that rule already, so it is
    reused rather than restated.
    """
    v = int(round(velocity_rpm))
    if not -0x80000000 <= v <= 0x7FFFFFFF:
        raise ValueError(f"target velocity {v} does not fit an INT32")
    if not 0 <= int(controlword) <= 0xFFFF:
        raise ValueError(f"controlword {controlword} does not fit a UINT16")
    guard_check(CONTROLWORD_INDEX, int(controlword))
    return _PAYLOAD.pack(int(controlword), v)


def unpack(data):
    """The inverse, for tests and for decoding a captured frame."""
    if len(data) < PAYLOAD_LEN:
        raise ValueError(f"an RPDO1 payload is {PAYLOAD_LEN} bytes, got {len(data)}")
    return _PAYLOAD.unpack_from(bytes(data), 0)


def message(node, controlword, velocity_rpm, base=RPDO1_COB_BASE):
    return can.Message(arbitration_id=cob_id(node, base),
                       data=pack(controlword, velocity_rpm),
                       is_extended_id=False)


def send(bus, node, controlword, velocity_rpm, base=RPDO1_COB_BASE):
    """One setpoint frame. NOT acknowledged - see the module docstring.

    This is the whole point: a queue append rather than a blocking round trip.
    """
    bus.send(message(node, controlword, velocity_rpm, base))


def configuration_steps(node, base=RPDO1_COB_BASE):
    """The (index, sub, value, size, description) writes that set RPDO1 up.

    Returned as data rather than executed, for three reasons: the sequence can
    be asserted in a test with no bus, it can be printed before being run on a
    bench, and the ROS node can drive it through whatever SDO client it has.

    *** The order is load-bearing. *** Disable, remap, re-enable. A mapping
    written to a live PDO is undefined in CiA 301, and the count at sub 0 must
    go to zero before the entries are touched or the drive validates each entry
    against a mapping that is still half the old one.
    """
    cob = cob_id(node, base)
    return [
        (RPDO1_COMM, 1, COB_DISABLED | cob, 4, "disable RPDO1 before remapping"),
        (RPDO1_MAP, 0, 0, 1, "clear the mapping entry count"),
        (RPDO1_MAP, 1, MAP_CONTROLWORD, 4, "map 6040h controlword (16 bit)"),
        (RPDO1_MAP, 2, MAP_TARGET_VELOCITY, 4, "map 60FFh target velocity (32 bit)"),
        (RPDO1_MAP, 0, 2, 1, "two mapped objects"),
        (RPDO1_COMM, 2, TRANSMISSION_ASYNC, 1, "transmission type 255 (async)"),
        (RPDO1_COMM, 1, cob, 4, "enable RPDO1"),
    ]


def check_steps(steps=None, node=1):
    """Every configuration write, through the deny-list. Raises on the first bad one.

    Called by configure() before anything is sent, and asserted directly in the
    tests. The mapping entries are the ones that matter: guard applies the
    deny-list recursively to the object each names, so this is what stops a
    forbidden object being reached through a PDO.
    """
    for index, sub, value, _size, _what in (steps or configuration_steps(node)):
        guard_check(index, value, sub)
    return True


def configure(bus, node, sdo_write, base=RPDO1_COB_BASE, echo=None):
    """Run the sequence over SDO. `sdo_write` is injected, not imported.

    Injected so this module never decides WHICH SDO implementation is in use -
    canworker passes its guarded _write, a bench script passes drive_forward's
    raw one, and the ROS node passes whatever python-canopen gives it. That is
    the same seam open_bus() uses for the bus itself.

    Returns the step list actually executed, so a caller can log it.
    """
    steps = configuration_steps(node, base)
    check_steps(steps, node)            # before the first frame, not during
    for index, sub, value, size, what in steps:
        if echo:
            echo(f"    {index:04X}h:{sub:02X} = 0x{value:0{size * 2}X}  {what}")
        ok = sdo_write(bus, node, index, sub, value, size)
        # A tuple result is (ok, detail); a bare truthy value is ok. Both call
        # conventions exist in this repo and neither is worth changing here.
        if isinstance(ok, tuple):
            ok, detail = ok
        else:
            detail = ""
        if not ok:
            raise RuntimeError(
                f"node {node}: RPDO1 setup failed at {index:04X}h:{sub:02X} "
                f"({what}): {detail}")
    return steps
