"""Shared rig for the offline suite. No CAN, no hardware, no motion.

*** The plant simulation that used to live here is gone with tape following. ***
It integrated e_dot = v*theta + Ls*omega against the real LineFollower,
including the 1 mm sensor quantisation, the 50 Hz tick, the transport delay and
the wheel slew limit imposed by 6083h - and it carried a negative control
asserting the rig could actually SEE instability (K_RATIO = 100 had to diverge),
without which the passes would have meant nothing.

That last property is the one worth rebuilding when a navigation controller
arrives. A simulation that cannot fail is not evidence, and the slew limit it
modelled is a property of these drives rather than of the line follower: 6083h
still caps how fast the wheel difference slews, so it still caps yaw
acceleration, whatever produces (v, omega).

FAIL is shared state, deliberately. Every module appends to the same list so
run_all.py can report one verdict for the whole suite rather than eight.
"""
import os
import pathlib
import struct
import sys

# Repo root, so a check that reads a source file keeps working wherever the
# suite is run from and wherever the module it inspects has been moved to.
ROOT = pathlib.Path(__file__).resolve().parent.parent
# Same flat layout the app uses - see the note in canworker.py.
for _d in ("", "core", "drivers", "drivers/canbus", "app"):
    sys.path.insert(0, str(ROOT / _d) if _d else str(ROOT))

import config  # noqa: E402

FAIL = []

# Counted so run_all.py can pin the total. A single-element list rather than an
# int because every test module imports this by value.
CHECKS = [0]


def check(name, cond, detail=""):
    CHECKS[0] += 1
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"   {detail}" if detail else ""))
    if not cond:
        FAIL.append(name)


class _FakeRaw:
    """A bus with unsolicited traffic arriving DURING an SDO transfer.

    This is the shape that produced the deadlock the suite guards against:
    sdo_read() drains the RX queue, TpdoTap hands the pushed frames to a
    Controller handler, and that handler takes the lock the caller may already
    hold. It used to be the MLS stream at 100 Hz; with tape following gone the
    pushed traffic is the drives' producer heartbeat, which is routed through
    exactly the same path and re-enters exactly the same lock.

    Each request queues a heartbeat and then its reply; the queue then drains
    empty. Replies echo the request's (index, sub), because the SDO helpers
    discard any reply that does not - an older version of this fake answered
    every request with a zero multiplexer, so every transfer timed out and the
    arm test "passed" on an exception nobody saw.

    objects maps (index, sub) -> the value an upload returns; unlisted objects
    read as 7. Every download is acknowledged.
    """

    def __init__(self, node=1, objects=None):
        self.node = node
        self.objects = dict(objects or {})
        self.pending = [("hb",), ("hb",)]
        self.sent = []

    def send(self, m):
        self.sent.append(m)
        if 0x600 <= m.arbitration_id <= 0x67F:
            self.pending += [("hb",),
                             ("sdo", m.arbitration_id - 0x600, bytes(m.data))]

    def recv(self, timeout=None):
        import can
        if not self.pending:
            return None
        kind = self.pending.pop(0)
        if kind[0] == "hb":
            # 0x05 = Operational, the byte a producer heartbeat carries.
            return can.Message(arbitration_id=0x700 + self.node,
                               data=bytes([0x05]), is_extended_id=False)
        _, node, req = kind
        mux = req[1:4]
        if req[0] == 0x40:
            value = self.objects.get((req[1] | (req[2] << 8), req[3]), 7)
            data = bytes([0x43]) + mux + struct.pack("<I", value)
        else:
            data = bytes([0x60]) + mux + bytes(4)
        return can.Message(arbitration_id=0x580 + node, data=data,
                           is_extended_id=False)

    def shutdown(self):
        pass


def _why(g, index, value=0x40, sub=None):
    """The refusal message, so a test can assert it explains itself."""
    try:
        g.check(index, value, sub)
        return ""
    except g.ForbiddenWrite as e:
        return str(e)
