"""Controller invariants around arming, disarming and the fault latch.

Three findings from the 2026-09-15 code review, each reproduced here against a
scripted SDO layer rather than a bus:

  C01  an arm that fails part-way must de-energise BOTH drives, not just
       return with the first one still in Operation enabled;
  C02  a disarm must cancel a blind run, active or counting down - the loop
       re-arms under MANUAL auto-arm on the same tick, so a run it did not
       cancel resumed without a new Start;
  C03  a latched fault refuses jogs at acceptance AND zeroes at the output.

The three Controller cases (a partial arm is rolled back, disarm cancels a
blind run, the fault gate) went with canworker at U11 (2026-09-21); the ROS
bus owner has them in amr_base/test/test_r05_arm_rollback.py and
test_canopen.py. What stays is the SDO reply-matching invariant, which the
shared bench helpers implement.
"""
from helpers import check


def test_sdo_replies_match_the_request():
    """C04: a reply is this transaction's only if it echoes our (index, sub)."""
    print("\nsdo: replies are matched to the requested object")
    import struct

    import can

    from agv_core.drivers.canbus.drive_forward import sdo_write
    from agv_core.drivers.canbus.verify_drivers import sdo_read

    def reply(node, cs, index, sub, payload=b"\0\0\0\0"):
        return can.Message(arbitration_id=0x580 + node,
                           data=bytes([cs, index & 0xFF, index >> 8, sub]) + payload,
                           is_extended_id=False)

    class _Bus:
        """Scripted replies, delivered in order once a request has been sent.

        Held back until send() so the pre-send drain cannot eat them - on the
        wire, a late reply lands AFTER the drain, which is the whole point.
        """

        def __init__(self, frames):
            self.frames = list(frames)
            self.sent = []
            self.live = []

        def send(self, m):
            self.sent.append(m)
            self.live, self.frames = self.frames, []

        def recv(self, timeout=None):
            return self.live.pop(0) if self.live else None

    # A late reply to an earlier 6041h read arrives while 6064h is pending.
    late = reply(1, 0x4B, 0x6041, 0, struct.pack("<I", 0x0627))
    real = reply(1, 0x43, 0x6064, 0, struct.pack("<I", 123456))
    st, val, note, _ = sdo_read(_Bus([late, real]), 1, 0x6064, 0, collision_window=0.0)
    check("a stale reply for another object is skipped, the real one returned",
          st is True and struct.unpack("<I", val)[0] == 123456, f"{st} {val!r}")
    st, val, note, _ = sdo_read(_Bus([late]), 1, 0x6064, 0, timeout=0.05)
    check("only a stale reply is a timeout, and the note says why",
          st is None and "another object" in note, f"{st} {note}")

    # A short frame from our node must not reach the unpacker.
    short = can.Message(arbitration_id=0x581, data=bytes([0x43, 0x64, 0x60]), is_extended_id=False)
    st, val, _, _ = sdo_read(_Bus([short, real]), 1, 0x6064, 0, collision_window=0.0)
    check("a malformed short frame is skipped, not unpacked",
          st is True and struct.unpack("<I", val)[0] == 123456, f"{st} {val!r}")

    # An abort is only OUR abort if it echoes our object.
    other_abort = reply(1, 0x80, 0x6041, 0, struct.pack("<I", 0x06020000))
    ours_abort = reply(1, 0x80, 0x6064, 0, struct.pack("<I", 0x06090011))
    st, code, _, _ = sdo_read(_Bus([other_abort, ours_abort]), 1, 0x6064, 0, collision_window=0.0)
    check("an abort for another object is not taken as ours",
          st is False and code == 0x06090011, f"{st} {code:#x}" if code else f"{st} {code}")

    # Same for writes: a stale 0x60 for the previous write must not confirm this one.
    stale_ack = reply(2, 0x60, 0x6083, 0)
    ack = reply(2, 0x60, 0x60FF, 0)
    ok, detail = sdo_write(_Bus([stale_ack, ack]), 2, 0x60FF, 0, 500, 4)
    check("a write waits for the ack that echoes its object", ok and detail == "ok", detail)
    ok, detail = sdo_write(_Bus([stale_ack]), 2, 0x60FF, 0, 500, 4, timeout=0.05)
    check("a stale ack alone is a timeout, not a success", not ok and detail == "timeout", detail)
    ok, detail = sdo_write(_Bus([reply(2, 0x80, 0x60FF, 0, struct.pack("<I", 0x06010002))]),
                           2, 0x60FF, 0, 500, 4)
    check("our own abort is still reported", not ok and "06010002" in detail, detail)
    # Collision detection still works on matching replies only.
    st, _, note, _ = sdo_read(_Bus([real, late, real]), 1, 0x6064, 0, collision_window=0.05)
    check("two MATCHING replies still flag a collision; the stale one is not counted",
          st is True and "2 REPLIES" in note, note)


TESTS = [test_sdo_replies_match_the_request]
