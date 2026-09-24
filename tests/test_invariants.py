"""Controller invariants around arming, disarming and the fault latch.

Three findings from the 2026-09-15 code review, each reproduced here against a
scripted SDO layer rather than a bus:

  C01  an arm that fails part-way must de-energise BOTH drives, not just
       return with the first one still in Operation enabled;
  C02  a disarm must cancel a blind run, active or counting down - the loop
       re-arms under MANUAL auto-arm on the same tick, so a run it did not
       cancel resumed without a new Start;
  C03  a latched fault refuses jogs at acceptance AND zeroes at the output.
"""
import time

from helpers import check

import config


class _Recorder:
    """Scripts _read/_write/sdo_write on a Controller and records every write."""

    def __init__(self, ctl, statusword, fail_write=None):
        self.writes = []                    # (node, index, value) in order
        self.statusword = statusword        # node -> value returned for 6041h
        self.fail_write = fail_write        # (node, index) that raises
        ctl._do_preflight = lambda: {"ok": True, "report": []}
        ctl._nmt = lambda cmd, node: None
        ctl._read_encoder_scale = lambda: None
        ctl._read = self._read
        ctl._write = self._write

    def _read(self, node, index, sub=0, fast=False, timeout=None):
        if index == 0x6041:
            return self.statusword.get(node, 0)
        return 0

    def _write(self, node, index, sub, value, size, what):
        if self.fail_write == (node, index):
            raise RuntimeError(f"node {node}: {what} ({index:04X}h) failed: timeout")
        self.writes.append((node, index, value))

    def sdo_write(self, bus, node, index, sub, value, size, timeout=0.5):
        self.writes.append((node, index, value))
        return True, ""

    def controlwords(self, node):
        return [v for n, i, v in self.writes if n == node and i == 0x6040]


def _controller():
    import canworker
    ctl = canworker.Controller()
    ctl.bus = type("Bus", (), {"send": lambda *a: None, "recv": lambda *a, **k: None,
                               "shutdown": lambda *a: None})()
    return canworker, ctl


def _patched(canworker, rec):
    """Route the module-level sdo_write _do_disarm uses through the recorder."""
    orig = canworker.sdo_write
    canworker.sdo_write = rec.sdo_write
    return lambda: setattr(canworker, "sdo_write", orig)


def _disarm_sequence_after_enable(rec, node):
    """Was CW_SHUTDOWN then CW_DISABLE_VOLTAGE written AFTER the enable?"""
    from drive_forward import CW_DISABLE_VOLTAGE, CW_ENABLE, CW_SHUTDOWN
    cws = rec.controlwords(node)
    if CW_ENABLE not in cws:
        return False
    tail = cws[cws.index(CW_ENABLE) + 1:]
    return CW_SHUTDOWN in tail and CW_DISABLE_VOLTAGE in tail


def test_partial_arm_is_rolled_back():
    """C01: the first drive is enabled, the second refuses - both end de-energised."""
    print("\ncanworker: an arm that fails part-way de-energises both drives")
    import events
    from drive_forward import SW_SPEED_IS_ZERO
    canworker, ctl = _controller()
    left, right = config.NODES
    ok_word = 0x0027 | SW_SPEED_IS_ZERO

    # -- statusword failure on the second drive ------------------------------
    rec = _Recorder(ctl, statusword={left: ok_word, right: SW_SPEED_IS_ZERO})
    restore = _patched(canworker, rec)
    events.clear()
    try:
        try:
            ctl._do_arm("manual")
            check("arm refuses when a drive stays out of Operation enabled", False, "accepted!")
        except RuntimeError as e:
            check("arm refuses when a drive stays out of Operation enabled",
                  f"node {right}" in str(e), str(e)[:70])
        check("the drive that HAD reached Operation enabled is de-energised",
              _disarm_sequence_after_enable(rec, left), str(rec.controlwords(left)))
        check("the refusing drive gets the disable sequence too",
              any(i == 0x6040 for n, i, v in rec.writes if n == right), str(rec.controlwords(right)))
        check("software state is unarmed and idle afterwards",
              ctl._armed is False and ctl._mode == "idle", f"{ctl._armed} {ctl._mode}")
        check("the rollback is logged",
              any("did not complete" in ev.get("msg", str(ev)) for ev in events.since(0)[1]),
              str([ev for ev in events.since(0)[1]][-2:]))
    finally:
        restore()

    # -- a write that raises on the second drive -----------------------------
    canworker, ctl = _controller()
    rec = _Recorder(ctl, statusword={left: ok_word, right: ok_word}, fail_write=(right, 0x6060))
    restore = _patched(canworker, rec)
    try:
        try:
            ctl._do_arm("manual")
            check("a write failure mid-arm raises", False, "accepted!")
        except RuntimeError as e:
            check("a write failure mid-arm raises", "6060" in str(e), str(e)[:70])
        check("an exception mid-arm still de-energises the enabled drive",
              _disarm_sequence_after_enable(rec, left), str(rec.controlwords(left)))
        check("software state is unarmed after the exception", ctl._armed is False)
    finally:
        restore()

    # -- the existing contract: a plain disarm on an unarmed controller is a no-op
    canworker, ctl = _controller()
    rec = _Recorder(ctl, statusword={})
    restore = _patched(canworker, rec)
    try:
        ctl._do_disarm()
        check("disarm without force on an unarmed controller touches no drive",
              rec.writes == [], str(rec.writes))
    finally:
        restore()


def test_disarm_cancels_blind_run():
    """C02: web disarm during a run, or during its start delay, kills the run."""
    print("\ncanworker: disarm cancels a blind run, active or pending")
    import events
    from drive_forward import SW_SPEED_IS_ZERO
    canworker, ctl = _controller()
    rec = _Recorder(ctl, statusword={n: SW_SPEED_IS_ZERO for n in config.NODES})
    restore = _patched(canworker, rec)
    try:
        # pending: Start pressed, AUTO_START_DELAY_S counting down
        ctl._armed, ctl._mode = True, "manual"
        ctl._blind_start_at = time.monotonic() + 5.0
        events.clear()
        ctl._do_disarm()
        check("a pending blind run is cancelled by disarm",
              ctl._blind_start_at == 0.0, str(ctl._blind_start_at))
        check("the cancellation is logged",
              any("cancelled" in ev.get("msg", str(ev)) for ev in events.since(0)[1]))

        # active: a run object is in flight
        class _Run:
            aborted = None

            def abort(self, reason):
                self.aborted = reason

            def snapshot(self):
                return {"phase": "aborted", "reason": self.aborted}

        run = _Run()
        ctl._armed, ctl._mode = True, "manual"
        ctl._blind = run
        ctl._target = (500, 500)
        ctl._do_disarm()
        check("an active blind run is aborted by disarm", ctl._blind is None and run.aborted is not None,
              f"blind={ctl._blind} aborted={run.aborted!r}")
        check("the setpoint is zero after that", ctl._target == (0, 0), str(ctl._target))
        check("the run's last snapshot is kept for the UI",
              (ctl._blind_last or {}).get("phase") == "aborted", str(ctl._blind_last))
    finally:
        restore()


def test_fault_gate():
    """C03: a latched fault refuses jogs and zeroes anything that got through."""
    print("\ncanworker: a latched fault gates motion at acceptance and at the output")
    canworker, ctl = _controller()
    ctl._armed, ctl._mode = True, "manual"
    ctl._fault = "driver silent - node 2, stopping"
    try:
        ctl.drive("forward")
        check("drive() refuses while a fault is latched", False, "accepted!")
    except RuntimeError as e:
        check("drive() refuses while a fault is latched", "fault latched" in str(e), str(e)[:70])
    check("drive() names Reset as the way out", True)

    ctl._target = (800, 800)
    out = ctl._fault_gate((800, 800))
    check("the output gate zeroes a non-zero target under a latched fault",
          out == (0, 0) and ctl._target == (0, 0), f"{out} {ctl._target}")
    check("the gate records why", "fault latched" in (ctl._last_stop_reason or ""),
          str(ctl._last_stop_reason))

    ctl._fault = None
    ctl._target = (800, 800)
    check("with no fault the gate passes the target through",
          ctl._fault_gate((800, 800)) == (800, 800))
    check("a zero target is never touched", ctl._fault_gate((0, 0)) == (0, 0))
    ctl._fault = "x"
    ctl.halt()
    check("halt() is still accepted under a fault (it only ever stops)", ctl._target == (0, 0))


TESTS = [
    test_partial_arm_is_rolled_back,
    test_disarm_cancels_blind_run,
    test_fault_gate,
]


def test_sdo_replies_match_the_request():
    """C04: a reply is this transaction's only if it echoes our (index, sub)."""
    print("\nsdo: replies are matched to the requested object")
    import struct

    import can
    from drive_forward import sdo_write
    from verify_drivers import sdo_read

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


TESTS.append(test_sdo_replies_match_the_request)
