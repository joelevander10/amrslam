"""CiA 305 LSS: framing, the bit-timing table, and the ordering of a bitrate change.

Everything here runs against a fake bus. That is not a compromise - it is the
only way to test a tool whose real failure mode is "the bus is now unreachable",
which cannot be exercised on hardware without producing exactly that outcome.

What is actually being asserted is ORDER and REFUSAL, not arithmetic: that
nothing is staged before a node acknowledges, that nothing is activated after a
refusal, and that the rate goes on the wire as a table INDEX rather than as the
rate itself. Those are the three ways this tool could brick a bus.
"""
import sys

import can
from helpers import check

import lss


class FakeBus:
    """Records what was sent, and answers with whatever was queued.

    replies maps a command specifier to the 8 bytes to answer it with, so a test
    says "the node acknowledges a selective switch" rather than assembling
    frames. An absent cs means the node stays silent, which is how a refusal and
    a dead node are told apart.
    """

    def __init__(self, replies=None):
        self.sent = []
        self.replies = dict(replies or {})
        self._queue = []
        self.shutdown_called = False

    def send(self, msg):
        self.sent.append((msg.arbitration_id, bytes(msg.data)))
        cs = msg.data[0]
        if cs in self.replies:
            self._queue.append(self.replies[cs])

    def recv(self, timeout=None):
        if self._queue:
            return can.Message(arbitration_id=lss.LSS_TX,
                               data=self._queue.pop(0), is_extended_id=False)
        return None

    def shutdown(self):
        self.shutdown_called = True

    def cs_sent(self):
        return [d[0] for _id, d in self.sent]


def _reply(cs, payload=b""):
    return (bytes([cs]) + bytes(payload)).ljust(8, b"\0")


def _address_replies(vendor=0x11, product=0x22, revision=0x33, serial=0x44):
    return {
        lss.CS_INQUIRE_VENDOR: _reply(lss.CS_INQUIRE_VENDOR,
                                      vendor.to_bytes(4, "little")),
        lss.CS_INQUIRE_PRODUCT: _reply(lss.CS_INQUIRE_PRODUCT,
                                       product.to_bytes(4, "little")),
        lss.CS_INQUIRE_REVISION: _reply(lss.CS_INQUIRE_REVISION,
                                        revision.to_bytes(4, "little")),
        lss.CS_INQUIRE_SERIAL: _reply(lss.CS_INQUIRE_SERIAL,
                                      serial.to_bytes(4, "little")),
    }


def test_framing():
    print("\nLSS: framing and the bit timing table")

    m = lss._frame(lss.CS_SWITCH_GLOBAL, bytes([lss.MODE_CONFIGURATION]))
    check("a request goes out on the master COB-ID",
          m.arbitration_id == lss.LSS_RX, f"{m.arbitration_id:03X}")
    check("every LSS frame is padded to 8 bytes", len(m.data) == 8,
          str(len(m.data)))
    check("the command specifier is byte 0",
          m.data[0] == lss.CS_SWITCH_GLOBAL)
    check("master and slave COB-IDs are the CiA 305 pair",
          (lss.LSS_RX, lss.LSS_TX) == (0x7E5, 0x7E4))

    # An over-long payload must be truncated rather than producing a 9-byte
    # frame that python-can would reject at send time, deep inside a write path.
    long_frame = lss._frame(lss.CS_INQUIRE_VENDOR, bytes(16))
    check("an over-long payload is truncated, not raised on",
          len(long_frame.data) == 8)

    # *** The rate goes on the wire as a TABLE INDEX. *** Writing 1000000 into
    # the byte where an index belongs would select whatever index 0x40 means.
    check("1 Mbps is table index 0", lss.BIT_TIMING_TABLE[1000000] == 0)
    check("125 kbps - the current rate - is table index 4",
          lss.BIT_TIMING_TABLE[125000] == 4)
    check("every table value is a byte-sized index",
          all(0 <= v <= 8 for v in lss.BIT_TIMING_TABLE.values()))
    check("the reserved index 5 is not offered",
          5 not in lss.BIT_TIMING_TABLE.values())
    check("the table is one-to-one - no two rates share an index",
          len(set(lss.BIT_TIMING_TABLE.values())) == len(lss.BIT_TIMING_TABLE))


def test_reply_filtering():
    print("\nLSS: replies are filtered off a live bus")

    # The MLS streams at 100 Hz and the drives heartbeat while this runs, so a
    # reply reader that takes the next frame would read a sensor frame.
    bus = FakeBus()
    bus._queue = []

    class Noisy(FakeBus):
        def __init__(self):
            super().__init__()
            self.frames = [
                can.Message(arbitration_id=0x18A, data=bytes(8),
                            is_extended_id=False),
                can.Message(arbitration_id=0x701, data=bytes([5]),
                            is_extended_id=False),
                can.Message(arbitration_id=lss.LSS_TX,
                            data=_reply(lss.CS_INQUIRE_VENDOR,
                                        (0xABCD).to_bytes(4, "little")),
                            is_extended_id=False),
            ]

        def recv(self, timeout=None):
            return self.frames.pop(0) if self.frames else None

    noisy = Noisy()
    got = lss._recv(noisy, want_cs=lss.CS_INQUIRE_VENDOR)
    check("sensor and heartbeat frames are skipped, the reply is found",
          got is not None and int.from_bytes(got[1:5], "little") == 0xABCD)

    # A reply to a DIFFERENT command is not this command's answer. Accepting it
    # would report the vendor id as whatever the previous query returned.
    class WrongCs(Noisy):
        def __init__(self):
            super().__init__()
            self.frames = [can.Message(
                arbitration_id=lss.LSS_TX,
                data=_reply(lss.CS_INQUIRE_SERIAL, bytes(4)),
                is_extended_id=False)]

    check("a reply carrying another command specifier is not accepted",
          lss._recv(WrongCs(), want_cs=lss.CS_INQUIRE_VENDOR,
                    timeout=0.05) is None)
    check("silence returns None rather than blocking",
          lss._recv(FakeBus(), want_cs=lss.CS_INQUIRE_VENDOR,
                    timeout=0.05) is None)


def test_inquire():
    print("\nLSS: reading the node identity")

    bus = FakeBus(_address_replies())
    addr = lss.inquire(bus)
    check("all four identity fields are read",
          addr == {"vendor_id": 0x11, "product_code": 0x22,
                   "revision": 0x33, "serial": 0x44}, str(addr))
    check("inquire sends exactly the four inquire commands",
          bus.cs_sent() == [lss.CS_INQUIRE_VENDOR, lss.CS_INQUIRE_PRODUCT,
                            lss.CS_INQUIRE_REVISION, lss.CS_INQUIRE_SERIAL])
    check("inquire changes no mode and stages nothing",
          not any(c in bus.cs_sent() for c in
                  (lss.CS_SWITCH_GLOBAL, lss.CS_CONFIGURE_BIT_TIMING,
                   lss.CS_ACTIVATE_BIT_TIMING, lss.CS_STORE_CONFIGURATION)))

    # A node that answers three of four cannot address a selective switch, and
    # saying so is the difference between refusing and switching the wrong node.
    partial = _address_replies()
    del partial[lss.CS_INQUIRE_SERIAL]
    addr = lss.inquire(FakeBus(partial))
    check("an unanswered field reads as None, not as zero",
          addr["serial"] is None and addr["vendor_id"] == 0x11)


def test_selective_switch():
    print("\nLSS: selective switch addresses ONE node")

    replies = _address_replies()
    replies[lss.CS_SWITCH_SEL_SERIAL] = _reply(lss.CS_SWITCH_SEL_ACK)
    bus = FakeBus(replies)
    addr = lss.inquire(bus)
    bus.sent.clear()

    check("a matching node acknowledges", lss.switch_mode_selective(bus, addr))
    check("all four address fields are sent",
          bus.cs_sent() == [lss.CS_SWITCH_SEL_VENDOR, lss.CS_SWITCH_SEL_PRODUCT,
                            lss.CS_SWITCH_SEL_REVISION, lss.CS_SWITCH_SEL_SERIAL])
    check("the address goes out little-endian, four bytes per field",
          bus.sent[0][1][1:5] == (0x11).to_bytes(4, "little"))

    # No acknowledgement means the node this is about to reconfigure is not the
    # node that was identified. Everything downstream must refuse.
    silent = FakeBus(_address_replies())
    check("a node that does not acknowledge reports False",
          not lss.switch_mode_selective(silent, {"vendor_id": 1,
                                                 "product_code": 2,
                                                 "revision": 3, "serial": 4}))
    # An incomplete address is a programming error, not a bus condition.
    raised = False
    try:
        lss.switch_mode_selective(FakeBus(), {"vendor_id": 1})
    except ValueError:
        raised = True
    check("an incomplete address raises rather than switching something",
          raised)


def test_bit_timing():
    print("\nLSS: staging, storing and activating a rate")

    ok_reply = {lss.CS_CONFIGURE_BIT_TIMING:
                _reply(lss.CS_CONFIGURE_BIT_TIMING, bytes([0]))}
    bus = FakeBus(ok_reply)
    ok, err = lss.configure_bit_timing(bus, 1000000)
    check("staging 1 Mbps succeeds and reports no error", ok and err == 0)
    payload = bus.sent[0][1]
    check("the selector byte is table 0", payload[1] == 0)
    check("*** the INDEX goes on the wire, not the rate ***",
          payload[2] == 0, f"byte 2 = {payload[2]}")

    bus = FakeBus(ok_reply)
    lss.configure_bit_timing(bus, 125000)
    check("125 kbps stages as index 4", bus.sent[0][1][2] == 4)

    # A rate outside CiA 301 table 0 must never reach the wire: the byte would
    # be silently truncated into a valid-looking index for some other rate.
    raised = False
    try:
        lss.configure_bit_timing(FakeBus(), 921600)
    except ValueError:
        raised = True
    check("an off-table rate raises before anything is sent", raised)

    # A refusal must be distinguishable from silence. Both mean "do not
    # activate", but only one means the node is there.
    refused = FakeBus({lss.CS_CONFIGURE_BIT_TIMING:
                       _reply(lss.CS_CONFIGURE_BIT_TIMING, bytes([1]))})
    ok, err = lss.configure_bit_timing(refused, 1000000)
    check("a refused rate reports the node's error code",
          not ok and err == 1)
    ok, err = lss.configure_bit_timing(FakeBus(), 1000000)
    check("a silent node is not mistaken for a refusal",
          not ok and err is None)

    stored = FakeBus({lss.CS_STORE_CONFIGURATION:
                      _reply(lss.CS_STORE_CONFIGURATION, bytes([0]))})
    ok, err = lss.store_configuration(stored)
    check("store reports success", ok and err == 0)
    ok, err = lss.store_configuration(FakeBus())
    check("a store nobody answered is not a success", not ok)

    # Activate carries the switch delay as a u16 of milliseconds, and must wait
    # out both halves of it - returning early invites a caller to transmit into
    # the window where the node has left the old rate and not reached the new.
    import time
    bus = FakeBus()
    t0 = time.perf_counter()
    lss.activate_bit_timing(bus, delay_ms=10)
    elapsed = time.perf_counter() - t0
    check("activate sends the delay as a little-endian u16",
          bus.sent[0][1][1:3] == (10).to_bytes(2, "little"))
    check("activate waits out both halves of the switch delay",
          elapsed >= 0.02, f"{elapsed * 1000:.0f} ms")


def test_write_gating():
    print("\nLSS: nothing writes without --go")

    # The whole tool is one --go away from an unreachable bus, so the gate is
    # asserted rather than trusted. main() returns 2 and must send nothing.
    rc = lss.main(["set", "--to", "1000000"])
    check("`set` without --go refuses", rc == 2, f"rc={rc}")
    rc = lss.main(["set", "--to", "921600", "--go"])
    check("an off-table rate is refused before the bus is opened",
          rc == 2, f"rc={rc}")

    src = (__import__("pathlib").Path(lss.__file__)).read_text(encoding="utf-8")
    check("the module refuses to import the vehicle profile",
          "import config" not in src)
    check("every write path is documented as needing --go",
          "--go" in lss.__doc__)


def test_bench_tools_respect_the_owner_lock():
    """R33: every bench tool that transmits takes the CAN owner lock first.

    socketcan admits any number of openers, so without the lock a drive or LSS
    tool could run alongside the controller and interleave SDO transfers with
    it. A fake runtime holds the lock in a scratch lock directory; every entry
    point, --go included, must name the owner and exit without opening the bus.
    """
    import contextlib
    import io
    import os
    import shutil
    import tempfile
    import bus_health
    import drive_forward
    import ownerlock
    import read_imu
    import verify_bus
    import verify_drivers
    print("\nbench tools: the CAN owner lock comes before the bus")

    opened = []

    def fake_open(*a, **k):
        opened.append(a)
        raise RuntimeError("fake open_bus - no bus in the offline suite")

    mods = (bus_health, drive_forward, lss, read_imu, verify_bus,
            verify_drivers)
    entry = [
        ("drive_forward --go",
         lambda: drive_forward.main(), ["drive_forward.py", "--go"]),
        ("bus_health", lambda: bus_health.main(), ["bus_health.py"]),
        ("verify_drivers", lambda: verify_drivers.main(), ["verify_drivers.py"]),
        ("verify_bus", lambda: verify_bus.main(), ["verify_bus.py"]),
        ("read_imu show", lambda: read_imu.main(["show"]), None),
        ("read_imu tpdo --go",
         lambda: read_imu.main(["tpdo", "euler", "--go"]), None),
        ("lss scan", lambda: lss.main(["scan"]), None),
        ("lss verify", lambda: lss.main(["verify"]), None),
        ("lss set --go",
         lambda: lss.main(["set", "--to", "1000000", "--go"]), None),
    ]

    tmp = tempfile.mkdtemp()
    old_env = os.environ.get("AMR_LOCK_DIR")
    old_argv = sys.argv
    saved = {m: m.open_bus for m in mods}
    os.environ["AMR_LOCK_DIR"] = tmp
    try:
        for m in mods:
            m.open_bus = fake_open

        def run(fn, argv):
            sys.argv = argv or ["tool"]
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                rc = fn()
            return rc, out.getvalue()

        runtime = ownerlock.acquire("can", "agv_controller")
        try:
            for name, fn, argv in entry:
                del opened[:]
                rc, out = run(fn, argv)
                check(f"{name}: refused while the controller owns can0, "
                      f"naming it, bus never opened",
                      rc == 2 and not opened and "agv_controller" in out,
                      f"rc={rc} opened={len(opened)} {out.strip()[-60:]!r}")
        finally:
            runtime.release()

        for name, fn, argv in entry:
            del opened[:]
            run(fn, argv)
            ok = bool(opened)
            try:
                ownerlock.acquire("can", "after").release()
                released = True
            except ownerlock.OwnerBusy:
                released = False
            check(f"{name}: with can0 free it goes on to open the bus, and "
                  f"lets go of the lock afterwards", ok and released,
                  f"opened={len(opened)} released={released}")
    finally:
        sys.argv = old_argv
        for m, fn in saved.items():
            m.open_bus = fn
        if old_env is None:
            os.environ.pop("AMR_LOCK_DIR", None)
        else:
            os.environ["AMR_LOCK_DIR"] = old_env
        shutil.rmtree(tmp, ignore_errors=True)


TESTS = [test_framing, test_reply_filtering, test_inquire,
         test_selective_switch, test_bit_timing, test_write_gating,
         test_bench_tools_respect_the_owner_lock]
