"""CAN monitoring: alarm decode, the write deny-list, the SDO poller."""
import math
import os
import pathlib
import struct
import sys
import threading

from helpers import FAIL, ROOT, check, _why

import config
import kinematics
import motion

def test_can_monitoring():
    """Alarm decode, the write deny-list, and the round-robin poller.

    All pure tables and bookkeeping, so all testable without a bus. The wiring
    that puts them on the bus is checked in the two tests below.
    """
    sys.path.insert(0, str(ROOT / "drivers" / "canbus"))
    import alarms
    import canmon
    import guard
    print("\nCAN monitoring: alarms, deny-list, poller")

    # -- EMCY decode -------------------------------------------------------
    a = alarms.decode_emcy(bytes([0x53, 0xFF, 0x81, 0, 0, 0, 0, 0]))
    check("the top-priority HWTO alarm decodes",
          a["name"] == "HWTO input circuit error" and a["level"] == "error"
          and a["tier"] == alarms.MUST, a["name"])
    check("the error register is decoded from byte 2",
          a["register_bits"] == ["generic", "manufacturer"],
          str(a["register_bits"]))
    # 0000h is the drive saying everything cleared, not a fault.
    clr = alarms.decode_emcy(bytes(8))
    check("0000h is a clear, not an alarm",
          clr["cleared"] and clr["level"] == "info")
    unk = alarms.decode_emcy(bytes([0x99, 0x99, 0, 0, 0, 0, 0, 0]))
    check("an unlisted code still reports as an error",
          not unk["cleared"] and unk["level"] == "error"
          and "9999" in unk["hex"], unk["name"])
    check("a short frame decodes rather than raising",
          alarms.decode_emcy(b"\x22\xff")["code"] == 0xFF22)
    check("decode_emcy never raises on random bytes",
          all(alarms.decode_emcy(bytes([i, i, i])) for i in range(0, 256, 17)))
    missing = [c for c in (0xFF53, 0xFF68, 0xFF50, 0xFF55, 0xFF22, 0xFF25,
                           0xFF21, 0xFF26, 0x8120, 0x8130, 0x8140, 0xFF31,
                           0xFF45, 0xFF41, 0xFFF0)
               if c not in alarms.EMCY_CODES]
    check("every MUST alarm from the plan is in the table", not missing,
          f"{len(alarms.EMCY_CODES)} codes"
          + (f", missing {[hex(c) for c in missing]}" if missing else ""))

    # -- statusword flags: ILA is how the FX3 quick stop becomes visible ----
    flags = {f["name"] for f in alarms.decode_statusword_flags(0x0800)}
    check("ILA decodes from statusword bit 11", flags == {"ILA"}, str(flags))
    check("a clear statusword yields no flags",
          alarms.decode_statusword_flags(0x0027) == [])
    check("decode_statusword_flags tolerates None",
          alarms.decode_statusword_flags(None) == [])
    check("NMT state decodes from the heartbeat byte",
          alarms.decode_nmt(0x05) == "Operational"
          and alarms.decode_nmt(0x7F) == "Pre-operational")

    # -- the write deny-list (monitoring plan section 8) --------------------
    check("the setpoint is writable", guard.is_allowed(0x60FF, 800))
    check("the controlword is writable", guard.is_allowed(0x6040, 0x000F))
    check("the heartbeat interval is writable", guard.is_allowed(0x1017, 200))
    # 403Eh bit 6 is FREE: it releases the holding brake on BOTH drive wheels.
    check("403Eh (FREE / brake release) is refused",
          not guard.is_allowed(0x403E, 0x40))
    check("40D0h (clear ETO / automatic restart) is refused",
          not guard.is_allowed(0x40D0, 1))
    check("40C0h (alarm reset) is refused", not guard.is_allowed(0x40C0, 1))
    check("1011h (restore defaults) is refused", not guard.is_allowed(0x1011, 1))
    check("1010h (store parameters) is refused", not guard.is_allowed(0x1010, 1))
    check("the whole 4xxxh parameter block is refused",
          not any(guard.is_allowed(i) for i in (0x4000, 0x40C6, 0x4123, 0x4FFF)))
    # 6040h is allowed, but bit 7 of it is fault reset by another name.
    check("controlword bit 7 (fault reset) is refused",
          not guard.is_allowed(0x6040, 0x0080))
    check("a refusal explains itself", "FREE" in _why(guard, 0x403E),
          _why(guard, 0x403E)[:60])
    check("an unlisted index is refused by default",
          not guard.is_allowed(0x1000, 1))

    # -- PDO configuration -------------------------------------------------
    # An RPDO mapping is a WRITE PATH: map a forbidden object into one and a
    # plain CAN frame does what a direct SDO write is refused. So the mapping
    # ranges are admitted by rule, and the deny-list is applied recursively to
    # the object each entry names.
    MAP_60FF = (0x60FF << 16) | 0x0020        # target velocity, 32 bits
    MAP_6040 = (0x6040 << 16) | 0x0010        # controlword, 16 bits
    MAP_403E = (0x403E << 16) | 0x0010        # FREE / brake release
    check("an RPDO may map the setpoint",
          guard.is_allowed(0x1600, MAP_60FF, sub=1))
    check("an RPDO may map the controlword",
          guard.is_allowed(0x1600, MAP_6040, sub=1))
    check("*** an RPDO may NOT map 403Eh behind the deny-list ***",
          not guard.is_allowed(0x1600, MAP_403E, sub=1))
    check("the smuggled object is named in the refusal",
          "403E" in _why(guard, 0x1600, MAP_403E, sub=1),
          _why(guard, 0x1600, MAP_403E, sub=1)[:70])
    check("no 4xxxh parameter can be mapped into an RPDO",
          not any(guard.is_allowed(0x1600, (i << 16) | 0x0010, sub=1)
                  for i in (0x4000, 0x40C0, 0x40D0, 0x40C6, 0x4FFF)))
    check("sub 0 is the entry count, not an object reference",
          guard.is_allowed(0x1600, 2, sub=0))
    check("an empty mapping slot maps nothing and is permitted",
          guard.is_allowed(0x1600, 0, sub=3))
    check("an RPDO mapping without a subindex is refused, not guessed",
          not guard.is_allowed(0x1600, MAP_60FF))
    # A TPDO is a READ path - the device transmits. Refusing these would forbid
    # reading a temperature by PDO while permitting it by SDO.
    check("a TPDO may map any object, including a forbidden one",
          guard.is_allowed(0x1A00, MAP_403E, sub=1)
          and guard.is_allowed(0x1A03, (0x2034 << 16) | 0x0010, sub=1))
    check("PDO communication parameters are writable",
          all(guard.is_allowed(i, 0x40000180, sub=1)
              for i in (0x1400, 0x1800, 0x1803, 0x1806)))
    check("the PDO ranges stop where CiA 301 says they do",
          not any(guard.is_allowed(i) for i in (0x13FF, 0x1C00)))

    # Every write in canworker must go through the guard, not around it.
    cw = (ROOT / "canworker.py").read_text(encoding="utf-8")
    check("_write() calls the guard", "guard_write(index, value, sub)" in cw)
    direct = [ln.strip() for ln in cw.splitlines()
              if "sdo_write(" in ln and "def " not in ln and "guard" not in ln
              and not ln.strip().startswith(("#", "*", '"'))
              and "only ever call" not in ln]
    # _do_disarm calls sdo_write directly on the shutdown path, where raising
    # would leave the motors energised. Those are 6040h/60FFh only; the count
    # is pinned so a new bypass cannot slip in unnoticed.
    check("direct sdo_write calls are only the disarm path",
          len(direct) == 3, f"{len(direct)}: " + "; ".join(direct)[:110])
    check("no direct write targets a forbidden index",
          not any(f"0x{i:04X}" in "".join(direct) for i in guard.FORBIDDEN))

    # -- round-robin poller -------------------------------------------------
    pol = canmon.MonitorPoller([1, 2])
    seen = [pol.next_object()[1] for _ in range(len(canmon.OBJECTS))]
    check("a sweep visits every object exactly once",
          sorted(seen) == sorted(o[1] for o in canmon.OBJECTS),
          f"{len(seen)} objects")
    check("the cursor wraps and counts sweeps", pol.sweeps == 1)

    # Types come from the MANUAL, not the plan - the plan guessed 40A4h as
    # INT32 when it is INT16, and a 16-bit signed value read as 32-bit is
    # plausible-looking garbage rather than an error.
    check("40A4h is decoded as INT16 per the manual",
          canmon.OBJECT_BY_KEY["bus_v"][3] == "i16")
    check("409Bh is decoded as INT32 per the manual",
          canmon.OBJECT_BY_KEY["cur_a"][3] == "i32")
    check("a negative current decodes as regeneration",
          canmon._decode("i32", struct.pack("<i", -2500)) == -2500)
    check("an i16 sign bit is honoured",
          canmon._decode("i16", struct.pack("<h", -55)) == -55)
    check("a short payload decodes rather than raising",
          canmon._decode("i32", b"\x01") == 1)
    check("a None payload yields None", canmon._decode("i16", None) is None)

    pol.store(1, "bus_v", 482)
    pol.store(1, "drv_c", 723)
    pol.store(1, "mtr_c", 400)
    pol.store(2, "bus_v", 375)
    snap = pol.snapshot(config.MONITOR_THRESHOLDS)
    check("a raw reading is scaled to its unit",
          snap["nodes"]["1"]["bus_v"]["value"] == 48.2,
          str(snap["nodes"]["1"]["bus_v"]["value"]))
    check("a temperature over its warn threshold is flagged",
          snap["nodes"]["1"]["drv_c"]["state"] == "warn")
    check("a temperature under its warn threshold is not",
          snap["nodes"]["1"]["mtr_c"]["state"] == "ok")
    # Two-sided: low bus voltage is battery sag, and stopping distance was
    # already degraded before anything alarmed.
    check("bus voltage is flagged at the LOW end too",
          snap["nodes"]["2"]["bus_v"]["state"] == "warn",
          str(snap["nodes"]["2"]["bus_v"]["value"]))
    pol.store(1, "info", None)
    check("an unanswered read stores as blank, not zero",
          pol.snapshot()["nodes"]["1"]["info"]["value"] is None)


TESTS = [
    test_can_monitoring,
]
