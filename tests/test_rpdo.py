"""RPDO1: the payload layout, the setup ordering, and the deny-list interaction.

Three things can go wrong here and all three are silent:

  1. the payload halves are swapped, so the drive reads the low word of the
     velocity as a controlword - a plausible value, not an error;
  2. the mapping is written while the PDO is live, which CiA 301 leaves
     undefined and drives implement differently;
  3. a forbidden object is reached through the mapping, which every existing
     deny-list check would pass.

Each has a test below, and the third is the one that matters most.
"""
import struct

from helpers import check

import guard
import rpdo


def test_payload():
    print("\nRPDO1: the 6-byte payload")

    check("the payload is 6 bytes - u16 controlword then i32 velocity",
          rpdo.PAYLOAD_LEN == 6)
    data = rpdo.pack(rpdo.CW_OPERATION_ENABLED, 1200)
    check("pack produces exactly that", len(data) == 6, str(len(data)))
    check("the controlword is the FIRST two bytes, little-endian",
          data[:2] == struct.pack("<H", 0x000F), data[:2].hex())
    check("the velocity is the LAST four bytes, little-endian",
          data[2:] == struct.pack("<i", 1200), data[2:].hex())
    check("pack and unpack round-trip",
          rpdo.unpack(rpdo.pack(0x000F, -800)) == (0x000F, -800))

    # *** 60FFh is SIGNED. *** A reverse command must not read as ~4.29e9.
    check("a negative setpoint stays negative",
          rpdo.unpack(rpdo.pack(0x000F, -1200))[1] == -1200)
    check("zero is zero", rpdo.unpack(rpdo.pack(0x000F, 0))[1] == 0)
    check("a float setpoint is rounded, not truncated toward zero",
          rpdo.unpack(rpdo.pack(0, 1199.6))[1] == 1200)

    # Wrapping silently is a wheel command nobody chose.
    for bad, why in ((0x80000000, "INT32 overflow"),
                     (-0x80000001, "INT32 underflow")):
        raised = False
        try:
            rpdo.pack(0, bad)
        except ValueError:
            raised = True
        check(f"a velocity past {why} raises rather than wrapping", raised)
    raised = False
    try:
        rpdo.pack(0x10000, 0)
    except ValueError:
        raised = True
    check("a controlword past UINT16 raises rather than wrapping", raised)

    raised = False
    try:
        rpdo.unpack(b"\x0f\x00\x01")
    except ValueError:
        raised = True
    check("a short frame raises rather than decoding half a velocity", raised)

    # *** Moving the controlword onto a PDO gives bit 7 - fault reset - a route
    # to the drive the SDO guard never sees, at 50 Hz. An alarm reset arriving
    # 50 times a second is the automatic restart ISO 3691-4 prohibits.
    raised = False
    try:
        rpdo.pack(0x000F | guard.CONTROLWORD_FAULT_RESET, 0)
    except guard.ForbiddenWrite:
        raised = True
    check("*** a controlword with bit 7 (fault reset) cannot be packed ***",
          raised)
    check("the ordinary Operation-enabled controlword still packs",
          len(rpdo.pack(rpdo.CW_OPERATION_ENABLED, 0)) == 6)
    check("every arm-sequence controlword still packs",
          all(len(rpdo.pack(cw, 0)) == 6 for cw in (0x0006, 0x0007, 0x000F)))


def test_cob_ids():
    print("\nRPDO1: COB-IDs")

    check("RPDO1 is the CiA 301 default 0x200 + node",
          rpdo.RPDO1_COB_BASE == 0x200)
    check("the two drives get distinct COB-IDs",
          (rpdo.cob_id(1), rpdo.cob_id(2)) == (0x201, 0x202))
    # The MLS streams TPDO1 on 0x18A. An RPDO1 on 0x200+n cannot collide with
    # it, which is worth asserting because a collision is two decoders reading
    # each other's frames rather than an error.
    check("neither drive's RPDO1 collides with the MLS stream on 0x18A",
          0x18A not in (rpdo.cob_id(1), rpdo.cob_id(2)))
    check("a frame carries the node's COB-ID and 6 bytes",
          rpdo.message(2, 0x000F, 500).arbitration_id == 0x202
          and len(rpdo.message(2, 0x000F, 500).data) == 6)


def test_mapping_values():
    print("\nRPDO1: the mapping entries")

    # index << 16 | subindex << 8 | bit length.
    check("the controlword maps as 6040h sub 0, 16 bits",
          rpdo.MAP_CONTROLWORD == 0x60400010,
          f"{rpdo.MAP_CONTROLWORD:08X}")
    check("the target velocity maps as 60FFh sub 0, 32 bits",
          rpdo.MAP_TARGET_VELOCITY == 0x60FF0020,
          f"{rpdo.MAP_TARGET_VELOCITY:08X}")
    check("the mapped bit lengths sum to the payload size",
          (0x10 + 0x20) // 8 == rpdo.PAYLOAD_LEN)
    check("transmission type 255 is asynchronous, needing no SYNC producer",
          rpdo.TRANSMISSION_ASYNC == 255)


def test_setup_ordering():
    print("\nRPDO1: disable, remap, re-enable - in that order")

    steps = rpdo.configuration_steps(1)
    seq = [(i, s) for i, s, _v, _sz, _w in steps]

    check("the sequence is seven writes", len(steps) == 7, str(len(steps)))
    # *** Order is load-bearing: a mapping written to a live PDO is undefined. ***
    check("the FIRST write disables the PDO",
          seq[0] == (rpdo.RPDO1_COMM, 1)
          and bool(steps[0][2] & rpdo.COB_DISABLED))
    check("the LAST write re-enables it",
          seq[-1] == (rpdo.RPDO1_COMM, 1)
          and not steps[-1][2] & rpdo.COB_DISABLED)
    check("the enable restores the same COB-ID the disable named",
          (steps[0][2] & 0x7FF) == steps[-1][2] == rpdo.cob_id(1))

    # The count must reach zero before the entries are touched, or the drive
    # validates each new entry against a mapping that is still half the old one.
    count_writes = [k for k, (i, s, v, _sz, _w) in enumerate(steps)
                    if (i, s) == (rpdo.RPDO1_MAP, 0)]
    entry_writes = [k for k, (i, s, _v, _sz, _w) in enumerate(steps)
                    if i == rpdo.RPDO1_MAP and s in (1, 2)]
    check("the entry count is zeroed BEFORE any mapping entry is written",
          count_writes[0] < min(entry_writes)
          and steps[count_writes[0]][2] == 0)
    check("the entry count is restored AFTER them, to 2",
          count_writes[-1] > max(entry_writes)
          and steps[count_writes[-1]][2] == 2)
    check("the controlword is mapped at sub 1 and the velocity at sub 2",
          steps[entry_writes[0]][2] == rpdo.MAP_CONTROLWORD
          and steps[entry_writes[1]][2] == rpdo.MAP_TARGET_VELOCITY)
    check("the transmission type is set while the PDO is still disabled",
          [k for k, (i, s, _v, _sz, _w) in enumerate(steps)
           if (i, s) == (rpdo.RPDO1_COMM, 2)][0] < len(steps) - 1)
    check("every step names itself, so a bench run can be read",
          all(w and isinstance(w, str) for _i, _s, _v, _sz, w in steps))


def test_guard_interaction():
    print("\nRPDO1: *** the mapping goes through the deny-list ***")

    # An RPDO mapping is a write path. Every step must survive guard.check(),
    # and this is what proves the D-9 guard change actually admits the sequence.
    check("every configuration write passes the deny-list",
          rpdo.check_steps(rpdo.configuration_steps(1), 1))
    check("...for both drives",
          rpdo.check_steps(rpdo.configuration_steps(2), 2))

    # And the hole it closes: mapping a forbidden object must still be refused,
    # even though the mapping OBJECT itself is now permitted.
    bad = list(rpdo.configuration_steps(1))
    bad[2] = (rpdo.RPDO1_MAP, 1, (0x403E << 16) | 0x0010, 4, "map FREE")
    raised = False
    try:
        rpdo.check_steps(bad, 1)
    except guard.ForbiddenWrite:
        raised = True
    check("a mapping of 403Eh (brake release) is refused by the deny-list",
          raised)

    bad[2] = (rpdo.RPDO1_MAP, 1, (0x40D0 << 16) | 0x0010, 4, "map clear ETO")
    raised = False
    try:
        rpdo.check_steps(bad, 1)
    except guard.ForbiddenWrite:
        raised = True
    check("a mapping of 40D0h (clear ETO) is refused too", raised)


def test_configure_runs_the_sequence():
    print("\nRPDO1: configure() checks before it sends")

    sent = []

    def fake_write(bus, node, index, sub, value, size):
        sent.append((index, sub, value, size))
        return True, ""

    steps = rpdo.configure(None, 1, fake_write)
    check("configure executes every step in order",
          [(i, s) for i, s, _v, _sz in sent]
          == [(i, s) for i, s, _v, _sz, _w in steps])
    check("configure returns what it ran", len(steps) == 7)

    # A refused SDO must stop the sequence, not leave a PDO half-mapped and
    # then enabled - which is the one state worse than not doing this at all.
    calls = []

    def failing_write(bus, node, index, sub, value, size):
        calls.append((index, sub))
        return (False, "abort 0601 0000h") if len(calls) == 3 else (True, "")

    raised = False
    try:
        rpdo.configure(None, 1, failing_write)
    except RuntimeError as e:
        raised = "RPDO1 setup failed" in str(e)
    check("a refused write aborts the sequence and names the step", raised)
    check("...before the PDO is ever re-enabled",
          (rpdo.RPDO1_COMM, 1) not in calls[1:], str(calls))

    # The guard runs before the first frame, so a bad sequence sends nothing.
    nothing = []
    bad = list(rpdo.configuration_steps(1))
    original = rpdo.configuration_steps
    rpdo.configuration_steps = lambda node, base=rpdo.RPDO1_COB_BASE: [
        (rpdo.RPDO1_MAP, 1, (0x403E << 16) | 0x0010, 4, "map FREE")]
    try:
        rpdo.configure(None, 1, lambda *a: nothing.append(a) or (True, ""))
    except guard.ForbiddenWrite:
        pass
    finally:
        rpdo.configuration_steps = original
    check("a forbidden mapping sends NOTHING at all", nothing == [], str(nothing))


def test_canworker_wiring():
    print("\nRPDO1: how canworker chooses between the two paths")

    from helpers import ROOT
    import config
    cw = (ROOT / "canworker.py").read_text(encoding="utf-8")

    check("the profile carries the flag and it ships OFF",
          hasattr(config, "CAN_USE_RPDO") and config.CAN_USE_RPDO is False,
          repr(getattr(config, "CAN_USE_RPDO", "missing")))
    check("_write_target branches on it",
          "if config.CAN_USE_RPDO:" in cw)
    check("the SDO path survives as the else branch",
          'self._write(nid, 0x60FF, 0, int(rpm), 4, "target velocity")' in cw)
    check("the RPDO path sends Operation-enabled, never a raw controlword",
          "rpdo.CW_OPERATION_ENABLED" in cw)

    # *** PDO mapping belongs in Pre-operational. *** Configuring after the NMT
    # start would remap a live PDO, which CiA 301 leaves undefined - drives
    # differ, and the ones that tolerate it do so undocumented.
    setup = cw.index("rpdo.configure(")
    nmt_start = cw.index("self._nmt(0x01, nid)")
    check("RPDO1 is configured BEFORE the NMT start, in Pre-operational",
          setup < nmt_start, f"configure at {setup}, NMT start at {nmt_start}")

    # The setup writes go through _write, which is guarded. A bench script may
    # bypass the guard; the vehicle controller may not.
    check("the setup writer goes through the guarded _write, not raw sdo_write",
          "_sdo_write_for_rpdo" in cw
          and 'self._write(node, index, sub, value, size, "RPDO1 setup")' in cw)


def test_pp_write_surface():
    print("\nguard: the profile-position write surface")

    check("607Ah target position is writable (the pp set-point)", guard.is_allowed(0x607A))
    check("6081h profile velocity is writable (the pp speed)", guard.is_allowed(0x6081))
    for idx in guard.PP_CONFIG_READ_ONLY:
        check(f"{idx:04X}h pp safety configuration is NOT writable - MEXE02 only",
              not guard.is_allowed(idx))
    check("607Ah may be carried by an RPDO (it is itself writable)",
          guard.is_allowed(0x1601, (0x607A << 16) | 0x0020, 1))
    check("6072h may NOT be smuggled in through an RPDO mapping",
          not guard.is_allowed(0x1601, (0x6072 << 16) | 0x0010, 1))


TESTS = [test_payload, test_cob_ids, test_mapping_values, test_setup_ordering,
         test_guard_interaction, test_configure_runs_the_sequence,
         test_canworker_wiring, test_pp_write_surface]
