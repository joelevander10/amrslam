"""The nanoScan3 telegram decoder, and the rule that keeps it honest.

The live UDP stream is owned by the ROS driver (amr_ws); what survives in this
repo is core/lidarframe.py and the bench tool drivers/lidar_scan.py, and this
is their test.

Everything here runs against tests/fixtures/nanoscan3_telegram.bin - five real
datagrams captured off the wire from the scanner at 192.168.3.10. No socket, no
scanner, no privileges.

The fixture is the point. A decoder built from a document and never checked
against a device is how you end up confidently plotting 1651 points at a
resolution the scanner is not using.
"""
import re
import struct

from helpers import ROOT, check

import lidarframe as lf

FIXTURE = ROOT / "tests" / "fixtures" / "nanoscan3_telegram.bin"

# What the fixture is known to contain, measured when it was captured.
TELEGRAM_LEN = 6812
BEAMS = 1652
BLOCKS = [(80, 16), (100, 24), (128, 6608), None, None, (6748, 64), None]
SPEC = (0, [6, 12, 13], False, False)


def code(path):
    """Source with docstrings and comments removed.

    Every source scan below needs this: these modules explain at length what
    they deliberately do NOT do - open port 2122, use the vendor library - and a
    naive substring search would fail on the prose forbidding the thing.
    """
    src = (ROOT / path).read_text(encoding="utf-8")
    src = re.sub(r'"""(?:.|\n)*?"""', '""', src)
    src = re.sub(r"'''(?:.|\n)*?'''", "''", src)
    return "\n".join(line.split("#")[0] for line in src.splitlines())


def datagrams():
    """The captured datagrams, in arrival order."""
    raw = FIXTURE.read_bytes()
    out, i = [], 0
    while i < len(raw):
        n = struct.unpack_from("<I", raw, i)[0]
        i += 4
        out.append(raw[i:i + n])
        i += n
    return out


def telegram():
    asm = lf.Reassembler(1.0)
    for d in datagrams():
        got = asm.push(d, 1.0)
        if got:
            return got[1]
    raise AssertionError("fixture does not reassemble")


def test_header_and_reassembly():
    """Five datagrams, one scan - in whatever order UDP feels like."""
    print("\nthe telegram reassembles")
    dgs = datagrams()
    check("the fixture is five datagrams", len(dgs) == 5, str(len(dgs)))

    h = lf.parse_datagram(dgs[0])
    check("the marker is recognised", h is not None)
    check("total_length is the telegram length", h["total_length"] == TELEGRAM_LEN,
          str(h["total_length"]))
    check("the header is 24 bytes", lf.HEADER_LEN == 24, str(lf.HEADER_LEN))

    # A bound port is not a promise about who sends to it.
    check("a foreign datagram is rejected, not decoded",
          lf.parse_datagram(b"XXXX" + bytes(40)) is None)
    check("a truncated datagram returns None rather than raising",
          lf.parse_datagram(b"MS3 ") is None)

    # UDP reorders. Arrival order must not be load-bearing.
    for order, label in ((list(reversed(dgs)), "reversed"),
                         ([dgs[2], dgs[0], dgs[4], dgs[1], dgs[3]], "shuffled")):
        asm = lf.Reassembler(1.0)
        got = None
        for d in order:
            got = asm.push(d, 1.0) or got
        check(f"{label} fragments still reassemble",
              got is not None and len(got[1]) == TELEGRAM_LEN,
              "no telegram" if got is None else str(len(got[1])))

    # A lost fragment must expire. Keeping it would let it pair with a
    # same-offset fragment from a LATER scan and emit one picture built out of
    # two different instants - which no consumer could possibly detect.
    asm = lf.Reassembler(0.05)
    for d in dgs[:4]:
        got = asm.push(d, 1.0)
    check("an incomplete telegram emits nothing", got is None)
    # A fragment of a DIFFERENT telegram, 1 s later. Re-sending a fragment of
    # the same one is not the case under test: that is a slow telegram, and
    # expiring it would be wrong.
    later = bytearray(dgs[0])
    struct.pack_into("<I", later, 12, 999999)      # a new identification
    asm.push(bytes(later), 2.0)
    check("the partial telegram was dropped, not kept", asm.dropped == 1,
          str(asm.dropped))
    check("a slow telegram is not dropped for being slow",
          lf.Reassembler(0.05).push(dgs[0], 1.0) is None)

    # Being handed a duplicate fragment must not produce an over-long telegram.
    asm = lf.Reassembler(1.0)
    got = None
    for d in dgs + [dgs[0]]:
        got = asm.push(d, 1.0) or got
    check("a duplicated fragment does not lengthen the telegram",
          got is not None and len(got[1]) == TELEGRAM_LEN)


def test_block_table_is_the_only_map():
    """Offsets come from the telegram's own directory, never from a constant."""
    print("\nthe block table is the map")
    tel = telegram()
    table = lf.block_table(tel)

    check("the table has seven slots", len(table) == 7, str(len(table)))
    check("the observed blocks are found", table == BLOCKS, str(table))

    # (0, 0) on the wire means "switched off". Read literally it is a
    # zero-length block AT offset 0, and a decoder that believes that will
    # cheerfully hand back a slice of the telegram header instead.
    check("an absent block is None, not offset 0",
          table[lf.BLK_INTRUSION] is None and table[lf.BLK_APPLICATION] is None)
    check("block() returns None for an absent block",
          lf.block(tel, table, lf.BLK_INTRUSION) is None)

    # The blocks tile the telegram exactly - which is what proves the base
    # offset is right rather than merely plausible.
    last = table[lf.BLK_LOCAL_IO]
    check("the last block ends exactly at the telegram end",
          last[0] + last[1] == len(tel), f"{last[0] + last[1]} vs {len(tel)}")

    # A block extending past the telegram is a decode error, not a short read.
    bad = bytearray(tel)
    struct.pack_into("<HH", bad, 32 + 4 * lf.BLK_STATUS, 60000, 4000)
    check("a block running past the end is refused",
          lf.block_table(bytes(bad))[lf.BLK_STATUS] is None)


def test_beam_count_comes_from_the_wire():
    """sec 2.4's trap: the datasheet says 1651, the scanner sends 1652."""
    print("\nthe beam count comes from the block size")
    tel = telegram()
    table = lf.block_table(tel)

    check("the fixture carries 1652 points", lf.beam_count(table) == BEAMS,
          str(lf.beam_count(table)))
    check("that is the block size divided by the point size",
          lf.beam_count(table) == BLOCKS[lf.BLK_MEASUREMENT][1] // lf.POINT_LEN)

    # The datasheet's 1651 is the count of VALID beams, and hardcoding it would
    # silently drop the last point of every scan. Assert it is nowhere near the
    # decode path.
    check("1651 is not hardcoded in lidarframe",
          "1651" not in code("core/lidarframe.py"))
    check("...nor in the bench tool", "1651" not in code("drivers/lidar_scan.py"))

    m = lf.measurement(tel, table)
    check("every point decodes", len(m["dist_mm"]) == BEAMS, str(len(m["dist_mm"])))
    valid = lf.valid_mask(m["status"], m["dist_mm"])
    check("most points are valid with an echo", 1400 < sum(valid) < BEAMS,
          str(sum(valid)))
    # Bit 0 is validity: the fixture's one point without it reads zero distance.
    zero = [i for i, d in enumerate(m["dist_mm"]) if d == 0]
    check("a zero-distance point is not marked valid",
          all(not (m["status"][i] & lf.STATUS_VALID) for i in zero), str(zero))
    check("no-echo returns are excluded from the mask",
          all(not v for v, d in zip(valid, m["dist_mm"]) if d >= lf.NO_ECHO_MM))

    # Decimation must sample, not resample onto a different geometry: the
    # browser reconstructs bearings as start + i*res*step.
    dec = lf.measurement(tel, table, step=4)
    check("decimation keeps every 4th point",
          dec["dist_mm"] == m["dist_mm"][::4] and dec["step"] == 4)


def test_geometry_is_read_not_assumed():
    """sec 2.4: take the angular range from the telegram, never hardcode it."""
    print("\nthe angular geometry is read from the telegram")
    tel = telegram()
    table = lf.block_table(tel)
    d = lf.derived_values(tel, table)

    check("the start angle decodes", d is not None and
          abs(d["start_angle_deg"] + 47.5) < 1e-6, str(d and d["start_angle_deg"]))
    check("the resolution decodes",
          abs(d["resolution_deg"] - 0.166666) < 1e-4, str(d["resolution_deg"]))
    span = BEAMS * d["resolution_deg"]
    # The manual warns the device may output a slightly LARGER range than
    # configured. 275.33 against a configured 275 is exactly that, and a decoder
    # that clamped to the configured value would misplace every point.
    check("the span is the configured 275 deg or a little more",
          275.0 <= span < 276.0, f"{span:.2f} deg")
    check("unnamed header words are kept raw rather than guessed",
          set(d["raw"]) == {"f0", "f3", "f4", "f5"}, str(sorted(d["raw"])))


def test_zones_are_never_silently_clear():
    """sec 2.5 and sec 4, which are the two ways this page could kill someone."""
    print("\ncut-off paths are never silently clear")
    tel = telegram()
    table = lf.block_table(tel)

    z = lf.zones(tel, table, SPEC)
    check("the profile's flag is reported, not inferred", z["validated"] is False)
    check("three paths are reported", len(z["paths"]) == 3, str(z["paths"]))
    check("the raw bytes travel with them for the walk-through", "raw" in z)

    # An offset outside the block must read as unknown. Returning False would be
    # the literal worst case: a mapping error rendering as "path clear".
    z2 = lf.zones(tel, table, (0, [6, 12, 999], False, True))
    check("an out-of-range offset reads as unknown, not clear",
          z2["paths"][2] is None, str(z2["paths"]))

    # An absent status block likewise.
    z3 = lf.zones(tel, table, (lf.BLK_INTRUSION, [0, 1, 2], False, True))
    check("an absent status block is not readable", z3["readable"] is False)
    check("...and yields no path claims", z3["paths"] == [])


def test_reader_never_writes_to_the_scanner():
    """Rule 2 of the brief, as a source scan.

    The failure this guards against is somebody adding the vendor library, or a
    "just set the UDP target" call, months from now: it would work, and it would
    quietly invalidate the scanner's safety verification.
    """
    print("\nnothing can write to the scanner")
    for name in ("drivers/lidar_scan.py", "core/lidarframe.py"):
        src = code(name)
        for bad in ("sendto", "sendall", "SOCK_STREAM", "2122",
                    "sick_safetyscanners"):
            check(f"{name} does not use {bad}", bad not in src)


TESTS = [
    test_header_and_reassembly,
    test_block_table_is_the_only_map,
    test_beam_count_comes_from_the_wire,
    test_geometry_is_read_not_assumed,
    test_zones_are_never_silently_clear,
    test_reader_never_writes_to_the_scanner,
]
