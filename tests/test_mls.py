"""The MLS track decoders (drivers/canbus/read_mls.py), restored for line following.

TPDO1 is eight bytes: LCP1..LCP3 as 16-bit words, #LCP plus marker in byte 6,
status in byte 7 (SICK 8021642 table 6). The word layout depends on 2006h:01:
Standard is a plain INT16 in mm, Combi packs a 10-bit position and a 6-bit
width. Both are asserted from hand-built frames, not from the decoder's own
output, so a mis-read table shows up as a wrong number here.
"""
import struct

from helpers import check

import read_mls as mls


def frame(lcp1, lcp2, lcp3, byte6, status):
    return struct.pack("<hhhBB", lcp1, lcp2, lcp3, byte6, status)


def test_standard_decoding():
    print("\nMLS: Standard (INT16) track data")

    r = mls.decode_tpdo1(frame(0, -37, 0, 2, 0x01), combi=False)
    check("one track: LCP2 is the track, in mm", r["lcp"][1] == (-37, None), str(r["lcp"]))
    check("...and #LCP 2 says only LCP2 is valid", r["nlcp"] == 2 and r["valid"] == (2,))
    check("a negative position keeps its sign", mls.decode_lcp(0xFFFF, False) == (-1, None))
    check("the full INT16 range decodes", mls.decode_lcp(0x7FFF, False)[0] == 32767
          and mls.decode_lcp(0x8000, False)[0] == -32768)


def test_combi_decoding():
    print("\nMLS: Combi packing (10-bit position + 6-bit width)")

    # position -5 (10-bit two's complement 0x3FB), width 20:
    # LSB = 0xFB, MSB bit 0 = pos bit 8 (1), bits 1-6 = width, bit 7 = pos bit 9 (1)
    word = 0xFB | ((1 | (20 << 1) | (1 << 7)) << 8)
    check("a negative Combi position and its width", mls.decode_lcp(word, True) == (-5, 20),
          str(mls.decode_lcp(word, True)))
    word = 100 | ((0 | (12 << 1)) << 8)
    check("a positive Combi position and its width", mls.decode_lcp(word, True) == (100, 12))
    check("variants 1/5/6/7 are Combi, 0 and 3 are not",
          mls.COMBI_VARIANTS == {1, 5, 6, 7} and 0 not in mls.COMBI_VARIANTS and 3 not in mls.COMBI_VARIANTS)


def test_nlcp_table():
    print("\nMLS: #LCP -> which LCPs are valid (table 17)")

    for nlcp, valid in ((0, ()), (2, (2,)), (3, (1, 2)), (6, (2, 3)), (7, (1, 2, 3))):
        r = mls.decode_tpdo1(frame(1, 2, 3, nlcp, 0), combi=False)
        check(f"#LCP {nlcp} -> valid {valid}", r["valid"] == valid, str(r["valid"]))
    r = mls.decode_tpdo1(frame(1, 2, 3, 5, 0), combi=False)
    check("a reserved #LCP validates nothing and says so",
          r["valid"] == () and r["nlcp_label"].startswith("reserved"))
    check("#LCP is only bits 0-2 of byte 6 (the marker bits do not leak in)",
          mls.decode_tpdo1(frame(0, 0, 0, 0xF2, 0), False)["nlcp"] == 2)


def test_status_and_marker_bits():
    print("\nMLS: status byte and marker bits")

    st = mls.decode_status(0x01 | (5 << 1) | 0x20)
    check("bit 0 is line good", st["line_good"])
    check("bits 1-3 are the track level", st["track_level"] == 5)
    check("bit 5 set is south polarity", st["polarity"] == "south")
    check("bit 5 clear is north", mls.decode_status(0)["polarity"] == "north")
    check("bit 4 is sensor flipped, bit 7 the event flag",
          mls.decode_status(0x90)["sensor_flipped"] and mls.decode_status(0x90)["event_flag"])
    m = mls.decode_marker(0x08 | (9 << 4))
    check("byte 6 bit 3 is the marker intro, bits 4-7 its code", m == {"intro": True, "code": 9})


def test_short_frames_and_sdo_equivalence():
    print("\nMLS: short frames, and the SDO path decodes like a TPDO")

    check("a frame under 8 bytes is None", mls.decode_tpdo1(b"\x00" * 7, False) is None)
    check("an empty frame is None", mls.decode_tpdo1(b"", False) is None)
    a = mls.decode_tpdo1(frame(0, -120, 0, 2, 0x03), False)
    # SDO returns unsigned words: -120 reads back as 0xFF88
    b = mls.decode_sdo([0, 0xFF88, 0], 2, 0x03, False)
    check("SDO-read fields decode to the same reading as the TPDO", a == b)
    check("the TPDO1 COB-ID for node 10 is 0x18A", mls.TPDO1_COB + mls.SENSOR_NODE == 0x18A)


TESTS = [test_standard_decoding, test_combi_decoding, test_nlcp_table,
         test_status_and_marker_bits, test_short_frames_and_sdo_equivalence]
