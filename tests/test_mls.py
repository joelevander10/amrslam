"""The MLS track decoders (agv_core/drivers/canbus/read_mls.py), restored for line
following.

TPDO1 is eight bytes: LCP1..LCP3 as 16-bit words, #LCP plus marker in byte 6,
status in byte 7 (SICK 8021642 table 6). The word layout depends on 2006h:01:
Standard is a plain INT16 in mm, Combi packs a 10-bit position and a 6-bit
width. Both are asserted from hand-built frames, not from the decoder's own
output, so a mis-read table shows up as a wrong number here.
"""
import struct

from helpers import check

from agv_core.drivers.canbus import guard
from agv_core.drivers.canbus import read_mls as mls


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
    check("byte 6 bit 3 is the marker intro, bits 4-7 its code, bits 3-7 the raw field",
          m == {"intro": True, "code": 9, "raw": 0b10011})


def test_short_frames_and_sdo_equivalence():
    print("\nMLS: short frames, and the SDO path decodes like a TPDO")

    check("a frame under 8 bytes is None", mls.decode_tpdo1(b"\x00" * 7, False) is None)
    check("an empty frame is None", mls.decode_tpdo1(b"", False) is None)
    a = mls.decode_tpdo1(frame(0, -120, 0, 2, 0x03), False)
    # SDO returns unsigned words: -120 reads back as 0xFF88
    b = mls.decode_sdo([0, 0xFF88, 0], 2, 0x03, False)
    check("SDO-read fields decode to the same reading as the TPDO", a == b)
    check("the TPDO1 COB-ID for node 10 is 0x18A", mls.TPDO1_COB + mls.SENSOR_NODE == 0x18A)


def _cal(step, fields, nlcp, lcp2=0, good=True, polarity="north"):
    samples = [{"field": f, "line_levels": [0, f // 10, 0], "nlcp": nlcp,
                "lcp_mm": [0, lcp2, 0], "valid": [2] if nlcp == 2 else [],
                "line_good": good, "track_level": 5 if nlcp else 0, "polarity": polarity}
               for f in fields]
    return {"step": step, "settings": {"min_level_2025h": 100, "zero_offset_mm_2026h": 0},
            "samples": samples}


def test_calibration_summary():
    print("\nMLS: background-vs-tape calibration summary")

    s = mls.cal_summary(_cal("background", [40, 50, 60], 0), _cal("tape", [400, 420, 440], 2, lcp2=-3))
    check("a clean pair is OK", s["verdict"] == "OK" and not s["problems"], str(s["problems"]))
    check("the ratio is weakest tape over strongest background", s["ratio_tape_min_to_background_max"] == 6.67)
    check("the min level is suggested midway", s["suggested_min_level_2025h"] == 230)
    check("LCP2 over the tape is the measured offset", s["tape"]["lcp2_mm"]["mean"] == -3)

    s = mls.cal_summary(_cal("background", [40, 300], 0), _cal("tape", [250, 400], 2))
    check("overlapping fields: no suggestion, CHECK",
          s["suggested_min_level_2025h"] is None and s["verdict"] == "CHECK")
    s = mls.cal_summary(_cal("background", [40, 50], 2), _cal("tape", [400, 400], 2))
    check("a track over bare floor is a problem", any("bare floor" in p for p in s["problems"]))
    s = mls.cal_summary(_cal("background", [40, 50], 0), _cal("tape", [400, 400, 400], 0))
    check("a tape step that never sees one track is a problem",
          any("one track" in p for p in s["problems"]))
    s = mls.cal_summary(_cal("background", [100, 100], 0), _cal("tape", [150, 150], 2))
    check("separated but under the ratio is a problem", any("ratio" in p for p in s["problems"]))


def test_marker_tooling():
    """mls-marker-plan (2026-10-09): set-markers, the marker watch and the Hall profile."""
    print("\nMLS: marker tooling (set-markers, markers, marker-level)")

    check("the raw marker field keeps all five bits (0xF8 -> 31)",
          mls.decode_marker(0xF8) == {"intro": True, "code": 15, "raw": 31})
    todo = mls.marker_writes({})
    check("set-markers writes markers on, standard, FailSafe and the teach lock - not polarity",
          [(i, s, v) for i, s, _z, v, _l, _n in todo]
          == [(0x2028, 1, 1), (0x2028, 2, 1), (0x2028, 3, 1), (0x2029, 0, 1)])
    check("--polarity-lock adds 202Dh:05 = 1",
          (0x202D, 5) in [(i, s) for i, s, *_ in mls.marker_writes({}, polarity_lock=True)])
    done = {(0x2028, 1): 1, (0x2028, 2): 1, (0x2028, 3): 1, (0x2029, 0): 1}
    check("a configured sensor needs no write", mls.marker_writes(done) == [])
    check("every value set-markers writes is on the guard's MLS list",
          all(guard.is_sensor_allowed(i, s) for i, s, *_ in mls.marker_writes({}, True)))

    class _Bus:  # a refused object must never reach the bus
        sent = []

        def send(self, m):
            self.sent.append(m)
    bus = _Bus()
    ok, why = mls._guarded_write(bus, 10, 0x202C, 0, 1, 1)
    check("a guarded write to the zero-point teach is refused before the bus",
          not ok and "refused" in why and bus.sent == [], why[:60])

    def rd(raw, reading=False, nlcp=2, polarity=0):
        return mls.decode_tpdo1(frame(0, 5, 0, nlcp | (raw << 3), 0x01 | (0x40 if reading else 0)
                                      | polarity), False)
    w = mls.MarkerWatch()
    lines = [w.feed(0.01 * k, r) for k, r in enumerate(
        [rd(0), rd(0, True), rd(0b00101, True), rd(0b00101), rd(0), rd(0, True), rd(0)])]
    check("the watch prints changes only",
          lines[0] is not None and all(x is not None for x in lines[1:]), str(lines))
    check("a code read counts once; a reading-code episode with no code is an abort",
          w.summary()["codes"] == {2: 1} and w.summary()["aborts"] == 1, str(w.summary()))
    w.feed(1.0, rd(0, polarity=0x20))
    check("the watch records the track polarity (B0)", w.summary()["polarity"] == ["north", "south"])

    # 36 elements over 200 mm (MLSE-0200): track peak in the middle, its overshoot
    # either side, a marker 60 mm to one side at a third of the track's size.
    vals = [0] * 36
    vals[17], vals[18] = 1000, 950
    vals[14], vals[21] = -150, -140
    vals[29] = -330
    p = mls.hall_profile(vals, 200.0, 30)
    top = p["opposite"][0]
    check("the main peak is the largest |value|, placed near the centre",
          p["main"] == 1000 and abs(p["main_mm"]) < 6, str(p["main_mm"]))
    check("a marker at a third of the track clears the 30 % level, near +60 mm",
          top["detected"] and top["pct"] == 33.0 and 55 < top["mm"] < 65, str(top))
    check("the track's own overshoot is reported but under the level",
          [o["detected"] for o in p["opposite"][1:]] == [False, False])
    # 2026-10-09 on the vehicle: 2000h answered all 168 sub-indices, 132 of them zero.
    check("the Hall element count comes from the sensor length (MLSE-0200: 36)",
          mls.HALL_ELEMENTS[200] == 36)
    check("below the min. level there is no track and no peak is judged",
          mls.hall_profile([5, -20, 18, -3], 200.0, 30, min_level=300)["no_track"]
          and mls.hall_profile([5, -20, 18, -3], 200.0, 30, min_level=300)["opposite"] == [])
    check("a bare-floor baseline is an element-wise mean",
          mls.hall_mean([[1, 2, 3], [3, 2, 1]]) == [2, 2, 2])



def test_event_and_temperature_decoding():
    print("\nMLS: event source/cause (2080h) and temperature (2070h), manual tables 22-24")

    ev = mls.decode_event(2, 1, 0)
    check("source 2 is a magnetic-field event, field code 1 below critical strength",
          ev["field"] and not ev["temp"] and ev["field_text"] == "field below the critical strength", str(ev))
    ev = mls.decode_event(6, 1, 2)
    check("source 6 is both, temp code 2 above the upper threshold",
          ev["field"] and ev["temp"] and ev["temp_text"] == "above the upper threshold", str(ev))
    check("temp code 1 is below the lower threshold",
          mls.decode_event(4, 0, 1)["temp_text"] == "below the lower threshold")
    ev = mls.decode_event(3, 7, None)
    check("an unlisted value is reported as unknown, not mapped to a listed one",
          ev["source_text"] == "unknown (3)" and ev["field_text"] == "unknown (7)", str(ev))
    ev = mls.decode_event(None, None, None)
    check("an unread source is neither field nor temperature, and says unread",
          not ev["field"] and not ev["temp"] and ev["source_text"] == "unread", str(ev))
    check("the cause reads are 2080h:02, :04, :05 and the reset 2080h:01",
          mls.EVENT_READS == ((0x2080, 2), (0x2080, 4), (0x2080, 5)) and mls.EVENT_RESET == (0x2080, 1))
    tp = mls.decode_temp(0xF6, 70, 0xF6)  # -10 degC, thresholds -10 .. 70
    check("2070h values are INT8: 0xF6 is -10 degC, on the threshold is not outside",
          tp["temp_c"] == -10 and tp["lo_c"] == -10 and not tp["out"], str(tp))
    check("above the upper threshold is outside", mls.decode_temp(71, 70, 0)["out"])
    check("below the lower threshold is outside", mls.decode_temp(0xFB, 70, 0)["out"])
    check("an unread threshold never makes a reading outside",
          not mls.decode_temp(90, None, None)["out"] and mls.decode_temp(None, 70, 0)["temp_c"] is None)
    check("the event reset is a guarded line-sensor write, the thresholds are not",
          guard.is_sensor_allowed(0x2080, 1)
          and not any(guard.is_sensor_allowed(0x2070, s) for s in (1, 6, 7))
          and not any(guard.is_sensor_allowed(0x2080, s) for s in (2, 3, 4, 5)))


TESTS = [test_standard_decoding, test_combi_decoding, test_nlcp_table,
         test_status_and_marker_bits, test_short_frames_and_sdo_equivalence,
         test_calibration_summary, test_marker_tooling, test_event_and_temperature_decoding]
