#!/usr/bin/env python3
"""Read the SICK MLS magnetic line sensor on can0. Read-only - nothing moves.

Restored 2026-09-19 from 8403dcb^ for line following as an engineering feature
(manuals/slam-generalized-plan/line-follow-u11-layout-plan.md §1.1). The ROS
side (amr_base/mls_track.py, inside drive_node) imports the decoders below; the
CLI is for the bench and refuses to run while drive_node owns the bus.

The sensor is node 10 at 125 kbps. Two ways to get measurements out of it:

  snapshot  one-shot SDO read of the whole picture (identity, config, live
            values). Works in Pre-operational, so it needs no state change.
  poll      snapshot's live values on a loop, via SDO. Slow (~3 ms/read) but
            leaves the bus in Pre-operational.
  stream    decode TPDO1 at 10 ms. TPDOs only flow in Operational. With --nmt
            this sends NMT Start addressed to node 10 only and puts it back to
            Pre-operational on exit; without it, it only listens.
  calibrate two-step field survey, still read-only: --step background with the
            sensor over bare floor, then --step tape with it centred over the
            tape. Each step samples by SDO and saves to ~/.amr/mls_cal/; once
            both exist it prints the comparison and a suggested 2025h min.
            level and the measured zero offset. It suggests, never writes:
            the sensor's parameters stay a deliberate, separate act.
  markers   (2026-10-09, mls-marker-plan) stream TPDO1 and print every CHANGE of
            the marker field (byte 7 bits 3-7, raw) and the "reading code" status
            bit, with the polarity: the bench instrument for B0 and B4-B7.
  marker-level  SDO-read the raw Hall elements (2000h) and report the main peak
            and every opposite-sign peak as a % of it, against the 30 % detection
            level (202Dh:01): will a marker strip be detected, and does the track's
            own overshoot look like one? Read-only.
  set-markers  the second write (2026-10-09): 2028h:01-03 (markers on, standard,
            FailSafe), 2029h (teach key locked) and, with --polarity-lock,
            202Dh:05 = 1. Dry run unless --go; resets node 10 and reads back.
  clear-event  (2026-10-18, mls-diagnostics-plan E0.5) read the event source and
            cause (2080h:02/04/05), then write 2080h:01 (--value, 0 or 1) to reset
            the latched event flag, and read the status bit back. Dry run unless --go.
  set-variant  a write in this tool (2026-10-07): 2006h:01 Variant TPDO1,
            e.g. --value 3 (Standard enhanced: SICK p.49-50, ON for FLUSH
            diverters). Dry run unless --go. With --go it writes, resets node 10
            only (NMT 0x81) and reads the value back, so a value that did not
            survive the reset is visible. (Until 2026-10-09 it also tried a 1010h
            store, which guard.py forbids and the sensor refuses anyway.)

Every write goes through guard.check_sensor() (_guarded_write); the line sensor
has its own allow-list there, apart from the drives'.
            Standard values only (0, 2, 3, 4): a Combi value repacks TPDO1 and
            drive_node would decode it on its next start, but this tool keeps
            that a separate decision. Needs amr.service stopped.

TPDO1 (COB-ID 0x180+NodeID, SICK MLS operating instructions 8021642 table 6):

  byte 0-1  LCP1   INT16, mm      byte 6  bits 0-2 #LCP, bits 3-7 marker
  byte 2-3  LCP2   INT16, mm      byte 7  status bits (table 8)
  byte 4-5  LCP3   INT16, mm

LCP2 is the one that matters for line following: per table 17 the sensor always
populates LCP2 first, so with a single tape under the sensor LCP2 is the track
position and LCP1/LCP3 are meaningless. Only on a diverter do the others fill in.

Position sign follows the cable outlet unless 2027h (sensor flipped) is set.

*** LCP layout depends on 2006h:01 (Variant TPDO1). ***
Values 1/5/6/7 select "Combi", which repacks each 16-bit field as a 10-bit
signed position plus a 6-bit track width. This script reads 2006h:01 and
decodes accordingly rather than assuming. This unit ships set to 3 (Standard
enhanced), i.e. plain INT16.

*** Set to 3 (Standard enhanced) on 2026-10-07, from 0. *** The "enhanced"
values are the improved diverter detection, and p.49 table 21 / p.50 recommend
it ON for FLUSH diverters and OFF for NON-FLUSH ones (a separate tape running
parallel and then curving away). It had been 0 on the belief the route was
non-flush; every branch on this site is FLUSH (joined to the line being
driven), so it is now 3, written with `set-variant --value 3 --go`. The MLS
refuses 1010h store (abort 06010002, read-only) and keeps SDO writes by itself:
the value survived an NMT reset. Both 0 and 3 are Standard packing, so the
decode below is the same either way; what changes is how the sensor behaves
where two tapes are in the window at once.

Deliberately free of `config`, like the rest of canbus/.
"""
import argparse
import json
import os
import statistics
import struct
import sys
import time

import can  # noqa: E402

from agv_core.drivers.canbus import guard  # noqa: E402
from agv_core.drivers.canbus.verify_drivers import BAD, OK, WARN, open_bus, sdo_read  # noqa: E402

SENSOR_NODE = 10
TPDO1_COB = 0x180
TPDO1_COMM = 0x1800           # :01 COB-ID (bit 31 = disabled), :02 type, :05 event timer
OBJ_VARIANT = (0x2006, 1)
OBJ_LCP = 0x2021              # :01..:03 LCP1..3, :04 #LCP (+marker), :0B..:0D line levels
OBJ_NLCP = (0x2021, 4)
OBJ_STATUS = (0x2022, 0)
COB_DISABLED = 1 << 31
# Event detection and temperature, firmware V5 (manual 8.7, 8.8, pp. 57-58).
OBJ_EVENT = 0x2080            # :01 reset the flag (write), :02 source, :04 field code, :05 temp code
OBJ_TEMP = 0x2070             # :01 temperature INT8 degC, :06 upper threshold, :07 lower threshold
EVENT_READS = ((OBJ_EVENT, 2), (OBJ_EVENT, 4), (OBJ_EVENT, 5))
TEMP_READS = ((OBJ_TEMP, 1), (OBJ_TEMP, 6), (OBJ_TEMP, 7))
EVENT_RESET = (OBJ_EVENT, 1)
EVENT_SOURCE = {0: "none", 2: "magnetic field", 4: "temperature", 6: "magnetic field and temperature"}
FIELD_CODES = {0: "no event", 1: "field below the critical strength"}
TEMP_CODES = {0: "no event", 1: "below the lower threshold", 2: "above the upper threshold"}

# 2006h:01 - which of these select the packed Combi track-data format (table 10).
COMBI_VARIANTS = {1, 5, 6, 7}
VARIANT_NAMES = {
    0: "Standard", 1: "Combi",
    2: "Standard compensated", 3: "Standard enhanced",
    4: "Standard enhanced compensated", 5: "Combi compensated",
    6: "Combi enhanced", 7: "Combi enhanced compensated",
}

# byte 6 bits 0-2. Table 7 labels both 3 and 6 "Left diverter", which cannot be
# right; table 17 only says "single diverter" for each. Report which LCPs are
# valid instead of guessing a handedness the manual contradicts itself on.
NLCP_MEANING = {
    0: ("no track", ()),
    2: ("one track", (2,)),
    3: ("diverter, LCP1+LCP2", (1, 2)),
    6: ("diverter, LCP2+LCP3", (2, 3)),
    7: ("three tracks / 90 deg intersection", (1, 2, 3)),
}

FIELD_LEVEL_MT = 0.00076   # 2024h resolution, manual p.43
LINE_LEVEL_MT = 0.049      # 2021h:0B..0D resolution, manual p.43


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def decode_lcp(word, combi):
    """One 16-bit track field -> (position_mm, width_mm or None)."""
    if not combi:
        return s16(word), None
    # LSB = LCP bits 0-7; MSB bit 0 = LCP bit 8, bits 1-6 = width, bit 7 = LCP bit 9.
    lsb, msb = word & 0xFF, word >> 8
    pos = lsb | ((msb & 0x01) << 8) | ((msb >> 7) << 9)
    if pos & 0x200:                      # 10-bit two's complement
        pos -= 0x400
    return pos, (msb >> 1) & 0x3F


def decode_status(b):
    return {
        "line_good": bool(b & 0x01),
        "track_level": (b >> 1) & 0x07,   # 0-7, see manual table 19
        "sensor_flipped": bool(b & 0x10),
        "polarity": "south" if b & 0x20 else "north",
        "reading_code": bool(b & 0x40),
        "event_flag": bool(b & 0x80),
    }


def decode_marker(b):
    """byte 6 bits 3-7: bit 3 is the introductory character, bits 4-7 the code.

    `raw` is the whole 5-bit field. The manual types it INT5 (signed) yet describes
    bit 0 as the intro and bits 1-4 as code 1-15, and never says where the travel
    direction goes: the raw value is kept so the bench (plan B6) can settle it."""
    return {"intro": bool(b & 0x08), "code": (b >> 4) & 0x0F, "raw": (b >> 3) & 0x1F}


def s8(v):
    return None if v is None else (v & 0xFF) - 0x100 if v & 0x80 else v & 0xFF


def decode_event(source, field_code, temp_code):
    """2080h:02/:04/:05 -> what raised the event flag (tables 22-24). An unlisted
    value is reported as such, never mapped to the nearest listed one; a cause that
    was not read (None) is unknown, not "no event"."""
    def name(table, v):
        return "unread" if v is None else table.get(v, f"unknown ({v})")

    return {
        "source": source,
        "field": source is not None and bool(source & 2),
        "temp": source is not None and bool(source & 4),
        "source_text": name(EVENT_SOURCE, source),
        "field_code": field_code,
        "field_text": name(FIELD_CODES, field_code),
        "temp_code": temp_code,
        "temp_text": name(TEMP_CODES, temp_code),
    }


def decode_temp(raw_t, raw_hi, raw_lo):
    """2070h:01/:06/:07 (INT8, degC) -> {"temp_c", "hi_c", "lo_c", "out"}. `out` is
    True only when the temperature and the threshold it crossed were both read."""
    t, hi, lo = s8(raw_t), s8(raw_hi), s8(raw_lo)
    out = t is not None and ((hi is not None and t > hi) or (lo is not None and t < lo))
    return {"temp_c": t, "hi_c": hi, "lo_c": lo, "out": out}


def decode_tpdo1(data, combi):
    if len(data) < 8:
        return None
    w = struct.unpack_from("<HHH", data, 0)
    lcps = [decode_lcp(x, combi) for x in w]
    nlcp = data[6] & 0x07
    label, valid = NLCP_MEANING.get(nlcp, (f"reserved ({nlcp})", ()))
    return {
        "lcp": lcps, "nlcp": nlcp, "nlcp_label": label, "valid": valid,
        "marker": decode_marker(data[6]), "status": decode_status(data[7]),
    }


def decode_sdo(words, nlcp, status, combi):
    """The TPDO1 fields read one by one over SDO (2021h:01..04, 2022h) -> the
    same dict decode_tpdo1 returns, by packing them into a TPDO1 frame."""
    data = struct.pack("<HHHBB", *(w & 0xFFFF for w in words), nlcp & 0xFF, status & 0xFF)
    return decode_tpdo1(data, combi)


def fmt_reading(r):
    """One line: the track position, then the qualifiers that explain it."""
    st = r["status"]
    if r["valid"]:
        parts = []
        for i in r["valid"]:
            pos, width = r["lcp"][i - 1]
            parts.append(f"LCP{i} {pos:>+5} mm" + (f" (w {width:>2})" if width is not None else ""))
        track = "  ".join(parts)
    else:
        track = "-- no track --"
    flags = []
    if st["reading_code"]:
        flags.append("reading code")
    if r["marker"]["intro"] or r["marker"]["code"]:
        flags.append(f"marker {r['marker']['code']}")
    if st["event_flag"]:
        flags.append(f"{WARN} event flag")
    if st["sensor_flipped"]:
        flags.append("flipped")
    # With no tape under the sensor, line_good = 0 is the resting state, not a
    # fault. It is only worth flagging when a track IS detected but too weak.
    if not r["valid"]:
        good = "  --"
    else:
        good = OK if st["line_good"] else f"{BAD} weak"
    return (f"{track:<44} {good} lvl {st['track_level']} "
            f"{st['polarity']:<5} " + " ".join(flags)).rstrip()


# --- SDO path ---------------------------------------------------------------

def rd(bus, node, index, sub, signed=False):
    st, val, _, _ = sdo_read(bus, node, index, sub)
    if not st:
        return None
    n = len(val)
    return int.from_bytes(val, "little", signed=signed) if n else None


def read_variant(bus, node):
    """2006h:01. Returns (raw, is_combi). Assumes Standard if unreadable."""
    v = rd(bus, node, *OBJ_VARIANT)
    if v is None:
        print(f"    {WARN} could not read 2006h:01 - assuming Standard (plain INT16)")
        return None, False
    return v, v in COMBI_VARIANTS


def live_values(bus, node, combi):
    """The same fields TPDO1 carries, but via SDO so it works Pre-operational."""
    words = [rd(bus, node, OBJ_LCP, s) for s in (1, 2, 3)]
    if any(w is None for w in words):
        return None
    nlcp = rd(bus, node, *OBJ_NLCP)
    status = rd(bus, node, *OBJ_STATUS)
    if nlcp is None or status is None:
        return None
    return decode_sdo(words, nlcp, status, combi)


def snapshot(bus, node):
    print(f"[1] identity (node {node})")
    order = rd(bus, node, 0x2019, 0)
    if order is None:
        print(f"    {BAD} node {node} did not answer 2019h - is the sensor powered?")
        return 1
    print(f"    order number   {order}")
    for sub, name in ((1, "vendor ID"), (2, "product code"),
                      (3, "revision"), (4, "serial number")):
        v = rd(bus, node, 0x1018, sub)
        if v is not None:
            print(f"    {name:<14} 0x{v:08X}")

    print("\n[2] configuration")
    variant, combi = read_variant(bus, node)
    if variant is not None:
        name = VARIANT_NAMES.get(variant, "?")
        print(f"    2006h:01 variant TPDO1   {variant} ({name}) "
              f"-> track data {'Combi (packed)' if combi else 'Standard (INT16)'}")
    for index, sub, label, signed, unit in (
        (0x2025, 0, "2025h min. level", False, "digits"),
        (0x2026, 0, "2026h zero offset", True, "mm"),
        (0x2027, 0, "2027h sensor flipped", False, ""),
        (0x2028, 1, "2028h:01 use markers", False, ""),
        (0x2028, 2, "2028h:02 marker style", False, ""),
        (0x2028, 3, "2028h:03 marker FailSafe", False, ""),
        (0x2029, 0, "2029h lock teach key", False, ""),
        (0x202D, 1, "202Dh:01 first level", False, "% of main peak"),
        (0x202D, 2, "202Dh:02 last level", False, "% of main peak"),
        (0x202D, 5, "202Dh:05 tape polarity", False, "(0 both, 1 north, 2 south)"),
    ):
        v = rd(bus, node, index, sub, signed)
        if v is not None:
            print(f"    {label:<24} {v}{' ' + unit if unit else ''}")
    cob = rd(bus, node, TPDO1_COMM, 1)
    ttype = rd(bus, node, TPDO1_COMM, 2)
    timer = rd(bus, node, TPDO1_COMM, 5)
    if cob is not None:
        state = f"{BAD} DISABLED" if cob & COB_DISABLED else f"{OK} enabled"
        print(f"    1800h TPDO1              COB-ID 0x{cob & 0x7FF:03X} {state}, "
              f"type 0x{ttype or 0:02X}, event timer {timer} ms")

    print("\n[3] live measurement")
    field = rd(bus, node, 0x2024, 0)
    minlvl = rd(bus, node, 0x2025, 0)
    if field is not None:
        note = ""
        if minlvl is not None and field < minlvl:
            note = f"   <-- below min. level {minlvl}, no track will be reported"
        print(f"    2024h field level        {field} "
              f"({field * FIELD_LEVEL_MT:.2f} mT){note}")
    for sub, i in ((0x0B, 1), (0x0C, 2), (0x0D, 3)):
        v = rd(bus, node, OBJ_LCP, sub, signed=True)
        if v is not None:
            print(f"    2021h:{sub:02X} line level {i}     {v:>+4} "
                  f"({v * LINE_LEVEL_MT:+.2f} mT)")
    r = live_values(bus, node, combi)
    if r is None:
        print(f"    {BAD} could not read the track objects")
        return 1
    print(f"\n    #LCP {r['nlcp']} - {r['nlcp_label']}")
    print(f"    {fmt_reading(r)}")

    print("\n[4] events and temperature (firmware V5, manual 8.7-8.8)")
    print_event_state(bus, node)
    return 0


def print_event_state(bus, node):
    """2080h source/cause, 2070h temperature and thresholds, as raw values and decoded."""
    ev = decode_event(*(rd(bus, node, *k) for k in EVENT_READS))
    tp = decode_temp(*(rd(bus, node, *k) for k in TEMP_READS))
    print(f"    2080h:02 event source    {ev['source']} ({ev['source_text']})")
    print(f"    2080h:04 field code      {ev['field_code']} ({ev['field_text']})")
    print(f"    2080h:05 temp code       {ev['temp_code']} ({ev['temp_text']})")
    print(f"    2070h:01 temperature     {tp['temp_c']} degC   thresholds "
          f"{tp['lo_c']} .. {tp['hi_c']} degC (2070h:07 .. :06)"
          + (f"   {WARN} outside" if tp["out"] else ""))
    return ev, tp


def poll(bus, node, seconds, interval):
    variant, combi = read_variant(bus, node)
    print(f"polling node {node} by SDO for {seconds:.0f} s "
          f"({'Combi' if combi else 'Standard'} track data). Ctrl-C to stop.\n")
    t_end = time.time() + seconds
    while time.time() < t_end:
        r = live_values(bus, node, combi)
        print(f"    {fmt_reading(r) if r else BAD + ' no response'}")
        time.sleep(interval)
    return 0


# --- calibrate: background vs tape ----------------------------------------

CAL_DIR = os.path.expanduser("~/.amr/mls_cal")
CAL_STEPS = ("background", "tape")
# A tape step must see one track this often, and its weakest field must clear the
# strongest background by this ratio. Engineering margins, not SICK figures.
CAL_DETECT_MIN = 0.95
CAL_RATIO_MIN = 2.0


def cal_sample(bus, node, combi):
    """One SDO sample of everything the comparison needs, or None."""
    r = live_values(bus, node, combi)
    field = rd(bus, node, 0x2024, 0)
    if r is None or field is None:
        return None
    levels = [rd(bus, node, OBJ_LCP, sub, signed=True) for sub in (0x0B, 0x0C, 0x0D)]
    return {
        "field": field,
        "line_levels": levels,
        "nlcp": r["nlcp"],
        "lcp_mm": [pos for pos, _ in r["lcp"]],
        "valid": list(r["valid"]),
        "line_good": r["status"]["line_good"],
        "track_level": r["status"]["track_level"],
        "polarity": r["status"]["polarity"],
    }


def cal_record(bus, node, step, samples, interval):
    """Sample one step and save it with the sensor's current settings."""
    variant, combi = read_variant(bus, node)
    settings = {
        "min_level_2025h": rd(bus, node, 0x2025, 0),
        "zero_offset_mm_2026h": rd(bus, node, 0x2026, 0, signed=True),
        "flipped_2027h": rd(bus, node, 0x2027, 0),
        "variant_2006h": variant,
    }
    print(f"[{step}] sampling {samples}x by SDO, every {interval:.2f} s - keep the vehicle still")
    got, missed = [], 0
    for _ in range(samples):
        x = cal_sample(bus, node, combi)
        if x is None:
            missed += 1
        else:
            got.append(x)
        time.sleep(interval)
    if not got:
        print(f"    {BAD} no response from node {node}")
        return None
    rec = {"step": step, "time": time.strftime("%Y-%m-%dT%H:%M:%S"), "node": node,
           "settings": settings, "missed": missed, "samples": got}
    os.makedirs(CAL_DIR, exist_ok=True)
    path = os.path.join(CAL_DIR, f"{step}.json")
    with open(path, "w") as f:
        json.dump(rec, f, indent=1)
    print(f"    {OK} {len(got)} samples ({missed} missed) -> {path}")
    return rec


def _stats(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return None
    return {"min": min(xs), "max": max(xs), "mean": round(statistics.fmean(xs), 2),
            "sd": round(statistics.pstdev(xs), 2)}


def cal_summary(bg, tape):
    """Compare a background and a tape record. Pure: the tests feed it dicts."""
    b, t = bg["samples"], tape["samples"]
    bf, tf = _stats([s["field"] for s in b]), _stats([s["field"] for s in t])
    lcp2 = _stats([s["lcp_mm"][1] for s in t if 2 in s["valid"]])
    out = {
        "background": {
            "field": bf,
            "line_level_max": max((abs(v) for s in b for v in s["line_levels"] if v is not None), default=None),
            "false_track_rate": round(sum(s["nlcp"] != 0 for s in b) / len(b), 3),
        },
        "tape": {
            "field": tf,
            "line_level_max": max((abs(v) for s in t for v in s["line_levels"] if v is not None), default=None),
            "one_track_rate": round(sum(s["nlcp"] == 2 for s in t) / len(t), 3),
            "line_good_rate": round(sum(s["line_good"] for s in t) / len(t), 3),
            "track_level": _stats([s["track_level"] for s in t]),
            "lcp2_mm": lcp2,
            "polarity": sorted({s["polarity"] for s in t if s["nlcp"]}),
        },
        "settings": tape["settings"],
    }
    ratio = (tf["min"] / bf["max"]) if bf["max"] > 0 else None
    out["ratio_tape_min_to_background_max"] = None if ratio is None else round(ratio, 2)
    # Midway between the strongest background and the weakest tape reading. Only
    # meaningful when the two do not overlap.
    out["suggested_min_level_2025h"] = (round((bf["max"] + tf["min"]) / 2)
                                        if tf["min"] > bf["max"] else None)
    problems = []
    if out["background"]["false_track_rate"] > 0:
        problems.append("a track was reported over bare floor (steel, rebar or a magnet nearby?)")
    if out["tape"]["one_track_rate"] < CAL_DETECT_MIN:
        problems.append(f"one track seen in only {out['tape']['one_track_rate']:.0%} of tape samples")
    if out["suggested_min_level_2025h"] is None:
        problems.append("tape and background field levels overlap")
    elif ratio is not None and ratio < CAL_RATIO_MIN:
        problems.append(f"tape/background ratio {ratio:.2f} below {CAL_RATIO_MIN}")
    if len(out["tape"]["polarity"]) > 1:
        problems.append("polarity changed during the tape step")
    out["problems"] = problems
    out["verdict"] = "OK" if not problems else "CHECK"
    return out


def cal_print(s):
    bf, tf, st = s["background"]["field"], s["tape"]["field"], s["settings"]
    mt = lambda v: f"{v} ({v * FIELD_LEVEL_MT:.2f} mT)"
    print("\n[report] 2024h field level, digits")
    print(f"    background  min {bf['min']}  mean {bf['mean']}  max {mt(bf['max'])}")
    print(f"    tape        min {mt(tf['min'])}  mean {tf['mean']}  max {tf['max']}")
    print(f"    ratio tape min / background max   {s['ratio_tape_min_to_background_max']}")
    print(f"    line level |max|  background {s['background']['line_level_max']}  "
          f"tape {s['tape']['line_level_max']}  (x{LINE_LEVEL_MT} mT)")
    t = s["tape"]
    print(f"    tape: one track {t['one_track_rate']:.0%}, line_good {t['line_good_rate']:.0%}, "
          f"track level {t['track_level']['mean'] if t['track_level'] else '-'}/7, "
          f"polarity {'/'.join(t['polarity']) or '-'}")
    print(f"    background: false track {s['background']['false_track_rate']:.0%}")
    print("\n[report] settings")
    sug = s["suggested_min_level_2025h"]
    print(f"    2025h min. level   now {st['min_level_2025h']}   suggested {sug if sug is not None else '-'}"
          "   (midway; not written)")
    lcp2 = t["lcp2_mm"]
    if lcp2:
        print(f"    LCP2 over the tape {lcp2['mean']:+.1f} mm (sd {lcp2['sd']})   "
              f"2026h zero offset now {st['zero_offset_mm_2026h']} mm")
        print("      if the tape was centred under the sensor, that LCP2 is the mounting offset")
    print(f"\n    verdict: {OK if s['verdict'] == 'OK' else WARN + ' CHECK'}")
    for p in s["problems"]:
        print(f"      - {p}")


def cal_load(step):
    try:
        with open(os.path.join(CAL_DIR, f"{step}.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def calibrate(bus, node, step, samples, interval):
    """--step background | tape | both (prompts between) | report (no bus)."""
    steps = CAL_STEPS if step == "both" else (() if step == "report" else (step,))
    for i, st in enumerate(steps):
        if step == "both":
            where = "bare floor, no tape within 0.5 m" if st == "background" else "centred over the tape"
            input(f"\n{'' if i == 0 else 'Move the vehicle. '}Put the sensor {where}, then press Enter ")
        if cal_record(bus, node, st, samples, interval) is None:
            return 1
    bg, tape = cal_load("background"), cal_load("tape")
    if not bg or not tape:
        print(f"\n    next: --step {'tape' if bg else 'background'} (have: "
              f"{', '.join(k for k, v in (('background', bg), ('tape', tape)) if v) or 'none'})")
        return 0
    s = cal_summary(bg, tape)
    cal_print(s)
    path = os.path.join(CAL_DIR, f"report-{time.strftime('%Y%m%d-%H%M%S')}.json")
    with open(path, "w") as f:
        json.dump(s, f, indent=1)
    print(f"\n    saved {path}")
    return 0 if s["verdict"] == "OK" else 1


# --- PDO path ---------------------------------------------------------------

STANDARD_VARIANTS = (0, 2, 3, 4)


def _guarded_write(bus, node, index, sub, value, size):
    """The ONLY MLS write path in this tool: guard.check_sensor first, then the SDO.
    Returns (ok, detail); a refused object never reaches the bus."""
    from agv_core.drivers.canbus.drive_forward import sdo_write  # noqa: PLC0415

    try:
        guard.check_sensor(index, sub)
    except guard.ForbiddenWrite as e:
        return False, str(e)
    return sdo_write(bus, node, index, sub, value, size)


def set_variant(bus, node, value, go=False):
    """Write 2006h:01, reset the node, read back. Dry run unless go."""
    if value not in STANDARD_VARIANTS:
        print(f"{BAD} {value} is not a Standard variant {STANDARD_VARIANTS}; refusing.")
        return 2
    now = rd(bus, node, *OBJ_VARIANT)
    if now is None:
        print(f"{BAD} node {node} did not answer 2006h:01 - is the sensor powered and on can0?")
        return 1
    print(f"    2006h:01 now  {now} ({VARIANT_NAMES.get(now, '?')})")
    print(f"    2006h:01 want {value} ({VARIANT_NAMES[value]})")
    if now == value:
        print(f"    {OK} already set; nothing written.")
        return 0
    if not go:
        print(f"\n{WARN} this writes to the sensor (2006h:01, then NMT reset of node {node}). "
              "Re-run with --go. Nothing has been changed.")
        return 2

    ok, detail = _guarded_write(bus, node, OBJ_VARIANT[0], OBJ_VARIANT[1], value, 1)
    if not ok:
        print(f"{BAD} write 2006h:01 refused: {detail}")
        return 1
    print(f"    {OK} wrote 2006h:01 = {value}; reads back {rd(bus, node, *OBJ_VARIANT)}")
    print(f"[nmt] Reset Node -> node {node} only")
    nmt(bus, 0x81, node)
    time.sleep(3.0)
    after = rd(bus, node, *OBJ_VARIANT)
    if after == value:
        print(f"    {OK} after reset 2006h:01 = {after} ({VARIANT_NAMES.get(after, '?')}): it persisted.")
        return 0
    print(f"    {BAD} after reset 2006h:01 = {after}: the value did NOT survive the reset.")
    return 1


# --- markers (mls-marker-plan, 2026-10-09) ----------------------------------

# (index, sub, size, value, label). Standard mode, FailSafe, teach key locked; the
# polarity lock only on request (plan B0: south-up track would vanish with it).
MARKER_SETTINGS = (
    (0x2028, 1, 1, 1, "use markers"),
    (0x2028, 2, 1, 1, "marker style: SICK standard"),
    (0x2028, 3, 1, 1, "FailSafe"),
    (0x2029, 0, 1, 1, "lock teach key"),
)
POLARITY_LOCK = (0x202D, 5, 1, 1, "tape polarity: north track, south markers")


def marker_writes(current, polarity_lock=False):
    """[(index, sub, size, want, label, now)] still to write. `current` maps
    (index, sub) -> the value read back (None = unread). Pure: the tests feed it."""
    wanted = MARKER_SETTINGS + ((POLARITY_LOCK,) if polarity_lock else ())
    return [(i, s, size, want, label, current.get((i, s)))
            for i, s, size, want, label in wanted if current.get((i, s)) != want]


def set_markers(bus, node, go=False, polarity_lock=False):
    """Write the marker configuration, reset node 10, read every value back."""
    keys = [(i, s) for i, s, *_ in MARKER_SETTINGS + (POLARITY_LOCK,)]
    current = {k: rd(bus, node, *k) for k in keys}
    if current[(0x2028, 1)] is None:
        print(f"{BAD} node {node} did not answer 2028h:01 - is the sensor powered and on can0?")
        return 1
    todo = marker_writes(current, polarity_lock)
    for i, s, _size, want, label, _now in marker_writes({}, polarity_lock):
        print(f"    {i:04X}h:{s:02X} {label:<42} now {current[(i, s)]}  want {want}")
    if not polarity_lock:
        print(f"    202Dh:05 {'tape polarity (left alone: --polarity-lock after B0)':<42} "
              f"now {current[(0x202D, 5)]}")
    if not todo:
        print(f"    {OK} already set; nothing written.")
        return 0
    if not go:
        print(f"\n{WARN} this writes {len(todo)} value(s) to the sensor, then resets node {node}. "
              "Re-run with --go. Nothing has been changed.")
        return 2
    for i, s, size, want, label, _now in todo:
        ok, detail = _guarded_write(bus, node, i, s, want, size)
        if not ok:
            print(f"{BAD} write {i:04X}h:{s:02X} ({label}) refused: {detail}")
            return 1
        print(f"    {OK} wrote {i:04X}h:{s:02X} = {want}")
    print(f"[nmt] Reset Node -> node {node} only")
    nmt(bus, 0x81, node)
    time.sleep(3.0)
    after = {k: rd(bus, node, *k) for k in keys}
    left = marker_writes(after, polarity_lock)
    for i, s, _size, want, _label, _now in marker_writes({}, polarity_lock):
        print(f"    after reset {i:04X}h:{s:02X} = {after[(i, s)]} (want {want})")
    if left:
        print(f"    {BAD} {len(left)} value(s) did NOT survive the reset.")
        return 1
    print(f"    {OK} every value persisted. Power-cycle the sensor and run snapshot (plan B2).")
    return 0


class MarkerWatch:
    """Edges of the marker field and the reading-code bit, from decoded readings.
    Pure: feed(t, reading) returns a line to print or None; summary() at the end."""

    def __init__(self):
        self.prev = None          # (raw, reading_code)
        self.counts = {}          # code -> times reported
        self.aborts = 0           # reading-code episodes that ended with no code
        self.polarity = set()     # seen while a track was present
        self._seen_code = False

    def feed(self, t, r):
        st, mk = r["status"], r["marker"]
        if r["nlcp"]:
            self.polarity.add(st["polarity"])
        cur = (mk["raw"], st["reading_code"])
        if cur == self.prev:
            return None
        prev, self.prev = self.prev, cur
        if mk["code"] and (prev is None or prev[0] != mk["raw"]):
            self.counts[mk["code"]] = self.counts.get(mk["code"], 0) + 1
        if st["reading_code"] or mk["code"]:
            self._seen_code = self._seen_code or bool(mk["code"])
        if prev is not None and prev[1] and not st["reading_code"]:
            if not self._seen_code and not mk["code"]:
                self.aborts += 1
            self._seen_code = False
        lcp2 = r["lcp"][1][0] if 2 in r["valid"] else None
        return (f"{t:9.3f}s  raw {mk['raw']:05b} ({mk['raw']:2d})  intro {int(mk['intro'])}  "
                f"code {mk['code']:2d}  reading {int(st['reading_code'])}  "
                f"LCP2 {'--' if lcp2 is None else f'{lcp2:+d}'}  #LCP {r['nlcp']}  {st['polarity']}")

    def summary(self):
        return {"codes": dict(sorted(self.counts.items())), "aborts": self.aborts,
                "polarity": sorted(self.polarity)}


def markers(bus, node, seconds, start_nmt):
    """Stream TPDO1 and print every marker / reading-code change. Read-only."""
    _variant, combi = read_variant(bus, node)
    cob = TPDO1_COB + node
    if start_nmt:
        print(f"[nmt] Start Remote Node -> node {node} only")
        nmt(bus, 0x01, node)
    print(f"watching markers on 0x{cob:03X} for {seconds:.0f} s. Ctrl-C to stop.\n")
    w, n, t0 = MarkerWatch(), 0, time.time()
    t_end = t0 + seconds
    try:
        while time.time() < t_end:
            m = bus.recv(timeout=max(0.0, t_end - time.time()))
            if m is None:
                break
            if m.arbitration_id != cob:
                continue
            r = decode_tpdo1(bytes(m.data), combi)
            if r is None:
                continue
            n += 1
            line = w.feed(time.time() - t0, r)
            if line:
                print(f"    {line}")
    except KeyboardInterrupt:
        print("\n    interrupted")
    s = w.summary()
    if n == 0:
        print(f"    {BAD} no TPDO1 seen on 0x{cob:03X} (Pre-operational? try --nmt)")
        return 1
    print(f"\n    {n} frames · codes {s['codes'] or 'none'} · aborted reads {s['aborts']} · "
          f"polarity {'/'.join(s['polarity']) or '-'}")
    return 0


def clear_event(bus, node, value, go=False):
    """E0.5: which value written to 2080h:01 resets the latched event flag?"""
    if value not in (0, 1):
        print(f"{BAD} --value must be 0 or 1 (the manual does not say which resets 2080h:01)")
        return 2
    status = rd(bus, node, *OBJ_STATUS)
    if status is None:
        print(f"{BAD} node {node} did not answer 2022h - is the sensor powered and on can0?")
        return 1
    print(f"    2022h status             0x{status:02X}, event flag {int(bool(status & 0x80))}")
    print_event_state(bus, node)
    if not go:
        print(f"\n{WARN} this writes 2080h:01 = {value} (reset the event flag). "
              "Re-run with --go. Nothing has been changed.")
        return 2
    ok, detail = _guarded_write(bus, node, EVENT_RESET[0], EVENT_RESET[1], value, 1)
    if not ok:
        print(f"{BAD} write 2080h:01 refused: {detail}")
        return 1
    time.sleep(0.05)
    after = rd(bus, node, *OBJ_STATUS)
    flag = None if after is None else bool(after & 0x80)
    print(f"    {OK} wrote 2080h:01 = {value}; status now "
          f"{'unread' if after is None else f'0x{after:02X}'}, event flag {flag}")
    print_event_state(bus, node)
    if flag:
        print(f"    {WARN} the flag is still set: the cause persists, or {value} is not the reset value")
        return 1
    return 0


HALL_OBJ = 0x2000
# Hall elements per sensor length (manual p. 44). 2000h answers all 168 sub-indices on
# every length - the MLSE-0200 here returned 36 values and 132 zeros (2026-10-09) - so
# the count cannot be found by reading until an abort.
HALL_ELEMENTS = {200: 36, 300: 54, 400: 66, 500: 84, 600: 102}


def hall_profile(values, range_mm, detection_pct=30, min_level=0):
    """The raw Hall elements across the sensor -> main peak and opposite-sign peaks.

    values: element readings left to right (2000h:01..n). The main peak is the largest
    absolute value; every local extremum of the opposite sign is listed with its
    position and its size as % of the main peak, flagged when it clears the detection
    level (a marker should; the track's own overshoot must not). Pure.

    Below `min_level` (2025h, the sensor's own "a line is present" threshold, in the same
    digits) there is no track to measure against: `no_track` is set and no peak is listed,
    because a percentage of noise is noise (2026-10-09, bare floor: +-20 digits)."""
    n = len(values)
    if n < 3:
        return None
    pitch = range_mm / n
    pos = [round((k + 0.5) * pitch - range_mm / 2, 1) for k in range(n)]
    k_main = max(range(n), key=lambda k: abs(values[k]))
    main = values[k_main]
    if main == 0 or abs(main) < min_level:
        return {"main": main, "main_mm": pos[k_main], "opposite": [], "elements": n, "no_track": True}
    sign = 1 if main > 0 else -1
    opp = []
    for k in range(n):
        v = values[k] * -sign       # opposite polarity, made positive
        if v <= 0:
            continue
        left = values[k - 1] * -sign if k > 0 else float("-inf")
        right = values[k + 1] * -sign if k < n - 1 else float("-inf")
        if v >= left and v >= right:
            pct = round(100.0 * v / abs(main), 1)
            opp.append({"mm": pos[k], "pct": pct, "detected": pct >= detection_pct})
    opp.sort(key=lambda o: -o["pct"])
    # Which sign a north-up tape gives is not stated in the manual: report the sign only.
    return {"main": main, "main_mm": pos[k_main], "opposite": opp, "elements": n,
            "pitch_mm": round(pitch, 2), "no_track": False}


def hall_mean(samples):
    """Element-wise mean of equal-length Hall readings (rounded ints)."""
    return [round(sum(col) / len(col)) for col in zip(*samples, strict=True)]


def marker_level(bus, node, samples, interval, range_mm=200, save=None, baseline=None):
    """Read the Hall elements `samples` times and report hall_profile() of their mean.

    The mounted sensor sees a standing field of its own (manual p. 45, figure 20: on this
    vehicle about +150 falling to -30 across the sensor with no tape), which hides a
    marker's peak. Save a bare-floor profile once (--save floor, sensor over no tape), then
    measure with --baseline floor: the floor is subtracted element by element."""
    n = HALL_ELEMENTS.get(int(range_mm))
    if n is None:
        print(f"{BAD} no Hall element count for a {range_mm:.0f} mm sensor {sorted(HALL_ELEMENTS)}")
        return 2
    first = rd(bus, node, 0x202D, 1)
    detect = first if first is not None else 30
    min_level = rd(bus, node, 0x2025, 0) or 0
    base = None
    if baseline:
        rec = cal_load(f"hall-{baseline}")
        if not rec or len(rec.get("mean", [])) != n:
            print(f"{BAD} no saved {n}-element profile 'hall-{baseline}' (record it with --save {baseline})")
            return 2
        base = rec["mean"]
    print(f"    {n} Hall elements over {range_mm:.0f} mm, detection level {detect} % (202Dh:01)"
          + (f", minus the saved '{baseline}' profile" if base else ""))
    got = []
    for i in range(samples):
        vals = [rd(bus, node, HALL_OBJ, k, signed=True) for k in range(1, n + 1)]
        if any(v is None for v in vals):
            print(f"    {WARN} sample {i + 1}: an element did not answer")
            continue
        got.append(vals)
        if base:
            vals = [v - b for v, b in zip(vals, base, strict=True)]
        p = hall_profile(vals, range_mm, detect, min_level)
        if p["no_track"]:
            print(f"    main {p['main']:+6d} at {p['main_mm']:+6.1f} mm   below min. level {min_level} "
                  "(2025h): no track under the sensor, nothing to compare a marker with")
            time.sleep(interval)
            continue
        top = p["opposite"][:3]
        print(f"    main {p['main']:+6d} at {p['main_mm']:+6.1f} mm   opposite peaks: "
              + (", ".join(f"{o['pct']:5.1f} % at {o['mm']:+6.1f} mm{' DETECTED' if o['detected'] else ''}"
                           for o in top) or "none"))
        time.sleep(interval)
    if not got:
        return 1
    if save:
        os.makedirs(CAL_DIR, exist_ok=True)
        path = os.path.join(CAL_DIR, f"hall-{save}.json")
        with open(path, "w") as f:
            json.dump({"time": time.strftime("%Y-%m-%dT%H:%M:%S"), "range_mm": range_mm,
                       "samples": len(got), "mean": hall_mean(got)}, f, indent=1)
        print(f"    {OK} mean of {len(got)} samples saved -> {path}")
    return 0


def nmt(bus, command, node):
    bus.send(can.Message(arbitration_id=0x000, data=[command, node],
                         is_extended_id=False))
    time.sleep(0.05)


def stream(bus, node, seconds, start_nmt):
    variant, combi = read_variant(bus, node)
    cob = TPDO1_COB + node

    if start_nmt:
        print(f"[nmt] Start Remote Node -> node {node} only (not a broadcast, so "
              f"the drivers stay in Pre-operational)")
        nmt(bus, 0x01, node)

    print(f"listening for TPDO1 on 0x{cob:03X} for {seconds:.0f} s "
          f"({'Combi' if combi else 'Standard'} track data). Ctrl-C to stop.\n")
    n, t_end = 0, time.time() + seconds
    last = 0.0
    while time.time() < t_end:
        m = bus.recv(timeout=max(0.0, t_end - time.time()))
        if m is None:
            break
        if m.arbitration_id != cob:
            continue
        n += 1
        r = decode_tpdo1(bytes(m.data), combi)
        if r is None:
            continue
        now = time.time()
        # 10 ms of frames is far more than anyone can read - throttle to 10 Hz.
        if now - last >= 0.1:
            print(f"    {fmt_reading(r)}")
            last = now

    if n == 0:
        print(f"    {BAD} no TPDO1 seen on 0x{cob:03X}.")
        print("      The sensor only transmits PDOs in Operational. Without --nmt it")
        print("      was probably still in Pre-operational.")
        print("      Also check 1800h:01 has not been disabled (MSB set).")
        return 1
    print(f"\n    {OK}: {n} TPDO1 frames in {seconds:.0f} s "
          f"({n / seconds:.0f}/s; 1800h:05 event timer sets the rate)")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Read the SICK MLS magnetic line sensor (read-only).")
    ap.add_argument("mode", nargs="?", default="snapshot",
                    choices=("snapshot", "poll", "stream", "calibrate", "set-variant",
                             "markers", "marker-level", "set-markers", "clear-event"))
    ap.add_argument("--node", type=int, default=SENSOR_NODE)
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="duration for poll/stream (default 10)")
    ap.add_argument("--interval", type=float, default=0.2,
                    help="poll period in seconds (default 0.2)")
    ap.add_argument("--nmt", action="store_true",
                    help="stream: send NMT Start to this node first (and Pre-operational on exit)")
    ap.add_argument("--step", default="both", choices=("both",) + CAL_STEPS + ("report",),
                    help="calibrate: which step (default both, with a prompt between)")
    ap.add_argument("--samples", type=int, default=40, help="calibrate: samples per step (default 40)")
    ap.add_argument("--value", type=int,
                    help="set-variant: the 2006h:01 value (Standard: 0, 2, 3, 4); clear-event: 0 or 1")
    ap.add_argument("--go", action="store_true",
                    help="set-variant / set-markers / clear-event: actually write")
    ap.add_argument("--polarity-lock", action="store_true",
                    help="set-markers: also 202Dh:05 = 1 (only after the B0 lap showed north everywhere)")
    ap.add_argument("--range-mm", type=float, default=200.0,
                    help="marker-level: sensor measuring range (MLSE-0200 = 200)")
    ap.add_argument("--save", help="marker-level: save the mean profile as ~/.amr/mls_cal/hall-NAME.json")
    ap.add_argument("--baseline", help="marker-level: subtract the saved profile NAME (e.g. floor)")
    args = ap.parse_args()

    if args.mode == "calibrate" and args.step == "report":
        return calibrate(None, args.node, "report", 0, 0.0)
    if args.mode in ("set-variant", "clear-event") and args.value is None:
        ap.error(f"{args.mode} needs --value")
    if args.mode in ("calibrate", "set-variant", "set-markers", "marker-level", "markers", "clear-event"):
        # SDO answers come back on one COB-ID: a second client on node 10 (drive_node's
        # IMU poll) would read the other's replies. Take the bus owner lock first.
        from agv_core import ownerlock  # noqa: PLC0415
        try:
            lock = ownerlock.acquire("can")  # noqa: F841 - held until exit
        except ownerlock.OwnerBusy as e:
            print(f"{BAD}: can0 is owned ({e}). Stop it first: sudo systemctl stop amr.service")
            return 2

    try:
        bus, how = open_bus()
    except Exception as e:
        print(f"{BAD}: {e}")
        return 2
    print(f"connected via {how}\n")

    try:
        if args.mode == "snapshot":
            return snapshot(bus, args.node)
        if args.mode == "poll":
            return poll(bus, args.node, args.seconds, args.interval)
        if args.mode == "calibrate":
            return calibrate(bus, args.node, args.step, args.samples, args.interval)
        if args.mode == "set-variant":
            return set_variant(bus, args.node, args.value, go=args.go)
        if args.mode == "set-markers":
            return set_markers(bus, args.node, go=args.go, polarity_lock=args.polarity_lock)
        if args.mode == "marker-level":
            return marker_level(bus, args.node, max(1, args.samples // 8), args.interval, args.range_mm,
                                save=args.save, baseline=args.baseline)
        if args.mode == "markers":
            return markers(bus, args.node, args.seconds, args.nmt)
        if args.mode == "clear-event":
            return clear_event(bus, args.node, args.value, go=args.go)
        return stream(bus, args.node, args.seconds, args.nmt)
    except KeyboardInterrupt:
        print("\n    interrupted")
        return 1
    finally:
        if args.mode in ("stream", "markers") and args.nmt:
            # Leave the bus as we found it: PDO traffic off.
            try:
                nmt(bus, 0x80, args.node)
                print(f"[nmt] node {args.node} -> Pre-operational")
            except Exception:
                pass
        bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
