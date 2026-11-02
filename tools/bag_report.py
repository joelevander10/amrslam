#!/usr/bin/env python3
"""One AUTO run's bag -> CSVs and a timeline report (tools/run_log.sh calls it).

    python3 tools/bag_report.py ~/.amr/runs/<stamp>        (needs amr_ws sourced)

Writes <run>/csv/*.csv and prints:
  - the timeline: line states, holds, RFID tags acted on, U-turn phases, speed
    zone, field warnings, mux source changes, drive power, panel edges, events;
  - per segment (between line-state/U-turn changes): duration, distance, mean and
    max speed, lateral error RMS / max and the dominant oscillation period;
  - per U-turn: approach length, overrun past the tape end, pivot angle by the
    encoders and by the gyro, how long it took to centre and the error it ended on;
  - per RFID tag pass (rfid_node's "RFID pass" log lines): reads, duration, speed,
    read-zone length and RSSI - the read margin behind a missed tag;
  - per MLS marker (/amr/line_marker events): code, track position at the read,
    then the count per code and the stream's last status, rejects and aborts.
Time is seconds from the first recorded message (bag receive time).
"""
import csv
import math
import os
import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from agv_core import config  # noqa: E402

LINE_STATES = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "HOLD", 4: "DONE", 5: "FAULT"}
# agv_core.drivers.rfid.pass_summary(); change both together.
PASS_RE = re.compile(r"RFID pass (\w+): (\d+) reads in ([\d.]+) s, RSSI peak (-?[\d.]+) "
                     r"mean (-?[\d.]+) min (-?[\d.]+) dBm")
# A pass with this few reads was nearly a missed tag.
MARGINAL_READS = 2
# line_track.csv's lateral_mm column; the marker columns come after it. (Until 2026-10-09 the
# U-turn report read column 11 here, one past the end of the row: an IndexError.)
LAT = 10
MUX_SOURCES = {0: "none", 1: "teleop", 2: "follow", 3: "rotate", 4: "manual", 5: "commissioning",
               6: "pendant", 7: "line"}


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        try:
            msg = deserialize_message(data, get_message(types[topic]))
        except Exception:  # noqa: BLE001 - an unknown type is skipped, not fatal
            continue
        key = "/amr/line_cmd" if topic.endswith("/amr/line_cmd") else topic
        out.setdefault(key, []).append((t_ns * 1e-9, msg))
    return out


def write_csv(d, name, header, rows):
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, name), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def lateral(track):
    """The followed track's offset (LCP2 when valid, else the nearest valid one), mm."""
    # int(): the message arrays are numpy int16, whose squares overflow in the RMS.
    vals = [int(track.lcp_mm[i]) for i in range(3) if track.valid[i]]
    if not vals:
        return None
    return int(track.lcp_mm[1]) if track.valid[1] else min(vals, key=abs)


def osc_period(ts, xs):
    """Mean period from zero crossings of the de-meaned signal, s (None if < 3)."""
    if len(xs) < 10:
        return None
    m = sum(xs) / len(xs)
    cross = [ts[i] for i in range(1, len(xs)) if (xs[i - 1] - m) * (xs[i] - m) < 0]
    if len(cross) < 3:
        return None
    return 2.0 * (cross[-1] - cross[0]) / (len(cross) - 1)


def main(run):
    bag = read_bag(os.path.join(run, "bag"))
    if not bag:
        print("empty bag")
        return
    t0 = min(v[0][0] for v in bag.values())
    csvd = os.path.join(run, "csv")
    r = config.WHEEL_DIA_M / 2.0
    timeline = []

    def ev(t, what):
        timeline.append((t - t0, what))

    # -- wheels: body speed, yaw rate, distance, heading by the encoders ---------------
    wheels, dist, yaw = [], 0.0, 0.0
    prev = None
    for t, m in bag.get("/wheel_states", []):
        v = r * (m.left_vel_rad_s + m.right_vel_rad_s) / 2.0
        w = r * (m.right_vel_rad_s - m.left_vel_rad_s) / config.TRACK_M
        if prev is not None and m.left_valid and m.right_valid:
            dl = (m.left_pos_rad - prev.left_pos_rad) * r
            dr = (m.right_pos_rad - prev.right_pos_rad) * r
            dist += abs(dl + dr) / 2.0
            yaw += (dr - dl) / config.TRACK_M
        prev = m
        wheels.append((t - t0, v, w, dist, math.degrees(yaw), m.left_counts, m.right_counts,
                       m.left_valid and m.right_valid))
    write_csv(csvd, "wheels.csv", ["t", "v_mps", "yaw_rate_rad_s", "dist_m", "yaw_enc_deg",
                                   "left_counts", "right_counts", "valid"], wheels)

    def at(series, t, col):
        """Value of series[col] at time t (series rows start with relative t)."""
        lo, hi = 0, len(series) - 1
        if hi < 0:
            return None
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if series[mid][0] <= t:
                lo = mid
            else:
                hi = mid - 1
        return series[lo][col]

    # -- gyro --------------------------------------------------------------------------
    imu, gyaw = [], 0.0
    last = None
    for t, m in bag.get("/imu/data", []):
        if last is not None:
            gyaw += m.angular_velocity.z * (t - last)
        last = t
        imu.append((t - t0, m.angular_velocity.z, math.degrees(gyaw)))
    write_csv(csvd, "imu.csv", ["t", "yaw_rate_rad_s", "yaw_gyro_deg"], imu)

    # -- tape --------------------------------------------------------------------------
    track = [(t - t0, *m.lcp_mm, *[int(x) for x in m.valid], m.nlcp, m.track_level, int(m.line_good),
              lateral(m), m.marker, int(m.marker_intro)) for t, m in bag.get("/amr/line_track", [])]
    write_csv(csvd, "line_track.csv", ["t", "lcp1", "lcp2", "lcp3", "v1", "v2", "v3", "nlcp", "level",
                                       "line_good", "lateral_mm", "marker", "marker_intro"], track)

    # -- commands ----------------------------------------------------------------------
    write_csv(csvd, "line_cmd.csv", ["t", "v_mps", "w_rad_s"],
              [(t - t0, m.linear.x, m.angular.z) for t, m in bag.get("/amr/line_cmd", [])])
    write_csv(csvd, "cmd_wheel_vel.csv", ["t", "left_rad_s", "right_rad_s"],
              [(t - t0, m.left_rad_s, m.right_rad_s) for t, m in bag.get("/cmd_wheel_vel", [])])

    # -- line state --------------------------------------------------------------------
    fields = ["state", "hold_cause", "engine_state", "error_mm", "v_mps", "followed_m", "mission",
              "destination", "station", "stop_where", "uturn_phase", "last_tag", "last_tag_action",
              "high_speed", "speed_reason", "at_home", "job_result", "drive_power", "message"]
    rows, was = [], {}
    for t, m in bag.get("/amr/line_state", []):
        row = {f: getattr(m, f, "") for f in fields}
        row["state"] = LINE_STATES.get(m.state, m.state)
        rows.append([t - t0] + [row[f] for f in fields])
        for f, label in (("state", "line"), ("hold_cause", "hold"), ("uturn_phase", "U-turn"),
                         ("stop_where", "stop"), ("high_speed", "HIGH"), ("drive_power", "drive power")):
            if row[f] != was.get(f):
                if (f in was or row[f]) and (f != "hold_cause" or row[f]):
                    ev(t, f"{label}: {row[f]!s}" + (f"  ({row['message']})" if f == "state" else ""))
        if row["last_tag"] and (row["last_tag"], row["last_tag_action"]) != (was.get("last_tag"),
                                                                             was.get("last_tag_action")):
            ev(t, f"tag {row['last_tag']}: {row['last_tag_action']}")
        was = row
    write_csv(csvd, "line_state.csv", ["t"] + fields, rows)

    # -- RFID encounters ---------------------------------------------------------------
    rf = [(t - t0, m.rfid_tag, m.encounter_seq, m.rssi_dbm, int(m.comms_ok))
          for t, m in bag.get("/amr/rfid", []) if not m.heartbeat]
    write_csv(csvd, "rfid.csv", ["t", "tag", "encounter_seq", "rssi_dbm", "comms_ok"], rf)
    last_d = {}
    for t, tag, seq, rssi, _ in rf:
        d = at(wheels, t, 3)
        gap = ""
        if d is not None and last_d:
            prev_tag, prev_d = last_d["tag"], last_d["d"]
            gap = f", {d - prev_d:.2f} m after {prev_tag}"
        timeline.append((t, f"RFID read {tag} (#{seq}, {rssi:.1f} dBm"
                            + (f", odo {d:.2f} m" if d is not None else "") + gap + ")"))
        if d is not None:
            last_d = {"tag": tag, "d": d}

    # -- MLS markers: events, and the last heartbeat's status --------------------------
    mk_msgs = bag.get("/amr/line_marker", [])
    mk = [(t - t0, m.seq, m.generation, m.code, m.raw, m.direction, m.lcp2_mm, m.nlcp, int(m.line_good))
          for t, m in mk_msgs if not m.heartbeat]
    write_csv(csvd, "line_marker.csv", ["t", "seq", "generation", "code", "raw", "direction", "lcp2_mm",
                                        "nlcp", "line_good"], mk)
    mk_beat = next((m for _, m in reversed(mk_msgs) if m.heartbeat), None)
    for t, seq, gen, code, _raw, _dir, lcp2, _nlcp, good in mk:
        timeline.append((t, f"MARKER {code} (#{seq} gen {gen}, LCP2 {lcp2:+d} mm"
                            + ("" if good else ", line not good") + ")"))

    # -- mux, drives, panel, I/O -------------------------------------------------------
    mux, was = [], None
    for t, m in bag.get("/amr/mux_state", []):
        src = MUX_SOURCES.get(m.source, m.source)
        mux.append((t - t0, src, int(m.inhibited), m.warning_level, m.speed_scale, int(m.protective_clear),
                    m.code, m.reason))
        key = (src, m.warning_level, int(m.protective_clear))
        if key != was:
            ev(t, f"mux: source {src}, warning level {m.warning_level}, protective "
                  f"{'clear' if m.protective_clear else 'VIOLATED'}, scale {m.speed_scale:.2f}")
            was = key
    write_csv(csvd, "mux.csv", ["t", "source", "inhibited", "warning_level", "speed_scale",
                                "protective_clear", "code", "reason"], mux)
    drv, was = [], None
    for t, m in bag.get("/drives/status", []):
        link, pw, why = getattr(m, "link_state", ""), getattr(m, "power_wanted", None), \
            getattr(m, "power_reason", "")
        drv.append((t - t0, int(m.operational), link, pw, why, m.left_state, m.right_state,
                    hex(m.left_error_code), hex(m.right_error_code)))
        key = (m.operational, link, pw)
        if key != was:
            ev(t, f"drives: operational={m.operational} link={link} power_wanted={pw} ({why})")
            was = key
    write_csv(csvd, "drives.csv", ["t", "operational", "link_state", "power_wanted", "power_reason",
                                   "left_state", "right_state", "left_err", "right_err"], drv)
    for t, m in bag.get("/amr/panel_state", []):
        if m.start_edge:
            ev(t, "PANEL Start")
        if m.reset_edge:
            ev(t, "PANEL Reset")
    io, was = [], None
    for t, m in bag.get("/amr/io", []):
        idx = [str(i) for i, b in enumerate(m.do_requested) if b]
        io.append((t - t0, " ".join(idx), " ".join(str(i) for i, b in enumerate(m.di) if b)))
        if idx != was:
            ev(t, f"DO on: {', '.join(f'DO{int(i):02d}' for i in idx) or 'none'}")
            was = idx
    write_csv(csvd, "io.csv", ["t", "do_on", "di_on"], io)
    for t, m in bag.get("/amr/events", []):
        ev(t, f"event [{m.source}] {m.code}: {m.text}")
    for t, m in bag.get("/rosout", []):
        if m.level >= 30:  # WARN and above
            ev(t, f"rosout {m.name}: {m.msg[:160]}")

    # -- report ------------------------------------------------------------------------
    print(f"run {run}")
    print(f"  {(max(v[-1][0] for v in bag.values()) - t0):.1f} s recorded, "
          f"{', '.join(f'{k} {len(v)}' for k, v in sorted(bag.items()))}")
    print("\nTIMELINE")
    for t, what in sorted(timeline):
        print(f"  {t:8.2f}  {what}")

    print("\nRFID PASSES (a pass ends tag_clear_s after its last read; logged when it ends)")
    passes = []
    for t, m in bag.get("/rosout", []):
        g = PASS_RE.search(m.msg)
        if not g:
            continue
        reads, dur = int(g[2]), float(g[3])
        end = t - t0
        mid = end - config.RFID_TAG_CLEAR_S - dur / 2.0   # logged clear_s after the last read
        v = at(wheels, mid, 1)
        passes.append((round(mid, 2), g[1], reads, dur, v, None if v is None else abs(v) * dur,
                       float(g[4]), float(g[5]), float(g[6])))
    write_csv(csvd, "rfid_passes.csv", ["t_mid", "tag", "reads", "duration_s", "v_mps", "zone_m",
                                        "rssi_peak", "rssi_mean", "rssi_min"], passes)
    if not passes:
        print("  none logged (rfid_node older than 2026-10-09, or no tag passed)")
    for mid, tag, reads, dur, v, zone, peak, mean, low in passes:
        moving = v is not None and abs(v) > 0.05
        flag = "  MARGINAL" if moving and reads <= MARGINAL_READS else ""
        print(f"  {mid:8.2f}  {tag}  {reads:4d} reads  {dur:5.2f} s"
              + (f"  {abs(v):.2f} m/s  zone {zone:.2f} m" if moving else "  standing      ")
              + f"  rate {reads / dur if dur > 0 else 0:5.1f}/s"
              + f"  RSSI {peak:.0f}/{mean:.0f}/{low:.0f} dBm{flag}")
    moving = [p for p in passes if p[4] is not None and abs(p[4]) > 0.05]
    if moving:
        n = sorted(p[2] for p in moving)
        print(f"  moving passes {len(moving)}: reads min {n[0]} median {n[len(n) // 2]}, "
              f"{sum(1 for x in n if x <= MARGINAL_READS)} marginal (<= {MARGINAL_READS} reads)")

    print("\nMLS MARKERS")
    if not mk_msgs:
        print("  not recorded (no /amr/line_marker: tools/run_log.sh TOPICS, or drive_node older "
              "than 2026-10-09)")
    for t, _seq, _gen, code, raw, _direction, lcp2, nlcp, good in mk:
        print(f"  {t:8.2f}  code {code}  raw {raw:05b}  LCP2 {lcp2:+4d} mm  nlcp {nlcp}"
              + ("" if good else "  line not good"))
    if mk_msgs:
        counts = {}
        for row in mk:
            counts[row[3]] = counts.get(row[3], 0) + 1
        print(f"  events {len(mk)}" + (": " + ", ".join(f"code {c} x{n}" for c, n in sorted(counts.items()))
                                       if counts else ""))
        if mk_beat is not None:
            print(f"  last status: {mk_beat.status} (markers_ok {mk_beat.markers_ok}), "
                  f"rejects {mk_beat.rejects}, aborts {mk_beat.aborts}")

    # Segments: consecutive line_state rows with the same (state, uturn_phase).
    print("\nSEGMENTS (RUNNING, by U-turn phase)")
    segs, cur = [], None
    for row in rows:
        key = (row[1], row[fields.index("uturn_phase") + 1])
        if cur is None or key != cur[0]:
            if cur:
                segs.append((cur[0], cur[1], row[0]))
            cur = (key, row[0])
    if cur:
        segs.append((cur[0], cur[1], wheels[-1][0] if wheels else cur[1]))
    for (state, phase), a, b in segs:
        if state != "RUNNING" or b - a < 0.2:
            continue
        lat = [(row[0], row[LAT]) for row in track if a <= row[0] < b and row[LAT] is not None]
        sp = [row[1] for row in wheels if a <= row[0] < b]
        d0, d1 = at(wheels, a, 3), at(wheels, b, 3)
        line = f"  {a:8.2f}-{b:8.2f}  {phase or 'follow':9s} {b - a:6.1f} s"
        if d0 is not None and d1 is not None:
            line += f"  {d1 - d0:6.2f} m"
        if sp:
            line += f"  v mean {sum(sp) / len(sp):5.2f} max {max(sp):5.2f} m/s"
        if lat:
            xs = [x for _, x in lat]
            rms = math.sqrt(sum(x * x for x in xs) / len(xs))
            per = osc_period([t for t, _ in lat], xs)
            line += f"  lateral rms {rms:5.1f} max {max(xs, key=abs):+5d} mm"
            line += f"  osc {per:.2f} s" if per else ""
        print(line)

    # U-turns: from the U-turn tag (approach) to the end of settle.
    print("\nU-TURNS")
    i = 0
    phases = [(row[0], row[fields.index("uturn_phase") + 1]) for row in rows]
    while i < len(phases):
        if phases[i][1] != "approach":
            i += 1
            continue
        start = phases[i][0]
        marks = {}
        j = i
        while j < len(phases) and phases[j][1]:
            marks.setdefault(phases[j][1], phases[j][0])
            j += 1
        end = phases[j][0] if j < len(phases) else (wheels[-1][0] if wheels else start)
        spin = marks.get("spin")
        print(f"  tag at {start:.2f} s; phases " + ", ".join(f"{p} {t:.2f}" for p, t in marks.items())
              + f"; ended {end:.2f} s")
        if "stopping" in marks:
            a = at(wheels, start, 3)
            b = at(wheels, marks["stopping"], 3)
            c = at(wheels, spin, 3) if spin else None
            if a is not None and b is not None:
                print(f"    approach {b - a:.2f} m (tag -> tape gone)"
                      + (f", overrun {c - b:.3f} m after the zero command" if c is not None else ""))
        if spin:
            y0, y1 = at(wheels, spin, 4), at(wheels, end, 4)
            g0, g1 = at(imu, spin, 2), at(imu, end, 2)
            if y0 is not None and y1 is not None:
                print(f"    pivot by encoders {y1 - y0:+.1f} deg"
                      + (f", by gyro {g1 - g0:+.1f} deg" if g0 is not None and g1 is not None else "")
                      + f", {end - spin:.1f} s")
            seen = [row for row in track if spin <= row[0] <= end and row[LAT] is not None]
            if seen:
                print(f"    tape back at {seen[0][0]:.2f} s ({seen[0][LAT]:+d} mm, "
                      f"encoder {at(wheels, seen[0][0], 4) - y0:+.1f} deg); ended on "
                      f"{at(track, end, LAT)} mm")
        i = j


if __name__ == "__main__":
    main(os.path.expanduser(sys.argv[1]) if len(sys.argv) > 1 else ".")
