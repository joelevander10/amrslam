#!/usr/bin/env python3
"""One AUTO run's bag -> CSVs and a timeline report (tools/run_log.sh calls it).

    python3 tools/bag_report.py ~/.amr/runs/<stamp>        (needs amr_ws sourced)

Writes <run>/csv/*.csv and prints the timeline: run states, steps and holds,
localization states, field warnings, mux source changes, drive power, panel
edges, DO changes, events and rosout warnings. Then, per step of the run
(/amr/run_state step changes): duration, distance, mean and max speed, peak
cross-track error and, when /drives/torque was recorded, the motor torque.
Then SLIP / IMPACT: hitch snatches, torque collapses and wheel-vs-gyro yaw-rate
mismatches (agv_core/slip_events.py), each with the map->odom correction and the
worst localization state that followed it.
Time is seconds from the first recorded message (bag receive time).
"""
import csv
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from agv_core import config, slip_events  # noqa: E402

RUN_STATES = {0: "IDLE", 1: "READY", 2: "EXECUTING", 3: "PAUSED", 4: "BLOCKED", 5: "FAULT", 6: "DONE"}
LOC_STATES = {0: "UNLOCALIZED", 1: "CHECKING", 2: "READY", 3: "LOST"}
MUX_SOURCES = {0: "none", 1: "teleop", 2: "follow", 3: "rotate", 4: "manual", 5: "commissioning",
               6: "pendant"}
RATED_MOTOR_NM = 1.27  # BLV motor rated torque, the 100 % of 6077h


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
        out.setdefault(topic, []).append((t_ns * 1e-9, msg))
    return out


def write_csv(d, name, header, rows):
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, name), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


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

    # -- motor torque (6077h, diagnostic): % of RATED motor torque, wheel-forward positive.
    # sum_pct is the drive force both wheels together put into the floor - the number a
    # towed load raises; v_mps is the encoder speed at the same instant, for gating on cruise.
    # *_nm is WHEEL torque: % x rated 1.27 N·m x gear ratio, no gearhead efficiency.
    nm = RATED_MOTOR_NM * config.GEAR_RATIO / 100.0
    torque = []
    for t, m in bag.get("/drives/torque", []):
        s = m.left_torque_pct + m.right_torque_pct
        torque.append((t - t0, m.left_torque_pct, m.right_torque_pct, s, at(wheels, t - t0, 1),
                       int(m.left_torque_limited), int(m.right_torque_limited),
                       m.left_valid and m.right_valid,
                       m.left_torque_pct * nm, m.right_torque_pct * nm, s * nm))
    write_csv(csvd, "torque.csv", ["t", "left_pct", "right_pct", "sum_pct", "v_mps",
                                   "left_tlc", "right_tlc", "valid",
                                   "left_nm", "right_nm", "sum_nm"], torque)

    # -- gyro --------------------------------------------------------------------------
    imu, gyaw = [], 0.0
    last = None
    for t, m in bag.get("/imu/data", []):
        if last is not None:
            gyaw += m.angular_velocity.z * (t - last)
        last = t
        imu.append((t - t0, m.angular_velocity.z, math.degrees(gyaw)))
    write_csv(csvd, "imu.csv", ["t", "yaw_rate_rad_s", "yaw_gyro_deg"], imu)

    # -- commands ----------------------------------------------------------------------
    cmd = [(t - t0, m.left_rad_s, m.right_rad_s, r * (m.left_rad_s + m.right_rad_s) / 2.0)
           for t, m in bag.get("/cmd_wheel_vel", [])]
    write_csv(csvd, "cmd_wheel_vel.csv", ["t", "left_rad_s", "right_rad_s", "v_mps"], cmd)

    # -- map->odom: AMCL's correction of the odometry ----------------------------------
    map_odom = []
    for t, m in bag.get("/tf", []):
        for tf in m.transforms:
            if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                q = tf.transform.rotation
                map_odom.append((t - t0, tf.transform.translation.x, tf.transform.translation.y,
                                 math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))))
    write_csv(csvd, "map_odom.csv", ["t", "x_m", "y_m", "yaw_rad"], map_odom)

    # -- run state ---------------------------------------------------------------------
    fields = ["state", "run_id", "mission_id", "route_id", "step_index", "step_id", "step_type",
              "step_v_mps", "cross_track_m", "hold_cause", "fault_code", "drive_power", "premove",
              "reason"]
    rows, was = [], {}
    for t, m in bag.get("/amr/run_state", []):
        row = {f: getattr(m, f, "") for f in fields}
        row["state"] = RUN_STATES.get(m.state, m.state)
        rows.append([t - t0] + [row[f] for f in fields])
        for f, label in (("state", "run"), ("step_id", "step"), ("hold_cause", "hold"),
                         ("drive_power", "drive power"), ("premove", "pre-move")):
            if row[f] != was.get(f):
                if (f in was or row[f]) and (f != "hold_cause" or row[f]):
                    ev(t, f"{label}: {row[f]!s}" + (f"  ({row['reason']})" if f == "state" else ""))
        was = row
    write_csv(csvd, "run_state.csv", ["t"] + fields, rows)

    # -- localization state ------------------------------------------------------------
    loc_fields = ["state", "operator_confirmed", "can_confirm", "cov_xx", "cov_yy", "cov_yaw",
                  "scan_match", "last_jump_m", "code", "reason"]
    loc, was = [], None
    for t, m in bag.get("/amr/localization_state", []):
        row = {f: getattr(m, f, "") for f in loc_fields}
        row["state"] = LOC_STATES.get(m.state, m.state)
        loc.append([t - t0] + [row[f] for f in loc_fields])
        if row["state"] != was:
            ev(t, f"localization: {row['state']}" + (f"  ({row['reason']})" if row["reason"] else ""))
            was = row["state"]
    write_csv(csvd, "localization_state.csv", ["t"] + loc_fields, loc)

    # -- slip / impact (agv_core/slip_events.py) ---------------------------------------
    s_torque = [(row[0], row[10], row[7]) for row in torque]
    s_cmd = [(row[0], row[3]) for row in cmd]
    s_wheels = [(row[0], row[1], row[2], row[7]) for row in wheels]
    s_gyro = [(row[0], row[1]) for row in imu]
    s_loc = [(row[0], row[1]) for row in loc]
    slips = slip_events.detect(s_torque, s_cmd, s_wheels, s_gyro, map_odom, s_loc)
    for e in slips:
        unit = "rad/s" if e.kind == "yaw" else "N·m"
        ev(e.t + t0, f"slip: {e.kind} {e.size:+.2f} {unit}")
    ix_step_id = fields.index("step_id") + 1

    def fmt(x, spec):
        return "" if x is None else format(x, spec)

    write_csv(csvd, "slip_events.csv",
              ["t", "kind", "size", "unit", "duration_s", "v_mps", "cmd_mps", "map_odom_m",
               "map_odom_deg", "loc_worst", "step"],
              [(f"{e.t:.3f}", e.kind, f"{e.size:.3f}", "rad/s" if e.kind == "yaw" else "Nm",
                f"{e.duration_s:.3f}", fmt(e.v_mps, ".3f"), fmt(e.cmd_mps, ".3f"),
                fmt(e.map_odom_m, ".3f"), fmt(e.map_odom_deg, ".2f"), e.loc_worst,
                at(rows, e.t, ix_step_id) if rows else "") for e in slips])
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


    # Steps: consecutive run_state rows with the same (state, step_id).
    print("\nSTEPS (EXECUTING)")
    segs, cur = [], None
    ix_step, ix_xt = fields.index("step_id") + 1, fields.index("cross_track_m") + 1
    for row in rows:
        key = (row[1], row[ix_step])
        if cur is None or key != cur[0]:
            if cur:
                segs.append((cur[0], cur[1], row[0]))
            cur = (key, row[0])
    if cur:
        segs.append((cur[0], cur[1], wheels[-1][0] if wheels else cur[1]))
    for (state, step), a, b in segs:
        if state != "EXECUTING" or b - a < 0.2:
            continue
        sp = [row[1] for row in wheels if a <= row[0] < b]
        xt = [row[ix_xt] for row in rows if a <= row[0] < b]
        d0, d1 = at(wheels, a, 3), at(wheels, b, 3)
        line = f"  {a:8.2f}-{b:8.2f}  {step or '-':12s} {b - a:6.1f} s"
        if d0 is not None and d1 is not None:
            line += f"  {d1 - d0:6.2f} m"
        if sp:
            line += f"  v mean {sum(sp) / len(sp):5.2f} max {max(sp):5.2f} m/s"
        if xt:
            line += f"  cross-track max {max(xt, key=abs):+.3f} m"
        tq = [row for row in torque if a <= row[0] < b and row[7]]
        if tq:
            s = [row[10] for row in tq]
            line += f"  wheel torque sum mean {sum(s) / len(s):+6.1f} max {max(s, key=abs):+6.1f} N·m"
            tlc = sum(1 for row in tq if row[5] or row[6])
            if tlc:
                line += f"  AT TORQUE LIMIT {tlc} samples"
        n_slip = sum(1 for e in slips if a <= e.t < b)
        if n_slip:
            line += f"  SLIP/IMPACT {n_slip}"
        print(line)

    # Slip and impact candidates. The thresholds are first guesses (slip_events.py), so
    # the run's largest values are listed too, whether or not they crossed them.
    se = slip_events
    print(f"\nSLIP / IMPACT  (snatch >= {se.SNATCH_NM:.0f} N·m/sample, collapse to "
          f"<= {100 * (1 - se.COLLAPSE_FRAC):.0f} % in {se.COLLAPSE_WINDOW_S:.1f} s, "
          f"yaw residual >= {se.YAW_RAD_S:.2f} rad/s; first guesses)")
    if not torque:
        print("  no /drives/torque in this bag: snatch and collapse not checked")
    base = se.map_odom_baseline(map_odom, s_wheels)
    print("  map->odom shift over 2 s while moving, p95: "
          + ("n/a" if base is None else f"{base:.3f} m") + "  (what an event's shift compares to)")
    for e in slips:
        unit = "rad/s" if e.kind == "yaw" else "N·m"
        line = (f"  {e.t:8.2f}  {e.kind:11s} {e.size:+7.2f} {unit:5s}"
                f"  v {fmt(e.v_mps, '5.2f') or '  -  '} m/s")
        if e.map_odom_m is not None:
            line += f"  map->odom {e.map_odom_m:.3f} m {e.map_odom_deg:4.1f} deg"
        if e.loc_worst:
            line += f"  loc {e.loc_worst}"
        print(line)
    if not slips:
        print("  none over the thresholds")
    jumps = se.top(se.torque_jumps(s_torque, s_cmd, s_wheels))
    print("  largest torque jumps:  "
          + (", ".join(f"{e.t:.2f}s {e.size:+.1f}" for e in jumps) or "none") + " N·m")
    resid = se.top(se.Event(t, "yaw", x) for t, x in se.yaw_residuals(s_wheels, s_gyro))
    print("  largest yaw residuals: "
          + (", ".join(f"{e.t:.2f}s {e.size:+.3f}" for e in resid) or "none") + " rad/s")
    k = se.yaw_gain(s_wheels, s_gyro)
    if k is None:
        print("  wheel/gyro yaw gain: too little turning to fit")
    elif k <= 0:
        print(f"  wheel/gyro yaw gain {k:.3f}: the gyro and the encoders disagree on the "
              "direction of turn - check drive_node gyro_sign")
    else:
        print(f"  wheel/gyro yaw gain {k:.3f}: the encoders read turns {100 * (k - 1):+.1f} %; "
              f"track width that would agree ~{config.TRACK_M * k:.3f} m "
              f"(profile {config.TRACK_M:.3f})")


if __name__ == "__main__":
    main(os.path.expanduser(sys.argv[1]) if len(sys.argv) > 1 else ".")
