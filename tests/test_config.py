"""The vehicle profile: loading, validation, and the derived constants."""
import math
import os

from helpers import check

from agv_core import config, kinematics


def rows_by_path(sections):
    """describe() rows addressed by their JSON path, not by where they display.

    describe() gathers the speed keys out of three sections into one at the top
    of the page, so a row's SECTION is a presentation choice while its path is
    the fact. Keying on the path lets these tests assert what they actually mean
    - this value reaches the page, with its note attached - without pinning the
    layout, which is free to change again.

    A gathered row already carries its full path; a row shown in its own section
    carries a bare key and gets the section prefixed back on.
    """
    out = {}
    for s in sections:
        for r in s["rows"]:
            key = r["key"]
            out[key if key.split(".")[0] in config._SCHEMA
                else f"{s['name']}.{key}"] = r
    return out


def test_config_profile():
    """The profile is the whole tuning surface, so a bad one must be refused
    loudly at boot rather than showing up as odd behaviour halfway through a run."""
    import copy
    import json
    import shutil
    import tempfile
    print("\nvehicle profile loading")

    base = json.load(open(config.profile_path()))

    def load_with(mutate, name="agv-01"):
        d = copy.deepcopy(base)
        mutate(d)
        # Written under its profile NAME, not a random temp name: the loader
        # requires profile_name to match the filename, so a tmpXXXX.json would
        # be refused for the wrong reason and every check below would pass
        # vacuously.
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, f"{name}.json")
        with open(path, "w") as fh:
            json.dump(d, fh)
        try:
            config.load(path)
            return None
        except config.ConfigError as e:
            return str(e)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
            config.load()          # always restore the real profile

    def refuses(name, mutate, expect=""):
        msg = load_with(mutate)
        check(name, msg is not None and expect in (msg or ""), msg or "accepted!")

    check("the real profile loads", config.PROFILE_NAME == "agv-01",
          config.PROFILE_NAME)
    refuses("a typo'd key is refused",
            lambda d: d["vehicle"].update({"track_mm": d["vehicle"].pop("track_m")}),
            "unknown key")
    refuses("a missing key is refused",
            lambda d: d["vehicle"].pop("gear_ratio"), "missing key")
    refuses("an unknown section is refused",
            lambda d: d.update({"extra": {}}), "unknown top-level")
    # *** A whole retired section must not come back by accident. *** The tracked
    # product was removed on 2026-10-27 (manuals/trackless-only-removal-plan.md):
    # a profile still carrying its sections or keys is a profile from before the
    # removal, and the strict loader refuses it rather than loading values
    # nothing reads.
    for section in ("autopilot", "rfid", "mls", "tape_loc"):
        refuses(f"a leftover {section} section is refused",
                lambda d, sec=section: d.update({sec: {}}), "unknown top-level")
    refuses("a leftover tracked key is refused",
            lambda d: d.update(tracked=None), "unknown top-level")
    refuses("a leftover vehicle.sensor_lookahead_m is refused",
            lambda d: d["vehicle"].update(sensor_lookahead_m=0.1), "unknown key")
    refuses("a leftover branch_latch table is refused",
            lambda d: d.update({"branch_latch": []}), "unknown top-level")
    refuses("a leftover stop_until_start_button table is refused",
            lambda d: d.update({"stop_until_start_button": []}),
            "unknown top-level")
    refuses("a leftover auto watchdog is refused",
            lambda d: d["timing"].update(auto_watchdog_s=1.5), "unknown key")
    refuses("a blind-run speed above the motor limit is refused",
            lambda d: d["blind_run"].update(max_rpm=9000), "blind_run.max_rpm")
    refuses("a blind-run speed above 0.8 m/s is refused",
            lambda d: d["blind_run"].update(max_speed_mps=1.0), "blind_run.max_speed_mps")
    # 2026-10-30: pp.* went with profile position (blind_run.* came back 2026-11-02, PV only).
    refuses("the retired pp section is refused",
            lambda d: d.update({"pp": {"enabled": False}}), "unknown top-level")
    refuses("a bool where a number belongs is refused",
            lambda d: d["vehicle"].update(gear_ratio=True), "expected a number")
    # 2026-10-02: manual.* (read only by the deleted motion.py) and drivers.ramp.manual
    # (drive_node always arms with "auto") were removed as dead speed parameters.
    refuses("the retired manual section is refused",
            lambda d: d.update({"manual": {"full_rpm": 1200, "half_ratio": 0.6}}),
            "unknown top-level")
    # *** R16: nonfinite numbers. *** A positive-only check passes infinity, so
    # an infinite manual watchdog would never expire. Python's json reader takes
    # the non-standard Infinity/NaN tokens, and 1e999 reads as inf, so each form
    # is written as raw text and must be refused with the FIELD named.
    def refuses_raw(name, token, expect):
        def mutate(d):
            d["timing"]["manual_watchdog_s"] = "@@TOKEN@@"
        d = copy.deepcopy(base)
        mutate(d)
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, "agv-01.json")
        with open(path, "w") as fh:
            fh.write(json.dumps(d).replace('"@@TOKEN@@"', token))
        try:
            config.load(path)
            msg = None
        except config.ConfigError as e:
            msg = str(e)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
            config.load()
        check(name, msg is not None and expect in msg, msg or "accepted!")

    for token in ("Infinity", "-Infinity", "NaN", "1e999", "-1e999"):
        refuses_raw(f"a watchdog of {token} is refused, naming the field",
                    token, "timing.manual_watchdog_s")
    refuses_raw("an integer too large for a float is refused, naming the field",
                "1" + "0" * 400, "timing.manual_watchdog_s")
    refuses("a finite wheel diameter that overflows a derived value is refused",
            lambda d: d["vehicle"].update(wheel_dia_m=1e308, gear_ratio=1e-300),
            "not finite")
    check("the live watchdog is finite after the refusals",
          math.isfinite(config.MANUAL_WATCHDOG_S), config.MANUAL_WATCHDOG_S)

    refuses("duplicate CAN node IDs are refused",
            lambda d: d["can"].update(sensor_node=2), "distinct")
    refuses("a non-boolean can.use_rpdo is refused",
            lambda d: d["can"].update(use_rpdo=1), "true/false")

    refuses("a driver timeout inside the telemetry period is refused",
            lambda d: d["timing"].update(driver_timeout_s=0.1),
            "driver_timeout_s")
    refuses("an empty can.channel is refused",
            lambda d: d["can"].update(channel=""), "can.channel")

    # The horn is the only output this software energises, and both ways of
    # mis-wiring it in the profile are refused: a channel the module does not
    # have would write nothing, and a horn enabled without the DIO scan has
    # nobody to write it.
    # MCQ4-4 (operator, 2026-10-27): DO00 rotary lamp and DO01 horn follow the motors in any
    # mode; DO02 is the alarm horn (pre-move warning, AUTO field alarm), and DO01 gives way to it.
    check("DO00 and DO01 follow the motors", config.DIO_MOTION_ON == [0, 1], str(config.DIO_MOTION_ON))
    check("DO02 is the alarm horn", config.DIO_ALARM_ON == [2], str(config.DIO_ALARM_ON))
    check("DO01 is the movement horn", config.DIO_MOVEMENT_HORN == [1], str(config.DIO_MOVEMENT_HORN))
    # Drive power (2026-10-27): the drives are powered only while DI09 (pendant enable) is HIGH.
    check("DI09 is the arm input", config.PANEL_DI_MANUAL_ARM == 9, str(config.PANEL_DI_MANUAL_ARM))
    refuses("a Manual Arm input shared with a panel button is refused",
            lambda d: d["panel"].update(di_manual_arm=d["panel"]["di_start"]), "distinct")
    refuses("a Manual Arm input on a pendant channel is refused",
            lambda d: d["panel"].update(di_manual_arm=d["pendant"]["di_fwd"]), "pendant channel")
    # Emergency suite (2026-10-30): DI08 is the Flexi Soft's Q3, E-stop OK, HIGH = no emergency.
    check("DI08 is the E-stop OK input", config.PANEL_DI_ESTOP_OK == 8, str(config.PANEL_DI_ESTOP_OK))
    refuses("an E-stop input shared with a panel button is refused",
            lambda d: d["panel"].update(di_estop_ok=d["panel"]["di_reset"]), "distinct")
    refuses("an E-stop input on the arm channel is refused",
            lambda d: d["panel"].update(di_estop_ok=d["panel"]["di_manual_arm"]), "distinct")
    refuses("an E-stop input on a pendant channel is refused",
            lambda d: d["panel"].update(di_estop_ok=d["pendant"]["di_rvs"]), "pendant channel")
    refuses("an E-stop input the module does not have is refused",
            lambda d: d["panel"].update(di_estop_ok=99), "di_estop_ok")
    refuses("a reset timeout under 0.5 s is refused",
            lambda d: d["panel"].update(estop_reset_timeout_s=0.2), "estop_reset_timeout_s")
    refuses("a reset timeout over 30 s is refused",
            lambda d: d["panel"].update(estop_reset_timeout_s=31.0), "estop_reset_timeout_s")
    # Route wait steps (2026-10-30, gls-wait-coding-plan §A.2): DI0D is PB Continue.
    check("DI13 is PB Continue", config.PANEL_DI_CONTINUE == 13, str(config.PANEL_DI_CONTINUE))
    refuses("a Continue input shared with Start is refused",
            lambda d: d["panel"].update(di_continue=d["panel"]["di_start"]), "distinct")
    refuses("a Continue input on the E-stop channel is refused",
            lambda d: d["panel"].update(di_continue=d["panel"]["di_estop_ok"]), "distinct")
    refuses("a Continue input on a pendant channel is refused",
            lambda d: d["panel"].update(di_continue=d["pendant"]["di_left"]), "pendant channel")
    refuses("a Continue input the module does not have is refused",
            lambda d: d["panel"].update(di_continue=16), "di_continue")
    # Destination buttons (owner 2026-11-03): DI10 UBC, DI11 ITG, DI12 PISTON.
    check("DI10-12 are the destination buttons",
          [(n, ch) for n, ch, _r in config.PANEL_DESTINATIONS] == [("UBC", 10), ("ITG", 11), ("PISTON", 12)],
          str(config.PANEL_DESTINATIONS))

    def dest(i, **kw):
        return lambda d: d["panel"]["destinations"][i].update(**kw)
    refuses("a destination button shared with Continue is refused",
            dest(0, di=13), "distinct")
    refuses("two destinations on one channel are refused", dest(1, di=10), "distinct")
    refuses("a destination button on a pendant channel is refused",
            dest(2, di=base["pendant"]["di_fwd"]), "pendant channel")
    refuses("a destination button the module does not have is refused", dest(0, di=16), "di (16)")
    refuses("two destinations with one name are refused", dest(1, name="ubc"), "share a name")
    refuses("an empty destination name is refused", dest(1, name=" "), "must not be empty")
    refuses("a route id with a slash is refused", dest(1, route_id="../x"), "plain route id")
    refuses("an unknown destination key is refused", dest(1, mission_id="x"), "exactly the keys")
    refuses("a missing panel.destinations is refused",
            lambda d: d["panel"].pop("destinations"), "destinations")
    refuses("the retired panel.manual_auto_arm is refused",
            lambda d: d["panel"].update(manual_auto_arm=True), "unknown key")
    refuses("a pre-move timeout not above the warning is refused",
            lambda d: d["timing"].update(premove_timeout_s=d["timing"]["auto_start_delay_s"]),
            "premove_timeout_s")
    refuses("a movement horn that is not a motion output is refused",
            lambda d: d["dio"].update(movement_horn=[2]), "among dio.motion_on")
    refuses("one channel as both a motion and an alarm output is refused",
            lambda d: d["dio"].update(motion_on=[0, 1], alarm_on=[1]), "share a channel")
    refuses("a motion channel past the end of the module is refused",
            lambda d: d["dio"].update(motion_on=[0, d["dio"]["num_do"]]), "dio.motion_on")
    refuses("a motion channel listed twice is refused",
            lambda d: d["dio"].update(motion_on=[8, 8]), "listed twice")
    refuses("the horn's channel is refused as a motion channel while the horn is enabled",
            lambda d: (d["horn"].update(enabled=True, do_channel=0), d["dio"].update(motion_on=[0])),
            "horn's channel")
    refuses("a horn channel past the end of the module is refused",
            lambda d: d["horn"].update(do_channel=d["dio"]["num_do"]),
            "horn.do_channel")
    refuses("a negative horn channel is refused",
            lambda d: d["horn"].update(do_channel=-1), "horn.do_channel")
    refuses("a horn without the DIO scan that writes it is refused",
            lambda d: (d["dio"].update(enabled=False),
                       d["panel"].update(enabled=False),
                       d["pendant"].update(enabled=False),
                       d["horn"].update(enabled=True, do_channel=5)),
            "horn.enabled")
    check("HORN_HOLD_S outlasts both the tick and the DIO scan",
          config.HORN_HOLD_S >= 5 * config.LOOP_PERIOD_S
          and config.HORN_HOLD_S >= 2 * config.DIO_SCAN_PERIOD_S,
          f"HORN_HOLD_S={config.HORN_HOLD_S}")

    # The IMU poll runs on the bus thread, so it is paced like every other
    # poll there, and its back-off has to be a back-off.
    refuses("an IMU poll faster than the tick is refused",
            lambda d: d["imu"].update(poll_period_s=d["timing"]["loop_period_s"] / 2),
            "imu.poll_period_s")
    refuses("an IMU retry no slower than the poll is refused",
            lambda d: d["imu"].update(retry_period_s=d["imu"]["poll_period_s"]),
            "imu.retry_period_s")

    # -- the GLS621 floor-label camera (gls-wait-coding-plan §B.3, P2) -------
    check("the GLS camera is enabled", config.GLS_ENABLED is True, str(config.GLS_ENABLED))
    check("the GLS scale ships as 0 = not measured: raw counts only",
          config.GLS_XY_SCALE_M_PER_COUNT == 0.0, str(config.GLS_XY_SCALE_M_PER_COUNT))
    check("the expected template outputs the label centre ([$XMCL], [$YMCL])",
          "[$XMCL]" in config.GLS_EXPECT_TEMPLATE and "[$YMCL]" in config.GLS_EXPECT_TEMPLATE,
          config.GLS_EXPECT_TEMPLATE)

    def gls(**kw):
        return lambda d: d["gls"].update(kw)

    refuses("a profile without the gls section is refused",
            lambda d: d.pop("gls"), "missing top-level")
    refuses("a non-boolean gls.enabled is refused", gls(enabled=1), "gls.enabled")
    refuses("an empty gls.host is refused", gls(host="  "), "gls.host")
    for port in (0, 65536):
        refuses(f"gls.port {port} is refused", gls(port=port), "gls.port")
    for key in ("connect_timeout_s", "keepalive_s", "verify_timeout_s", "frame_gap_s", "stale_s"):
        refuses(f"gls.{key} of 0 is refused", gls(**{key: 0.0}), f"gls.{key} must be > 0")
    refuses("a frame gap over 0.5 s is refused", gls(frame_gap_s=0.6, stale_s=1.0),
            "frame_gap_s must be at most")
    refuses("a stale time not above the frame gap is refused", gls(frame_gap_s=0.2, stale_s=0.2),
            "must exceed frame_gap_s")
    for marker in ("[$XMCL]", "[$YMCL]"):
        refuses(f"a template without {marker} is refused",
                lambda d, m=marker: d["gls"].update(
                    expect_template=d["gls"]["expect_template"].replace(m, "[$XCL]")),
                "gls.expect_template")
    for n in (0, 17):
        refuses(f"gls.min_codes {n} is refused", gls(min_codes=n), "gls.min_codes")
    refuses("a negative gls.outlier_m is refused", gls(outlier_m=-0.001), "gls.outlier_m")
    for scale in (-1e-6, 0.0101):
        refuses(f"gls.xy_scale_m_per_count {scale} is refused", gls(xy_scale_m_per_count=scale),
                "gls.xy_scale_m_per_count")
    for key, bad in (("x_sign", 0), ("y_sign", 2), ("angle_sign", -2)):
        refuses(f"gls.{key} {bad} is refused", gls(**{key: bad}), f"gls.{key} must be 1 or -1")
    for key, bad in (("angle_offset_rad", 3.15), ("mount_yaw_rad", -3.15)):
        refuses(f"gls.{key} beyond pi is refused", gls(**{key: bad}), f"gls.{key}")
    for key, bad in (("mount_x_m", 2.01), ("mount_y_m", -2.01)):
        refuses(f"gls.{key} beyond 2 m is refused", gls(**{key: bad}), f"gls.{key}")
    msg = load_with(gls(frame_gap_s=0.5, stale_s=0.51, min_codes=16, xy_scale_m_per_count=0.01,
                        x_sign=-1, y_sign=-1, angle_sign=-1, angle_offset_rad=-math.pi,
                        mount_yaw_rad=math.pi, mount_x_m=-2.0, mount_y_m=2.0, port=65535))
    check("the gls bounds themselves are accepted", msg is None, msg or "")
    check("gls.swap_xy ships true: the device's Y is the vehicle's fore-aft (bench 2026-10-30)",
          config.GLS_SWAP_XY is True, str(config.GLS_SWAP_XY))
    refuses("a non-boolean gls.swap_xy is refused", gls(swap_xy=1), "gls.swap_xy")
    # -- one label observation (gls-wait-coding-plan §C.2, P3) ---------------
    check("the label-observation keys ship as 15 frames / 2 mm / 0.009 rad / 1 s",
          (config.GLS_FIX_FRAMES, config.GLS_FIX_SPREAD_XY_M, config.GLS_FIX_SPREAD_YAW_RAD,
           config.GLS_STATIONARY_S) == (15, 0.002, 0.009, 1.0),
          str((config.GLS_FIX_FRAMES, config.GLS_FIX_SPREAD_XY_M, config.GLS_FIX_SPREAD_YAW_RAD,
               config.GLS_STATIONARY_S)))
    for key, bad in (("fix_frames", 0), ("fix_frames", 65), ("fix_spread_xy_m", 0.0),
                     ("fix_spread_xy_m", 0.0501), ("fix_spread_yaw_rad", 0.0),
                     ("fix_spread_yaw_rad", 0.201), ("stationary_s", 0.0), ("stationary_s", 10.01)):
        refuses(f"gls.{key} {bad} is refused", gls(**{key: bad}), f"gls.{key} must be in")
    refuses("a fractional gls.fix_frames is refused", gls(fix_frames=15.5), "gls.fix_frames")
    msg = load_with(gls(fix_frames=64, fix_spread_xy_m=0.05, fix_spread_yaw_rad=0.2, stationary_s=10.0))
    msg2 = load_with(gls(fix_frames=1))
    check("the label-observation bounds themselves are accepted", msg is None and msg2 is None,
          msg or msg2 or "")

    # The name in the file must match the file. A profile copied for a second
    # vehicle and not renamed would report the old identity in the event log
    # and in every run CSV header.
    msg = load_with(lambda d: None, name="agv-99")
    check("profile_name must match the filename", msg is not None
          and "agv-99" in (msg or ""), msg or "accepted!")

    # -- which vehicle this process is --------------------------------------
    check("the default profile resolves into profiles/",
          config.profile_path().endswith(os.path.join("profiles", "agv-01.json")),
          config.profile_path())
    os.environ[config.PROFILE_ENV_VAR] = "agv-02"
    try:
        check("AGV_PROFILE selects the profile",
              config.profile_path().endswith("agv-02.json"),
              config.profile_path())
        check("an explicit path still wins over the env var",
              config.profile_path("agv-03").endswith("agv-03.json"))
        try:
            config.load()
            check("a missing profile is fatal", False, "accepted!")
        except config.ConfigError as e:
            check("a missing profile names the env var and what exists",
                  config.PROFILE_ENV_VAR in str(e) and "agv-01" in str(e),
                  str(e)[:80])
    finally:
        del os.environ[config.PROFILE_ENV_VAR]
        config.load()

    # A rejected profile must leave the live one untouched - this is what makes
    # load() safe to call again later from a reload endpoint.
    load_with(lambda d: d["timing"].update(auto_start_delay_s=-1.0))
    check("a rejected profile leaves the live one intact",
          config.AUTO_START_DELAY_S != -1.0,
          f"AUTO_START_DELAY_S={config.AUTO_START_DELAY_S}")


def test_derived_constants():
    """Values that used to be literals in .py files are now computed from the
    profile primitives.

    The geometry constants are checked against their former literals, because
    those come from a tape measure and must not drift. Everything else is
    checked as a RELATIONSHIP to its inputs - asserting a tuned number here
    would just break the suite every time someone edits the profile, which is
    the whole point of the profile existing."""
    print("\nderived constants follow their inputs")
    def close(a, b):
        return abs(a - b) < abs(b) * 1e-6
    check("MPS_PER_RPM", close(config.MPS_PER_RPM, 3.14159265e-4),
          f"{config.MPS_PER_RPM:.6e}")
    check("RPM_PER_MPS", close(config.RPM_PER_MPS, 3183.0989),
          f"{config.RPM_PER_MPS:.4f}")
    check("RAD_S_PER_RPM_DIFF",
          close(config.RAD_S_PER_RPM_DIFF, config.MPS_PER_RPM / config.TRACK_M),
          f"{config.RAD_S_PER_RPM_DIFF:.6e} (track {config.TRACK_M} m)")
    check("MAX_SPEED_MPS", close(config.MAX_SPEED_MPS, 1.2566371),
          f"{config.MAX_SPEED_MPS:.4f}")
    check("ACCEL/DECEL track the auto ramp block",
          config.ACCEL_RPM_S == config.RAMP["auto"]["accel"]
          and config.DECEL_RPM_S == config.RAMP["auto"]["decel"],
          f"{config.ACCEL_RPM_S} / {config.DECEL_RPM_S}")
    # *** 6083h is the ceiling on yaw acceleration, not just on forward ramp. ***
    # It caps how fast the wheel DIFFERENCE slews, so whatever produces
    # (v, omega) next must derive its limit from here rather than hardcode one.
    check("max yaw acceleration is derived from 6083h, not hardcoded",
          abs(kinematics.max_yaw_accel(config.ACCEL_RPM_S)
              - 2.0 * config.ACCEL_RPM_S * config.RAD_S_PER_RPM_DIFF) < 1e-12,
          f"{kinematics.max_yaw_accel(config.ACCEL_RPM_S):.3f} rad/s2")
    check("NODES maps the configured driver IDs",
          config.NODES == {config.LEFT: "left", config.RIGHT: "right"},
          str(config.NODES))


def test_params_view():
    """describe() is what /params renders, and it is generated from _SCHEMA.

    The failure this guards is silent and specific to a display page: a
    parameter the vehicle is running on that nobody can see. A hand-written
    page drifts the moment a key is added - so the page is generated, and this
    checks the generator covers the schema rather than checking a list.
    """
    print("\nthe parameters view covers the whole profile")

    sections = config.describe()
    rows = rows_by_path(sections)

    missing = [f"{sec}.{key}" for sec, fields in config._SCHEMA.items()
               for key in fields if f"{sec}.{key}" not in rows]
    check("every schema key is displayed", not missing, str(missing))

    # The three the flat schema cannot express, and the one top-level list.
    for sec, key in (("drivers", "ramp.auto.accel"), ("dio", "di_names"),
                     ("lidar", "zone_bytes")):
        check(f"the nested {sec}.{key} is displayed", f"{sec}.{key}" in rows)
    # The RFID rule table went with tape following, and the reader with the
    # tracked product (2026-10-27).
    check("no retired rule table is still rendered",
          not any(s["name"] == "rfid rules" for s in sections))

    derived = {r["key"] for s in sections if s["name"] == "derived"
               for r in s["rows"]}
    for const in ("MPS_PER_RPM", "MAX_SPEED_MPS",
                  "ACCEL_RPM_S", "RAD_S_PER_RPM_DIFF"):
        check(f"{const} is shown as derived", const in derived)
    # Derived values cannot be edited, so they must not be offered as if they
    # could: no JSON key, and the relation that produced them instead.
    check("a derived row names its relation, not a JSON key",
          all(r.get("from") and not r["const"]
              for s in sections if s["name"] == "derived" for r in s["rows"]))

    check("the profile's own values are shown",
          rows["vehicle.track_m"]["value"] == config._fmt(config.TRACK_M)
          and rows["can.channel"]["value"] == config.CAN_CHANNEL,
          rows["vehicle.track_m"]["value"])

    # Units are read off the exported name's suffix. The ones worth pinning are
    # the ones a naive suffix rule gets wrong.
    units = {r["const"]: r["unit"] for s in sections for r in s["rows"]}
    units.update({r["key"]: r["unit"] for s in sections
                  if s["name"] == "derived" for r in s["rows"]})
    for const, want in (("LOOP_PERIOD_S", "s"), ("CAN_HEARTBEAT_MS", "ms"),
                        ("TRACK_M", "m"),
                        ("MON_DRV_WARN_C", "\u00b0C"),
                        ("MON_BUS_V_WARN_LOW", "V"),
                        ("MPS_PER_RPM", "m/s per r/min")):
        check(f"{const} reads as {want}", units.get(const) == want,
              repr(units.get(const)))

    # true/false, not Python's True/False - the page is read next to the JSON
    # file it describes, and the two must be the same word.
    check("booleans render as JSON does",
          rows["can.use_rpdo"]["value"] in ("true", "false"),
          rows["can.use_rpdo"]["value"])
    check("a whole float drops its .0",
          rows["vehicle.motor_max_rpm"]["value"] == "4000",
          rows["vehicle.motor_max_rpm"]["value"])
    # An empty value is a SETTING here (an empty DO list means no output is
    # driven), so it must never render as a blank cell.
    check("no cell is blank", all(r["value"] for s in sections
                                  for r in s["rows"]))


def test_tuning_notes_are_parsed_not_restated():
    """The prose on /params is config.py's own docstring, parsed out of it.

    Every setting that is not self-evident is already explained at the top of
    this module - it is written there BECAUSE JSON cannot carry comments.
    Copying those paragraphs into a template makes two versions of one
    explanation, and the one that goes stale is the one on the screen.
    """
    print("\ntuning notes are parsed from the module docstring")

    notes = config.tuning_notes()
    # Was > 20 while the autopilot block was documented here. The threshold
    # tracks the docstring rather than being a round number, so it still fails
    # if the parser breaks - which is the only thing it is for.
    check("the notes parse at all", len(notes) > 12, f"{len(notes)} note(s)")
    check("no note is empty", all(v.strip() for v in notes.values()))

    # A heading naming several keys has to reach all of them, or the second and
    # third key look undocumented while their paragraph exists.
    for key in ("timing.loop_period_s", "timing.telemetry_period_s"):
        check(f"{key} carries the shared timing note", key in notes)
    check("a key named without its section still resolves",
          "vehicle.invert_right" in notes)
    check("a section-wide note is kept as such", "monitor.*" in notes)

    # Attachment, not just parsing: the note has to land on the row.
    rows = rows_by_path(config.describe())
    check("the use_rpdo note reaches its row",
          "rpdo" in (rows["can.use_rpdo"]["note"] or "").lower())
    check("a nested ramp row inherits the drivers.ramp note",
          "6083h" in (rows["drivers.ramp.auto.accel"]["note"] or ""))
    check("a section note reaches a row that has none of its own",
          "DIAGNOSTIC ONLY" in (rows["monitor.driver_temp_warn_c"]["note"] or ""))

    # Parsing must never be able to take the page down: a docstring rewritten
    # into prose yields no notes, not an exception.
    doc, config.__doc__ = config.__doc__, "no headings here at all"
    try:
        check("a docstring with no notes yields none, quietly",
              config.tuning_notes() == {})
    finally:
        config.__doc__ = doc
    check("...and the notes come back", len(config.tuning_notes()) > 12)


TESTS = [
    test_config_profile,
    test_derived_constants,
    test_params_view,
    test_tuning_notes_are_parsed_not_restated,
]
