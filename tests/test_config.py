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
    loudly at boot rather than showing up as odd behaviour on a length of tape."""
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
    # *** A whole retired section must not come back by accident. *** The
    # autopilot block was deleted with tape following; a profile still carrying
    # it is a profile from before the retirement, and running it would mean
    # loading gains nothing reads.
    # The autopilot section came BACK on 2026-09-21 for the ROS LINE layer
    # (dual-product plan, Increment 1), so a profile carrying it is now valid.
    # What is still refused is a PARTIAL one: the loader demands every key,
    # because a half-restored gain set is how a vehicle ends up steering on a
    # default nobody chose.
    refuses("a partial autopilot section is refused",
            lambda d: d.update({"autopilot": {"k_ratio": 11.3}}),
            "missing key")
    refuses("a leftover branch_latch table is refused",
            lambda d: d.update({"branch_latch": []}), "unknown top-level")
    refuses("a leftover stop_until_start_button table is refused",
            lambda d: d.update({"stop_until_start_button": []}),
            "unknown top-level")
    refuses("a leftover auto watchdog is refused",
            lambda d: d["timing"].update(auto_watchdog_s=1.5), "unknown key")
    refuses("a blind-run speed above the motor limit is refused",
            lambda d: d["blind_run"].update(max_rpm=9000), "blind_run.max_rpm")
    refuses("a bool where a number belongs is refused",
            lambda d: d["vehicle"].update(gear_ratio=True), "expected a number")
    # 2026-10-02: manual.* (read only by the deleted motion.py) and drivers.ramp.manual
    # (drive_node always arms with "auto") were removed as dead speed parameters.
    refuses("the retired manual section is refused",
            lambda d: d.update({"manual": {"full_rpm": 1200, "half_ratio": 0.6}}),
            "unknown top-level")
    # *** Profile position is LOCKED until the vendor has answered. *** The
    # 400 W geared motor needs motion extension, which pp cannot select; a pp
    # section switched on without the verified drive values or the vendor
    # reference must not load.
    # Either locked, or unlocked with every drive value and the vendor reference
    # recorded - whichever state the vehicle's profile is in today.
    check("pp is locked, or fully configured with a vendor reference",
          config.PP_ENABLED is False
          or (all(v is not None for v in config.PP_EXPECT.values())
              and config.PP_VENDOR_REF.strip() != ""),
          f"enabled={config.PP_ENABLED} expect={config.PP_EXPECT}")
    # The vehicle's own profile may already carry every value (agv-01 does since
    # 2026-09-18), so the test unsets them itself rather than assume a locked base.
    def pp_unset(d):
        d["pp"]["expect"] = {k: None for k in d["pp"]["expect"]}
        d["pp"].update(enabled=True, vendor_ref="OM ticket 1")
    refuses("pp enabled with unset drive values is refused", pp_unset, "null value")

    def pp_all_set(d, ref):
        d["pp"]["expect"] = {k: 1 for k in d["pp"]["expect"]}
        d["pp"].update(enabled=True, vendor_ref=ref)
    refuses("pp enabled without a vendor reference is refused",
            lambda d: pp_all_set(d, " "), "vendor_ref")
    check("pp enabled with every value and a vendor reference loads",
          load_with(lambda d: pp_all_set(d, "OM ticket 1")) is None)
    refuses("a pp.expect key missing is refused",
            lambda d: d["pp"]["expect"].pop("halt_option"), "missing key")
    refuses("a pp speed above the blind-run cap is refused",
            lambda d: d["pp"].update(max_speed_mps=0.9), "pp.max_speed_mps")
    refuses("a blind-run speed above 0.8 m/s is refused",
            lambda d: d["blind_run"].update(max_speed_mps=1.0), "blind_run.max_speed_mps")
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
    # dio.motion_on (2026-10-02): DO00 and DO08 on while the motors turn.
    check("DO00 and DO08 follow the motors", config.DIO_MOTION_ON == [0, 8], str(config.DIO_MOTION_ON))
    check("DO01 is the alarm horn", config.DIO_ALARM_ON == [1], str(config.DIO_ALARM_ON))
    check("DO00 is the movement horn", config.DIO_MOVEMENT_HORN == [0], str(config.DIO_MOVEMENT_HORN))
    # Drive power (2026-10-08): under MANUAL the drives are powered only while DI08 is HIGH.
    check("DI08 is the Manual Arm input", config.PANEL_DI_MANUAL_ARM == 8, str(config.PANEL_DI_MANUAL_ARM))
    refuses("a Manual Arm input shared with a panel button is refused",
            lambda d: d["panel"].update(di_manual_arm=d["panel"]["di_start"]), "distinct")
    refuses("a Manual Arm input on a pendant channel is refused",
            lambda d: d["panel"].update(di_manual_arm=d["pendant"]["di_fwd"]), "pendant channel")
    refuses("the retired panel.manual_auto_arm is refused",
            lambda d: d["panel"].update(manual_auto_arm=True), "unknown key")
    # Line sensor (mls-marker-plan, 2026-10-09): markers off until the bench passes.
    # 2026-10-23: markers are always on; no profile switch, so no MLS_MARKERS name either.
    check("the MLS is expected on variant 3 with codes 1-3, markers always on",
          config.MLS_VARIANT == 3 and not hasattr(config, "MLS_MARKERS")
          and config.MLS_MARKER_CODES == [1, 2, 3] and config.MLS_POLARITY_LOCK is False,
          f"{config.MLS_VARIANT} {config.MLS_MARKER_CODES}")
    refuses("an MLS variant outside 2006h:01's 0..7 is refused",
            lambda d: d["mls"].update(variant=8), "mls.variant")
    for bad, why in (([], "at least one"), ([1, 1], "duplicate"), ([0], "1..7"), ([8], "1..7"),
                     (["2"], "integers"), ([True], "integers"), (2, "integers")):
        refuses(f"mls.marker_codes {bad!r} is refused",
                lambda d, bad=bad: d["mls"].update(marker_codes=bad), why)
    refuses("the retired mls.markers_enabled is refused (markers are always on)",
            lambda d: d["mls"].update(markers_enabled=True), "unknown key")
    check("the MLS event-flag reset is off until the bench found its value (E0.5)",
          config.MLS_EVENT_CLEAR == -1, str(config.MLS_EVENT_CLEAR))
    refuses("an mls.event_clear other than -1, 0 or 1 is refused",
            lambda d: d["mls"].update(event_clear=2), "mls.event_clear")
    refuses("an unknown key in the mls block is refused",
            lambda d: d["mls"].update(marker_width_mm=15), "unknown key")
    refuses("a profile without the mls block is refused",
            lambda d: d.pop("mls"), "missing top-level")
    # Tape-graph localization (tracked plan 3): off until P1; thresholds provisional until P0.
    check("tape localization is off and its station limit fits +-10 cm",
          config.TAPE_LOC_ENABLED is False and 2 * config.TAPE_LOC_SIGMA_STATION_M <= 0.05
          and config.TAPE_LOC_AMCL_PARAMS in ("default", "lean"),
          f"{config.TAPE_LOC_ENABLED} {config.TAPE_LOC_SIGMA_STATION_M}")
    refuses("a station sigma limit above 5 cm is refused",
            lambda d: d["tape_loc"].update(sigma_station_m=0.08), "+-10 cm")
    refuses("a junction sigma limit not above the station limit is refused",
            lambda d: d["tape_loc"].update(sigma_junction_m=0.02), "sigma_junction_m")
    refuses("an unknown AMCL parameter set is refused",
            lambda d: d["tape_loc"].update(amcl_params="fast"), "default or lean")
    refuses("tape localization on a trackless profile is refused",
            lambda d: (d.update(tracked=False), d["tape_loc"].update(enabled=True)), "trackless")
    refuses("a profile without the tape_loc block is refused",
            lambda d: d.pop("tape_loc"), "missing top-level")
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
                       d["horn"].update(enabled=True, do_channel=2)),
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

    # 2026-10-08: tag meanings moved to the mission's tag table (agv_core.mission v2).
    # A profile still carrying them is refused, not silently ignored: its toggle
    # tags would otherwise look configured and never fire.
    refuses("a leftover autopilot.speed_toggle_tags is refused",
            lambda d: d["autopilot"].update(speed_toggle_tags=["00A1"]), "unknown key")
    refuses("a leftover autopilot.speed_toggle_lockout_s is refused",
            lambda d: d["autopilot"].update(speed_toggle_lockout_s=4.0), "unknown key")
    refuses("a leftover autopilot.u_turn_stop_distance_m is refused",
            lambda d: d["autopilot"].update(u_turn_stop_distance_m=0.4), "unknown key")
    # 2026-10-08 (tracked-speed-plan-1): the slow zone's keys went with the toggle.
    for key, val in (("auto_slow_ratio", 0.5), ("slow_k_ratio", 25.0), ("slow_kd", 5.0),
                     ("gain_blend_s", 0.4)):
        refuses(f"a leftover autopilot.{key} is refused",
                lambda d, k=key, v=val: d["autopilot"].update({k: v}), "unknown key")
    refuses("HIGH at or below NORMAL is refused",
            lambda d: d["autopilot"].update(auto_high_ratio=1.0), "auto_high_ratio")
    refuses("HIGH above the motor limit is refused",
            lambda d: d["autopilot"].update(auto_rpm=3000, auto_high_ratio=1.7), "motor_max_rpm")
    refuses("a curve guard error limit beyond the sensor range is refused",
            lambda d: d["autopilot"].update(curve_guard_e_mm=150.0), "curve_guard_e_mm")
    refuses("a zero high ramp is refused",
            lambda d: d["autopilot"].update(high_ramp_s=0.0), "high_ramp_s")
    refuses("a U-turn window with min >= max is refused",
            lambda d: d["autopilot"].update(u_turn_min_deg=230.0), "u_turn_min_deg")

    # A rejected profile must leave the live one untouched - this is what makes
    # load() safe to call again later from a reload endpoint.
    load_with(lambda d: d["blind_run"].update(max_rpm=9000))
    check("a rejected profile leaves the live one intact",
          config.BLIND_MAX_RPM != 9000,
          f"BLIND_MAX_RPM={config.BLIND_MAX_RPM}")


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
    # The junction table moved into the "rfid rules" block, grouped with every
    # other kind of thing a tag can mean rather than standing on its own.
    # The RFID rule table went with tape following - see config.describe().
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
    # An empty value is a SETTING here (rfid.ignore_tags empty means nothing is
    # discarded), so it must never render as a blank cell.
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
    check("a section-wide note is kept as such", "rfid.*" in notes)

    # Attachment, not just parsing: the note has to land on the row.
    rows = rows_by_path(config.describe())
    check("the use_rpdo note reaches its row",
          "rpdo" in (rows["can.use_rpdo"]["note"] or "").lower())
    check("a nested ramp row inherits the drivers.ramp note",
          "6083h" in (rows["drivers.ramp.auto.accel"]["note"] or ""))
    check("a section note reaches a row that has none of its own",
          "CF821" in (rows["rfid.ip"]["note"] or ""))

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
