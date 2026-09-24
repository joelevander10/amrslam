"""The vehicle profile: loading, validation, and the derived constants."""
import math
import os
import pathlib
import struct
import sys
import threading

from helpers import FAIL, ROOT, check

import config
import kinematics
import motion


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
            lambda d: d["manual"].update({"full_rmp": d["manual"].pop("full_rpm")}),
            "unknown key")
    refuses("a missing key is refused",
            lambda d: d["manual"].pop("half_ratio"), "missing key")
    refuses("an unknown section is refused",
            lambda d: d.update({"extra": {}}), "unknown top-level")
    # *** A whole retired section must not come back by accident. *** The
    # autopilot block was deleted with tape following; a profile still carrying
    # it is a profile from before the retirement, and running it would mean
    # loading gains nothing reads.
    refuses("a leftover autopilot section is refused",
            lambda d: d.update({"autopilot": {"k_ratio": 11.3}}),
            "unknown top-level")
    refuses("a leftover branch_latch table is refused",
            lambda d: d.update({"branch_latch": []}), "unknown top-level")
    refuses("a leftover stop_until_start_button table is refused",
            lambda d: d.update({"stop_until_start_button": []}),
            "unknown top-level")
    refuses("a leftover auto watchdog is refused",
            lambda d: d["timing"].update(auto_watchdog_s=1.5), "unknown key")
    refuses("a jog speed above the motor limit is refused",
            lambda d: d["manual"].update(full_rpm=9000), "manual.full_rpm")
    refuses("a bool where a number belongs is refused",
            lambda d: d["manual"].update(half_ratio=True), "expected a number")
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
    # expect is nulled explicitly: agv-01 now ships every pp.expect value filled in,
    # so "enabled" alone no longer leaves anything unset to be refused.
    refuses("pp enabled with unset drive values is refused",
            lambda d: d["pp"].update(enabled=True, vendor_ref="OM ticket 1",
                                     expect={k: None for k in d["pp"]["expect"]}),
            "null value")

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
    refuses("a horn channel past the end of the module is refused",
            lambda d: d["horn"].update(do_channel=d["dio"]["num_do"]),
            "horn.do_channel")
    refuses("a negative horn channel is refused",
            lambda d: d["horn"].update(do_channel=-1), "horn.do_channel")
    refuses("a horn without the DIO scan that writes it is refused",
            lambda d: (d["dio"].update(enabled=False),
                       d["panel"].update(enabled=False)),
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

    # A rejected profile must leave the live one untouched - this is what makes
    # load() safe to call again later from a reload endpoint.
    load_with(lambda d: d["manual"].update(full_rpm=9000))
    check("a rejected profile leaves the live one intact",
          config.MANUAL_FULL_RPM == 1200,
          f"MANUAL_FULL_RPM={config.MANUAL_FULL_RPM}")


def test_derived_constants():
    """Values that used to be literals in .py files are now computed from the
    profile primitives.

    The geometry constants are checked against their former literals, because
    those come from a tape measure and must not drift. Everything else is
    checked as a RELATIONSHIP to its inputs - asserting a tuned number here
    would just break the suite every time someone edits the profile, which is
    the whole point of the profile existing."""
    print("\nderived constants follow their inputs")
    close = lambda a, b: abs(a - b) < abs(b) * 1e-6
    check("MPS_PER_RPM", close(config.MPS_PER_RPM, 3.14159265e-4),
          f"{config.MPS_PER_RPM:.6e}")
    check("RPM_PER_MPS", close(config.RPM_PER_MPS, 3183.0989),
          f"{config.RPM_PER_MPS:.4f}")
    check("RAD_S_PER_RPM_DIFF",
          close(config.RAD_S_PER_RPM_DIFF, config.MPS_PER_RPM / config.TRACK_M),
          f"{config.RAD_S_PER_RPM_DIFF:.6e} (track {config.TRACK_M} m)")
    check("MAX_SPEED_MPS", close(config.MAX_SPEED_MPS, 1.2566371),
          f"{config.MAX_SPEED_MPS:.4f}")
    check("MANUAL_HALF_RPM is full_rpm * half_ratio, rounded",
          config.MANUAL_HALF_RPM
          == round(config.MANUAL_FULL_RPM * config.MANUAL_HALF_RATIO),
          f"{config.MANUAL_FULL_RPM} x {config.MANUAL_HALF_RATIO} "
          f"-> {config.MANUAL_HALF_RPM}")
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
    # motion's table is built from the profile now, not from module literals.
    full, half = config.MANUAL_FULL_RPM, config.MANUAL_HALF_RPM
    check("manual jog table is built from the profile",
          motion.velocities("forward_left") == (half, full)
          and motion.velocities("forward") == (full, full)
          and motion.velocities("stop") == (0, 0),
          f"fwd-left {motion.velocities('forward_left')}")


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
    for const in ("MPS_PER_RPM", "MAX_SPEED_MPS", "MANUAL_HALF_RPM",
                  "ACCEL_RPM_S", "RAD_S_PER_RPM_DIFF"):
        check(f"{const} is shown as derived", const in derived)
    # Derived values cannot be edited, so they must not be offered as if they
    # could: no JSON key, and the relation that produced them instead.
    check("a derived row names its relation, not a JSON key",
          all(r.get("from") and not r["const"]
              for s in sections if s["name"] == "derived" for r in s["rows"]))

    check("the profile's own values are shown",
          rows["manual.full_rpm"]["value"] == config._fmt(config.MANUAL_FULL_RPM)
          and rows["can.channel"]["value"] == config.CAN_CHANNEL,
          rows["manual.full_rpm"]["value"])

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
    # An empty string is a SETTING here (rfid.init_hex empty means the reader is
    # never told to start), so it must never render as a blank cell.
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
