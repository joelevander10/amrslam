"""Tape-AGV missions: missions/<name>.json, validated against the vehicle profile.

Schema v2 (2026-10-08): the mission IS the site's RFID tag reference table. One
row per tag, and the row says everything the tag does:

    stop          decelerate to rest over stop_distance_m, then by role:
                    home         the run ends there (DONE); the next run is a new job
                    always       wait for the physical Start, every pass
                    destination  stop only when it is the job's destination, and
                                 only until it has been served once
    u_turn        decelerate to approach_mps within decel_m, creep until the tape
                  ends, stop at once (at creep speed or not), pivot (cw|ccw) until
                  the tape is reacquired and centred - 180 deg by geometry, the
                  encoders only gate it (amr_line.uturn)
    ignore_s      after a read is ACTED ON, the same tag is ignored this long.
                  A stop's window restarts at departure, since a parked vehicle
                  outlasts any window.

RFID decides WHERE TO STOP; the MLS marker codes decide SPEED (operator, 2026-10-09).
The v2.1 RFID zone rows (zone_outer/zone_inner) and the v2 speed_toggle are refused:
speed zones work whatever the RFID setting is, and RFID never changes the speed.

high_zone (v2.2, 2026-10-09): null (the run stays at NORMAL) or
    {"shortest_straight_m": surveyed inner marker to inner marker,
     "pair_spacing_m": 0.5, "outer_code": 2, "inner_code": 1}
Every straight end carries two MLS markers: the OUTER code on the corner side, the
INNER code on the straight side. Outer then inner = entering the straight: HIGH for
the mission's budget. Inner while HIGH = leaving (amr_line.speed_zone). The codes
are shared by every pair, must be laid on the floor (mls.marker_codes), and are
reserved: nothing else may use them. The one HIGH budget for every straight:
    high_for_m = shortest_straight_m - brake_m - autopilot.high_margin_m
    brake_m    = (v_high + v_normal) / 2 x autopilot.high_ramp_s
Marker events are always on (2026-10-23): a mission without a high_zone runs NORMAL only.

Branching stays its own table (branch_latch + branch_default) and its own engine
(amr_line.branch): speed, stops and branch orders never share a tag.

v1 (gy-demo's route sequence, route_guard, stop_until_start_button, u_turn,
outbound/inbound direction) is retired: a v1 document is refused by name rather
than half-loaded. The repo ships no site mission; "empty" (or no mission) is
plain line following. missions/empty.json, when present, is what plain line
following runs - the site's U-turn tags, so a vehicle without a job still turns
at the tape ends (2026-10-08).

    list_missions()          -> ["empty", ...]
    load(name, vehicle=None) -> dict of the names below, or MissionError
    destinations(m)          -> the destination labels, in table order
    summary(m)               -> the picker's note: branch rule, speed rule

The four basic jobs (2026-10-23) are plain line following with the site's U-turn tags:
empty (always left), basic-right, basic-high-left, basic-high-right (high zones on the
line-a marker pair).

MISSION_NAME, BRANCH_DEFAULT, BRANCH_LATCH, TAGS {tag: row}, HOME (row | None),
DESTINATIONS (labels), HIGH_ZONE ({"inner", "outer" (marker codes, int), "high_for_m",
"arm_m", "spacing_m", "brake_m"} | None), MARKER_RUN (dict | None, below).

marker_run (optional, 2026-10-18, tracked plan 3 section 7.5): a run with no RFID at all,
for the straight in front of MRU. Two MLS marker codes do what tags 0180 and Home did:
    {"uturn_code": 2, "home_code": 1, "direction": "cw", "approach_mps": 0.1,
     "decel_m": 0.3, "max_approach_m": 2.0, "stop_distance_m": 0.5, "label": "T-HOME"}
Out: the first uturn_code starts the U-turn approach (as a u_turn tag row). Back, after
the pivot: the first home_code ends the run at home (a measured stop, DONE); a uturn_code
there means the home marker was missed - FAULT at once (the backstop strip). The codes
mean nothing else, so the block is refused with a high_zone (the same codes are the speed
zone elsewhere) and with any RFID tag or branch row.

Free of ROS; the line layer and the web both read it.
"""
import json
import os

from agv_core.config import ConfigError, _coerce

MISSION_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "missions")

ACTIONS = ("stop", "u_turn")
# The outer marker waits this far beyond the pair spacing for its inner marker. RFID
# needed 1.0 m (first reads of a 0.5 m pair landed 0.65-0.99 m apart, 2026-10-08); an
# MLS marker is reported at a fixed point - FailSafe, after the 100 mm marker plus the
# t90 filter delay, the same lag for both - so 0.15 m covers encoder scale and skew.
# Wider is safe (an expired arm is NORMAL) but it must stay below the exit-outer ->
# entry-outer distance through a corner.
ZONE_ARM_TOLERANCE_M = 0.15
# A budget shorter than this is not worth the two ramps.
ZONE_MIN_BUDGET_M = 3.0
ROLES = ("home", "always", "destination")

# Keys per action, beyond the common tag/action/ignore_s. "label" is optional on
# u_turn and speed_toggle rows, required on stops (the operator picks by it).
_KEYS = {
    "stop": {"stop_distance_m", "role", "label"},
    "u_turn": {"direction", "approach_mps", "decel_m", "max_approach_m"},
}
_COMMON = {"tag", "action", "ignore_s"}
_OPTIONAL = {"stop": set(), "u_turn": {"label"}}

_V1_KEYS = ("route", "route_guard", "stop_until_start_button", "u_turn")


class MissionError(ConfigError):
    """A mission file that cannot be run as written."""


def _tag(raw, width, where, ignored):
    """The reader hands tags over as hex text (rfid.tag_of): a JSON number, a
    wrong width or an ignored tag is a rule that never fires, and fails silently."""
    if not isinstance(raw, str):
        raise ConfigError(f"{where}: expected a {width}-character hex string like \"0010\", "
                          f"got {raw!r}. Tag ids come from rfid.tag_of() as hex text, "
                          f"so a number never matches.")
    tag = raw.strip().upper()
    if len(tag) != width or any(c not in "0123456789ABCDEF" for c in tag):
        raise ConfigError(f"{where}: expected {width} hex characters as the RFID panel "
                          f"shows them, got {raw!r}")
    if tag in ignored:
        raise ConfigError(f"{where}: {tag} is also in rfid.ignore_tags, so the reader "
                          f"discards it before any rule can see it")
    return tag


def _read_branch_latch(rows, tag_len, ignore_tags):
    """branch_latch -> validated rows, or [] when there are no junctions.

    An entry tag sets a direction and an exit tag clears it (branch.Ladder).
    Every check here exists to stop a rung being silently DEAD: a tag that never
    matches produces no error, no log line and no motion - the AGV simply drives
    past the junction.

    slow_speed is OPTIONAL and must be false (2026-10-09, operator: no RFID tag affects the
    speed - the MLS marker codes do). It is still type-checked: the string "True" is
    refused, not silently accepted as truthy.
    """
    if not isinstance(rows, list):
        raise ConfigError("branch_latch: expected a list")
    want = {"entry_tag", "exit_tag", "branch"}
    optional = {"slow_speed"}
    width = tag_len * 2
    seen = {}
    out = []
    for i, row in enumerate(rows):
        where = f"branch_latch[{i}]"
        if not isinstance(row, dict):
            raise ConfigError(f"{where}: expected an object")
        unknown = set(row) - want - optional
        if unknown:
            raise ConfigError(f"{where}: unknown key(s) {sorted(unknown)}")
        missing = want - set(row)
        if missing:
            raise ConfigError(f"{where}: missing key(s) {sorted(missing)}")

        slow = _coerce(row.get("slow_speed", False), bool, f"{where}.slow_speed")
        if slow:
            raise ConfigError(f"{where}.slow_speed: retired 2026-10-09 - no RFID tag affects the speed; "
                              f"a junction that must not be passed at HIGH lies outside the marker "
                              f"high zone (high_zone.outer_code/inner_code)")
        side = _coerce(row["branch"], str, f"{where}.branch")
        if side not in ("left", "right"):
            raise ConfigError(f"{where}.branch: expected 'left' or 'right', got {side!r}")

        tags = {key: _tag(row[key], width, f"{where}.{key}", ignore_tags)
                for key in ("entry_tag", "exit_tag")}
        if tags["entry_tag"] == tags["exit_tag"]:
            raise ConfigError(f"{where}: entry_tag and exit_tag are both {tags['entry_tag']} - "
                              f"the same read cannot both set and clear the latch")
        for key, tag in tags.items():
            if tag in seen:
                raise ConfigError(f"{where}.{key}: tag {tag} is already used by {seen[tag]} - "
                                  f"one tag cannot mean two things")
            seen[tag] = f"{where}.{key}"
        out.append({"entry_tag": tags["entry_tag"], "exit_tag": tags["exit_tag"],
                    "branch": side, "slow_speed": slow})
    return out


def _bounded(row, key, where, lo, hi, lo_open=True):
    v = _coerce(row[key], float, f"{where}.{key}")
    ok = (lo < v if lo_open else lo <= v) and v <= hi
    if not ok:
        raise ConfigError(f"{where}.{key} must be in {'(' if lo_open else '['}{lo:g}, {hi:g}], got {v:g}")
    return v


def _read_tags(rows, vehicle, branch_tags):
    """tags -> {tag: normalised row}. Every rule a tag can carry, one row each."""
    if not isinstance(rows, list):
        raise ConfigError("tags: expected a list")
    width = 2 * vehicle.RFID_TAG_LEN
    ignored = {t.upper() for t in vehicle.RFID_IGNORE_TAGS}
    out, labels = {}, {}
    for i, row in enumerate(rows):
        where = f"tags[{i}]"
        if not isinstance(row, dict):
            raise ConfigError(f"{where}: expected an object")
        action = row.get("action")
        if action == "speed_toggle":
            raise ConfigError(f"{where}: speed_toggle is retired (2026-10-08): a missed read "
                              f"carried HIGH into a corner. Speed is the MLS marker pair: "
                              f"high_zone.outer_code/inner_code")
        if action in ("zone_inner", "zone_outer"):
            raise ConfigError(f"{where}: RFID zone tags are retired (2026-10-09): RFID decides "
                              f"where to stop, the MLS marker codes decide speed. Lay an "
                              f"outer/inner marker pair and set high_zone.outer_code/inner_code")
        if action not in ACTIONS:
            raise ConfigError(f"{where}.action: expected one of {list(ACTIONS)}, got {action!r}")
        allowed = _COMMON | _KEYS[action] | _OPTIONAL[action]
        unknown = set(row) - allowed
        if unknown:
            raise ConfigError(f"{where}: unknown key(s) {sorted(unknown)} for a {action} row")
        missing = (_COMMON | _KEYS[action]) - set(row)
        if missing:
            raise ConfigError(f"{where}: missing key(s) {sorted(missing)} for a {action} row")

        tag = _tag(row["tag"], width, f"{where}.tag", ignored)
        where = f"tags[{i}] ({tag})"
        if tag in out:
            raise ConfigError(f"{where}: tag {tag} is listed twice - one tag cannot mean two things")
        if tag in branch_tags:
            raise ConfigError(f"{where}: tag {tag} is also a branch_latch tag - "
                              f"one tag cannot mean two things")
        rec = {"tag": tag, "action": action,
               "ignore_s": _bounded(row, "ignore_s", where, 0.0, 30.0, lo_open=False),
               "label": ""}
        if "label" in row:
            rec["label"] = _coerce(row["label"], str, f"{where}.label").strip()

        if action == "stop":
            rec["stop_distance_m"] = _bounded(row, "stop_distance_m", where, 0.0, 5.0)
            role = row["role"]
            if role not in ROLES:
                raise ConfigError(f"{where}.role: expected one of {list(ROLES)}, got {role!r}")
            rec["role"] = role
            if not rec["label"]:
                raise ConfigError(f"{where}.label: a stop needs a name the operator recognises")
            if rec["label"] in labels:
                raise ConfigError(f"{where}.label: {rec['label']!r} is already {labels[rec['label']]}")
            labels[rec["label"]] = tag
        else:  # u_turn
            if row["direction"] not in ("cw", "ccw"):
                raise ConfigError(f"{where}.direction must be cw or ccw")
            rec["direction"] = row["direction"]
            normal_mps = vehicle.AUTO_RPM * vehicle.MPS_PER_RPM
            rec["approach_mps"] = _bounded(row, "approach_mps", where, 0.0, round(normal_mps, 3))
            rec["max_approach_m"] = _bounded(row, "max_approach_m", where, 0.0, 5.0)
            rec["decel_m"] = _bounded(row, "decel_m", where, 0.0, 2.0)
            # NORMAL -> approach_mps over decel_m within the drives' deceleration. The
            # tag belongs in a NORMAL area; arriving HIGH, the rate is capped at runtime
            # and the slowdown takes longer than decel_m.
            creep_rpm = rec["approach_mps"] / vehicle.MPS_PER_RPM
            rate = (vehicle.AUTO_RPM ** 2 - creep_rpm ** 2) * vehicle.MPS_PER_RPM / (2.0 * rec["decel_m"])
            decel = float(vehicle.RAMP["auto"]["decel"])
            if rate > decel:
                raise ConfigError(f"{where}.decel_m: slowing from {vehicle.AUTO_RPM:.0f} r/min to "
                                  f"{rec['approach_mps']:g} m/s in {rec['decel_m']:g} m needs {rate:.0f} "
                                  f"r/min/s, beyond drivers.ramp.auto.decel {decel:.0f}")
        out[tag] = rec
    return out


def _check_table(tags, vehicle, fastest_rpm):
    """Rules that span rows: one home, stops the drives can make from the fastest speed."""
    homes = [r for r in tags.values() if r["action"] == "stop" and r["role"] == "home"]
    if len(homes) > 1:
        raise ConfigError(f"tags: {len(homes)} home stops ({', '.join(r['tag'] for r in homes)}); "
                          f"a run ends at exactly one place")
    dests = [r for r in tags.values() if r["action"] == "stop" and r["role"] == "destination"]
    if dests and not homes:
        raise ConfigError("tags: destination stops need a home stop - the run starts and ends there")

    # A measured stop from the fastest speed the mission can reach (HIGH when it has a
    # high zone: a stop tag read during a late exit) must be within what the drives
    # decelerate at: beyond it, 6084h clips the profile and the vehicle stops past the mark.
    decel = float(vehicle.RAMP["auto"]["decel"])
    for r in tags.values():
        if r["action"] != "stop":
            continue
        rate = fastest_rpm ** 2 * vehicle.MPS_PER_RPM / (2.0 * r["stop_distance_m"])
        if rate > decel:
            raise ConfigError(f"tags ({r['tag']}): stopping from {fastest_rpm:.0f} r/min in "
                              f"{r['stop_distance_m']:g} m needs {rate:.0f} r/min/s, beyond "
                              f"drivers.ramp.auto.decel {decel:.0f}; commission a longer stop_distance_m")
    return homes[0] if homes else None


_MISSION_KEYS = ("mission_name", "branch_default", "branch_latch", "high_zone", "tags")
_OPTIONAL_MISSION_KEYS = ("marker_run",)
_MARKER_RUN_KEYS = {"uturn_code", "home_code", "direction", "approach_mps", "decel_m", "max_approach_m",
                    "stop_distance_m", "label"}


def _read_marker_run(raw, vehicle):
    """marker_run -> the validated block, or None."""
    if raw is None:
        return None
    if not isinstance(raw, dict) or set(raw) != _MARKER_RUN_KEYS:
        raise ConfigError(f"marker_run: expected null or exactly {sorted(_MARKER_RUN_KEYS)}")
    laid = list(vehicle.MLS_MARKER_CODES)
    out = {}
    for key in ("uturn_code", "home_code"):
        v = raw[key]
        if isinstance(v, bool) or not isinstance(v, int):
            raise ConfigError(f"marker_run.{key}: expected an integer marker code, got {v!r}")
        if v not in laid:
            raise ConfigError(f"marker_run.{key}: code {v} is not laid on the floor "
                              f"(mls.marker_codes {laid})")
        out[key] = v
    if out["uturn_code"] == out["home_code"]:
        raise ConfigError("marker_run: uturn_code and home_code must differ")
    if raw["direction"] not in ("cw", "ccw"):
        raise ConfigError("marker_run.direction must be cw or ccw")
    out["direction"] = raw["direction"]
    out["label"] = _coerce(raw["label"], str, "marker_run.label").strip()
    if not out["label"]:
        raise ConfigError("marker_run.label: the home stop needs a name the operator recognises")
    normal_mps = vehicle.AUTO_RPM * vehicle.MPS_PER_RPM
    out["approach_mps"] = _bounded(raw, "approach_mps", "marker_run", 0.0, round(normal_mps, 3))
    out["max_approach_m"] = _bounded(raw, "max_approach_m", "marker_run", 0.0, 5.0)
    out["decel_m"] = _bounded(raw, "decel_m", "marker_run", 0.0, 2.0)
    out["stop_distance_m"] = _bounded(raw, "stop_distance_m", "marker_run", 0.0, 5.0)
    decel = float(vehicle.RAMP["auto"]["decel"])
    creep_rpm = out["approach_mps"] / vehicle.MPS_PER_RPM
    rate = (vehicle.AUTO_RPM ** 2 - creep_rpm ** 2) * vehicle.MPS_PER_RPM / (2.0 * out["decel_m"])
    if rate > decel:
        raise ConfigError(f"marker_run.decel_m: {rate:.0f} r/min/s is beyond "
                          f"drivers.ramp.auto.decel {decel:.0f}")
    rate = vehicle.AUTO_RPM ** 2 * vehicle.MPS_PER_RPM / (2.0 * out["stop_distance_m"])
    if rate > decel:
        raise ConfigError(f"marker_run.stop_distance_m: {rate:.0f} r/min/s is beyond drivers.ramp.auto.decel "
                          f"{decel:.0f}")
    return out


def _read_high_zone(raw, vehicle):
    """high_zone -> the marker pair and the one budget every straight shares, or None."""
    if raw is None:
        return None
    keys = {"shortest_straight_m", "pair_spacing_m", "outer_code", "inner_code"}
    if not isinstance(raw, dict) or set(raw) != keys:
        raise ConfigError(f"high_zone: expected null or exactly {sorted(keys)} (v2.2, 2026-10-09: "
                          f"the zone pair is two MLS marker codes, not RFID tags)")
    laid = list(vehicle.MLS_MARKER_CODES)
    codes = {}
    for key in ("outer_code", "inner_code"):
        v = raw[key]
        if isinstance(v, bool) or not isinstance(v, int):
            raise ConfigError(f"high_zone.{key}: expected an integer marker code, got {v!r}")
        if v not in laid:
            raise ConfigError(f"high_zone.{key}: code {v} is not laid on the floor "
                              f"(mls.marker_codes {laid}); the zone would never be entered")
        codes[key] = v
    if codes["outer_code"] == codes["inner_code"]:
        raise ConfigError("high_zone: outer_code and inner_code must differ - the read ORDER "
                          "tells entering from leaving")
    spacing = _bounded(raw, "pair_spacing_m", "high_zone", 0.3, 1.0, lo_open=False)
    shortest = _bounded(raw, "shortest_straight_m", "high_zone", 0.0, 500.0)
    v_normal = vehicle.AUTO_RPM * vehicle.MPS_PER_RPM
    v_high = vehicle.AUTO_HIGH_RPM * vehicle.MPS_PER_RPM
    brake = (v_high + v_normal) / 2.0 * vehicle.HIGH_RAMP_S
    budget = shortest - brake - vehicle.HIGH_MARGIN_M
    if budget < ZONE_MIN_BUDGET_M:
        raise ConfigError(f"high_zone: {shortest:g} m straight leaves a {budget:.2f} m HIGH budget "
                          f"after {brake:.2f} m braking and {vehicle.HIGH_MARGIN_M:g} m margin "
                          f"(< {ZONE_MIN_BUDGET_M:g} m: not worth it)")
    return {"inner": codes["inner_code"], "outer": codes["outer_code"], "high_for_m": budget,
            "brake_m": brake, "spacing_m": spacing, "arm_m": spacing + ZONE_ARM_TOLERANCE_M}


def _parse_mission(doc, vehicle):
    if not isinstance(doc, dict):
        raise ConfigError("mission must be a JSON object")
    v1 = sorted(set(doc) & set(_V1_KEYS))
    if v1:
        raise ConfigError(f"mission: {v1} are the retired v1 route schema (2026-10-08); "
                          f"write the site's tag reference as the 'tags' table")
    unknown = set(doc) - set(_MISSION_KEYS) - set(_OPTIONAL_MISSION_KEYS)
    if unknown:
        raise ConfigError(f"mission: unknown key(s) {sorted(unknown)}")
    missing = set(_MISSION_KEYS) - set(doc)
    if missing:
        raise ConfigError(f"mission: missing key(s) {sorted(missing)}")

    out = {"MISSION_NAME": _coerce(doc["mission_name"], str, "mission_name")}
    default = _coerce(doc["branch_default"], str, "branch_default")
    if default not in ("straight", "left", "right"):
        raise ConfigError(f"branch_default ({default!r}) must be straight, left or right")
    out["BRANCH_DEFAULT"] = default
    out["BRANCH_LATCH"] = _read_branch_latch(doc["branch_latch"], vehicle.RFID_TAG_LEN,
                                             {t.upper() for t in vehicle.RFID_IGNORE_TAGS})
    branch_tags = {r[k] for r in out["BRANCH_LATCH"] for k in ("entry_tag", "exit_tag")}
    out["TAGS"] = _read_tags(doc["tags"], vehicle, branch_tags)
    out["HIGH_ZONE"] = _read_high_zone(doc["high_zone"], vehicle)
    fastest = vehicle.AUTO_HIGH_RPM if out["HIGH_ZONE"] else vehicle.AUTO_RPM
    out["HOME"] = _check_table(out["TAGS"], vehicle, float(fastest))
    out["DESTINATIONS"] = [r["label"] for r in out["TAGS"].values()
                           if r["action"] == "stop" and r["role"] == "destination"]
    out["MARKER_RUN"] = _read_marker_run(doc.get("marker_run"), vehicle)
    if out["MARKER_RUN"] is not None:
        if out["HIGH_ZONE"] is not None:
            raise ConfigError("marker_run with a high_zone: the speed zone's codes and the marker run's "
                              "would mean two things")
        if out["TAGS"] or out["BRANCH_LATCH"]:
            raise ConfigError("marker_run with RFID tag or branch rows: a marker run has no RFID; "
                              "two triggers for one U-turn or home stop would race")
    return out


def destinations(m):
    """The destination labels of a parsed mission (or None), in table order."""
    return list(m["DESTINATIONS"]) if m else []


_BRANCH_NOTE = {"left": "always branch left", "right": "always branch right",
                "straight": "straight through branches"}


def summary(m):
    """The mission picker's note for a parsed mission: its branch rule and its speed rule."""
    branch = _BRANCH_NOTE[m["BRANCH_DEFAULT"]]
    if m["BRANCH_LATCH"]:
        branch += f" (+{len(m['BRANCH_LATCH'])} tag orders)"
    z = m["HIGH_ZONE"]
    parts = [branch, f"HIGH zones (markers {z['outer']}→{z['inner']})" if z else "normal speed"]
    if m["MARKER_RUN"] is not None:
        parts.append("marker run")
    elif m["DESTINATIONS"]:
        parts.append(f"{len(m['DESTINATIONS'])} destinations")
    return " · ".join(parts)


def list_missions(directory=None):
    """Mission names on disk, sorted. "empty" is always offered."""
    d = directory or MISSION_DIR
    try:
        names = {f[:-5] for f in os.listdir(d) if f.endswith(".json")}
    except OSError:
        names = set()
    # Site missions (schema site-3) are not tag-table jobs: listed by list_site_missions().
    names -= set(list_site_missions(d))
    return sorted(names | {"empty"})


EMPTY = {
    "mission_name": "empty",
    "branch_default": "straight",
    "branch_latch": [],
    "high_zone": None,
    "tags": [],
}


def parse(doc, vehicle=None):
    """A mission document -> the validated names. MissionError on any problem."""
    if vehicle is None:
        from agv_core import config as vehicle  # noqa: PLC0415
    try:
        return _parse_mission(doc, vehicle)
    except MissionError:
        raise
    except ConfigError as e:
        raise MissionError(str(e)) from None


def load(name, vehicle=None, directory=None):
    """missions/<name>.json -> validated names. "empty" needs no file."""
    if not isinstance(name, str) or not name or "/" in name or name.startswith("."):
        raise MissionError(f"not a mission name: {name!r}")
    path = os.path.join(directory or MISSION_DIR, f"{name}.json")
    if name == "empty" and not os.path.exists(path):
        doc = EMPTY
    else:
        try:
            with open(path) as f:
                doc = json.load(f)
        except OSError:
            raise MissionError(f"no mission {name!r} ({path}); have {list_missions(directory)}") from None
        except ValueError as e:
            raise MissionError(f"mission {name!r}: not valid JSON ({e})") from None
    if is_site(doc):
        raise MissionError(f"{name!r} is a site mission (tracked plan 3): it runs on tape-graph "
                           f"localization, not as an RFID tag-table job")
    out = parse(doc, vehicle)
    if out["MISSION_NAME"] != name:
        raise MissionError(f"mission_name {out['MISSION_NAME']!r} does not match the file name {name!r}")
    return out


# -- site missions (schema "site-3", tracked plan 3 section 2.5) ---------------------
#
# A site mission is designed data: groups of machines, circuits with every junction
# order written out, and what happens at each stop. WHERE things are (legs, stations)
# is taught on the vehicle into the tape graph (amr_line.tape_graph), bound to a map
# revision; the two meet in tape_graph.compile_route, which names anything not taught.
#
#   schema            "site-3"
#   mission_name      the file name
#   home, attach      station ids: the bay where the AGV parks empty, the attach stop
#   trip_calls        calls of one group that form a trip (site D3/D4: 2)
#   stop_distance_m   braking distance into every station
#   junctions         {id: {"shape": "Y"|"T"|"X", "laid": bool}}. An unlaid junction is
#                     not a node: the legs run through it until it is laid and re-taught
#   uturns            {id: {"direction": "cw"|"ccw", "approach_mps", "decel_m",
#                     "max_approach_m", "lead_m"}}: a tape-end pivot, empty only
#   circuits          {id: [item, ...]}, items in travel order:
#                       {"stop": id} | {"junction": id, "order": o} | {"uturn": id}
#                     o: left | right | straight | merge (Y), cross (X). A T item also says
#                     "approach": "stem" (left | right only: no straight on) or "bar" (as a Y)
#                     A circuit starts and ends with {"stop": home} and passes attach once
#   groups            {name: {"circuit": id, "machines": {machine: {"swap": stop} |
#                     {"release": stop, "take": stop}}}}
#
# Parsed names: MISSION_NAME, SCHEMA, HOME, ATTACH, TRIP_CALLS, STOP_DISTANCE_M,
# JUNCTIONS, UTURNS, CIRCUITS {id: {"items", "legs", "home_depart", "home_arrive"}}, GROUPS,
# MACHINES {machine: group}, STATIONS {station: leg id}.

SITE_SCHEMA = "site-3"
_SITE_KEYS = ("schema", "mission_name", "home", "attach", "trip_calls", "stop_distance_m",
              "junctions", "uturns", "circuits", "groups")
SHAPES = ("Y", "T", "X")
_ORDERS = {"Y": ("left", "right", "straight", "merge"), "X": ("cross",),
           # A T from its stem has no straight on; along its bar it is a Y.
           "T-stem": ("left", "right"), "T-bar": ("left", "right", "straight", "merge")}
_UTURN_KEYS = {"direction", "approach_mps", "decel_m", "max_approach_m", "lead_m"}


def is_site(doc):
    return isinstance(doc, dict) and doc.get("schema") == SITE_SCHEMA


def _ident(v, where):
    if not isinstance(v, str) or not v.strip() or v != v.strip() or any(c in v for c in ":>/ "):
        raise ConfigError(f"{where}: expected an id without spaces, ':', '>' or '/', got {v!r}")
    return v


def _site_uturn(name, row, vehicle):
    where = f"uturns.{name}"
    if not isinstance(row, dict) or set(row) != _UTURN_KEYS:
        raise ConfigError(f"{where}: expected exactly {sorted(_UTURN_KEYS)}")
    if row["direction"] not in ("cw", "ccw"):
        raise ConfigError(f"{where}.direction must be cw or ccw")
    out = {"direction": row["direction"]}
    normal_mps = vehicle.AUTO_RPM * vehicle.MPS_PER_RPM
    out["approach_mps"] = _bounded(row, "approach_mps", where, 0.0, round(normal_mps, 3))
    out["max_approach_m"] = _bounded(row, "max_approach_m", where, 0.0, 5.0)
    out["decel_m"] = _bounded(row, "decel_m", where, 0.0, 2.0)
    out["lead_m"] = _bounded(row, "lead_m", where, 0.0, 10.0)
    if out["lead_m"] < out["decel_m"]:
        raise ConfigError(f"{where}.lead_m ({out['lead_m']:g}) must cover decel_m ({out['decel_m']:g}): "
                          f"the approach starts lead_m before the taught tape end")
    creep_rpm = out["approach_mps"] / vehicle.MPS_PER_RPM
    rate = (vehicle.AUTO_RPM ** 2 - creep_rpm ** 2) * vehicle.MPS_PER_RPM / (2.0 * out["decel_m"])
    if rate > float(vehicle.RAMP["auto"]["decel"]):
        raise ConfigError(f"{where}.decel_m: {rate:.0f} r/min/s is beyond drivers.ramp.auto.decel")
    return out


def _site_items(cid, raw, junctions, uturns):
    if not isinstance(raw, list) or len(raw) < 3:
        raise ConfigError(f"circuits.{cid}: expected a list of at least three items")
    items = []
    for i, it in enumerate(raw):
        where = f"circuits.{cid}[{i}]"
        if not isinstance(it, dict):
            raise ConfigError(f"{where}: expected an object")
        if set(it) == {"stop"}:
            items.append({"kind": "stop", "id": _ident(it["stop"], f"{where}.stop")})
        elif set(it) == {"uturn"}:
            if it["uturn"] not in uturns:
                raise ConfigError(f"{where}: U-turn {it['uturn']!r} is not in uturns")
            items.append({"kind": "uturn", "id": it["uturn"], "order": "pivot"})
        elif "junction" in it:
            if "order" not in it:
                raise ConfigError(f"{where}: junction {it['junction']!r} without an order - every "
                                  f"junction on a circuit says what to do there")
            j = junctions.get(it["junction"])
            if j is None:
                raise ConfigError(f"{where}: junction {it['junction']!r} is not in junctions")
            keys = {"junction", "order"} | ({"approach"} if j["shape"] == "T" else set())
            if set(it) != keys:
                raise ConfigError(f"{where}: expected exactly {sorted(keys)} at {it['junction']} "
                                  f"(shape {j['shape']}" + ("; approach is stem or bar)" if j["shape"] == "T"
                                                           else ")"))
            shape = j["shape"]
            if shape == "T":
                if it["approach"] not in ("stem", "bar"):
                    raise ConfigError(f"{where}.approach: expected stem or bar, got {it['approach']!r}")
                shape = f"T-{it['approach']}"
            allowed = _ORDERS[shape]
            if it["order"] not in allowed:
                raise ConfigError(f"{where}: order {it['order']!r} at {it['junction']} ({shape}): "
                                  f"expected one of {list(allowed)}"
                                  + (" - a T has no straight on from its stem" if shape == "T-stem" else ""))
            items.append({"kind": "junction", "id": it["junction"], "order": it["order"]})
        else:
            raise ConfigError(f"{where}: expected {{'stop'}}, {{'uturn'}} or {{'junction', 'order'}}")
    return items


def _site_legs(cid, items, junctions, home):
    """Items -> (legs with the home leg LAST, the home-leg stops of the departure, those
    of the arrival)."""
    nodes = [k for k, it in enumerate(items)
             if it["kind"] == "uturn" or (it["kind"] == "junction" and junctions[it["id"]]["laid"])]
    if not nodes:
        raise ConfigError(f"circuits.{cid}: no laid junction or U-turn - nothing to split the tape at")
    legs = []
    for a, b in zip(nodes, nodes[1:], strict=False):
        frm, to = items[a], items[b]
        legs.append({"id": f"{frm['id']}:{frm['order']}>{to['id']}", "from": frm["id"],
                     "order": frm["order"], "to": to["id"],
                     "stops": [it["id"] for it in items[a + 1:b] if it["kind"] == "stop"]})
    last, first = items[nodes[-1]], items[nodes[0]]
    before = [it["id"] for it in items[:nodes[0]] if it["kind"] == "stop"]
    after = [it["id"] for it in items[nodes[-1] + 1:] if it["kind"] == "stop"]
    legs.append({"id": f"{last['id']}:{last['order']}>{first['id']}", "from": last["id"],
                 "order": last["order"], "to": first["id"], "stops": after + before})
    # The home leg is driven twice (out of the bay, back into it): `after` then `before`
    # in tape order, but `before` is the trip's start.
    return legs, before, after


def _parse_site(doc, vehicle):
    if not isinstance(doc, dict):
        raise ConfigError("mission must be a JSON object")
    unknown = set(doc) - set(_SITE_KEYS)
    if unknown:
        raise ConfigError(f"site mission: unknown key(s) {sorted(unknown)}")
    missing = set(_SITE_KEYS) - set(doc)
    if missing:
        raise ConfigError(f"site mission: missing key(s) {sorted(missing)}")
    out = {"SCHEMA": SITE_SCHEMA, "MISSION_NAME": _coerce(doc["mission_name"], str, "mission_name")}
    home, attach = _ident(doc["home"], "home"), _ident(doc["attach"], "attach")
    if home == attach:
        raise ConfigError("home and attach must be different stations")
    out["HOME"], out["ATTACH"] = home, attach
    calls = doc["trip_calls"]
    if isinstance(calls, bool) or not isinstance(calls, int) or not 1 <= calls <= 2:
        raise ConfigError(f"trip_calls: expected 1 or 2 (site D3: at most two machines a trip), "
                          f"got {calls!r}")
    out["TRIP_CALLS"] = calls
    out["STOP_DISTANCE_M"] = _bounded(doc, "stop_distance_m", "site mission", 0.0, 5.0)
    rate = (vehicle.AUTO_RPM ** 2 * vehicle.MPS_PER_RPM / (2.0 * out["STOP_DISTANCE_M"]))
    if rate > float(vehicle.RAMP["auto"]["decel"]):
        raise ConfigError(f"stop_distance_m: stopping from {vehicle.AUTO_RPM:.0f} r/min in "
                          f"{out['STOP_DISTANCE_M']:g} m needs {rate:.0f} r/min/s, "
                          f"beyond drivers.ramp.auto.decel")

    if not isinstance(doc["junctions"], dict):
        raise ConfigError("junctions: expected an object")
    junctions = {}
    for jid, row in doc["junctions"].items():
        _ident(jid, f"junctions.{jid}")
        if not isinstance(row, dict) or set(row) != {"shape", "laid"}:
            raise ConfigError(f"junctions.{jid}: expected exactly ['laid', 'shape']")
        if row["shape"] not in SHAPES:
            raise ConfigError(f"junctions.{jid}.shape: expected one of {list(SHAPES)}, got {row['shape']!r}")
        junctions[jid] = {"shape": row["shape"], "laid": _coerce(row["laid"], bool, f"junctions.{jid}.laid")}
    if not isinstance(doc["uturns"], dict):
        raise ConfigError("uturns: expected an object")
    uturns = {_ident(k, f"uturns.{k}"): _site_uturn(k, v, vehicle) for k, v in doc["uturns"].items()}
    clash = set(junctions) & set(uturns)
    if clash:
        raise ConfigError(f"{sorted(clash)} are both junctions and U-turns")
    out["JUNCTIONS"], out["UTURNS"] = junctions, uturns

    if not isinstance(doc["circuits"], dict) or not doc["circuits"]:
        raise ConfigError("circuits: expected a non-empty object")
    circuits, stations = {}, {}
    for cid, raw in doc["circuits"].items():
        _ident(cid, f"circuits.{cid}")
        items = _site_items(cid, raw, junctions, uturns)
        stops = [it["id"] for it in items if it["kind"] == "stop"]
        if items[0] != {"kind": "stop", "id": home} or items[-1] != {"kind": "stop", "id": home}:
            raise ConfigError(f"circuits.{cid}: must start and end with {{'stop': '{home}'}}")
        if stops.count(home) != 2:
            raise ConfigError(f"circuits.{cid}: {home} only at the start and the end")
        if stops.count(attach) != 1:
            raise ConfigError(f"circuits.{cid}: passes {attach} {stops.count(attach)} times, expected once")
        inner = stops[1:-1]
        dup = sorted({s for s in inner if inner.count(s) > 1})
        if dup:
            raise ConfigError(f"circuits.{cid}: stop(s) {dup} twice - one station is one place on one pass")
        if stops.index(attach) != 1:
            raise ConfigError(f"circuits.{cid}: {attach} must be the first stop after {home} "
                              f"(trolleys attach before any machine)")
        turns = [it["id"] for it in items if it["kind"] == "uturn"]
        if len(set(turns)) != len(turns):
            raise ConfigError(f"circuits.{cid}: a U-turn twice")
        legs, before, after = _site_legs(cid, items, junctions, home)
        for leg in legs:
            for sid in leg["stops"]:
                if sid in stations and stations[sid] != leg["id"]:
                    raise ConfigError(f"circuits.{cid}: station {sid} lies on {leg['id']} here and on "
                                      f"{stations[sid]} in another circuit - one station, one place")
                stations[sid] = leg["id"]
        circuits[cid] = {"items": items, "legs": legs, "home_depart": before, "home_arrive": after}
    out["CIRCUITS"] = circuits
    out["STATIONS"] = stations

    if not isinstance(doc["groups"], dict) or not doc["groups"]:
        raise ConfigError("groups: expected a non-empty object")
    groups, machines = {}, {}
    for name, g in doc["groups"].items():
        where = f"groups.{name}"
        _ident(name, where)
        if not isinstance(g, dict) or set(g) != {"circuit", "machines"}:
            raise ConfigError(f"{where}: expected exactly ['circuit', 'machines']")
        cid = g["circuit"]
        if cid not in circuits:
            raise ConfigError(f"{where}.circuit: {cid!r} is not in circuits")
        items = circuits[cid]["items"]
        pos = {it["id"]: k for k, it in enumerate(items) if it["kind"] == "stop"}
        pos[home] = 0
        turn_at = [k for k, it in enumerate(items) if it["kind"] == "uturn"]
        if not isinstance(g["machines"], dict) or not g["machines"]:
            raise ConfigError(f"{where}.machines: expected a non-empty object")
        rows = {}
        for m, act in g["machines"].items():
            mw = f"{where}.machines.{m}"
            _ident(m, mw)
            if m in machines:
                raise ConfigError(f"{mw}: machine {m} is already in group {machines[m]} - a machine "
                                  f"belongs to one group")
            if not isinstance(act, dict) or set(act) not in ({"swap"}, {"release", "take"}):
                raise ConfigError(f"{mw}: expected {{'swap': stop}} or {{'release': stop, 'take': stop}}")
            for kind, sid in act.items():
                if sid in (home, attach) or sid not in pos:
                    raise ConfigError(f"{mw}.{kind}: stop {sid!r} is not a machine stop on {cid}")
            if "release" in act:
                if not turn_at:
                    raise ConfigError(f"{mw}: release/take needs a U-turn on {cid} between them")
                r, t = pos[act["release"]], pos[act["take"]]
                if not r < turn_at[0] < t:
                    raise ConfigError(f"{mw}: the release ({act['release']}) must come before the U-turn and "
                                      f"the take-back ({act['take']}) after it - the AGV never pivots "
                                      f"with a trolley attached")
            rows[m] = dict(act)
            machines[m] = name
        groups[name] = {"circuit": cid, "machines": rows}
    out["GROUPS"], out["MACHINES"] = groups, machines
    return out


def parse_site(doc, vehicle=None):
    """A site mission document -> the validated names. MissionError on any problem."""
    if vehicle is None:
        from agv_core import config as vehicle  # noqa: PLC0415
    if not is_site(doc):
        raise MissionError(f"not a site mission (schema must be {SITE_SCHEMA!r})")
    try:
        return _parse_site(doc, vehicle)
    except MissionError:
        raise
    except ConfigError as e:
        raise MissionError(str(e)) from None


def _read_doc(name, directory=None):
    if not isinstance(name, str) or not name or "/" in name or name.startswith("."):
        raise MissionError(f"not a mission name: {name!r}")
    path = os.path.join(directory or MISSION_DIR, f"{name}.json")
    try:
        with open(path) as f:
            return json.load(f)
    except OSError:
        raise MissionError(f"no mission {name!r} ({path})") from None
    except ValueError as e:
        raise MissionError(f"mission {name!r}: not valid JSON ({e})") from None


def load_site(name, vehicle=None, directory=None):
    """missions/<name>.json as a site mission."""
    out = parse_site(_read_doc(name, directory), vehicle)
    if out["MISSION_NAME"] != name:
        raise MissionError(f"mission_name {out['MISSION_NAME']!r} does not match the file name {name!r}")
    return out


def list_site_missions(directory=None):
    d = directory or MISSION_DIR
    out = []
    try:
        names = sorted(f[:-5] for f in os.listdir(d) if f.endswith(".json"))
    except OSError:
        return out
    for n in names:
        try:
            if is_site(_read_doc(n, d)):
                out.append(n)
        except MissionError:
            continue
    return out
