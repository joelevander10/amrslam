"""One LINE run's mission state: the RFID tag table, branches, marker speed zone, stops, U-turn.

Schema v2 (2026-10-08): the mission is the site's tag reference table
(agv_core.mission). Pure: no ROS, no clock of its own, no I/O. FollowJob owns
one, feeds it a frozen RFID snapshot and the follower each tick, and acts on
what it asks for:

    stop          {"kind": "station", "tag", "where", "role", "distance_m"} while
                  a measured stop is in progress or parked at a stop
    uturn_req     {"tag", "direction", "approach_mps", "decel_m", "decel_rpm_s",
                  "max_approach_m", "phase", "travel_m", "level", "gone"} from the
                  U-turn tag until the turn ends. phase: approach (slow to the
                  creep within decel_m, follow until the tape is gone) -> stopping
                  (command zeroed at once) -> the pivot (self.uturn, amr_line.uturn)
    link_lost     why the RFID stream can no longer be trusted while driving
                  (link down, reconnect, buffer overrun): the job holds the
                  vehicle for a person. A missed tag here means a station or the
                  U-turn driven past.
    fault         a mission error the run cannot continue from
    events        (level, text) pairs for the operator, drained by the node

Which reads count:
    departed tag  the stop just left (or, at the first Start, the stop the
                  vehicle is parked past) is suppressed until it has left the
                  antenna field - however slowly the vehicle pulls away
    U-turn re-pass  the U-turn's own tag, driven back over after the pivot, is
                  suppressed once within the approach distance plus a margin
    ignore_s      the table row's window (amr_line.tag_table.TagTable)
    held          reads while stopped or held cannot start a stop or a U-turn

Speed (tracked-speed-plan-1, 2026-10-08; MLS markers since 2026-10-09): NORMAL unless
the high zone (amr_line.speed_zone) grants HIGH. RFID never touches the speed: the
zone is a pair of MLS marker codes, fed by markers() and acted on only while driving,
on a clean read (line good, within MARKER_MAX_LCP2_MM of the tape centre - a pivot
over a marker reads a code off the tape, seen 2026-10-09). The marker stream going
down, or a marker event lost on the way, takes ARMED or HIGH away. Marker events are
always on (2026-10-23): a mission without a high_zone is the way to run NORMAL only.
No RFID tag sets the speed (2026-10-09): a stop parks HIGH and the departure resumes it,
an RFID-link or protective-field hold parks it and the resume restores it
(park_high/resume_high). The U-turn tag is the one exception: the pivot reverses the
vehicle, so the run goes on NORMAL until the markers grant HIGH again. A U-turn, any
other hold or a curve-guard trip takes HIGH away. Every distance it counts is WHEEL
travel handed in by the job (advance()).

Branch tags are honoured held or driving: a branch exit passed during a hold must
still clear its latch.

No mission (or "empty") is plain line following: nothing here ever asks for
anything, and the follower runs at AUTO_RPM with the default branch order.

Marker run (mission marker_run, 2026-10-18, tracked plan 3 section 7.5): no RFID. Phase
"out" until the pivot: the first uturn_code starts the same U-turn a u_turn tag would.
Phase "back" after it: the uturn_code re-passed within the approach distance is skipped,
the first home_code ends the run at home (role home), and any later uturn_code is the
BACKSTOP - the home marker was missed - and faults the run at once. The marker stream
going down, or marker events lost, while driving outside a U-turn sets marker_lost: the
job holds for a person (a missed marker here is the home stop driven past). Reads are
acted on whether or not they are clean (lcp2, line_good): every action they start is a
slowdown or a stop.
"""

from agv_core import config as vehicle

from amr_line import branch, speed_zone, tag_table, uturn

INFO, WARN = "info", "warn"

# Consecutive ticks with no track under the sensor that mean "the tape ended" on a
# U-turn approach: 2 ticks at 50 Hz is 4 mm at 0.1 m/s - a flicker is not an end.
TAPE_GONE_TICKS = 2
# The approach slowdown stays below the drives' own deceleration (6084h) so the
# drive never reshapes it; arriving faster than planned it simply takes longer.
APPROACH_FRACTION_OF_DRIVE = 0.95
# A zone marker counts only this close to the tape centre (LCP2). A straight pass reads
# within a few mm (2026-10-09: +2, -3, -1 mm); a read during a pivot came at +121 mm.
MARKER_MAX_LCP2_MM = 30


def approach_rate(v_rpm, creep_rpm, decel_m):
    """r/min/s that takes v_rpm down to creep_rpm over decel_m (None: already there)."""
    if v_rpm <= creep_rpm or decel_m <= 0:
        return None
    rate = (v_rpm ** 2 - creep_rpm ** 2) * vehicle.MPS_PER_RPM / (2.0 * decel_m)
    return min(rate, APPROACH_FRACTION_OF_DRIVE * vehicle.DECEL_RPM_S)


def u_turn_error(sensor):
    """Nearest track's error in the follower's sign, and the MLS track level."""
    sensor = sensor or {}
    usable = [t for t in sensor.get("tracks") or () if abs(t["pos_mm"]) <= vehicle.SENSOR_MAX_MM]
    level = sensor.get("track_level")
    if not usable:
        return None, level
    pos = min(usable, key=lambda t: abs(t["pos_mm"]))["pos_mm"]
    return (-pos if vehicle.INVERT_ERROR else pos), level


class TapeRun:
    def __init__(self, mission=None, destination=None):
        from agv_core import mission as missions  # noqa: PLC0415

        m = mission or missions.parse(missions.EMPTY)
        self.mission = m
        self.name = m["MISSION_NAME"]
        self.table = tag_table.TagTable(m, destination)
        self.branch = branch.BranchEngine(
            m["BRANCH_LATCH"], positive_is_left=vehicle.BRANCH_POSITIVE_IS_LEFT, default=m["BRANCH_DEFAULT"]
        )
        self.events: list[tuple[str, str]] = []
        self.speed = speed_zone.SpeedZone(m["HIGH_ZONE"], normal_rpm=vehicle.AUTO_RPM,
                                          rpm_per_mps=vehicle.RPM_PER_MPS,
                                          drive_decel_rpm_s=vehicle.DECEL_RPM_S)
        self.guard = speed_zone.CurveGuard(vehicle.CURVE_GUARD_KAPPA, vehicle.CURVE_GUARD_E_MM)
        self.fault: str | None = None
        self.link_lost: str | None = None
        self.stop: dict | None = None
        self.uturn_req: dict | None = None
        self.uturn: uturn.UTurn | None = None
        self.uturn_last: dict | None = None
        self.uturn_skip: str | None = None
        self.uturn_skip_m = 0.0
        self.uturn_rearm_m = 0.0
        self.last_encounter: dict | None = None
        self._cursor = 0
        self._generation = None
        self._departure_tag: str | None = None
        self._departure_at: float | None = None
        self.mrun = m.get("MARKER_RUN")
        self.mrun_phase = "out" if self.mrun else None
        self.marker_lost: str | None = None
        self._mrun_skip_m = 0.0       # travel left in which the U-turn marker re-pass is ignored

    @property
    def active(self):
        """Does this run act on any tag at all?"""
        return bool(self.mission["TAGS"] or self.mission["BRANCH_LATCH"] or self.mrun)

    @property
    def needs_link(self):
        """Must the RFID link be up to start and to keep driving?

        Stops and branches: a missed tag is a station driven past or a wrong
        turn. Never the speed zone: that is the MLS markers' (markers()). U-turn
        tags alone (plain line following, missions/empty.json) do not: a missed
        U-turn tag ends at the tape end, where the follower's line-loss stop
        already halts the vehicle.
        """
        return bool(self.mission["BRANCH_LATCH"]
                    or any(r["action"] != "u_turn" for r in self.mission["TAGS"].values()))

    # -- RFID cursor -------------------------------------------------------
    def sync(self, rfid):
        """Not running: everything read so far is history, not route input."""
        rfid = rfid or {}
        self._cursor = rfid.get("encounter_seq", 0)
        self._generation = rfid.get("generation", 0)
        self.link_lost = None

    def depart(self, now, rfid, resumed_tag=None):
        """Start (or resume from a stop) committed.

        resumed_tag is the stop being left. At the first Start there is none, so
        the last tag encountered stands in when it is a stop: a vehicle parked
        past Home must not stop at Home again on its first read of the tag.
        """
        rfid = rfid or {}
        if resumed_tag is None:
            enc = rfid.get("encounters") or ()
            last = enc[-1][1] if enc else None
            row = self.table.row(last) if last else None
            if row is not None and row["action"] == "stop":
                resumed_tag = last
        self.sync(rfid)
        if resumed_tag is not None:
            self._departure_tag, self._departure_at = resumed_tag, now
            self.table.departed(now, resumed_tag)
        self.stop = None
        ev = self.speed.resume()
        if ev:
            self.events.append(ev)
        if self.table.destination:
            self.events.append((INFO, f"departing for {self.table.destination}"))

    @property
    def needs_markers(self):
        """Must the marker stream be up to start and keep driving? A marker run's U-turn
        and home stop are markers."""
        return self.mrun is not None

    def advance(self, distance_m):
        """Wheel travel since the last tick (encoder; commanded only as a fallback)."""
        ev = self.speed.travel(distance_m)
        if ev:
            self.events.append(ev)
        if self._mrun_skip_m > 0.0:
            self._mrun_skip_m = max(0.0, self._mrun_skip_m - max(0.0, distance_m))
        if self.uturn_skip is not None:
            self.uturn_skip_m += max(0.0, distance_m)
            if self.uturn_skip_m > self.uturn_rearm_m:
                self.uturn_skip = None

    def _lose_link(self, why):
        if self.needs_link and self.link_lost is None:
            self.link_lost = why

    def scan(self, now, rfid, driving, follower):
        """Consume the frozen, ordered batch once. Returns the tags, in order,
        for the branch ladder - which sees every one, held or not."""
        rfid = rfid or {}
        seq = rfid.get("encounter_seq", 0)
        generation = rfid.get("generation", 0)
        changed = self._generation is not None and generation != self._generation
        connected = bool(rfid.get("comms_ok", False))
        if changed or not connected:
            self._generation, self._cursor = generation, seq
            self._lose_link("RFID link lost" if not connected
                            else "RFID link re-established mid-run: a tag may have been missed")
            return []
        self._generation = generation
        pending = [(n, t) for n, t in rfid.get("encounters", ()) if n > self._cursor]
        if pending and pending[0][0] != self._cursor + 1:
            self._cursor = seq
            self._lose_link("RFID encounter buffer overrun: tags were missed")
            return []
        self._cursor = seq
        # The departed tag stays suppressed until it has left the field.
        age = rfid.get("tag_age_s")
        clear = vehicle.RFID_TAG_CLEAR_S
        if (self._departure_tag is not None and now - self._departure_at >= clear
                and (age is None or age >= clear)):
            self._departure_tag = None
        tags = []
        for number, tag in pending:
            tags.append(tag)
            if self._departure_tag is not None:
                if tag == self._departure_tag:
                    self._record(now, number, tag, "suppressed", "departed stop, still in the field")
                    continue
                self._departure_tag = None
            row = self.table.row(tag)
            if row is None:
                self._record(now, number, tag, "no tag rule")
                continue
            left = self.table.window_left(now, tag)
            if left > 0:
                self._record(now, number, tag, "ignored", f"ignore window, {left:.1f} s left")
                continue
            # Reads while stopped cannot start anything: a dwell or a recovery
            # must not queue up stops.
            if not driving or self.stop is not None or self.uturn_req is not None:
                self._record(now, number, tag, "suppressed", "held or stopped")
                continue
            if row["action"] == "u_turn":
                self._u_turn_tag(now, number, row)
                continue
            # Station tags never touch the speed zone (operator, 2026-10-08): passing a
            # machine is not a speed event, and a stop parks HIGH for the departure.
            applies, why = self.table.stop_applies(row)
            self.table.acted(now, tag)
            if not applies:
                self._record(now, number, tag, f"passed {row['label']}", why)
                self.events.append((INFO, f"passed {row['label']} (tag {tag}): {why}"))
                continue
            self._begin_stop(row, follower)
            self._record(now, number, tag, f"stop {row['label']}")
        return tags

    def _record(self, now, sequence, tag, action, reason=None):
        self.last_encounter = dict(sequence=sequence, tag=tag, action=action, reason=reason, at=now)

    # -- stops ---------------------------------------------------------------
    def _begin_stop(self, row, follower):
        dist = row["stop_distance_m"]
        # Every stop parks HIGH the same way, Home included: the stop tag decides where to
        # stop, never the speed. A run from Home is a new run and starts NORMAL anyway.
        if self.speed.park(f"stop {row['label']}"):
            self.events.append((INFO, f"speed NORMAL: {self.speed.reason}"))
        rate = follower.begin_measured_stop(dist)
        if row["role"] == "destination":
            self.table.reached = True
        self.stop = {"kind": "station", "tag": row["tag"], "where": row["label"], "role": row["role"],
                     "distance_m": dist}
        then = "run complete" if row["role"] == "home" else "press Start to go on"
        self.events.append((INFO, f"{row['label']} (tag {row['tag']}) - stopping over {dist:.2f} m"
                            + (f" ({rate:.0f} r/min/s)" if rate else "") + f", {then}"))

    # -- speed ------------------------------------------------------------------
    def markers(self, reads, ok, driving, follower=None):
        """This tick's new MLS markers, [(code, direction, lcp2_mm, gap, line_good)] in
        order (amr_line.marker_reader), and whether the stream is up.

        Only the zone's two codes act, and only a clean read while driving. Anything
        that may have hidden a zone marker - the stream down, events lost - takes ARMED
        and HIGH away: every failure ends at NORMAL. A marker run routes them instead
        (_marker_run); follower is needed there for the home stop.
        """
        if self.mrun is not None:
            self._marker_run(reads, ok, driving, follower)
            return
        z = self.speed
        if not z.enabled:
            return
        zone = (z.cfg["outer"], z.cfg["inner"])
        if not ok:
            if z.state != speed_zone.NORMAL:
                self.drop_high("MLS markers down: a zone marker could be missed")
            return
        for code, _direction, lcp2, gap, line_good in reads:
            if gap and z.state != speed_zone.NORMAL:
                self.drop_high(f"{gap} marker event(s) lost: a zone marker may have been missed")
            if code not in zone:
                continue
            if not driving or self.stop is not None or self.uturn_req is not None:
                continue
            if not line_good or abs(lcp2) > MARKER_MAX_LCP2_MM:
                why = ("line not good" if not line_good
                       else f"{lcp2:+d} mm off the tape centre (> {MARKER_MAX_LCP2_MM} mm)")
                self.events.append((WARN, f"zone marker {code} ignored: {why}"))
                continue
            ev = z.cue(code)
            if ev:
                self.events.append(ev)

    def _marker_run(self, reads, ok, driving, follower):
        mr = self.mrun
        busy = self.stop is not None or self.uturn_req is not None
        if driving and not busy and self.marker_lost is None:
            if not ok:
                self.marker_lost = "MLS markers down: the U-turn or home marker could be missed"
            elif any(gap for _c, _d, _l, gap, _g in reads):
                self.marker_lost = "marker events lost: the U-turn or home marker may have been missed"
        for code, _direction, lcp2, _gap, line_good in reads:
            if code not in (mr["uturn_code"], mr["home_code"]):
                continue
            where = f"marker {code} at {lcp2:+d} mm" + ("" if line_good else ", line not good")
            if not driving or busy:
                why = "held" if not driving else "stopping or turning"
                self.events.append((INFO, f"{where} ignored: {why}"))
                continue
            if self.mrun_phase == "out":
                if code == mr["uturn_code"]:
                    self._start_u_turn({"tag": f"marker {code}", "direction": mr["direction"],
                                        "approach_mps": mr["approach_mps"], "decel_m": mr["decel_m"],
                                        "max_approach_m": mr["max_approach_m"]}, f"U-turn marker ({where})")
                    busy = True
                # the home marker on the way out: the departure passes it again
                continue
            if code == mr["uturn_code"]:
                if self._mrun_skip_m > 0.0:
                    self.events.append((INFO, f"{where}: own U-turn marker passed after turning"))
                    continue
                self.fault = (f"home marker missed: {where} read on the way back (the backstop strip "
                              f"past {mr['label']}); check the home marker, jog back, Reset")
                return
            if follower is None:
                continue
            self._begin_stop({"tag": f"marker {code}", "label": mr["label"], "role": "home",
                              "stop_distance_m": mr["stop_distance_m"]}, follower)
            busy = True

    def parked(self, distance_m):
        """Wheel travel while parked at a station: taken off the kept budget, and
        without counts the budget cannot be trusted any more."""
        if self.speed.parked_left is None:
            return
        if distance_m is None:
            self.speed.drop("wheel travel unavailable while stopped")
            self.events.append((INFO, "departs NORMAL: wheel travel unavailable while stopped"))
        else:
            self.speed.travel(distance_m)

    def park_high(self, reason):
        """A protective-field or RFID-link hold while running (operator, 2026-10-09): HIGH comes back on
        the resume, like a station departure, with the budget less the wheel travel while
        held. A measured station stop in progress already parked its budget: left alone."""
        if self.stop is not None:
            return
        if self.speed.park(reason):
            self.events.append((INFO, f"speed NORMAL: {self.speed.reason}"))

    def resume_high(self):
        """The hold park_high() began is over. Not mid-stop: a station departs on Start."""
        if self.stop is not None:
            return
        ev = self.speed.resume()
        if ev:
            self.events.append(ev)

    def drop_high(self, reason, urgent=False):
        why = self.speed.drop(reason, urgent=urgent)
        if why:
            self.events.append((WARN if urgent else INFO, f"speed NORMAL: {why}"))

    @property
    def high(self):
        """HIGH granted by the marker zone. No RFID tag touches it (2026-10-09): junction
        slow zones are refused by the mission loader."""
        return self.speed.high

    def change_rate(self):
        """r/min/s between NORMAL and HIGH (high_ramp_s), None without a high zone."""
        if not self.speed.enabled:
            return None
        return abs(vehicle.AUTO_HIGH_RPM - vehicle.AUTO_RPM) / vehicle.HIGH_RAMP_S

    # -- branches ------------------------------------------------------------
    def steer(self, sensor, tags, followed_mm):
        """Scan each new tag, then the no-tag fork scan. Returns (choice, high)."""
        nlcp = (sensor or {}).get("nlcp")
        for tag in tags:
            self._branch_scan(nlcp, tag)
        self._branch_scan(nlcp, None)
        was = self.branch.unhonoured
        _, choice = self.branch.choose(nlcp, (sensor or {}).get("tracks"), followed_mm)
        if self.branch.unhonoured and not was:
            self.events.append((WARN, f"branch {choice} ordered, but that side is not in this "
                                      f"diverter - carrying straight on"))
        return choice, self.high

    def _branch_scan(self, nlcp, tag):
        was, was_slow, was_forks = self.branch.ladder.intent(), self.branch.slow, self.branch.forks
        intent = self.branch.scan(tag, nlcp)
        if intent != was:
            if intent != branch.STRAIGHT:
                self.events.append((INFO, f"branch {intent} (tag {tag})"))
            elif self.branch.forks != was_forks:
                self.events.append((INFO, f"branch {was} taken - order consumed at the fork"))
            else:
                self.events.append((INFO, f"branch cleared (tag {tag})"))
        if self.branch.slow != was_slow:
            self.events.append((INFO, f"slow zone {'entered' if self.branch.slow else 'left'} (tag {tag})"))

    # -- U-turn ----------------------------------------------------------------
    def _u_turn_tag(self, now, number, row):
        tag = row["tag"]
        if tag == self.uturn_skip:
            self.uturn_skip = None
            self._record(now, number, tag, "suppressed", "own U-turn tag passed after turning")
            return
        self.table.acted(now, tag)
        missed = self.table.note_u_turn()
        if missed:
            self.events.append((WARN, f"destination {missed} not reached before the U-turn: its tag "
                                      f"was missed. It will not be served from the return leg."))
        self._start_u_turn(row, f"U-turn tag {tag}")
        self._record(now, number, tag, f"u-turn {row['direction']}")

    def _start_u_turn(self, row, what):
        """The U-turn approach, from a tag or a marker: slow to the creep, stop at the tape end."""
        self.drop_high("U-turn")
        self.uturn_req = {"tag": row["tag"], "direction": row["direction"],
                          "approach_mps": row["approach_mps"],
                          "decel_m": row["decel_m"], "decel_rpm_s": None,
                          "max_approach_m": row["max_approach_m"], "phase": "approach",
                          "travel_m": 0.0, "level": None, "gone": 0}
        self.uturn = None
        self.uturn_last = None
        self.events.append((INFO, f"{what} - slowing to {row['approach_mps']:.2f} m/s within "
                                  f"{row['decel_m']:.2f} m, stopping where the tape ends"))

    def approach(self, sensor, has_track, distance_m):
        """One tick of the approach to the tape end. Returns a fault reason or None.

        The tape is gone after TAPE_GONE_TICKS ticks without a track: phase
        'stopping', and the job zeroes the command that same tick - whether or
        not the creep speed was reached (no line-loss grace, no ramp).
        """
        req = self.uturn_req
        if req["phase"] != "approach":
            return None
        req["travel_m"] += max(0.0, distance_m)
        if has_track:
            req["gone"] = 0
            level = (sensor or {}).get("track_level")
            if level is not None:
                req["level"] = level
        else:
            req["gone"] += 1
            if req["gone"] >= TAPE_GONE_TICKS:
                req["phase"] = "stopping"
                self.events.append((INFO, f"tape ended {req['travel_m']:.2f} m after the U-turn tag - "
                                          f"stopping now to pivot {req['direction']}"))
                return None
        if req["travel_m"] > req["max_approach_m"]:
            return (f"U-turn: the tape did not end within {req['max_approach_m']:.1f} m of tag "
                    f"{req['tag']}")
        return None

    def begin_spin(self, counts, counts_per_rev):
        """Stopped past the tape end: start the pivot. Returns why not, or None.

        No tape is under the sensor here - that is the trigger - so the start
        level is the last one seen on the approach, and the spin begins 'lost':
        the sensor ahead of the axle meets the tape again only near 180 deg.
        """
        req = self.uturn_req
        if not counts_per_rev:
            return "U-turn needs the drive encoder scale (counts_per_wheel_rev on /wheel_states)"
        if req["level"] is None:
            return "no tape level recorded on the approach to the U-turn"
        if counts is None:
            return "U-turn encoder position (/wheel_states counts) unavailable"
        self.uturn = uturn.UTurn(req["direction"], counts_per_rev, counts, req["level"])
        req["phase"] = "spin"
        self.events.append((INFO, f"U-turn {req['direction']} - pivoting at "
                                  f"{vehicle.AUTO_U_TURN_RPM:.0f} r/min, track level {req['level']}"))
        return None

    def finish_u_turn(self):
        req, angle = self.uturn_req, self.uturn.angle_deg
        self.uturn_last = dict(self.uturn_snapshot() or {}, active=False)
        self.uturn = self.uturn_req = None
        # Its own tag lies about the approach distance back along the tape.
        self.uturn_skip, self.uturn_skip_m = req["tag"], 0.0
        self.uturn_rearm_m = req["travel_m"] + uturn.REARM_MARGIN_M
        if self.mrun is not None:
            # Back: the U-turn marker is re-passed within the approach distance; after that it
            # is the backstop.
            self.mrun_phase = "back"
            self._mrun_skip_m = req["travel_m"] + uturn.REARM_MARGIN_M
        self.events.append((INFO, f"U-turn {req['direction']} complete at {angle:.0f} deg by encoder, "
                                  f"centred - resuming"))

    def abort_u_turn(self):
        self.uturn = self.uturn_req = None

    def uturn_snapshot(self):
        req = self.uturn_req
        if req is None:
            return self.uturn_last
        snap = {"active": True, "phase": req["phase"], "direction": req["direction"],
                "angle_deg": 0.0, "tape_lost": False, "reason": None}
        if self.uturn is not None:
            snap.update(self.uturn.snapshot())
        snap["tag"] = req["tag"]
        return snap

    def drain_events(self):
        out, self.events = self.events, []
        return out
