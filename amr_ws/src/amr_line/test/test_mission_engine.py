"""The mission engine on the line layer (schema v2, 2026-10-08: the RFID tag table).

The real job, follower and TapeRun, fed dataclass inputs; no ROS. The missions
are TEST documents validated by the real agv_core.mission.parse, shaped like the
site's table (missions/line-a.json): Home, an always-stop, a track-end U-turn
and destination stops. The high zone is a pair of MLS marker codes (2026-10-09):
RFID decides where to stop, the markers decide speed.
"""
import copy
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_line")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from agv_core import config as vehicle  # noqa: E402
from agv_core import kinematics  # noqa: E402
from agv_core import mission as missions  # noqa: E402

from amr_line import autopilot, runtime, uturn  # noqa: E402
from amr_line import job as lj  # noqa: E402

runtime.load_from_profile()

LEASE_LINE = 8
CENTRED = {"tracks": [{"index": 2, "pos_mm": 0, "width": 10}], "has_track": True, "nlcp": 2, "track_level": 5}
NO_TAPE = {"tracks": [], "has_track": False, "nlcp": 0, "track_level": 0}
COUNTS_PER_MOTOR_REV = 10000.0
PER_REV = COUNTS_PER_MOTOR_REV * vehicle.GEAR_RATIO

HOME, TROLLEY, UTURN = "0010", "0020", "0180"
INNER, OUTER = 1, 2      # MLS marker codes of the zone pair (missions/line-a.json, swapped 2026-10-09)
MRU1, MRU2 = "0110", "0120"


def stop(tag, role, label, ignore_s=4):
    return {"tag": tag, "action": "stop", "ignore_s": ignore_s, "stop_distance_m": 0.5,
            "role": role, "label": label}


SITE_ROWS = [
    stop(HOME, "home", "Home", 2),
    stop(TROLLEY, "always", "Trolley release", 2),
    {"tag": UTURN, "action": "u_turn", "ignore_s": 2, "direction": "cw",
     "approach_mps": 0.1, "decel_m": 0.3, "max_approach_m": 2.0},
    stop(MRU1, "destination", "MRU1"),
    stop(MRU2, "destination", "MRU2"),
]


def doc(rows=(), **over):
    d = copy.deepcopy(missions.EMPTY)
    d["mission_name"] = "test"
    d["tags"] = copy.deepcopy(list(rows))
    d.update(over)
    return missions.parse(d)


SITE = doc(SITE_ROWS)
# The same table with a high zone: shared codes, an 8 m shortest straight -> a 5.65 m budget.
ZONE = {"shortest_straight_m": 8.0, "pair_spacing_m": 0.5, "outer_code": OUTER, "inner_code": INNER}
ZONED = doc(SITE_ROWS, high_zone=ZONE)


class World:
    """Clock, RFID stream and wheel feedback around one job.

    Wheel counts integrate the job's own command, so the travel-since-tag that
    proves "at Home" is the travel the vehicle actually made. The U-turn test
    drives the counts by hand instead (integrate=False).
    """

    def __init__(self, mission=None, destination=None):
        self.t = 100.0
        self.seq = 0
        self.gen = 0
        self.comms = True
        self.enc = []
        self.counts = (0, 0)
        self.integrate = True
        self.still = False
        self.yaw = 0.0          # gyro yaw rate fed to the curve guard; None = stale
        self.v_meas = 0.0       # odometry speed: the body speed of the last command
        self.mk_ok = True       # /amr/line_marker: the stream, and the events on it
        self.mk_seq = 0
        self.mk_enc = []
        self.job = lj.FollowJob(autopilot.LineFollower(), premove_s=0.6)
        ok, why = self.job.set_mission(mission, destination)
        assert ok, why
        self.tick()  # the layer is up before anything is read, as on the vehicle

    def rfid(self):
        return {"comms_ok": self.comms, "encounter_seq": self.seq, "generation": self.gen,
                "tag_age_s": None, "encounters": list(self.enc)}

    def read(self, tag):
        self.seq += 1
        self.enc.append((self.seq, tag))

    def markers(self):
        return {"ok": self.mk_ok, "status": "ok" if self.mk_ok else "no marker data", "generation": 0,
                "base": 0, "encounters": list(self.mk_enc)}

    def mark(self, code, lcp2=0, line_good=True, lost=0):
        """An MLS marker event; `lost` events vanish on the topic before it."""
        self.mk_seq += 1 + lost
        self.mk_enc.append((self.mk_seq, code, 0, lcp2, line_good))

    def inputs(self, **over):
        base = dict(
            now=self.t, dt=0.02, sensor=CENTRED, sensor_age_s=0.005, track_ok=True, track_cause="",
            panel_valid=True, panel_auto=True, start_edge=False, start_edge_t=None, reset_edge=False,
            lease_allowed=LEASE_LINE, lease_line=LEASE_LINE, authority=("sup", 1), torque_off=False,
            field_clear=True, drives_fresh=True, rfid=self.rfid(), counts=self.counts,
            counts_per_rev=PER_REV, wheels_still=self.still,
            yaw_rate=self.yaw, v_meas=self.v_meas, markers=self.markers(),
        )
        base.update(over)
        return lj.Inputs(**base)

    def tick(self, **over):
        self.t += 0.02
        left, right = self.job.tick(self.inputs(**over))
        self.v_meas = kinematics.wheels_to_body(left, right)[0]
        if self.integrate:
            step = lambda rpm: int(round(rpm / 60.0 * 0.02 * COUNTS_PER_MOTOR_REV))  # noqa: E731
            self.counts = (self.counts[0] + step(left), self.counts[1] + step(right))
        return left, right

    def move(self, metres):
        """A manual jog: wheel travel the job did not command."""
        c = metres / (3.141592653589793 * vehicle.WHEEL_DIA_M) * PER_REV
        self.counts = (self.counts[0] + int(c), self.counts[1] + int(c))
        self.tick()

    def press_start(self):
        self.t += 0.1
        self.job.tick(self.inputs(start_edge=True, start_edge_t=self.t - 0.05))

    def start(self):
        """Start, then the pre-move warning (ARMED, nothing commanded), then RUNNING."""
        self.press_start()
        assert self.job.state == lj.ARMED, self.job.reason
        assert self.until(lambda: self.job.state == lj.RUNNING, 1.0), self.job.reason

    def drive(self, seconds, **over):
        for _ in range(int(seconds / 0.02)):
            self.tick(**over)

    def until(self, cond, seconds=10.0, **over):
        for _ in range(int(seconds / 0.02)):
            self.tick(**over)
            if cond():
                return True
        return False

    def park_at_home(self):
        """Jog over the Home tag in IDLE, as an operator would."""
        self.read(HOME)
        self.tick()
        self.move(0.2)
        assert self.job.at_home()[0], self.job.at_home()


def at_rest_stopped(w):
    return w.until(lambda: w.job.state != lj.RUNNING)


# -- plain line following --------------------------------------------------------

def test_no_mission_is_plain_line_following():
    w = World(None)
    w.start()
    w.read("0010")  # a tag means nothing without a mission
    w.drive(2.0)
    assert w.job.state == lj.RUNNING and w.job.tape.stop is None
    assert w.job.mission_snapshot()["mission"] == ""


def test_plain_line_following_does_not_need_the_rfid_link():
    w = World(None)
    w.start()
    w.comms = False
    w.drive(1.0)
    assert w.job.state == lj.RUNNING


# -- the job: destination and Home --------------------------------------------------

def test_a_destination_mission_refuses_start_without_a_destination():
    w = World(SITE)
    w.park_at_home()
    w.press_start()
    assert w.job.state == lj.IDLE and "no destination selected" in w.job.reason


def test_start_is_accepted_away_from_home():
    w = World(SITE, "MRU2")
    assert not w.job.at_home()[0]
    w.start()  # no tag read yet: anywhere on the line
    _run_to(w)
    w.read(MRU2)
    w.tick()
    assert w.job.tape.stop["where"] == "MRU2", "the destination is still served"
    w = World(SITE, "MRU2")
    w.park_at_home()
    w.move(1.0)  # pushed off Home
    w.start()
    assert w.job.state == lj.RUNNING


def test_an_unknown_destination_is_refused():
    w = World(SITE)
    ok, why = w.job.set_mission(SITE, "MRU9")
    assert not ok and "not a stop of this mission" in why
    ok, why = w.job.set_mission(None, "MRU1")
    assert not ok


def test_the_job_changes_only_in_idle_or_done():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    ok, why = w.job.set_mission(SITE, "MRU2")
    assert not ok and "clear first" in why
    w.job.clear()
    assert w.job.set_mission(SITE, "MRU2")[0]


def _run_to(w, seconds=3.0):
    w.drive(seconds)
    assert w.job.state == lj.RUNNING, w.job.reason


def _uturn(w):
    """From the U-turn tag: creep, tape ends, stop, pivot cw, reacquire, resume."""
    w.read(UTURN)
    w.tick()
    assert w.job.tape.uturn_req["phase"] == "approach"
    w.drive(2.0)  # down to the creep
    assert abs(w.job.diag["speed_target_rpm"] - 0.1 * vehicle.RPM_PER_MPS) < 1.0
    assert w.job.state == lj.RUNNING
    assert w.until(lambda: w.job.tape.uturn_req["phase"] == "stopping", 3.0, sensor=NO_TAPE), \
        "the end of the tape is the trigger"
    w.still = True
    w.integrate = False
    assert w.until(lambda: w.job.tape.uturn is not None, 3.0, sensor=NO_TAPE), w.job.reason
    base = w.counts
    for deg, sensor in ((40, NO_TAPE), (120, NO_TAPE), (175, CENTRED)):
        c = uturn.counts_for_angle(deg, PER_REV)
        w.counts = (base[0] + int(round(c)), base[1] + int(round(-c)))
        w.tick(sensor=sensor)
    assert w.job.tape.uturn.phase in (uturn.CENTER, uturn.SETTLE), w.job.tape.uturn.phase
    w.drive(vehicle.U_TURN_RESUME_DELAY_S + 0.2)
    assert w.job.tape.uturn_req is None and w.job.state == lj.RUNNING, w.job.reason
    w.still, w.integrate = False, True


def test_a_full_job_home_to_mru2_and_back():
    w = World(SITE, "MRU2")
    w.park_at_home()
    w.start()
    w.read(HOME)  # still in the field after Start
    w.tick()
    assert w.job.tape.stop is None and w.job.tape.last_encounter["action"] == "suppressed"
    _run_to(w)

    w.read(MRU1)
    w.tick()
    assert w.job.tape.stop is None, "not the destination: passed at speed"
    assert w.job.tape.last_encounter["action"] == "passed MRU1"

    w.drive(1.0)
    w.read(MRU2)
    w.tick()
    assert w.job.tape.stop["where"] == "MRU2"
    assert at_rest_stopped(w) and w.job.state == lj.HOLD and w.job.hold_cause == "station"
    w.drive(1.0)
    assert w.job.state == lj.HOLD, "only the physical Start moves it on"
    w.tick(start_edge=True, start_edge_t=w.t)
    w.drive(0.7)
    assert w.job.state == lj.RUNNING, w.job.reason
    _run_to(w)

    _uturn(w)
    _run_to(w, 1.0)
    w.read(MRU2)  # the return leg
    w.tick()
    assert w.job.tape.stop is None and "already served" in w.job.tape.last_encounter["reason"]
    w.drive(1.0)
    w.read(HOME)
    w.tick()
    assert w.job.tape.stop["role"] == "home"
    assert at_rest_stopped(w)
    assert w.job.state == lj.DONE and "MRU2 served" in w.job.reason, w.job.reason
    assert w.job.destination is None, "a finished job is spent"
    assert w.job.at_home()[0], w.job.at_home()

    w.press_start()
    assert w.job.state == lj.DONE and "no destination" in w.job.reason, "one PB press cannot repeat a job"
    assert w.job.set_mission(SITE, "MRU1")[0], "a new job is accepted at DONE"
    w.start()


def test_a_missed_destination_is_never_served_from_the_return_leg():
    w = World(SITE, "MRU2")
    w.park_at_home()
    w.start()
    _run_to(w)
    _uturn(w)  # MRU2's tag was never read outbound
    events = [text for _, text in w.job.tape.drain_events()]
    assert any("MRU2 not reached before the U-turn" in e for e in events)
    w.read(MRU2)
    w.tick()
    assert w.job.tape.stop is None
    w.drive(1.0)
    w.read(HOME)
    assert at_rest_stopped(w)
    assert w.job.state == lj.DONE and "MISSED" in w.job.reason


def test_the_always_stop_stops_every_pass():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.read(TROLLEY)
    assert at_rest_stopped(w) and w.job.hold_cause == "station"
    assert w.job.mission_snapshot()["stop_where"] == "Trolley release"


def test_the_ignore_window_swallows_a_second_read():
    w = World(SITE, "MRU2")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.read(MRU1)
    w.tick()
    w.drive(1.0)
    w.read(MRU1)  # a second distinct encounter inside the 4 s window
    w.tick()
    assert w.job.tape.last_encounter["action"] == "ignored"
    w.drive(3.5)
    w.read(MRU1)
    w.tick()
    assert w.job.tape.last_encounter["action"] == "passed MRU1", "the window expired"


# -- speed: NORMAL / HIGH zones (tracked-speed-plan-1) ---------------------------------

def _zoned_at_normal():
    w = World(ZONED, "MRU2")
    w.park_at_home()
    w.start()
    w.drive(4.0)
    assert w.job.diag["v_base"] == vehicle.AUTO_RPM, "a run starts and cruises at NORMAL"
    assert w.job.mission_snapshot()["high_speed"] is False
    return w


def _enter(w):
    w.mark(OUTER)
    w.tick()
    w.drive(0.3)
    w.mark(INNER)
    w.tick()
    assert w.job.tape.speed.high, w.job.tape.speed.reason
    return w


def test_normal_is_the_default_and_a_mission_without_a_zone_never_goes_high():
    w = World(SITE, "MRU2")
    w.park_at_home()
    w.start()
    w.drive(4.0)
    assert w.job.diag["v_base"] == vehicle.AUTO_RPM
    w.mark(OUTER)
    w.tick()
    w.drive(0.3)
    w.mark(INNER)
    w.tick()
    w.drive(1.0)
    assert w.job.diag["v_base"] == vehicle.AUTO_RPM


def test_a_zone_entry_ramps_to_high_and_the_budget_brings_it_back():
    w = _enter(_zoned_at_normal())
    assert w.job.diag["speed_target_rpm"] == vehicle.AUTO_HIGH_RPM
    w.drive(1.0)
    up = w.job.diag["v_base"] - vehicle.AUTO_RPM
    full = vehicle.AUTO_HIGH_RPM - vehicle.AUTO_RPM
    assert 0.3 * full < up < 0.7 * full, f"half way through the 2 s ramp: {up:.0f} of {full:.0f}"
    w.drive(1.5)
    assert w.job.diag["v_base"] == vehicle.AUTO_HIGH_RPM
    snap = w.job.mission_snapshot()
    assert snap["high_speed"] is True and "entered" in snap["speed_reason"]
    assert w.until(lambda: not w.job.tape.speed.high, 15.0), "the budget ends HIGH"
    assert "budget" in w.job.tape.speed.reason
    w.drive(2.5)
    assert w.job.diag["v_base"] == vehicle.AUTO_RPM


def _at_high():
    w = _enter(_zoned_at_normal())
    w.drive(2.6)
    assert w.job.diag["v_base"] == vehicle.AUTO_HIGH_RPM
    return w


def test_passing_machine_tags_never_changes_the_speed():
    """Operator, 2026-10-08: machines that are not the destination are passed at HIGH,
    and an RFID read between the outer and the inner marker does not cancel the entry."""
    w = World(missions.load("line-a"), "MRU4")
    w.park_at_home()
    w.start()
    w.drive(2.0)
    w.mark(OUTER)
    w.tick()
    w.drive(0.2)
    w.read("0110")
    w.tick()
    w.drive(0.2)
    w.mark(INNER)
    w.tick()
    assert w.job.tape.speed.high, w.job.tape.speed.reason
    w.drive(2.5)
    for tag in ("0120", "0130"):
        w.read(tag)
        w.tick()
        w.drive(0.3)
        assert w.job.tape.speed.high and w.job.tape.stop is None, (tag, w.job.tape.speed.reason)
    assert w.job.diag["v_base"] == vehicle.AUTO_HIGH_RPM


def test_a_corner_on_the_site_markers_leaves_and_enters():
    """line-a: leaving one straight (inner 1, outer 2), the corner, entering the next
    (outer 2, inner 1). Marker positions are exact, so the 0.65 m window holds."""
    w = World(missions.load("line-a"), "MRU4")
    w.park_at_home()
    w.start()
    w.drive(2.0)

    def travel(m):
        start = w.job.tape.speed.at_m
        while w.job.tape.speed.at_m - start < m:
            w.tick()

    w.mark(INNER)
    w.tick()
    travel(0.5)
    w.mark(OUTER)
    w.tick()
    assert w.job.tape.speed.state == "normal" and "leaving" in w.job.tape.speed.reason
    travel(1.8)
    w.mark(OUTER)
    w.tick()
    travel(0.5)
    w.mark(INNER)
    w.tick()
    assert w.job.tape.speed.high, w.job.tape.speed.reason


# -- speed is the markers', never RFID's (operator, 2026-10-09) -------------------------

def test_the_zone_works_with_the_rfid_link_down():
    """A zone-only mission does not need the RFID link, and the link being down changes
    nothing about the speed."""
    w = World(doc([], high_zone=ZONE))
    w.comms = False
    w.start()
    w.drive(1.0)
    _enter(w)
    w.drive(2.6)
    assert w.job.state == lj.RUNNING and w.job.diag["v_base"] == vehicle.AUTO_HIGH_RPM


def test_no_machine_tag_ever_resets_high():
    """Operator, 2026-10-09: RFID decides where to stop, never the speed. At HIGH, every
    machine tag of line-a is passed at HIGH - not the destination, or already served -
    and a machine the vehicle does stop at departs HIGH with the budget it had left."""
    line_a = missions.load("line-a")
    machines = [t for t, r in line_a["TAGS"].items() if r["action"] == "stop" and r["role"] != "home"]
    assert set(machines) == {"0020", "0110", "0120", "0130", "0140"}

    def at_high(destination):
        w = World(line_a, destination)
        w.park_at_home()
        w.start()
        w.drive(2.0)
        _enter(w)
        w.drive(2.6)
        assert w.job.diag["v_base"] == vehicle.AUTO_HIGH_RPM
        return w

    w = at_high("MRU4")
    for tag in ("0110", "0120", "0130"):               # destinations that are not this job's
        w.read(tag)
        w.tick()
        w.drive(0.3)
        assert w.job.tape.speed.high and w.job.tape.stop is None, (tag, w.job.tape.speed.reason)
        assert w.job.diag["v_base"] == vehicle.AUTO_HIGH_RPM, tag

    for tag in ("0020", "0140"):                        # the always-stop, and the destination
        w = at_high("MRU4")
        w.read(tag)
        w.tick()
        assert w.until(lambda w=w: w.job.state == lj.HOLD, 5.0) and w.job.hold_cause == "station", tag
        assert w.job.tape.speed.parked_left is not None, (tag, w.job.tape.speed.reason)
        w.tick(start_edge=True, start_edge_t=w.t)
        assert w.until(lambda w=w: w.job.state == lj.RUNNING, 2.0), w.job.reason
        assert w.job.tape.speed.high and "as arrived" in w.job.tape.speed.reason, tag
        if tag == "0140":
            w.read(tag)                                 # served: driven past on the way back
            w.drive(3.0)
            assert w.job.tape.stop is None


def test_the_old_zone_tag_ids_are_just_unknown_tags():
    w = _zoned_at_normal()
    for tag in ("0060", "0040"):
        w.read(tag)
        w.tick()
        w.drive(0.3)
    assert w.job.tape.speed.state == "normal" and w.job.tape.last_encounter["action"] == "no tag rule"


def test_a_zone_marker_read_off_the_tape_centre_is_ignored():
    """2026-10-09: a pendant pivot read code 2 at +121 mm. Only a clean, centred read counts."""
    w = _zoned_at_normal()
    w.mark(OUTER, lcp2=45)
    w.tick()
    assert w.job.tape.speed.state == "normal"
    w.mark(OUTER, line_good=False)
    w.tick()
    assert w.job.tape.speed.state == "normal"
    events = [t for _, t in w.job.tape.drain_events()]
    assert any("+45 mm off the tape centre" in e for e in events)
    assert any("line not good" in e for e in events)
    w.mark(OUTER, lcp2=-30)
    w.tick()
    assert w.job.tape.speed.state == "armed", "30 mm is still on the tape"


def test_markers_down_or_lost_take_high_away():
    w = _at_high()
    w.mk_ok = False
    w.tick()
    assert not w.job.tape.speed.high and "markers down" in w.job.tape.speed.reason
    w = _at_high()
    w.mark(3, lost=1)
    w.tick()
    assert not w.job.tape.speed.high and "lost" in w.job.tape.speed.reason
    w = _zoned_at_normal()
    w.mark(OUTER)
    w.tick()
    assert w.job.tape.speed.state == "armed"
    w.mk_ok = False
    w.tick()
    assert w.job.tape.speed.state == "normal", "an armed entry is forgotten too"


def test_a_marker_seen_before_the_run_does_not_arm_it():
    w = World(ZONED, "MRU2")
    w.park_at_home()
    w.mark(OUTER)                   # pushed over the outer marker while IDLE
    w.tick()
    w.start()
    w.mark(INNER)
    w.tick()
    assert w.job.tape.speed.state == "normal"


def test_inner_while_high_drops_at_half_a_metre_per_s2_without_a_jerk_ramp():
    w = _at_high()
    w.mark(INNER)
    w.tick()
    v0 = w.job.diag["v_base"]
    w.drive(0.3)
    dropped = v0 - w.job.diag["v_base"]
    assert dropped == pytest.approx(0.3 * 0.5 * vehicle.RPM_PER_MPS, rel=0.1), dropped
    events = [t for _, t in w.job.tape.drain_events()]
    assert any("late drop" in e for e in events)


def test_outer_while_high_drops_at_the_urgent_rate():
    w = _at_high()
    w.mark(OUTER)
    w.tick()
    v0 = w.job.diag["v_base"]
    w.drive(0.2)
    dropped = v0 - w.job.diag["v_base"]
    assert dropped == pytest.approx(0.2 * 0.95 * vehicle.DECEL_RPM_S, rel=0.1), dropped
    assert w.until(lambda: w.job.diag["v_base"] == vehicle.AUTO_RPM, 1.0)


def test_the_curve_guard_drops_high_on_measured_curvature():
    w = _at_high()
    w.yaw = 1.0                     # ~1.2 1/m at 0.85 m/s: a corner
    w.drive(0.1)
    assert not w.job.tape.speed.high and "curve guard" in w.job.tape.speed.reason
    assert w.job.tape.speed.exit_rate is not None, "an urgent drop"


def test_high_needs_fresh_guard_inputs_and_encoder_counts():
    w = _at_high()
    w.yaw = None
    w.tick()
    assert not w.job.tape.speed.high and "stale" in w.job.tape.speed.reason
    w = _at_high()
    w.tick(counts=None)
    assert not w.job.tape.speed.high and "wheel travel" in w.job.tape.speed.reason


def test_a_stop_tag_at_high_goes_normal_and_still_stops():
    w = _at_high()
    w.read(MRU2)
    w.tick()
    assert not w.job.tape.speed.high and w.job.tape.stop["where"] == "MRU2"
    assert w.until(lambda: w.job.state == lj.HOLD, 5.0) and w.job.hold_cause == "station"


def _parked_at_mru2_from_high():
    w = _at_high()
    w.read(MRU2)
    w.tick()
    assert w.until(lambda: w.job.state == lj.HOLD, 5.0) and w.job.hold_cause == "station"
    assert w.job.tape.speed.parked_left is not None, w.job.tape.speed.reason
    return w


def test_a_station_reached_at_high_departs_at_high():
    """2026-10-08: a station on a straight goes on at the speed it arrived at."""
    w = _parked_at_mru2_from_high()
    left = w.job.tape.speed.parked_left - w.job.tape.speed.parked_m
    w.drive(2.0)
    w.tick(start_edge=True, start_edge_t=w.t)
    assert w.until(lambda: w.job.state == lj.RUNNING, 2.0), w.job.reason
    assert w.job.tape.speed.high and "as arrived" in w.job.tape.speed.reason
    assert w.job.tape.speed.budget_left_m() == pytest.approx(left, abs=0.01)
    assert w.until(lambda: w.job.diag["v_base"] == vehicle.AUTO_HIGH_RPM, 4.0)


def test_a_protective_field_at_the_station_keeps_the_high_departure():
    """Parked, the drives are in standby (2026-10-08): torque off is expected and a field
    trip stops nothing, so the station hold stays. Operator, 2026-10-23: a protective stop
    never resets HIGH - the departure is as arrived (it cost the parked HIGH until then)."""
    w = _parked_at_mru2_from_high()
    left = w.job.tape.speed.parked_left - w.job.tape.speed.parked_m
    w.tick(torque_off=True, field_clear=False)
    assert w.job.state == lj.HOLD and w.job.hold_cause == "station"
    assert w.job.tape.speed.parked_left is not None
    w.drive(1.0, torque_off=True, field_clear=False)
    w.drive(1.0, torque_off=True)
    w.tick(start_edge=True, start_edge_t=w.t, torque_off=True)
    assert w.job.state == lj.ARMED, w.job.reason
    assert w.until(lambda: w.job.state == lj.RUNNING, 1.0), w.job.reason
    assert w.job.tape.speed.high and "as arrived" in w.job.tape.speed.reason
    assert w.job.tape.speed.budget_left_m() == pytest.approx(left, abs=0.05)


def test_a_run_from_home_starts_normal_even_after_arriving_high():
    """The Home tag parks HIGH like any stop (no RFID tag sets the speed); the next run
    is a new run, which starts NORMAL."""
    w = _at_high()
    w.read(HOME)
    w.tick()
    assert w.job.tape.speed.parked_left is not None, "Home parks, it does not drop"
    assert at_rest_stopped(w) and w.job.state == lj.DONE, w.job.reason
    w.job.set_mission(w.job.mission, "MRU1")
    w.start()
    assert not w.job.tape.speed.high and w.job.tape.speed.parked_left is None


def test_an_rfid_link_hold_at_high_resumes_high():
    """2026-10-09: no RFID event changes the speed - the link hold parks HIGH like the
    protective field, and the Start that resumes the run restores it."""
    w = _at_high()
    left = w.job.tape.speed.budget_left_m()
    w.comms = False
    w.tick()
    assert w.job.state == lj.HOLD and w.job.hold_cause == "rfid", w.job.reason
    assert w.job.tape.speed.parked_left is not None
    w.comms = True
    w.gen += 1
    w.enc = []
    w.drive(1.0)
    w.tick(start_edge=True, start_edge_t=w.t)
    assert w.until(lambda: w.job.state == lj.RUNNING, 2.0), w.job.reason
    assert w.job.tape.speed.high and w.job.tape.speed.budget_left_m() == pytest.approx(left, abs=0.05)


def test_a_warning_field_hold_does_not_use_up_the_budget():
    """The mux slows the vehicle where the follower cannot see it; the budget counts
    wheel travel, so a held vehicle keeps its HIGH budget."""
    w = _at_high()
    w.integrate = False             # wheels stopped by the field, command still HIGH
    w.tick()                        # the last integrated step lands one tick late
    used = w.job.tape.speed.used_m
    w.drive(10.0)
    assert w.job.tape.speed.high and w.job.tape.speed.used_m == pytest.approx(used)


def test_a_protective_stop_at_high_resumes_high():
    """Operator, 2026-10-09: after a protective stop the vehicle goes back to HIGH, with
    the budget it had less any wheel travel while held - like a station departure."""
    w = _at_high()
    left = w.job.tape.speed.budget_left_m()
    w.tick(torque_off=True, field_clear=False)
    assert w.job.state == lj.HOLD and w.job.hold_cause == "field"
    assert not w.job.tape.speed.high, "NORMAL while held"
    assert w.job.tape.speed.parked_left is not None
    w.drive(1.0, torque_off=True, field_clear=False)
    assert w.until(lambda: w.job.state == lj.RUNNING, 5.0), w.job.reason
    assert w.job.tape.speed.high and "as arrived" in w.job.tape.speed.reason
    assert w.job.tape.speed.budget_left_m() == pytest.approx(left, abs=0.05)
    assert w.until(lambda: w.job.diag["v_base"] == vehicle.AUTO_HIGH_RPM, 4.0)


@pytest.mark.parametrize("scale", [0.5, 0.0])
def test_warning_fields_1_and_2_never_take_high_away(scale):
    """Operator, 2026-10-23: a warning field slows (W1) or stops (W2, x0.00) the vehicle in
    the mux; the run stays RUNNING, HIGH stays granted and the budget only counts travel."""
    w = _at_high()
    w.integrate = scale > 0.0       # W2: the wheels stand still under a HIGH command
    w.tick(warning_scale=scale)
    used = w.job.tape.speed.used_m
    # W1: this world does not slow the wheels for the mux, so keep within the 5.6 m budget
    w.drive(8.0 if scale == 0.0 else 1.0, warning_scale=scale)
    assert w.job.state == lj.RUNNING and w.job.tape.speed.high, w.job.tape.speed.reason
    if scale == 0.0:
        assert w.job.tape.speed.used_m == pytest.approx(used)
    w.integrate = True
    w.drive(0.5)
    assert w.job.tape.speed.high


def _estop_hold_at_high(field_after_ticks):
    """Torque off with the field still reported clear: held on E-stop terms. The field
    report then arrives `field_after_ticks` ticks later (None: never)."""
    w = _at_high()
    left = w.job.tape.speed.budget_left_m()
    w.tick(torque_off=True, field_clear=True)
    assert w.job.state == lj.HOLD and w.job.hold_cause == "estop", w.job.reason
    assert not w.job.tape.speed.high and w.job.tape.speed.parked_left is not None
    if field_after_ticks is not None:
        w.drive(0.02 * field_after_ticks, torque_off=True)
        w.drive(1.0, torque_off=True, field_clear=False)
    w.drive(1.0, torque_off=True)
    assert w.job.hold_cause == "estop", "the hold keeps its E-stop terms: a physical Start"
    w.drive(0.5)
    w.tick(start_edge=True, start_edge_t=w.t)
    assert w.until(lambda: w.job.state == lj.RUNNING, 2.0), w.job.reason
    return w, left


def test_a_protective_stop_reported_after_the_torque_loss_still_resumes_high():
    w, left = _estop_hold_at_high(field_after_ticks=3)
    assert w.job.tape.speed.high and "as arrived" in w.job.tape.speed.reason
    assert w.job.tape.speed.budget_left_m() == pytest.approx(left, abs=0.05)


def test_a_real_estop_resumes_normal():
    w, _ = _estop_hold_at_high(field_after_ticks=None)
    assert not w.job.tape.speed.high and "E-stop" in w.job.tape.speed.reason
    w, _ = _estop_hold_at_high(field_after_ticks=40)  # the field 0.8 s later: someone walked up
    assert not w.job.tape.speed.high


def test_travel_during_a_protective_stop_comes_off_the_budget():
    w = _at_high()
    left = w.job.tape.speed.budget_left_m()
    w.tick(torque_off=True, field_clear=False)
    w.move(2.0)  # pushed while held
    assert w.until(lambda: w.job.state == lj.RUNNING, 5.0), w.job.reason
    assert w.job.tape.speed.budget_left_m() == pytest.approx(left - 2.0, abs=0.05)
    w = _at_high()
    left = w.job.tape.speed.budget_left_m()
    w.tick(torque_off=True, field_clear=False)
    w.move(left + 0.5)  # pushed past the budget's end
    assert w.until(lambda: w.job.state == lj.RUNNING, 5.0), w.job.reason
    assert not w.job.tape.speed.high


def test_other_holds_still_drop_high():
    w = _at_high()
    w.tick(torque_off=True)  # no field evidence (yet): an E-stop story
    assert w.job.state == lj.HOLD and w.job.hold_cause == "estop"
    # NORMAL now; the budget is kept only until the resume knows whether the field was
    # behind it (test_a_real_estop_resumes_normal / ..._reported_after_the_torque_loss...)
    assert w.job.tape.speed.parked_left is not None and not w.job.tape.speed.high
    w = _at_high()
    w.tick(drives_fresh=False)
    assert w.job.state == lj.HOLD and w.job.hold_cause == "drives"
    assert w.job.tape.speed.parked_left is None and not w.job.tape.speed.high
    assert w.until(lambda: w.job.state == lj.RUNNING, 5.0), w.job.reason
    w.drive(0.5)
    assert not w.job.tape.speed.high


# -- U-turn ----------------------------------------------------------------------------

def test_the_u_turn_tag_is_ignored_once_on_the_way_back():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.read(MRU1)
    assert at_rest_stopped(w)
    w.tick(start_edge=True, start_edge_t=w.t)
    w.drive(0.7)
    _run_to(w)
    _uturn(w)
    assert w.job.tape.uturn_skip == UTURN
    w.t += 3.0  # past its ignore window: only the re-pass rule holds it
    w.read(UTURN)
    w.tick()
    assert w.job.tape.uturn_req is None and w.job.tape.last_encounter["action"] == "suppressed"


def test_the_u_turn_slows_to_the_creep_within_decel_m():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    assert w.job.diag["v_base"] == vehicle.AUTO_RPM
    w.read(UTURN)
    w.tick()
    m0, creep = w.job.followed_m, 0.1 * vehicle.RPM_PER_MPS
    assert w.until(lambda: w.job.diag["v_base"] <= creep + 1.0, 3.0)
    assert w.job.followed_m - m0 == pytest.approx(0.3, abs=0.03), w.job.followed_m - m0


def test_the_u_turn_stops_the_moment_the_tape_is_gone_creep_reached_or_not():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.read(UTURN)
    w.tick()
    w.drive(0.2)                                     # still slowing, well above the creep
    assert w.job.diag["v_base"] > 2 * 0.1 * vehicle.RPM_PER_MPS
    w.tick(sensor=NO_TAPE)                           # one tick: a flicker, keeps going
    assert w.job.tape.uturn_req["phase"] == "approach" and w.job.diag["v_base"] > 0
    left, right = w.tick(sensor=NO_TAPE)             # second tick: the tape ended
    assert w.job.tape.uturn_req["phase"] == "stopping"
    assert (left, right) == (0.0, 0.0) and w.job.diag["v_base"] == 0.0


def _plain():
    w = World(missions.load("empty"))
    w.comms = False                                  # U-turn tags alone do not need the link:
    w.start()                                        # it starts, and keeps driving, without it
    _run_to(w)
    w.comms = True
    w.tick()
    return w


def test_plain_line_following_u_turns_cw_on_0180():
    w = _plain()
    w.read("0180")
    w.tick()
    assert w.job.tape.uturn_req is not None and w.job.tape.uturn_req["direction"] == "cw"
    assert w.job.mission_snapshot()["mission"] == "", "still shown as plain line following"


def test_plain_line_following_u_turns_ccw_on_0190_and_completes():
    w = _plain()
    w.read("0190")
    w.tick()
    assert w.job.tape.uturn_req["direction"] == "ccw"
    assert w.until(lambda: w.job.tape.uturn_req["phase"] == "stopping", 4.0, sensor=NO_TAPE)
    w.still, w.integrate = True, False
    assert w.until(lambda: w.job.tape.uturn is not None, 3.0, sensor=NO_TAPE), w.job.reason
    base = w.counts
    for deg, sensor in ((90, NO_TAPE), (170, CENTRED)):
        c = uturn.counts_for_angle(deg, PER_REV)
        w.counts = (base[0] - int(round(c)), base[1] + int(round(c)))   # ccw: right wheel forward
        w.tick(sensor=sensor)
    w.drive(vehicle.U_TURN_RESUME_DELAY_S + 0.2)
    assert w.job.tape.uturn_req is None and w.job.state == lj.RUNNING, w.job.reason


def test_right_after_the_u_turn_it_is_regular_line_following():
    """No creep ceiling, no leftover U-turn state: NORMAL speed, live steering, and its
    own tag driven back over is not a second U-turn."""
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    _uturn(w)
    assert w.job.tape.uturn_req is None and w.job.tape.uturn is None
    assert w.until(lambda: w.job.diag["v_base"] == vehicle.AUTO_RPM, 4.0), w.job.diag["v_base"]
    assert w.job.diag["speed_target_rpm"] == vehicle.AUTO_RPM, "the approach creep is not carried over"
    assert w.job.diag["state"] == "run" and w.job.state == lj.RUNNING
    off = dict(CENTRED, tracks=[{"index": 2, "pos_mm": 30, "width": 10}])
    left, right = w.tick(sensor=off)
    w.drive(0.2, sensor=off)
    left, right = w.tick(sensor=off)
    assert abs(left - right) > 1.0, "steering is live after the turn"
    w.read(UTURN)
    w.tick()
    assert w.job.tape.uturn_req is None, "its own tag on the way back does not start another U-turn"


def test_the_tape_that_never_ends_faults_the_u_turn():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.read(UTURN)
    assert w.until(lambda: w.job.state == lj.FAULT, 40.0), "max_approach_m bounds the creep"
    assert "did not end" in w.job.reason


def test_a_silent_sensor_on_the_approach_is_a_fault_not_a_tape_end():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.read(UTURN)
    w.tick()
    w.tick(sensor_age_s=1.0)
    assert w.job.state == lj.FAULT and "sensor silent" in w.job.reason


def test_a_hold_during_the_u_turn_faults_rather_than_resuming_mid_manoeuvre():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.read(UTURN)
    w.tick()
    w.tick(torque_off=True)
    assert w.job.state == lj.FAULT and "U-turn" in w.job.reason


# -- RFID continuity ---------------------------------------------------------------------

def test_rfid_link_loss_holds_for_a_person():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.comms = False
    w.tick()
    assert w.job.state == lj.HOLD and w.job.hold_cause == "rfid", w.job.reason
    w.drive(1.0)
    w.tick(start_edge=True, start_edge_t=w.t)
    assert w.job.state == lj.HOLD, "Start is refused while the link is down"
    w.comms = True
    w.gen += 1
    w.enc = []
    w.drive(3.0)
    assert w.job.state == lj.HOLD, "the link coming back does not resume by itself"
    w.tick(start_edge=True, start_edge_t=w.t)
    assert w.job.state == lj.ARMED, w.job.reason
    assert w.until(lambda: w.job.state == lj.RUNNING, 1.0), w.job.reason
    w.drive(0.5)
    assert w.job.state == lj.RUNNING, "a reconnect seen while held is not a second hold"


def test_an_encounter_gap_holds_the_vehicle():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    w.seq += 5  # entries never arrived
    w.enc.append((w.seq, MRU1))
    w.tick()
    assert w.job.state == lj.HOLD and w.job.hold_cause == "rfid" and "overrun" in w.job.reason


def test_rfid_loss_during_the_u_turn_is_a_fault():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.start()
    _run_to(w)
    w.read(UTURN)
    w.tick()
    w.comms = False
    w.tick()
    assert w.job.state == lj.FAULT and "U-turn" in w.job.reason


def test_start_needs_the_rfid_link_for_a_tag_mission():
    w = World(SITE, "MRU1")
    w.park_at_home()
    w.comms = False
    w.press_start()
    assert w.job.state == lj.IDLE and "RFID link down" in w.job.reason
