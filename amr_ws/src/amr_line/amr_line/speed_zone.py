"""Tracked speed decision: NORMAL by default, HIGH only inside a marker high zone.

tracked-speed-plan-1 section 3/4 (2026-10-08); the cue moved from RFID tags to MLS
marker codes on 2026-10-09 (operator: RFID decides where to stop, the markers decide
speed). Pure: no ROS, no clock, no I/O. Distances are WHEEL travel handed in by the
job (encoder counts), never commanded speed: a warning-field hold must not burn a
budget that was not driven.

SpeedZone - the high-zone state machine. Every straight end carries a marker pair,
the OUTER code (corner side) and the INNER code (straight side), with one shared code
for every outer marker and one for every inner marker: the read ORDER gives direction.
cue() takes the code of one accepted marker (amr_line.tape_run filters them).

    NORMAL --outer--> ARMED --inner within arm_m--> HIGH (budget high_for_m)
    ARMED  --arm_m travelled-->                     NORMAL   (other codes: no effect)
    HIGH   --budget used up-->                      NORMAL, comfort ramp      (planned)
    HIGH   --inner-->                               NORMAL at LATE_INNER_MPS2 (late)
    HIGH   --outer-->                               NORMAL at the urgent rate (inner missed)
    HIGH   --drop(reason) from the job-->           NORMAL (U-turn, hold, guard, markers down...)
    HIGH   --park(reason) at a station stop-->      NORMAL, budget left kept
                  or a protective-field hold
    parked --resume() on the departure-->           HIGH with what is left, less the
                                                    wheel travel since the stop began

*** Every failure ends at NORMAL. *** A missed outer leaves the inner alone (no
HIGH); a missed inner lets the arm expire; a missed exit inner is covered by the
budget, which ends brake_m + margin before it. Nothing but a fresh, ordered
outer->inner raises HIGH again - or the departure from a planned station stop
(2026-10-08: a station on a straight goes on at the speed it arrived at), and
only while nothing else touched the zone in between: any drop or zone marker
forgets the kept budget. Home is not parked: a run from Home starts NORMAL.

Two pass rules, kept from the RFID read zones (harmless for a marker, which is
reported once per pass):
  - same pass: a zone code read again within arm_m of wheel travel since it was
    last seen is the same marker - ignored.
  - exit pair: an outer read within arm_m after an inner is a straight being LEFT
    (inner then outer) - it never arms.

CurveGuard - the backstop for when every exit failed: curvature kappa = |omega| / v
from the gyro (odometry as fallback), or a lateral error beyond the limit,
sustained over a short distance while HIGH or still above NORMAL. It cannot make a
corner safe at HIGH - the sensor sees the curve only 0.1 m ahead - it limits the
damage.
"""

NORMAL, ARMED, HIGH = "normal", "armed", "high"

# Late exit at the inner marker: 0.85 -> 0.50 m/s in 0.47 m, reaching NORMAL ~0.44 m before
# the tangent point at 0.5 m spacing (plan 1 section 3, corner-entry table).
LATE_INNER_MPS2 = 0.5
# Below the drive's 6084h deceleration and the mux line_d_max (both ~1.0 m/s^2), so
# neither reshapes it.
URGENT_FRACTION_OF_DRIVE = 0.95
# An inner read this soon after an outer that did not grant HIGH is reported as a
# near miss, with the measured distance.
SEEN_REPORT_M = 3.0


class SpeedZone:
    def __init__(self, cfg=None, *, normal_rpm=0.0, rpm_per_mps=0.0, drive_decel_rpm_s=0.0):
        """cfg = agv_core.mission HIGH_ZONE (or None: NORMAL always)."""
        self.cfg = cfg
        self.state = NORMAL
        self.reason = "run start" if cfg else "no high zone in this mission"
        # r/min/s for an urgent drop to NORMAL, None = the comfort ramp. Cleared by the
        # job once the speed is down (settled()).
        self.exit_rate = None
        self.late_inner_rpm_s = LATE_INNER_MPS2 * rpm_per_mps
        self.urgent_rpm_s = URGENT_FRACTION_OF_DRIVE * drive_decel_rpm_s
        self.normal_rpm = normal_rpm
        self.armed_m = 0.0
        self.used_m = 0.0
        # HIGH budget left when a station stop began (None: the departure is NORMAL),
        # and the wheel travel since, which the departure takes off it.
        self.parked_left = None
        self.parked_m = 0.0
        # Wheel travel so far and where each zone code was last seen on it.
        self.at_m = 0.0
        self.seen_at: dict[str, float] = {}

    @property
    def enabled(self):
        return self.cfg is not None

    @property
    def high(self):
        return self.state == HIGH

    def budget_left_m(self):
        return max(0.0, self.cfg["high_for_m"] - self.used_m) if self.high else 0.0

    def _normal(self, reason, rate=None):
        was = self.state
        self.state = NORMAL
        self.reason = reason
        if was == HIGH:
            # An urgent rate never relaxes into a gentler one mid-drop.
            if rate is not None:
                self.exit_rate = rate if self.exit_rate is None else max(self.exit_rate, rate)
            return reason
        return None

    def drop(self, reason, urgent=False):
        """The job takes HIGH away. Returns the reason when it was HIGH, else None.
        A budget kept at a station stop is forgotten too."""
        self.parked_left = None
        return self._normal(reason, self.urgent_rpm_s if urgent else None)

    def park(self, reason):
        """A planned station stop: NORMAL now, the budget left kept for the departure.
        The measured stop sets its own deceleration. Returns the reason when it was HIGH."""
        left = self.budget_left_m() if self.high else None
        why = self._normal(reason)
        self.parked_left, self.parked_m = left, 0.0
        if left is not None:
            self.reason = f"{reason}: departs HIGH ({left:.1f} m of budget left)"
        return why

    def resume(self):
        """Departure from the station stop. Returns (level, text) when HIGH again."""
        left, self.parked_left = self.parked_left, None
        if left is None or not self.enabled:
            return None
        left -= self.parked_m
        if left <= 0.0:
            self.reason = "HIGH budget used up during the stop: NORMAL"
            return None
        self.state, self.used_m, self.exit_rate = HIGH, self.cfg["high_for_m"] - left, None
        self.reason = f"departed HIGH, as arrived: {left:.1f} m of budget left"
        return "info", self.reason

    def cue(self, code):
        """One accepted zone marker read while driving. Returns (level, text) or None."""
        if not self.enabled:
            return None
        inner, outer = self.cfg["inner"], self.cfg["outer"]
        if code in (inner, outer):
            self.parked_left = None     # the stop no longer sits inside the zone it entered
            last = self.seen_at.get(code)
            self.seen_at[code] = self.at_m
            if last is not None and self.at_m - last < self.cfg["arm_m"]:
                return None             # same pass: the same marker reported again
        if code == outer and self.state != HIGH:
            last_inner = self.seen_at.get(inner)
            if last_inner is not None and self.at_m - last_inner < self.cfg["arm_m"]:
                self.state = NORMAL
                self.reason = (f"outer marker {code} {self.at_m - last_inner:.2f} m after inner {inner}: "
                               f"leaving a high zone")
                return "info", self.reason
        if code == outer:
            if self.state == HIGH:
                return "warn", self._normal(
                    f"outer marker {code} read while HIGH: inner marker missed - urgent drop",
                    self.urgent_rpm_s)
            self.state, self.armed_m = ARMED, 0.0
            self.reason = f"outer marker {code}: waiting for inner within {self.cfg['arm_m']:.2f} m"
            return "info", self.reason
        if code == inner:
            if self.state == ARMED:
                self.state, self.used_m, self.exit_rate = HIGH, 0.0, None
                self.reason = f"high zone entered: HIGH for {self.cfg['high_for_m']:.1f} m"
                return "info", self.reason
            if self.state == HIGH:
                return "warn", self._normal(
                    f"inner marker {code} read while HIGH: budget did not end before the corner "
                    f"(survey or encoder scale) - late drop", self.late_inner_rpm_s)
            # Say how far the last outer was: an entry missed by a few cm reads as one.
            outer_at = self.seen_at.get(outer)
            if outer_at is not None and self.at_m - outer_at < SEEN_REPORT_M:
                self.reason = (f"inner marker {code} {self.at_m - outer_at:.2f} m after outer {outer} - "
                               f"outside the {self.cfg['arm_m']:.2f} m entry window: stays NORMAL")
                return "warn", self.reason
            self.reason = f"inner marker {code} without outer: stays NORMAL"
            return "info", self.reason
        return None

    def travel(self, metres):
        """Wheel travel since the last call. Returns (level, text) or None."""
        if not self.enabled or metres <= 0.0:
            return None
        self.at_m += metres
        if self.parked_left is not None:
            self.parked_m += metres
        if self.state == ARMED:
            self.armed_m += metres
            if self.armed_m > self.cfg["arm_m"]:
                self.state = NORMAL
                self.reason = (f"inner marker not read within {self.cfg['arm_m']:.2f} m of the outer: "
                               f"entry window closed, stays NORMAL")
                return "warn", self.reason
            return None
        if self.state == HIGH:
            self.used_m += metres
            if self.used_m >= self.cfg["high_for_m"]:
                return "info", self._normal(f"HIGH budget {self.cfg['high_for_m']:.1f} m used: NORMAL "
                                            f"before the corner")
        return None

    def settled(self, v_base_rpm):
        """The speed is down at NORMAL: an urgent rate has done its job."""
        if self.exit_rate is not None and v_base_rpm <= self.normal_rpm + 1.0:
            self.exit_rate = None


class CurveGuard:
    def __init__(self, kappa_trip, e_trip_mm, window_m=0.05, v_min_mps=0.2):
        self.kappa_trip = float(kappa_trip)
        self.e_trip_mm = float(e_trip_mm)
        self.window_m = float(window_m)
        self.v_min = float(v_min_mps)
        self.over_m = 0.0
        self.kappa = None           # last estimate, for the diagnostics

    def update(self, active, yaw_rate, v_mps, e_mm, metres):
        """Returns the trip reason, or None. `active`: HIGH or still above NORMAL."""
        self.kappa = (abs(yaw_rate) / v_mps
                      if yaw_rate is not None and v_mps is not None and v_mps >= self.v_min else None)
        if not active or v_mps is None or v_mps < self.v_min:
            self.over_m = 0.0
            return None
        why = None
        if self.kappa is not None and self.kappa > self.kappa_trip:
            why = f"curvature {self.kappa:.2f} 1/m > {self.kappa_trip:g} (R < {1 / self.kappa_trip:.1f} m)"
        elif e_mm is not None and abs(e_mm) > self.e_trip_mm:
            why = f"lateral error {abs(e_mm):.0f} mm > {self.e_trip_mm:g} mm"
        if why is None:
            self.over_m = 0.0
            return None
        self.over_m += max(0.0, metres)
        if self.over_m >= self.window_m:
            self.over_m = 0.0
            return f"curve guard: {why}"
        return None
