"""Differential U-turn: encoder-gated pivot, then MLS-closed centring.

Pure computation owned by the CAN thread: no clocks, no I/O. Each tick the
caller supplies the drives' position counts (6064h), the cross-track error in
the follower's sign convention and the MLS track level, and writes the wheel
speeds returned.

Encoder travel only GATES the manoeuvre; the tape decides where it ends. The
sensor sits ahead of the axle, so a pivot sweeps it off the tape and back on
near 180 degrees. A reacquisition counts only after the tape was lost, past
u_turn_min_deg, at a track level close to the one the spin started on.

Field run 2026-10-08 (tag 0180): the spin handed over to centring the moment the
tape reached the sensor's edge, centring commanded zero on every tick the tape
was outside the usable +/-100 mm, and the vehicle stalled with the tape at
-101 mm until "tape lost while centring" - short of 180 deg. Hence:
  - the spin keeps going until the tape is near the centre, not merely seen;
  - centring never stops for a missing reading: it turns slowly toward the side
    the tape was last seen, for at most CENTER_LOST_DEG by encoder;
  - centring has a speed floor (CENTER_MIN_FRACTION) so a 150 kg pivot cannot
    stall short of the band on a vanishing P command.

Field run 2026-10-08 (later): handing over to proportional centring at 60 mm
slowed the last ~30 deg to 20-50 % of the spin speed, and the front wheels stuck
on the tape's bulge. Now the spin runs at FULL speed until the tape is inside the
centring band and stops there: near 180 deg the reading moves ~1.75 mm per degree,
so stopping from the spin speed overshoots ~5 mm and stays in the band. Centring
is only the correction for a stop outside the band, and never below half the
spin speed.
"""
import math

from agv_core import config, kinematics  # vehicle constants, read at call time

SPIN, CENTER, SETTLE, DONE, FAILED = "spin", "center", "settle", "done", "failed"

# Travel against the commanded direction beyond this means the count sign or
# wiring is not what the geometry assumes; stop rather than spin on.
REVERSE_LIMIT_DEG = 20.0
# Centring (only a correction after the spin stopped outside the band) speed limits,
# as fractions of the spin speed: a floor high enough that a 150 kg pivot does not
# stall on the tape's bulge (field run 2026-10-08), the spin speed as the ceiling.
CENTER_MAX_FRACTION = 1.0
CENTER_MIN_FRACTION = 0.5
# Tape not seen while centring: keep turning toward where it was last seen, at
# most this far by encoder (or CENTER_LOST_S standing), then fail.
CENTER_LOST_DEG = 15.0
CENTER_LOST_S = 3.0
# Spin time allowed, as a multiple of the nominal time to u_turn_max_deg. A
# frozen or wrongly scaled counter would otherwise never reach the angle limit.
TIME_MARGIN = 2.0
# After turning, the vehicle drives back over its own U-turn tag, which lies the
# measured approach distance ahead (tape_run.finish_u_turn). That one read is
# ignored within this much further travel; beyond it the tag re-arms in case the
# read was missed.
REARM_MARGIN_M = 1.0


def counts_delta(new, old):
    """6064h is INT32 and wraps; the shorter way round is the real travel."""
    d = (int(new) - int(old)) & 0xFFFFFFFF
    return d - (1 << 32) if d >= 1 << 31 else d


def spin_omega(wheel_rpm):
    """Yaw rate with both wheels at +/-wheel_rpm."""
    return 2.0 * wheel_rpm * config.RAD_S_PER_RPM_DIFF


def counts_for_angle(deg, counts_per_wheel_rev):
    """Per-wheel counts for a pivot of `deg`: arc (track/2)*angle over pi*D."""
    arc = config.TRACK_M / 2.0 * math.radians(deg)
    return arc / (math.pi * config.WHEEL_DIA_M) * counts_per_wheel_rev


class UTurn:
    def __init__(self, direction, counts_per_wheel_rev, start_counts, start_level,
                 phase=SPIN):
        """phase=CENTER skips the spin: centre on the tape already under the
        sensor, then settle. manualturn.py uses it after an encoder pivot."""
        if direction not in ("cw", "ccw"):
            raise ValueError(f"U-turn direction {direction!r}")
        if phase not in (SPIN, CENTER):
            raise ValueError(f"U-turn start phase {phase!r}")
        self.direction = direction
        self.sign = -1.0 if direction == "cw" else 1.0     # omega > 0 is CCW
        self._m_per_count = math.pi * config.WHEEL_DIA_M / counts_per_wheel_rev
        self._last = tuple(start_counts)
        self._left_m = self._right_m = 0.0
        self.start_level = start_level
        self.phase = phase
        self.lost = False
        self.reason = None
        self.elapsed_s = 0.0
        self._settled_s = 0.0
        self._unseen_s = 0.0
        self._unseen_from = None    # encoder angle where centring lost the tape
        self._last_e = None         # last centring error seen, for the side to turn to
        self._budget_s = (TIME_MARGIN * math.radians(config.U_TURN_MAX_DEG)
                          / spin_omega(config.AUTO_U_TURN_RPM))

    @property
    def angle_deg(self):
        """Pivot so far in the commanded direction, from wheel travel."""
        yaw = (self._right_m - self._left_m) / config.TRACK_M
        return math.degrees(yaw) * self.sign

    @property
    def active(self):
        return self.phase not in (DONE, FAILED)

    def _integrate(self, counts):
        dl = counts_delta(counts[0], self._last[0])
        dr = counts_delta(counts[1], self._last[1])
        self._last = tuple(counts)
        # Driver terms -> vehicle terms: the rule kinematics applies to r/min.
        if config.INVERT_LEFT:
            dl = -dl
        if config.INVERT_RIGHT:
            dr = -dr
        self._left_m += dl * self._m_per_count
        self._right_m += dr * self._m_per_count

    def _fail(self, reason):
        self.phase, self.reason = FAILED, reason
        return 0.0, 0.0

    def abort(self, reason):
        """Stopped from outside (Reset, Stop, a fault). A finished turn stays so."""
        if self.active:
            self._fail(reason)
        return 0.0, 0.0

    def update(self, counts, e_mm, level, dt, paused=False):
        """One tick. Returns (left_rpm, right_rpm) in driver terms.

        paused: a warning field is slowing or holding the pivot in the mux; the
        spin's time budget stands still meanwhile (2026-10-08) instead of faulting
        a turn the fields stopped. The angle and reverse limits still apply.
        """
        if not self.active:
            return 0.0, 0.0
        if not paused:
            self.elapsed_s += dt
        self._integrate(counts)
        angle = self.angle_deg
        seen = e_mm is not None

        if self.phase == SPIN:
            if angle < -REVERSE_LIMIT_DEG:
                return self._fail(f"U-turn encoder angle {angle:.0f} deg opposes "
                                  f"the commanded {self.direction} spin")
            if not seen:
                self.lost = True
            elif (self.lost and angle >= config.U_TURN_MIN_DEG
                  and abs(e_mm) <= config.U_TURN_CENTER_TOL_MM
                  and level is not None
                  and level >= self.start_level - config.U_TURN_LEVEL_TOLERANCE):
                # Inside the band at full spin speed: stop here and settle.
                self.phase, self._settled_s, self._last_e = SETTLE, 0.0, e_mm
                return 0.0, 0.0
            elif seen:
                self._last_e = e_mm     # the side the tape is on, should the spin overshoot
            if self.phase == SPIN:
                if angle >= config.U_TURN_MAX_DEG:
                    return self._fail(f"U-turn tape not reacquired within "
                                      f"{config.U_TURN_MAX_DEG:.0f} deg")
                if self.elapsed_s > self._budget_s:
                    return self._fail(f"U-turn spin exceeded {self._budget_s:.0f} s "
                                      f"at {angle:.0f} deg by encoder")
                return kinematics.body_to_wheels(
                    0.0, self.sign * spin_omega(config.AUTO_U_TURN_RPM))

        floor = CENTER_MIN_FRACTION * config.AUTO_U_TURN_RPM
        if not seen:
            # Never stop on a missing reading (that stalled the field run at the
            # sensor edge): turn slowly toward the side the tape was last seen.
            if self._unseen_from is None:
                self._unseen_from = angle
            if not paused:
                self._unseen_s += dt
            if abs(angle - self._unseen_from) > CENTER_LOST_DEG or self._unseen_s > CENTER_LOST_S:
                return self._fail(f"U-turn tape lost while centring (at {angle:.0f} deg by encoder)")
            self.phase = CENTER
            side = 1.0 if (self._last_e or 0.0) >= 0.0 else -1.0
            return kinematics.body_to_wheels(0.0, -spin_omega(side * floor))
        self._unseen_s, self._unseen_from, self._last_e = 0.0, None, e_mm

        if abs(e_mm) <= config.U_TURN_CENTER_TOL_MM:
            if self.phase == CENTER:
                self.phase, self._settled_s = SETTLE, 0.0
            else:
                self._settled_s += dt
                if self._settled_s >= config.U_TURN_RESUME_DELAY_S:
                    self.phase = DONE
            return 0.0, 0.0

        # Outside the band: centre, between the floor and the ceiling.
        self.phase = CENTER
        ceiling = CENTER_MAX_FRACTION * config.AUTO_U_TURN_RPM
        mag = max(floor, min(ceiling, config.U_TURN_CENTER_KP_RPM_PER_MM * abs(e_mm)))
        rpm = mag if e_mm > 0 else -mag
        # The follower's omega = -K*e sign: in a pivot de/dt = Ls*omega.
        return kinematics.body_to_wheels(0.0, -spin_omega(rpm))

    def snapshot(self):
        return {"active": self.active, "phase": self.phase,
                "direction": self.direction, "angle_deg": self.angle_deg,
                "tape_lost": self.lost, "reason": self.reason}
