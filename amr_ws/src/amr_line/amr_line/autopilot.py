"""Line-following controller. Pure computation - no CAN, no Flask, no file I/O.

Control law:

    omega_cmd  = -(Kp*e + Ki*integral + Kd*d_filt)      Kp = K_RATIO * v
    v_cmd      = v_base - speed_reduction
    (N_L, N_R) = body_to_wheels(v_cmd, omega_cmd), then joint saturation

Kp scales with v deliberately. For a sensor mounted Ls ahead of the axle the
error dynamics are  e_dot = v*theta + Ls*omega, and substituting Kp = K_RATIO*v
into the closed-loop damping gives

    zeta = (Kd + Ls*K_RATIO) / (2*sqrt(K_RATIO*(1 + Ls*Kd)))

with no v in it. Damping is therefore the same at 800 r/min and at 2546, which
is what makes commissioning by working the speed up valid rather than a re-tune
at every step. KIM2A reached the same relationship empirically ("KP scales
roughly linearly with target speed").

*** The binding limit on K_RATIO is 6083h, not stability of the ideal plant. ***
The wheel difference cannot slew faster than the drivers ramp, which caps yaw
acceleration at 2*ACCEL*RAD_S_PER_RPM_DIFF = 2.59 rad/s^2 at the present 2000
(r/min)/s. An ideal-plant model says K_RATIO = 100 is optimal; with the real
slew limit that diverges at 0.8 m/s. Raising K_RATIO far requires raising 6083h
with it, and re-checking traction before doing so.

Every constant this file reads comes from config (the vehicle profile). Values
are read at call time, not bound at import, so a test can retarget a gain and
the next tick picks it up.
"""
import math

from agv_core import kinematics

from amr_line import branch
from amr_line import runtime as config


def predicted_zeta(k_ratio=None, kd=None, high=False):
    """Closed-loop damping for the current gains. Speed-independent by design.

    high=True reports the HIGH (straight) pair instead of the NORMAL (corner)
    pair, which is a different operating point and wants its own number.
    """
    if k_ratio is None:
        k_ratio = config.HIGH_K_RATIO if high else config.K_RATIO
    if kd is None:
        kd = config.HIGH_KD if high else config.KD
    k = k_ratio
    d = kd
    ls = config.SENSOR_LOOKAHEAD_M
    return (d + ls * k) / (2.0 * math.sqrt(k * (1.0 + ls * d)))


# How much faster than the vehicle's own forward speed the tape may appear to
# move sideways before the reading is treated as an artefact rather than motion.
#
# The geometry is de/dt = v*sin(theta) + Ls*omega, so the lateral rate the sensor
# can legitimately see is of the order of v. A factor of two is generous for real
# steering and still an order of magnitude below what a track SWAP produces: at
# 0.25 m/s this admits 10 mm per tick, where a swap arrives as the full
# sensor_max_step_mm - 40 mm, or 2 m/s, which no 0.25 m/s vehicle can do.
#
# Clamping the RATE rather than the position is what makes this robust. The
# position clamp cannot help: it turns one impossible jump into a run of
# impossible steps, and the last of them lands under the clamp threshold and
# reaches the derivative anyway.
LATERAL_RATE_FACTOR = 2.0


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class LineFollower:
    """One instance per vehicle. Not thread-safe; call update() from one thread."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._v_rpm = 0.0            # ramp state: current base speed
        self._a_rpm_s = 0.0          # ramp state: current acceleration
        self._integral = 0.0
        self._d_filt = 0.0
        self._speed_red = 0.0
        self._omega = 0.0            # last commanded yaw, held while coasting
        # None, not 0.0: priming from the first live sample avoids the
        # derivative kick KIM2A documents, where reset() zeroes last_pv and the
        # next tick differentiates the full standing error over one period.
        self._last_e_m = None
        self._last_valid_mm = None
        # Was the PREVIOUS tick a usable, continuous sample of the same track?
        # The derivative is only meaningful between two such samples - see the
        # discontinuity handling in update().
        self._track_continuous = False
        # Deceleration rate for a station stop, r/min per second, or None for
        # the ordinary profile ramp. Set once at the tag - see begin_measured_stop.
        self._stop_rate = None
        self._track_gap_m = 0.0      # distance travelled since the track was seen
        self._track_age_s = 0.0      # standstill backstop for the same
        self._saturated = False
        self.state = "idle"

    def begin_measured_stop(self, distance_m):
        """Decelerate to rest over `distance_m`, from whatever speed we are at.

        Solved once, here, rather than tracked as the vehicle slows: constant
        deceleration from v to rest covers v^2/2a, so a = v^2/2d. Recomputing it
        every tick against the falling speed would shrink the rate as fast as
        the speed and the vehicle would asymptote toward the tag instead of
        stopping at it.

        Distance rather than time because the resting point is what matters at a
        station. The same 2 s ramp stops 0.25 m past the tag at 800 r/min and
        0.38 m at 1200; this stops at d from either.

        Returns the rate, so a caller can log what it committed to.
        """
        v = self._v_rpm
        if distance_m > 0 and v > 0:
            self._stop_rate = v * v * config.MPS_PER_RPM / (2.0 * distance_m)
        else:
            self._stop_rate = None
        return self._stop_rate

    def hard_stop(self):
        """Collapse the profile to zero with no ramp - watchdog and fault path.

        Distinct from reset(): the PID history survives, only the motion stops.
        Without this a watchdog that zeroes the setpoint would be undone on the
        very next tick, because the ramp would still be carrying speed.
        """
        self._v_rpm = 0.0
        self._a_rpm_s = 0.0
        self._speed_red = 0.0
        self._omega = 0.0
        self._stop_rate = None
        self.state = "halted"

    # ---- pieces ----------------------------------------------------------

    @property
    def followed_mm(self):
        """The position of the track being followed, or None before the first.

        Public because BranchEngine.choose() has to re-select from the same
        anchor this does - the two are only guaranteed to land on the same track
        because they are handed identical inputs.
        """
        return self._last_valid_mm

    def _read_error(self, sensor, choice=branch.STRAIGHT):
        """Sensor dict -> (error_m, n_tracks, guard) with guard = why it changed.

        Returns error_m None when there is no usable track this tick.

        `choice` is the standing branch order. STRAIGHT holds the main track,
        which the sensor keeps on LCP2, so the default behaves as it always did
        on plain tape. See branch.select_track for why this needs no case
        analysis on #LCP.
        """
        tracks = (sensor or {}).get("tracks") or []
        if not tracks:
            return None, 0, ""

        # last_mm is what makes STRAIGHT follow the same TAPE rather than the
        # same LCP index - see select_track(). A branch order ignores it, which
        # is the one moment changing tape is intended.
        picked = branch.select_track(tracks, choice,
                                     config.BRANCH_POSITIVE_IS_LEFT,
                                     self._last_valid_mm)
        if picked is None:
            return None, len(tracks), ""
        pos_mm = float(picked["pos_mm"])
        guard = ""

        if abs(pos_mm) > config.SENSOR_MAX_MM:
            return None, len(tracks), "discard"      # implausible - drop it

        if self._last_valid_mm is not None:
            step = pos_mm - self._last_valid_mm
            if abs(step) > config.SENSOR_MAX_STEP_MM:
                pos_mm = self._last_valid_mm + math.copysign(
                    config.SENSOR_MAX_STEP_MM, step)
                guard = "slew"

        self._last_valid_mm = pos_mm
        e_mm = -pos_mm if config.INVERT_ERROR else pos_mm
        return e_mm / 1000.0, len(tracks), guard

    def _pid(self, e_m, v_mps, dt, k_ratio=None, kd=None):
        """One PID evaluation at the gains handed in.

        k_ratio/kd are arguments rather than reads of config, because they now
        differ between a straight and a slow zone - see update(). None keeps the
        profile's ordinary gains, so every existing caller is unaffected.
        """
        k_ratio = config.K_RATIO if k_ratio is None else k_ratio
        kd = config.KD if kd is None else kd
        kp = k_ratio * v_mps
        p = kp * e_m

        # Conditional integration: only inside the deadband, and never while the
        # wheels are saturated (the extra command would go nowhere but the
        # integrator).
        if abs(e_m) * 1000.0 < config.TI_DEADBAND_MM and not self._saturated:
            self._integral += e_m * dt
        i = _clamp(config.KI * self._integral, -config.I_CLAMP, config.I_CLAMP)

        # Derivative on measurement. Primed on the first tick after a reset so
        # the standing error is not differentiated against a fake zero.
        if self._last_e_m is None:
            d_raw = 0.0
        else:
            d_raw = (e_m - self._last_e_m) / dt
            # A rate no vehicle at this speed could produce did not come from
            # the vehicle. Clamped, not discarded: a genuine fast correction
            # still gets through at the physical limit. See LATERAL_RATE_FACTOR.
            lim = LATERAL_RATE_FACTOR * abs(v_mps) + 0.01
            d_raw = _clamp(d_raw, -lim, lim)
        tau_d = config.TAU_D_S
        alpha = dt / (tau_d + dt) if tau_d > 0 else 1.0
        self._d_filt += alpha * (d_raw - self._d_filt)
        self._last_e_m = e_m
        d = kd * self._d_filt

        return -(p + i + d), p, i, d

    def _gains(self):
        """The gains for this tick, interpolated on the CURRENT base speed.

        tracked-speed-plan-1 (2026-10-08): NORMAL (auto_rpm) is the corner speed and
        carries the corner pair k_ratio/kd; HIGH (auto_high_rpm) is the straight speed
        and carries high_k_ratio/high_kd. Between them the gains follow the speed, not
        a timer: a 2 s HIGH -> NORMAL ramp must not hold the corner gain near 0.85 m/s
        (Kp = K*v, and the binding limit on K is how fast 6083h slews the wheel
        difference - large K diverges near 0.8 m/s), and no gain/speed pair is ever
        reachable that was not tuned at one end. Below NORMAL (start, stops, the
        U-turn creep) it is the corner pair. This replaces the 2026-10-07 time blend
        across a slow-zone tag (run 0020): a tag step in speed is already shaped by
        the ramp, and the gain now rides the same ramp.
        """
        lo, hi = config.AUTO_RPM, config.AUTO_HIGH_RPM
        if hi <= lo:
            return config.K_RATIO, config.KD
        t = _clamp((self._v_rpm - lo) / (hi - lo), 0.0, 1.0)
        return (config.K_RATIO + t * (config.HIGH_K_RATIO - config.K_RATIO),
                config.KD + t * (config.HIGH_KD - config.KD))

    def _ramp(self, target_rpm, dt, accel_limit=None, rate_limit=None, no_jerk=False):
        """Jerk-limited approach to target_rpm. This is the whole S-curve.

        accel_limit overrides the profile's ramp for one caller only: a station
        stop, which has to arrive at rest after a set distance rather than at the
        profile's usual rate.

        rate_limit (2026-10-08, autopilot.high_ramp_s) only ever SLOWS the
        profile's ramp, and keeps its S-curve taper: a speed change spread over
        the time the site asked for, never a harder one.

        no_jerk (2026-10-08, with accel_limit): a late HIGH -> NORMAL exit at a zone
        tag or a curve-guard trip. The deceleration is applied at once rather than
        built at the jerk limit: building 1.0 m/s^2 at 4000 r/min/s^2 takes 0.8 s and
        ~0.2 m more than the 0.5 m an outer-tag exit has before the corner.
        """
        limit = config.RAMP_ACCEL_RPM_S if accel_limit is None else accel_limit
        if accel_limit is None and rate_limit is not None and rate_limit > 0:
            limit = min(limit, rate_limit)
        err = target_rpm - self._v_rpm
        a_want = _clamp(err / dt, -limit, limit)
        if no_jerk:
            self._a_rpm_s = a_want
            self._v_rpm += a_want * dt
            if (err > 0 and self._v_rpm > target_rpm) or (err < 0 and self._v_rpm < target_rpm):
                self._v_rpm, self._a_rpm_s = target_rpm, 0.0
            return self._v_rpm
        if accel_limit is None:
            # The tail of the S (2026-10-07): ask only for the acceleration that,
            # ramped back down at the jerk limit, lands on target: a^2/2j + a*dt/2 = |dv|,
            # the discrete form of sqrt(2 j |dv|) (the continuous one lags a tick and
            # leaves a 0.1 m/s^2 step at the end). Without it the acceleration stayed
            # at `limit` until the overshoot clamp below cut it to zero in one tick
            # (~14 m/s^3 at 0.75 m/s). A measured station stop keeps the constant
            # rate it was solved for: tapering it would carry the vehicle past the tag.
            half = 0.5 * config.RAMP_JERK_RPM_S2 * dt
            taper = math.sqrt(half * half + 2.0 * config.RAMP_JERK_RPM_S2 * abs(err)) - half
            a_want = _clamp(a_want, -taper, taper)
        step = config.RAMP_JERK_RPM_S2 * dt
        self._a_rpm_s += _clamp(a_want - self._a_rpm_s, -step, step)
        self._v_rpm += self._a_rpm_s * dt
        # The jerk limit makes the acceleration lag, which would otherwise let
        # the ramp sail past its target.
        if (err > 0 and self._v_rpm > target_rpm) or \
           (err < 0 and self._v_rpm < target_rpm):
            self._v_rpm = target_rpm
            self._a_rpm_s = 0.0
        return self._v_rpm

    def _reduce_speed(self, e_m, dt):
        """Shed speed on error magnitude and rate, as a FRACTION of base speed.

        The coefficients are fractions rather than absolute r/min so authority
        tracks speed automatically - as absolutes they silently fell from 15% to
        6% of base when cruise went 800 -> 2000. Scaling by self._v_rpm rather
        than AUTO_RPM means it also tracks the ramp, and matches the SR_CAP
        ceiling below, which is already a fraction of the same quantity.
        """
        rate_mm_s = abs(self._d_filt) * 1000.0
        raw = (abs(e_m) * 1000.0 * config.SR_POS_FRAC
               + rate_mm_s * config.SR_RATE_FRAC) * self._v_rpm
        raw = min(raw, self._v_rpm * config.SR_CAP)   # cap BEFORE the filter
        tau = config.SR_TAU_S
        alpha = dt / (tau + dt) if tau > 0 else 1.0
        self._speed_red += alpha * (raw - self._speed_red)
        return self._speed_red

    def _to_wheels(self, v_rpm, omega):
        """Wheel commands with the differential floored and the pair scaled.

        Works in vehicle terms throughout, then converts once through
        kinematics.body_to_wheels() so the INVERT_LEFT/RIGHT rule stays in
        exactly one place.
        """
        base = v_rpm
        diff = omega / config.RAD_S_PER_RPM_DIFF

        # Floor the inner wheel by limiting the DIFFERENTIAL, not by clipping one
        # wheel - clipping one alters the effective turn ratio.
        head = base - config.INNER_WHEEL_MIN_RPM
        if head <= 0:
            diff = 0.0
        elif abs(diff) > 2.0 * head:
            diff = math.copysign(2.0 * head, diff)

        left, right = base - diff / 2.0, base + diff / 2.0

        # Ceiling: scale BOTH by the same factor so the arc is preserved.
        peak = max(abs(left), abs(right))
        scale = 1.0
        if peak > config.MOTOR_MAX_RPM:
            scale = config.MOTOR_MAX_RPM / peak
            left *= scale
            right *= scale
        self._saturated = scale < 1.0

        v_mps = (left + right) / 2.0 * config.MPS_PER_RPM
        omega_out = (right - left) * config.RAD_S_PER_RPM_DIFF
        left_out, right_out = kinematics.body_to_wheels(v_mps, omega_out)
        return left_out, right_out, scale

    # ---- the tick --------------------------------------------------------

    def update(self, sensor, sensor_age_s, dt, running, choice=branch.STRAIGHT,
               high=False, cruise_rpm=None, change_rate=None, urgent_rate=None):
        """One control tick.

        sensor       : canworker._sensor_json() dict, or None if none seen yet
        sensor_age_s : seconds since the last TPDO1 frame, None if never
        dt           : measured seconds since the previous update()
        running      : False ramps down but keeps steering while it decelerates
        choice       : standing branch order (branch.STRAIGHT/LEFT/RIGHT)
        high         : HIGH is granted (amr_line.speed_zone): aim at AUTO_HIGH_RPM
        cruise_rpm   : a lower ceiling for this tick (the U-turn approach creep);
                       never raises the speed above the zone's cruise
        change_rate  : r/min/s for a move BETWEEN the two cruise speeds
                       (high_ramp_s); only slows the profile ramp. Below NORMAL
                       (starting, stopping) the profile's ramp applies
        urgent_rate  : r/min/s, a late drop from HIGH to NORMAL: constant, no jerk
                       build-up (see _ramp no_jerk)

        Returns (left_rpm, right_rpm, diag).
        """
        # Two tracked speeds (2026-10-08): NORMAL (auto_rpm, corners) by default,
        # HIGH (auto_high_rpm, straights) only while the high zone grants it. Corner
        # tracking is a GAIN question, not a speed one - steady error on a curve is
        # kappa / K_RATIO with no v in it (Kp = K_RATIO*v, holding a curve needs
        # omega = v*kappa), so run 0018 lost a 0.5 m U-turn at 441 r/min with K 11.3.
        # Hence the corner pair at NORMAL and _gains() riding the speed.
        cruise = config.AUTO_HIGH_RPM if high else config.AUTO_RPM
        if cruise_rpm is not None:
            cruise = min(cruise, max(0.0, cruise_rpm))
        k_ratio, kd = self._gains()
        dt = _clamp(dt if dt and dt > 0 else config.DT_NOMINAL_S,
                    config.DT_MIN_S, config.DT_MAX_S)

        stale = sensor_age_s is None or sensor_age_s > config.SENSOR_TIMEOUT_S
        e_m, n_tracks, guard = (None, 0, "stale") if stale else \
            self._read_error(sensor, choice)

        p = i = d = 0.0
        if stale:
            # Comms failure, not a tape gap. We no longer know where the line
            # is, so hold straight and stop - the least-bad blind action.
            self.state = "sensor_lost"
            self._omega = 0.0
            target = 0.0
        elif e_m is None:
            # Budget the gap in DISTANCE, not time: how far the AGV travels on a
            # stale correction is the thing that matters, and it rescales itself
            # with speed instead of having to be re-tuned at every step. The time
            # ceiling is the standstill case - stopped, no distance accrues and
            # the budget would never deplete.
            self._track_gap_m += kinematics.rpm_to_mps(self._v_rpm) * dt
            self._track_age_s += dt
            within = (self._track_gap_m <= config.LINE_LOSS_GRACE_M
                      and self._track_age_s <= config.LINE_LOSS_GRACE_MAX_S)
            if within and running:
                self.state = "coast"      # bridge a gap on the last correction
                target = cruise
            else:
                self.state = "line_lost"
                self._omega = 0.0
                target = 0.0
        else:
            self._track_gap_m = 0.0
            self._track_age_s = 0.0
            self.state = "run" if running else "stopping"
            target = cruise if running else 0.0

            # *** NEVER DIFFERENTIATE ACROSS A DISCONTINUITY. ***
            #
            # Two cases, and both used to reach the D term as though they were
            # motion. A slew clamp means the reading jumped further than the
            # guard allows, so what is fed forward is the CLAMP, not the track -
            # and the clamp is SENSOR_MAX_STEP_MM per tick, which at 50 Hz is
            # 2 m/s of apparent lateral velocity, eight times what this vehicle
            # can do. A gap means the last usable sample is several ticks old,
            # so dividing by one tick's dt inflates it by the length of the gap.
            #
            # Run 0020 at t=31.08 is what this costs: the followed position
            # jumped ~120 mm at the merge, the guard turned it into three 40 mm
            # steps, and the D term turned those into 6.66 rad/s of commanded
            # yaw - wheels to 0 and 1177 r/min, then the mirror image 0.4 s
            # later. The P term was 0.26 of that. It was all derivative.
            #
            # Re-priming through _last_e_m = None reuses exactly what reset()
            # relies on: _pid() takes d_raw = 0 for one tick and starts
            # differentiating again from the new sample.
            if guard == "slew" or not self._track_continuous:
                self._last_e_m = None
                # The filter state describes a track we are no longer following.
                self._d_filt = 0.0

            # Steering stays live while stopping so it tracks as it slows.
            v_now = kinematics.rpm_to_mps(max(self._v_rpm, 1.0))
            self._omega, p, i, d = self._pid(e_m, v_now, dt, k_ratio, kd)

        # For the next tick's discontinuity test. A tick with no usable reading
        # breaks the chain, whatever the reason - stale, no tape, discarded.
        self._track_continuous = e_m is not None

        # A station stop owns the ramp down; anything else uses the profile's.
        # Cleared the moment the vehicle is driving again, so the rate can never
        # outlive the stop it was computed for.
        if running:
            self._stop_rate = None
        rate = self._stop_rate if not running else None
        if running and urgent_rate and target < self._v_rpm:
            v_base = self._ramp(target, dt, accel_limit=urgent_rate, no_jerk=True)
        else:
            # The comfort rate only between the cruise speeds: from rest, the profile's.
            between = running and min(self._v_rpm, target) >= config.AUTO_RPM - 1.0
            v_base = self._ramp(target, dt, rate, change_rate if between else None)
        red = self._reduce_speed(e_m if e_m is not None else 0.0, dt)
        left, right, scale = self._to_wheels(max(v_base - red, 0.0), self._omega)

        return left, right, {
            "dt": dt,
            "state": self.state,
            "e_mm": None if e_m is None else e_m * 1000.0,
            "e_used": self._last_valid_mm,
            "p": p, "i": i, "d": d,
            "omega_cmd": self._omega,
            "speed_red": red,
            "v_base": v_base,
            "n_l": left, "n_r": right,
            "sat_scale": scale,
            "has_track": e_m is not None,
            "n_tracks": n_tracks,
            "guard": guard,
            "branch": choice,
            "high": bool(high),
            # Which gain set produced this row. The header line records both
            # sets, but they now vary WITHIN a run, so the row has to say.
            "k_used": k_ratio,
            "speed_mode": "high" if high else "normal",
            "speed_target_rpm": target,
        }
