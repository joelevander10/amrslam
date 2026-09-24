"""AMR QR wheel drive: analog-voltage BLDC drivers under a software speed loop. Pure, clock-fed.

The QR drivers take a speed VOLTAGE (CKDA08ETH) plus FWD / REV / BRAKE inputs (CK5162E
coils); they report nothing back. The speed loop therefore lives here, closed on the
axle encoders (qr_encoders), and so do the interlocks a CiA 402 drive would have had
inside it:

    WheelLoop   one wheel: direction interlock, feedforward + PI, rest/brake, and the
                wheel-level fault detectors (wrong-way runaway, commanded-but-not-turning,
                moving at rest).
    DriveLogic  both wheels plus the arm/fault state the ROS side sees as /drives/status
                (DISARMED / ARMED / FAULT, E-stop), the latch and its acknowledgement.

Nothing here does I/O. The node reads the encoders and the DIO image, calls
DriveLogic.tick(), writes the returned voltages to the analog module and renews the
returned coil claims on the DIO island.

Sign conventions: targets and measurements are VEHICLE terms (positive = that wheel
drives the vehicle forward). `invert` (profile vehicle.invert_left/right) maps a wheel to
DRIVER terms, where +1 raises the FWD coil and -1 the REV coil. The QR wiring has the
right motor mirrored (amr_controller.set_motor: right FWD when the speed is negative),
which is invert_right = true.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

REST, DWELL, DRIVE, COAST = "rest", "dwell", "drive", "coast"
DISARMED, ARMED, FAULT = "disarmed", "armed", "fault"


@dataclass(frozen=True)
class LoopParams:
    gear_ratio: float
    rpm_per_volt: float  # motor r/min per volt above the offset (feedforward slope)
    offset_v: float  # voltage at which the motor just turns (feedforward intercept)
    v_max: float  # hard cap on the analog output
    kp: float  # V per wheel rad/s of speed error
    ki: float  # V per wheel rad of accumulated error
    i_max: float  # |integral| cap, V
    zero_rad_s: float  # |target| below this = commanded rest
    dwell_s: float  # both coils low this long (and read back low) before a direction is raised
    runaway_rad_s: float
    runaway_s: float
    stall_s: float
    brake_on_stop: bool = True
    rest_motion_s: float = 1.0  # moving above runaway_rad_s this long while at rest = fault
    # A zero command while driving first COASTS: direction coil kept, 0 V, brake off, for up
    # to coast_s or until the wheel has stopped. A command back in the same direction then
    # continues with no dwell and no brake. The path follower asks a wheel for ~0 during a
    # steering correction; dropping the coil + braking + 0.1 s dwell each time was the jerk
    # felt on the AMR QR (2026-09-24). 0 = brake at once, as before.
    coast_s: float = 0.3


@dataclass(frozen=True)
class WheelOut:
    fwd: bool
    rev: bool
    brake: bool
    volts: float
    phase: str
    fault: str | None = None


def motor_rpm(wheel_rad_s: float, gear_ratio: float) -> float:
    return wheel_rad_s * gear_ratio * 60.0 / (2.0 * math.pi)


def feedforward_v(wheel_rad_s: float, p: LoopParams) -> float:
    """Open-loop voltage for |wheel_rad_s| (the calibration's straight line)."""
    return p.offset_v + abs(motor_rpm(wheel_rad_s, p.gear_ratio)) / p.rpm_per_volt


class WheelLoop:
    def __init__(self, p: LoopParams, invert: bool, name: str = "") -> None:
        self.p = p
        self.sign = -1.0 if invert else 1.0
        self.name = name
        self.dir = 0  # applied driver direction: -1, 0, +1
        self.phase = REST
        self._dwell_since: float | None = None
        self._integ = 0.0
        self._t_last: float | None = None
        self._runaway_since: float | None = None
        self._stall_since: float | None = None
        self._rest_motion_since: float | None = None
        self._coast_since: float | None = None
        self.volts = 0.0

    def reset(self, now: float | None = None) -> None:
        """Back to rest with nothing remembered (disarm, fault, E-stop)."""
        self.dir = 0
        self.phase = REST
        self._dwell_since = now
        self._integ = 0.0
        self._t_last = None
        self._runaway_since = self._stall_since = self._rest_motion_since = None
        self._coast_since = None
        self.volts = 0.0

    def _rest(self, brake: bool) -> WheelOut:
        self.volts = 0.0
        return WheelOut(False, False, brake, 0.0, self.phase)

    def step(
        self,
        now: float,
        target: float,
        measured: float | None,
        coils_low: bool | None,
    ) -> WheelOut:
        """One tick. target/measured in vehicle wheel rad/s; measured None = unknown.

        coils_low: the DIO readback shows this wheel's FWD and REV both low (None = no
        readback). A direction is only raised after the readback proves the other one
        dropped - FWD and REV are never commanded high together.
        """
        p = self.p
        dt = 0.0 if self._t_last is None else max(0.0, min(0.1, now - self._t_last))
        self._t_last = now
        if not (math.isfinite(target)):
            target = 0.0
        t_drv = target * self.sign
        m_drv = None if measured is None else measured * self.sign
        want = 0 if abs(t_drv) < p.zero_rad_s else (1 if t_drv > 0 else -1)

        # -- coast through a brief zero command ---------------------------------
        if want == 0 and self.dir != 0 and p.coast_s > 0:
            if self._coast_since is None:
                self._coast_since = now
            along = None if m_drv is None else m_drv * self.dir
            still_moving = along is not None and along >= p.zero_rad_s
            if still_moving and now - self._coast_since < p.coast_s:
                self.phase = COAST
                self.volts = 0.0
                self._stall_since = self._runaway_since = None
                return WheelOut(self.dir > 0, self.dir < 0, False, 0.0, COAST)
            # stopped, unknown speed or coasted long enough: a real stop (below)
        if want != 0:
            self._coast_since = None

        # -- direction interlock ------------------------------------------------
        if want != self.dir:
            if self.dir != 0:
                # leave the current direction first: both coils low, voltage zero
                self.dir = 0
                self._integ = 0.0
                self._dwell_since = now
                self._stall_since = self._runaway_since = None
            if want != 0:
                since = self._dwell_since
                settled = since is None or now - since >= p.dwell_s
                if settled and coils_low is not False:
                    self.dir = want
                    self._integ = 0.0
                    self._stall_since = self._runaway_since = None

        if self.dir == 0:
            if self._dwell_since is None:
                self._dwell_since = now
            self.phase = DWELL if want != 0 else REST
            fault = self._check_rest_motion(now, m_drv) if want == 0 else None
            out = self._rest(brake=p.brake_on_stop and want == 0)
            return WheelOut(out.fwd, out.rev, out.brake, 0.0, self.phase, fault)

        # -- drive: feedforward + PI along the applied direction -----------------
        self.phase = DRIVE
        self._dwell_since = None
        self._rest_motion_since = None
        ff = feedforward_v(t_drv, p)
        along = None if m_drv is None else m_drv * self.dir  # speed in the driven direction
        err = 0.0 if along is None else abs(t_drv) - along
        integ = self._integ + p.ki * err * dt
        integ = max(-p.i_max, min(p.i_max, integ))
        v = ff + p.kp * err + integ
        if v > p.v_max and err > 0:
            integ = self._integ  # saturated high: do not wind further up
            v = ff + p.kp * err + integ
        elif v < 0.0 and err < 0:
            integ = self._integ
            v = ff + p.kp * err + integ
        self._integ = integ
        v = max(0.0, min(p.v_max, v))
        self.volts = v

        fault = self._check_drive(now, along, v, abs(t_drv))
        return WheelOut(self.dir > 0, self.dir < 0, False, v, DRIVE, fault)

    # ---- detectors ----

    def _check_drive(self, now: float, along: float | None, v: float, target: float) -> str | None:
        p = self.p
        if along is not None and along < -p.runaway_rad_s:
            if self._runaway_since is None:
                self._runaway_since = now
            if now - self._runaway_since >= p.runaway_s:
                return (
                    f"{self.name} wheel turning AGAINST the command ({along:+.2f} rad/s for "
                    f"{now - self._runaway_since:.2f} s): check vehicle.invert_{self.name} and "
                    f"qr_base.enc_invert_{self.name}"
                )
        else:
            self._runaway_since = None
        # Commanded clearly above rest, and not turning. Deliberately NOT "at the voltage
        # cap": the analog drivers report nothing, so a driver stopped from outside (alarm,
        # E-stop or scanner OSSD wired to its enable, no motor power) looks exactly like
        # this at whatever voltage the PI has reached - and must not sit there waiting to
        # lurch when the driver comes back.
        if along is not None and target >= 4.0 * p.zero_rad_s and abs(along) < p.zero_rad_s:
            if self._stall_since is None:
                self._stall_since = now
            if now - self._stall_since >= p.stall_s:
                return (
                    f"{self.name} wheel stalled: commanded {target:.2f} rad/s at {v:.2f} V and no motion "
                    f"for {now - self._stall_since:.1f} s - blocked wheel, driver alarm/disabled or no "
                    f"motor power"
                )
        else:
            self._stall_since = None
        return None

    def _check_rest_motion(self, now: float, m_drv: float | None) -> str | None:
        p = self.p
        if m_drv is not None and abs(m_drv) > p.runaway_rad_s:
            if self._rest_motion_since is None:
                self._rest_motion_since = now
            if now - self._rest_motion_since >= max(p.rest_motion_s, p.runaway_s):
                return (
                    f"{self.name} wheel moving ({m_drv * self.sign:+.2f} rad/s) while commanded to rest "
                    f"for {now - self._rest_motion_since:.1f} s - output stuck or vehicle pushed"
                )
        else:
            self._rest_motion_since = None
        return None


@dataclass(frozen=True)
class Inputs:
    now: float
    cmd: tuple[float, float] | None  # vehicle wheel rad/s, None = zero (stale/gated)
    left_rad_s: float | None  # measured; None = encoder not fresh
    right_rad_s: float | None
    left_coils_low: bool | None
    right_coils_low: bool | None
    io_ok: bool  # DIO scan fresh AND the last analog write succeeded
    estop: bool


@dataclass(frozen=True)
class Output:
    left: WheelOut
    right: WheelOut
    state: str  # DISARMED / ARMED / FAULT
    estop: bool
    operational: bool
    fault_reason: str
    event: str | None = None  # a transition worth one log line / Event


class DriveLogic:
    """Arm state, fault latch and E-stop around the two wheel loops. Pure."""

    def __init__(self, left: WheelLoop, right: WheelLoop, auto_arm: bool = True) -> None:
        self.left, self.right = left, right
        self.want_armed = auto_arm
        self.state = DISARMED
        self.fault_reason = ""
        self._was_estop: bool | None = None

    # ---- operator / service requests ----

    def arm(self) -> tuple[bool, str]:
        self.want_armed = True
        return True, "arm requested"

    def disarm(self) -> tuple[bool, str]:
        self.want_armed = False
        if self.state == ARMED:
            self.state = DISARMED
        return True, "disarmed"

    def ack(self) -> tuple[bool, str]:
        if self.state != FAULT:
            return False, f"no fault latched (state {self.state})"
        reason, self.fault_reason = self.fault_reason, ""
        self.state = DISARMED
        return True, f"fault acknowledged ({reason}); re-arming if wanted"

    def fault(self, reason: str) -> None:
        if self.state != FAULT:
            self.state = FAULT
            self.fault_reason = reason

    # ---- tick ----

    def tick(self, i: Inputs) -> Output:
        events = []
        if i.estop != self._was_estop:
            if self._was_estop is not None:
                events.append("E-STOP active" if i.estop else "E-STOP released")
            self._was_estop = i.estop

        enc_ok = i.left_rad_s is not None and i.right_rad_s is not None
        if self.state == ARMED:
            why = None
            if not i.io_ok:
                why = "I/O modules lost (DIO scan stale or analog write failed)"
            elif not enc_ok:
                missing = [n for n, v in (("left", i.left_rad_s), ("right", i.right_rad_s)) if v is None]
                why = f"encoder feedback lost ({', '.join(missing)})"
            if why:
                self.fault(why)
                events.append(f"FAULT: {why}")
            elif not self.want_armed:
                self.state = DISARMED
        elif self.state == DISARMED and self.want_armed and i.io_ok and enc_ok:
            self.state = ARMED
            self.left.reset(i.now)
            self.right.reset(i.now)
            events.append("armed (targets zero)")

        if self.state != ARMED or i.estop:
            # FAULT and E-stop hold the wheels (QR stop_motor: brake on); a plain disarm
            # frees them (QR shutdown_system: brake off), so the vehicle can be pushed.
            hold = self.state == FAULT or i.estop
            for w in (self.left, self.right):
                w.reset(i.now)
            rest = WheelOut(False, False, hold, 0.0, REST)
            return Output(
                rest, rest, self.state, i.estop, False, self.fault_reason, "; ".join(events) or None
            )

        cmd = i.cmd or (0.0, 0.0)
        lo = self.left.step(i.now, cmd[0], i.left_rad_s, i.left_coils_low)
        ro = self.right.step(i.now, cmd[1], i.right_rad_s, i.right_coils_low)
        why = lo.fault or ro.fault
        if why:
            self.fault(why)
            events.append(f"FAULT: {why}")
            for w in (self.left, self.right):
                w.reset(i.now)
            rest = WheelOut(False, False, True, 0.0, REST)
            return Output(rest, rest, self.state, i.estop, False, self.fault_reason, "; ".join(events))
        return Output(lo, ro, self.state, i.estop, True, "", "; ".join(events) or None)
