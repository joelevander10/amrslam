"""Profile position (CiA 402 pp) blind moves, executed inside the drives. Pure, clock-fed.

The drive's own position loop runs each wheel to an absolute count target on a
trapezoid (profile velocity 6081h, acceleration 6083h, deceleration 6084h). This
module holds everything about that which can be decided without a bus:

  move_spec()      a blind-run segment (core/blindrun.plan) -> one PP move per wheel.
                   For an arc the inner wheel's velocity AND ramps are scaled by its
                   distance ratio, so the two trapezoids take the same time and the
                   wheel-speed ratio - hence the curvature - holds through the ramps,
                   not only at cruise. Equal ramps on both wheels would land both
                   counts exactly and still leave the vehicle off the arc.
  validate_move()  the drive owner's own bounds check on a requested move.
  verify_config()  the pp safety configuration read back from both drives against
                   the profile. This software never writes it (guard.py).
  Controller       the per-tick state machine: IDLE -> MOVING -> (HALTING) -> exit.
                   Blocking bus work (enter: verify + mode switch + set-points;
                   exit: back to pv) is RETURNED as an action for the bus thread.

LOCKED by default: the profile's pp.enabled is false until someone records, in
pp.vendor_ref, the decision to run pp on the 400 W motor with a 1:30 gearhead
(the manual requires motion extension there, which no positioning type offers;
there is no vendor confirmation, so this is an internal risk acceptance).

Held, not latched. A move continues only while the requester keeps sending a
fresh hold for the same run id; staleness, hold=false, a gate failure, a
following error or the time limit HALTs both wheels (controlword bit 8, the
profile ramp per 605Dh). A halt that is not confirmed at rest in stop_confirm_s
becomes a drive-owner FAULT, whose existing path withholds the PC heartbeat.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# ---- CiA 402 words (BLV-R CANopen manual 7.3.3 / 7.3.4) -------------------------
MODE_PP, MODE_PV = 1, 3
CW_ENABLED = 0x000F  # Operation enabled; bit 6 = 0: the target is ABSOLUTE
CW_NEW_SETPOINT = 1 << 4
CW_IMMEDIATE = 1 << 5  # single set-point: applied at once, not queued
CW_HALT = 1 << 8
CW_START = CW_ENABLED | CW_NEW_SETPOINT | CW_IMMEDIATE
CW_HALTED = CW_ENABLED | CW_HALT
SW_TARGET_REACHED = 1 << 10
SW_SETPOINT_ACK = 1 << 12
SW_FOLLOWING_ERROR = 1 << 13
INT32_MIN, INT32_MAX = -(1 << 31), (1 << 31) - 1

# Controller states
IDLE, MOVING, HALTING = "idle", "moving", "halting"
# Results of the last run
DONE, HALTED, REFUSED, FAULTED = "done", "halted", "refused", "faulted"


@dataclass(frozen=True)
class WheelMove:
    delta_counts: int  # driver terms (the drive's own sign), relative to the position at entry
    velocity_rpm: int  # 6081h, motor r/min, > 0 when delta != 0
    accel_rpm_s: int  # 6083h
    decel_rpm_s: int  # 6084h

    @property
    def moves(self) -> bool:
        return self.delta_counts != 0


@dataclass(frozen=True)
class MoveSpec:
    left: WheelMove
    right: WheelMove
    duration_s: float  # planned trapezoid time of the dominant wheel
    ratio_error: float = 0.0  # |velocity ratio - distance ratio|, from integer rounding

    def wheels(self) -> tuple[WheelMove, WheelMove]:
        return self.left, self.right


def move_spec(segment: dict, accel_rpm_s: float, decel_rpm_s: float) -> MoveSpec:
    """One blindrun.plan() segment -> per-wheel PP moves.

    `segment["counts"]` are driver-terms targets, `segment["rpm"]` the planned
    per-wheel motor r/min (signed, vehicle terms; only the magnitude is used - the
    direction of a PP move is the sign of its count delta).
    """
    counts = [int(c) for c in segment["counts"]]
    rpm = [abs(float(r)) for r in segment["rpm"]]
    dom = max(abs(c) for c in counts)
    if dom == 0:
        raise ValueError("the move is zero counts on both wheels")
    wheels, ratios = [], []
    for c, r in zip(counts, rpm, strict=True):
        if c == 0:
            wheels.append(WheelMove(0, 0, 0, 0))
            continue
        k = abs(c) / dom
        wheels.append(
            WheelMove(
                c,
                max(1, round(r)),
                max(1, round(accel_rpm_s * k)),
                max(1, round(decel_rpm_s * k)),
            )
        )
        ratios.append(k)
    err = 0.0
    moving = [w for w in wheels if w.moves]
    if len(moving) == 2:
        v = [w.velocity_rpm for w in moving]
        want = min(ratios) / max(ratios)
        err = abs(min(v) / max(v) - want)
    return MoveSpec(wheels[0], wheels[1], float(segment.get("duration_s", 0.0)), err)


def validate_move(spec: MoveSpec, *, max_rpm: float, max_counts: int, max_accel_rpm_s: float) -> str | None:
    """The drive owner's bounds on a requested move (it trusts no requester). None = ok."""
    if not (spec.left.moves or spec.right.moves):
        return "the move is zero counts on both wheels"
    if not (math.isfinite(spec.duration_s) and spec.duration_s > 0):
        return "planned duration missing or not positive"
    for name, w in (("left", spec.left), ("right", spec.right)):
        if not w.moves:
            continue
        if abs(w.delta_counts) > max_counts:
            return f"{name}: {abs(w.delta_counts)} counts exceeds the {max_counts} limit"
        if not 0 < w.velocity_rpm <= max_rpm:
            return f"{name}: profile velocity {w.velocity_rpm} r/min outside (0, {max_rpm:.0f}]"
        for what, a in (("acceleration", w.accel_rpm_s), ("deceleration", w.decel_rpm_s)):
            if not 0 < a <= max_accel_rpm_s:
                return f"{name}: {what} {a} r/min/s outside (0, {max_accel_rpm_s:.0f}]"
    return None


def absolute_targets(actual: tuple[int, int], spec: MoveSpec) -> tuple[int, int]:
    """Absolute 607Ah targets = actual 6064h + delta. Refuses rather than wrap: an
    INT32 target on the far side of the wrap would drive the long way round."""
    out = []
    for a, w in zip(actual, spec.wheels(), strict=True):
        t = int(a) + w.delta_counts
        if not INT32_MIN <= t <= INT32_MAX:
            raise ValueError(f"target {t} is outside INT32; re-arm to rebaseline the position")
        out.append(t)
    return out[0], out[1]


def verify_config(
    read, nodes: dict[int, str], expect: dict[str, int | None], objects: dict[str, int]
) -> list[str]:
    """Read every pp safety object from every drive; -> the problems (empty = ok).

    `read(node, index)` returns the value or None. `expect` holds the profile's
    pp.expect values (None = not configured). Both drives must hold the expected
    value - and so each other's.
    """
    problems = []
    unset = sorted(k for k, v in expect.items() if v is None)
    if unset:
        return [f"pp.expect not configured: {', '.join(unset)}"]
    for key, index in objects.items():
        for nid, label in nodes.items():
            got = read(nid, index)
            if got is None:
                problems.append(f"{label} {index:04X}h ({key}) unreadable")
            elif int(got) != int(expect[key]):
                problems.append(f"{label} {index:04X}h ({key}) = {int(got)}, profile expects {expect[key]}")
    return problems


# ---- the per-tick controller ------------------------------------------------------


@dataclass
class Request:
    """The latest hold from the requester (the commissioning node)."""

    run_id: str
    generation: int
    hold: bool
    spec: MoveSpec
    t_recv: float


@dataclass
class Feedback:
    """One wheel's latest TPDO1 (statusword + 606Ch) with its receive time."""

    statusword: int | None
    rpm: int | None
    t: float | None


@dataclass
class Tick:
    now: float
    request: Request | None
    gate: str | None  # the node's authority/health verdict: None = motion permitted
    left: Feedback
    right: Feedback


@dataclass
class Out:
    action: str = "none"  # none | enter | cw | exit | fault
    controlwords: tuple[int, int] | None = None
    spec: MoveSpec | None = None
    reason: str = ""


@dataclass
class Result:
    run_id: str
    outcome: str  # done | halted | refused | faulted
    reason: str
    targets: tuple[int, int] | None = None
    started_t: float | None = None
    ended_t: float | None = None


@dataclass
class Controller:
    hold_timeout_s: float = 0.2
    feedback_timeout_s: float = 0.1
    settle_s: float = 0.1  # both wheels reached and at rest this long -> done
    stop_confirm_s: float = 3.0
    time_margin: float = 2.0
    time_slack_s: float = 2.0

    state: str = IDLE
    run_id: str = ""
    spec: MoveSpec | None = None
    targets: tuple[int, int] | None = None
    reason: str = ""
    last: Result | None = None
    t_start: float | None = None
    t_halt: float | None = None
    t_settled: float | None = None
    acked: list = field(default_factory=lambda: [False, False])
    t_acked: list = field(default_factory=lambda: [None, None])
    _consumed: str = ""  # run ids already run (or refused): a hold never re-runs one
    _pending_outcome: str = DONE  # what "exit" completes as: done, or halted

    @property
    def active(self) -> bool:
        """While not IDLE the drive owner sends OUR controlwords and no velocity."""
        return self.state != IDLE

    # -- results of the blocking work the bus thread performed --

    def entered(self, ok: bool, reason: str, targets: tuple[int, int] | None, now: float) -> None:
        if not ok:
            self.state, self.reason = IDLE, reason
            self.last = Result(self.run_id, REFUSED, reason, None, None, now)
            return
        self.state, self.targets, self.reason = MOVING, targets, ""
        self.t_start, self.t_settled = now, None
        self.acked, self.t_acked = [not w.moves for w in self.spec.wheels()], [None, None]

    def exited(self, ok: bool, reason: str, now: float) -> None:
        """Back in pv (ok) - or not, in which case the owner faults."""
        outcome = self._pending_outcome
        why = self.reason if ok else f"{self.reason}; return to pv failed: {reason}".strip("; ")
        if not ok:
            outcome = FAULTED
        self.last = Result(self.run_id, outcome, why, self.targets, self.t_start, now)
        self.state = IDLE

    # -- the tick --

    def _halt_reason(self, i: Tick) -> str | None:
        r = i.request
        if r is None or i.now - r.t_recv > self.hold_timeout_s:
            return "hold went stale (requester silent)"
        if r.run_id != self.run_id:
            return f"a different run ({r.run_id}) was requested"
        if not r.hold:
            return "hold released by the requester"
        if i.gate:
            return i.gate
        for label, fb in (("left", i.left), ("right", i.right)):
            if fb.t is None or i.now - fb.t > self.feedback_timeout_s or fb.statusword is None:
                return f"{label} feedback stale"
            if fb.statusword & SW_FOLLOWING_ERROR:
                return f"{label} following error (6041h bit 13)"
        limit = self.time_margin * (self.spec.duration_s if self.spec else 0.0) + self.time_slack_s
        if self.t_start is not None and i.now - self.t_start > limit:
            return f"time limit {limit:.1f} s exceeded"
        return None

    def _begin_halt(self, now: float, why: str) -> Out:
        self.state, self.reason, self.t_halt = HALTING, why, now
        return Out("cw", (CW_HALTED, CW_HALTED), reason=why)

    def tick(self, i: Tick) -> Out:
        if self.state == IDLE:
            r = i.request
            if r is None or not r.hold or i.now - r.t_recv > self.hold_timeout_s:
                return Out()
            if r.run_id == self._consumed or not r.run_id:
                return Out()  # one Start = one move; the same run id never runs twice
            self._consumed = r.run_id
            if i.gate:
                self.run_id, self.reason = r.run_id, i.gate
                self.last = Result(r.run_id, REFUSED, i.gate, None, None, i.now)
                return Out()
            self.run_id, self.spec, self.reason = r.run_id, r.spec, ""
            self._pending_outcome = DONE
            return Out("enter", spec=r.spec)

        if self.state == MOVING:
            why = self._halt_reason(i)
            if why:
                self._pending_outcome = HALTED
                return self._begin_halt(i.now, why)
            fbs = (i.left, i.right)
            cws = []
            for k, (fb, w) in enumerate(zip(fbs, self.spec.wheels(), strict=True)):
                if not self.acked[k] and fb.statusword & SW_SETPOINT_ACK:
                    self.acked[k], self.t_acked[k] = True, fb.t
                # NSP is held until the drive acknowledges the set-point, then released.
                cws.append(CW_START if (w.moves and not self.acked[k]) else CW_ENABLED)
            reached = all(
                (not w.moves)
                or (
                    self.acked[k]
                    and fbs[k].t is not None
                    and self.t_acked[k] is not None
                    and fbs[k].t > self.t_acked[k]
                    and fbs[k].statusword & SW_TARGET_REACHED
                    and fbs[k].rpm == 0
                )
                for k, w in enumerate(self.spec.wheels())
            )
            if reached:
                if self.t_settled is None:
                    self.t_settled = i.now
                elif i.now - self.t_settled >= self.settle_s:
                    self._pending_outcome = DONE
                    return Out("exit", reason="target reached")
            else:
                self.t_settled = None
            return Out("cw", (cws[0], cws[1]))

        # HALTING: keep Halt asserted until both wheels report zero on fresh feedback
        at_rest = all(
            fb.t is not None and i.now - fb.t <= self.feedback_timeout_s and fb.rpm == 0
            for fb in (i.left, i.right)
        )
        if at_rest:
            return Out("exit", reason=self.reason)
        if i.now - self.t_halt > self.stop_confirm_s:
            self.last = Result(
                self.run_id,
                FAULTED,
                f"{self.reason}; halt not confirmed at rest",
                self.targets,
                self.t_start,
                i.now,
            )
            self.state = IDLE
            return Out(
                "fault", reason=f"pp halt not confirmed in {self.stop_confirm_s:.1f} s ({self.reason})"
            )
        return Out("cw", (CW_HALTED, CW_HALTED), reason=self.reason)

    def abandon(self, why: str, now: float) -> None:
        """The owner lost the drives (fault/disarm) under a move: record it, go IDLE."""
        if self.state != IDLE:
            self.last = Result(self.run_id, FAULTED, why, self.targets, self.t_start, now)
            self.state, self.reason = IDLE, why


LEASE_COMMISSIONING = 4  # ControlLease.COMMISSIONING


def authority_gate(
    now: float,
    *,
    enabled: bool,
    lease: tuple[float, int, int] | None,
    move_generation: int,
    panel: tuple[float, bool, bool] | None,
    lease_timeout_s: float = 0.3,
    panel_timeout_s: float = 0.2,
) -> str | None:
    """The drive owner's OWN authority check for a pp move; None = permitted.

    Stricter than the velocity path's gating.drive_gate: the supervisor lease is
    required even where require_supervisor is off (a bench launch), because a pp
    move is carried out by the drives and not re-commanded every tick.
      lease  (t_recv, generation, allowed bits) of the latest ControlLease
      panel  (t_recv, valid, mode_auto) of the latest PanelState
    """
    if not enabled:
        return "pp locked in the profile (pp.enabled false)"
    if lease is None or now - lease[0] > lease_timeout_s:
        return "no fresh supervisor lease"
    if not lease[2] & LEASE_COMMISSIONING:
        return "supervisor does not allow commissioning"
    if int(lease[1]) != int(move_generation):
        return f"move generation {move_generation} != lease {lease[1]}"
    if panel is None or now - panel[0] > panel_timeout_s:
        return "panel state stale"
    if not panel[1]:
        return "panel not valid"
    if panel[2]:
        return "selector is AUTO"
    return None
