"""Commissioning (blind-run) job state machine (unified plan §7.2). Pure, clock-fed.

Wraps the repo's blind-run library (agv_core/blindrun.py: plan + BlindRun, the
straight/arc/pivot/pulses semantics, bounds, settling, per-segment records)
in the ROS-side authority rules:

    IDLE  --plan()-->  PREPARED  --Start edge under MANUAL, lease COMMISSIONING,
                                   fresh counts, still-->  RUNNING/SETTLING
          --clear()-->  IDLE          --library done-->  DONE
                                       --any loss-->     ABORTED

  * A plan is admitted only with a fresh supervisor lease that is not
    NAVIGATION, a fresh valid encoder scale and wheels known to be at rest. It
    is bound to that lease's (instance, generation) and to the scale; a later
    change of either drops a PREPARED plan (IDLE) and aborts a running job.
  * A plan upload moves nothing. Only a FRESH physical Start edge (this tick,
    from a valid MANUAL panel) starts a job, and it starts exactly one: the
    edge is consumed, it must be newer than the plan's acceptance, a held
    button does not restart, and DONE/ABORTED need a new plan before another
    Start can do anything.
  * Every tick of a running job re-checks the authority it started with:
    panel valid + MANUAL, supervisor lease with the COMMISSIONING class,
    fresh raw counts, the same lease identity and encoder scale, and no
    emergency. Loss of any -> ABORTED with the reason, output zero.
  * An EMERGENCY aborts a running job, never to resume: a new plan and Start
    are needed. A blind run is a single measured move, like a survey move
    (emergency-suite plan §4.6), and a move cut by an E-stop measures the stop,
    not the move (owner, 2026-11-02; the "never ends a program" rule of
    2026-10-30 is for runs, waits and the mapping session).
  * Output is per-wheel motor rpm in driver terms from the library, converted
    here to VEHICLE-terms wheel rad/s (undoing the invert flags the drive
    owner will apply again).

Profile velocity only on this vehicle: the profile-position (pp) backend was
removed on 2026-10-30 and is not part of this. The centre-path speed is capped
by blind_run.max_speed_mps.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field

from agv_core import (
    blindrun,  # repo module: agv_core/blindrun.py
    config,
)

# CommissioningState.phase. APPEND ONLY.
IDLE, PREPARED, RUNNING, SETTLING, DONE, ABORTED = range(6)
PHASE_NAMES = {
    IDLE: "IDLE",
    PREPARED: "PREPARED",
    RUNNING: "RUNNING",
    SETTLING: "SETTLING",
    DONE: "DONE",
    ABORTED: "ABORTED",
}
# A job exists (the supervisor grants COMMISSIONING instead of MANUAL, mode changes refused).
ACTIVE = (PREPARED, RUNNING, SETTLING)
LEASE_AUTONOMOUS, LEASE_COMMISSIONING = 2, 4
# A plain file-name stem: no path separators, no leading dot.
PLAN_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}")
WHEEL_RAD_S_PER_MOTOR_RPM = 2.0 * math.pi / 60.0 / config.GEAR_RATIO


@dataclass
class Inputs:
    """What the node sees this tick."""

    now: float
    dt: float
    counts: tuple[int, int] | None  # raw driver-terms 6064h, None if invalid/stale
    counts_per_rev: float
    stopped: bool | None  # both wheels at rest per the owner (None: unknown)
    panel_valid: bool
    panel_manual: bool
    start_edge: bool
    lease_allowed: int  # ControlLease.allowed bits, 0 if stale/none
    gyro_yaw_rad_s: float | None = None  # corrected yaw rate, for evidence; None if stale
    authority: tuple[str, int] | None = None  # fresh lease (instance, generation), None if stale/none
    start_edge_t: float | None = None  # when the Start edge was received (same clock as `now`)
    emergency: bool = False  # a fresh PanelState.emergency != EMG_NORMAL: abort, never resume


@dataclass
class Job:
    plan_id: str = ""
    planned: dict | None = None
    run: blindrun.BlindRun | None = None
    phase: int = IDLE
    reason: str = ""
    gyro_heading: float = 0.0
    results: list = field(default_factory=list)
    started_counts: tuple[int, int] | None = None
    authority: tuple[str, int] | None = None  # the lease identity the plan was admitted under
    counts_per_rev: float = 0.0  # the encoder scale the plan's count targets were computed with
    accepted_t: float | None = None
    aborted_segment: dict | None = None  # library state of the segment a loss interrupted
    gyro_gaps: list = field(default_factory=list)  # [start_s, end_s] since Start without a fresh gyro
    _started_t: float | None = None
    _gyro_gap_open: bool = False

    # -- operator actions (never move anything) --

    def plan(
        self,
        plan_json: str,
        counts_per_rev: float,
        *,
        now: float,
        authority: tuple[str, int] | None,
        lease_allowed: int,
        stopped: bool | None,
        mode: tuple[str, int, str] | None = None,
    ) -> str:
        """Admit a plan. `counts_per_rev` must come from fresh, valid wheel feedback (0 otherwise).
        `mode` is a fresh supervisor ModeState snapshot (instance, generation, mode name): a
        plan is admitted in IDLE only, under the same identity as the lease (review Q16) -
        the lease's permission bits alone cannot tell IDLE from MAPPING or an inhibited
        transition."""
        if self.phase in (RUNNING, SETTLING):
            raise ValueError("a job is running; abort it first")
        if authority is None:
            raise ValueError("no fresh supervisor lease: cannot bind a plan")
        if lease_allowed & LEASE_AUTONOMOUS:
            raise ValueError("supervisor is in NAVIGATION: blind runs need IDLE")
        if mode is None:
            raise ValueError("no fresh supervisor mode: blind runs need IDLE")
        if (mode[0], int(mode[1])) != (authority[0], int(authority[1])):
            raise ValueError("supervisor mode and lease disagree (transition in progress): try again")
        if mode[2] != "IDLE":
            raise ValueError(f"supervisor is in {mode[2]}: blind runs need IDLE")
        if stopped is not True:
            raise ValueError("wheels not known to be at rest")
        if not (math.isfinite(counts_per_rev) and counts_per_rev > 0.0):
            raise ValueError("no fresh valid encoder scale")
        try:
            spec = json.loads(plan_json)
            planned = blindrun.plan(spec.get("segments"), spec.get("speed"), counts_per_rev)
        except (ValueError, TypeError, AttributeError) as e:
            raise ValueError(str(e)) from None
        backend = spec.get("backend", "pv")
        if backend != "pv":
            raise ValueError("backend must be pv (profile position was removed on this vehicle)")
        speed = float(planned["speed_mps"])
        if speed > config.BLIND_MAX_SPEED_MPS + 1e-9:
            raise ValueError(
                f"speed {speed:.2f} m/s exceeds blind_run.max_speed_mps ({config.BLIND_MAX_SPEED_MPS:g})"
            )
        plan_id = spec.get("id")
        if plan_id is None or plan_id == "":
            plan_id = f"plan{len(planned['segments'])}"
        elif not (isinstance(plan_id, str) and PLAN_ID.fullmatch(plan_id)):
            raise ValueError("plan id must be 1-64 of A-Z a-z 0-9 _ . - starting with a letter or digit")
        self.planned, self.run, self.plan_id = planned, None, plan_id
        self.authority, self.counts_per_rev, self.accepted_t = tuple(authority), counts_per_rev, now
        self.phase, self.reason, self.results, self.gyro_heading = PREPARED, "", [], 0.0
        self.started_counts, self.aborted_segment, self.gyro_gaps = None, None, []
        self._started_t, self._gyro_gap_open = None, False
        return json.dumps(dict(planned, backend="pv"))

    def clear(self, reason: str = "cleared by operator") -> dict | None:
        """Drop the plan; a running job is aborted (output zero next tick).

        -> the aborted job's evidence (taken before the run is dropped), or None
        if nothing was running.
        """
        ev = None
        if self.phase in (RUNNING, SETTLING):
            self._abort(reason)
            ev = self.evidence()
        else:
            self.phase, self.reason = IDLE, ""
        self.planned = None
        self.run = None
        return ev

    # -- the tick --

    def _authority(self, i: Inputs) -> str | None:
        if i.emergency:
            return "emergency stop"
        if not (i.panel_valid and i.panel_manual):
            return "panel not valid MANUAL"
        if not (i.lease_allowed & LEASE_COMMISSIONING):
            return "supervisor does not allow commissioning"
        if i.authority != self.authority:
            return "supervisor lease is not the one the plan was accepted under"
        if i.counts is None:
            return "raw wheel counts stale or invalid"
        if i.counts_per_rev != self.counts_per_rev:
            return "encoder scale differs from the plan's"
        return None

    def _binding_changed(self, i: Inputs) -> str | None:
        """A definite change of authority or scale (not mere staleness) voids the plan."""
        if i.authority is not None and i.authority != self.authority:
            return "supervisor instance/generation changed"
        if i.counts_per_rev > 0.0 and i.counts_per_rev != self.counts_per_rev:
            return "encoder scale changed"
        return None

    def _abort(self, why: str) -> None:
        self.phase, self.reason = ABORTED, why
        self._gyro_gap_open = False
        self.run.abort(why)
        self.results = list(self.run.results)
        self.aborted_segment = dict(self.run.snapshot(), elapsed_s=self.run.seg_elapsed)

    def tick(self, i: Inputs) -> tuple[float, float]:
        """-> (left, right) wheel rad/s in vehicle terms; (0, 0) unless RUNNING/SETTLING."""
        if self.phase == PREPARED:
            changed = self._binding_changed(i)
            if changed:
                self.phase, self.reason = IDLE, f"plan {self.plan_id} dropped: {changed}"
                self.planned = None
                return 0.0, 0.0
            if not i.start_edge:
                return 0.0, 0.0
            if i.start_edge_t is None or self.accepted_t is None or i.start_edge_t <= self.accepted_t:
                self.reason = "Start ignored: pressed before the plan was accepted"
                return 0.0, 0.0
            why = self._authority(i)
            if why:
                self.reason = f"Start ignored: {why}"
                return 0.0, 0.0
            if i.stopped is not True:
                self.reason = "Start ignored: wheels not known to be at rest"
                return 0.0, 0.0
            self.run = blindrun.BlindRun(self.planned, i.counts)
            self.started_counts = tuple(i.counts)
            self._started_t = i.now
            self.phase, self.reason = RUNNING, ""
            return 0.0, 0.0  # first motion on the next tick, from a known baseline
        if self.phase not in (RUNNING, SETTLING):
            return 0.0, 0.0
        why = self._binding_changed(i) or self._authority(i)
        if why:
            self._abort(why)
            return 0.0, 0.0
        rel = i.now - (self._started_t if self._started_t is not None else i.now)
        if i.gyro_yaw_rad_s is not None and math.isfinite(i.gyro_yaw_rad_s):
            self.gyro_heading += i.gyro_yaw_rad_s * i.dt
            self._gyro_gap_open = False
        elif self._gyro_gap_open:
            self.gyro_gaps[-1][1] = rel
        else:
            self.gyro_gaps.append([max(0.0, rel - i.dt), rel])
            self._gyro_gap_open = True
        left_rpm, right_rpm = self.run.update(i.counts, i.dt, i.stopped)
        lib = self.run.phase
        if lib == blindrun.DONE:
            self.phase, self.reason = DONE, ""
        elif lib == blindrun.ABORTED:
            self.phase, self.reason = ABORTED, self.run.reason or "aborted"
            self.aborted_segment = dict(self.run.snapshot(), elapsed_s=self.run.seg_elapsed)
        elif lib == blindrun.SETTLING:
            self.phase = SETTLING
        else:
            self.phase = RUNNING
        self.results = list(self.run.results)
        if self.phase in (DONE, ABORTED):
            return 0.0, 0.0
        sl = -1.0 if config.INVERT_LEFT else 1.0
        sr = -1.0 if config.INVERT_RIGHT else 1.0
        return sl * left_rpm * WHEEL_RAD_S_PER_MOTOR_RPM, sr * right_rpm * WHEEL_RAD_S_PER_MOTOR_RPM

    def snapshot(self) -> dict:
        d = {}
        if self.run is not None:
            d.update(self.run.snapshot())  # library detail (its own lower-case phase is overridden below)
        elif self.planned is not None:
            d.update({"segment": 0, "segments": len(self.planned["segments"]), "completed": 0})
        d.update(
            {
                "phase": PHASE_NAMES[self.phase],
                "plan_id": self.plan_id,
                "reason": self.reason,
                "gyro_heading_deg": math.degrees(self.gyro_heading),
                "results": self.results,
            }
        )
        return d

    def evidence(self, extra: dict | None = None) -> dict:
        return {
            "plan_id": self.plan_id,
            "phase": PHASE_NAMES[self.phase],
            "reason": self.reason,
            "plan": self.planned,
            "started_counts": list(self.started_counts) if self.started_counts else None,
            "results": self.results,
            "gyro_heading_deg": math.degrees(self.gyro_heading),
            "gyro_gaps_s": [list(g) for g in self.gyro_gaps],  # intervals the heading has no gyro for
            "aborted_segment": self.aborted_segment,
            "supervisor_instance": self.authority[0] if self.authority else None,
            "supervisor_generation": self.authority[1] if self.authority else None,
            "counts_per_wheel_rev": self.counts_per_rev,
            "profile": config.PROFILE_NAME,
            "invert": [config.INVERT_LEFT, config.INVERT_RIGHT],
            "track_m": config.TRACK_M,
            "wheel_dia_m": config.WHEEL_DIA_M,
            "backend": "pv",
            **(extra or {}),
        }
