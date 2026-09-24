"""Commissioning job state machine (unified plan §7.2). Pure, clock-fed.

Wraps the repo's blind-run library (core/blindrun.py: plan + BlindRun, the
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
    fresh raw counts, the same lease identity and encoder scale. Loss of any
    -> ABORTED with the reason, output zero.
  * Output is per-wheel motor rpm in driver terms from the library, converted
    here to VEHICLE-terms wheel rad/s (undoing the invert flags the drive
    owner will apply again).

Two backends, chosen per plan ("backend": "pv" | "pp"):

  pv  (default) the library's software position loop in profile velocity: this
      job computes wheel speeds every tick, through the mux, as before.
  pp  profile position, executed INSIDE the drives (amr_base/pp.py): one segment
      per plan; this job holds a run id at the drive owner (PpMove, re-sent every
      tick) and follows /drives/pp_status. Admitted and started only while the
      drive owner reports pp available - it is locked by the profile until the
      decision to run pp on this motor is recorded (pp.vendor_ref). An abort or clear sends hold=false
      for a few ticks, so the drive halts at once rather than on the stale timeout.

Both: the centre-path speed is capped by blind_run.max_speed_mps (and pp by
pp.max_speed_mps).
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field

from amr_base import pp
from amr_base.agv_repo import config

import blindrun  # repo module: core/blindrun.py

IDLE, PREPARED, RUNNING, SETTLING, DONE, ABORTED = range(6)
PHASE_NAMES = {
    IDLE: "IDLE",
    PREPARED: "PREPARED",
    RUNNING: "RUNNING",
    SETTLING: "SETTLING",
    DONE: "DONE",
    ABORTED: "ABORTED",
}
LEASE_AUTONOMOUS, LEASE_COMMISSIONING = 2, 4
BACKENDS = ("pv", "pp")
PP_STATUS_FRESH_S = 0.5  # /drives/pp_status is 10 Hz
PP_TAKE_S = 1.0  # the drive owner must report our run id this soon after Start
PP_RELEASE_TICKS = 10  # hold=false repeats after an abort/clear
# A plain file-name stem: no path separators, no leading dot.
PLAN_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}")
WHEEL_RAD_S_PER_MOTOR_RPM = 2.0 * math.pi / 60.0 / config.GEAR_RATIO


@dataclass
class PpView:
    """The drive owner's latest PpStatus, with its receive time."""

    t: float
    available: bool
    reason: str = ""
    state: str = "idle"
    run_id: str = ""
    outcome: str = ""
    targets: tuple[int, int] = (0, 0)
    actuals: tuple[int, int] = (0, 0)


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
    pp_status: PpView | None = None  # latest /drives/pp_status (pp backend only)


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
    backend: str = "pv"
    pp_spec: pp.MoveSpec | None = None
    run_id: str = ""  # pp: the id held at the drive owner for this Start
    pp_result: dict | None = None  # pp: what the drive owner reported at the end
    _release_ticks: int = 0  # pp: hold=false still to send after an abort/clear
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
        pp_status: PpView | None = None,
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
            raise ValueError("supervisor is in NAVIGATION: commissioning plans need IDLE")
        if mode is None:
            raise ValueError("no fresh supervisor mode: commissioning plans need IDLE")
        if (mode[0], int(mode[1])) != (authority[0], int(authority[1])):
            raise ValueError("supervisor mode and lease disagree (transition in progress): try again")
        if mode[2] != "IDLE":
            raise ValueError(f"supervisor is in {mode[2]}: commissioning plans need IDLE")
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
        if backend not in BACKENDS:
            raise ValueError(f"backend must be one of {', '.join(BACKENDS)}")
        speed = float(planned["speed_mps"])
        if speed > config.BLIND_MAX_SPEED_MPS + 1e-9:
            raise ValueError(
                f"speed {speed:.2f} m/s exceeds blind_run.max_speed_mps ({config.BLIND_MAX_SPEED_MPS:g})"
            )
        pp_spec = None
        if backend == "pp":
            if speed > config.PP_MAX_SPEED_MPS + 1e-9:
                raise ValueError(
                    f"speed {speed:.2f} m/s exceeds pp.max_speed_mps ({config.PP_MAX_SPEED_MPS:g})"
                )
            if len(planned["segments"]) != 1:
                raise ValueError("a pp plan is exactly one segment (one move per Start)")
            why = pp_unavailable(pp_status, now)
            if why:
                raise ValueError(f"pp not available: {why}")
            pp_spec = pp.move_spec(planned["segments"][0], config.BLIND_ACCEL_RPM_S, config.BLIND_ACCEL_RPM_S)
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
        self.backend, self.pp_spec, self.run_id, self.pp_result = backend, pp_spec, "", None
        out = dict(planned, backend=backend)
        if pp_spec is not None:
            out["pp"] = pp_spec_dict(pp_spec)
        return json.dumps(out)

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
        if self.backend == "pp":
            self._release_ticks = PP_RELEASE_TICKS
            return
        self.run.abort(why)
        self.results = list(self.run.results)
        self.aborted_segment = dict(self.run.snapshot(), elapsed_s=self.run.seg_elapsed)

    def pp_hold(self) -> tuple[str, bool] | None:
        """What the node publishes on /amr/commissioning_pp this tick: (run id, hold)
        while a pp job runs, (run id, False) for a few ticks after it ended early,
        else None (nothing to say)."""
        if self.backend != "pp" or not self.run_id:
            return None
        if self.phase == RUNNING:
            return self.run_id, True
        if self._release_ticks > 0:
            self._release_ticks -= 1
            return self.run_id, False
        return None

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
            if self.backend == "pp":
                why = pp_unavailable(i.pp_status, i.now)
                if why:
                    self.reason = f"Start ignored: pp not available: {why}"
                    return 0.0, 0.0
                self.run_id = f"{self.plan_id}-{int(i.now * 1000) % 100_000_000}"
                self.started_counts = tuple(i.counts)
                self._started_t = i.now
                self.phase, self.reason = RUNNING, ""
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
        if self.backend == "pp":
            self._pp_follow(i)
            return 0.0, 0.0
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

    def _pp_follow(self, i: Inputs) -> None:
        """RUNNING, pp: the drive owner does the move; this watches it end."""
        v = i.pp_status
        if v is None or i.now - v.t > PP_STATUS_FRESH_S:
            self._abort("drive pp status stale")
            return
        if v.run_id != self.run_id:
            if self._started_t is not None and i.now - self._started_t > PP_TAKE_S:
                self._abort(f"the drive owner did not take the pp move ({v.reason or v.state})")
            return
        if v.outcome == "":
            return  # moving / halting
        self.pp_result = {
            "outcome": v.outcome,
            "reason": v.reason,
            "targets": list(v.targets),
            "actuals": list(v.actuals),
            "error_counts": [a - t for a, t in zip(v.actuals, v.targets, strict=True)],
            "end_counts": list(i.counts) if i.counts else None,
            "duration_s": i.now - (self._started_t or i.now),
        }
        self.results = [dict(self.pp_result, segment=1, kind=self.planned["segments"][0]["kind"])]
        if v.outcome == pp.DONE:
            self.phase, self.reason = DONE, ""
        else:
            self.phase, self.reason = ABORTED, f"pp {v.outcome}: {v.reason}"
            self._release_ticks = PP_RELEASE_TICKS

    def pp_progress(self, counts: tuple[int, int] | None) -> list[float]:
        if counts is None or self.started_counts is None or not self.counts_per_rev:
            return [0.0, 0.0]
        m = math.pi * config.WHEEL_DIA_M / self.counts_per_rev
        return [blindrun.counts_delta(c, s) * m for c, s in zip(counts, self.started_counts, strict=True)]

    def snapshot(self) -> dict:
        d = {}
        if self.run is not None:
            d.update(self.run.snapshot())  # library detail (its own lower-case phase is overridden below)
        elif self.planned is not None:
            d.update({"segment": 0, "segments": len(self.planned["segments"]), "completed": 0})
            if self.backend == "pp" and self.phase in (RUNNING, SETTLING, DONE, ABORTED) and self.run_id:
                d.update(
                    {
                        "segment": 1,
                        "kind": self.planned["segments"][0]["kind"],
                        "speed_mps": self.planned.get("speed_mps", 0.0),
                        "completed": 1 if self.phase == DONE else 0,
                    }
                )
        d.update(
            {
                "phase": PHASE_NAMES[self.phase],
                "plan_id": self.plan_id,
                "reason": self.reason,
                "gyro_heading_deg": math.degrees(self.gyro_heading),
                "results": self.results,
                "backend": self.backend,
                "run_id": self.run_id,
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
            "backend": self.backend,
            "run_id": self.run_id,
            "pp_move": pp_spec_dict(self.pp_spec) if self.pp_spec else None,
            "pp_result": self.pp_result,
            "pp_config": {"expect": dict(config.PP_EXPECT), "vendor_ref": config.PP_VENDOR_REF}
            if self.backend == "pp"
            else None,
            **(extra or {}),
        }


def pp_unavailable(v: PpView | None, now: float) -> str | None:
    """None when the drive owner freshly reports pp available, else why not."""
    if v is None or now - v.t > PP_STATUS_FRESH_S:
        return "no fresh pp status from the drive owner"
    if not v.available:
        return v.reason or "drive owner reports pp unavailable"
    return None


def pp_spec_dict(s: pp.MoveSpec) -> dict:
    def w(m: pp.WheelMove) -> dict:
        return {
            "delta_counts": m.delta_counts,
            "velocity_rpm": m.velocity_rpm,
            "accel_rpm_s": m.accel_rpm_s,
            "decel_rpm_s": m.decel_rpm_s,
        }

    return {"left": w(s.left), "right": w(s.right), "duration_s": s.duration_s, "ratio_error": s.ratio_error}
