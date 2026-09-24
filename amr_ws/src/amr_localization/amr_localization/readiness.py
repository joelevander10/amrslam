"""Localisation readiness (spec §4.3). Pure, clock-fed, no ROS.

States: UNLOCALIZED -> CHECKING (initial pose given) -> READY (automatic
conditions hold AND the operator confirmed scans align) -> LOST (a stream went
stale, covariance grew, or map->odom jumped beyond the trigger).

Low covariance alone cannot establish the correct aisle, so READY needs the
operator's confirmation; a later jump or staleness drops READY regardless of
covariance. Confirmation also needs a finite, recent scan-consistency comparison
made in the current attempt: no comparison is not the same as a good one. A
correction right after an initial pose is expected (the filter is
snapping to it) and is ignored for `initial_grace_s`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

UNLOCALIZED, CHECKING, READY, LOST = 0, 1, 2, 3
NAMES = {UNLOCALIZED: "UNLOCALIZED", CHECKING: "CHECKING", READY: "READY", LOST: "LOST"}


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _apply(frame: tuple[float, float, float], pose: tuple[float, float, float]) -> tuple[float, float, float]:
    """Pose expressed in `frame`'s child, returned in `frame`'s parent."""
    fx, fy, fyaw = frame
    px, py, pyaw = pose
    c, s = math.cos(fyaw), math.sin(fyaw)
    return fx + c * px - s * py, fy + s * px + c * py, _wrap(fyaw + pyaw)


@dataclass(frozen=True)
class Limits:
    cov_xy_max: float = 0.05  # m^2 on xx and yy
    cov_yaw_max: float = 0.03  # rad^2 (sigma 10 deg): a gross-loss check; jumps are the fine one
    cov_hold_s: float = 1.0  # covariance must stay over the limit this long before LOST
    min_scan_match: float = 0.6  # endpoints near mapped obstacles: operator-facing, gates confirm only
    max_scan_long: float = 0.15  # beams passing THROUGH mapped obstacles: the loss trigger
    match_hold_s: float = 1.0  # too many long beams must persist this long before LOST
    jump_dist_m: float = 0.15  # spec §4.3 initial trigger
    jump_angle_rad: float = math.radians(5.0)
    settle_s: float = 2.0  # no jump and covariance under limits for this long
    initial_grace_s: float = 1.5  # corrections right after /initialpose are expected
    amcl_age_max_s: float = 5.0  # AMCL only publishes on update; stationary is fine for a while
    match_age_max_s: float = 2.0  # a successful scan-vs-map comparison older than this proves nothing
    age_limits: dict = field(default_factory=lambda: {"scan": 0.15, "wheels": 0.10, "imu": 0.20, "tf": 0.20})
    # Safety stop (auto-resume plan 2026-09-19): when the safety chain takes the drives' torque
    # away, wheel feedback stops. The caller EXCUSES the wheels while the drives report torque
    # off (the vehicle cannot drive), and a READY state waits this long for that report before
    # a stale wheel stream counts as a loss - the drive status (10 Hz) can trail the wheel
    # timeout (0.10 s) by a tick.
    wheels_grace_s: float = 0.4


@dataclass
class Jump:
    dist_m: float
    angle_rad: float
    t: float


@dataclass
class Readiness:
    limits: Limits = field(default_factory=Limits)
    state: int = UNLOCALIZED
    reason: str = "no initial pose"
    confirmed: bool = False
    can_confirm: bool = False
    cov: tuple[float, float, float] | None = None  # xx, yy, yaw
    cov_t: float | None = None
    last_jump: Jump | None = None
    _initial_t: float | None = None
    _stable_since: float | None = None
    _cov_bad_since: float | None = None
    scan_match: float | None = None
    scan_long: float | None = None
    scan_match_t: float | None = None  # when the last SUCCESSFUL comparison was made (not scan receipt)
    _match_bad_since: float | None = None
    _prev_map_odom: tuple[float, float, float] | None = None
    _prev_map_odom_t: float | None = None

    # ---- inputs ------------------------------------------------------------

    def on_initialpose(self, t: float) -> None:
        """Operator (or explicit seed) estimate: (re)start validation."""
        self.state, self.reason = CHECKING, "initial pose received; converging"
        self.confirmed, self.can_confirm = False, False
        self._clear_evidence()
        self._initial_t = t

    def on_amcl_pose(
        self, t: float, cov_xx: float, cov_yy: float, cov_yaw: float, stamp: float | None = None
    ) -> None:
        """Latest AMCL covariance. A sample stamped before the current initial pose belongs to
        an earlier attempt and is ignored; a non-finite or negative covariance is not evidence
        of anything and replaces the held one with "unknown" (review Q07)."""
        if self._initial_t is not None and (stamp if stamp is not None else t) < self._initial_t:
            return
        vals = (cov_xx, cov_yy, cov_yaw)
        if not all(math.isfinite(v) and v >= 0.0 for v in vals):
            self.cov, self.cov_t = None, None
            return
        self.cov, self.cov_t = vals, t

    def on_scan_match(self, t: float, match: float, long: float, stamp: float | None = None) -> None:
        """Latest scan against the map at the estimated pose.

        `match`: fraction of endpoints within tolerance of an occupied cell.
        Unmapped clutter lowers it without the pose being wrong, so it only
        informs the operator and gates confirmation.
        `long`: fraction of beams that pass through a mapped wall AND end in
        mapped free space (scan_consistency.compare). Seeing through mesh or
        glass onto other mapped obstacles does not count; a wrong pose puts
        endpoints in free space. That cannot be clutter; sustained, it is a loss.
        `t` is when the comparison succeeded; `stamp` the scan's own time. A scan
        taken before the current initial pose belongs to an earlier attempt and
        is ignored; a non-finite result is unavailable evidence, not a match.
        """
        if self._initial_t is not None and (stamp if stamp is not None else t) < self._initial_t:
            return
        if not (math.isfinite(match) and math.isfinite(long)):
            self.scan_match = self.scan_long = self.scan_match_t = None
            return
        self.scan_match, self.scan_long, self.scan_match_t = match, long, t
        if long <= self.limits.max_scan_long:
            self._match_bad_since = None
            return
        if self._match_bad_since is None:
            self._match_bad_since = t
        elif self.state == READY and t - self._match_bad_since >= self.limits.match_hold_s:
            self._lose(f"scan passes through mapped obstacles ({100 * long:.0f} % of beams)")

    def on_map_odom(
        self,
        t: float,
        x: float,
        y: float,
        yaw: float,
        odom_base: tuple[float, float, float] | None = None,
    ) -> None:
        """One sample of map->odom. Detects corrections AT THE ROBOT between samples.

        map->odom is expressed at the odom origin, so a 0.5 deg yaw correction
        with the robot 20 m from where it started moves the frame 0.17 m while
        the robot's own map pose barely changes. The trigger therefore compares
        the robot's map pose under the previous and the new transform, using
        the same current odom pose (`odom_base` = x, y, yaw of base in odom).
        Without an odom pose the raw transform delta is used.
        """
        prev = self._prev_map_odom
        self._prev_map_odom, self._prev_map_odom_t = (x, y, yaw), t
        if prev is None:
            return
        if odom_base is None:
            d = math.hypot(x - prev[0], y - prev[1])
            a = abs(_wrap(yaw - prev[2]))
        else:
            px, py, _ = _apply(prev, odom_base)
            nx, ny, _ = _apply((x, y, yaw), odom_base)
            d = math.hypot(nx - px, ny - py)
            a = abs(_wrap(yaw - prev[2]))
        if d <= self.limits.jump_dist_m and a <= self.limits.jump_angle_rad:
            return
        if self._initial_t is not None and t - self._initial_t < self.limits.initial_grace_s:
            return  # snapping to the initial pose
        self.last_jump = Jump(d, a, t)
        self._stable_since = None
        why = f"localisation corrected by {d:.2f} m / {math.degrees(a):.1f} deg"
        if self.state == READY:
            self._lose(why)
        elif self.state == CHECKING:
            self.reason = why + "; waiting to settle"

    # ---- evaluation ----------------------------------------------------------

    def evaluate(self, t: float, ages: dict[str, float | None], excused: frozenset = frozenset()) -> int:
        """Apply staleness and covariance rules. `ages` keys: scan, wheels, imu, tf. Streams in
        `excused` are not stale (wheels while the drives report torque off: a safety stop)."""
        stale = [
            name
            for name, limit in self.limits.age_limits.items()
            if name not in excused and (ages.get(name) is None or ages[name] > limit)
        ]
        if self.state == READY and stale == ["wheels"]:
            a = ages.get("wheels")
            if a is not None and a <= self.limits.age_limits["wheels"] + self.limits.wheels_grace_s:
                stale = []  # a safety stop's drive report may still be on its way
        if self.state == UNLOCALIZED:
            self.can_confirm = False
            return self.state

        cov_ok = (
            self.cov is not None
            and self.cov[0] <= self.limits.cov_xy_max
            and self.cov[1] <= self.limits.cov_xy_max
            and self.cov[2] <= self.limits.cov_yaw_max
        )
        amcl_fresh = self.cov_t is not None and t - self.cov_t <= self.limits.amcl_age_max_s

        if self.state == READY:
            # READY is held on continuing evidence (Q07): the raw streams AND the independent
            # scan-vs-map comparison must keep arriving. AMCL itself may legitimately go quiet
            # while stationary, so its age is not a READY condition - the scan check is.
            match_stale = self.scan_match_t is None or t - self.scan_match_t > self.limits.match_age_max_s
            if stale:
                self._lose("stale: " + ", ".join(stale))
            elif match_stale:
                self._lose("scan-consistency check against the map stopped (gate or transform failure)")
            elif not cov_ok:
                # A single wide sample during a turn is a transient; sustained growth is a loss.
                if self._cov_bad_since is None:
                    self._cov_bad_since = t
                elif t - self._cov_bad_since >= self.limits.cov_hold_s:
                    self._lose(f"covariance grew: {self._cov_text()}")
            else:
                self._cov_bad_since = None
            return self.state

        if self.state == LOST:
            self.can_confirm = False
            return self.state

        # CHECKING
        if stale:
            self._stable_since = None
            self.reason = "stale: " + ", ".join(stale)
            self.can_confirm = False
            return self.state
        if not amcl_fresh:
            self._stable_since = None
            self.reason = "no AMCL update yet - drive slowly so the filter updates"
            self.can_confirm = False
            return self.state
        if not cov_ok:
            self._stable_since = None
            self.reason = f"not converged: {self._cov_text()}"
            self.can_confirm = False
            return self.state
        if self.scan_match_t is None or t - self.scan_match_t > self.limits.match_age_max_s:
            self._stable_since = None
            self.reason = "no recent scan-consistency check against the map"
            self.can_confirm = False
            return self.state
        if self.scan_long > self.limits.max_scan_long:
            self._stable_since = None
            self.reason = f"scan passes through mapped obstacles ({100 * self.scan_long:.0f} % of beams)"
            self.can_confirm = False
            return self.state
        if self.scan_match < self.limits.min_scan_match:
            self._stable_since = None
            self.reason = f"scan does not match the map ({100 * self.scan_match:.0f} % of beams)"
            self.can_confirm = False
            return self.state
        if self._stable_since is None:
            self._stable_since = t
        settled = t - self._stable_since >= self.limits.settle_s
        self.can_confirm = settled
        self.reason = (
            "converged and stable - confirm that scans align with fixed structure"
            if settled
            else f"converged; settling ({t - self._stable_since:.1f}/{self.limits.settle_s:.1f} s)"
        )
        return self.state

    # ---- operator ------------------------------------------------------------

    def confirm(self) -> bool:
        if self.state != CHECKING or not self.can_confirm:
            return False
        self.state, self.confirmed = READY, True
        self.reason = "operator confirmed scan alignment"
        return True

    def reset(self, reason: str = "reset by operator") -> None:
        """Back to UNLOCALIZED; every piece of readiness proof is dropped."""
        self.state, self.reason = UNLOCALIZED, reason
        self.confirmed, self.can_confirm = False, False
        self._clear_evidence()

    # ---- helpers -------------------------------------------------------------

    def _clear_evidence(self) -> None:
        """Covariance, scan consistency and jump history all belong to one attempt."""
        self._initial_t, self._stable_since = None, None
        self.cov = self.cov_t = self._cov_bad_since = None
        self.scan_match = self.scan_long = self.scan_match_t = self._match_bad_since = None
        self._prev_map_odom = self._prev_map_odom_t = self.last_jump = None

    def _lose(self, why: str) -> None:
        self.state, self.reason = LOST, why
        self.confirmed, self.can_confirm = False, False

    def _cov_text(self) -> str:
        if self.cov is None:
            return "no covariance"
        return f"xx {self.cov[0]:.3f} yy {self.cov[1]:.3f} yaw {self.cov[2]:.4f}"
