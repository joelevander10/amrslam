"""Command source selection for the mux (spec §2.1, §3.5; unified plan §4.3). Pure, clock-fed.

Authority comes from three places, never from the command streams themselves:
  * the supervisor's ControlLease: instance + generation + allowed classes,
    expiring 0.3 s after local receipt. Under `require_supervisor` nothing is
    selected without a fresh one; a lease from another instance or generation
    is not a lease;
  * the physical panel: selector MANUAL is manual authority, AUTO is the
    executor's; a stale or invalid panel image is no authority at all. Under
    MANUAL the panel image also carries the jog pendant levels, which win over
    every other manual stream while a direction is held;
  * the executor's MotionPermit: FOLLOW or ROTATE, expiring 0.3 s after local
    receipt, honoured only under AUTO, only with the AUTONOMOUS lease class,
    and only if it carries the lease's instance and generation.
The drive owner must also be fresh and operational for anything but zero.
Within an authority a command must still be fresh (0.2 s); a browser
ManualCommand is additionally bounded by the lifetime it carries. Expiry is
zero at once - the mux never lets a ramp extend a command that is gone.

Sequence monotonicity of permit.seq is enforced by the node, which is the thing
that sees the stream. The browser jog stream goes through ManualIntake, which
also owns session revocation (a release is terminal, whatever its seq).
"""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass

NONE, TELEOP, FOLLOW, ROTATE, MANUAL, COMMISSIONING, PENDANT = 0, 1, 2, 3, 4, 5, 6
NAMES = {
    NONE: "none",
    TELEOP: "teleop",
    FOLLOW: "follow",
    ROTATE: "rotate",
    MANUAL: "manual",
    COMMISSIONING: "commissioning",
    PENDANT: "pendant",
}

# ControlLease.allowed bits
LEASE_MANUAL, LEASE_AUTONOMOUS, LEASE_COMMISSIONING = 1, 2, 4


@dataclass(frozen=True)
class Params:
    cmd_timeout_s: float = 0.2
    teleop_window_s: float = 0.5
    permit_timeout_s: float = 0.3
    panel_timeout_s: float = 0.2
    lease_timeout_s: float = 0.3
    drives_timeout_s: float = 0.3
    require_supervisor: bool = False  # production sets True; the bench wrappers leave it False
    teleop_enabled: bool = True  # /cmd_vel_teleop is an engineering input; production turns it off
    # Jog pendant (2026-09-18): FWD/RVS drive at pendant_v (the FAST wheel's speed); with LEFT
    # or RIGHT held as well the vehicle arcs with the slow wheel at pendant_turn_ratio of the
    # fast one (no wheel ever exceeds pendant_v); LEFT/RIGHT alone spins in place at pendant_w.
    pendant_v: float = 0.50  # body m/s while FWD or RVS is held (0.60 tried and reverted 09-19)
    pendant_w: float = 0.39  # body rad/s for a spin in place (0.30 until 2026-09-19, +30 %)
    pendant_turn_ratio: float = 0.75  # slow wheel / fast wheel while driving and turning
    track_m: float = 0.487  # wheel track, for the arc's yaw rate (config.TRACK_M on the mux)


def survey_spin_cap(w: float, surveying: bool, cap: float) -> float:
    """A manual turn rate while surveying is capped at `cap` (2026-09-19: fast spins smear the
    scans slam_toolbox matches). Only |w| above the cap changes: a pendant arc at 0.50 m/s
    (0.26 rad/s) and the browser diagonals keep their shape."""
    if not surveying:
        return w
    return max(-cap, min(cap, w))


def pendant_twist(p: Params, fwd: bool, rvs: bool, left: bool, right: bool) -> tuple[float, float]:
    """Body (v, w) for the held pendant directions (opposing pairs already cancelled)."""
    drive = int(fwd) - int(rvs)
    turn = int(left) - int(right)
    if drive == 0:
        return 0.0, p.pendant_w * turn
    if turn == 0:
        return p.pendant_v * drive, 0.0
    fast, slow = p.pendant_v, p.pendant_v * p.pendant_turn_ratio
    v = drive * (fast + slow) / 2.0
    # +w turns left (ccw): the right wheel is the fast one. In reverse the same lever still
    # swings the front to the left (v < 0 and w > 0 arc the front leftwards).
    w = turn * (fast - slow) / p.track_m
    return v, w


DEFAULT = Params()


@dataclass
class Stamped:
    t: float
    v: float
    w: float


@dataclass
class Manual(Stamped):
    """A browser jog refresh (ManualCommand) as received."""

    instance: str = ""
    generation: int = 0
    session: str = ""
    seq: int = 0
    valid_for_s: float = 0.0


class ManualIntake:
    """ManualCommand stream filter (review R02, R03). Pure; the node feeds every sample.

    * valid_for_s <= 0 is a REVOCATION of that session: the held command is dropped
      at once (the next tick zeroes) and the session is tombstoned, so a delayed or
      reordered refresh from it can never revive it - a release carries no newer
      seq than the refresh it cancels, so seq ordering alone cannot express it.
      An empty session revokes whatever is held (/api/stop).
    * within a live session seq must increase (duplicates / reordering dropped);
    * a nonfinite sample drops the held command rather than leaving the previous
      nonzero one in force.
    """

    TOMBSTONES = 64

    def __init__(self) -> None:
        self.current: Manual | None = None
        self._revoked: OrderedDict[str, None] = OrderedDict()

    def offer(
        self,
        now: float,
        instance: str,
        generation: int,
        session: str,
        seq: int,
        valid_for_s: float,
        v: float,
        w: float,
    ) -> bool:
        """Returns True when the sample changed the held command."""
        if not (math.isfinite(v) and math.isfinite(w) and math.isfinite(valid_for_s)):
            self.current = None
            return True
        if session in self._revoked:
            return False
        cur = self.current
        if valid_for_s <= 0.0:
            # Tombstone the named session, and for a global stop (empty session) the one
            # currently held (review Q10): a refresh of it that arrives after the stop must
            # not revive it - the operator pressed Stop, a new press is required.
            for sid in (session, cur.session if cur is not None and not session else ""):
                if sid:
                    self._revoked[sid] = None
            while len(self._revoked) > self.TOMBSTONES:
                self._revoked.popitem(last=False)
            if cur is None or not session or cur.session == session:
                self.current = None
                return True
            return False
        if cur is not None and cur.session == session and seq <= cur.seq:
            return False  # reordered / duplicate refresh
        self.current = Manual(now, v, w, instance, int(generation), session, int(seq), float(valid_for_s))
        return True

    def clear(self) -> None:
        """Generation change: forget the held command (tombstones stay; ids are random)."""
        self.current = None


def finite_or_zero(t: float, v: float, w: float) -> Stamped:
    """A body-twist sample (teleop / Nav2). Nonfinite becomes an explicit zero (review R03)."""
    if math.isfinite(v) and math.isfinite(w):
        return Stamped(t, v, w)
    return Stamped(t, 0.0, 0.0)


@dataclass
class Wheels:
    """A per-wheel command (commissioning) as received."""

    t: float
    left: float
    right: float
    generation: int = 0


@dataclass
class Permit:
    t_recv: float
    source: int
    enabled: bool
    instance: str = ""
    generation: int = 0
    seq: int = 0
    v_max: float = 0.0  # route caps carried with the permission (review Q04); 0 = none
    w_max: float = 0.0


def capped(x: float, limit: float) -> float:
    """|x| clamped to `limit` when the limit is a positive finite number, else x unchanged."""
    if math.isfinite(limit) and limit > 0.0:
        return max(-limit, min(limit, x))
    return x


@dataclass
class Panel:
    t_recv: float
    valid: bool
    auto: bool
    # jog pendant direction levels, opposing pairs already cancelled
    fwd: bool = False
    rvs: bool = False
    left: bool = False
    right: bool = False


@dataclass
class Lease:
    t_recv: float
    instance: str
    generation: int
    seq: int
    allowed: int


@dataclass
class Drives:
    t_recv: float
    operational: bool


@dataclass
class Selection:
    source: int
    v: float
    w: float
    reason: str
    generation: int = 0  # the lease generation applied (0 unsupervised)
    inhibited: bool = False  # true when the supervisor/drives gate closed, not merely "no command"
    wheels: bool = False  # v/w are per-wheel rad/s (COMMISSIONING), not a body twist


def _fresh(t: float | None, now: float, limit: float) -> bool:
    return t is not None and now - t <= limit


def select(
    now: float,
    teleop: Stamped | None,
    follow: Stamped | None,
    rotate: Stamped | None,
    permit: Permit | None,
    panel: Panel | None,
    p: Params = DEFAULT,
    lease: Lease | None = None,
    manual: Manual | None = None,
    drives: Drives | None = None,
    commissioning: Wheels | None = None,
) -> Selection:
    gen = 0
    if p.require_supervisor:
        if lease is None or not _fresh(lease.t_recv, now, p.lease_timeout_s):
            return Selection(NONE, 0.0, 0.0, "no supervisor lease", 0, True)
        gen = lease.generation
        if lease.allowed == 0:
            return Selection(NONE, 0.0, 0.0, "inhibited by supervisor", gen, True)
        if drives is None or not _fresh(drives.t_recv, now, p.drives_timeout_s) or not drives.operational:
            return Selection(NONE, 0.0, 0.0, "drives not operational or stale", gen, True)

    panel_ok = panel is not None and panel.valid and _fresh(panel.t_recv, now, p.panel_timeout_s)
    if not panel_ok:
        return Selection(NONE, 0.0, 0.0, "no panel authority", gen)

    if not panel.auto:
        # Commissioning: the supervisor grants this class INSTEAD of MANUAL while a
        # job is prepared/running; only a fresh per-wheel command of this generation counts.
        if p.require_supervisor and (lease.allowed & LEASE_COMMISSIONING):
            c = commissioning
            if c is not None and c.generation == lease.generation and _fresh(c.t, now, p.cmd_timeout_s):
                return Selection(COMMISSIONING, c.left, c.right, "commissioning", gen, False, True)
            return Selection(NONE, 0.0, 0.0, "commissioning: no fresh wheel command", gen)
        if p.require_supervisor and not (lease.allowed & LEASE_MANUAL):
            return Selection(NONE, 0.0, 0.0, "MANUAL not allowed by supervisor", gen, True)
        # Physical pendant: a held deadman outranks any browser or keyboard stream.
        # Its freshness is the panel image's own (panel_ok above).
        if panel.fwd or panel.rvs or panel.left or panel.right:
            v, w = pendant_twist(p, panel.fwd, panel.rvs, panel.left, panel.right)
            return Selection(PENDANT, v, w, "pendant", gen)
        # Browser jog: bound by its own carried lifetime as well as the mux timeout.
        if manual is not None and p.require_supervisor:
            same = manual.instance == lease.instance and manual.generation == lease.generation
            limit = min(p.cmd_timeout_s, max(0.0, manual.valid_for_s))
            if same and _fresh(manual.t, now, limit):
                return Selection(MANUAL, manual.v, manual.w, "manual", gen)
        if p.teleop_enabled and teleop is not None and now - teleop.t <= p.teleop_window_s:
            if now - teleop.t <= p.cmd_timeout_s:
                return Selection(TELEOP, teleop.v, teleop.w, "teleop", gen)
            return Selection(TELEOP, 0.0, 0.0, "teleop command timed out", gen)
        return Selection(NONE, 0.0, 0.0, "MANUAL, no fresh command", gen)

    if p.require_supervisor and not (lease.allowed & LEASE_AUTONOMOUS):
        return Selection(NONE, 0.0, 0.0, "AUTO not allowed by supervisor", gen, True)
    permit_ok = permit is not None and permit.enabled and _fresh(permit.t_recv, now, p.permit_timeout_s)
    if not permit_ok:
        return Selection(NONE, 0.0, 0.0, "AUTO, no motion permit", gen)
    if p.require_supervisor and (permit.instance != lease.instance or permit.generation != lease.generation):
        return Selection(NONE, 0.0, 0.0, "permit from another generation", gen, True)
    # The route's authored limits travel with the permit and are enforced HERE, the last
    # software arbitration point (review Q04): Nav2 keeps its configured speeds.
    if permit.source == FOLLOW:
        if follow is not None and now - follow.t <= p.cmd_timeout_s:
            v, w = capped(follow.v, permit.v_max), capped(follow.w, permit.w_max)
            return Selection(FOLLOW, v, w, "follow", gen)
        return Selection(NONE, 0.0, 0.0, "permit FOLLOW, no fresh /cmd_vel", gen)
    if permit.source == ROTATE:
        if rotate is not None and now - rotate.t <= p.cmd_timeout_s:
            v, w = capped(rotate.v, permit.v_max), capped(rotate.w, permit.w_max)
            return Selection(ROTATE, v, w, "rotate", gen)
        return Selection(NONE, 0.0, 0.0, "permit ROTATE, no fresh /cmd_vel_rotate", gen)
    return Selection(NONE, 0.0, 0.0, "permit NONE", gen)


def nav_topic(base: str, generation: int) -> str:
    """Generation-private Nav2 output topic (unified plan §4.3 item 4).

    A controller from a replaced layer keeps publishing on ITS topic; the new
    mux listens on the new generation's, so the old stream cannot look fresh.
    Generation 0 (unsupervised bench) keeps the plain topic.
    """
    return base if generation == 0 else f"/amr/layers/g{generation}{base}"


def drive_gate(
    now: float,
    lease: Lease | None,
    cmd_generation: int,
    p: Params = DEFAULT,
) -> str | None:
    """The drive owner's independent check before any nonzero setpoint (§4.3 item 7).

    Returns None when the command may be applied, else the reason to zero.
    """
    if not p.require_supervisor:
        return None
    if lease is None or not _fresh(lease.t_recv, now, p.lease_timeout_s):
        return "no supervisor lease"
    if lease.allowed == 0:
        return "inhibited by supervisor"
    if cmd_generation != lease.generation:
        return f"command generation {cmd_generation} != lease {lease.generation}"
    return None
