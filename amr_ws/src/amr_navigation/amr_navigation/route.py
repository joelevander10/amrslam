"""The canonical route artefact (spec §6.4): editor and executor share it.

Human-readable degrees and cw/ccw in the file; the compiler converts to SI.
A route belongs to one map bundle revision, named by id, revision AND sha256,
so a re-surveyed map invalidates every route drawn on the old one.
"""

from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass, field

import yaml

SCHEMA_VERSION = 1
ALLOWED_ANGLES_DEG = (45, 90, 180, 270)
DIRECTIONS = ("cw", "ccw")
STRAIGHT, ROTATE, REVERSE = "straight", "rotate", "reverse"
# A reverse straight (2026-09-18) backs up along the current heading, facing forward. The
# rear is outside the nanoScan3 field (275 deg, blind sector behind), so it is bounded:
# at most REVERSE_MAX_M and at half the trackless speed.
REVERSE_MAX_M = 2.0
REVERSE_SPEED_RATIO = 0.5
# ... and never above 0.30 m/s (gvievo-01's half of 0.60): a faster route speed (AMR QR 0.80,
# 2026-11-06) must not make the unscanned reverse faster.
REVERSE_V_MAX_MPS = 0.30
# An arc (2026-09-18) drives forward along a circle of radius_m through angle_deg, left
# (ccw) or right (cw). Bounded: at least an eighth of a circle, at most a half, and a
# radius of at least twice the wheel track (2 x 0.487 -> 1.0 m). It runs at ARC_V_MPS (owner
# 2026-11-03: 0.30 m/s), lowered further only where the yaw rate v/R would exceed
# VEHICLE_ARC_W_MAX (compiler.arc_speed).
ARC = "arc"
ARC_MIN_DEG, ARC_MAX_DEG = 45.0, 180.0
ARC_MIN_RADIUS_M = 1.0
ARC_YAW_RATE_RATIO = 0.9  # of VEHICLE_ARC_W_MAX: headroom for the controller's corrections
ARC_V_MPS = 0.30  # every arc (corner) at 0.30 m/s, whatever the straights run at (owner 2026-11-03)
# A wait (2026-10-30, gls-wait-coding-plan §A.1) stops at the previous step's end pose until
# the operator presses PB Continue (or Start): trolley hitch / release points. `name` is what
# the operator reads ("UBC hitch"); `label_id` names the GLS floor label under the camera
# there, a NAME (never a coordinate) checked against the map's label set. Placement rules
# (never first or last, never two in a row, never straight after an arc) are the compiler's.
WAIT = "wait"
WAIT_NAME_MAX = 40
LABEL_ID_RE = re.compile(r"^x\d{4}y\d{4}$")
# The home leg (2026-11-03): the route's last step drives forward from wherever the step
# before it ends to the route's start pose (HOME), arriving on HOME's heading. No parameters:
# the compiler builds it (amr_navigation.scurve.home_leg), a straight when the vehicle is
# already on HOME's line, else an S-curve. Bounded: it starts behind HOME, at most
# HOME_MAX_RUNUP_M back, within HOME_MAX_HEADING_DEG of HOME's heading; the curve never
# points more than HOME_MAX_SLOPE_DEG off HOME's line nor turns tighter than
# HOME_MIN_RADIUS_M (owner 2026-11-03: 1.0 m, the arc minimum).
HOME = "home"
HOME_MAX_RUNUP_M = 5.0
HOME_MAX_HEADING_DEG = 20.0
HOME_MAX_SLOPE_DEG = 30.0
HOME_MIN_RADIUS_M = 1.0


class RouteError(ValueError):
    """A route that cannot be compiled. `step_id` names the offending step when known."""

    def __init__(self, message: str, step_id: str | None = None):
        super().__init__(message)
        self.step_id = step_id


# Schema bounds (R21): enough for any real site, small enough that a malformed request
# cannot make the compiler or the sweep rasteriser do unbounded work.
MAX_STEPS = 500
MAX_REPEAT = 100  # = amr_mission run_fsm MAX_PASSES: the executor's pass bound
MAX_ABS_COORD_M = 10_000.0
MAX_NAME_LEN = 128
FRAMES = ("map",)
# Vehicle ceilings (policy). nav2_params.yaml mirrors both (FollowPath desired_linear_vel =
# VEHICLE_V_MAX, Spin max_rotational_vel = VEHICLE_W_MAX; test_route.py checks). Angular is
# CLAMPED, not refused: route files saved before 2026-09-17 carry the old 0.30 default,
# which Spin capped at 0.24 until 2026-09-18, so refusing them would only force a re-save.
#
# ONE trackless speed (2026-10-02): every forward straight and arc runs at VEHICLE_V_MAX.
# It is no longer a route setting; the long-straight boost and the separate arc speed are
# gone. Older files still load: their speed keys (LEGACY_SPEED_LIMITS) are read and dropped.
ROUTE_V_MPS_DEFAULT = 0.60  # straights; arcs run at ARC_V_MPS (owner 2026-11-03)
# A straight may carry its own, LOWER speed (2026-11-06, "slow zones"): v_mps on the step,
# STEP_V_MIN..STEP_V_MAX in the file, capped at the route speed when compiled. Split a straight
# into consecutive straights (they drive as one move, no stop) and give the middle one a lower
# speed: the vehicle slows before it (the executor's taper) and speeds up again after it.
STEP_V_MIN, STEP_V_MAX = 0.10, 2.0


def route_speed() -> float:
    """The straights' speed: qr_base.route_v_mps on the AMR QR (2026-11-06: 0.80), else
    ROUTE_V_MPS_DEFAULT. No profile (a dev box): the default."""
    try:
        from agv_core import config  # noqa: PLC0415 - optional, as in top_speed_cap

        if config.PLATFORM == config.PLATFORM_QR:
            return float(config.QR_ROUTE_V_MPS)
    except Exception:  # noqa: BLE001
        pass
    return ROUTE_V_MPS_DEFAULT


ROUTE_V_MPS = route_speed()


def top_speed_cap(v: float) -> float:
    """v, capped at 95 % of the vehicle's top speed (agv_core.config.MAX_SPEED_MPS, from
    motor_max_rpm). gvievo-01 (1.26 m/s) keeps 0.60. The AMR QR tops out near 0.28 m/s: a
    0.60 m/s route there would be clipped by the mux's wheel limit, which flattens the S-curve
    into a corner at the clip - so its routes are planned at what its wheels can do. No
    profile (a dev box): the route speed as it is."""
    try:
        from agv_core import config  # noqa: PLC0415 - optional: route.py stays usable without it

        return min(v, round(0.95 * float(config.MAX_SPEED_MPS), 2))
    except Exception:  # noqa: BLE001
        return v


_top_speed_cap = top_speed_cap  # the old private name
VEHICLE_V_MAX = top_speed_cap(ROUTE_V_MPS)
# Automatic speed changes on the AMR QR (owner 2026-11-06: "the slow down from straight to curve
# is too long"): the mux's AUTO a_max / d_max / auto_jerk (amr_bringup base.launch) and the route
# executor's taper and end approach (navigation_layer executor_overrides) use these. They are
# the values from before the 2026-11-03 softening: 0 -> 0.80 m/s in 4.5 s / 1.8 m, 0.80 ->
# 0.30 before an arc in 2.6 s / 1.7 m (was 5.3 s / 3.1 m), a stop from 0.80 in about 1.6 m.
QR_AUTO_A_MAX, QR_AUTO_D_MAX, QR_AUTO_JERK = 0.2, 0.25, 0.4
QR_END_DECEL = 0.2  # the executor's stop profile, under QR_AUTO_D_MAX so the mux can follow it
# RPP's approach ramp (approach_velocity_scaling_dist) only works INSIDE the local costmap: RPP
# prunes its path to the costmap's half width (nav2_params.yaml 5 x 5 m -> 2.5 m), so a ramp
# longer than that is ALWAYS active and caps every move at (half width / D) of its speed -
# 2026-11-06: D 5.2 m held 0.80 m/s straights at 0.38 and 0.30 m/s arcs at about 0.15. On the
# QR the executor brakes for the end of a move itself (QR_END_DECEL) and RPP only rounds off
# the last RPP_APPROACH_M.
LOCAL_COSTMAP_HALF_M = 2.5
RPP_APPROACH_M = 0.3


def _is_qr() -> bool:
    try:
        from agv_core import config  # noqa: PLC0415 - optional, as in top_speed_cap

        return config.PLATFORM == config.PLATFORM_QR
    except Exception:  # noqa: BLE001
        return False


def rpp_overrides(v: float | None = None, qr: bool | None = None) -> dict:
    """controller_server parameters for the AMR QR ({} elsewhere: nav2_params.yaml as is):
    desired_linear_vel = the route speed, and the short RPP approach ramp (see
    RPP_APPROACH_M; the executor's end profile does the braking). Pure but for the profile."""
    v = VEHICLE_V_MAX if v is None else float(v)
    if not (_is_qr() if qr is None else qr):
        return {}
    return {
        "FollowPath.desired_linear_vel": v,
        "FollowPath.approach_velocity_scaling_dist": RPP_APPROACH_M,
    }


VEHICLE_W_MAX = 0.37  # spin: 0.24 -> 0.34 (+40 %) on 2026-09-18, -> 0.37 (+10 %) on 2026-09-19
# Arcs turn at v/R: 0.40 m/s on the 1.0 m minimum radius is 0.40 rad/s, above the spin cap.
# Their own ceiling (the permit w_max of an arc step) keeps 0.40 m/s possible from R 1.0 m.
VEHICLE_ARC_W_MAX = 0.45
LIMIT_BOUNDS = {  # key -> (exclusive min, inclusive max)
    "angular_rad_s": (0.0, 5.0),
    "position_tolerance_m": (0.0, 1.0),
    "heading_tolerance_deg": (0.0, 45.0),
    "cross_track_limit_m": (0.0, 1.0),
}
LEGACY_SPEED_LIMITS = ("linear_mps", "long_linear_mps", "long_min_length_m", "arc_linear_mps")


def _field(d, key: str, where: str, step_id: str | None = None, default=None, required: bool = True):
    if not isinstance(d, dict):
        raise RouteError(f"{where} must be an object", step_id)
    if key not in d:
        if required:
            raise RouteError(f"{where}.{key} is required", step_id)
        return default
    return d[key]


def _int(v, where: str, lo: int, hi: int, step_id: str | None = None) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise RouteError(f"{where} must be an integer, got {v!r}", step_id)
    if not lo <= v <= hi:
        raise RouteError(f"{where} must be in {lo}..{hi}, got {v}", step_id)
    return v


def _float(v, where: str, lim: float = MAX_ABS_COORD_M, step_id: str | None = None) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise RouteError(f"{where} must be a finite number, got {v!r}", step_id)
    if abs(v) > lim:
        raise RouteError(f"{where} out of range (|value| <= {lim:g})", step_id)
    return float(v)


def _str(v, where: str, step_id: str | None = None) -> str:
    if not isinstance(v, str) or len(v) > MAX_NAME_LEN:
        raise RouteError(f"{where} must be a string of at most {MAX_NAME_LEN} characters", step_id)
    return v


@dataclass
class MapRef:
    id: str
    revision: int
    sha256: str


@dataclass
class StartPose:
    x_m: float
    y_m: float
    yaw_deg: float

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)


@dataclass
class Limits:
    angular_rad_s: float = 0.37  # route default (spec §5.3; 0.24 09-17, 0.34 09-18, 0.37 09-19)
    position_tolerance_m: float = 0.05
    heading_tolerance_deg: float = 2.0
    cross_track_limit_m: float = 0.10

    @property
    def linear_mps(self) -> float:
        """The trackless speed: a vehicle constant, not a route setting (2026-10-02)."""
        return VEHICLE_V_MAX

    @property
    def w_mps(self) -> float:
        """The rotation speed the vehicle actually turns at: the file's value, capped."""
        return min(float(self.angular_rad_s), VEHICLE_W_MAX)


@dataclass
class Step:
    id: str
    type: str  # STRAIGHT | ROTATE | REVERSE | ARC | WAIT | HOME
    to: tuple[float, float] | None = None  # straight: end point (x_m, y_m)
    direction: str | None = None  # rotate / arc: cw | ccw
    angle_deg: float | None = None  # rotate: 45 | 90 | 180 | 270; arc: ARC_MIN_DEG..ARC_MAX_DEG
    distance_m: float | None = None  # reverse: how far back, 0 < d <= REVERSE_MAX_M
    radius_m: float | None = None  # arc: >= ARC_MIN_RADIUS_M
    name: str = ""  # wait: shown to the operator
    label_id: str | None = None  # wait: the GLS floor label expected there, or None
    v_mps: float | None = None  # straight: its own lower speed (slow zone), None = the route speed

    def to_dict(self) -> dict:
        if self.type == STRAIGHT:
            d = {"id": self.id, "type": STRAIGHT, "to": {"x_m": self.to[0], "y_m": self.to[1]}}
            if self.v_mps is not None:
                d["v_mps"] = self.v_mps
            return d
        if self.type == REVERSE:
            return {"id": self.id, "type": REVERSE, "distance_m": self.distance_m}
        if self.type == ARC:
            return {
                "id": self.id,
                "type": ARC,
                "direction": self.direction,
                "angle_deg": self.angle_deg,
                "radius_m": self.radius_m,
            }
        if self.type == WAIT:
            d = {"id": self.id, "type": WAIT, "name": self.name}
            if self.label_id is not None:
                d["label_id"] = self.label_id
            return d
        if self.type == HOME:
            return {"id": self.id, "type": HOME}
        if self.type == ROTATE:
            return {"id": self.id, "type": ROTATE, "direction": self.direction, "angle_deg": self.angle_deg}
        raise RouteError(f"unknown step type {self.type!r}"[:200], self.id)

    @staticmethod
    def from_dict(d: dict) -> Step:
        if not isinstance(d, dict):
            raise RouteError(f"each step must be an object, got {d!r}"[:200])
        sid = _str(d.get("id", ""), "step.id")
        t = d.get("type")
        if t == STRAIGHT:
            to = _field(d, "to", "step", sid)
            x = _float(_field(to, "x_m", "step.to", sid), "step.to.x_m", step_id=sid)
            y = _float(_field(to, "y_m", "step.to", sid), "step.to.y_m", step_id=sid)
            v = d.get("v_mps")
            if v is not None:
                v = _float(v, "step.v_mps", lim=STEP_V_MAX, step_id=sid)
                if not STEP_V_MIN <= v <= STEP_V_MAX:
                    raise RouteError(
                        f"straight v_mps must be in [{STEP_V_MIN:g}, {STEP_V_MAX:g}] m/s, got {v:g}", sid
                    )
            return Step(sid, STRAIGHT, to=(x, y), v_mps=v)
        if t == ROTATE:
            direction = _field(d, "direction", "step", sid)
            if direction not in DIRECTIONS:
                raise RouteError(f"direction must be one of {DIRECTIONS}", sid)
            a = _field(d, "angle_deg", "step", sid)
            # stored revisions carry 90.0; an integral float is the same angle, a fraction is not
            if isinstance(a, float) and math.isfinite(a) and a.is_integer():
                a = int(a)
            if isinstance(a, bool) or not isinstance(a, int) or a not in ALLOWED_ANGLES_DEG:
                raise RouteError(f"angle_deg must be one of {ALLOWED_ANGLES_DEG}", sid)
            return Step(sid, ROTATE, direction=direction, angle_deg=float(a))
        if t == REVERSE:
            raw = _field(d, "distance_m", "step", sid)
            dist = _float(raw, "step.distance_m", lim=REVERSE_MAX_M, step_id=sid)
            if not 0.0 < dist <= REVERSE_MAX_M:
                raise RouteError(f"reverse distance_m must be in (0, {REVERSE_MAX_M:g}] m, got {dist:g}", sid)
            return Step(sid, REVERSE, distance_m=dist)
        if t == ARC:
            direction = _field(d, "direction", "step", sid)
            if direction not in DIRECTIONS:
                raise RouteError(f"direction must be one of {DIRECTIONS}", sid)
            a = _float(_field(d, "angle_deg", "step", sid), "step.angle_deg", lim=360.0, step_id=sid)
            if not ARC_MIN_DEG <= a <= ARC_MAX_DEG:
                raise RouteError(
                    f"arc angle_deg must be in [{ARC_MIN_DEG:g}, {ARC_MAX_DEG:g}], got {a:g}", sid
                )
            r = _float(_field(d, "radius_m", "step", sid), "step.radius_m", lim=1000.0, step_id=sid)
            if r < ARC_MIN_RADIUS_M:
                raise RouteError(f"arc radius_m must be at least {ARC_MIN_RADIUS_M:g} m, got {r:g}", sid)
            return Step(sid, ARC, direction=direction, angle_deg=a, radius_m=r)
        if t == WAIT:
            name = d.get("name", "")
            if not isinstance(name, str) or len(name) > WAIT_NAME_MAX:
                raise RouteError(f"wait name must be a string of at most {WAIT_NAME_MAX} characters", sid)
            label = d.get("label_id")
            if label is not None and (not isinstance(label, str) or not LABEL_ID_RE.match(label)):
                raise RouteError(f"wait label_id must look like x0100y0000, got {label!r}"[:200], sid)
            return Step(sid, WAIT, name=name, label_id=label)
        if t == HOME:
            return Step(sid, HOME)
        raise RouteError(f"unknown step type {t!r}"[:200], sid)


@dataclass
class Branch:
    """A POS route (2026-11-06): the steps of a MAIN route up to `leave_after`, its own branch
    steps (`steps`, ids), then the main route again after `rejoin_after` - same start, same
    homepost. The file holds every step (it runs like any other route); this records which
    are the branch so the editor can rebase it on a newer main revision, and validation can
    check the main part is still exactly main rev `main_revision` (branch_problem)."""

    main_route_id: str
    main_revision: int
    leave_after: str  # a main step id: the branch starts at its end
    rejoin_after: str  # a main step id (at or after leave_after): the main route resumes after it
    steps: list[str] = field(default_factory=list)  # the branch's own step ids, in order

    @staticmethod
    def from_dict(d) -> Branch:
        if not isinstance(d, dict):
            raise RouteError("branch must be an object")
        ids = d.get("steps")
        if not isinstance(ids, list) or not ids or not all(isinstance(i, str) and i for i in ids):
            raise RouteError("branch.steps must be a non-empty list of step ids")
        return Branch(
            main_route_id=_str(_field(d, "main_route_id", "branch"), "branch.main_route_id"),
            main_revision=_int(_field(d, "main_revision", "branch"), "branch.main_revision", 1, 2**31 - 1),
            leave_after=_str(_field(d, "leave_after", "branch"), "branch.leave_after"),
            rejoin_after=_str(_field(d, "rejoin_after", "branch"), "branch.rejoin_after"),
            steps=list(ids),
        )


def branch_problem(route: Route, main: Route) -> str | None:
    """Why `route` is not main + its branch (None if it is): the same start, main's steps up
    to leave_after exactly, the branch steps, main's steps after rejoin_after exactly. Pure."""
    b = route.branch
    if b is None:
        return None
    if main.branch is not None:
        return f"main route {b.main_route_id} is itself a branch route: branch from a plain route"
    ids = [s.id for s in main.steps]
    for key in ("leave_after", "rejoin_after"):
        if getattr(b, key) not in ids:
            return f"{b.main_route_id} rev{b.main_revision} has no step {getattr(b, key)!r} ({key})"
    li, ri = ids.index(b.leave_after), ids.index(b.rejoin_after)
    if ri < li:
        return f"rejoin_after {b.rejoin_after} comes before leave_after {b.leave_after} on the main route"
    clash = sorted(set(b.steps) & set(ids))
    if clash:
        return f"branch step id(s) {clash} are also main route step ids"
    mine = {s.id: s for s in route.steps}
    if any(i not in mine for i in b.steps):
        return "branch.steps names a step that is not in the route"
    want = main.steps[: li + 1] + [mine[i] for i in b.steps] + main.steps[ri + 1 :]
    if (route.start.x_m, route.start.y_m, route.start.yaw_deg) != (
        main.start.x_m,
        main.start.y_m,
        main.start.yaw_deg,
    ):
        return f"the start is not {b.main_route_id} rev{b.main_revision}'s start: rebase on the main route"
    got = [s.to_dict() for s in route.steps]
    exp = [s.to_dict() for s in want]
    if got != exp:
        k = next((i for i, (g, e) in enumerate(zip(got, exp)) if g != e), min(len(got), len(exp)))  # noqa: B905
        where = got[k]["id"] if k < len(got) else "the end"
        return (
            f"the main part differs from {b.main_route_id} rev{b.main_revision} at {where}: "
            "rebase on the main route (Routes page, Branch)"
        )
    return None


@dataclass
class Route:
    route_id: str
    map: MapRef
    start: StartPose
    steps: list[Step] = field(default_factory=list)
    limits: Limits = field(default_factory=Limits)
    revision: int = 0  # 0 = unsaved draft
    repeat_count: int = 1
    frame_id: str = "map"
    schema_version: int = SCHEMA_VERSION
    branch: Branch | None = None  # a POS route: main route + its branch (see Branch)

    def to_dict(self) -> dict:
        d = self._dict()
        if self.branch is not None:
            d["branch"] = asdict(self.branch)
        return d

    def _dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "route_id": self.route_id,
            "revision": self.revision,
            "map": asdict(self.map),
            "frame_id": self.frame_id,
            "start": asdict(self.start),
            "limits": asdict(self.limits),
            "steps": [s.to_dict() for s in self.steps],
            "repeat_count": self.repeat_count,
        }

    @staticmethod
    def from_dict(d: dict) -> Route:
        """Strict: every malformed field is a RouteError naming it, never a TypeError,
        a silent coercion (2.5 -> 2, "3" -> 3) or a default standing in for a missing pose."""
        if not isinstance(d, dict):
            raise RouteError("route must be an object")
        sv = d.get("schema_version", 0)
        if isinstance(sv, bool) or not isinstance(sv, int) or sv != SCHEMA_VERSION:
            raise RouteError(f"schema_version must be {SCHEMA_VERSION}")
        m = d.get("map") or {}
        if not isinstance(m, dict):
            raise RouteError("map must be an object")
        st = _field(d, "start", "route")
        frame_id = _str(d.get("frame_id", "map"), "frame_id")
        if frame_id not in FRAMES:
            raise RouteError(f"frame_id must be one of {FRAMES}")
        limits_in = d.get("limits") or {}
        if not isinstance(limits_in, dict):
            raise RouteError("limits must be an object")
        unknown = set(limits_in) - set(LIMIT_BOUNDS) - set(LEGACY_SPEED_LIMITS)
        if unknown:
            raise RouteError(f"unknown limits {sorted(map(str, unknown))}")
        limits = {}
        for k, v in limits_in.items():
            if k in LEGACY_SPEED_LIMITS:
                continue  # a speed saved before 2026-10-02: the vehicle's trackless speed wins
            lo, hi = LIMIT_BOUNDS[k]
            f = _float(v, f"limits.{k}")
            if not lo < f <= hi:
                raise RouteError(f"limits.{k} must be in ({lo:g}, {hi:g}], got {f:g}")
            limits[k] = f
        lim = Limits(**limits)
        steps_in = d.get("steps") or []
        if not isinstance(steps_in, list):
            raise RouteError("steps must be a list")
        if len(steps_in) > MAX_STEPS:
            raise RouteError(f"too many steps ({len(steps_in)} > {MAX_STEPS})")
        return Route(
            route_id=_str(d.get("route_id", ""), "route_id"),
            revision=_int(d.get("revision", 0), "revision", 0, 2**31 - 1),
            map=MapRef(
                _str(m.get("id", ""), "map.id"),
                _int(m.get("revision", 0), "map.revision", 0, 2**31 - 1),
                _str(m.get("sha256", ""), "map.sha256"),
            ),
            frame_id=frame_id,
            start=StartPose(
                _float(_field(st, "x_m", "start"), "start.x_m"),
                _float(_field(st, "y_m", "start"), "start.y_m"),
                _float(_field(st, "yaw_deg", "start"), "start.yaw_deg", lim=3600.0),
            ),
            limits=lim,
            steps=[Step.from_dict(s) for s in steps_in],
            repeat_count=_int(d.get("repeat_count", 1), "repeat_count", 0, MAX_REPEAT),
            branch=None if d.get("branch") is None else Branch.from_dict(d["branch"]),
        )

    def dumps(self) -> str:
        return yaml.safe_dump(self.to_dict(), sort_keys=False)

    @staticmethod
    def loads(text: str) -> Route:
        return Route.from_dict(yaml.safe_load(text) or {})
