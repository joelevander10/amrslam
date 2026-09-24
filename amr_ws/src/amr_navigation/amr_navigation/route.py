"""The canonical route artefact (spec §6.4): editor and executor share it.

Human-readable degrees and cw/ccw in the file; the compiler converts to SI.
A route belongs to one map bundle revision, named by id, revision AND sha256,
so a re-surveyed map invalidates every route drawn on the old one.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import yaml

SCHEMA_VERSION = 1
ALLOWED_ANGLES_DEG = (45, 90, 180, 270)
DIRECTIONS = ("cw", "ccw")
STRAIGHT, ROTATE, REVERSE = "straight", "rotate", "reverse"
# A reverse straight (2026-09-18) backs up along the current heading, facing forward. The
# rear is outside the nanoScan3 field (275 deg, blind sector behind), so it is bounded:
# at most REVERSE_MAX_M and at half the route's base speed (never the long-straight boost).
REVERSE_MAX_M = 2.0
REVERSE_SPEED_RATIO = 0.5
# An arc (2026-09-18) drives forward along a circle of radius_m through angle_deg, left
# (ccw) or right (cw). Bounded: at least an eighth of a circle, at most a half, and a
# radius of at least twice the wheel track (2 x 0.487 -> 1.0 m). It runs at the route's
# arc_linear_mps (0.40 by default, 2026-09-19), capped so the yaw rate v/R stays under
# VEHICLE_ARC_W_MAX (compiler.arc_speed). Never the boost.
ARC = "arc"
ARC_MIN_DEG, ARC_MAX_DEG = 45.0, 180.0
ARC_MIN_RADIUS_M = 1.0
ARC_YAW_RATE_RATIO = 0.9  # of VEHICLE_ARC_W_MAX: headroom for the controller's corrections


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
# Vehicle ceilings (policy, 2026-09-18). nav2_params.yaml mirrors both (FollowPath
# desired_linear_vel = VEHICLE_V_MAX, Spin max_rotational_vel = VEHICLE_W_MAX; test_route.py
# checks). A route file may not ask for more linear speed than VEHICLE_V_MAX. Angular is
# CLAMPED, not refused: route files saved before 2026-09-17 carry the old 0.30 default,
# which Spin capped at 0.24 until 2026-09-18, so refusing them would only force a re-save.
VEHICLE_V_MAX = 0.85  # 0.70 -> 0.85 on 2026-09-19 (speed boost 2): only the long-straight boost
VEHICLE_W_MAX = 0.37  # spin: 0.24 -> 0.34 (+40 %) on 2026-09-18, -> 0.37 (+10 %) on 2026-09-19
# The base (non-boost) speed is bounded lower: every chain ENDS at it (the executor tapers a
# boosted chain down first), so RPP's approach ramp is planned from at most BASE_V_MAX.
BASE_V_MAX = 0.60
# Arcs turn at v/R: 0.40 m/s on the 1.0 m minimum radius is 0.40 rad/s, above the spin cap.
# Their own ceiling (the permit w_max of an arc step) keeps 0.40 m/s possible from R 1.0 m.
VEHICLE_ARC_W_MAX = 0.45
LIMIT_BOUNDS = {  # key -> (exclusive min, inclusive max)
    "linear_mps": (0.0, BASE_V_MAX),
    "long_linear_mps": (0.0, VEHICLE_V_MAX),
    "arc_linear_mps": (0.0, BASE_V_MAX),
    "long_min_length_m": (0.0, 100.0),
    "angular_rad_s": (0.0, 5.0),
    "position_tolerance_m": (0.0, 1.0),
    "heading_tolerance_deg": (0.0, 45.0),
    "cross_track_limit_m": (0.0, 1.0),
}
NULLABLE_LIMITS = ("long_linear_mps",)  # null / absent = feature off


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
    linear_mps: float = 0.55  # autonomous route default (spec §5.2; 0.40, 0.50 on 09-18, 0.55 on 09-19)
    # A straight LONGER than long_min_length_m may run at long_linear_mps ("boost"). None =
    # no boost: route files written before 2026-09-18 never speed up by themselves; the
    # editor writes the value explicitly into new routes.
    long_linear_mps: float | None = None
    long_min_length_m: float = 4.0
    # Arc speed (2026-09-19; before: 60 % of linear_mps). Absent in older files -> 0.40. Not
    # refused above linear_mps (every saved file carries it, a slow 0.30 route included):
    # compiler.arc_speed and the editor take the lower of the two.
    arc_linear_mps: float = 0.40
    angular_rad_s: float = 0.37  # route default (spec §5.3; 0.24 09-17, 0.34 09-18, 0.37 09-19)
    position_tolerance_m: float = 0.05
    heading_tolerance_deg: float = 2.0
    cross_track_limit_m: float = 0.10

    @property
    def w_mps(self) -> float:
        """The rotation speed the vehicle actually turns at: the file's value, capped."""
        return min(float(self.angular_rad_s), VEHICLE_W_MAX)


@dataclass
class Step:
    id: str
    type: str  # STRAIGHT | ROTATE | REVERSE | ARC
    to: tuple[float, float] | None = None  # straight: end point (x_m, y_m)
    direction: str | None = None  # rotate / arc: cw | ccw
    angle_deg: float | None = None  # rotate: 45 | 90 | 180 | 270; arc: ARC_MIN_DEG..ARC_MAX_DEG
    distance_m: float | None = None  # reverse: how far back, 0 < d <= REVERSE_MAX_M
    radius_m: float | None = None  # arc: >= ARC_MIN_RADIUS_M

    def to_dict(self) -> dict:
        if self.type == STRAIGHT:
            return {"id": self.id, "type": STRAIGHT, "to": {"x_m": self.to[0], "y_m": self.to[1]}}
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
        return {"id": self.id, "type": ROTATE, "direction": self.direction, "angle_deg": self.angle_deg}

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
            return Step(sid, STRAIGHT, to=(x, y))
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
        raise RouteError(f"unknown step type {t!r}"[:200], sid)


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

    def to_dict(self) -> dict:
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
        unknown = set(limits_in) - set(LIMIT_BOUNDS)
        if unknown:
            raise RouteError(f"unknown limits {sorted(map(str, unknown))}")
        limits = {}
        for k, v in limits_in.items():
            lo, hi = LIMIT_BOUNDS[k]
            if v is None and k in NULLABLE_LIMITS:
                limits[k] = None
                continue
            f = _float(v, f"limits.{k}")
            if not lo < f <= hi:
                raise RouteError(f"limits.{k} must be in ({lo:g}, {hi:g}], got {f:g}")
            limits[k] = f
        lim = Limits(**limits)
        if lim.long_linear_mps is not None and lim.long_linear_mps < lim.linear_mps:
            raise RouteError(
                f"limits.long_linear_mps ({lim.long_linear_mps:g}) is below linear_mps "
                f"({lim.linear_mps:g}): a long straight may not be slower than a short one"
            )
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
        )

    def dumps(self) -> str:
        return yaml.safe_dump(self.to_dict(), sort_keys=False)

    @staticmethod
    def loads(text: str) -> Route:
        return Route.from_dict(yaml.safe_load(text) or {})
