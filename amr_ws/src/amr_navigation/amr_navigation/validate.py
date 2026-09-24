"""Route validation against a map bundle (spec §6.3, §6.4, §5.4). Errors block approval;
`info` issues (a sweep through a dynamic area, dynamic-mapping plan §1.2) never do."""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

from amr_maps import grid as gridio
from amr_navigation import footprint as fpmod
from amr_navigation.compiler import ARC, ROTATE, CompiledRoute, compile_route
from amr_navigation.route import MAX_REPEAT, Route, RouteError

ERROR, INFO = "error", "info"


@dataclass
class Issue:
    code: str
    message: str
    step_id: str | None = None
    severity: str = ERROR

    def to_dict(self) -> dict:
        return {
            "code": self.code,
            "message": self.message,
            "step_id": self.step_id,
            "severity": self.severity,
        }


@dataclass
class Validation:
    issues: list[Issue]
    compiled: CompiledRoute | None

    @property
    def errors(self) -> list[Issue]:
        return [i for i in self.issues if i.severity != INFO]

    @property
    def ok(self) -> bool:
        return not self.errors and self.compiled is not None


def load_mask(rev_dir: str, name: str) -> gridio.Grid | None:
    """A revision's optional mask grid (`keepout`, `dynamic`), or None when it has none."""
    path = os.path.join(rev_dir, f"{name}.yaml")
    return gridio.read(path) if os.path.isfile(path) else None


def load_keepout(rev_dir: str) -> gridio.Grid | None:
    return load_mask(rev_dir, "keepout")


def load_dynamic(rev_dir: str) -> gridio.Grid | None:
    return load_mask(rev_dir, "dynamic")


def validate(
    route: Route,
    manifest,
    grid: gridio.Grid,
    fp: fpmod.Footprint,
    keepout: gridio.Grid | None = None,
    dynamic: gridio.Grid | None = None,
) -> Validation:
    issues: list[Issue] = []
    if route.map.id != manifest.map_id or route.map.revision != manifest.revision:
        issues.append(
            Issue(
                "map_ref",
                f"route is for map {route.map.id}/rev{route.map.revision}, "
                f"loaded {manifest.map_id}/rev{manifest.revision}",
            )
        )
    elif route.map.sha256 != manifest.sha256:
        issues.append(
            Issue(
                "map_hash", "map bundle hash differs: the map was re-saved, re-validate on the new revision"
            )
        )
    if not route.route_id or "/" in route.route_id or route.route_id.startswith("."):
        issues.append(Issue("route_id", "route_id must be a plain name"))
    rc = route.repeat_count
    rc_ok = isinstance(rc, int) and not isinstance(rc, bool) and 1 <= rc <= MAX_REPEAT
    if not rc_ok:
        issues.append(Issue("repeat", f"repeat_count must be an integer in 1..{MAX_REPEAT}"))
    try:
        compiled = compile_route(route)
    except RouteError as e:
        issues.append(Issue("geometry", str(e), e.step_id))
        return Validation(issues, None)

    if rc_ok and rc > 1 and not compiled.closes:
        issues.append(
            Issue(
                "repeat",
                "repeat_count > 1 needs the route to end where it starts (within tolerance); "
                "add an explicit return sequence",
            )
        )
    try:
        gridio.require_axis_aligned(grid.meta)
    except gridio.GridError as e:
        issues.append(Issue("map_geometry", str(e)))
        return Validation(issues, compiled)
    if keepout is not None:
        problem = keepout_misalignment(grid, keepout)
        if problem:
            issues.append(Issue("keepout", f"keepout mask not aligned with the map: {problem}"))
            keepout = None  # the issue already fails validation; do not index a mismatched grid
    dyn: np.ndarray | None = None
    if dynamic is not None:
        problem = gridio.mask_misalignment(grid, dynamic)
        if problem:
            # like keepout: the issue fails validation and the mismatched grid is never indexed
            issues.append(Issue("dynamic", f"dynamic mask not aligned with the map: {problem}"))
        else:
            dyn = dynamic.data >= 65
    # Start pose itself must be clear (the vehicle stands there, at the start heading).
    start = (route.start.x_m, route.start.y_m, route.start.yaw_rad)
    c = _clearance(grid, fp, keepout, (start, start), None, dynamic=dyn)
    if not c.clear:
        issues.append(Issue("clearance", "start pose " + _describe(c)))
    elif c.provisional:
        issues.append(_provisional("start pose", c, None))
    for st in compiled.steps:
        if st.type == ROTATE:
            c = _clearance(grid, fp, keepout, None, (st.start, st.signed_angle_rad), dynamic=dyn)
        elif st.type == ARC:
            c = _clearance(
                grid,
                fp,
                keepout,
                None,
                None,
                (st.centre, st.radius_m, st.start[2], st.signed_angle_rad),
                dynamic=dyn,
            )
        else:  # straight or reverse: the footprint translated along the line, either way
            c = _clearance(grid, fp, keepout, (st.start, st.end), None, dynamic=dyn)
        what = {ROTATE: "rotation sweep", ARC: "arc sweep"}.get(st.type, "line")
        if not c.clear:
            issues.append(Issue("clearance", f"{what} {_describe(c)} (margin included)", st.id))
        elif c.provisional:
            issues.append(_provisional(what, c, st.id))
    return Validation(issues, compiled)


def _provisional(what: str, c: fpmod.Clearance, step_id: str | None) -> Issue:
    return Issue(
        "provisional",
        f"{what} crosses {c.provisional} mapped cells in a dynamic area: "
        "passable only if the live scan agrees (a return there stops the run)",
        step_id,
        INFO,
    )


def keepout_misalignment(grid: gridio.Grid, keepout: gridio.Grid) -> str | None:
    """None if the keepout grid indexes the same cells as the map, else what differs."""
    return gridio.mask_misalignment(grid, keepout)


def _clearance(grid, fp, keepout, line, turn, arc=None, dynamic=None) -> fpmod.Clearance:
    # Bounds first: a sweep leaving the map is rejected outright, and the (clipped) mask of
    # an off-map sweep is never rasterised, so absurd coordinates cost nothing.
    # `turn` is ((x, y, yaw), signed_angle): the exact sweep of that rotation, not a disc.
    # `arc` is (centre, radius, yaw0, signed_angle): the footprint driven along the circle.
    if line is not None:
        if fpmod.line_outside(grid, fp, *line):
            return fpmod.Clearance(0, 0, 0, 0, outside=True)
        return fpmod.check(grid, fpmod.swept_line(grid, fp, *line), keepout, dynamic=dynamic)
    if arc is not None:
        if fpmod.arc_outside(grid, fp, *arc):
            return fpmod.Clearance(0, 0, 0, 0, outside=True)
        return fpmod.check(grid, fpmod.swept_arc(grid, fp, *arc), keepout, dynamic=dynamic)
    (x, y, yaw), angle = turn
    if fpmod.rotation_outside(grid, fp, (x, y), yaw, angle):
        return fpmod.Clearance(0, 0, 0, 0, outside=True)
    return fpmod.check(grid, fpmod.swept_rotation(grid, fp, (x, y), yaw, angle), keepout, dynamic=dynamic)


def _describe(c: fpmod.Clearance) -> str:
    if c.outside:
        return "not clear: footprint (with margin) extends outside the map"
    return f"not clear: {c.occupied} occupied, {c.unknown} unknown, {c.keepout} keepout cells"
