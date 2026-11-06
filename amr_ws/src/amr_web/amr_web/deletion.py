"""Remove maps, routes and missions from the web (2026-11-06), recoverably.

Nothing is erased: everything removed is MOVED into <maps_dir>/.trash/<time>-<what>/ with its
path inside the store kept, so an engineer can put it back by hand (or empty the trash with
`rm -rf <maps_dir>/.trash`). A dot name is never a map id or a mission (store._plain), so the
trash never shows up in a list.

What goes with what - nothing may be left pointing at something that is gone:
  * a whole map: its directory (every revision, its routes, its floor labels) and every mission
    on it;
  * one map revision: that revision, the route revisions drawn on it, the missions using either.
    The last revision of a map is the whole map;
  * a route (all revisions) or one route revision: those files, and the missions using them;
  * a mission: its file.

plan_*() only looks (pure but for reading the store); execute() moves. The server refuses what
is in use (the active map, the loaded mission) before either runs.
"""

from __future__ import annotations

import os
import re
import time
from dataclasses import dataclass, field

from amr_mission import map_bundle as mb
from amr_navigation import store

TRASH = ".trash"


class DeleteError(ValueError):
    pass


@dataclass
class Plan:
    what: str  # "map block_c rev2", for messages and the trash folder name
    paths: list[str] = field(default_factory=list)  # files / directories to move
    missions: list[str] = field(default_factory=list)  # mission ids removed with it
    routes: list[str] = field(default_factory=list)  # "route_a rev3", for the message
    map_revisions: list[tuple[str, int]] = field(default_factory=list)  # for the server's caches

    def summary(self) -> str:
        parts = [self.what]
        extra = [r for r in self.routes if not self.what.startswith("route")]
        if extra:
            parts.append(
                f"{len(extra)} route revision(s) ({', '.join(extra[:6])}{', ...' if len(extra) > 6 else ''})"
            )
        if self.missions:
            m = self.missions
            parts.append(f"{len(m)} mission(s) ({', '.join(m[:6])}{', ...' if len(m) > 6 else ''})")
        return " + ".join(parts)


def _map_dir(maps_dir: str, map_id: str) -> str:
    d = os.path.join(maps_dir, store._name("map id", map_id))
    if not os.path.isdir(d):
        raise DeleteError(f"no such map: {map_id}")
    return d


def _missions(maps_dir: str, pick) -> list[str]:
    return sorted(m["mission_id"] for m in store.list_missions(maps_dir) if isinstance(m, dict) and pick(m))


def _ref(m: dict, key: str) -> tuple[str, int]:
    r = m.get(key) or {}
    return str(r.get("id", "")), int(r.get("revision", 0) or 0)


def _route_map_revision(maps_dir: str, map_id: str, route_id: str, rrev: int) -> int | None:
    """The map revision a stored route was drawn on (None if unreadable)."""
    try:
        route, _ = store.load_route(maps_dir, map_id, route_id, rrev)
    except Exception:  # noqa: BLE001 - an unreadable route is not this revision's
        return None
    return int(route.map.revision)


def plan_map(maps_dir: str, map_id: str, revision: int | None = None) -> Plan:
    d = _map_dir(maps_dir, map_id)
    revs = mb.list_revisions(maps_dir, map_id)
    if revision is not None and revision not in revs:
        raise DeleteError(f"no such map revision: {map_id} rev{revision}")
    if revision is None or revs == [revision]:
        routes = store.list_routes(maps_dir, map_id)
        return Plan(
            what=f"map {map_id} (all {len(revs)} revision(s))"
            if revision is None
            else f"map {map_id} rev{revision} (its last)",
            paths=[d],
            missions=_missions(maps_dir, lambda m: _ref(m, "map")[0] == map_id),
            routes=[f"{rid} rev{r}" for rid, rr in routes.items() for r in rr],
            map_revisions=[(map_id, r) for r in revs],
        )
    plan = Plan(what=f"map {map_id} rev{revision}", map_revisions=[(map_id, revision)])
    plan.paths.append(mb.revision_dir(maps_dir, map_id, revision))
    gone: set[tuple[str, int]] = set()
    for rid, rrevs in store.list_routes(maps_dir, map_id).items():
        drawn = [r for r in rrevs if _route_map_revision(maps_dir, map_id, rid, r) == revision]
        _add_route_files(maps_dir, map_id, rid, rrevs, drawn, plan)
        gone |= {(rid, r) for r in drawn}
    plan.missions = _missions(
        maps_dir,
        lambda m: (
            _ref(m, "map") == (map_id, revision) or (_ref(m, "map")[0] == map_id and _ref(m, "route") in gone)
        ),
    )
    return plan


def _add_route_files(maps_dir, map_id, rid, all_revs, drop, plan: Plan) -> None:
    if not drop:
        return
    if set(drop) == set(all_revs):
        plan.paths.append(store.routes_dir(maps_dir, map_id, rid))  # the whole route
    else:
        plan.paths += [store.route_path(maps_dir, map_id, rid, r) for r in drop]
    plan.routes += [f"{rid} rev{r}" for r in sorted(drop)]


def plan_route(maps_dir: str, map_id: str, route_id: str, revision: int | None = None) -> Plan:
    _map_dir(maps_dir, map_id)
    revs = store.list_routes(maps_dir, map_id).get(store._name("route_id", route_id))
    if not revs:
        raise DeleteError(f"no such route: {route_id} on {map_id}")
    if revision is not None and revision not in revs:
        raise DeleteError(f"no such route revision: {route_id} rev{revision}")
    drop = revs if revision is None else [revision]
    plan = Plan(
        what=f"route {route_id} "
        + (f"(all {len(revs)} revision(s))" if revision is None else f"rev{revision}")
    )
    _add_route_files(maps_dir, map_id, route_id, revs, drop, plan)
    gone = {(route_id, r) for r in drop}
    plan.missions = _missions(maps_dir, lambda m: _ref(m, "map")[0] == map_id and _ref(m, "route") in gone)
    return plan


def plan_mission(maps_dir: str, mission_id: str) -> Plan:
    store.load_mission(maps_dir, mission_id)  # exists and is one (raises StoreError)
    return Plan(what=f"mission {mission_id}", missions=[mission_id])


def execute(maps_dir: str, plan: Plan, now: float | None = None) -> str:
    """Move everything in the plan into a new trash folder; returns that folder."""
    root = os.path.realpath(maps_dir)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", plan.what)[:60].strip("_")
    dest = os.path.join(maps_dir, TRASH, f"{stamp}-{slug}")
    n = 1
    while os.path.exists(dest):
        n += 1
        dest = os.path.join(maps_dir, TRASH, f"{stamp}-{slug}-{n}")
    paths = list(plan.paths) + [
        os.path.join(store.missions_dir(maps_dir), f"{store._name('mission_id', m)}.yaml")
        for m in plan.missions
    ]
    for src in paths:
        real = os.path.realpath(src)
        if not real.startswith(root + os.sep) or not os.path.lexists(src):
            continue  # gone already, or outside the store: never touched
        rel = os.path.relpath(real, root)
        target = os.path.join(dest, rel)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        os.replace(src, target)  # same filesystem: atomic per item
    return dest
