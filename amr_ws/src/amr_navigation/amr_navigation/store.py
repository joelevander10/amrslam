"""Route and mission revisions on disk, next to the map bundles.

    <maps_dir>/<map_id>/routes/<route_id>/rev<N>.yaml     immutable once written
    <maps_dir>/<map_id>/labels/rev<N>.yaml                 floor-label poses, immutable
    <maps_dir>/missions/<mission_id>.yaml                  one validated route revision

Revisions and missions are immutable once published. Allocation and publication run
under an exclusive flock (threads and processes alike); bytes go to a unique temp file,
are fsynced, then hard-linked into place, which fails rather than replace an existing
name. The returned sha256 is of exactly the committed bytes.
"""

from __future__ import annotations

import contextlib
import fcntl
import hashlib
import math
import os
import re
import tempfile
import time

import yaml

from amr_navigation.route import Route

REV_RE = re.compile(r"^rev(\d+)\.yaml$")


class StoreError(RuntimeError):
    pass


class StoreConflict(StoreError):
    """The name is taken by different content: nothing was written."""


LOCK_NAME = ".lock"
TMP_PREFIX = ".tmp-"


@contextlib.contextmanager
def _locked(d: str):
    """Exclusive lock on directory `d`. flock binds to the open file description, so two
    threads of one process exclude each other as well as two processes do."""
    os.makedirs(d, exist_ok=True)
    fd = os.open(os.path.join(d, LOCK_NAME), os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        # temp files are only created under this lock, so any left now were abandoned
        for name in os.listdir(d):
            if name.startswith(TMP_PREFIX):
                with contextlib.suppress(FileNotFoundError):
                    os.unlink(os.path.join(d, name))
        yield
    finally:
        os.close(fd)  # releases the lock


def _fsync_dir(d: str) -> None:
    fd = os.open(d, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _commit_new(dest: str, data: bytes) -> str:
    """Publish `data` at `dest` only if `dest` does not exist. Returns sha256(data)."""
    d = os.path.dirname(dest)
    fd, tmp = tempfile.mkstemp(prefix=TMP_PREFIX, dir=d)
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(data)
            fh.flush()
            os.fsync(fh.fileno())
        try:
            os.link(tmp, dest)  # no-replace commit
        except FileExistsError:
            raise StoreConflict(f"already exists: {dest}") from None
    finally:
        with contextlib.suppress(FileNotFoundError):
            os.unlink(tmp)
    _fsync_dir(d)
    return hashlib.sha256(data).hexdigest()


def _plain(name: str) -> bool:
    return (
        isinstance(name, str)
        and bool(name)
        and "/" not in name
        and "\\" not in name
        and "\0" not in name
        and not name.startswith(".")
    )


def _name(kind: str, name: str) -> str:
    """Every identifier that becomes a path component, on READS as well as
    writes: `load_mission(maps, "../../outside")` used to open that file
    (review 2026-09-21, F10)."""
    if not _plain(name):
        raise StoreError(f"{kind} must be a plain name: {name!r}")
    return name


def _revision(revision: int) -> int:
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 1:
        raise StoreError(f"revision must be a positive integer: {revision!r}")
    return revision


def _inside(maps_dir: str, path: str) -> str:
    """The path, unless it (or a symlink on it) resolves outside the store."""
    root = os.path.realpath(maps_dir)
    real = os.path.realpath(path)
    if real != root and not real.startswith(root + os.sep):
        raise StoreError("path escapes the map store")
    return path


def _doc(path: str, what: str) -> dict:
    """A YAML mapping, or a StoreError: corrupt or scalar YAML is a bounded refusal."""
    try:
        with open(path) as fh:
            doc = yaml.safe_load(fh)
    except (OSError, yaml.YAMLError) as e:
        raise StoreError(f"{what}: {e}") from e
    if not isinstance(doc, dict):
        raise StoreError(f"{what}: not a mapping")
    return doc


def routes_dir(maps_dir: str, map_id: str, route_id: str | None = None) -> str:
    d = os.path.join(maps_dir, _name("map id", map_id), "routes")
    return os.path.join(d, _name("route_id", route_id)) if route_id else d


def list_routes(maps_dir: str, map_id: str) -> dict[str, list[int]]:
    d = routes_dir(maps_dir, map_id)
    out: dict[str, list[int]] = {}
    if not os.path.isdir(d):
        return out
    for rid in sorted(os.listdir(d)):
        if not os.path.isdir(os.path.join(d, rid)):
            continue
        revs = sorted(int(m.group(1)) for f in os.listdir(os.path.join(d, rid)) if (m := REV_RE.match(f)))
        if revs:
            out[rid] = revs
    return out


def route_path(maps_dir: str, map_id: str, route_id: str, revision: int) -> str:
    return os.path.join(routes_dir(maps_dir, map_id, route_id), f"rev{_revision(revision)}.yaml")


def sha256_file(path: str) -> str:
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def save_route(maps_dir: str, route: Route) -> tuple[int, str, str]:
    """Assign the next revision and publish it. Returns (revision, path, sha256)."""
    if not (_plain(route.map.id) and _plain(route.route_id)):
        raise StoreError("map id and route_id must be plain names")
    d = routes_dir(maps_dir, route.map.id, route.route_id)
    with _locked(d):
        revs = list_routes(maps_dir, route.map.id).get(route.route_id, [])
        revision = (revs[-1] + 1) if revs else 1
        dest = route_path(maps_dir, route.map.id, route.route_id, revision)
        if os.path.exists(dest):
            raise StoreError(f"route revision exists: {dest}")
        route.revision = revision
        sha = _commit_new(dest, route.dumps().encode())
    return revision, dest, sha


def load_route(maps_dir: str, map_id: str, route_id: str, revision: int) -> tuple[Route, str]:
    path = _inside(maps_dir, route_path(maps_dir, map_id, route_id, revision))
    if not os.path.isfile(path):
        raise StoreError(f"no such route revision: {path}")
    try:
        with open(path) as fh:
            route = Route.loads(fh.read())
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as e:
        raise StoreError(f"route {map_id}/{route_id} rev{revision}: {e}") from e
    # the file's identity must be the one asked for, not just a well-formed route
    if route.map.id != map_id or route.route_id != route_id or int(route.revision) != revision:
        raise StoreError(
            f"route file identity {route.map.id}/{route.route_id} rev{route.revision} "
            f"is not {map_id}/{route_id} rev{revision}"
        )
    return route, sha256_file(path)


# ---- floor-label sets (gls-wait-coding-plan §C.1) ----------------------------
#
# A label set belongs to ONE map revision (its `map` ref) but is revisioned on its own,
# next to the routes: recording a label is not a new map revision, so it invalidates no
# route. Missions bind one set by revision + sha256.

LABEL_ID_RE = re.compile(r"^x\d{4}y\d{4}$")
LABEL_ROLES = ("home", "dest")
LABEL_METHODS = ("survey", "amcl")
LABEL_FLOATS = ("x_m", "y_m", "yaw_rad", "spread_xy_m", "spread_yaw_rad")


def labels_dir(maps_dir: str, map_id: str) -> str:
    return os.path.join(maps_dir, _name("map id", map_id), "labels")


def labels_path(maps_dir: str, map_id: str, revision: int) -> str:
    return os.path.join(labels_dir(maps_dir, map_id), f"rev{_revision(revision)}.yaml")


def list_labels(maps_dir: str, map_id: str) -> list[int]:
    """The label-set revisions of a map id (every map revision), ascending."""
    d = labels_dir(maps_dir, map_id)
    if not os.path.isdir(d):
        return []
    return sorted(int(m.group(1)) for f in os.listdir(d) if (m := REV_RE.match(f)))


def check_labels(labels) -> str | None:
    """Why a list of label entries is not a valid set, or None. Not empty; at most one
    home (a map surveyed without a HOME label has none, owner 2026-11-02); IDs and names
    unique; IDs like x0100y0000; every pose finite."""
    if not isinstance(labels, list):
        return "labels must be a list"
    if not labels:
        return "a label set needs at least one label"
    ids, names, homes = set(), set(), 0
    for i, e in enumerate(labels):
        if not isinstance(e, dict):
            return f"label {i}: not a mapping"
        lid, name = e.get("label_id"), e.get("name")
        if not (isinstance(lid, str) and LABEL_ID_RE.match(lid)):
            return f"label {i}: label_id must look like x0100y0000, got {lid!r}"
        if not (isinstance(name, str) and name.strip()):
            return f"label {lid}: name must not be empty"
        if e.get("role") not in LABEL_ROLES:
            return f"label {lid}: role must be one of {LABEL_ROLES}"
        if e.get("method") not in LABEL_METHODS:
            return f"label {lid}: method must be one of {LABEL_METHODS}"
        for k in LABEL_FLOATS:
            v = e.get(k)
            if not (isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)):
                return f"label {lid}: {k} must be a finite number"
        n = e.get("n_frames")
        if not (isinstance(n, int) and not isinstance(n, bool) and n >= 1):
            return f"label {lid}: n_frames must be a positive integer"
        if lid in ids:
            return f"label {lid} appears twice"
        if name in names:
            return f"label name {name!r} appears twice"
        ids.add(lid)
        names.add(name)
        homes += e["role"] == "home"
    if homes > 1:
        return f"a label set has at most one home label, has {homes}"
    return None


def label_entry(
    label_id: str,
    role: str,
    name: str,
    pose: tuple[float, float, float],
    method: str,
    n_frames: int,
    spread_xy_m: float,
    spread_yaw_rad: float,
    note: str = "",
) -> dict:
    return {
        "label_id": label_id,
        "role": role,
        "name": name,
        "x_m": float(pose[0]),
        "y_m": float(pose[1]),
        "yaw_rad": float(pose[2]),
        "method": method,
        "recorded": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "n_frames": int(n_frames),
        "spread_xy_m": float(spread_xy_m),
        "spread_yaw_rad": float(spread_yaw_rad),
        "note": note,
    }


def save_labels(
    maps_dir: str, map_id: str, map_revision: int, map_sha256: str, labels: list[dict]
) -> tuple[int, str, str]:
    """Publish the next label-set revision for map `map_id` rev `map_revision`.
    Returns (revision, path, sha256). The set must pass check_labels()."""
    _revision(map_revision)
    why = check_labels(labels)
    if why:
        raise StoreError(why)
    doc = {"map": {"id": map_id, "revision": map_revision, "sha256": map_sha256}, "labels": labels}
    d = labels_dir(maps_dir, map_id)
    with _locked(d):
        revs = list_labels(maps_dir, map_id)
        revision = (revs[-1] + 1) if revs else 1
        dest = labels_path(maps_dir, map_id, revision)
        sha = _commit_new(dest, yaml.safe_dump({"revision": revision, **doc}, sort_keys=False).encode())
    return revision, dest, sha


def load_labels(maps_dir: str, map_id: str, revision: int) -> tuple[dict, str]:
    """(doc, sha256) of one label-set revision, checked: identity, map ref and entries."""
    path = _inside(maps_dir, labels_path(maps_dir, map_id, revision))
    if not os.path.isfile(path):
        raise StoreError(f"no such label set: {map_id} labels rev{revision}")
    doc = _doc(path, f"label set {map_id} rev{revision}")
    ref = doc.get("map")
    if doc.get("revision") != revision or not isinstance(ref, dict) or ref.get("id") != map_id:
        raise StoreError(f"label file {path} is not {map_id} labels rev{revision}")
    if not isinstance(ref.get("revision"), int) or isinstance(ref.get("revision"), bool):
        raise StoreError(f"label set {map_id} rev{revision}: bad map revision")
    why = check_labels(doc.get("labels"))
    if why:
        raise StoreError(f"label set {map_id} rev{revision}: {why}")
    return doc, sha256_file(path)


def latest_labels(maps_dir: str, map_id: str, map_revision: int) -> tuple[dict, str] | None:
    """(doc, sha256) of the newest label set made for that map revision, or None.
    A corrupt newest file is a StoreError, never a silent fall-back to an older set."""
    for rev in reversed(list_labels(maps_dir, map_id)):
        path = labels_path(maps_dir, map_id, rev)
        ref = _doc(path, f"label set {map_id} rev{rev}").get("map")
        if isinstance(ref, dict) and ref.get("revision") == map_revision:
            return load_labels(maps_dir, map_id, rev)
    return None


def with_label(labels: list[dict], entry: dict) -> list[dict]:
    """A copy of the set with `entry` recorded: it replaces any entry with the same ID or
    name, and a new home replaces the old home."""
    out = [
        e
        for e in labels
        if e["label_id"] != entry["label_id"]
        and e["name"] != entry["name"]
        and not (entry["role"] == "home" and e["role"] == "home")
    ]
    return [*out, entry]


# ---- missions --------------------------------------------------------------


def missions_dir(maps_dir: str) -> str:
    return os.path.join(maps_dir, "missions")


def list_missions(maps_dir: str) -> list[dict]:
    d = missions_dir(maps_dir)
    if not os.path.isdir(d):
        return []
    out = []
    for f in sorted(os.listdir(d)):
        if f.endswith(".yaml") and _plain(f):
            with contextlib.suppress(StoreError):  # one corrupt file does not hide the rest
                out.append(_doc(os.path.join(d, f), f"mission {f}"))
    return out


def save_mission(
    maps_dir: str,
    mission_id: str,
    map_id: str,
    map_revision: int,
    map_sha256: str,
    route_id: str,
    route_revision: int,
    route_sha256: str,
    labels: dict | None = None,
) -> str:
    """`labels` = {"revision", "sha256"} binds a label set of the same map id; without one
    the mission gets no label fixes, only raw logging (gls-wait-coding-plan §C.1)."""
    if not _plain(mission_id):
        raise StoreError("mission_id must be a plain name")
    path = os.path.join(missions_dir(maps_dir), f"{mission_id}.yaml")
    refs = {
        "map": {"id": map_id, "revision": map_revision, "sha256": map_sha256},
        "route": {"id": route_id, "revision": route_revision, "sha256": route_sha256},
    }
    if labels is not None:
        refs["labels"] = {"revision": _revision(labels.get("revision")), "sha256": str(labels.get("sha256"))}
    doc = {"mission_id": mission_id, "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"), **refs}
    with _locked(missions_dir(maps_dir)):
        if os.path.lexists(path):
            try:
                with open(path) as fh:
                    old = yaml.safe_load(fh)
            except (OSError, yaml.YAMLError):
                old = None
            same = isinstance(old, dict) and all(old.get(k) == v for k, v in refs.items())
            if same and ("labels" in old) == ("labels" in refs):
                return path  # idempotent retry: same references, original bytes untouched
            raise StoreConflict(f"mission {mission_id!r} exists with different references")
        _commit_new(path, yaml.safe_dump(doc, sort_keys=False).encode())
    return path


def load_mission(maps_dir: str, mission_id: str) -> dict:
    path = _inside(maps_dir, os.path.join(missions_dir(maps_dir), f"{_name('mission_id', mission_id)}.yaml"))
    if not os.path.isfile(path):
        raise StoreError(f"no such mission: {mission_id}")
    doc = _doc(path, f"mission {mission_id}")
    if doc.get("mission_id") != mission_id:
        raise StoreError(f"mission file {mission_id!r} names {doc.get('mission_id')!r}")
    for key in ("map", "route"):
        ref = doc.get(key)
        if not isinstance(ref, dict) or not _plain(str(ref.get("id", ""))):
            raise StoreError(f"mission {mission_id}: bad {key} reference")
        if (
            not isinstance(ref.get("revision"), int)
            or isinstance(ref.get("revision"), bool)
            or ref["revision"] < 1
        ):
            raise StoreError(f"mission {mission_id}: {key} revision must be a positive integer")
    lab = doc.get("labels")
    if lab is not None and (
        not isinstance(lab, dict)
        or not isinstance(lab.get("revision"), int)
        or isinstance(lab.get("revision"), bool)
        or lab["revision"] < 1
        or not isinstance(lab.get("sha256"), str)
    ):
        raise StoreError(f"mission {mission_id}: bad labels reference")
    return doc


def mission_labels(maps_dir: str, mission: dict) -> dict | None:
    """The label set a loaded mission binds, checked against the mission's map and the
    bound hash; None when it binds none."""
    lab = mission.get("labels")
    if lab is None:
        return None
    m = mission["map"]
    doc, sha = load_labels(maps_dir, m["id"], int(lab["revision"]))
    if sha != lab["sha256"]:
        raise StoreError("label set hash differs from the mission manifest")
    ref = doc["map"]
    if ref.get("revision") != m["revision"] or ref.get("sha256") != m.get("sha256"):
        raise StoreError(
            f"label set rev{lab['revision']} is for map {ref.get('id')} rev{ref.get('revision')}, "
            f"mission is for rev{m['revision']}"
        )
    return doc
