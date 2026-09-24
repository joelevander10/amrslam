"""Route and mission revisions on disk, next to the map bundles.

    <maps_dir>/<map_id>/routes/<route_id>/rev<N>.yaml     immutable once written
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
    return bool(name) and "/" not in name and "\0" not in name and not name.startswith(".")


def routes_dir(maps_dir: str, map_id: str, route_id: str | None = None) -> str:
    d = os.path.join(maps_dir, map_id, "routes")
    return os.path.join(d, route_id) if route_id else d


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
    return os.path.join(routes_dir(maps_dir, map_id, route_id), f"rev{revision}.yaml")


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
    path = route_path(maps_dir, map_id, route_id, revision)
    if not os.path.isfile(path):
        raise StoreError(f"no such route revision: {path}")
    with open(path) as fh:
        return Route.loads(fh.read()), sha256_file(path)


# ---- missions --------------------------------------------------------------


def missions_dir(maps_dir: str) -> str:
    return os.path.join(maps_dir, "missions")


def list_missions(maps_dir: str) -> list[dict]:
    d = missions_dir(maps_dir)
    if not os.path.isdir(d):
        return []
    out = []
    for f in sorted(os.listdir(d)):
        if f.endswith(".yaml"):
            with open(os.path.join(d, f)) as fh:
                out.append(yaml.safe_load(fh))
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
) -> str:
    if not _plain(mission_id):
        raise StoreError("mission_id must be a plain name")
    path = os.path.join(missions_dir(maps_dir), f"{mission_id}.yaml")
    refs = {
        "map": {"id": map_id, "revision": map_revision, "sha256": map_sha256},
        "route": {"id": route_id, "revision": route_revision, "sha256": route_sha256},
    }
    doc = {"mission_id": mission_id, "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"), **refs}
    with _locked(missions_dir(maps_dir)):
        if os.path.lexists(path):
            try:
                with open(path) as fh:
                    old = yaml.safe_load(fh)
            except (OSError, yaml.YAMLError):
                old = None
            if isinstance(old, dict) and all(old.get(k) == v for k, v in refs.items()):
                return path  # idempotent retry: same references, original bytes untouched
            raise StoreConflict(f"mission {mission_id!r} exists with different references")
        _commit_new(path, yaml.safe_dump(doc, sort_keys=False).encode())
    return path


def load_mission(maps_dir: str, mission_id: str) -> dict:
    path = os.path.join(missions_dir(maps_dir), f"{mission_id}.yaml")
    if not os.path.isfile(path):
        raise StoreError(f"no such mission: {mission_id}")
    with open(path) as fh:
        return yaml.safe_load(fh)
