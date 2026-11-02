"""Tape graph files (tracked plan 3, section 2.2). Filesystem only; no ROS.

    <maps_dir>/<map_id>/tapegraph/g<N>/graph.json      a published graph, never rewritten
                                       manifest.json   sha256 of graph.json, binding, summary
    <maps_dir>/<map_id>/tapegraph/.lock               writer lock (flock)
    <state_dir>/teach/<map_id>-rev<R>/passes/*.json    recorded passes (the teach workspace)
                                      stations.json    taught station poses
    <state_dir>/line_graph.json                        the ACTIVE pointer: map, revision, graph

Published like a map revision (amr_mission.map_bundle): staged in a unique directory,
fsynced, renamed under the lock, refused if g<N> exists. The teach workspace is the
mutable part: passes and stations are added and deleted there, and a graph is BUILT from
it and published as the next g<N>. The pointer is read once when the LINE layer starts.
"""

import contextlib
import fcntl
import hashlib
import json
import os
import re
import tempfile

from amr_line import tape_graph as tg

GRAPH_DIR = "tapegraph"
REV_RE = re.compile(r"^g(\d+)$")
POINTER = "line_graph.json"


class StoreError(RuntimeError):
    pass


def _plain(name, what):
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_.-]+", name) or name.startswith("."):
        raise StoreError(f"not a {what}: {name!r}")
    return name


def graph_root(maps_dir, map_id):
    return os.path.join(maps_dir, _plain(map_id, "map id"), GRAPH_DIR)


def list_revisions(maps_dir, map_id):
    try:
        names = os.listdir(graph_root(maps_dir, map_id))
    except OSError:
        return []
    return sorted(int(m.group(1)) for m in map(REV_RE.match, names) if m)


@contextlib.contextmanager
def _lock(root):
    os.makedirs(root, exist_ok=True)
    fd = os.open(os.path.join(root, ".lock"), os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        yield
    finally:
        os.close(fd)


def _fsync(path, directory=False):
    fd = os.open(path, os.O_RDONLY | (os.O_DIRECTORY if directory else 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _sha(path):
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def publish(maps_dir, graph, summary=""):
    """Write `graph` as the next g<N> of its map. Returns N."""
    map_id = graph.binding["map_id"]
    root = graph_root(maps_dir, map_id)
    with _lock(root):
        revs = list_revisions(maps_dir, map_id)
        n = (revs[-1] + 1) if revs else 1
        graph.revision = n
        dest = os.path.join(root, f"g{n}")
        if os.path.lexists(dest):
            raise StoreError(f"graph revision already exists: {dest}")
        stage = tempfile.mkdtemp(prefix=f".staging-g{n}-", dir=root)
        try:
            gpath = os.path.join(stage, "graph.json")
            with open(gpath, "w") as fh:
                json.dump(graph.to_dict(), fh, indent=1)
            manifest = {"graph_revision": n, "sha256": _sha(gpath), **graph.binding,
                        "mission": graph.mission, "legs": sorted(graph.legs),
                        "stations": sorted(graph.stations), "summary": summary}
            with open(os.path.join(stage, "manifest.json"), "w") as fh:
                json.dump(manifest, fh, indent=1)
            for name in os.listdir(stage):
                _fsync(os.path.join(stage, name))
            _fsync(stage, directory=True)
            os.rename(stage, dest)
            _fsync(root, directory=True)
        except BaseException:
            if os.path.isdir(stage):
                for name in os.listdir(stage):
                    os.unlink(os.path.join(stage, name))
                os.rmdir(stage)
            raise
    return n


def load(maps_dir, map_id, revision):
    """g<N> -> Graph, its file hash verified against the manifest."""
    d = os.path.join(graph_root(maps_dir, map_id), f"g{int(revision)}")
    try:
        with open(os.path.join(d, "manifest.json")) as fh:
            manifest = json.load(fh)
        gpath = os.path.join(d, "graph.json")
        sha = _sha(gpath)
        with open(gpath) as fh:
            doc = json.load(fh)
    except (OSError, ValueError) as e:
        raise StoreError(f"graph {map_id} g{revision}: {e}") from None
    if sha != manifest.get("sha256"):
        raise StoreError(f"graph {map_id} g{revision}: graph.json does not match its manifest hash")
    g = tg.Graph.from_dict(doc)
    if g.revision != int(revision) or g.binding["map_id"] != map_id:
        raise StoreError(f"graph {map_id} g{revision}: the file says {g.binding['map_id']} g{g.revision}")
    return g, sha


# -- the active pointer --------------------------------------------------------------

def _atomic_json(path, doc):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".tmp-", dir=os.path.dirname(path))
    with os.fdopen(fd, "w") as fh:
        json.dump(doc, fh, indent=1)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def read_active(state_dir):
    """{"map_id", "map_revision", "graph_revision" (int | None), "sha256"} or None."""
    try:
        with open(os.path.join(state_dir, POINTER)) as fh:
            doc = json.load(fh)
    except (OSError, ValueError):
        return None
    if not isinstance(doc, dict) or not {"map_id", "map_revision", "graph_revision"} <= set(doc):
        return None
    return doc


def write_active(state_dir, map_id, map_revision, graph_revision=None, sha256=""):
    """Select the map (and graph) the LINE layer localises on at its next start. A map
    with no graph yet (graph_revision None) is the teaching state."""
    _atomic_json(os.path.join(state_dir, POINTER),
                 {"map_id": _plain(map_id, "map id"), "map_revision": int(map_revision),
                  "graph_revision": None if graph_revision is None else int(graph_revision),
                  "sha256": sha256})


# -- the teach workspace -------------------------------------------------------------

def teach_dir(state_dir, map_id, map_revision):
    return os.path.join(state_dir, "teach", f"{_plain(map_id, 'map id')}-rev{int(map_revision)}")


def save_pass(state_dir, map_id, map_revision, rec):
    d = os.path.join(teach_dir(state_dir, map_id, map_revision), "passes")
    name = f"{_plain(rec['circuit'], 'circuit')}-{rec['started'].replace(':', '')}.json"
    _atomic_json(os.path.join(d, _plain(name, "pass name")), rec)
    return name[:-5]


def list_passes(state_dir, map_id, map_revision):
    d = os.path.join(teach_dir(state_dir, map_id, map_revision), "passes")
    try:
        return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".json"))
    except OSError:
        return []


def load_passes(state_dir, map_id, map_revision):
    d = os.path.join(teach_dir(state_dir, map_id, map_revision), "passes")
    out = []
    for name in list_passes(state_dir, map_id, map_revision):
        with open(os.path.join(d, f"{name}.json")) as fh:
            out.append(json.load(fh))
    return out


def delete_pass(state_dir, map_id, map_revision, name):
    path = os.path.join(teach_dir(state_dir, map_id, map_revision), "passes", f"{_plain(name, 'pass')}.json")
    try:
        os.unlink(path)
        return True
    except FileNotFoundError:
        return False


def load_stations(state_dir, map_id, map_revision):
    try:
        with open(os.path.join(teach_dir(state_dir, map_id, map_revision), "stations.json")) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def save_station(state_dir, map_id, map_revision, record):
    """record: {"id", "pose": [x, y, yaw], "taught", "samples"}; replaces that station."""
    st = load_stations(state_dir, map_id, map_revision)
    st[record["id"]] = record
    _atomic_json(os.path.join(teach_dir(state_dir, map_id, map_revision), "stations.json"), st)
