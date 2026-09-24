"""Occupancy grids as files: the nav2 map_server PGM + YAML convention.

Pure functions, numpy arrays. Used by the scan synthesiser (world in), the
survey save (map out) and the route editor (map in). Cell values follow
nav_msgs/OccupancyGrid: -1 unknown, 0 free, 100 occupied.

PGM rows run top-down; the grid's row 0 is the bottom (lowest y), so the
image is flipped on read and write. origin = world pose of the grid's
(0, 0) cell corner; only yaw-free origins are written or loaded here (read()
rejects a rotated origin: cell indexing everywhere assumes axis alignment).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
import yaml


class GridError(ValueError):
    """A map file that cannot be loaded as a supported, consistent occupancy grid."""


MAX_PIXELS = 64_000_000  # e.g. 400 m x 400 m at 0.05 m; bounds allocation before any array is built
YAW_TOL_RAD = 1e-9


@dataclass(frozen=True)
class GridMeta:
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196  # 205 (unknown) is 0.19608: not free, not occupied
    negate: int = 0


@dataclass
class Grid:
    data: np.ndarray  # int8 [rows, cols], row 0 = bottom
    meta: GridMeta

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        c = int(np.floor((x - self.meta.origin_x) / self.meta.resolution))
        r = int(np.floor((y - self.meta.origin_y) / self.meta.resolution))
        return r, c

    def cell_to_world(self, r: int, c: int) -> tuple[float, float]:
        """Centre of cell (r, c)."""
        return (
            self.meta.origin_x + (c + 0.5) * self.meta.resolution,
            self.meta.origin_y + (r + 0.5) * self.meta.resolution,
        )


def to_pgm_bytes(grid: Grid) -> bytes:
    """Binary PGM (P5). unknown -> 205, free -> 254, occupied -> 0 (map_server trinary)."""
    img = np.full(grid.data.shape, 205, dtype=np.uint8)
    img[grid.data == 0] = 254
    img[grid.data >= 65] = 0
    img = np.flipud(img)
    header = f"P5\n{grid.width} {grid.height}\n255\n".encode()
    return header + img.tobytes()


def require_axis_aligned(meta: GridMeta) -> None:
    """Cell indexing, footprint sweeps and raycasts assume a yaw-free origin. Until every
    consumer handles origin yaw, a rotated origin is refused rather than half-honoured."""
    if not math.isfinite(meta.origin_yaw) or abs(meta.origin_yaw) > YAW_TOL_RAD:
        raise GridError(f"map origin yaw {meta.origin_yaw!r} is not supported: origins must be yaw-free")


def _header_tokens(raw: bytes) -> tuple[list[bytes], int]:
    """The four P5 header tokens and the offset just past maxval. Always terminates."""
    tokens: list[bytes] = []
    pos, n = 0, len(raw)
    while len(tokens) < 4:
        while pos < n and raw[pos : pos + 1].isspace():
            pos += 1
        if pos >= n:
            raise GridError(f"PGM header truncated: EOF after {len(tokens)} of 4 fields")
        if raw[pos : pos + 1] == b"#":
            nl = raw.find(b"\n", pos)
            if nl < 0:
                raise GridError("PGM header truncated: comment without line end")
            pos = nl + 1
            continue
        end = pos
        while end < n and not raw[end : end + 1].isspace() and raw[end : end + 1] != b"#":
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    if pos >= n or not raw[pos : pos + 1].isspace():
        raise GridError("PGM header truncated: no whitespace after maxval")
    return tokens, pos + 1  # single whitespace after maxval


def _header_int(tok: bytes, what: str) -> int:
    if not tok.isdigit() or len(tok) > 9:
        raise GridError(f"PGM {what} is not a positive integer: {tok[:16]!r}")
    return int(tok)


def from_pgm_bytes(raw: bytes, meta: GridMeta) -> Grid:
    if raw[:2] != b"P5":
        raise GridError("only binary PGM (P5) is supported")
    tokens, pos = _header_tokens(raw)
    if tokens[0] != b"P5":
        raise GridError("only binary PGM (P5) is supported")
    w, h = _header_int(tokens[1], "width"), _header_int(tokens[2], "height")
    maxval = _header_int(tokens[3], "maxval")
    if w < 1 or h < 1 or w * h > MAX_PIXELS:
        raise GridError(f"PGM dimensions {w}x{h} out of range (1..{MAX_PIXELS} pixels)")
    if not 1 <= maxval <= 255:
        raise GridError(f"PGM maxval {maxval} unsupported (8-bit only, 1..255)")
    if len(raw) - pos < w * h:
        raise GridError(f"PGM payload short: {len(raw) - pos} bytes for {w}x{h}")
    img = np.frombuffer(raw[pos : pos + w * h], dtype=np.uint8).reshape(h, w)
    img = np.flipud(img)
    occ = (maxval - img.astype(np.float64)) / maxval if not meta.negate else img / maxval
    data = np.full(img.shape, -1, dtype=np.int8)
    data[occ >= meta.occupied_thresh] = 100
    data[occ <= meta.free_thresh] = 0
    return Grid(data, meta)


def write(grid: Grid, path_stem: str) -> tuple[str, str]:
    """Write <stem>.pgm and <stem>.yaml. Returns their paths."""
    pgm, yml = path_stem + ".pgm", path_stem + ".yaml"
    with open(pgm, "wb") as fh:
        fh.write(to_pgm_bytes(grid))
    doc = {
        "image": os.path.basename(pgm),
        "mode": "trinary",
        "resolution": float(grid.meta.resolution),
        "origin": [float(grid.meta.origin_x), float(grid.meta.origin_y), float(grid.meta.origin_yaw)],
        "negate": int(grid.meta.negate),
        "occupied_thresh": float(grid.meta.occupied_thresh),
        "free_thresh": float(grid.meta.free_thresh),
    }
    with open(yml, "w") as fh:
        yaml.safe_dump(doc, fh, sort_keys=False)
    return pgm, yml


def _num(key: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise GridError(f"map yaml {key} must be a finite number, got {value!r}")
    return float(value)


def read(yaml_path: str) -> Grid:
    with open(yaml_path) as fh:
        try:
            doc = yaml.safe_load(fh)
        except yaml.YAMLError as e:
            raise GridError(f"map yaml unparsable: {yaml_path}: {e}") from None
    if not isinstance(doc, dict):
        raise GridError(f"map yaml is not a mapping: {yaml_path}")
    try:
        origin = doc["origin"]
        if not isinstance(origin, list) or len(origin) not in (2, 3):
            raise GridError(f"map yaml origin must be [x, y] or [x, y, yaw], got {origin!r}")
        meta = GridMeta(
            resolution=_num("resolution", doc["resolution"]),
            origin_x=_num("origin", origin[0]),
            origin_y=_num("origin", origin[1]),
            origin_yaw=_num("origin", origin[2]) if len(origin) > 2 else 0.0,
            occupied_thresh=_num("occupied_thresh", doc.get("occupied_thresh", 0.65)),
            free_thresh=_num("free_thresh", doc.get("free_thresh", 0.196)),
            negate=int(doc.get("negate", 0)),
        )
        image = str(doc["image"])
    except KeyError as e:
        raise GridError(f"map yaml lacks {e.args[0]!r}: {yaml_path}") from None
    if meta.resolution <= 0.0:
        raise GridError(f"map resolution must be positive, got {meta.resolution}")
    if not 0.0 <= meta.free_thresh < meta.occupied_thresh <= 1.0:
        raise GridError("map thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1")
    require_axis_aligned(meta)
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    with open(image, "rb") as fh:
        return from_pgm_bytes(fh.read(), meta)


def rasterize_polygon(grid: Grid, poly: np.ndarray, mask: np.ndarray) -> None:
    """OR the cells whose centres lie inside `poly` (world coords, [n, 2]) into `mask`.
    Axis-aligned grids only (require_axis_aligned). Footprint sweeps and map edits share it."""
    poly = np.asarray(poly, dtype=np.float64)
    res, ox, oy = grid.meta.resolution, grid.meta.origin_x, grid.meta.origin_y
    c0 = max(0, int(math.floor((poly[:, 0].min() - ox) / res)))
    c1 = min(grid.width - 1, int(math.floor((poly[:, 0].max() - ox) / res)))
    r0 = max(0, int(math.floor((poly[:, 1].min() - oy) / res)))
    r1 = min(grid.height - 1, int(math.floor((poly[:, 1].max() - oy) / res)))
    if c1 < c0 or r1 < r0:
        return
    cols = np.arange(c0, c1 + 1)
    rows = np.arange(r0, r1 + 1)
    px = ox + (cols + 0.5) * res
    py = oy + (rows + 0.5) * res
    X, Y = np.meshgrid(px, py)  # [rows, cols]
    inside = np.zeros(X.shape, dtype=bool)
    n = len(poly)
    for i in range(n):  # even-odd crossing test, vectorised over the bbox
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        crosses = (y1 > Y) != (y2 > Y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = x1 + (Y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (X < xint)
    mask[r0 : r1 + 1, c0 : c1 + 1] |= inside


def mask_misalignment(grid: Grid, mask: Grid) -> str | None:
    """None if `mask` (a keepout or dynamic-area grid) indexes the same cells as the map, else
    what differs. A misaligned mask must never be indexed with the map's cell coordinates."""
    a, b = grid.meta, mask.meta
    if mask.data.shape != grid.data.shape:
        return f"shape {mask.data.shape} != map {grid.data.shape}"
    if abs(a.resolution - b.resolution) > 1e-9:
        return f"resolution {b.resolution} != map {a.resolution}"
    if abs(a.origin_x - b.origin_x) > 1e-6 or abs(a.origin_y - b.origin_y) > 1e-6:
        return f"origin ({b.origin_x}, {b.origin_y}) != map ({a.origin_x}, {a.origin_y})"
    if abs(a.origin_yaw - b.origin_yaw) > YAW_TOL_RAD:
        return f"origin yaw {b.origin_yaw} != map {a.origin_yaw}"
    return None


def from_occupancy_grid_msg(msg) -> Grid:
    """nav_msgs/OccupancyGrid -> Grid (row 0 = bottom, as in the message)."""
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    q = msg.info.origin.orientation
    yaw = float(np.arctan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z))
    meta = GridMeta(
        resolution=float(msg.info.resolution),
        origin_x=float(msg.info.origin.position.x),
        origin_y=float(msg.info.origin.position.y),
        origin_yaw=yaw,
    )
    return Grid(data.copy(), meta)


# ---------------------------------------------------------------------------
# Image (pixel) <-> world, for the route editor. The browser uses the SAME
# formulas (amr_web/static/editor.js); these are the reference, tested with
# non-zero origin, origin yaw, row inversion and non-default resolution.
# Pixel (u, v): u = column from the left, v = row from the TOP of the image,
# continuous (a pixel's centre is at +0.5). Origin yaw rotates the grid axes
# about the grid origin corner (map_server convention).
# ---------------------------------------------------------------------------


def world_to_pixel(meta: GridMeta, height: int, x: float, y: float) -> tuple[float, float]:
    dx, dy = x - meta.origin_x, y - meta.origin_y
    c, s = np.cos(meta.origin_yaw), np.sin(meta.origin_yaw)
    gx = c * dx + s * dy  # into grid axes
    gy = -s * dx + c * dy
    u = gx / meta.resolution
    v = height - gy / meta.resolution
    return float(u), float(v)


def pixel_to_world(meta: GridMeta, height: int, u: float, v: float) -> tuple[float, float]:
    gx = u * meta.resolution
    gy = (height - v) * meta.resolution
    c, s = np.cos(meta.origin_yaw), np.sin(meta.origin_yaw)
    return float(meta.origin_x + c * gx - s * gy), float(meta.origin_y + s * gx + c * gy)
