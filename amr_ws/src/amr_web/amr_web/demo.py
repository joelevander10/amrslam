"""The DEMO page's live panel: a STATIONARY vehicle on display, explained to visitors.

The display unit never moves, so nothing about driving is shown. What a visitor can
see live is the safety laser scanner: the page draws what it sees around the
vehicle's outline, and says so when someone walks up.

The audience is purchasing, plant management and people shopping for material
handling - not engineers. So the whole vocabulary is the three sentences below,
decided HERE, on the server. The browser never receives a fault code, a reason
string, a hold cause or a hex number, and a test scans the output for them.

*** Anything missing reads as "standing by", on purpose. *** This page diagnoses
nothing. The operator's Home page still tells the truth.

Pure: no Flask, no rclpy. `view()` takes `adapter.live.pose_scan()` and the
footprint polygon.
"""

from __future__ import annotations

import math

WATCHING, NEAR, STANDBY = "watching", "near", "standby"

SENTENCES = {
    WATCHING: "Watching all around. Walk closer and see yourself appear.",
    NEAR: "It sees you. On the move, it would stop and wait for you.",
    STANDBY: "Standing by for the next demonstration",
}

# Older than this, a scan or pose is not evidence of anything.
FRESH_S = 2.0
# The radar shows this far from the vehicle centre; farther points are not sent.
VIEW_M = 4.0
# Default "someone is close" distance from the vehicle outline; content.json near_m overrides.
NEAR_M = 1.0


def _fresh(d: dict | None) -> dict | None:
    if not d:
        return None
    age = d.get("age_s")
    if age is None or age > FRESH_S:
        return None
    return d


def _seg_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _inside(px: float, py: float, poly) -> bool:
    n, hit = len(poly), False
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        if (ay > py) != (by > py) and px < ax + (py - ay) * (bx - ax) / (by - ay):
            hit = not hit
    return hit


def outline_distance(px: float, py: float, poly) -> float:
    """Distance from a body-frame point to the vehicle outline; 0 inside it."""
    if _inside(px, py, poly):
        return 0.0
    n = len(poly)
    return min(_seg_dist(px, py, *poly[i], *poly[(i + 1) % n]) for i in range(n))


def body_points(pose_scan: dict | None) -> list[tuple[float, float]] | None:
    """Scan points in the vehicle's own frame (x forward, y left), or None when the
    scan or the pose is missing, stale, or in a different frame."""
    if not pose_scan:
        return None
    pose, scan = _fresh(pose_scan.get("pose")), _fresh(pose_scan.get("scan"))
    if not pose or not scan or pose.get("frame") != scan.get("frame"):
        return None
    try:
        x0, y0, yaw = float(pose["x"]), float(pose["y"]), float(pose["yaw"])
    except (KeyError, TypeError, ValueError):
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for p in scan.get("points") or []:
        try:
            dx, dy = float(p[0]) - x0, float(p[1]) - y0
        except (IndexError, TypeError, ValueError):
            continue
        bx, by = c * dx + s * dy, -s * dx + c * dy
        if math.hypot(bx, by) <= VIEW_M:
            out.append((round(bx, 2), round(by, 2)))
    return out


def view(pose_scan: dict | None, outline, near_m: float = NEAR_M) -> dict:
    """The /api/demo body."""
    pts = body_points(pose_scan)
    if pts is None:
        return {
            "status": STANDBY,
            "sentence": SENTENCES[STANDBY],
            "points": [],
            "near_points": [],
            "nearest_m": None,
        }
    # a return from inside the outline is the vehicle's own body: not shown, not counted
    far, near, nearest = [], [], None
    for x, y in pts:
        if _inside(x, y, outline):
            continue
        d = outline_distance(x, y, outline)
        nearest = d if nearest is None else min(nearest, d)
        (near if d <= near_m else far).append((x, y))
    return {
        "status": NEAR if near else WATCHING,
        "sentence": SENTENCES[NEAR if near else WATCHING],
        "points": far,
        "near_points": near,
        "nearest_m": None if nearest is None else round(nearest, 1),
    }
