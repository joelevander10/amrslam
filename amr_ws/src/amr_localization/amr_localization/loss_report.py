"""Why localisation was lost: a report written the moment it happens (2026-11-09). Pure.

The localization monitor keeps the last WINDOW_S of samples (pose, speed, sensor ages, AMCL
covariance, how the scan fits the map) and the latest classified scan. When READY drops it
builds a report from them: what triggered it, the correction split into ahead / left of the
AMR, how fast it was going and turning, which sensor went quiet, how the scan fitted the map
just before (on mapped walls / on things that are not on the map / through mapped walls) and
plain-language hints. The node writes it to <state_dir>/loc_losses/<time>.json with the scan
points (map frame, by class) for the web page, and sends the summary as the event text.
"""

from __future__ import annotations

import collections
import json
import math
import os
import time
from dataclasses import asdict, dataclass

WINDOW_S = 10.0
KEEP_REPORTS = 50
MAX_POINTS = 400
CLASS_NAMES = {1: "map", 2: "unmapped", 3: "through", 4: "dynamic", 5: "other"}
STREAM_PARTS = {
    "scan": "the lidar (network to 192.168.3.x, the driver, or the scanner itself)",
    "wheels": "the wheel encoders (CAN bus / qr_base_node)",
    "imu": "the IMU (USB serial)",
    "tf": "the localisation itself (AMCL stopped publishing map->odom)",
}


@dataclass
class Sample:
    t: float
    pose: tuple[float, float, float] | None  # map frame
    ages: dict
    cov: tuple[float, float, float] | None
    scan_match: float | None
    scan_long: float | None
    unmapped: float | None
    # odom frame (wheels + IMU only): the speed and turn rate come from it, so a localisation
    # correction inside the window does not show up as speed (2026-11-09: "0.96 m/s")
    odom: tuple[float, float, float] | None = None


class Recorder:
    def __init__(self, window_s: float = WINDOW_S) -> None:
        self.window_s = window_s
        self.samples: collections.deque[Sample] = collections.deque()
        self.scan: dict | None = None  # {t, laser (x, y, yaw), x, y, cls} of the latest comparison

    def add(self, s: Sample) -> None:
        self.samples.append(s)
        while self.samples and s.t - self.samples[0].t > self.window_s:
            self.samples.popleft()

    def motion(self, t: float, span: float = 1.0) -> tuple[float, float, float]:
        """(speed m/s, turn rate rad/s, distance over the window) from the odom pose history
        (the map pose where no odom pose was recorded)."""
        use_odom = any(s.odom is not None for s in self.samples)
        poses = [(s.t, s.odom if use_odom else s.pose) for s in self.samples]
        poses = [p for p in poses if p[1] is not None]
        dist = sum(math.hypot(b[1][0] - a[1][0], b[1][1] - a[1][1]) for a, b in zip(poses, poses[1:]))  # noqa: B905
        recent = [p for p in poses if t - p[0] <= span]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < 0.2:
            return 0.0, 0.0, dist
        (t0, a), (t1, b) = recent[0], recent[-1]
        dt = t1 - t0
        dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return math.hypot(b[0] - a[0], b[1] - a[1]) / dt, dyaw / dt, dist


def _fmt_pct(v: float | None) -> str:
    return "–" if v is None else f"{100 * v:.0f} %"


def build(rec: Recorder, t: float, code: str, reason: str, jump=None, limits=None) -> dict:
    """The report for a loss at time t (code / reason: readiness's)."""
    v, w, dist = rec.motion(t)
    last = rec.samples[-1] if rec.samples else None
    before = [s for s in rec.samples if t - s.t <= 3.0]
    match_vals = [s.scan_match for s in before if s.scan_match is not None]
    unm_vals = [s.unmapped for s in before if s.unmapped is not None]
    long_vals = [s.scan_long for s in before if s.scan_long is not None]
    rep: dict = {
        "time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
        "code": code,
        "reason": reason,
        "pose": list(last.pose) if last and last.pose else None,
        "speed_mps": round(v, 3),
        "turn_rad_s": round(w, 3),
        "travelled_m": round(dist, 2),
        "ages_s": {k: (None if x is None else round(x, 3)) for k, x in (last.ages if last else {}).items()},
        "cov": list(last.cov) if last and last.cov else None,
        "scan": {
            "on_map": min(match_vals) if match_vals else None,
            "unmapped": max(unm_vals) if unm_vals else None,
            "through": max(long_vals) if long_vals else None,
        },
        "jump": None,
        "hints": [],
    }
    if jump is not None:
        c, s = math.cos(jump.robot_yaw), math.sin(jump.robot_yaw)
        rep["jump"] = {
            "dist_m": round(jump.dist_m, 3),
            "ahead_m": round(jump.dx_m * c + jump.dy_m * s, 3),
            "left_m": round(-jump.dx_m * s + jump.dy_m * c, 3),
            "turn_deg": round(math.degrees(jump.signed_angle_rad), 2),
        }
    rep["hints"] = hints(rep, limits)
    if rec.scan is not None and t - rec.scan["t"] <= 3.0:
        pts = {name: [] for name in CLASS_NAMES.values()}
        n = len(rec.scan["cls"])
        step = max(1, n // MAX_POINTS)
        for i in range(0, n, step):
            name = CLASS_NAMES.get(int(rec.scan["cls"][i]))
            if name:
                pts[name].append([round(float(rec.scan["x"][i]), 3), round(float(rec.scan["y"][i]), 3)])
        rep["scan_points"] = {
            "laser": [round(float(x), 4) for x in rec.scan["laser"]],
            "age_s": round(t - rec.scan["t"], 2),
            **pts,
        }
    return rep


def hints(rep: dict, limits=None) -> list[str]:
    """Plain-language causes, most likely first."""
    out: list[str] = []
    ages = rep.get("ages_s") or {}
    lim = getattr(limits, "age_limits", None) or {"scan": 0.15, "wheels": 0.10, "imu": 0.20, "tf": 0.20}
    for name, part in STREAM_PARTS.items():
        a = ages.get(name)
        if a is None or a > lim.get(name, 1.0):
            out.append(f"{name} went quiet ({'never seen' if a is None else f'{a:.2f} s old'}): check {part}")
    sc = rep["scan"]
    unm, on_map, thr = sc.get("unmapped"), sc.get("on_map"), sc.get("through")
    if unm is not None and unm >= 0.25:
        out.append(
            f"{100 * unm:.0f} % of the scan hit things that are not on the map (trolleys, pallets, people, "
            "a new rack): mark that area dynamic (Edit areas) or map it again as it is now"
        )
    if thr is not None and thr >= 0.10:
        out.append(
            f"{100 * thr:.0f} % of beams went THROUGH walls on the map: something mapped is gone "
            "(a door open, a rack moved) or the pose was already off"
        )
    if on_map is not None and on_map < 0.5 and (unm is None or unm < 0.25):
        out.append(
            f"only {100 * on_map:.0f} % of the scan lay on mapped walls, without many unmapped objects: "
            "the map does not fit here - twisted while mapping (lidar angle) or the place changed"
        )
    j = rep.get("jump")
    if rep.get("code") == "LOC_CORRECTED":
        out.append(
            "the scan fitted the map before and after this correction, so it was AMCL fixing the wheel "
            "odometry and the AMR kept going. Only if it repeats along the path in the same direction: "
            "check the wheel diameter / gear ratio (wheel scale)"
        )
    if j:
        ah, le = abs(j["ahead_m"]), abs(j["left_m"])
        if le > 2 * ah and abs(rep["turn_rad_s"]) < 0.1:
            out.append(
                f"the correction was sideways ({j['left_m']:+.2f} m) while driving straight: a lidar "
                "not facing straight ahead (tools/lidar_yaw.py) or a twisted map"
            )
        elif ah > 2 * le:
            out.append(
                f"the correction was along the path ({j['ahead_m']:+.2f} m): a long aisle with little "
                "across it to measure distance by, or wheel slip"
            )
        if abs(rep["turn_rad_s"]) >= 0.15:
            out.append(
                f"it happened while turning ({math.degrees(rep['turn_rad_s']):+.0f} deg/s): lidar position "
                "(laser_x / laser_y) or turning where few walls are in view"
            )
    cov = rep.get("cov")
    if cov and (cov[0] > 0.05 or cov[1] > 0.05):
        out.append(
            f"AMCL was unsure (spread {math.sqrt(max(cov[0], cov[1])):.2f} m): few fixed walls in view here"
        )
    if rep["speed_mps"] >= 0.6:
        out.append(f"at {rep['speed_mps']:.2f} m/s: if it only happens fast, try the same place slower")
    if not out:
        out.append(
            "no single cause stands out: look at the scan points on the map (Alarms page, Show on map)"
        )
    return out


def summary(rep: dict, name: str) -> str:
    """One line for the event text."""
    parts = [rep["reason"]]
    j = rep.get("jump")
    if j:
        parts.append(f"ahead {j['ahead_m']:+.2f} m, left {j['left_m']:+.2f} m, turn {j['turn_deg']:+.1f} deg")
    parts.append(f"at {rep['speed_mps']:.2f} m/s, {math.degrees(rep['turn_rad_s']):+.0f} deg/s")
    sc = rep["scan"]
    parts.append(
        f"scan: {_fmt_pct(sc['on_map'])} on the map, {_fmt_pct(sc['unmapped'])} on unmapped things, "
        f"{_fmt_pct(sc['through'])} through walls"
    )
    return "; ".join(parts) + f". Likely: {rep['hints'][0]} (report {name})"


def write(state_dir: str, rep: dict, keep: int = KEEP_REPORTS) -> str:
    """<state_dir>/loc_losses/<time>.json (the oldest beyond `keep` removed). Returns the name."""
    d = os.path.join(state_dir, "loc_losses")
    os.makedirs(d, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    # the counter keeps names in time order, also within one second (never reused after a prune)
    used = [
        f[len(stamp) + 1 : -5] for f in os.listdir(d) if f.startswith(stamp + "-") and f.endswith(".json")
    ]
    k = 1 + max((int(u) for u in used if u.isdigit()), default=0)
    path = os.path.join(d, f"{stamp}-{k:02d}.json")
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(rep, fh, indent=1)
    os.replace(tmp, path)
    old = sorted(f for f in os.listdir(d) if f.endswith(".json"))
    for f in old[:-keep] if len(old) > keep else []:
        try:
            os.remove(os.path.join(d, f))
        except OSError:
            pass
    return os.path.basename(path)[:-5]


def list_reports(state_dir: str) -> list[dict]:
    """Newest first: name, time, code, reason, hints (no scan points)."""
    d = os.path.join(state_dir, "loc_losses")
    if not os.path.isdir(d):
        return []
    out = []
    for f in sorted((f for f in os.listdir(d) if f.endswith(".json")), reverse=True):
        try:
            with open(os.path.join(d, f)) as fh:
                rep = json.load(fh)
        except (OSError, ValueError):
            continue
        out.append(
            {k: rep.get(k) for k in ("time", "code", "reason", "pose", "speed_mps", "jump", "scan", "hints")}
            | {"name": f[:-5]}
        )
    return out


def load(state_dir: str, name: str) -> dict | None:
    if not name or "/" in name or name.startswith("."):
        return None
    try:
        with open(os.path.join(state_dir, "loc_losses", f"{name}.json")) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


__all__ = ["Recorder", "Sample", "asdict", "build", "hints", "list_reports", "load", "summary", "write"]
