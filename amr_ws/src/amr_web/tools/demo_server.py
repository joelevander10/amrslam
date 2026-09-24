"""Offline demo of the operator web app: every page, no ROS, no service, no hardware.

    python amr_ws/src/amr_web/tools/demo_server.py            # http://127.0.0.1:5051/
    python amr_ws/src/amr_web/tools/demo_server.py --port 5099 --maps-dir /tmp/demo_maps

Runs the REAL `amr_web.server.create_app` (routes, templates, static files, jog
sessions, route validation/storage) against `DemoAdapter`, a pure-Python stand-in
for `RosAdapter` that fabricates the same dict shapes the ROS callbacks produce.
A small kinematic sim moves a pose on the sim_factory world, so the jog pad, the
live survey view, localisation and route execution all animate.

A floating DEMO bar (injected into every HTML response, not into the templates)
switches scenarios and plays the physical panel: selector, Start, Reset, E-stop.

Nothing here is ever imported by the vehicle stack. Works on Windows and Linux;
the only third-party needs are flask, numpy and pyyaml.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import threading
import time
import types
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))  # amr_ws/src
REPO = os.path.abspath(os.path.join(SRC, "..", ".."))
for pkg in ("amr_web", "amr_maps", "amr_mission", "amr_navigation", "amr_base"):
    sys.path.insert(0, os.path.join(SRC, pkg))
os.environ.setdefault("AGV_CAN_ROOT", REPO)

# ---- portability shims (demo process only) -----------------------------------------
if sys.platform == "win32":
    fcntl = types.ModuleType("fcntl")
    fcntl.LOCK_EX, fcntl.LOCK_NB, fcntl.LOCK_UN = 2, 4, 8
    fcntl.flock = lambda *a: None
    sys.modules.setdefault("fcntl", fcntl)
_ament = types.ModuleType("ament_index_python")
_ament_pk = types.ModuleType("ament_index_python.packages")
_ament_pk.get_package_share_directory = lambda name: os.path.join(SRC, name)
_ament.packages = _ament_pk
sys.modules.setdefault("ament_index_python", _ament)
sys.modules.setdefault("ament_index_python.packages", _ament_pk)

import numpy as np  # noqa: E402
from amr_navigation.compiler import compile_route  # noqa: E402
from amr_web.png import encode_gray  # noqa: E402
from flask import request  # noqa: E402

from amr_maps import grid as gridio  # noqa: E402
from amr_mission import fixtures  # noqa: E402
from amr_mission import map_bundle as mb  # noqa: E402
from amr_navigation import store  # noqa: E402
from amr_web import server as web_server  # noqa: E402

if sys.platform == "win32":  # a directory cannot be opened for fsync on Windows
    store._fsync_dir = lambda d: None
    if hasattr(mb, "_fsync_dir"):
        mb._fsync_dir = lambda d: None

WORLD_YAML = os.path.join(SRC, "amr_maps", "worlds", "sim_factory", "world.yaml")
LASER_X = 0.964  # nanoScan3 ahead of the axle (amr_ws/README "Geometry")
FOV = math.radians(95.0)
MODE = {"STARTING": 0, "IDLE": 1, "MAPPING": 2, "NAVIGATION": 3, "TRANSITIONING": 4, "FAULT": 5}
MAP_STATES = {0: "IDLE", 1: "MAPPING", 2: "RETURN_REVIEW", 3: "SAVING", 4: "SAVED"}
LOC_STATES = {0: "UNLOCALIZED", 1: "CHECKING", 2: "READY", 3: "LOST"}
RUN_STATES = {0: "IDLE", 1: "READY", 2: "EXECUTING", 3: "PAUSED", 4: "BLOCKED", 5: "FAULT", 6: "DONE"}
DI_NAMES = [
    "spare",
    "start",
    "reset",
    "auto_manual",
    "pendant_fwd",
    "pendant_rvs",
    "pendant_left",
    "pendant_right",
] + [f"spare{i}" for i in range(8, 16)]
DO_NAMES = ["horn_lights"] + [f"spare{i}" for i in range(1, 16)]
SCENARIOS = [
    ("idle", "Idle"),
    ("mapping", "Mapping"),
    ("review", "Return review"),
    ("nav", "Nav: checking"),
    ("ready", "Nav: ready + mission"),
    ("executing", "Executing"),
    ("blocked", "Blocked"),
    ("runfault", "Run fault"),
    ("lost", "Loc lost"),
    ("layerfault", "Layer fault"),
    ("commissioning", "Commissioning"),
    ("offline", "Disconnected"),
]


def mode_dict(**kw) -> dict:
    d = {
        "header": {},
        "instance": "demo-instance",
        "generation": 1,
        "mode": 1,
        "requested_mode": 1,
        "phase": "",
        "operation_id": "",
        "base_ready": True,
        "layer_ready": False,
        "active_map_id": "",
        "active_map_revision": 0,
        "active_map_sha256": "",
        "manual_available": True,
        "autonomous_available": False,
        "fault_code": "",
        "reason": "",
        "last_survey_map_id": "",
        "last_survey_revision": 0,
    }
    d.update(kw)
    inv = {v: k for k, v in MODE.items()}
    d["mode_name"] = inv.get(d["mode"], str(d["mode"]))
    d["requested_name"] = inv.get(d["requested_mode"], str(d["requested_mode"]))
    return d


class Sim:
    """Kinematic unicycle on the world grid plus a ray-cast ±95° scan."""

    def __init__(self, world: gridio.Grid) -> None:
        self.world = world
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.v = self.w = 0.0
        self.revealed = np.zeros(world.data.shape, dtype=bool)
        self.reveal_at: tuple[float, float] | None = None
        self.snapshot = 1
        m = world.meta
        self._res, self._ox, self._oy = m.resolution, m.origin_x, m.origin_y
        self._angles = np.linspace(-FOV, FOV, 191)
        self._radii = np.arange(0.1, 18.0, self._res)

    def cell(self, x, y):
        c = np.floor((np.asarray(x) - self._ox) / self._res).astype(int)
        r = np.floor((np.asarray(y) - self._oy) / self._res).astype(int)
        return r, c

    def occupied(self, x, y) -> np.ndarray:
        r, c = self.cell(x, y)
        h, w = self.world.data.shape
        inside = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        out = np.ones(r.shape, dtype=bool)
        out[inside] = self.world.data[r[inside], c[inside]] >= 65
        return out

    def scan(self, max_points: int = 400) -> list[tuple[float, float]]:
        lx = self.x + LASER_X * math.cos(self.yaw)
        ly = self.y + LASER_X * math.sin(self.yaw)
        a = self.yaw + self._angles[:, None]
        px, py = lx + np.cos(a) * self._radii, ly + np.sin(a) * self._radii
        hit = self.occupied(px, py)
        idx = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        pts = [(round(float(px[i, j]), 3), round(float(py[i, j]), 3)) for i, j in enumerate(idx) if j >= 0]
        return pts[:max_points]

    def reveal(self, radius_m: float = 9.0) -> None:
        if self.reveal_at and math.hypot(self.x - self.reveal_at[0], self.y - self.reveal_at[1]) < 0.3:
            return
        self.reveal_at = (self.x, self.y)
        h, w = self.revealed.shape
        rr, cc = np.ogrid[:h, :w]
        r0, c0 = self.cell(self.x, self.y)
        rad = radius_m / self._res
        self.revealed |= (rr - r0) ** 2 + (cc - c0) ** 2 <= rad * rad
        self.snapshot += 1

    def step(self, dt: float) -> None:
        nyaw = self.yaw + self.w * dt
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        # refuse to drive the axle into a wall; the demo is not a physics engine
        if not self.occupied(np.array([nx]), np.array([ny]))[0]:
            self.x, self.y = nx, ny
        self.yaw = math.atan2(math.sin(nyaw), math.cos(nyaw))


class DemoLive:
    """Same surface as amr_web.live.LiveStore: map_meta / map_png / pose_scan."""

    def __init__(self, demo: DemoAdapter) -> None:
        self.d = demo

    def _layer(self) -> str:
        return self.d.mode["mode_name"]

    def map_meta(self) -> dict | None:
        if self._layer() not in ("MAPPING", "NAVIGATION"):
            return None
        g = self.d.sim.world
        return {
            "snapshot": self.d.sim.snapshot,
            "generation": self.d.mode["generation"],
            "width": g.width,
            "height": g.height,
            "resolution": g.meta.resolution,
            "origin": [g.meta.origin_x, g.meta.origin_y, g.meta.origin_yaw],
            "frame_id": "map",
            "age_s": 0.3,
        }

    def map_png(self, snapshot: int) -> bytes | None:
        g = self.d.sim.world
        img = np.full(g.data.shape, 205, dtype=np.uint8)
        img[g.data == 0] = 254
        img[g.data >= 65] = 0
        if self._layer() == "MAPPING":
            img[~self.d.sim.revealed] = 205
        return encode_gray(np.flipud(img))

    def pose_scan(self) -> dict:
        s = self.d.sim
        frame = "map" if self._layer() in ("MAPPING", "NAVIGATION") else "odom"
        return {
            "generation": self.d.mode["generation"],
            "pose": {"x": s.x, "y": s.y, "yaw": s.yaw, "frame": frame, "age_s": 0.05},
            "scan": {"points": s.scan(), "frame": frame, "age_s": 0.03},
        }


class DemoAdapter:
    """Implements server.Adapter with fabricated state; mutations follow the real
    state machines closely enough to click through every workflow."""

    def __init__(self, maps_dir: str, state_dir: str) -> None:
        self.maps_dir = maps_dir
        self.evidence_dir = os.path.join(state_dir, "commissioning")
        self.pp_unlocked = False  # the real profile ships pp locked; the DEMO bar can flip it
        self.comm_plan: dict | None = None
        self.lock = threading.RLock()
        self.sim = Sim(fixtures.surfaces_only(gridio.read(WORLD_YAML)))
        self.live = DemoLive(self)
        self.offline = False
        self.ops: dict[str, dict] = {}
        self.events_: list[dict] = []
        self.manual = (0.0, 0.0, 0.0)  # v, w, expires (monotonic)
        self.exec_plan: list | None = None
        self.exec_i, self.exec_t = 0, 0.0
        self.seq = 0
        self.completed_jobs = 0
        self.scenario("idle")
        threading.Thread(target=self._loop, daemon=True, name="demo-sim").start()

    # ---- scenario presets ----------------------------------------------------------

    def _reset(self) -> None:
        self.mode = mode_dict()
        self.panel = {"valid": True, "mode_auto": False}
        self.drives = {"operational": True, "left": "Operation enabled", "right": "Operation enabled"}
        self.mapping = None
        self.loc = None
        self.run = None
        self.comm = None
        self.exec_plan = None
        self.offline = False
        self.manual = (0.0, 0.0, 0.0)

    def _bundle1(self):
        m, _ = mb.load(self.maps_dir, "sim_factory", 1)
        return m

    def _nav(self, gen: int = 2) -> None:
        m = self._bundle1()
        self.mode = mode_dict(
            generation=gen,
            mode=3,
            requested_mode=3,
            layer_ready=True,
            active_map_id="sim_factory",
            active_map_revision=1,
            active_map_sha256=m.sha256,
            autonomous_available=True,
        )

    def _loc(self, state: int, **kw) -> dict:
        d = {
            "generation": self.mode["generation"],
            "state": state,
            "operator_confirmed": state == 2,
            "can_confirm": state == 1,
            "cov_xx": 0.0009,
            "cov_yy": 0.0011,
            "cov_yaw": 0.0004,
            "scan_match": 0.83,
            "scan_long": 0.01,
            "last_jump_m": 0.012,
            "last_jump_rad": 0.004,
            "scan_age_s": 0.03,
            "wheels_age_s": 0.02,
            "imu_age_s": 0.02,
            "tf_age_s": 0.05,
            "amcl_age_s": 0.4,
            "reason": "",
        }
        d.update(kw)
        d["state_name"] = LOC_STATES[d["state"]]
        return d

    def _run(self, state: int, **kw) -> dict:
        mission = self._first_mission()
        d = {
            "generation": self.mode["generation"],
            "state": state,
            "run_id": "run-0007",
            "mission_id": mission.get("mission_id", "") if mission else "",
            "map_id": "sim_factory",
            "map_revision": 1,
            "route_id": "demo_loop",
            "route_revision": 1,
            "step_index": -1,
            "step_id": "",
            "step_type": "",
            "remaining_turn_rad": 0.0,
            "cross_track_m": 0.0,
            "localization_state": 2,
            "resume_prepared": False,
            "pose_valid": True,
            "pose_x": self.sim.x,
            "pose_y": self.sim.y,
            "pose_yaw": self.sim.yaw,
            "reason": "",
        }
        d.update(kw)
        d["state_name"] = RUN_STATES[d["state"]]
        return d

    def _first_mission(self) -> dict | None:
        ms = store.list_missions(self.maps_dir)
        return ms[0] if ms else None

    def scenario(self, name: str) -> None:
        with self.lock:
            self._reset()
            s = self.sim
            s.x, s.y, s.yaw = 0.0, 0.0, 0.0
            if name == "mapping" or name == "review":
                self.mode = mode_dict(generation=2, mode=2, requested_mode=2, layer_ready=True)
                s.revealed[:] = False
                s.reveal_at = None
                s.x = 6.0 if name == "mapping" else 0.03
                s.reveal()
                if name == "review":
                    s.revealed[:] = True
                    s.snapshot += 1
                self.mapping = self._mapping_dict(1 if name == "mapping" else 2)
            elif name in ("nav", "ready", "executing", "blocked", "runfault", "lost"):
                self._nav()
                s.x, s.y, s.yaw = 0.02, -0.01, 0.01
                loc_state = {"nav": 1, "lost": 3}.get(name, 2)
                self.loc = self._loc(
                    loc_state,
                    reason="covariance grew: sigma_xy 0.27 m > 0.22 m for 1.0 s" if name == "lost" else "",
                )
                if name == "ready":
                    self.run = self._run(1, reason="waiting for AUTO + physical Start")
                elif name == "executing":
                    self.panel["mode_auto"] = True
                    self._start_exec()
                elif name == "blocked":
                    self.panel["mode_auto"] = True
                    s.x = 4.2
                    self.run = self._run(
                        4,
                        step_index=0,
                        step_id="s1",
                        step_type="straight",
                        cross_track_m=0.018,
                        reason="obstruction in the swept footprint 1.1 m ahead (23 scan points)",
                    )
                elif name == "runfault":
                    s.x, s.yaw = 12.0, 1.2
                    self.run = self._run(
                        5,
                        step_index=1,
                        step_id="s2",
                        step_type="rotate",
                        remaining_turn_rad=1.94,
                        reason="turn travel 4.6 deg from commanded (limit 4.0)",
                    )
                    self._event(2, "route_executor", "RUN_FAULT", self.run["reason"])
            elif name == "layerfault":
                self.mode = mode_dict(
                    generation=3,
                    mode=5,
                    requested_mode=3,
                    fault_code="LAYER_EXITED",
                    reason="navigation layer exited (amcl: exit code -6); see ~/.amr/logs/layer.log",
                    manual_available=True,
                )
                self._event(2, "supervisor", "LAYER_EXITED", self.mode["reason"])
            elif name == "commissioning":
                self.comm = {
                    "phase": 1,
                    "phase_name": "PREPARED",
                    "generation": 1,
                    "plan_id": "square_1m",
                    "segment": 0,
                    "segments": 8,
                    "kind": "",
                    "progress_left_m": 0.0,
                    "progress_right_m": 0.0,
                    "speed_mps": 0.0,
                    "pose_x_m": 0.0,
                    "pose_y_m": 0.0,
                    "heading_deg": 0.0,
                    "gyro_heading_deg": 0.0,
                    "completed": self.completed_jobs,
                    "reason": "plan held; press physical Start under MANUAL",
                    "results_path": "",
                    "backend": "pv",
                    "run_id": "",
                }
                self.comm_plan = None
                self.mode["manual_available"] = False
            elif name == "offline":
                self.offline = True
            self._event(0, "demo", "SCENARIO", f"scenario -> {name}")

    def _mapping_dict(self, state: int, **kw) -> dict:
        d = {
            "generation": self.mode["generation"],
            "state": state,
            "map_id": "line_section",
            "revision": 0,
            "description": "tape arrow A, facing the rack ends",
            "start": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "closure_available": state >= 2,
            "closure_dx_m": 0.031,
            "closure_dy_m": -0.012,
            "closure_dyaw_rad": math.radians(0.8),
            "saved_path": "",
            "message": "",
        }
        d.update(kw)
        d["state_name"] = MAP_STATES[d["state"]]
        return d

    # ---- physical panel (DEMO bar) --------------------------------------------------

    def panel_action(self, what: str) -> str:
        with self.lock:
            if what == "selector":
                self.panel["mode_auto"] = not self.panel["mode_auto"]
                self.manual = (0.0, 0.0, 0.0)
                if not self.panel["mode_auto"] and self.run and self.run["state"] in (2, 3, 4):
                    self.run = self._run(5, reason="selector switched to MANUAL during the run")
                    self.exec_plan = None
                return "AUTO" if self.panel["mode_auto"] else "MANUAL"
            if what == "estop":
                op = not self.drives["operational"]
                st = "Operation enabled" if op else "Switch on disabled"
                self.drives = {"operational": op, "left": st, "right": st}
                if not op:
                    self.exec_plan = None
                    self.sim.v = self.sim.w = 0.0
                    if self.run and self.run["state"] == 2:
                        self.run = self._run(5, reason="drives not operational")
                return "released" if op else "pressed"
            if what == "start":
                if self.comm and self.comm["phase_name"] == "PREPARED" and not self.panel["mode_auto"]:
                    self.comm_t = time.monotonic()
                    run_id = f"{self.comm['plan_id']}-{int(self.comm_t * 1000) % 100_000_000}"
                    kind = self.comm_plan["segments"][0]["kind"] if self.comm_plan else "straight"
                    self.comm.update(
                        phase=2,
                        phase_name="RUNNING",
                        segment=1,
                        kind=kind,
                        reason="",
                        run_id=run_id if self.comm.get("backend") == "pp" else "",
                    )
                    return "commissioning started"
                if not self.panel["mode_auto"]:
                    return "Start ignored: selector MANUAL"
                if self.run and (self.run["state"] == 1 or self.run.get("resume_prepared")):
                    self._start_exec(resume=self.run["state"] != 1)
                    return "EXECUTING"
                return "Start ignored: nothing ready"
            if what == "pp":
                self.pp_unlocked = not self.pp_unlocked
                return "PP unlocked (demo only)" if self.pp_unlocked else "PP locked"
            if what == "reset":
                if self.run and self.run["state"] == 5:
                    self.run = self._run(0, reason="fault acknowledged (physical Reset)")
                return "reset"
        return "?"

    # ---- executor ---------------------------------------------------------------------

    def _start_exec(self, resume: bool = False) -> None:
        mission = self._first_mission() if not self.run else None
        mid = (self.run or {}).get("mission_id") or (mission or {}).get("mission_id")
        if not mid:
            self.run = self._run(0, reason="no mission")
            return
        mis = store.load_mission(self.maps_dir, mid)
        route, _ = store.load_route(
            self.maps_dir, mis["map"]["id"], mis["route"]["id"], int(mis["route"]["revision"])
        )
        comp = compile_route(route)
        self.exec_limits = (route.limits.linear_mps, route.limits.angular_rad_s)
        if not resume:
            self.exec_i = 0
            st = comp.steps[0].start
            self.sim.x, self.sim.y, self.sim.yaw = st
        self.exec_plan = comp.steps
        self.exec_t = time.monotonic()
        self.run = self._run(2, mission_id=mid, route_id=route.route_id, route_revision=route.revision)
        self._event(0, "route_executor", "RUN_START", f"{mid} {'resumed' if resume else 'started'}")

    def _exec_tick(self) -> None:
        s = self.sim
        if self.exec_i >= len(self.exec_plan):
            self.exec_plan = None
            s.v = s.w = 0.0
            self.run = self._run(6, step_index=self.exec_i - 1, reason="route complete")
            self._event(0, "route_executor", "RUN_DONE", self.run["mission_id"])
            return
        st = self.exec_plan[self.exec_i]
        v, w = self.exec_limits
        extra = {"step_index": self.exec_i, "step_id": st.id, "step_type": st.type}
        if st.type == "straight":
            ex, ey, eyaw = st.end
            left = (ex - s.x) * math.cos(eyaw) + (ey - s.y) * math.sin(eyaw)
            s.yaw, s.w = eyaw, 0.0
            s.v = min(v, max(0.05, left * 1.5))
            if left <= 0.01:
                s.x, s.y, s.v = ex, ey, 0.0
                self.exec_i += 1
            extra["cross_track_m"] = 0.004 * math.sin(time.monotonic() * 1.3)
        else:
            target = st.start[2] + st.signed_angle_rad
            done = getattr(self, "_turned", 0.0)
            rem = st.signed_angle_rad - done
            s.v, s.w = 0.0, math.copysign(w, rem)
            self._turned = done + s.w * 0.05
            extra["remaining_turn_rad"] = rem
            if abs(rem) <= w * 0.05:
                s.w, self._turned = 0.0, 0.0
                s.yaw = math.atan2(math.sin(target), math.cos(target))
                self.exec_i += 1
        self.run = self._run(2, mission_id=self.run["mission_id"], **extra)

    # ---- sim loop ---------------------------------------------------------------------

    def _loop(self) -> None:
        dt = 0.05
        while True:
            time.sleep(dt)
            with self.lock:
                s = self.sim
                now = time.monotonic()
                if self.exec_plan and self.drives["operational"] and self.panel["mode_auto"]:
                    self._exec_tick()
                elif self.drives["operational"] and not self.panel["mode_auto"] and now < self.manual[2]:
                    s.v, s.w = self.manual[0], self.manual[1]
                else:
                    s.v = s.w = 0.0
                s.step(dt)
                if self.mode["mode_name"] == "MAPPING" and self.mapping and self.mapping["state"] == 1:
                    s.reveal()
                if self.run and self.run["state"] != 2:
                    self.run.update(pose_x=s.x, pose_y=s.y, pose_yaw=s.yaw)
                if self.comm and self.comm["phase_name"] == "RUNNING" and self.comm_plan:
                    self._comm_plan_tick(now)
                elif self.comm and self.comm["phase_name"] == "RUNNING":
                    t = now - self.comm_t
                    seg = min(8, 1 + int(t / 4))
                    self.comm.update(
                        segment=seg,
                        kind="straight" if seg % 2 else "pivot",
                        progress_left_m=round(0.25 * (t % 4), 3),
                        progress_right_m=round(0.25 * (t % 4), 3),
                        speed_mps=0.25 if seg % 2 else 0.0,
                    )
                    if t > 32:
                        self.completed_jobs += 1
                        self.comm.update(
                            phase=4,
                            phase_name="DONE",
                            speed_mps=0.0,
                            completed=self.completed_jobs,
                            reason="8/8 segments",
                            results_path="~/.amr/commissioning/square_1m-demo.json",
                        )

    def _comm_plan_tick(self, now: float) -> None:
        seg = self.comm_plan["segments"][0]
        dur = max(0.5, float(seg["duration_s"]))
        f = min(1.0, (now - self.comm_t) / dur)
        self.comm.update(
            progress_left_m=round(f * seg["left_m"], 4),
            progress_right_m=round(f * seg["right_m"], 4),
            speed_mps=self.comm_plan["speed_mps"] if f < 1.0 else 0.0,
            heading_deg=round(f * seg["commanded"]["heading_deg"], 2),
            gyro_heading_deg=round(f * seg["commanded"]["heading_deg"] * 1.004, 2),
        )
        if f < 1.0:
            return
        self.completed_jobs += 1
        ev = {
            "plan_id": self.comm["plan_id"],
            "phase": "DONE",
            "reason": "",
            "plan": self.comm_plan,
            "results": [],
            "gyro_heading_deg": self.comm["gyro_heading_deg"],
            "backend": self.comm.get("backend", "pv"),
            "run_id": self.comm.get("run_id", ""),
            "pp_result": {"outcome": "done", "error_counts": [-7, 4]}
            if self.comm.get("backend") == "pp"
            else None,
            "profile": "agv-01 (demo)",
        }
        os.makedirs(self.evidence_dir, exist_ok=True)
        path = os.path.join(
            self.evidence_dir, f"{self.comm['plan_id']}-{time.strftime('%Y%m%d-%H%M%S')}.json"
        )
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(ev, fh, indent=1)
        self.comm.update(phase=4, phase_name="DONE", completed=self.completed_jobs, results_path=path)

    def pp_status(self) -> dict:
        return {
            "available": self.pp_unlocked,
            "reason": "" if self.pp_unlocked else "pp locked in the profile (pp.enabled false)",
            "state": "idle",
            "run_id": "",
            "outcome": "",
            "age_s": 0.1,
        }

    # ---- server.Adapter: state ------------------------------------------------------

    def state(self) -> dict:
        if self.offline:
            from flask import abort  # noqa: PLC0415

            abort(503)
        with self.lock:
            s = self.sim
            wl = (s.v - s.w * 0.487 / 2) / 0.09
            wr = (s.v + s.w * 0.487 / 2) / 0.09
            src = (
                "follow"
                if self.exec_plan and s.v
                else "rotate"
                if self.exec_plan
                else ("manual" if (s.v or s.w) else "none")
            )
            return {
                "mode": dict(self.mode),
                "mode_age_s": 0.2,
                "lease": {
                    "instance": "demo-instance",
                    "generation": self.mode["generation"],
                    "allowed": (1 if self.mode["manual_available"] else 0) | (2 if self.exec_plan else 0),
                },
                "panel": dict(self.panel, age_s=0.02),
                "drives": dict(self.drives, age_s=0.05),
                "mux": {
                    "source": src,
                    "inhibited": not self.drives["operational"],
                    "reason": "" if self.drives["operational"] else "drives not operational",
                    "generation": self.mode["generation"],
                    "left_rad_s": wl,
                    "right_rad_s": wr,
                    "age_s": 0.02,
                },
                "mapping": self.mapping and dict(self.mapping),
                "mapping_age_s": 0.5 if self.mapping else None,
                "mapping_stale": False,
                "localization": self.loc and dict(self.loc),
                "localization_age_s": 0.3 if self.loc else None,
                "localization_stale": False,
                "run": self.run and dict(self.run),
                "run_age_s": 0.1 if self.run else None,
                "run_stale": False,
                "t": time.time(),
            }

    def supervisor_identity(self):
        return "demo-instance", self.mode["generation"]

    def manual_allowed(self):
        if self.comm and self.comm["phase_name"] in ("PREPARED", "RUNNING"):
            return False, "a commissioning job is held or running; clear it first"
        if self.panel["mode_auto"]:
            return False, "selector is AUTO"
        return True, ""

    def manual_publish(self, cmd) -> None:
        with self.lock:
            self.manual = (float(cmd.v), float(cmd.w), time.monotonic() + float(cmd.valid_for_s))

    # ---- supervisor operations --------------------------------------------------------

    def _op(self, kind: str, ok: bool = True, message: str = "done", **kw) -> str:
        oid = uuid.uuid4().hex[:12]
        self.ops[oid] = {
            "operation_id": oid,
            "request_id": "",
            "kind": kind,
            "phase": "done",
            "status": 1 if ok else 2,
            "status_name": "SUCCEEDED" if ok else "FAILED",
            "map_id": kw.get("map_id", ""),
            "map_revision": kw.get("map_revision", 0),
            "map_sha256": "",
            "message": message,
        }
        return oid

    def request_mode(self, target, map_id, map_revision, request_id):
        with self.lock:
            if self.run and self.run["state"] in (1, 2, 3, 4):
                return False, "", "a run is READY/EXECUTING/PAUSED/BLOCKED; abort it first"
            if target == MODE["NAVIGATION"]:
                try:
                    m, _ = mb.load(self.maps_dir, map_id, int(map_revision))
                except mb.BundleError as e:
                    return True, self._op("mode", False, str(e)), "accepted"
                self.mode = mode_dict(
                    generation=self.mode["generation"] + 1,
                    mode=3,
                    requested_mode=3,
                    layer_ready=True,
                    active_map_id=map_id,
                    active_map_revision=int(map_revision),
                    active_map_sha256=m.sha256,
                    autonomous_available=True,
                )
                self.loc = self._loc(0)
                self.run = None
                self._event(0, "supervisor", "MODE_CHANGE", f"NAVIGATION on {map_id} rev{map_revision}")
            else:
                self._reset()
                self.mode = mode_dict(generation=self.mode["generation"] + 1)
                self._event(0, "supervisor", "MODE_CHANGE", "IDLE")
            return True, self._op("mode"), "accepted"

    def survey_move(self, kind, value):
        return False, "demo: no vehicle to move"

    def survey_move_stop(self):
        return False, "no move running"

    def survey_request(self, operation, map_id, description, request_id):
        with self.lock:
            name = {0: "start", 1: "returned", 2: "save", 3: "abort"}[operation]
            if name == "start":
                self.mode = mode_dict(
                    generation=self.mode["generation"] + 1, mode=2, requested_mode=2, layer_ready=True
                )
                self.sim.revealed[:] = False
                self.sim.reveal_at = None
                self.sim.reveal()
                self.mapping = self._mapping_dict(
                    1,
                    map_id=map_id,
                    description=description,
                    closure_available=False,
                    start={"x": self.sim.x, "y": self.sim.y, "theta": self.sim.yaw},
                )
            elif name == "returned":
                st = self.mapping["start"] if self.mapping else {"x": 0, "y": 0, "theta": 0}
                self.mapping = self._mapping_dict(
                    2,
                    map_id=self.mapping["map_id"],
                    description=self.mapping["description"],
                    start=st,
                    closure_dx_m=self.sim.x - st["x"],
                    closure_dy_m=self.sim.y - st["y"],
                    closure_dyaw_rad=self.sim.yaw - st["theta"],
                )
            elif name == "save":
                mid = self.mapping["map_id"] if self.mapping else map_id
                rev_dir = fixtures.write_world_as_bundle(self.maps_dir, mid, WORLD_YAML)
                rev = int(os.path.basename(rev_dir)[3:])
                self._event(0, "mapping_session", "SAVED", f"{mid} rev{rev}")
                self._reset()
                self.mode = mode_dict(
                    generation=self.mode["generation"] + 1, last_survey_map_id=mid, last_survey_revision=rev
                )
                return (
                    True,
                    self._op("survey_save", map_id=mid, map_revision=rev, message=f"saved {mid} rev{rev}"),
                    "accepted",
                )
            else:
                self._reset()
                self.mode = mode_dict(generation=self.mode["generation"] + 1)
            return True, self._op(f"survey_{name}"), "accepted"

    def get_operation(self, operation_id):
        return self.ops.get(operation_id)

    def recover(self):
        with self.lock:
            if self.mode["mode_name"] != "FAULT":
                return False, "nothing to recover"
            self._reset()
            self.mode = mode_dict(generation=self.mode["generation"] + 1)
            return True, "recovered to IDLE"

    # ---- localisation / run -----------------------------------------------------------

    def set_initial_pose(self, x, y, yaw, map_id, map_revision, generation, sha256=""):
        # amr_web.adapter imports rclpy at module level, so its initial_pose_mismatch() is not
        # importable here; this is the same identity check, abridged
        with self.lock:
            m = self.mode
            why = ""
            if m["mode_name"] != "NAVIGATION":
                why = "no navigation map is active"
            elif (map_id, int(map_revision)) != (m["active_map_id"], m["active_map_revision"]):
                why = (
                    f"the pose was drawn on {map_id} rev{map_revision}, but the vehicle is running "
                    f"{m['active_map_id']} rev{m['active_map_revision']}"
                )
            elif int(generation) != m["generation"]:
                why = "the vehicle changed mode since the map was shown; look again"
            if why:
                return False, why
            self.sim.x, self.sim.y, self.sim.yaw = x, y, yaw
            self.loc = self._loc(1)
            return True, "initial pose published"

    def localization_confirm(self):
        with self.lock:
            if not self.loc or not self.loc["can_confirm"]:
                return False, "cannot confirm yet"
            self.loc = self._loc(2)
            return True, "READY"

    def localization_reset(self):
        with self.lock:
            self.loc = self._loc(0)
            return True, "localisation reset"

    def publish_route_preview(self, compiled, frame_id):
        pass

    def run_mission(self, mission_id):
        with self.lock:
            if not self.loc or self.loc["state"] != 2:
                return False, "localisation not READY"
            self.run = self._run(1, mission_id=mission_id, reason="waiting for AUTO + physical Start")
            return True, f"mission {mission_id} loaded"

    def pause(self):
        with self.lock:
            if not self.run or self.run["state"] != 2:
                return False, "not executing"
            self.exec_plan = None
            self.run = self._run(
                3, mission_id=self.run["mission_id"], step_index=self.exec_i, reason="operator pause"
            )
            return True, "paused"

    def abort(self):
        with self.lock:
            self.exec_plan = None
            self.run = self._run(0, reason="aborted by operator")
            return True, "aborted"

    def prepare_resume(self):
        with self.lock:
            if not self.run or self.run["state"] not in (3, 4):
                return False, "nothing to resume"
            self.run["resume_prepared"] = True
            self.run["reason"] = "resume prepared; press physical Start under AUTO"
            return True, "resume prepared"

    def ack_fault(self):
        with self.lock:
            if not self.run or self.run["state"] != 5:
                return False, "no fault"
            self.run = self._run(0, reason="fault acknowledged")
            return True, "acknowledged"

    # ---- diagnostics ------------------------------------------------------------------

    def diagnostics(self) -> dict:
        with self.lock:
            s, op = self.sim, self.drives["operational"]
            rpm_l = (s.v - s.w * 0.2435) / 0.09 * 60 / (2 * math.pi) * 30
            rpm_r = (s.v + s.w * 0.2435) / 0.09 * 60 / (2 * math.pi) * 30
        t = time.monotonic()

        def drive(label, node, rpm):
            return {
                "name": f"drive/{label}",
                "hardware_id": f"node {node}",
                "level": 0 if op else 1,
                "message": "" if op else "not operational",
                "age_s": 0.1,
                "t": t,
                "values": {
                    "state": self.drives[label],
                    "statusword": "0x1737" if op else "0x1A50",
                    "error_register": "0x00",
                    "rpm": f"{rpm:.0f}",
                    "position_counts": str(int(t * 1000) % 1_000_000),
                    "nmt": "OPERATIONAL",
                },
            }

        return {
            "drive/left": drive("left", 1, rpm_l),
            "drive/right": drive("right", 2, rpm_r),
            "can/bus": {
                "name": "can/bus",
                "hardware_id": "can0",
                "level": 0,
                "message": "",
                "age_s": 0.1,
                "t": t,
                "values": {
                    "link_state": "up",
                    "fault_reason": "",
                    "monitor_reads": "18422",
                    "sdo_timeouts": "0",
                    "require_supervisor": "True",
                    "feedback_hz": "50",
                    "pc_loss_ms": "500",
                    "setpoint_gate": "open",
                    "nonfinite_cmds": "0",
                    "stop_unconfirmed": "",
                    "heartbeat_withheld": "False",
                    "cleanup_failures": "",
                },
            },
            "imu/mls": {
                "name": "imu/mls",
                "hardware_id": "node 10",
                "level": 0,
                "message": "",
                "age_s": 0.1,
                "t": t,
                "values": {"mode": "sdo_poll", "samples": str(int(t * 50)), "misses": "3", "gyro_sign": "1"},
            },
        }

    def io_image(self) -> dict | None:
        with self.lock:
            di = [False] * 16
            di[3] = self.panel["mode_auto"]
            horn = bool(self.drives["operational"] and (self.sim.v or self.sim.w))
        do = [False] * 16
        do[0] = horn
        return {
            "comms_ok": True,
            "rx_age_s": 0.02,
            "di_names": DI_NAMES,
            "di": di,
            "do_names": DO_NAMES,
            "do_readback": do,
            "do_requested": do,
            "scans": int(time.monotonic() * 50) % 10**7,
            "errors": 0,
            "writes": 412,
            "detail": "192.168.1.30:502 (demo)",
            "age_s": 0.02,
        }

    def _event(self, level: int, source: str, code: str, text: str) -> None:
        self.seq += 1
        self.events_.append(
            {
                "n": self.seq,
                "t": time.time(),
                "source": source,
                "level": ["info", "warn", "error"][level],
                "code": code,
                "text": text,
                "seq": self.seq,
            }
        )
        del self.events_[:-300]

    def events(self, since: int = 0) -> list[dict]:
        return [e for e in self.events_ if e["n"] > since]

    # ---- commissioning ----------------------------------------------------------------

    def commissioning(self) -> dict | None:
        with self.lock:
            return (
                dict(self.comm, age_s=0.1)
                if self.comm
                else {
                    "phase": 0,
                    "phase_name": "IDLE",
                    "generation": 1,
                    "plan_id": "",
                    "segment": 0,
                    "segments": 0,
                    "kind": "",
                    "progress_left_m": 0.0,
                    "progress_right_m": 0.0,
                    "speed_mps": 0.0,
                    "pose_x_m": 0.0,
                    "pose_y_m": 0.0,
                    "heading_deg": 0.0,
                    "gyro_heading_deg": 0.0,
                    "completed": self.completed_jobs,
                    "reason": "",
                    "results_path": "",
                    "backend": "pv",
                    "run_id": "",
                    "age_s": 0.1,
                }
            )

    def commissioning_plan(self, plan_json: str):
        """The real planner and the real job's backend rules, minus the node around them."""
        try:
            import amr_base.agv_repo  # noqa: F401, PLC0415  puts repo core/ on sys.path
            from amr_base import commissioning as cj  # noqa: PLC0415
            from amr_base import pp  # noqa: PLC0415

            import blindrun  # noqa: PLC0415
            import config  # noqa: PLC0415

            spec = json.loads(plan_json)
            planned = blindrun.plan(spec.get("segments"), spec.get("speed"), 1_080_000)
            backend = spec.get("backend", "pv")
            if backend not in cj.BACKENDS:
                raise ValueError(f"backend must be one of {', '.join(cj.BACKENDS)}")
            if planned["speed_mps"] > config.BLIND_MAX_SPEED_MPS + 1e-9:
                raise ValueError(f"speed exceeds blind_run.max_speed_mps ({config.BLIND_MAX_SPEED_MPS:g})")
            out = dict(planned, backend=backend)
            if backend == "pp":
                if len(planned["segments"]) != 1:
                    raise ValueError("a pp plan is exactly one segment (one move per Start)")
                if not self.pp_unlocked:
                    raise ValueError("pp not available: pp locked in the profile (pp.enabled false)")
                ms = pp.move_spec(planned["segments"][0], config.BLIND_ACCEL_RPM_S, config.BLIND_ACCEL_RPM_S)
                out["pp"] = cj.pp_spec_dict(ms)
        except Exception as e:  # noqa: BLE001  - show the planner's own message, as the node does
            return False, str(e), ""
        with self.lock:
            self.scenario("commissioning")
            self.comm_plan = out if len(planned["segments"]) == 1 else None
            self.comm.update(
                segments=len(planned["segments"]),
                plan_id=str(spec.get("id") or "plan"),
                backend=backend,
                speed_mps=0.0,
            )
        return (
            True,
            f"plan {spec.get('id') or 'plan'} held: press physical Start under MANUAL to run",
            json.dumps(out),
        )

    def commissioning_clear(self):
        with self.lock:
            self.comm = None
            self.comm_plan = None
            self.mode["manual_available"] = True
            return True, "cleared"


# ---- seed data -------------------------------------------------------------------------

DEMO_ROUTE = {
    "schema_version": 1,
    "route_id": "demo_loop",
    "revision": 0,
    "map": {"id": "sim_factory", "revision": 1, "sha256": ""},
    "frame_id": "map",
    "start": {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0},
    "limits": {"linear_mps": 0.4},
    "steps": [
        {"id": "s1", "type": "straight", "to": {"x_m": 12.0, "y_m": 0.0}},
        {"id": "s2", "type": "rotate", "direction": "ccw", "angle_deg": 180},
        {"id": "s3", "type": "straight", "to": {"x_m": 0.0, "y_m": 0.0}},
        {"id": "s4", "type": "rotate", "direction": "ccw", "angle_deg": 180},
    ],
    "repeat_count": 1,
}


def seed(maps_dir: str, client) -> None:
    if not mb.list_revisions(maps_dir, "sim_factory"):
        fixtures.write_world_as_bundle(maps_dir, "sim_factory", WORLD_YAML)
    if not store.list_routes(maps_dir, "sim_factory"):
        r = client.post("/api/maps/sim_factory/1/routes/save", json=DEMO_ROUTE)
        if r.status_code != 200:
            print("demo route not saved:", r.get_json(), file=sys.stderr)
            return
        rrev = r.get_json().get("revision", 1)
        client.post(
            "/api/missions",
            json={
                "map_id": "sim_factory",
                "map_revision": 1,
                "route_id": "demo_loop",
                "route_revision": int(rrev),
            },
        )


# ---- DEMO bar --------------------------------------------------------------------------

BAR = """
<div id="demo-bar" style="position:fixed;right:12px;bottom:12px;z-index:99999;
 max-width:min(560px,calc(100vw - 24px));
 font:12px/1.3 system-ui,sans-serif;background:#1d1d1f;color:#f5f5f7;border-radius:10px;padding:8px 10px;
 box-shadow:0 6px 24px rgba(0,0,0,.35);opacity:.94">
 <div style="display:flex;justify-content:space-between;gap:8px;align-items:center;margin-bottom:6px">
  <b>DEMO &middot; no hardware</b><span id="demo-msg" style="opacity:.75"></span>
  <button onclick="this.closest('#demo-bar').querySelector('.b').hidden^=1"
   style="all:unset;cursor:pointer">&#9776;</button>
 </div>
 <div class="b">
  <div style="display:flex;flex-wrap:wrap;gap:4px;margin-bottom:6px">%SCEN%</div>
  <div style="display:flex;flex-wrap:wrap;gap:4px">
   <button data-p="selector">Selector AUTO/MANUAL</button><button data-p="start">&#9654; Start</button>
   <button data-p="reset">Reset</button>
   <button data-p="pp" title="Flip the demo PP availability">PP lock</button>
   <button data-p="estop" style="background:#b3261e;color:#fff">E-stop</button>
  </div>
 </div>
</div>
<style>#demo-bar button{border:0;border-radius:6px;padding:4px 8px;
 background:#3a3a3c;color:#f5f5f7;cursor:pointer}
#demo-bar button:hover{background:#48484a}</style>
<script>(()=>{const m=document.getElementById('demo-msg');
document.querySelectorAll('#demo-bar [data-s]').forEach(b=>b.onclick=async()=>{
 const r=await fetch('/demo/scenario/'+b.dataset.s,{method:'POST'});m.textContent=(await r.json()).message;
 if(b.dataset.s!=='offline')setTimeout(()=>location.reload(),150);});
document.querySelectorAll('#demo-bar [data-p]').forEach(b=>b.onclick=async()=>{
 const r=await fetch('/demo/panel/'+b.dataset.p,{method:'POST'});
 m.textContent=(await r.json()).message;});})();</script>
"""


def build(maps_dir: str, state_dir: str | None = None):
    os.makedirs(maps_dir, exist_ok=True)
    state_dir = state_dir or f"{maps_dir.rstrip(os.sep)}_state"
    if not mb.list_revisions(maps_dir, "sim_factory"):
        fixtures.write_world_as_bundle(maps_dir, "sim_factory", WORLD_YAML)
    demo = DemoAdapter(maps_dir, state_dir)
    app = web_server.create_app(demo, maps_dir, wifi_iface="demo0", state_dir=state_dir)
    bar = BAR.replace("%SCEN%", "".join(f'<button data-s="{k}">{t}</button>' for k, t in SCENARIOS))

    @app.post("/demo/scenario/<name>")
    def demo_scenario(name):
        if name not in dict(SCENARIOS):
            return {"ok": False, "message": f"unknown scenario {name}"}, 404
        demo.scenario(name)
        return {"ok": True, "message": f"scenario: {name}"}

    @app.post("/demo/panel/<what>")
    def demo_panel(what):
        return {"ok": True, "message": demo.panel_action(what)}

    # the real reader shells out to `iw`; keep the route, swap the view
    app.view_functions["api_wifi"] = lambda: {
        "iface": "demo0",
        "connected": True,
        "ssid": "IGP-FACTORY",
        "dbm": -58.0,
        "bars": 3,
    }

    @app.after_request
    def inject_bar(resp):
        if resp.mimetype == "text/html" and not request.path.startswith("/api/"):
            body = resp.get_data(as_text=True)
            if "</body>" in body:
                resp.set_data(body.replace("</body>", bar + "</body>", 1))
        return resp

    seed(maps_dir, app.test_client())  # after every route is registered: the first request freezes setup
    demo.scenario("idle")
    return app, demo


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5051)
    ap.add_argument("--maps-dir", default=os.path.join(tempfile.gettempdir(), "amr_web_demo_maps"))
    a = ap.parse_args()
    app, _ = build(a.maps_dir)
    print(f"demo: http://{a.host}:{a.port}/   maps: {a.maps_dir}")
    app.run(host=a.host, port=a.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
