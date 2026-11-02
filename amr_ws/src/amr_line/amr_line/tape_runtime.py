"""Tape localization inside the LINE layer, P1: teach and shadow (tracked plan 3, section 4).

Pure and clock-fed, like amr_line.job: the node hands it wheel travel, the MLS track
count, the U-turn phase, AMCL poses and readiness, and publishes what it says. NOTHING
here reaches the follower or the command: in P1 the locator only logs where it would
have stopped and branched (Shadow), and teaching only records.

Times are ROS time in seconds (AMCL stamps are ROS time; the odometry buffer must use
the same clock). Wheel travel is SIGNED forward travel, so a pivot adds about nothing.

    at_home()        the AGV is parked at HOME-N: /initialpose gets HOME-N's taught pose;
                     the locator starts on the shadow circuit once readiness is READY
    teach(action)    map <id> <rev> | pass_start <circuit> | pass_stop | pass_discard |
                     passes | pass_delete <name> | station <id> | build | publish |
                     activate <g> | status
"""

import collections
import math

from amr_line import tape_graph as tg
from amr_line import tape_graph_store as store
from amr_line import tape_locator as tl
from amr_line import tape_teach as tt

READY, LOST_STATES = 2, (0, 3)       # LocalizationState: UNLOCALIZED 0, CHECKING 1, READY 2, LOST 3
READINESS_STALE_S = 2.0
STATION_CAPTURE_S = 3.0              # standstill AMCL poses averaged for a station
STATION_MIN_POSES = 5
START_SIGMA_M = 0.03                 # locator sigma at HOME-N after At home
INFO, WARN = "info", "warn"


class TapeRuntime:
    def __init__(self, mission, params, *, maps_dir, state_dir, selection=None, resolve_sha=None,
                 status="", shadow_circuit="C-MRU"):
        """selection: the active pointer ({"map_id", "map_revision", "graph_revision"}) the
        layer was started with, or None. resolve_sha(map_id, rev) -> the bundle's sha256
        (raises on a bundle that does not verify). status: why localization is not running."""
        self.m = mission
        self.p = params
        self.maps_dir, self.state_dir = maps_dir, state_dir
        self.sel = selection
        self.resolve_sha = resolve_sha
        self.shadow_circuit = shadow_circuit
        self.events = []
        self.detail = status
        self.map_sha = ""
        self.graph = self.staged = None
        self.graph_name = ""
        self.route = self.loc = self.shadow = None
        self.recorder = None
        self.pending_home = False
        self.ready, self.ready_t, self.ready_why = False, None, "no readiness yet"
        self.travel = None
        self._poses = collections.deque(maxlen=200)    # (t, x, y, yaw) for station capture
        self._still_since = None
        self._forks = tg.ForkDetector()
        self._uturn_phase = ""
        self._was_lost = ""
        if selection is not None and not status:
            self._load_active()

    # -- setup ---------------------------------------------------------------------
    @property
    def running(self):
        """Is AMCL expected on this layer at all?"""
        return self.sel is not None and not self.detail.startswith("off")

    def _load_active(self):
        sel = self.sel
        try:
            self.map_sha = self.resolve_sha(sel["map_id"], sel["map_revision"]) if self.resolve_sha else ""
        except Exception as e:  # noqa: BLE001 - any bundle problem is a reason, not a crash
            self.detail = f"map {sel['map_id']} rev{sel['map_revision']} refused: {e}"
            return
        g = sel.get("graph_revision")
        if g is None:
            self.detail = f"teaching on {sel['map_id']} rev{sel['map_revision']}: no graph active"
            return
        try:
            graph, _sha = store.load(self.maps_dir, sel["map_id"], g)
        except (store.StoreError, tg.GraphError) as e:
            self.detail = f"graph g{g} refused: {e}"
            return
        why = graph.check_binding(sel["map_id"], sel["map_revision"], self.map_sha)
        if why:
            self.detail = why
            return
        self.graph = graph
        self.graph_name = f"{sel['map_id']} rev{sel['map_revision']} g{g}"
        try:
            self.route = tg.compile_route(self.shadow_circuit, self.m["CIRCUITS"][self.shadow_circuit], graph)
        except (tg.RouteError, KeyError) as e:
            self.detail = f"{self.graph_name}: {e}"
            return
        self.detail = f"{self.graph_name}: press At home at {self.m['HOME']}"

    # -- inputs --------------------------------------------------------------------
    def tick(self, t, travel, nlcp, still, uturn_phase=""):
        """Every control tick. travel: cumulative signed wheel travel (None: unknown)."""
        if self.ready_t is not None and t - self.ready_t > READINESS_STALE_S and self.ready:
            self._readiness(False, "localization state stale")
        if travel is None:
            return
        self.travel = travel
        self._still_since = (self._still_since or t) if still else None
        if self.recorder is not None:
            self.recorder.tick(t, travel, nlcp)
        loc = self.loc
        if loc is not None:
            loc.predict(t, travel)
        ep = self._forks.update(travel, nlcp)
        if ep is not None and loc is not None:
            verdict, node = loc.fork(ep[1], ep[2])
            if verdict == "unexpected":
                self.events.append((WARN, f"shadow: fork with no taught junction near S {loc.S:.2f} m "
                                          f"(sigma {loc.sigma:.2f}): would HOLD position"))
            elif verdict == "resync":
                self.events.append((INFO, f"shadow: resync at {node['id']}, S {loc.S:.2f} m"))
        self._uturn(travel, uturn_phase)
        if self.pending_home and self.ready and self.route is not None:
            self._start_locator(t, travel)
        if loc is not None:
            self._shadow_tick(still)

    def _uturn(self, travel, phase):
        prev, self._uturn_phase = self._uturn_phase, phase or ""
        if prev == "approach" and phase == "stopping" and self.recorder is not None:
            self.recorder.uturn(travel)
        if prev in ("spin", "center", "settle") and not phase and self.loc is not None:
            near = [n for n in self.route.nodes if n["kind"] == "uturn"]
            node = min(near, key=lambda n: abs(n["S"] - self.loc.S)) if near else None
            if node is not None and abs(node["S"] - self.loc.S) <= max(3 * self.loc.sigma, 1.0):
                self.loc.uturn_done(node["id"])
                self.events.append((INFO, f"shadow: U-turn {node['id']} done, S {self.loc.S:.2f} m"))
            else:
                self.events.append((WARN, "shadow: U-turn where the route has none: would HOLD position"))

    def amcl(self, t, stamp, x, y, yaw, cov_xx, cov_xy, cov_yy):
        self._poses.append((stamp, x, y, yaw))
        if self.recorder is not None:
            self.recorder.pose(stamp, x, y)
        if self.loc is None:
            return
        _, _, h = self.route.point(self.loc.S)
        c, s = math.cos(h), math.sin(h)
        var = cov_xx * c * c + 2 * cov_xy * c * s + cov_yy * s * s
        self.loc.correct(t, stamp, x, y, var)

    def readiness(self, t, state, reason=""):
        self.ready_t = t
        if state == READY:
            self._readiness(True, "")
        elif state in LOST_STATES:
            self._readiness(False, reason or ("LOST" if state == 3 else "UNLOCALIZED"))
        else:
            self._readiness(False, "")      # CHECKING: no corrections, not LOST

    def _readiness(self, ready, why):
        self.ready, self.ready_why = ready, why
        if self.loc is not None:
            self.loc.readiness(ready, why)

    # -- the locator -----------------------------------------------------------------
    def at_home(self):
        """(ok, message, pose or None to publish on /initialpose)."""
        if self.route is None:
            return False, f"At home: no route ({self.detail})", None
        st = self.graph.stations.get(self.m["HOME"])
        self.pending_home = True
        self.loc = self.shadow = None
        return (True, f"At home: initial pose {self.m['HOME']} sent; confirm the scans align",
                st.pose)

    def _start_locator(self, t, travel):
        self.pending_home = False
        self.loc = tl.TapeLocator(self.route, self.p, self.graph)
        self.loc.readiness(True)
        self.loc.start(self.route.S_start, START_SIGMA_M, t, travel)
        self.shadow = tl.Shadow(self.route, self.p, self.m["STOP_DISTANCE_M"], self.m["UTURNS"])
        self._was_lost = ""
        self.events.append((INFO, f"shadow: locator started at {self.m['HOME']} on {self.route.circuit}"))

    def _shadow_tick(self, still):
        loc = self.loc
        self.events += self.shadow.tick(loc)
        if loc.lost != self._was_lost:
            self.events.append((WARN, f"shadow: LOST - {loc.lost}") if loc.lost
                               else (INFO, f"shadow: re-acquired at S {loc.S:.2f} m"))
            self._was_lost = loc.lost
        if loc.S >= self.route.S_end or (still and self.shadow.done):
            # Back at HOME (past the end, or every decision logged and standing there):
            # the next lap starts where this one ended.
            loc.S -= self.route.S_end - self.route.S_start
            self.shadow = tl.Shadow(self.route, self.p, self.m["STOP_DISTANCE_M"], self.m["UTURNS"])
            self.events.append((INFO, f"shadow: lap complete at {self.m['HOME']}, sigma {loc.sigma:.3f} m"))

    # -- teaching --------------------------------------------------------------------
    def _need_map(self):
        if self.sel is None:
            return "no map selected: teach map <map_id> <revision>, then restart LINE"
        if not self.map_sha:
            return self.detail or "the selected map did not verify"
        return ""

    def teach(self, t, action, arg=""):
        """(ok, message). Engineer actions; the web gates them by PIN."""
        fn = getattr(self, f"_t_{action}", None)
        if fn is None:
            return False, (f"unknown teach action {action!r}: map, pass_start, pass_stop, pass_discard, "
                           f"passes, pass_delete, station, build, publish, activate, status")
        try:
            return fn(t, (arg or "").strip())
        except (store.StoreError, tg.GraphError, tt.TeachError, ValueError) as e:
            return False, f"{action}: {e}"

    def _t_status(self, _t, _arg):
        rec = f"; recording {self.recorder.summary()}" if self.recorder else ""
        staged = f"; built: {len(self.staged.legs)} legs, {len(self.staged.stations)} stations" \
            if self.staged else ""
        return True, f"{self.detail}{rec}{staged}"

    def _t_map(self, _t, arg):
        parts = arg.split()
        if len(parts) != 2:
            return False, "map <map_id> <revision>"
        map_id, rev = parts[0], int(parts[1])
        sha = self.resolve_sha(map_id, rev) if self.resolve_sha else ""
        store.write_active(self.state_dir, map_id, rev, None, "")
        return True, (f"teaching map {map_id} rev{rev} ({sha[:12]}) selected: it is loaded when LINE "
                      f"starts next (Navigation -> Trackless -> Tracked, or a service restart)")

    def _t_pass_start(self, t, arg):
        why = self._need_map()
        if why:
            return False, why
        if arg not in self.m["CIRCUITS"]:
            return False, f"circuit {arg!r}: expected one of {sorted(self.m['CIRCUITS'])}"
        if not self.ready:
            return False, f"localization is not READY ({self.ready_why or 'confirm the scans align'})"
        if self.recorder is not None:
            return False, f"already recording {self.recorder.circuit}: pass_stop or pass_discard first"
        self.recorder = tt.PassRecorder(arg, started=tt.now_iso())
        return True, f"recording a {arg} pass: start at {self.m['HOME']}, drive the circuit, stop there"

    def _t_pass_stop(self, _t, _arg):
        if self.recorder is None:
            return False, "not recording"
        rec, self.recorder = self.recorder, None
        try:
            tt.split_pass(rec.to_dict(), self.m)
        except tt.TeachError as e:
            return False, f"pass NOT saved: {e} ({rec.summary()})"
        name = store.save_pass(self.state_dir, self.sel["map_id"], self.sel["map_revision"], rec.to_dict())
        return True, f"pass {name} saved: {rec.summary()}"

    def _t_pass_discard(self, _t, _arg):
        was, self.recorder = self.recorder, None
        return (True, "pass discarded") if was else (False, "not recording")

    def _t_passes(self, _t, _arg):
        why = self._need_map()
        if why:
            return False, why
        names = store.list_passes(self.state_dir, self.sel["map_id"], self.sel["map_revision"])
        return True, ", ".join(names) if names else "no passes recorded"

    def _t_pass_delete(self, _t, arg):
        why = self._need_map()
        if why:
            return False, why
        ok = store.delete_pass(self.state_dir, self.sel["map_id"], self.sel["map_revision"], arg)
        return ok, f"pass {arg} deleted" if ok else f"no pass {arg}"

    def _binding(self):
        return {"map_id": self.sel["map_id"], "map_revision": int(self.sel["map_revision"]),
                "map_sha256": self.map_sha}

    def _t_build(self, _t, _arg):
        why = self._need_map()
        if why:
            return False, why
        sel = self.sel
        passes = store.load_passes(self.state_dir, sel["map_id"], sel["map_revision"])
        stations = store.load_stations(self.state_dir, sel["map_id"], sel["map_revision"]).values()
        self.staged, report = tt.build(passes, self.m, self._binding(), stations=stations)
        missing = []
        for cid, c in self.m["CIRCUITS"].items():
            try:
                tg.compile_route(cid, c, self.staged)
            except tg.RouteError as e:
                missing.append(str(e))
        return True, "; ".join(report + (missing or ["every circuit compiles"]))

    def _t_station(self, t, arg):
        why = self._need_map()
        if why:
            return False, why
        if not self.ready:
            return False, "localization is not READY"
        if self._still_since is None or t - self._still_since < STATION_CAPTURE_S:
            return False, f"hold the vehicle still for {STATION_CAPTURE_S:g} s first"
        poses = [p[1:] for p in self._poses if p[0] >= t - STATION_CAPTURE_S]
        if len(poses) < STATION_MIN_POSES:
            return False, (f"only {len(poses)} AMCL poses in the last {STATION_CAPTURE_S:g} s: AMCL updates "
                           f"only on motion; nudge the vehicle and hold again")
        graph = self.staged or self.graph
        if graph is None:
            return False, "no legs yet: record passes and build first"
        pose = tt.mean_pose(poses)
        st = tt.project_station(graph, self.m, arg, pose, tt.now_iso(), len(poses))
        graph.put_station(st)       # refuses a station off its leg
        store.save_station(self.state_dir, self.sel["map_id"], self.sel["map_revision"],
                           {"id": arg, "pose": list(pose), "taught": st.taught, "samples": st.samples})
        return True, f"{arg} taught on {st.leg} at s {st.s:.3f} m, {st.d * 100:+.1f} cm from the tape"

    def _t_publish(self, _t, _arg):
        if self.staged is None:
            return False, "build first"
        n = store.publish(self.maps_dir, self.staged, summary=f"mission {self.m['MISSION_NAME']}")
        return True, f"published {self.sel['map_id']} g{n}: activate {n} to use it"

    def _t_activate(self, _t, arg):
        why = self._need_map()
        if why:
            return False, why
        g, sha = store.load(self.maps_dir, self.sel["map_id"], int(arg))
        bad = g.check_binding(self.sel["map_id"], self.sel["map_revision"], self.map_sha)
        if bad:
            return False, bad
        store.write_active(self.state_dir, self.sel["map_id"], self.sel["map_revision"], int(arg), sha)
        return True, f"g{arg} active: it is loaded when LINE starts next"

    # -- outputs ---------------------------------------------------------------------
    def drain_events(self):
        out, self.events = self.events, []
        return out

    def snapshot(self):
        """LineState's loc_* fields."""
        loc = self.loc
        snap = loc.snapshot() if loc is not None else {}
        idle = "IDLE" if self.running else ""
        health = snap.get("health") or ("WAITING" if self.pending_home else idle)
        detail = (snap.get("lost") or snap.get("last_reject") or "") if loc is not None else self.detail
        return {"loc_health": health, "loc_circuit": self.route.circuit if self.route else "",
                "loc_leg": snap.get("leg", ""), "loc_s_m": float(snap.get("S", 0.0)),
                "loc_sigma_m": float(snap.get("sigma", 0.0)), "loc_rejects": int(snap.get("rejected", 0)),
                "loc_detail": detail, "loc_graph": self.graph_name,
                "teach": self.recorder.summary() if self.recorder else ""}
