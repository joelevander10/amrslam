"""Flask app for the operator pages (spec §1.2, §6.3; unified plan §6). Framework-
independent of ROS: everything ROS goes through an `Adapter` object so the app is
testable with a stub.

One endpoint publishes a velocity: /api/manual/refresh, one ManualCommand per
accepted request, held not latched (unified plan §6.3) - the mux accepts it only
under the physical selector MANUAL with a fresh supervisor lease, and it expires
0.2 s after it was sent. Everything else only selects, draws, validates, saves or
asks the supervisor / coordinators; the physical panel authorises motion.

Mode and survey operations are asynchronous: 202 + operation_id, then
GET /api/operations/<id> until it is terminal.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from typing import Any, Protocol

import numpy as np
from amr_navigation.route import Route, RouteError
from amr_navigation.validate import load_dynamic, load_keepout, validate
from flask import Flask, Response, jsonify, redirect, render_template, request

from amr_maps import edit as mapedit
from amr_maps import grid as gridio
from amr_mission import map_bundle as mb
from amr_navigation import footprint as fpmod
from amr_navigation import store
from amr_web import commissioning_log as clog
from amr_web import jog, netcheck, wifi
from amr_web.png import encode_gray

MODE_IDLE, MODE_NAVIGATION = 1, 3
SURVEY_OPS = {"start": 0, "returned": 1, "save": 2, "abort": 3}


class Adapter(Protocol):
    """What the pages need from ROS. `amr_web.adapter.RosAdapter` implements it."""

    def state(self) -> dict[str, Any]: ...
    def survey_start(self, map_id: str, description: str) -> tuple[bool, str]: ...
    def survey_returned(self) -> tuple[bool, str]: ...
    def survey_save(self, note: str) -> tuple[bool, str]: ...
    def survey_abort(self) -> tuple[bool, str]: ...
    def survey_move(self, kind: str, value: float) -> tuple[bool, str]: ...
    def survey_move_stop(self) -> tuple[bool, str]: ...
    def localization_confirm(self) -> tuple[bool, str]: ...
    def localization_reset(self) -> tuple[bool, str]: ...
    def set_initial_pose(
        self, x: float, y: float, yaw: float, map_id: str, map_revision: int, generation: int, sha256: str
    ) -> tuple[bool, str]: ...
    def publish_route_preview(self, compiled, frame_id: str) -> None: ...
    def run_mission(self, mission_id: str) -> tuple[bool, str]: ...
    def pause(self) -> tuple[bool, str]: ...
    def abort(self) -> tuple[bool, str]: ...
    def prepare_resume(self) -> tuple[bool, str]: ...
    def ack_fault(self) -> tuple[bool, str]: ...
    # supervisor (unified plan §4.2, §6.2)
    def supervisor_identity(self) -> tuple[str, int]: ...
    def request_mode(
        self, target: int, map_id: str, map_revision: int, request_id: str
    ) -> tuple[bool, str, str]: ...
    def survey_request(
        self, operation: int, map_id: str, description: str, request_id: str
    ) -> tuple[bool, str, str]: ...
    def get_operation(self, operation_id: str) -> dict | None: ...
    def recover(self) -> tuple[bool, str]: ...
    def manual_publish(self, cmd) -> None: ...
    # diagnostics (unified plan §7): snapshots only; a GET never causes bus/Modbus traffic
    def diagnostics(self) -> dict: ...
    def io_image(self) -> dict | None: ...
    def events(self, since: int = 0) -> list[dict]: ...
    # commissioning (unified plan §7.2): plan/clear move nothing; only physical Start executes
    def commissioning(self) -> dict | None: ...
    def commissioning_plan(self, plan_json: str) -> tuple[bool, str, str]: ...
    def commissioning_clear(self) -> tuple[bool, str]: ...


def _result(ok: bool, message: str, status_fail: int = 409, **extra):
    body = {"ok": ok, "message": message, **extra}
    return jsonify(body), (200 if ok else status_fail)


def _op_kind(op) -> str:
    """dynamic | undynamic | free | unknown | occupied, for an edits.json op."""
    if not isinstance(op, dict):
        return ""
    return str(op.get("value", "")) if op.get("op") == "paint" else str(op.get("op", ""))


def _edits_summary(e: dict | None) -> dict | None:
    """What the Maps page shows about how a revision was derived (edits.json)."""
    if not e:
        return None
    parent = e.get("parent") or {}
    ops = e.get("ops") or []
    kinds = [_op_kind(o) for o in ops]
    return {
        "parent_revision": parent.get("revision"),
        "created": e.get("created", ""),
        "note": e.get("note", ""),
        "ops": len(ops),
        "counts": {
            k: kinds.count(k) for k in ("dynamic", "undynamic", "free", "unknown", "occupied") if k in kinds
        },
    }


def create_app(
    adapter: Adapter,
    maps_dir: str,
    footprint_path: str | None = None,
    wifi_iface: str = "wlp1s0",
    state_dir: str = "~/.amr",
    internet_probe: netcheck.InternetProbe | None = None,
) -> Flask:
    app = Flask(__name__, template_folder="templates", static_folder="static", static_url_path="/static")
    app.config["MAPS_DIR"] = os.path.expanduser(maps_dir)
    app.config["EVIDENCE_DIR"] = clog.evidence_dir(state_dir)
    fp = fpmod.load(footprint_path or fpmod.default_path())
    app.config["FOOTPRINT"] = fp
    _grids: dict[tuple[str, int], tuple[Any, gridio.Grid]] = {}

    def bundle(map_id: str, rev: int):
        key = (map_id, rev)
        if key not in _grids:
            _grids[key] = mb.load(app.config["MAPS_DIR"], map_id, rev)
        return _grids[key]

    # Revisions are immutable (a verified bundle's files never change), so a revision's masks
    # and edit record are cached like its grid.
    _extras: dict[tuple[str, int], dict] = {}

    def extras(map_id: str, rev: int) -> dict:
        key = (map_id, rev)
        if key not in _extras:
            bundle(map_id, rev)  # verified first
            rev_dir = mb.revision_dir(app.config["MAPS_DIR"], map_id, rev)
            dyn = load_dynamic(rev_dir)
            _extras[key] = {
                "keepout": load_keepout(rev_dir),
                "dynamic": dyn,
                "dynamic_cells": int((dyn.data >= 65).sum()) if dyn is not None else 0,
                "edits": mb.load_edits(rev_dir),
            }
        return _extras[key]

    # ---- pages -------------------------------------------------------------------

    @app.get("/")
    def index():
        return redirect("/status")

    @app.get("/status")
    def page_status():
        return render_template("status.html", page="status")

    @app.get("/monitor")
    def page_monitor():
        return render_template("monitor.html", page="monitor")

    @app.get("/io")
    def page_io():
        return render_template("io.html", page="io")

    @app.get("/alarms")
    def page_alarms():
        return render_template("alarms.html", page="alarms")

    @app.get("/params")
    def page_params():
        return render_template("params.html", page="params")

    @app.get("/commissioning")
    def page_commissioning():
        return render_template("commissioning.html", page="commissioning")

    @app.get("/manual")
    def page_manual():
        return render_template("manual.html", page="manual")

    @app.get("/maps")
    def page_maps():
        return render_template("maps.html", page="maps")

    @app.get("/editor")
    def page_editor():
        return render_template("editor.html", page="editor")

    @app.get("/review")
    def page_review():
        return render_template("review.html", page="maps")

    @app.get("/run")
    def page_run():
        return render_template("run.html", page="run")

    # ---- state -------------------------------------------------------------------

    @app.get("/api/state")
    def api_state():
        return jsonify(adapter.state())

    wifi_reader = wifi.WifiReader(wifi_iface)

    @app.get("/api/wifi")
    def api_wifi():
        return jsonify(wifi_reader.read())

    # the robot's own internet reachability (header indicator); probes at most once per 4 s
    internet = internet_probe or netcheck.InternetProbe()

    @app.get("/api/internet")
    def api_internet():
        return jsonify(internet.read())

    @app.get("/api/footprint")
    def api_footprint():
        return jsonify({"polygon": list(fp.polygon), "margin_m": fp.margin_m, "reach_m": fp.reach_m})

    # ---- maps ----------------------------------------------------------------------

    @app.get("/api/maps")
    def api_maps():
        maps_dir = app.config["MAPS_DIR"]
        out = []
        if os.path.isdir(maps_dir):
            for map_id in sorted(os.listdir(maps_dir)):
                revs = mb.list_revisions(maps_dir, map_id)
                if not revs:
                    continue
                items = []
                for r in revs:
                    try:
                        m, _ = bundle(map_id, r)
                        ex = extras(map_id, r)
                    except (mb.BundleError, gridio.GridError) as e:
                        items.append({"revision": r, "error": str(e)})
                        continue
                    items.append(
                        {
                            "revision": r,
                            "sha256": m.sha256,
                            "created": m.created,
                            "width": m.width,
                            "height": m.height,
                            "resolution": m.resolution,
                            "origin": m.origin,
                            "start": m.start,
                            "review": m.review,
                            "routes": store.list_routes(maps_dir, map_id),
                            "dynamic_cells": ex["dynamic_cells"],
                            "edits": _edits_summary(ex["edits"]),
                        }
                    )
                out.append({"map_id": map_id, "revisions": items})
        return jsonify(out)

    @app.get("/api/maps/<map_id>/<int:rev>")
    def api_map(map_id: str, rev: int):
        try:
            m, g = bundle(map_id, rev)
            ex = extras(map_id, rev)
        except (mb.BundleError, gridio.GridError) as e:
            return _result(False, str(e), 404)
        return jsonify(
            {
                "map_id": map_id,
                "revision": rev,
                "sha256": m.sha256,
                "width": g.width,
                "height": g.height,
                "resolution": g.meta.resolution,
                "origin": [g.meta.origin_x, g.meta.origin_y, g.meta.origin_yaw],
                "start": m.start,
                "routes": store.list_routes(app.config["MAPS_DIR"], map_id),
                "dynamic_cells": ex["dynamic_cells"],
                "edits": _edits_summary(ex["edits"]),
            }
        )

    @app.get("/api/maps/<map_id>/<int:rev>/image.png")
    def api_map_image(map_id: str, rev: int):
        try:
            _, g = bundle(map_id, rev)
        except mb.BundleError as e:
            return _result(False, str(e), 404)
        img = np.full(g.data.shape, 205, dtype=np.uint8)
        img[g.data == 0] = 254
        img[g.data >= 65] = 0
        png = encode_gray(np.flipud(img))  # row 0 = bottom in the grid, top in the image
        return Response(png, mimetype="image/png", headers={"Cache-Control": "max-age=3600"})

    @app.get("/api/maps/<map_id>/<int:rev>/dynamic.png")
    def api_map_dynamic(map_id: str, rev: int):
        """The dynamic-area mask in the map image's geometry: 255 = dynamic, 0 = not."""
        try:
            dyn = extras(map_id, rev)["dynamic"]
        except (mb.BundleError, gridio.GridError) as e:
            return _result(False, str(e), 404)
        if dyn is None:
            return _result(False, "this revision has no dynamic areas", 404)
        img = np.where(dyn.data >= 65, 255, 0).astype(np.uint8)
        png = encode_gray(np.flipud(img))
        return Response(png, mimetype="image/png", headers={"Cache-Control": "max-age=3600"})

    @app.post("/api/maps/<map_id>/<int:rev>/edit")
    def api_map_edit(map_id: str, rev: int):
        """Operator edits (mark/clear dynamic areas, paint free/unknown) -> a NEW revision
        derived from this one (dynamic-mapping plan §1.1). Nothing is modified in place:
        routes and missions on this revision keep meaning what they meant."""
        d = request.get_json(force=True, silent=True)
        if not isinstance(d, dict):
            return _result(False, "a JSON object {ops, note} is required", 400)
        note = d.get("note", "")
        if not isinstance(note, str):
            return _result(False, "note must be text", 400)
        try:
            bundle(map_id, rev)
            _path, new_rev, res = mb.derive_edit(app.config["MAPS_DIR"], map_id, rev, d.get("ops"), note)
        except mapedit.EditError as e:
            return _result(False, str(e), 422)
        except (mb.BundleError, gridio.GridError) as e:
            return _result(False, str(e), 404)
        return jsonify(
            {
                "ok": True,
                "map_id": map_id,
                "revision": new_rev,
                "parent_revision": rev,
                "dynamic_cells": res.dynamic_cells,
                "cells": res.cells,
                "message": f"{map_id} rev{new_rev} saved from rev{rev} ({len(res.cells)} edits)",
            }
        )

    # ---- routes --------------------------------------------------------------------

    def _validate_payload(map_id: str, rev: int, payload: dict):
        m, g = bundle(map_id, rev)
        route = Route.from_dict(payload)
        route.map.id, route.map.revision, route.map.sha256 = map_id, rev, m.sha256
        ex = extras(map_id, rev)
        return route, validate(route, m, g, fp, ex["keepout"], ex["dynamic"])

    def _compiled_dict(compiled) -> dict:
        return {
            "total_length_m": compiled.total_length_m,
            "total_turn_deg": float(np.degrees(compiled.total_turn_rad)),
            "closes": compiled.closes,
            "end": list(compiled.end),
            "steps": [
                {
                    "id": s.id,
                    "type": s.type,
                    "start": list(s.start),
                    "end": list(s.end),
                    "length_m": s.length_m,
                    "v_mps": s.v_mps,
                    "radius_m": s.radius_m,
                    "centre": list(s.centre) if s.centre is not None else None,
                    "signed_angle_deg": float(np.degrees(s.signed_angle_rad)),
                    "duration_est_s": s.duration_est_s,
                }
                for s in compiled.steps
            ],
        }

    @app.post("/api/maps/<map_id>/<int:rev>/routes/validate")
    def api_route_validate(map_id: str, rev: int):
        try:
            route, v = _validate_payload(map_id, rev, request.get_json(force=True) or {})
        except (mb.BundleError, gridio.GridError) as e:
            return _result(False, str(e), 404)
        except RouteError as e:
            return jsonify(
                {"ok": False, "issues": [{"code": "schema", "message": str(e), "step_id": e.step_id}]}
            ), 422
        if v.ok:
            adapter.publish_route_preview(v.compiled, route.frame_id)
        body = {"ok": v.ok, "issues": [i.to_dict() for i in v.issues], "route": route.to_dict()}
        if v.compiled is not None:
            body["compiled"] = _compiled_dict(v.compiled)
        return jsonify(body), (200 if v.ok else 422)

    @app.post("/api/maps/<map_id>/<int:rev>/routes/save")
    def api_route_save(map_id: str, rev: int):
        try:
            route, v = _validate_payload(map_id, rev, request.get_json(force=True) or {})
        except (mb.BundleError, gridio.GridError) as e:
            return _result(False, str(e), 404)
        except RouteError as e:
            return jsonify(
                {"ok": False, "issues": [{"code": "schema", "message": str(e), "step_id": e.step_id}]}
            ), 422
        if not v.ok:
            return jsonify({"ok": False, "issues": [i.to_dict() for i in v.issues]}), 422
        try:
            revision, path, sha = store.save_route(app.config["MAPS_DIR"], route)
        except store.StoreError as e:
            return _result(False, str(e), 409)
        return jsonify(
            {"ok": True, "route_id": route.route_id, "revision": revision, "sha256": sha, "path": path}
        )

    @app.get("/api/maps/<map_id>/<int:rev>/routes/<route_id>/<int:rrev>")
    def api_route_get(map_id: str, rev: int, route_id: str, rrev: int):
        try:
            route, sha = store.load_route(app.config["MAPS_DIR"], map_id, route_id, rrev)
            m, g = bundle(map_id, rev)
        except (store.StoreError, mb.BundleError) as e:
            return _result(False, str(e), 404)
        ex = extras(map_id, rev)
        v = validate(route, m, g, fp, ex["keepout"], ex["dynamic"])
        body = {
            "route": route.to_dict(),
            "sha256": sha,
            "ok": v.ok,
            "issues": [i.to_dict() for i in v.issues],
        }
        if v.compiled is not None:
            body["compiled"] = _compiled_dict(v.compiled)
            if v.ok:
                adapter.publish_route_preview(v.compiled, route.frame_id)
        return jsonify(body)

    # ---- missions ------------------------------------------------------------------

    @app.get("/api/missions")
    def api_missions():
        return jsonify(store.list_missions(app.config["MAPS_DIR"]))

    @app.post("/api/missions")
    def api_mission_create():
        d = request.get_json(force=True) or {}
        if not isinstance(d, dict):
            d = {}
        req = (d.get("map_id"), d.get("map_revision"), d.get("route_id"), d.get("route_revision"))
        if not (
            isinstance(req[0], str)
            and isinstance(req[2], str)
            and all(isinstance(x, int) and not isinstance(x, bool) for x in (req[1], req[3]))
            and isinstance(d.get("mission_id") or "", str)
        ):
            return _result(False, "map_id, map_revision, route_id, route_revision required", 400)
        map_id, rev, route_id, rrev = req
        try:
            m, g = bundle(map_id, rev)
            route, sha = store.load_route(app.config["MAPS_DIR"], map_id, route_id, rrev)
        except RouteError as e:
            return _result(False, f"stored route unreadable: {e}", 422)
        except (store.StoreError, mb.BundleError, gridio.GridError) as e:
            return _result(False, str(e), 404)
        ex = extras(map_id, rev)
        v = validate(route, m, g, fp, ex["keepout"], ex["dynamic"])
        if not v.ok:
            return jsonify({"ok": False, "issues": [i.to_dict() for i in v.issues]}), 422
        # route ids/revisions are per map; missions share one directory: the default id names the map
        mission_id = d.get("mission_id") or f"{map_id}_rev{rev}_{route_id}_rev{rrev}"
        try:
            path = store.save_mission(
                app.config["MAPS_DIR"], mission_id, map_id, rev, m.sha256, route_id, rrev, sha
            )
        except store.StoreConflict as e:
            return _result(False, str(e), 409)
        except store.StoreError as e:
            return _result(False, str(e), 400)
        return jsonify({"ok": True, "mission_id": mission_id, "path": path})

    # ---- coordinator calls (services on the robot; the browser never commands wheels) ----

    def _call(fn, *args):
        ok, msg = fn(*args)
        return _result(ok, msg)

    # ---- diagnostics (unified plan §7): owner snapshots, never a second CAN/Modbus client ----

    @app.get("/api/diagnostics")
    def api_diagnostics():
        return jsonify(adapter.diagnostics() if hasattr(adapter, "diagnostics") else {})

    @app.get("/api/io")
    def api_io():
        return jsonify(adapter.io_image() if hasattr(adapter, "io_image") else None)

    @app.get("/api/events")
    def api_events():
        try:
            since = int(request.args.get("since", "0"))
        except ValueError:
            since = 0
        return jsonify(adapter.events(since) if hasattr(adapter, "events") else [])

    @app.get("/api/config")
    def api_config():
        """Effective configuration, read-only, with provenance: the strict vehicle
        profile (config.describe(), the same rows the legacy page showed) plus what
        the owners report they are actually running (from /diagnostics)."""
        import amr_base.agv_repo  # noqa: F401, PLC0415

        import config  # noqa: PLC0415

        diag = adapter.diagnostics() if hasattr(adapter, "diagnostics") else {}
        runtime = {k: v["values"] for k, v in diag.items() if k in ("can/bus", "imu/mls")}
        return jsonify(
            {
                "profile": config.PROFILE_NAME,
                "profile_path": config.PROFILE_PATH_LOADED,
                "sections": config.describe(),
                "runtime": runtime,
                "note": "profile = the vehicle profile; runtime = what the owning node reports it uses",
            }
        )

    # ---- commissioning (unified plan §7.2) ----

    @app.get("/api/commissioning")
    def api_commissioning():
        return jsonify(adapter.commissioning() if hasattr(adapter, "commissioning") else None)

    @app.post("/api/commissioning/plan")
    def api_commissioning_plan():
        d = request.get_json(force=True) or {}
        plan = d.get("plan")
        if not isinstance(plan, dict):
            return _result(False, "plan object required", 400)
        try:
            text = json.dumps(plan)
        except (TypeError, ValueError):
            return _result(False, "plan is not JSON-serialisable", 400)
        if len(text) > 20000:
            return _result(False, "plan too large", 413)
        ok, msg, planned = adapter.commissioning_plan(text)
        body = {"ok": ok, "message": msg}
        if ok and planned:
            body["planned"] = json.loads(planned)
        return jsonify(body), (200 if ok else 409)

    @app.post("/api/commissioning/clear")
    def api_commissioning_clear():
        return _call(adapter.commissioning_clear)

    @app.get("/api/commissioning/capabilities")
    def api_commissioning_capabilities():
        """What the blind-run form may offer: the profile's caps, and whether the drive
        owner reports profile position available right now (locked until the decision
        to run pp on this motor is recorded in pp.vendor_ref). Advisory: the commissioning node and
        the drive owner enforce the same rules on their own."""
        import amr_base.agv_repo  # noqa: F401, PLC0415

        import config  # noqa: PLC0415

        st = adapter.pp_status() if hasattr(adapter, "pp_status") else None
        fresh = st is not None and float(st.get("age_s", 1e9)) <= 1.0
        if not fresh:
            pp_ok, pp_reason = False, "no fresh pp status from the drive owner"
        else:
            pp_ok, pp_reason = bool(st.get("available")), str(st.get("reason", ""))
        return jsonify(
            {
                "max_speed_mps": config.BLIND_MAX_SPEED_MPS,
                "pp_max_speed_mps": config.PP_MAX_SPEED_MPS,
                "max_distance_m": config.BLIND_MAX_DISTANCE_M,
                "min_arc_radius_m": config.TRACK_M / 2.0,
                "track_m": config.TRACK_M,
                "decel_mps2": config.BLIND_ACCEL_RPM_S * config.MPS_PER_RPM,
                "field_check_above_mps": 0.4,
                "pp_available": pp_ok,
                "pp_reason": "" if pp_ok else pp_reason,
                "pp_status": st,
            }
        )

    @app.get("/api/commissioning/history")
    def api_commissioning_history():
        return jsonify(clog.history(app.config["EVIDENCE_DIR"]))

    @app.post("/api/commissioning/measurement")
    def api_commissioning_measurement():
        d = request.get_json(force=True) or {}
        if not isinstance(d, dict):
            return _result(False, "object required", 400)
        try:
            row = clog.save_measurement(app.config["EVIDENCE_DIR"], d.get("run"), d)
        except clog.LogError as e:
            return _result(False, str(e), 400)
        except OSError as e:
            return _result(False, f"measurement not stored: {e}", 500)
        return jsonify({"ok": True, "message": "measurement stored", "row": row})

    # ---- live view (unified plan §6.4): bounded rates, encoded once per snapshot ----

    @app.get("/api/live/map")
    def api_live_map():
        meta = adapter.live.map_meta() if hasattr(adapter, "live") else None
        if meta is None:
            return jsonify({"snapshot": 0, "available": False})
        return jsonify(dict(meta, available=True))

    @app.get("/api/live/map.png")
    def api_live_map_png():
        try:
            snap = int(request.args.get("snapshot", "0"))
        except ValueError:
            return _result(False, "snapshot must be an integer", 400)
        png = adapter.live.map_png(snap) if hasattr(adapter, "live") else None
        if png is None:
            return _result(False, "no such live snapshot (a newer grid exists)", 404)
        return Response(png, mimetype="image/png", headers={"Cache-Control": "no-store"})

    @app.get("/api/live/pose")
    def api_live_pose():
        if not hasattr(adapter, "live"):
            return jsonify({"generation": 0, "pose": None, "scan": None})
        return jsonify(adapter.live.pose_scan())

    # ---- supervisor operations: accepted (202) vs completed (poll /api/operations/<id>) ----

    def _accepted(accepted: bool, operation_id: str, message: str):
        if accepted:
            return jsonify({"ok": True, "operation_id": operation_id, "message": message}), 202
        status = 503 if "unavailable" in message else 409
        return jsonify({"ok": False, "operation_id": operation_id, "message": message}), status

    @app.post("/api/mode")
    def api_mode():
        d = request.get_json(force=True) or {}
        target = {"idle": MODE_IDLE, "navigation": MODE_NAVIGATION}.get(str(d.get("target", "")).lower())
        if target is None:
            return _result(False, "target must be idle or navigation", 400)
        try:
            rev = int(d.get("map_revision", 0))
        except (TypeError, ValueError):
            return _result(False, "map_revision must be an integer", 400)
        if target == MODE_NAVIGATION and (not d.get("map_id") or rev <= 0):
            return _result(False, "navigation needs map_id and an exact map_revision", 422)
        rid = str(d.get("request_id") or uuid.uuid4().hex)
        return _accepted(*adapter.request_mode(target, str(d.get("map_id", "")), rev, rid))

    @app.post("/api/survey/<op>")
    def api_survey(op: str):
        if op not in SURVEY_OPS:
            return _result(False, "unknown survey operation", 404)
        d = request.get_json(force=True) or {}
        rid = str(d.get("request_id") or uuid.uuid4().hex)
        desc = str(d.get("description", d.get("note", "")))
        return _accepted(*adapter.survey_request(SURVEY_OPS[op], str(d.get("map_id", "")), desc, rid))

    # preset survey moves (press once): validated again by survey_move_node, which also checks
    # the MANUAL authority; a refused move answers 409 with the node's reason
    @app.post("/api/survey_move")
    def api_survey_move():
        d = request.get_json(force=True) or {}
        kind = str(d.get("kind", ""))
        try:
            value = float(d.get("value"))
        except (TypeError, ValueError):
            return _result(False, "value must be a number", 422)
        if kind not in ("straight", "rotate") or not math.isfinite(value):
            return _result(False, "kind must be straight or rotate, value a finite number", 422)
        return _result(*adapter.survey_move(kind, value))

    @app.post("/api/survey_move/stop")
    def api_survey_move_stop():
        return _result(*adapter.survey_move_stop())

    @app.get("/api/operations/<operation_id>")
    def api_operation(operation_id: str):
        op = adapter.get_operation(operation_id)
        if op is None:
            return _result(False, "no such operation", 404)
        return jsonify(op)

    @app.post("/api/recover")
    def api_recover():
        return _call(adapter.recover)

    # ---- manual jog (unified plan §6.3): the only endpoint that publishes a velocity ----

    sessions = jog.JogSessions(time.monotonic)
    jog_lock = threading.Lock()
    app.config["JOG"] = sessions

    def _jog_error(e: jog.JogError):
        return jsonify({"ok": False, "message": str(e)}), e.status

    @app.post("/api/manual/press")
    def api_manual_press():
        d = request.get_json(force=True) or {}
        owner = str(d.get("owner", ""))[:64]
        if not owner:
            return _result(False, "owner token required", 400)
        inst, gen = adapter.supervisor_identity()
        if hasattr(adapter, "manual_allowed"):
            ok, why = adapter.manual_allowed()
            if not ok:
                return _result(False, why, 409)
        with jog_lock:
            try:
                s = sessions.press(owner, inst, gen)
            except jog.JogError as e:
                return _jog_error(e)
        return jsonify(
            {
                "ok": True,
                "session": s.id,
                "ticket": s.ticket,
                "generation": gen,
                "v_max": jog.V_MAX,
                "w_max": jog.W_MAX,
            }
        )

    @app.post("/api/manual/refresh")
    def api_manual_refresh():
        d = request.get_json(force=True) or {}
        inst, gen = adapter.supervisor_identity()
        with jog_lock:
            try:
                cmd = sessions.refresh(
                    str(d.get("session", "")),
                    str(d.get("ticket", "")),
                    d.get("seq"),
                    d.get("v"),
                    d.get("w"),
                    inst,
                    gen,
                )
            except jog.JogError as e:
                return _jog_error(e)
            adapter.manual_publish(cmd)  # exactly one command per accepted request, under the lock
        return jsonify(
            {"ok": True, "ticket": cmd.ticket, "valid_for_s": cmd.valid_for_s, "v": cmd.v, "w": cmd.w}
        )

    @app.post("/api/manual/release")
    def api_manual_release():
        d = request.get_json(force=True) or {}
        inst, gen = adapter.supervisor_identity()
        with jog_lock:
            had = sessions.release(str(d.get("session", "")))
            # The session is dead before this goes out. valid_for_s = 0 is a revocation at
            # the mux (gating.ManualIntake): terminal for the session whatever its seq, so
            # neither this zero nor a delayed refresh can lose to the sequence filter (R02).
            # Published under the lock so it cannot interleave with a refresh publication.
            adapter.manual_publish(
                jog.Command(inst, gen, str(d.get("session", "")), 0, 0.0, 0.0, 0.0, "", 0.0)
            )
        return jsonify({"ok": True, "released": had})

    @app.post("/api/stop")
    def api_stop():
        """Revoke this browser's manual session. During a route use pause/abort."""
        inst, gen = adapter.supervisor_identity()
        with jog_lock:
            sessions.invalidate_all()
            # empty session + valid_for_s 0: the mux drops whatever it holds
            adapter.manual_publish(jog.Command(inst, gen, "", 0, 0.0, 0.0, 0.0, "", 0.0))
        return jsonify({"ok": True, "message": "manual sessions revoked"})

    @app.post("/api/localization/confirm")
    def api_loc_confirm():
        return _call(adapter.localization_confirm)

    @app.post("/api/localization/reset")
    def api_loc_reset():
        return _call(adapter.localization_reset)

    @app.post("/api/localization/initialpose")
    def api_loc_initialpose():
        d = request.get_json(force=True) or {}
        try:
            x, y, yaw = float(d["x_m"]), float(d["y_m"]), float(d["yaw_rad"])
            map_id, map_rev, gen = str(d["map_id"]), int(d["map_revision"]), int(d["generation"])
        except (KeyError, ValueError, TypeError):
            return _result(False, "x_m, y_m, yaw_rad, map_id, map_revision, generation required", 400)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return _result(False, "x_m, y_m, yaw_rad must be finite", 400)
        # the adapter re-checks the identity against the live ModeState right before publishing (R12)
        return _call(adapter.set_initial_pose, x, y, yaw, map_id, map_rev, gen, str(d.get("sha256", "")))

    @app.post("/api/mission/run")
    def api_mission_run():
        d = request.get_json(force=True) or {}
        return _call(adapter.run_mission, str(d.get("mission_id", "")))

    @app.post("/api/mission/pause")
    def api_mission_pause():
        return _call(adapter.pause)

    @app.post("/api/mission/abort")
    def api_mission_abort():
        return _call(adapter.abort)

    @app.post("/api/mission/resume")
    def api_mission_resume():
        return _call(adapter.prepare_resume)

    @app.post("/api/mission/ack")
    def api_mission_ack():
        return _call(adapter.ack_fault)

    return app
