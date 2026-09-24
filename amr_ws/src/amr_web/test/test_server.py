"""Flask API against a stub adapter and a temp maps dir (spec §8 "Coordinates", T6)."""

import math
import struct
import zlib

import numpy as np
import pytest
from amr_maps.generate_sim_factory import build
from amr_maps.grid import Grid, write
from amr_web.png import encode_gray
from amr_web.server import create_app

from amr_mission import map_bundle as mb

FOOTPRINT_YAML = "polygon: [[-0.5, -0.35], [1.1, -0.35], [1.1, 0.35], [-0.5, 0.35]]\nmargin_m: 0.20\n"


class Stub:
    def __init__(self):
        self.calls = []
        self.previews = []
        self.run_state = None
        self.identity = ("inst-1", 3)
        self.published = []
        self.ops = {}

    def state(self):
        return {"mapping": None, "localization": {"state_name": "READY"}, "run": self.run_state}

    # supervisor surface (unified plan §6.2)
    def supervisor_identity(self):
        return self.identity

    def request_mode(self, target, map_id, rev, request_id):
        self.calls.append(("request_mode", (target, map_id, rev, request_id)))
        self.ops["op-mode"] = {"operation_id": "op-mode", "status": 0, "status_name": "PENDING"}
        return True, "op-mode", "accepted"

    def survey_request(self, operation, map_id, description, request_id):
        self.calls.append(("survey_request", (operation, map_id, description, request_id)))
        return True, "op-survey", "accepted"

    def get_operation(self, oid):
        return self.ops.get(oid)

    def survey_move(self, kind, value):
        self.calls.append(("survey_move", (kind, value)))
        return (False, "refused: selector is not MANUAL") if value == 99 else (True, "forward 1.00 m")

    def survey_move_stop(self):
        self.calls.append(("survey_move_stop", ()))
        return True, "stopped"

    def manual_publish(self, cmd):
        self.published.append(cmd)

    def _rec(self, name):
        def fn(*a):
            self.calls.append((name, a))
            return True, f"{name} ok"

        return fn

    def __getattr__(self, name):
        if name == "publish_route_preview":
            return lambda compiled, frame: self.previews.append((compiled, frame))
        return self._rec(name)


@pytest.fixture()
def env(tmp_path):
    maps = tmp_path / "maps"
    maps.mkdir()
    # world as bundle, plus an unknown patch and a keepout mask for the validation paths
    g = build()
    stage = mb.staging_dir(str(maps), "sim_factory", 1)
    m = mb.Manifest(
        "sim_factory",
        1,
        mb.now_iso(),
        "map",
        0.05,
        [-3.0, -10.0, 0.0],
        g.width,
        g.height,
        {"x_m": 0, "y_m": 0, "yaw_rad": 0, "description": "mark"},
        {"dx_m": 0, "dy_m": 0, "dyaw_rad": 0, "note": "t"},
    )
    mb.stage_bundle(stage, g, None, m, required_posegraph=False)
    mb.verify(stage)
    rev_dir = mb.publish(stage, str(maps), "sim_factory", 1)
    fp = tmp_path / "footprint.yaml"
    fp.write_text(FOOTPRINT_YAML)
    stub = Stub()
    app = create_app(stub, str(maps), str(fp))
    app.config["TESTING"] = True
    return app.test_client(), stub, str(maps), rev_dir, mb.verify(rev_dir)


def route_payload(steps, start=(0.0, 0.0, 0.0), repeat=1, route_id="r1"):
    return {
        "schema_version": 1,
        "route_id": route_id,
        "revision": 0,
        "map": {"id": "x", "revision": 9, "sha256": "ignored"},
        "frame_id": "map",
        "start": {"x_m": start[0], "y_m": start[1], "yaw_deg": start[2]},
        "limits": {"linear_mps": 0.3},
        "steps": steps,
        "repeat_count": repeat,
    }


def S(sid, x, y):
    return {"id": sid, "type": "straight", "to": {"x_m": x, "y_m": y}}


def R(sid, d, a):
    return {"id": sid, "type": "rotate", "direction": d, "angle_deg": a}


def test_maps_index_and_image(env):
    client, stub, maps, rev_dir, manifest = env
    r = client.get("/api/maps")
    assert r.status_code == 200 and r.json[0]["map_id"] == "sim_factory"
    assert r.json[0]["revisions"][0]["sha256"] == manifest.sha256
    r = client.get("/api/maps/sim_factory/1")
    assert r.json["width"] == 600 and r.json["origin"] == [-3.0, -10.0, 0.0]
    r = client.get("/api/maps/sim_factory/1/image.png")
    assert r.status_code == 200 and r.data[:8] == b"\x89PNG\r\n\x1a\n"
    w, h = struct.unpack(">II", r.data[16:24])
    assert (w, h) == (600, 400)
    assert client.get("/api/maps/nope/1").status_code == 404


def test_png_encoder_round_trip():
    img = (np.arange(12, dtype=np.uint8) * 20).reshape(3, 4)
    png = encode_gray(img)
    idat_len = struct.unpack(">I", png[33:37])[0]
    raw = zlib.decompress(png[41 : 41 + idat_len])
    rows = [raw[i * 5 + 1 : i * 5 + 5] for i in range(3)]
    assert rows == [img[r].tobytes() for r in range(3)]


def test_validate_save_load_and_mission(env):
    client, stub, maps, rev_dir, manifest = env
    good = route_payload([S("s1", 15.0, 0.0), R("s2", "ccw", 180), S("s3", 2.0, 0.0)])
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=good)
    assert r.status_code == 200 and r.json["ok"] and r.json["compiled"]["steps"][1]["signed_angle_deg"] == 180
    assert r.json["route"]["map"]["sha256"] == manifest.sha256  # server binds the route to the loaded bundle
    assert len(stub.previews) == 1
    r = client.post("/api/maps/sim_factory/1/routes/save", json=good)
    assert r.status_code == 200 and r.json["revision"] == 1
    r = client.post("/api/maps/sim_factory/1/routes/save", json=good)
    assert r.json["revision"] == 2
    r = client.get("/api/maps/sim_factory/1/routes/r1/2")
    assert r.status_code == 200 and r.json["ok"] and r.json["route"]["revision"] == 2
    r = client.post(
        "/api/missions",
        json={"map_id": "sim_factory", "map_revision": 1, "route_id": "r1", "route_revision": 2},
    )
    assert r.status_code == 200 and r.json["mission_id"] == "sim_factory_rev1_r1_rev2"
    r = client.get("/api/missions")
    assert r.json[0]["route"]["revision"] == 2 and r.json[0]["map"]["sha256"] == manifest.sha256
    assert (
        client.post(
            "/api/missions",
            json={"map_id": "sim_factory", "map_revision": 1, "route_id": "r1", "route_revision": 7},
        ).status_code
        == 404
    )
    assert client.post("/api/missions", json={}).status_code == 400


def test_invalid_routes_are_422_with_step_ids(env):
    client, stub, maps, rev_dir, manifest = env
    r = client.post(
        "/api/maps/sim_factory/1/routes/validate",
        json=route_payload([S("s1", 5.0, 0.0), R("s2", "ccw", 90), S("s3", 5.0, 4.0)]),
    )
    assert r.status_code == 422 and not r.json["ok"]
    assert any(i["step_id"] == "s3" and i["code"] == "clearance" for i in r.json["issues"])
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=route_payload([S("s1", 3.0, 0.5)]))
    assert (
        r.status_code == 422
        and r.json["issues"][0]["step_id"] == "s1"
        and "turn" in r.json["issues"][0]["message"]
    )
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=route_payload([R("t", "cw", 60)]))
    assert r.status_code == 422
    r = client.post("/api/maps/sim_factory/1/routes/save", json=route_payload([S("s1", 3.0, 0.5)]))
    assert r.status_code == 422
    r = client.post("/api/maps/sim_factory/1/routes/validate", json={"schema_version": 3})
    assert r.status_code == 422 and r.json["issues"][0]["code"] == "schema"
    assert stub.previews == []  # nothing invalid is previewed


def test_keepout_mask_is_honoured(env):
    client, stub, maps, rev_dir, manifest = env
    g = build()
    ko = Grid(np.zeros(g.data.shape, dtype=np.int8), g.meta)
    r0, c0 = g.world_to_cell(6.0, 0.0)
    ko.data[r0 - 2 : r0 + 2, c0 - 2 : c0 + 2] = 100
    # a keepout is navigation content: it goes into a NEW revision whose identity covers it (Q14)
    write(ko, f"{maps}/keepout")  # the yaml names its image by stem: keepout.pgm, as listed
    _, rev = mb.derive_revision(
        maps, "sim_factory", 1, {"keepout.yaml": f"{maps}/keepout.yaml", "keepout.pgm": f"{maps}/keepout.pgm"}
    )
    r = client.post(f"/api/maps/sim_factory/{rev}/routes/validate", json=route_payload([S("s1", 12.0, 0.0)]))
    assert r.status_code == 422 and any("keepout" in i["message"] for i in r.json["issues"])
    # dropped into the published revision unlisted, it is not a usable revision at all
    write(ko, f"{rev_dir}/keepout")
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=route_payload([S("s1", 12.0, 0.0)]))
    assert r.status_code == 404


def test_coordinator_endpoints_delegate_and_never_touch_wheels(env):
    client, stub, maps, rev_dir, manifest = env
    for path, body, name in (
        ("/api/localization/confirm", {}, "localization_confirm"),
        (
            "/api/localization/initialpose",
            {"x_m": 1, "y_m": 2, "yaw_rad": 0.5, "map_id": "a", "map_revision": 2, "generation": 7},
            "set_initial_pose",
        ),
        ("/api/mission/run", {"mission_id": "m"}, "run_mission"),
        ("/api/mission/pause", {}, "pause"),
        ("/api/mission/resume", {}, "prepare_resume"),
        ("/api/mission/abort", {}, "abort"),
        ("/api/mission/ack", {}, "ack_fault"),
    ):
        r = client.post(path, json=body)
        assert r.status_code == 200 and r.json["ok"], path
        assert stub.calls[-1][0] == name
    assert stub.calls[1] == ("set_initial_pose", (1.0, 2.0, 0.5, "a", 2, 7, ""))
    # R12: a pose without the map identity it was drawn on is not accepted at all
    no_map = {"x_m": 1, "y_m": 2, "yaw_rad": 0}
    assert client.post("/api/localization/initialpose", json=no_map).status_code == 400
    assert client.post("/api/localization/initialpose", json={"x_m": "no"}).status_code == 400
    rules = [str(r.rule) for r in client.application.url_map.iter_rules()]
    # the one velocity-publishing route is the held manual refresh; nothing else
    assert not any("cmd_vel" in r or "drive" in r or "jog" in r for r in rules)
    assert (
        sum("/api/manual" in r for r in rules) == 3
    )  # press, refresh, release (the /manual page is not an API)


def test_mode_and_survey_are_asynchronous_operations(env):
    client, stub, *_ = env
    r = client.post(
        "/api/mode", json={"target": "navigation", "map_id": "m", "map_revision": 2, "request_id": "r1"}
    )
    assert r.status_code == 202 and r.json["operation_id"] == "op-mode"
    assert stub.calls[-1] == ("request_mode", (3, "m", 2, "r1"))
    assert client.get("/api/operations/op-mode").json["status_name"] == "PENDING"
    assert client.get("/api/operations/nope").status_code == 404
    assert client.post("/api/mode", json={"target": "navigation", "map_id": "m"}).status_code == 422
    assert client.post("/api/mode", json={"target": "fly"}).status_code == 400
    r = client.post("/api/survey/start", json={"map_id": "a", "description": "mark"})
    assert (
        r.status_code == 202
        and stub.calls[-1][0] == "survey_request"
        and stub.calls[-1][1][:3] == (0, "a", "mark")
    )
    r = client.post("/api/survey/save", json={"note": "seams ok"})
    assert r.status_code == 202 and stub.calls[-1][1][:3] == (2, "", "seams ok")
    assert client.post("/api/survey/bogus", json={}).status_code == 404


def test_state_and_footprint(env):
    client, stub, maps, rev_dir, manifest = env
    assert client.get("/api/state").json["localization"]["state_name"] == "READY"
    fp = client.get("/api/footprint").json
    assert (
        fp["margin_m"] == 0.2
        and len(fp["polygon"]) == 4
        and fp["reach_m"] == pytest.approx((1.1**2 + 0.35**2) ** 0.5)
    )
    assert client.get("/").status_code == 302
    for page in ("/maps", "/editor", "/run"):
        assert client.get(page).status_code == 200


# ---- R21/R22/R23: route schema errors, mission identity, concurrent saves ----------------


def _publish_bundle(maps: str, map_id: str) -> mb.Manifest:
    g = build()
    stage = mb.staging_dir(maps, map_id, 1)
    m = mb.Manifest(
        map_id,
        1,
        mb.now_iso(),
        "map",
        0.05,
        [-3.0, -10.0, 0.0],
        g.width,
        g.height,
        {"x_m": 0, "y_m": 0, "yaw_rad": 0, "description": "mark"},
        {"dx_m": 0, "dy_m": 0, "dyaw_rad": 0, "note": "t"},
    )
    mb.stage_bundle(stage, g, None, m, required_posegraph=False)
    return mb.verify(mb.publish(stage, maps, map_id, 1))


def test_r21_malformed_route_payloads_are_422_and_never_stored(env):
    from amr_navigation import store

    client, stub, maps, rev_dir, manifest = env
    good = route_payload([S("s1", 5.0, 0.0)])
    bad = []
    for key, value in (
        ("repeat_count", 2.5),
        ("repeat_count", "2"),
        ("repeat_count", 101),
        ("limits", {"linear_mps": 0}),
        ("limits", {"linear_mps": "fast"}),
        ("limits", {"bogus": 1.0}),
        ("start", None),
        ("start", {"x_m": 0.0, "y_m": 0.0}),
        ("steps", [{"id": "s1", "type": "straight", "to": {"x_m": "5", "y_m": 0}}]),
        ("steps", [{"id": "s1", "type": "rotate", "direction": "cw", "angle_deg": 90.5}]),
        ("steps", [S("s", 5.0, 0.0)] * 1000),
    ):
        bad.append({**good, key: value})
    bad.append({**good, "start": {"x_m": 1e6, "y_m": 0.0, "yaw_deg": 0.0}, "steps": [S("s1", 1e6 + 1, 0.0)]})
    bad.append([1, 2])
    for payload in bad:
        for path in ("validate", "save"):
            r = client.post(f"/api/maps/sim_factory/1/routes/{path}", json=payload)
            assert r.status_code in (400, 422), (path, payload if not isinstance(payload, dict) else "")
            assert not r.json["ok"]
    assert store.list_routes(maps, "sim_factory") == {}
    assert stub.previews == []


def test_r22_same_route_name_on_two_maps_keeps_both_missions(env):
    from amr_navigation import store

    client, stub, maps, rev_dir, manifest = env
    _publish_bundle(maps, "other_map")
    payload = route_payload([S("s1", 5.0, 0.0)])
    ids = []
    for map_id in ("sim_factory", "other_map"):
        r = client.post(f"/api/maps/{map_id}/1/routes/save", json=payload)
        assert r.status_code == 200 and r.json["revision"] == 1
        r = client.post(
            "/api/missions", json={"map_id": map_id, "map_revision": 1, "route_id": "r1", "route_revision": 1}
        )
        assert r.status_code == 200, r.json
        ids.append(r.json["mission_id"])
    assert len(set(ids)) == 2 and all(
        map_id in i for map_id, i in zip(("sim_factory", "other_map"), ids, strict=True)
    )
    assert {m["map"]["id"] for m in store.list_missions(maps)} == {"sim_factory", "other_map"}
    # idempotent retry of the same references
    body = {"map_id": "sim_factory", "map_revision": 1, "route_id": "r1", "route_revision": 1}
    assert client.post("/api/missions", json=body).status_code == 200
    # an explicit id already used for other references fails and leaves the original bytes alone
    path = f"{maps}/missions/{ids[0]}.yaml"
    before = open(path, "rb").read()
    r = client.post(
        "/api/missions",
        json={
            "mission_id": ids[0],
            "map_id": "other_map",
            "map_revision": 1,
            "route_id": "r1",
            "route_revision": 1,
        },
    )
    assert r.status_code == 409 and not r.json["ok"]
    assert open(path, "rb").read() == before
    for bad in (
        {**body, "map_revision": "1"},
        {**body, "route_revision": 1.5},
        {**body, "map_revision": True},
        {**body, "mission_id": 5},
    ):
        assert client.post("/api/missions", json=bad).status_code == 400


def test_r23_concurrent_route_saves_get_distinct_revisions(env):
    import hashlib
    import threading

    client, stub, maps, rev_dir, manifest = env
    app = client.application
    n = 6
    barrier = threading.Barrier(n)
    out, errors = [], []

    def save(k):
        try:
            c = app.test_client()
            payload = route_payload([S("s1", 4.0 + k, 0.0)])
            barrier.wait(timeout=10)
            r = c.post("/api/maps/sim_factory/1/routes/save", json=payload)
            out.append((k, r.status_code, r.json))
        except Exception as e:  # noqa: BLE001
            errors.append(e)

    threads = [threading.Thread(target=save, args=(k,)) for k in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=60)
    assert not errors and len(out) == n and all(code == 200 for _, code, _ in out)
    assert sorted(j["revision"] for _, _, j in out) == list(range(1, n + 1))
    for k, _, j in out:
        data = open(j["path"], "rb").read()
        assert hashlib.sha256(data).hexdigest() == j["sha256"] and f"x_m: {4.0 + k}".encode() in data


def test_pages_fonts_and_style_guards(env):
    """web-style W8: every page renders, fonts are self-hosted, and the style rules that
    are mechanical (no external URLs, light only, flat, square) hold in the source."""
    import pathlib  # noqa: PLC0415
    import re  # noqa: PLC0415

    client = env[0]
    for page in (
        "/status",
        "/manual",
        "/maps",
        "/editor",
        "/review",
        "/run",
        "/monitor",
        "/io",
        "/alarms",
        "/params",
        "/commissioning",
    ):
        r = client.get(page)
        assert r.status_code == 200, page
        assert b'id="telemetry"' in r.data and b"/static/amr.css" in r.data, page
    r = client.get("/static/fonts/plex-mono-400.woff2")
    assert r.status_code == 200 and r.data[:4] == b"wOF2"
    root = pathlib.Path(__file__).resolve().parents[1] / "amr_web"
    sources = [*root.glob("static/*.css"), *root.glob("static/*.js"), *root.glob("templates/*.html")]
    assert len(sources) > 15
    for f in sources:
        text = f.read_text()
        assert not re.search(r"https?://", text), f"external URL in {f.name}"
        assert not re.search(r"@media[^{]*prefers-color-scheme", text), f"dark-mode query in {f.name}"
        assert "box-shadow" not in text, f.name
        assert "gradient(" not in text, f.name
        for radius in re.findall(r"border-radius\s*:\s*([^;}]+)", text):
            assert radius.strip() in ("50%", "2px", "0"), f"{f.name}: border-radius {radius}"


def test_every_element_id_a_page_script_uses_exists_on_that_page(env):
    """web-style §6: no JS runtime in CI - an id renamed in a template must not silently
    break its handler. Collects $('id') / getElementById('id') / tile|tiles('id') from the
    page's inline and linked scripts and checks the rendered page defines each id."""
    import pathlib  # noqa: PLC0415
    import re  # noqa: PLC0415

    client = env[0]
    static = pathlib.Path(__file__).resolve().parents[1] / "amr_web" / "static"
    pat = re.compile(r"""(?:\$|getElementById|tiles?)\(\s*'([A-Za-z][\w-]*)'""")
    for page in (
        "/status",
        "/manual",
        "/maps",
        "/editor",
        "/review",
        "/run",
        "/monitor",
        "/io",
        "/alarms",
        "/params",
        "/commissioning",
    ):
        html = client.get(page).data.decode()
        ids = set(re.findall(r'\bid="([^"]+)"', html))
        code = "\n".join(re.findall(r"<script>(.*?)</script>", html, re.S))
        for src in re.findall(r'<script src="/static/([\w.]+)"', html):
            code += (static / src).read_text()  # amr.js too: the rail ids must exist on every page
        missing = {i for i in pat.findall(code) if i not in ids}
        assert not missing, f"{page}: scripts use ids not on the page: {sorted(missing)}"


def test_wifi_parse_and_bars(tmp_path):
    from amr_web import wifi

    proc = (
        "Inter-| sta-|   Quality        | Discarded packets | Missed | WE\n"
        " face | tus | link level noise |  nwid  crypt frag retry misc | beacon | 22\n"
        "wlp1s0: 0000   70.  -40.  -256        0      0      0      0     83        0\n"
    )
    assert wifi.parse_proc(proc, "wlp1s0") == -40.0
    assert wifi.parse_proc(proc, "wlan9") is None
    assert [wifi.bars_for(d) for d in (-40, -60, -70, -80, -90, None)] == [4, 3, 2, 1, 0, 0]

    p = tmp_path / "wireless"
    p.write_text(proc)
    r = wifi.WifiReader("wlp1s0", proc_path=str(p))
    r._ssid, r._ssid_t = "agv_field", float("inf")  # skip nmcli in the test
    out = r.read()
    assert (out["connected"], out["dbm"], out["bars"], out["ssid"]) == (True, -40.0, 4, "agv_field")
    p.write_text(proc.splitlines()[0] + "\n")
    assert wifi.WifiReader("wlp1s0", proc_path=str(p)).read()["connected"] is False
    assert wifi.WifiReader("wlp1s0", proc_path=str(tmp_path / "missing")).read()["bars"] == 0


def test_wifi_endpoint(env):
    client = env[0]
    r = client.get("/api/wifi")
    assert r.status_code == 200
    body = r.get_json()
    assert set(body) >= {"iface", "connected", "ssid", "dbm", "bars"}


def test_route_without_limits_is_stored_at_055_037_and_an_explicit_030_survives(env):
    client, stub, maps, rev_dir, manifest = env
    body = route_payload([S("s1", 3.0, 0.0)], route_id="dflt")
    del body["limits"]
    r = client.post("/api/maps/sim_factory/1/routes/save", json=body)
    assert r.status_code == 200
    r = client.get("/api/maps/sim_factory/1/routes/dflt/1")
    assert (
        r.json["route"]["limits"]["linear_mps"] == 0.55 and r.json["route"]["limits"]["angular_rad_s"] == 0.37
    )
    assert r.json["route"]["limits"]["arc_linear_mps"] == 0.40
    r = client.post(
        "/api/maps/sim_factory/1/routes/save", json=route_payload([S("s1", 3.0, 0.0)], route_id="slow")
    )
    assert r.status_code == 200
    r = client.get("/api/maps/sim_factory/1/routes/slow/1")
    assert r.json["route"]["limits"]["linear_mps"] == 0.3  # the stored value, not the default


def test_validate_route_with_an_arc_and_a_reverse(env):
    client, stub, maps, rev_dir, manifest = env
    # in the sim factory's 3.6 m aisle: 5 m out, a left 45 deg arc of R 1.5 (stays in the aisle), 1 m back
    arc = {"id": "s2", "type": "arc", "direction": "ccw", "angle_deg": 45, "radius_m": 1.5}
    body = route_payload([S("s1", 5.0, 0.0), arc, {"id": "s3", "type": "reverse", "distance_m": 1.0}])
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=body)
    assert r.status_code == 200, r.json
    steps = r.json["compiled"]["steps"]
    assert [s["type"] for s in steps] == ["straight", "arc", "reverse"]
    assert steps[1]["radius_m"] == 1.5 and steps[1]["centre"] == pytest.approx([5.0, 1.5])
    q = math.pi / 4
    assert steps[1]["end"] == pytest.approx([5.0 + 1.5 * math.sin(q), 1.5 - 1.5 * math.cos(q), q])
    assert steps[1]["v_mps"] == pytest.approx(0.3)  # arc 0.40, but never above the route's 0.3 cap
    assert steps[2]["v_mps"] == pytest.approx(0.15)
    assert steps[2]["end"][0] == pytest.approx(steps[1]["end"][0] - math.cos(q))
    assert len(stub.previews) == 1  # the preview carries the arc and reverse samples too
    # a 90 deg arc of the same radius turns the nose into the racks: clearance fails on that step
    body = route_payload([S("s1", 5.0, 0.0), {**arc, "angle_deg": 90}])
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=body)
    assert r.status_code == 422 and any(
        i["step_id"] == "s2" and i["code"] == "clearance" for i in r.json["issues"]
    )


# ---- dynamic-mapping plan §1.2/§1.3: map review -> a derived revision -------------------------


def _rect(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def test_edit_endpoint_derives_a_revision_and_validation_turns_provisional(env):
    client, stub, maps, rev_dir, manifest = env
    # a trolley surveyed into the aisle: rev1 has it as a wall, so the straight is refused
    g = build()
    r0, c0 = g.world_to_cell(8.0, 0.0)
    g.data[r0 - 3 : r0 + 3, c0 - 3 : c0 + 3] = 100
    stage = mb.staging_dir(maps, "trolley", 1)
    m = mb.Manifest(
        "trolley",
        1,
        mb.now_iso(),
        "map",
        0.05,
        [-3.0, -10.0, 0.0],
        g.width,
        g.height,
        {"x_m": 0, "y_m": 0, "yaw_rad": 0, "description": "mark"},
        {"dx_m": 0, "dy_m": 0, "dyaw_rad": 0, "note": "t"},
    )
    mb.stage_bundle(stage, g, None, m, required_posegraph=False)
    mb.publish(stage, maps, "trolley", 1)
    r = client.post("/api/maps/trolley/1/routes/validate", json=route_payload([S("s1", 12.0, 0.0)]))
    assert r.status_code == 422
    assert client.get("/api/maps/trolley/1/dynamic.png").status_code == 404  # no dynamic areas yet
    # mark it dynamic from the review page
    body = {"ops": [{"op": "dynamic", "polygon": _rect(7.6, -0.4, 8.4, 0.4)}], "note": "trolley bay"}
    r = client.post("/api/maps/trolley/1/edit", json=body)
    assert r.status_code == 200 and r.json["revision"] == 2 and r.json["dynamic_cells"] > 0, r.json
    idx = {x["map_id"]: x for x in client.get("/api/maps").json}["trolley"]["revisions"]
    assert idx[0]["edits"] is None and idx[0]["dynamic_cells"] == 0
    assert idx[1]["edits"]["parent_revision"] == 1 and idx[1]["edits"]["counts"] == {"dynamic": 1}
    assert idx[1]["edits"]["note"] == "trolley bay" and idx[1]["dynamic_cells"] == r.json["dynamic_cells"]
    meta = client.get("/api/maps/trolley/2").json
    assert meta["dynamic_cells"] == r.json["dynamic_cells"]
    png = client.get("/api/maps/trolley/2/dynamic.png")
    assert png.status_code == 200 and png.mimetype == "image/png" and png.data[1:4] == b"PNG"
    # the same straight on rev2: valid, with a provisional (info) issue naming the step
    r = client.post("/api/maps/trolley/2/routes/validate", json=route_payload([S("s1", 12.0, 0.0)]))
    assert r.status_code == 200 and r.json["ok"], r.json
    assert [(i["code"], i["step_id"], i["severity"]) for i in r.json["issues"]] == [
        ("provisional", "s1", "info")
    ]
    r = client.post("/api/maps/trolley/2/routes/save", json=route_payload([S("s1", 12.0, 0.0)]))
    assert r.status_code == 200
    r = client.post(
        "/api/missions",
        json={"map_id": "trolley", "map_revision": 2, "route_id": "r1", "route_revision": r.json["revision"]},
    )
    assert r.status_code == 200, r.json
    # rev1 is untouched: still refused there
    assert (
        client.post(
            "/api/maps/trolley/1/routes/validate", json=route_payload([S("s1", 12.0, 0.0)])
        ).status_code
        == 422
    )


def test_edit_endpoint_refuses_bad_or_empty_edits(env):
    client, stub, maps, rev_dir, manifest = env
    for body, code in (
        ([], 400),
        ({"ops": []}, 422),
        ({"ops": [{"op": "paint", "value": "wall", "polygon": _rect(0, 0, 1, 1)}]}, 422),
        ({"ops": [{"op": "dynamic", "polygon": _rect(500, 500, 501, 501)}]}, 422),  # off the map: no change
        ({"ops": [{"op": "dynamic", "polygon": _rect(0, 0, 1, 1)}], "note": 5}, 400),
    ):
        r = client.post("/api/maps/sim_factory/1/edit", json=body)
        assert r.status_code == code, (body, r.status_code, r.json)
    assert (
        client.post(
            "/api/maps/nope/1/edit", json={"ops": [{"op": "dynamic", "polygon": _rect(0, 0, 1, 1)}]}
        ).status_code
        == 404
    )
    assert mb.list_revisions(maps, "sim_factory") == [1]  # nothing was published


def test_survey_move_endpoints_forward_to_the_node_and_validate(env):
    client, stub, *_ = env
    r = client.post("/api/survey_move", json={"kind": "straight", "value": 1.0})
    assert r.status_code == 200 and r.json["message"] == "forward 1.00 m"
    assert ("survey_move", ("straight", 1.0)) in stub.calls
    r = client.post("/api/survey_move", json={"kind": "straight", "value": 99})
    assert r.status_code == 409 and "MANUAL" in r.json["message"]  # the node's refusal, passed on
    for bad in ({"kind": "strafe", "value": 1}, {"kind": "rotate", "value": "x"}, {"kind": "rotate"}):
        assert client.post("/api/survey_move", json=bad).status_code == 422
    assert client.post("/api/survey_move/stop", json={}).status_code == 200
    page = client.get("/maps").data
    for el in (b'id="sm-dist"', b'id="sm-fwd"', b'id="sm-rev"', b'id="sm-stop"', b'data-deg="-135"'):
        assert el in page
