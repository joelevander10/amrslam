"""Floor labels on the web API (gls-wait-coding-plan §C, P3): the label-set listing,
"Record label here", missions binding the newest set, and route validation against it.
Same app / stub adapter / temp maps dir as test_server.py."""

import os

import pytest
from amr_web.server import create_app
from test_server import FOOTPRINT_YAML, R, S, Stub, env, route_payload  # noqa: F401  (env is a fixture)

from amr_navigation import store


def W(sid, label="x0100y0000", name="UBC"):
    d = {"id": sid, "type": "wait", "name": name}
    if label:
        d["label_id"] = label
    return d


def wait_route(label="x0100y0000"):
    return route_payload([S("s1", 15.0, 0.0), W("w1", label), R("s2", "ccw", 180), S("s3", 2.0, 0.0)])


def home():
    return store.label_entry("x0000y0000", "home", "HOME", (0.0, 0.0, 0.0), "survey", 15, 0.001, 0.002)


def dest(lid="x0100y0000", name="UBC"):
    return store.label_entry(lid, "dest", name, (15.0, 0.0, 0.0), "amcl", 15, 0.001, 0.002)


def mission(client, rrev=1, **kw):
    body = {"map_id": "sim_factory", "map_revision": 1, "route_id": "r1", "route_revision": rrev, **kw}
    return client.post("/api/missions", json=body)


# ---- GET labels --------------------------------------------------------------------------


def test_get_labels_none_then_the_newest_set(env):  # noqa: F811
    client, _stub, maps, _rev_dir, manifest = env
    r = client.get("/api/maps/sim_factory/1/labels")
    assert r.status_code == 200 and r.json == {"ok": True, "revision": None, "sha256": None, "labels": []}
    store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home()])
    _, _, sha = store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home(), dest()])
    r = client.get("/api/maps/sim_factory/1/labels")  # never cached: the newer set shows at once
    assert r.status_code == 200 and r.json["revision"] == 2 and r.json["sha256"] == sha
    assert [(e["label_id"], e["role"], e["name"]) for e in r.json["labels"]] == [
        ("x0000y0000", "home", "HOME"),
        ("x0100y0000", "dest", "UBC"),
    ]
    assert client.get("/api/maps/nope/1/labels").status_code == 404
    assert client.get("/api/maps/sim_factory/2/labels").status_code == 404


def test_get_labels_with_a_corrupt_newest_set_is_a_404_not_an_older_set(env):  # noqa: F811
    client, _stub, maps, _rev_dir, manifest = env
    store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home()])
    _, p, _ = store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home(), dest()])
    open(p, "w").write("labels: [unclosed\n")
    r = client.get("/api/maps/sim_factory/1/labels")
    assert r.status_code == 404 and not r.json["ok"]


# ---- POST /api/labels/record -------------------------------------------------------------


@pytest.mark.parametrize(
    "body, why",
    [
        ({}, "role"),
        ({"role": "station", "name": "UBC"}, "role"),
        ({"role": "home", "name": 7}, "role"),
        ({"role": "home", "note": ["x"]}, "role"),
        ({"role": "dest"}, "needs a name"),
        ({"role": "dest", "name": "   "}, "needs a name"),
    ],
)
def test_record_validation_is_a_400_before_the_adapter(env, body, why):  # noqa: F811
    client, stub, *_ = env
    r = client.post("/api/labels/record", json=body)
    assert r.status_code == 400 and not r.json["ok"] and why in r.json["message"]
    assert stub.calls == []
    assert client.post("/api/labels/record", data="[1]").status_code == 400
    assert stub.calls == []


def test_record_passes_through_to_the_adapter(env):  # noqa: F811
    client, stub, *_ = env
    r = client.post("/api/labels/record", json={"role": "home"})
    assert r.status_code == 200 and r.json["ok"] and r.json["message"] == "record_label ok"
    r = client.post("/api/labels/record", json={"role": "dest", "name": "  UBC ", "note": "n" * 300})
    assert r.status_code == 200
    r = client.post("/api/labels/record", json={"role": "dest", "name": "U" * 50})
    assert stub.calls == [
        ("record_label", ("home", "", "")),
        ("record_label", ("dest", "UBC", "n" * 200)),
        ("record_label", ("dest", "U" * 40, "")),
    ]


def test_record_refused_by_the_executor_is_a_409(env):  # noqa: F811
    client, stub, *_ = env
    stub.record_label = lambda role, name, note: (False, "label not recorded: the wheels are moving")
    r = client.post("/api/labels/record", json={"role": "home"})
    assert r.status_code == 409 and r.json["message"] == "label not recorded: the wheels are moving"


class NoRecord(Stub):
    def __getattr__(self, name):
        if name == "record_label":
            raise AttributeError(name)
        return super().__getattr__(name)


def test_record_without_the_adapter_method_is_a_409(tmp_path, env):  # noqa: F811
    _client, _stub, maps, *_ = env
    fp = tmp_path / "fp2.yaml"
    fp.write_text(FOOTPRINT_YAML)
    adapter = NoRecord()
    assert not hasattr(adapter, "record_label")
    app = create_app(adapter, maps, str(fp), state_dir=str(tmp_path / "state2"))
    app.config["TESTING"] = True
    r = app.test_client().post("/api/labels/record", json={"role": "home"})
    assert r.status_code == 409 and "not available" in r.json["message"] and adapter.calls == []


# ---- missions bind the newest set ---------------------------------------------------------


def test_mission_binds_the_newest_set_and_names_it(env):  # noqa: F811
    client, _stub, maps, _rev_dir, manifest = env
    assert client.post("/api/maps/sim_factory/1/routes/save", json=wait_route()).status_code == 200
    r = mission(client)  # no set yet: unbound, the plain id
    assert (
        r.status_code == 200
        and r.json["labels"] is None
        and r.json["mission_id"] == "sim_factory_rev1_r1_rev1"
    )
    assert "labels" not in store.load_mission(maps, "sim_factory_rev1_r1_rev1")
    store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home(), dest()])
    _, _, sha2 = store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home(), dest(name="UBC hitch")])
    r = mission(client)
    assert r.status_code == 200 and r.json["mission_id"] == "sim_factory_rev1_r1_rev1_labels2"
    assert r.json["labels"] == {"revision": 2, "sha256": sha2}
    m = store.load_mission(maps, "sim_factory_rev1_r1_rev1_labels2")
    assert m["labels"] == {"revision": 2, "sha256": sha2}
    assert store.mission_labels(maps, m)["revision"] == 2
    assert mission(client).status_code == 200  # an idempotent retry
    # an explicit id keeps its name; the newest set is still bound
    r = mission(client, mission_id="hitch_run")
    assert r.status_code == 200 and r.json["mission_id"] == "hitch_run" and r.json["labels"]["revision"] == 2


def test_mission_ignores_a_set_recorded_on_another_bundle(env):  # noqa: F811
    client, _stub, maps, *_ = env
    assert client.post("/api/maps/sim_factory/1/routes/save", json=wait_route()).status_code == 200
    store.save_labels(maps, "sim_factory", 1, "another-bundle", [home()])  # lacks x0100y0000 too
    r = mission(client)
    assert (
        r.status_code == 200
        and r.json["labels"] is None
        and r.json["mission_id"] == "sim_factory_rev1_r1_rev1"
    )


def test_mission_refuses_a_wait_label_missing_from_the_newest_set(env):  # noqa: F811
    client, _stub, maps, _rev_dir, manifest = env
    assert client.post("/api/maps/sim_factory/1/routes/save", json=wait_route()).status_code == 200
    store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home(), dest("x0200y0000", "UBD")])
    r = mission(client)
    assert r.status_code == 422 and any(i["code"] == "label_unknown" for i in r.json["issues"])
    assert not os.path.exists(os.path.join(store.missions_dir(maps), "sim_factory_rev1_r1_rev1_labels1.yaml"))


# ---- route validation against the newest set ---------------------------------------------


def _label_issues(r):
    return [
        (i["code"], i["severity"], i["step_id"]) for i in r.json["issues"] if i["code"].startswith("label")
    ]


def test_route_validate_and_get_report_the_label_issues(env):  # noqa: F811
    client, _stub, maps, _rev_dir, manifest = env
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=wait_route())
    assert r.status_code == 200 and r.json["ok"] and _label_issues(r) == [("label_set", "info", "w1")]
    assert client.post("/api/maps/sim_factory/1/routes/save", json=wait_route()).status_code == 200
    r = client.get("/api/maps/sim_factory/1/routes/r1/1")
    assert r.json["ok"] and _label_issues(r) == [("label_set", "info", "w1")]
    store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home(), dest("x0200y0000", "UBD")])
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=wait_route())
    assert (
        r.status_code == 422 and not r.json["ok"] and _label_issues(r) == [("label_unknown", "error", "w1")]
    )
    r = client.get("/api/maps/sim_factory/1/routes/r1/1")  # the stored route, against today's set
    assert (
        r.status_code == 200 and not r.json["ok"] and _label_issues(r) == [("label_unknown", "error", "w1")]
    )
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=wait_route("x0200y0000"))
    assert r.status_code == 200 and r.json["ok"] and _label_issues(r) == []
    r = client.post("/api/maps/sim_factory/1/routes/validate", json=wait_route(None))
    assert r.status_code == 200 and _label_issues(r) == []


# ---- defects pinned -------------------------------------------------------------------------


@pytest.mark.xfail(
    strict=True,
    reason="BUG server.py _validate_payload/api_route_validate (and api_route_save, api_route_get): "
    "labels_for() raises store.StoreError on a corrupt newest label set, and those handlers do "
    "not catch StoreError -> HTTP 500 instead of a bounded 4xx",
)
@pytest.mark.parametrize("call", ["validate", "save", "get"])
def test_a_corrupt_label_set_is_a_bounded_error_on_the_route_endpoints(env, call):  # noqa: F811
    client, _stub, maps, _rev_dir, manifest = env
    assert client.post("/api/maps/sim_factory/1/routes/save", json=wait_route()).status_code == 200
    _, p, _ = store.save_labels(maps, "sim_factory", 1, manifest.sha256, [home(), dest()])
    open(p, "w").write("labels: [unclosed\n")
    client.application.config["PROPAGATE_EXCEPTIONS"] = False
    if call == "get":
        r = client.get("/api/maps/sim_factory/1/routes/r1/1")
    else:
        r = client.post(f"/api/maps/sim_factory/1/routes/{call}", json=wait_route())
    assert 400 <= r.status_code < 500, r.status_code
