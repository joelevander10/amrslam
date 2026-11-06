"""Removing maps, routes and missions from the web (2026-11-06): asked first, moved to the
maps trash, never what is in use, and nothing left pointing at what went."""

import os

from test_server import R, S, env, route_payload  # noqa: F401 - env is the shared fixture

from amr_mission import map_bundle as mb
from amr_navigation import store

GOOD = [S("s1", 15.0, 0.0), R("s2", "ccw", 180), S("s3", 2.0, 0.0)]


def _setup(client):
    """sim_factory rev1 with r1 rev1+rev2 and a mission on r1 rev2; rev2 (rotated) with r2."""
    assert client.post("/api/maps/sim_factory/1/routes/save", json=route_payload(GOOD)).status_code == 200
    assert client.post("/api/maps/sim_factory/1/routes/save", json=route_payload(GOOD)).status_code == 200
    m = {"map_id": "sim_factory", "map_revision": 1, "route_id": "r1", "route_revision": 2}
    assert client.post("/api/missions", json=m).status_code == 200
    assert client.post("/api/maps/sim_factory/1/rotate", json={"angle_deg": 1.0}).status_code == 200
    r = client.post("/api/maps/sim_factory/2/routes/save", json=route_payload(GOOD, route_id="r2"))
    assert r.status_code == 200, r.json
    m2 = {"map_id": "sim_factory", "map_revision": 2, "route_id": "r2", "route_revision": 1}
    assert client.post("/api/missions", json=m2).status_code == 200


def _missions(maps):
    return sorted(m["mission_id"] for m in store.list_missions(maps))


def test_a_route_revision_asks_first_then_goes_with_its_mission(env):  # noqa: F811
    client, _stub, maps, *_ = env
    _setup(client)
    url = "/api/maps/sim_factory/routes/r1/delete"
    r = client.post(url, json={"revision": 2})
    assert (
        r.status_code == 409
        and r.json["needs_confirm"]
        and r.json["missions"] == ["sim_factory_rev1_r1_rev2"]
    )
    assert store.list_routes(maps, "sim_factory")["r1"] == [1, 2]  # nothing moved yet
    r = client.post(url, json={"revision": 2, "confirm": True})
    assert r.status_code == 200, r.json
    assert store.list_routes(maps, "sim_factory")["r1"] == [1]
    assert _missions(maps) == ["sim_factory_rev2_r2_rev1"]
    # kept, not erased: the file is in the trash under its store path
    trash = r.json["trash"]
    assert os.path.isfile(os.path.join(trash, "sim_factory", "routes", "r1", "rev2.yaml"))
    assert os.path.isfile(os.path.join(trash, "missions", "sim_factory_rev1_r1_rev2.yaml"))
    # the whole route
    r = client.post(url, json={"revision": None, "confirm": True})
    assert r.status_code == 200 and "r1" not in store.list_routes(maps, "sim_factory")
    assert client.post(url, json={"confirm": True}).status_code == 404


def test_a_map_revision_takes_the_routes_drawn_on_it_and_their_missions(env):  # noqa: F811
    client, _stub, maps, *_ = env
    _setup(client)
    r = client.post("/api/maps/sim_factory/delete", json={"revision": 2})
    assert (
        r.status_code == 409
        and r.json["routes"] == ["r2 rev1"]
        and r.json["missions"] == ["sim_factory_rev2_r2_rev1"]
    )
    r = client.post("/api/maps/sim_factory/delete", json={"revision": 2, "confirm": True})
    assert r.status_code == 200
    assert mb.list_revisions(maps, "sim_factory") == [1]
    assert list(store.list_routes(maps, "sim_factory")) == ["r1"]  # r1 was drawn on rev1
    assert _missions(maps) == ["sim_factory_rev1_r1_rev2"]
    assert client.get("/api/maps/sim_factory/2").status_code == 404
    # the last revision is the whole map: routes and missions with it, and the list skips the trash
    r = client.post("/api/maps/sim_factory/delete", json={"revision": 1, "confirm": True})
    assert r.status_code == 200 and not os.path.exists(os.path.join(maps, "sim_factory"))
    assert _missions(maps) == [] and client.get("/api/maps").json == []
    assert client.get("/api/maps/sim_factory/1").status_code == 404  # not served from the cache


def test_what_is_in_use_is_never_removed(env):  # noqa: F811
    client, stub, maps, *_ = env
    _setup(client)
    base = stub.state()
    stub.state = lambda: {
        **base,
        "mode": {"mode_name": "NAVIGATION", "active_map_id": "sim_factory", "active_map_revision": 1},
    }
    for body in ({"revision": 1, "confirm": True}, {"revision": None, "confirm": True}):
        r = client.post("/api/maps/sim_factory/delete", json=body)
        assert r.status_code == 409 and "active map" in r.json["message"]
    # another revision of the active map may go
    assert (
        client.post("/api/maps/sim_factory/delete", json={"revision": 2, "confirm": True}).status_code == 200
    )
    # a loaded mission, and the route and map under it
    stub.state = lambda: {**base, "run": {"mission_id": "sim_factory_rev1_r1_rev2", "state_name": "READY"}}
    for url, body in (
        ("/api/missions/sim_factory_rev1_r1_rev2/delete", {"confirm": True}),
        ("/api/maps/sim_factory/routes/r1/delete", {"revision": 2, "confirm": True}),
    ):
        r = client.post(url, json=body)
        assert r.status_code == 409 and "is loaded" in r.json["message"], r.json
    stub.state = lambda: {**base, "mode": {"mode_name": "MAPPING"}}
    assert client.post("/api/maps/sim_factory/delete", json={"confirm": True}).status_code == 409
    assert mb.list_revisions(maps, "sim_factory") == [1] and _missions(maps) == ["sim_factory_rev1_r1_rev2"]


def test_the_operator_view_cannot_remove(env):  # noqa: F811
    client, _stub, maps, *_ = env
    _setup(client)
    assert client.post("/api/role", json={"role": "operator"}).status_code == 200
    r = client.post("/api/missions/sim_factory_rev1_r1_rev2/delete", json={"confirm": True})
    assert r.status_code == 403 and len(_missions(maps)) == 2
