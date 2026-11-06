"""POS routes (2026-11-06): a main route + its own branch, checked against the main revision."""

from amr_navigation.compiler import compile_route

from amr_navigation import route as rt


def _route(steps, branch=None, start=(0.0, 0.0, 0.0)):
    d = {
        "schema_version": rt.SCHEMA_VERSION,
        "route_id": "pos1" if branch else "main",
        "map": {"id": "m", "revision": 1, "sha256": "x"},
        "start": {"x_m": start[0], "y_m": start[1], "yaw_deg": start[2]},
        "steps": steps,
    }
    if branch:
        d["branch"] = branch
    return rt.Route.from_dict(d)


S1 = {"id": "s1", "type": "straight", "to": {"x_m": 4.0, "y_m": 0.0}}
S2 = {"id": "s2", "type": "straight", "to": {"x_m": 8.0, "y_m": 0.0}}
BR = [
    {"id": "b1", "type": "rotate", "direction": "ccw", "angle_deg": 90},
    {"id": "b2", "type": "straight", "to": {"x_m": 4.0, "y_m": 2.0}},
    {"id": "b3", "type": "wait", "name": "POS 1"},
    {"id": "b4", "type": "rotate", "direction": "ccw", "angle_deg": 180},
    {"id": "b5", "type": "straight", "to": {"x_m": 4.0, "y_m": 0.0}},
    {"id": "b6", "type": "rotate", "direction": "ccw", "angle_deg": 90},
]
META = {
    "main_route_id": "main",
    "main_revision": 2,
    "leave_after": "s1",
    "rejoin_after": "s1",
    "steps": [s["id"] for s in BR],
}
MAIN = _route([S1, S2])


def test_a_pos_route_is_main_plus_its_branch_and_runs_as_one_route():
    pos = _route([S1, *BR, S2], META)
    assert rt.branch_problem(pos, MAIN) is None
    assert pos.to_dict()["branch"] == META and rt.Route.from_dict(pos.to_dict()).branch == pos.branch
    c = compile_route(pos)
    assert [s.id for s in c.steps] == ["s1", "b1", "b2", "b3", "b4", "b5", "b6", "s2"]
    assert abs(c.end[0] - 8.0) < 1e-9 and abs(c.end[1]) < 1e-9


def test_a_changed_main_part_or_start_is_reported():
    s2b = dict(S2, v_mps=0.3)  # a slow zone added on the POS route's copy of main
    assert "differs from main rev2 at s2" in rt.branch_problem(_route([S1, *BR, s2b], META), MAIN)
    assert "start" in rt.branch_problem(_route([S1, *BR, S2], META, start=(0.1, 0.0, 0.0)), MAIN)
    assert "no step 's9'" in rt.branch_problem(_route([S1, *BR, S2], dict(META, rejoin_after="s9")), MAIN)
    clash = [dict(BR[0], id="s2"), *BR[1:]]
    meta = dict(META, steps=["s2", *META["steps"][1:]])
    assert "also main route step ids" in rt.branch_problem(_route([S1, *clash, S2], meta), MAIN)
    # a branch cannot hang off another branch route
    assert "itself a branch" in rt.branch_problem(_route([S1, *BR, S2], META), _route([S1, *BR, S2], META))


def test_a_plain_route_has_no_branch():
    assert MAIN.branch is None and "branch" not in MAIN.to_dict() and rt.branch_problem(MAIN, MAIN) is None
