"""P4 (unified plan §12.2): a mission loads only onto the ACTIVE map revision."""

from amr_mission.route_executor_node import mission_matches_active_map as check


def mission(map_id="line_section", rev=2, sha="abc"):
    return {"map": {"id": map_id, "revision": rev, "sha256": sha}, "route": {}}


def test_matching_mission_loads_and_others_are_refused():
    active = ("line_section", 2, "abc")
    assert check(mission(), active) is None
    assert "rev1" in check(mission(rev=1), active)  # older revision of the same map
    assert "active map is" in check(mission(map_id="other"), active)
    assert "hash" in check(mission(sha="zzz"), active)  # same id/rev, different bundle content


def test_unsupervised_stack_binds_to_nothing():
    assert check(mission(), ("", 0, "")) is None
