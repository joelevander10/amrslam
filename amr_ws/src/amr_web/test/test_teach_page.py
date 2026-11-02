"""Teach page and API (tracked plan 3, P1): engineer-gated teaching, open At home, the site lists."""

from test_operator_ux import app_for


def test_teach_page_and_site_lists(tmp_path):
    client, _stub = app_for({}, tmp_path)
    assert client.get("/teach").status_code == 200
    from agv_core import config
    site = client.get("/api/line/site").get_json()
    # The profile's tape_loc.site_mission: the straight test today, site-103 later.
    assert site["mission"] == config.TAPE_LOC_SITE_MISSION and not site["error"]
    assert config.TAPE_LOC_SHADOW_CIRCUIT in site["circuits"] and site["stations"]


def test_teaching_needs_the_engineer_view(tmp_path):
    client, stub = app_for({}, tmp_path)
    client.post("/api/role", json={"role": "operator"})
    r = client.post("/api/line/teach", json={"action": "pass_start", "arg": "C-MRU"})
    assert r.status_code == 403 and not any(c[0] == "line_teach" for c in stub.calls)
    # At home only sets AMCL's initial pose: open to the operator.
    assert client.post("/api/line/at_home").status_code == 200
    assert ("line_at_home", ()) in stub.calls


def test_the_engineer_teach_call_is_forwarded(tmp_path):
    client, stub = app_for({}, tmp_path)   # engineer is the default view
    r = client.post("/api/line/teach", json={"action": "station", "arg": "MW-1"})
    assert r.status_code == 200 and ("line_teach", ("station", "MW-1")) in stub.calls
