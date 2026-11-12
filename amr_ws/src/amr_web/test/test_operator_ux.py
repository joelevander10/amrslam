"""The operator surface: one catalogue, one standing list, one button (plan phases 0-3).

These tests exist because every page used to derive "what is wrong" in its own
JavaScript, in its own words, with no action attached. The contract now is:
`/api/alarms` decides, the pages render. So the assertions here are about wording
and about which button the operator is offered - not about internals.
"""

import json
import os
import zipfile

import pytest
from agv_core import alarms as cat
from amr_web.eventlog import EventLog
from amr_web.server import create_app

from amr_web import alarms as view
from amr_web import reports, role


class Stub:
    def __init__(self, state=None):
        self._state = state or {}
        self.emitted = []
        self.calls = []

    def state(self):
        return self._state

    def events(self, since=0):
        return [{"n": 1, "t": 0.0, "source": "x", "level": "warn", "code": "ESTOP", "text": "t", "seq": 1}]

    def diagnostics(self):
        return {}

    def event_log_files(self):
        return []

    def emit(self, level, code, text, source="amr_web"):
        self.emitted.append((level, code, text))

    def __getattr__(self, name):
        def fn(*a):
            self.calls.append((name, a))
            return True, f"{name} ok"

        return fn


def app_for(state, tmp_path):
    maps = tmp_path / "maps"
    maps.mkdir(exist_ok=True)
    stub = Stub(state)
    app = create_app(stub, str(maps), state_dir=str(tmp_path / "state"))
    app.config["TESTING"] = True
    return app.test_client(), stub


READY_STATE = {
    "mode": {"mode_name": "IDLE", "fault_code": "", "reason": "", "base_ready": True, "generation": 1},
    "mode_age_s": 0.1,
    "panel": {"valid": True, "mode_auto": False, "age_s": 0.05},
    "drives": {"operational": True, "left": "Operation enabled", "right": "Operation enabled", "age_s": 0.05},
    "mux": {"inhibited": False, "source": "none", "reason": "", "code": "NO_SOURCE", "age_s": 0.05},
    "run": None,
    "localization": None,
}


def merged(**over):
    st = json.loads(json.dumps(READY_STATE))
    st.update(over)
    return st


# ---- the standing list ----------------------------------------------------------


def test_a_healthy_vehicle_stands_nothing_and_offers_no_button():
    rows = view.standing(merged())
    assert rows == []
    h = view.headline(merged(), rows)
    assert h["state"] == view.READY and h["button"] == ""


def test_no_supervisor_is_the_only_thing_reported():
    """Without ModeState nothing else can be trusted, so one row, not six."""
    rows = view.standing({"mode": None})
    assert [r["code"] for r in rows] == ["SUPERVISOR_DOWN"]
    assert view.headline({"mode": None}, rows)["button"] == "Restart"


def test_switch_on_disabled_reads_as_the_cabinet_reset_not_a_broken_drive():
    """The 2026-09-22 case: the safety chain holds STO, the drives are fine."""
    st = merged(
        drives={
            "operational": False,
            "left": "Switch on disabled",
            "right": "Switch on disabled",
            "age_s": 0.05,
        }
    )
    rows = view.standing(st)
    assert rows[0]["code"] == "SAFETY_RESET_NEEDED"
    assert "reset" in rows[0]["action"].lower()


def test_a_disarmed_drive_at_idle_is_not_an_alarm():
    st = merged(
        drives={
            "operational": False,
            "left": "Ready to switch on",
            "right": "Ready to switch on",
            "age_s": 0.05,
        }
    )
    assert view.standing(st) == []


def test_a_field_stop_says_it_resumes_by_itself_and_offers_no_button():
    st = merged(
        run={
            "state_name": "BLOCKED",
            "hold_cause": "field",
            "fault_code": "FIELD_BLOCKED",
            "auto_resume": True,
            "reason": "safety stop",
            "mission_id": "m1",
            "step_index": 2,
        }
    )
    rows = view.standing(st)
    h = view.headline(st, rows)
    assert rows[0]["code"] == "FIELD_BLOCKED"
    assert h["state"] == view.STOPPED_WILL_RESUME and h["button"] == ""


def test_an_inferred_estop_hold_sends_the_operator_to_the_cabinet_reset():
    """The executor's torque-inferred 'estop' hold (panel stale or not yet reporting): the
    same ESTOP words as the panel's emergency - release, then the physical Reset."""
    st = merged(
        run={
            "state_name": "BLOCKED",
            "hold_cause": "estop",
            "fault_code": "ESTOP",
            "auto_resume": False,
            "reason": "safety stop",
            "mission_id": "m1",
            "step_index": 0,
        }
    )
    h = view.headline(st, view.standing(st))
    assert h["state"] == view.STOPPED_NEEDS_YOU and "Reset" in h["action"] and h["button"] == ""


def test_an_executor_fault_offers_acknowledge():
    st = merged(run={"state_name": "FAULT", "fault_code": "EXEC_OFF_PATH", "reason": "cross-track"})
    h = view.headline(st, view.standing(st))
    assert h["button"] == "Acknowledge" and h["code"] == "EXEC_OFF_PATH"


def test_a_dead_base_offers_restart_and_says_needs_service():
    st = merged(
        mode={
            "mode_name": "FAULT",
            "fault_code": "BASE_EXITED",
            "reason": "base died",
            "base_ready": False,
            "generation": 1,
        }
    )
    h = view.headline(st, view.standing(st))
    assert h["state"] == view.NEEDS_SERVICE and h["button"] == "Restart"


def test_lost_localisation_sends_the_operator_to_the_run_page():
    st = merged(
        mode={"mode_name": "NAVIGATION", "fault_code": "", "reason": "", "base_ready": True, "generation": 1},
        localization={"state_name": "LOST", "code": "LOC_SCAN_MISMATCH", "reason": "beams through walls"},
    )
    rows = view.standing(st)
    assert rows[0]["code"] == "LOC_SCAN_MISMATCH"
    assert view.headline(st, rows)["button"] == "Set position"


def test_localisation_is_not_the_operators_problem_outside_navigation():
    st = merged(localization={"state_name": "UNLOCALIZED", "code": "LOC_NOT_SET", "reason": "no pose"})
    assert view.standing(st) == []


def test_an_unplugged_scanner_is_reported_at_all():
    """It reports nothing itself: the driver is an optional launch member, so the
    supervisor stays IDLE, the drives still jog, and before 2026-09-22 the operator had
    no way to learn the lidar was unplugged."""
    st = merged(scan_age_s=None, up_s=120.0)
    rows = view.standing(st)
    assert [r["code"] for r in rows] == ["SCANNER_SILENT"]
    assert "hand" in rows[0]["action"]  # manual driving is unaffected, and says so
    assert view.headline(st, rows)["state"] == view.READY  # a warning, not a stop


def test_a_scanner_that_dies_mid_run_is_reported_separately():
    st = merged(scan_age_s=30.0, up_s=300.0)
    assert [r["code"] for r in view.standing(st)] == ["SCANNER_STALE"]
    assert view.standing(merged(scan_age_s=0.1, up_s=300.0)) == []


def test_a_page_loaded_during_boot_does_not_accuse_the_scanner():
    assert view.standing(merged(scan_age_s=None, up_s=3.0)) == []


def test_a_low_disk_warns_and_a_full_one_is_an_error():
    warn = merged(disk={"free_mb": 800, "level": "warn", "path": "/home"})
    rows = view.standing(warn)
    assert [r["code"] for r in rows] == ["DISK_LOW"] and rows[0]["level"] == "warn"
    full = merged(disk={"free_mb": 100, "level": "stop", "path": "/home"})
    rows = view.standing(full)
    assert rows[0]["code"] == "DISK_FULL" and rows[0]["level"] == "error"
    assert "already running" in rows[0]["action"]  # a moving vehicle is not stopped
    assert view.standing(merged(disk={"free_mb": 9000, "level": "ok", "path": "/home"})) == []


def test_the_worst_alarm_is_first_and_a_code_is_never_repeated():
    st = merged(
        panel={"valid": False, "mode_auto": False, "age_s": 0.05},
        mux={
            "inhibited": True,
            "source": "none",
            "reason": "no panel authority",
            "code": "PANEL_STALE",
            "age_s": 0.05,
        },
    )
    rows = view.standing(st)
    assert [r["code"] for r in rows] == ["PANEL_STALE"]
    assert rows[0]["level"] == "error"


def test_every_standing_row_carries_an_action_the_operator_can_perform():
    st = merged(
        mode={
            "mode_name": "FAULT",
            "fault_code": "MUX_ACK_TIMEOUT",
            "reason": "no ack",
            "base_ready": True,
            "generation": 1,
        },
        run={"state_name": "FAULT", "fault_code": "EXEC_PREREQ_LOST", "reason": "wheels"},
    )
    for row in view.standing(st):
        assert row["title"] and row["action"].endswith(".")
        assert row["clears_by"] in cat.CLEARS


def test_since_is_kept_while_an_alarm_stands_and_dropped_when_it_clears():
    tracker = view.StandingTracker()
    st = merged(panel=None)
    rows = tracker.apply(view.standing(st), 100.0)
    assert rows[0]["since"] == 100.0
    rows = tracker.apply(view.standing(st), 130.0)
    assert rows[0]["since"] == 100.0 and rows[0]["for_s"] == 30.0
    tracker.apply(view.standing(merged()), 140.0)
    rows = tracker.apply(view.standing(st), 150.0)
    assert rows[0]["since"] == 150.0  # it cleared in between: the clock restarts


# ---- the endpoints ---------------------------------------------------------------


def test_api_alarms_serves_the_list_and_the_headline(tmp_path):
    client, _ = app_for(merged(), tmp_path)
    d = client.get("/api/alarms").get_json()
    assert d["alarms"] == [] and d["headline"]["state"] == view.READY


def test_events_carry_the_catalogue_title(tmp_path):
    client, _ = app_for(merged(), tmp_path)
    rows = client.get("/api/events").get_json()
    assert rows[0]["title"] == cat.CATALOGUE["ESTOP"].title


def test_a_fresh_session_lands_on_the_engineer_view(tmp_path):
    """2026-10-30: Status folded into Home and Monitor, so both roles land on Home."""
    client, _ = app_for(merged(), tmp_path)
    assert client.get("/").headers["Location"].endswith("/home")
    assert client.get("/api/role").get_json()["role"] == role.ENGINEER


def test_the_home_page_is_the_operators_landing_page(tmp_path):
    client, _ = app_for(merged(), tmp_path)
    client.post("/api/role", json={"role": "operator"})
    assert client.get("/").headers["Location"].endswith("/home")
    body = client.get("/home").get_data(as_text=True)
    assert 'id="home-primary"' in body
    # the operator's nav: no Params, no Monitor, no Commission
    assert "/params" not in body and "/monitor" not in body
    assert "/run" in body and "/alarms" in body


def test_the_engineer_pin_opens_the_engineering_nav(tmp_path):
    client, _ = app_for(merged(), tmp_path)
    assert client.post("/api/role", json={"role": "engineer", "pin": "0000"}).status_code == 403
    assert client.post("/api/role", json={"role": "engineer", "pin": role.DEFAULT_PIN}).status_code == 200
    body = client.get("/home").get_data(as_text=True)
    assert "/params" in body and "/monitor" in body
    assert "/commissioning" not in body and 'href="/status"' not in body  # removed 2026-10-30
    # and the way back is always open: a guard you cannot leave is a trap
    assert client.post("/api/role", json={"role": "operator"}).status_code == 200
    assert "/params" not in client.get("/home").get_data(as_text=True)


def test_only_the_engineer_gets_an_unconditional_restart_button(tmp_path):
    """The operator gets Restart on Home only when an alarm says the base cannot be
    rebuilt; an engineer restarts for other reasons (a rebuilt overlay, a changed env)."""
    client, _ = app_for(merged(), tmp_path)
    client.post("/api/role", json={"role": "operator"})
    assert 'id="btn-restart"' not in client.get("/monitor").get_data(as_text=True)
    client.post("/api/role", json={"role": "engineer", "pin": role.DEFAULT_PIN})
    assert 'id="btn-restart"' in client.get("/monitor").get_data(as_text=True)


def test_the_pin_lives_outside_the_repository_and_can_be_overridden(tmp_path, monkeypatch):
    state_dir = str(tmp_path / "state")
    assert role.pin(state_dir) == role.DEFAULT_PIN
    written = tmp_path / "state" / "web.json"
    assert written.exists() and oct(os.stat(written).st_mode)[-3:] == "600"
    monkeypatch.setenv("AMR_WEB_ENGINEER_PIN", "4242")
    assert role.pin(state_dir) == "4242" and role.check(state_dir, "4242")


def test_restart_is_refused_while_the_wheels_turn(tmp_path):
    st = merged(
        mux={
            "inhibited": False,
            "source": "manual",
            "reason": "",
            "code": "",
            "left_rad_s": 1.2,
            "right_rad_s": 1.2,
            "age_s": 0.05,
        }
    )
    client, _ = app_for(st, tmp_path)
    r = client.post("/api/service/restart")
    assert r.status_code == 409 and "turning" in r.get_json()["message"]


def test_restart_is_refused_while_a_run_is_live(tmp_path):
    st = merged(run={"state_name": "EXECUTING", "mission_id": "m1", "step_index": 1})
    client, _ = app_for(st, tmp_path)
    assert client.post("/api/service/restart").status_code == 409


def test_a_page_error_becomes_an_event(tmp_path):
    client, stub = app_for(merged(), tmp_path)
    client.post("/api/page-error", json={"where": "/run", "what": "x is not defined"})
    assert stub.emitted and stub.emitted[0][1] == "WEB_BUG"


def test_an_unhandled_bug_answers_json_with_a_reference(tmp_path):
    client, stub = app_for(merged(), tmp_path)

    @client.application.get("/api/boom")
    def boom():
        raise RuntimeError("kaboom")

    r = client.get("/api/boom")
    assert r.status_code == 500
    body = r.get_json()
    assert body["code"] == "WEB_BUG" and len(body["ref"]) == 8
    assert "kaboom" not in r.get_data(as_text=True)  # never a stack trace on the panel
    assert stub.emitted[-1][1] == "WEB_BUG"


def test_a_report_packages_the_evidence(tmp_path):
    client, _ = app_for(merged(), tmp_path)
    r = client.post("/api/report", json={"note": "it stopped"})
    assert r.status_code == 200
    name = r.get_json()["name"]
    listing = client.get("/api/reports").get_json()
    assert listing[0]["name"] == name
    blob = client.get(f"/api/report/{name}")
    assert blob.status_code == 200
    path = reports.path_of(str(tmp_path / "state"), name)
    with zipfile.ZipFile(path) as z:
        assert {"note.txt", "state.json", "events.json", "journal.txt"} <= set(z.namelist())
        assert z.read("note.txt").decode() == "it stopped"


def test_a_report_name_from_a_url_cannot_escape_the_reports_directory(tmp_path):
    client, _ = app_for(merged(), tmp_path)
    assert reports.path_of(str(tmp_path), "../../etc/passwd") is None
    assert client.get("/api/report/..%2F..%2Fetc%2Fpasswd").status_code in (404, 308)


# ---- the event log on disk -------------------------------------------------------


def test_the_event_log_survives_a_restart_and_rotates(tmp_path):
    path = str(tmp_path / "logs" / "events.jsonl")
    log = EventLog(path, max_bytes=400, keep=2)
    for i in range(40):
        log.append({"t": i, "code": "ESTOP", "text": f"event {i}", "level": "warn"})
    assert os.path.exists(path + ".1")  # it rotated rather than growing without bound
    tail = log.tail(100)
    assert tail[-1]["text"] == "event 39"
    assert len(EventLog(path, max_bytes=400, keep=2).tail(100)) == len(tail)  # reread = same history


def test_an_error_event_is_forced_to_the_platter(tmp_path, monkeypatch):
    """W4: after a cut the line you need is the last error, so that one is synced and
    the chatty info events are not."""
    synced = []
    real = os.fsync
    monkeypatch.setattr(os, "fsync", lambda fd: (synced.append(fd), real(fd))[1])
    log = EventLog(str(tmp_path / "logs" / "events.jsonl"))
    log.append({"level": "info", "code": "MUX_SOURCE", "text": "t"})
    assert synced == []
    log.append({"level": "error", "code": "DRIVE_SILENT", "text": "t"}, sync=True)
    assert len(synced) == 1
    assert len(log.tail()) == 2  # both are in the file either way


def test_the_role_file_is_written_atomically(tmp_path):
    """A cut during the first run must not leave a half-written PIN file."""
    state = str(tmp_path / "state")
    role.ensure(state)
    assert sorted(p.name for p in (tmp_path / "state").iterdir()) == ["web.json"]  # no .tmp left
    assert role.check(state, role.DEFAULT_PIN)


def test_a_half_written_line_is_skipped_not_raised(tmp_path):
    path = tmp_path / "logs" / "events.jsonl"
    path.parent.mkdir(parents=True)
    path.write_text('{"code":"ESTOP","text":"good"}\n{"code":"EST')  # power cut mid-append
    assert [e["text"] for e in EventLog(str(path)).tail()] == ["good"]


@pytest.mark.parametrize("code", sorted(cat.CATALOGUE))
def test_every_catalogue_row_renders_for_the_operator(code):
    row = cat.describe(code, "detail")
    assert row["title"] and row["action"] and row["level"] in ("info", "warn", "error")
    assert cat.BUTTON[row["clears_by"]] in ("", "Acknowledge", "Recover", "Restart", "Set position")


def test_the_engineer_pin_is_typed_on_an_on_page_keypad(tmp_path):
    """The kiosk has no on-screen keyboard, so prompt() cannot be typed into on the panel."""
    client, _ = app_for(merged(), tmp_path)
    client.post("/api/role", json={"role": "operator"})
    body = client.get("/home").get_data(as_text=True)
    assert 'id="pin-pad"' in body
    assert all(f'data-k="{d}"' in body for d in "0123456789")
    js = client.get("/static/amr.js").get_data(as_text=True)
    assert "prompt(" not in js


def test_home_shows_the_speed_tile_and_loads_no_mission(tmp_path):
    """2026-10-08: setting a mission is not a Home thing. Home shows the measured speed."""
    client, _ = app_for(merged(), tmp_path)
    body = client.get("/home").get_data(as_text=True)
    assert 'id="live-speed"' in body and 'class="home-live"' in body
    for gone in ("live-rfid", "live-marker", "live-line"):
        assert f'id="{gone}"' not in body, gone
    assert "home-mission" not in body and "/api/mission/run" not in body
    assert "linetrack.js" not in body


def test_home_shows_the_job_beside_the_status_and_cannot_set_it(tmp_path):
    """2026-10-08: the job is shown on Home, wider than the status block."""
    client, _ = app_for(merged(), tmp_path)
    body = client.get("/home").get_data(as_text=True)
    top = body.index('class="home-top"')
    assert top < body.index('id="home-job"') < body.index('id="home-line"')
    assert "/api/line/mission" not in body and "<select" not in body


def test_the_adapter_reports_measured_wheel_speed_as_body_speed(monkeypatch):
    """/wheel_states_ui is the measured 606Ch feedback; v is the mean of the two rims."""
    import threading
    from types import SimpleNamespace

    from amr_web import adapter as ad

    monkeypatch.setattr(ad, "WHEEL_RADIUS_M", 0.1)
    a = ad.RosAdapter.__new__(ad.RosAdapter)
    a._lock = threading.Lock()
    a._now = lambda: 5.0

    def msg(left, right, valid=True):
        return SimpleNamespace(
            left_vel_rad_s=left, right_vel_rad_s=right, left_valid=valid, right_valid=valid,
            left_pos_rad=20.0, right_pos_rad=-3.0, left_counts=1234, right_counts=-56, counts_valid=valid,
        )

    a._on_wheels(msg(5.0, 3.0))
    assert a._wheels["v_mps"] == pytest.approx(0.4)
    assert (a._wheels["left_mps"], a._wheels["right_mps"]) == (pytest.approx(0.5), pytest.approx(0.3))
    # the Run page's encoder tiles: raw counters and the distance each wheel rolled
    assert (a._wheels["left_counts"], a._wheels["right_counts"]) == (1234, -56)
    assert (a._wheels["left_m"], a._wheels["right_m"]) == (pytest.approx(2.0), pytest.approx(-0.3))
    a._on_wheels(msg(-2.0, -2.0))
    assert a._wheels["v_mps"] == pytest.approx(-0.2), "reverse is negative"
    a._on_wheels(msg(9.0, 9.0, valid=False))
    assert a._wheels["valid"] is False and a._wheels["v_mps"] is None
    assert a._wheels["left_counts"] is None and a._wheels["left_m"] is None


def test_one_run_tab_and_maps_routes_live_under_run(tmp_path):
    """2026-10-27 (trackless only): one Run tab, no tracked pages; Maps/Routes in its subnav."""
    client, _ = app_for(merged(), tmp_path)
    body = client.get("/home").get_data(as_text=True)
    nav = body.split('<nav>')[1].split('</nav>')[0]
    assert '<a href="/run"' in nav and ">Run<" in nav
    assert "Run tracked" not in body and "Run trackless" not in body and ">Teach<" not in body
    assert "/run-tracked" not in nav and "/teach" not in nav
    assert "/maps" not in nav and "/editor" not in nav, "Maps/Routes left the top nav"
    sub = client.get("/maps").get_data(as_text=True)
    assert 'class="subnav"' in sub and 'href="/maps"' in sub and 'href="/editor"' in sub
    for path in ("/run", "/run-trackless", "/maps", "/editor", "/review"):
        assert client.get(path).status_code == 200, path


def test_the_tracked_pages_and_line_apis_are_gone(tmp_path):
    client, stub = app_for(merged(), tmp_path)
    for path in ("/run-tracked", "/teach", "/api/line/missions", "/api/line/site"):
        assert client.get(path).status_code == 404, path
    for path in ("/api/line/mission", "/api/line/clear", "/api/line/teach", "/api/line/at_home"):
        assert client.post(path, json={}).status_code == 404, path
    r = client.post("/api/mode", json={"target": "line"})
    assert r.status_code == 400 and r.get_json()["message"] == "target must be idle or navigation"
    assert stub.calls == [], "nothing reached the adapter"
    home = client.get("/home").get_data(as_text=True)
    assert 'id="nav-tracked"' not in home and "home-nav" not in home


def test_base_renders_the_run_tab_with_no_product_variable(tmp_path):
    """An undefined Jinja variable is falsy: a leftover `{% if trackless_ok %}` would hide Run."""
    from flask import render_template_string

    client, _ = app_for(merged(), tmp_path)
    app = client.application
    with app.test_request_context("/home"):
        body = render_template_string('{% extends "base.html" %}', page="home")
    assert '<a href="/run"' in body and ">Run<" in body
    src = open(os.path.join(app.root_path, app.template_folder, "base.html"), encoding="utf-8").read()
    assert "trackless_ok" not in src and "tracked_ok" not in src and "product" not in src


def test_the_adapter_state_carries_no_tracked_keys(monkeypatch):
    import threading

    from amr_web import adapter as ad

    a = ad.RosAdapter.__new__(ad.RosAdapter)
    a._lock = threading.Lock()
    a._now = lambda: 10.0
    for name in ("_mode", "_lease", "_panel", "_drives", "_mux", "_wheels", "_mapping", "_loc", "_run",
                 "_survey_move", "_scan_recv_t", "_ipc_temp", "_emg", "_emg_active_t"):
        setattr(a, name, None)
    for name in ("_mode_t", "_lease_t", "_panel_t", "_drives_t", "_mux_t", "_wheels_t", "_mapping_t",
                 "_loc_t", "_run_t", "_survey_move_t", "_started_t", "_emg_t"):
        setattr(a, name, 0.0)
    a._ipc_temp_t, a._diag = 10.0, {}
    a._disk_t, a._disk_cache = 10.0, {"free_mb": 1, "level": "ok", "path": "/"}
    st = a.state()
    assert "wheels" in st and "mux" in st
    for gone in ("line", "line_track", "rfid", "markers"):
        assert gone not in st, gone
    assert 7 not in ad.MODE_NAMES and 7 not in ad.MUX_NAMES
    assert not hasattr(ad, "LINE_NAMES") and not hasattr(ad, "RFID_RECENT")


def test_ipc_temperature_reads_coretemp_package_and_hottest_core(tmp_path):
    from amr_web.adapter import read_ipc_temp

    h = tmp_path / "hwmon" / "hwmon0"
    h.mkdir(parents=True)
    (tmp_path / "thermal").mkdir()
    (h / "name").write_text("coretemp\n")
    for i, (label, milli) in enumerate((("Package id 0", 66000), ("Core 0", 64000), ("Core 1", 71000)), 1):
        (h / f"temp{i}_label").write_text(label)
        (h / f"temp{i}_input").write_text(str(milli))
    assert read_ipc_temp(str(tmp_path)) == {"package_c": 66.0, "core_max_c": 71.0}
    assert read_ipc_temp(str(tmp_path / "missing")) is None


def test_supply_and_temperature_chips_sit_beside_net_on_every_page(tmp_path):
    client, _ = app_for(merged(), tmp_path)
    for page in ("/home", "/monitor", "/manual"):
        body = client.get(page).get_data(as_text=True)
        net = body.index('id="net"')
        assert net < body.index('id="supply"') < body.index('id="ipc-temp"'), page


def test_the_adapter_never_assigns_one_attribute_twice_in_init():
    """2026-10-09: the marker deque was named `_markers`, and the route-marker publisher
    created later in __init__ took the name - /api/state answered 500 on the vehicle while
    every unit test (which skips __init__) passed."""
    import ast
    import pathlib

    from amr_web import adapter as ad

    tree = ast.parse(pathlib.Path(ad.__file__).read_text())
    init = next(f for c in tree.body if isinstance(c, ast.ClassDef) and c.name == "RosAdapter"
                for f in c.body if isinstance(f, ast.FunctionDef) and f.name == "__init__")
    names = []
    for node in ast.walk(init):
        targets = node.targets if isinstance(node, ast.Assign) else [node.target] if isinstance(
            node, ast.AnnAssign) else []
        for t in targets:
            for e in (t.elts if isinstance(t, ast.Tuple) else [t]):
                if isinstance(e, ast.Attribute) and isinstance(e.value, ast.Name) and e.value.id == "self":
                    names.append(e.attr)
    dup = sorted({n for n in names if names.count(n) > 1})
    assert not dup, f"assigned twice in RosAdapter.__init__: {dup}"


# ---- route wait steps (gls-wait-coding-plan §A.5, 2026-10-30) ---------------------------


def _waiting(**over):
    run = {"state_name": "WAITING", "wait_name": "UBC hitch", "mission_id": "m1", "step_index": 1,
           "step_id": "w1", "continue_refused": "", "reason": "waiting at UBC hitch"}
    run.update(over)
    return merged(run=run)


def test_a_waiting_run_says_press_continue_and_offers_no_web_button():
    st = _waiting()
    rows = view.standing(st)
    h = view.headline(st, rows)
    assert h["state"] == f"{view.WAITING} · UBC hitch" and "Continue" in h["action"]
    assert h["code"] == "RUN_WAITING" and h["button"] == "", "only the physical button releases a wait (W4)"
    assert h["run_active"] and "step 2" in h["detail"]


def test_a_refused_continue_is_the_row_the_operator_reads_first():
    st = _waiting(continue_refused="0.25 m / 0.0 deg off the wait pose; reposition or abort")
    h = view.headline(st, view.standing(st))
    assert h["code"] == "CONTINUE_REFUSED" and h["state"].startswith(view.WAITING)
    assert [r["code"] for r in view.standing(st)][:2] == ["CONTINUE_REFUSED", "RUN_WAITING"]


def test_the_status_and_commissioning_pages_are_gone(tmp_path):
    """2026-10-30: Status folded into Home (mode, reason, fault, Return to idle, Recover) and
    Monitor (map, readiness, standing codes, Restart); Commission removed with pp. The blind
    run came back on 2026-11-02 as its own page, /blindrun (PV only), not as /commissioning."""
    client, _ = app_for(merged(), tmp_path)
    for path in ("/status", "/commissioning"):
        assert client.get(path).status_code == 404, path
    assert client.get("/blindrun").status_code == 200
    home = client.get("/home").get_data(as_text=True)
    for el in ("home-mode", "home-reason", "home-fault", "home-idle", "home-recover"):
        assert f'id="{el}"' in home, el
    mon = client.get("/monitor").get_data(as_text=True)
    for el in ('id="st-active"', 'id="st-standing"', 'id="st-ready"', 'id="mon"'):
        assert el in mon, el
