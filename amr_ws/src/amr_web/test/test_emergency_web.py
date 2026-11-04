"""The web side of the emergency suite (manuals/emergency-suite-plan.md §5, §7, owner answers
2026-10-30): one emergency picture from PanelState, its row first, a red banner on every
page, motion requests refused with 409 - and no web Reset, ever (E4)."""

import json
import pathlib
import re

from agv_core import alarms as cat
from amr_web.server import create_app

from amr_web import alarms as view
from amr_web import role

ROOT = pathlib.Path(__file__).resolve().parents[1] / "amr_web"
PAGES = (
    "/home",
    "/manual",
    "/maps",
    "/editor",
    "/review",
    "/run",
    "/monitor",
    "/io",
    "/alarms",
    "/params",
    "/blindrun",
)
SOS = "Switch on disabled"
READY = {
    "mode": {"mode_name": "IDLE", "fault_code": "", "reason": "", "base_ready": True, "generation": 1},
    "mode_age_s": 0.1,
    "panel": {"valid": True, "mode_auto": False, "age_s": 0.05},
    "emergency": {"state": 0, "state_name": "NORMAL", "estop_ok": True, "since_s": 60.0, "active_s": None},
    "drives": {"operational": True, "left": "Operation enabled", "right": "Operation enabled", "age_s": 0.05},
    "mux": {"inhibited": False, "source": "none", "reason": "", "code": "NO_SOURCE", "age_s": 0.05},
    "run": None,
    "localization": None,
}


def st(emg=0, estop_ok=None, **over):
    s = json.loads(json.dumps(READY))
    s["emergency"].update(
        state=emg,
        state_name=view.EMG_NAMES[emg],
        estop_ok=(emg not in (1,)) if estop_ok is None else estop_ok,
        since_s=3.0,
        active_s=None if emg == 0 else 42.0,
    )
    if emg:
        s["drives"] = {"operational": False, "left": SOS, "right": SOS, "age_s": 0.05}
        s["mux"] = {"inhibited": True, "source": "none", "reason": "E-stop", "code": "ESTOP", "age_s": 0.05}
    s.update(over)
    return s


class Stub:
    def __init__(self, state):
        self._state = state
        self.calls = []
        self.published = []

    def state(self):
        return self._state

    def supervisor_identity(self):
        return "inst", 1

    def manual_allowed(self):
        return True, ""

    def manual_publish(self, cmd):
        self.published.append(cmd)

    def events(self, since=0):
        return []

    def diagnostics(self):
        return {}

    def __getattr__(self, name):
        def fn(*a):
            self.calls.append((name, a))
            return True, f"{name} ok"

        return fn


def client_for(state, tmp_path):
    maps = tmp_path / "maps"
    maps.mkdir(exist_ok=True)
    stub = Stub(state)
    app = create_app(stub, str(maps), state_dir=str(tmp_path / "state"))
    app.config["TESTING"] = True
    return app, app.test_client(), stub


def codes(s):
    return [r["code"] for r in view.standing(s)]


# ---- standing() and headline() --------------------------------------------------------


def test_each_emergency_state_puts_its_row_first():
    want = {1: "ESTOP", 2: "ESTOP_RESET_NEEDED", 3: "ESTOP_RESET_NEEDED", 4: "ESTOP_RESET_FAILED"}
    for emg, code in want.items():
        rows = view.standing(st(emg))
        assert rows[0]["code"] == code, emg
    assert view.standing(st(3))[0]["detail"] == view.RESETTING_DETAIL
    assert cat.CATALOGUE["ESTOP"].title == "EMERGENCY STOP"


def test_a_warn_level_press_reset_still_outranks_every_error():
    s = st(2, run={"state_name": "FAULT", "fault_code": "EXEC_OFF_PATH", "reason": "cross-track"})
    s["mode"].update(fault_code="MUX_ACK_TIMEOUT", reason="no ack")
    c = codes(s)
    assert c[0] == "ESTOP_RESET_NEEDED" and "EXEC_OFF_PATH" in c and "MUX_ACK_TIMEOUT" in c
    # even without a supervisor the panel's picture is still reported, first
    assert codes(dict(st(1), mode=None)) == ["ESTOP", "SUPERVISOR_DOWN"]


def test_the_same_stop_is_not_said_twice():
    """Drives in 'Switch on disabled', the mux's ESTOP inhibit and an inferred executor
    hold are all the E-stop: one row."""
    s = st(
        1,
        run={"state_name": "BLOCKED", "hold_cause": "estop", "fault_code": "ESTOP", "reason": "safety stop"},
    )
    assert codes(s) == ["ESTOP"]
    assert codes(st(2)) == ["ESTOP_RESET_NEEDED"]
    # with no emergency the drive row is the safety-chain reset, as before
    s = st(0, drives={"operational": False, "left": SOS, "right": SOS, "age_s": 0.05})
    assert codes(s) == ["SAFETY_RESET_NEEDED"]


def test_a_stale_or_invalid_panel_is_panel_stale_never_a_false_emergency():
    for panel in (
        {"valid": True, "mode_auto": False, "age_s": 3.0},
        {"valid": False, "mode_auto": False, "age_s": 0.05},
        None,
    ):
        s = st(1, panel=panel)
        s["mux"].update(code="PANEL_STALE", reason="no panel authority")
        c = codes(s)
        assert "PANEL_STALE" in c and not any(x.startswith("ESTOP") for x in c), panel
        assert view.banner(s) is None and view.emergency_refusal(s) == ""
        assert view.headline(s, view.standing(s))["state"] != view.EMERGENCY_STOP


def test_the_headline_is_emergency_stop_above_needs_service():
    s = st(1)
    s["mode"].update(mode_name="FAULT", fault_code="BASE_EXITED", reason="base died")
    h = view.headline(s, view.standing(s))
    assert h["state"] == view.EMERGENCY_STOP and h["level"] == "error"
    assert h["code"] == "ESTOP" and h["button"] == ""  # Reset is the physical button only
    assert view.headline(st(0), view.standing(st(0)))["state"] == view.READY


def test_after_normal_the_held_run_says_press_start_to_continue():
    """Owner policy 2026-10-30: the emergency held the run (it never ends it)."""
    held = {"state_name": "BLOCKED", "hold_cause": "estop", "reason": "safety stop (E-stop / safety chain): "
            "emergency stop", "mission_id": "m1", "step_index": 2, "step_id": "s3", "auto_resume": False}
    s = st(0, run=held)
    h = view.headline(s, view.standing(s))
    assert h["state"] == view.STOPPED_NEEDS_YOU and h["code"] == "ESTOP_HOLD"
    assert "press Start" in h["title"] and h["button"] == ""
    # while the emergency still stands, the hold is the same stop said twice: banner only
    s = st(1, run=held)
    assert [r["code"] for r in view.standing(s)].count("ESTOP_HOLD") == 0


def test_banner_steps_follow_the_state():
    b = view.banner(st(1))
    assert b["title"] == "EMERGENCY STOP" and b["since_s"] == 42.0  # since it left NORMAL
    assert [s["text"][:7] for s in b["steps"]] == ["Make th", "Release", "Press t"]
    assert [s["done"] for s in b["steps"]] == [False, False, False] and b["steps"][1]["now"]
    for emg in (2, 3):
        b = view.banner(st(emg))
        assert b["steps"][1]["done"] and b["steps"][2]["now"] and not b["failed"]
    assert view.banner(st(3))["note"] == view.RESETTING_DETAIL
    b = view.banner(st(4))
    assert b["failed"] and cat.CATALOGUE["ESTOP_RESET_FAILED"].action in b["failed_text"]
    assert view.banner(st(0)) is None


# ---- pages ----------------------------------------------------------------------------


def _banner_tag(html):
    m = re.search(r'<div id="emg-banner"[^>]*>', html)
    assert m, "no banner element"
    return m.group(0)


def test_the_banner_is_on_every_page_while_not_normal(tmp_path):
    _, client, _ = client_for(st(2), tmp_path)
    for rl in (role.OPERATOR, role.ENGINEER):
        client.post("/api/role", json={"role": rl, "pin": role.DEFAULT_PIN})
        for page in PAGES:
            html = client.get(page).get_data(as_text=True)
            tag = _banner_tag(html)
            assert "hidden" not in tag, page
            assert "Release (twist) the E-stop button." in html, page
            assert "Press the blue RESET button on the cabinet." in html, page
            assert 'id="emg-since">00:42<' in html, page
            assert html.index('id="emg-banner"') < html.index('class="shell"'), page  # under the nav


def test_the_banner_is_hidden_when_normal(tmp_path):
    _, client, _ = client_for(st(0), tmp_path)
    for page in PAGES:
        html = client.get(page).get_data(as_text=True)
        assert "hidden" in _banner_tag(html), page
        assert "Release (twist)" not in html, page


def test_api_state_carries_the_banner(tmp_path):
    _, client, _ = client_for(st(4), tmp_path)
    d = client.get("/api/state").get_json()
    assert d["banner"]["state_name"] == "RESET_FAILED" and d["emergency"]["state"] == 4
    _, client, _ = client_for(st(0), tmp_path)
    assert client.get("/api/state").get_json()["banner"] is None


def test_the_manual_pad_gives_the_banner_height_back():
    css = (ROOT / "static" / "amr.css").read_text()
    assert "calc(100vh - 11rem - var(--emg-h))" in css and "--emg-h:0px" in css
    js = (ROOT / "static" / "amr.js").read_text()
    assert "'--emg-h'" in js and "data.banner" in js


# ---- motion refused while not NORMAL --------------------------------------------------


def test_jog_and_run_start_are_refused_with_409(tmp_path):
    for emg in (1, 2, 3, 4):
        _, client, stub = client_for(st(emg), tmp_path)
        for path, body in (
            ("/api/manual/press", {"owner": "o1"}),
            ("/api/manual/refresh", {"session": "s", "ticket": "t", "seq": 1, "v": 0.1, "w": 0.0}),
            ("/api/mission/run", {"mission_id": "m1"}),
            ("/api/mission/resume", {}),
            ("/api/survey_move", {"kind": "straight", "value": 0.5}),
        ):
            r = client.post(path, json=body)
            assert r.status_code == 409, (emg, path)
            assert r.get_json()["message"].startswith("E-stop active"), (emg, path)
        assert not [c for c in stub.calls if c[0] in ("run_mission", "prepare_resume", "survey_move")]
        # a refused refresh revokes (zero, valid_for 0); nothing else is ever published
        assert all(c.v == 0.0 and c.w == 0.0 and c.valid_for_s == 0.0 for c in stub.published)


def test_the_same_requests_pass_when_normal(tmp_path):
    _, client, stub = client_for(st(0), tmp_path)
    assert client.post("/api/manual/press", json={"owner": "o1"}).status_code == 200
    assert client.post("/api/mission/run", json={"mission_id": "m1"}).status_code == 200
    assert client.post("/api/mission/resume", json={}).status_code == 200
    # a stale panel is not refused as an E-stop: the mux / lease hold it, in their words
    _, client, _ = client_for(st(1, panel={"valid": True, "mode_auto": False, "age_s": 9.0}), tmp_path)
    assert client.post("/api/mission/run", json={"mission_id": "m1"}).status_code == 200


def test_pages_grey_their_motion_controls_from_the_banner():
    jog = (ROOT / "static" / "jogpad.js").read_text()
    assert "E-stop active" in jog and "st.banner" in jog
    run = (ROOT / "static" / "run.js").read_text()
    assert "st.banner" in run and "'btn-run-load', 'btn-run-resume'" in run
    io = (ROOT / "templates" / "io.html").read_text()
    assert "data-estop-ch" in io and "emergency" in io


# ---- no web Reset (owner answer E4) ---------------------------------------------------


def test_there_is_no_web_reset_route_or_button(tmp_path):
    app, _, _ = client_for(st(2), tmp_path)
    resets = {r.rule for r in app.url_map.iter_rules() if re.search(r"reset|estop|emergenc", r.rule, re.I)}
    assert resets == {"/api/localization/reset"}, resets  # the localisation monitor, not the E-stop
    for f in [*(ROOT / "templates").glob("*.html"), *(ROOT / "static").glob("*.js")]:
        text = f.read_text()
        for b in re.findall(r"<button\b[^>]*>(.*?)</button>", text, re.S):
            words = re.sub(r"<[^>]+>", " ", b).strip()
            assert not re.fullmatch(r"(?i)\s*reset\s*", words), f"{f.name}: a bare Reset button"
            assert not re.search(r"(?i)(e-?stop|emergency).*reset|reset.*(e-?stop|emergency)", words), f.name
        assert not re.search(r"api/(estop|emergency|panel)/?reset|api/reset", text), f.name


# ---- adapter ----------------------------------------------------------------------------


def test_the_adapter_tracks_when_the_emergency_state_changed():
    import threading
    import types

    from amr_web import adapter as ad

    a = ad.RosAdapter.__new__(ad.RosAdapter)
    a._lock = threading.Lock()
    a._emg, a._emg_t, a._emg_active_t = None, 0.0, None
    clock = [100.0]
    a._now = lambda: clock[0]

    def panel(emg, ok):
        a._on_panel(types.SimpleNamespace(valid=True, mode_auto=False, emergency=emg, estop_ok=ok))

    panel(0, True)
    assert a._emg == {"state": 0, "state_name": "NORMAL", "estop_ok": True} and a._emg_active_t is None
    clock[0] = 110.0
    panel(1, False)
    clock[0] = 115.0
    panel(1, False)  # same state: the clock does not restart
    assert (a._emg_t, a._emg_active_t) == (110.0, 110.0)
    clock[0] = 120.0
    panel(2, True)
    assert a._emg["state_name"] == "RESET_NEEDED" and a._emg_t == 120.0 and a._emg_active_t == 110.0
    clock[0] = 130.0
    panel(0, True)
    assert a._emg_active_t is None and a._emg_t == 130.0
