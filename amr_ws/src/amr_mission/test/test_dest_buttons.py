"""Destination buttons in the executor (panel.destinations, DI10 UBC / DI11 ITG / DI12 PISTON;
owner 2026-11-03). A press in AUTO with no run under way loads the newest saved mission for the
button's route on the active map and starts it as Start would. Built with __new__ like
test_wait_step; the mission load itself is the service's code path, stubbed here."""

from amr_navigation.compiler import STRAIGHT, CompiledStep
from test_route_executor_logic import _clock_stub, make_node

from amr_interfaces.msg import PanelState
from amr_mission import route_executor_node as ren
from amr_mission import run_fsm as fsm
from amr_navigation import store

DESTS = (("UBC", 10, ""), ("ITG", 11, "route-itg-02"), ("PISTON", 12, "route-piston"))


def straight(sid, x0, x1):
    n = round((x1 - x0) / 0.1)
    return CompiledStep(
        sid, STRAIGHT, (x0, 0.0, 0.0), (x1, 0.0, 0.0), length_m=x1 - x0,
        samples=[(x0 + 0.1 * k, 0.0, 0.0) for k in range(n + 1)], v_mps=0.55,
    )


def node(monkeypatch, tmp_path, state="idle"):
    monkeypatch.setattr(ren.vehicle_config, "PANEL_DESTINATIONS", DESTS)
    n = make_node([straight("s1", 0.0, 2.0)])
    _clock_stub(n)
    n.maps_dir = str(tmp_path)
    n.active_map = ("kyb-map-02", 3, "sha-map3")
    n.notes = []
    n._note = lambda level, code, text: n.notes.append((code, text))
    n.loaded = []

    def load(mission_id):
        n.loaded.append(mission_id)
        assert n.fsm.load(mission_id, 1)
        return True, n.fsm.reason

    n._load_mission = load
    n.premoves = []
    n._begin_premove = lambda: n.premoves.append(n.fsm.run_id)
    if state == "idle":
        n.fsm = fsm.RunFsm()
    return n


def save(tmp_path, route_id, rrev, map_rev=3, sha="sha-map3", labels=None, mid=None):
    mid = mid or f"kyb-map-02_rev{map_rev}_{route_id}_rev{rrev}" + (f"_labels{labels}" if labels else "")
    lab = {"revision": labels, "sha256": "l"} if labels else None
    store.save_mission(str(tmp_path), mid, "kyb-map-02", map_rev, sha, route_id, rrev, "r", lab)
    return mid


def press(n, k, auto=True, emergency=PanelState.EMG_NORMAL):
    m = PanelState()
    m.valid, m.mode_auto, m.emergency = True, auto, emergency
    m.estop_ok = emergency == PanelState.EMG_NORMAL
    m.dest_edge = k
    n._on_panel(m)


def test_itg_loads_the_newest_mission_on_the_active_map_and_starts(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path)
    save(tmp_path, "route-itg-02", 1)
    newest = save(tmp_path, "route-itg-02", 2)
    save(tmp_path, "route-itg-02", 9, map_rev=2, sha="sha-map2")  # another map revision
    save(tmp_path, "route-itg-01", 5)  # another route
    press(n, 2)
    assert n.loaded == [newest]
    assert n.fsm.state == fsm.EXECUTING and n.premoves, "started with the pre-move warning"
    assert n.fsm.reason.startswith("ITG: run ")
    assert n.notes == []


def test_a_newer_label_set_wins_on_the_same_route_revision(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path)
    save(tmp_path, "route-itg-02", 1)
    with_labels = save(tmp_path, "route-itg-02", 1, labels=2)
    assert n._dest_mission("route-itg-02") == (with_labels, "")


def test_a_ready_or_done_mission_is_replaced_by_the_press(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path, state="ready")
    assert n.fsm.state == fsm.READY and n.fsm.mission_id == "m1"
    mid = save(tmp_path, "route-itg-02", 1)
    press(n, 2)
    assert n.loaded == [mid] and n.fsm.state == fsm.EXECUTING


def test_manual_refuses_and_says_why(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path)
    save(tmp_path, "route-itg-02", 1)
    press(n, 2, auto=False)
    assert n.loaded == [] and n.fsm.state == fsm.IDLE
    assert "selector is not AUTO" in n.fsm.reason
    assert n.notes and n.notes[0][0] == "DEST_REFUSED"


def test_a_button_with_no_route_or_no_mission_is_refused(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path)
    press(n, 1)
    assert "UBC: no route assigned" in n.fsm.reason and n.loaded == []
    press(n, 3)
    assert "no saved mission for route route-piston on map kyb-map-02 rev3" in n.fsm.reason
    assert n.fsm.state == fsm.IDLE and [c for c, _ in n.notes] == ["DEST_REFUSED"] * 2


def test_off_the_start_pose_loads_but_does_not_start(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path)
    save(tmp_path, "route-itg-02", 1)
    n._pose = lambda: (0.5, 0.0, 0.0)
    press(n, 2)
    assert n.fsm.state == fsm.READY and not n.premoves
    assert n.fsm.reason.startswith("ITG: Start refused:") and "route start" in n.fsm.reason
    assert n.notes[-1][0] == "DEST_REFUSED"


def test_a_run_under_way_is_untouched(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path, state="ready")
    save(tmp_path, "route-itg-02", 1)
    assert n.fsm.start(True, True)
    for state in (fsm.EXECUTING, fsm.WAITING, fsm.PAUSED, fsm.BLOCKED, fsm.FAULT):
        n.fsm.state, n.fsm.reason = state, "the run's own reason"
        press(n, 2)
        assert n.fsm.state == state and n.fsm.reason == "the run's own reason"
    assert n.loaded == [] and len(n.notes) == 5


def test_an_emergency_refuses_the_press(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path)
    save(tmp_path, "route-itg-02", 1)
    press(n, 2, emergency=PanelState.EMG_ACTIVE)
    assert n.loaded == [] and n.fsm.state == fsm.IDLE and "E-stop active" in n.fsm.reason


def test_an_unknown_button_index_does_nothing(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path)
    press(n, 7)
    assert n.loaded == [] and n.fsm.state == fsm.IDLE and n.notes[0][0] == "DEST_REFUSED"


def test_an_unsupervised_stack_takes_missions_from_any_map(monkeypatch, tmp_path):
    n = node(monkeypatch, tmp_path)
    n.active_map = ("", 0, "")
    mid = save(tmp_path, "route-itg-02", 4, map_rev=2, sha="x")
    assert n._dest_mission("route-itg-02") == (mid, "")


def test_a_second_press_starts_the_ready_mission_without_reloading(monkeypatch, tmp_path):
    """Vehicle 2026-11-03: a reload is ~3 s, so a press after a start-gate refusal starts the
    mission already READY instead of loading it again."""
    n = node(monkeypatch, tmp_path)
    mid = save(tmp_path, "route-itg-02", 1)
    n._pose = lambda: (0.5, 0.0, 0.0)
    press(n, 2)
    assert n.loaded == [mid] and n.fsm.state == fsm.READY
    n._pose = lambda: (0.0, 0.0, 0.0)
    press(n, 2)
    assert n.loaded == [mid], "not reloaded"
    assert n.fsm.state == fsm.EXECUTING and n.premoves
    newer = save(tmp_path, "route-itg-02", 2)
    n.fsm.state = fsm.DONE
    press(n, 2)
    assert n.loaded == [mid, newer], "a newer mission, or a finished run, loads again"


def test_the_web_destination_button_is_the_pushbutton(monkeypatch, tmp_path):
    """AMR QR POS 1..6 (2026-11-06): /amr/destination with web_destinations on does exactly
    what the destination's pushbutton does; off (gvievo-01) it refuses."""
    from amr_interfaces.srv import Destination

    n = node(monkeypatch, tmp_path)
    newest = save(tmp_path, "route-itg-02", 2)
    req, res = Destination.Request(), Destination.Response()
    req.index = 2
    out = n._srv_destination(req, res)
    assert not out.ok and "off on this vehicle" in out.message and n.loaded == []
    n.web_destinations = True
    n._panel = PanelState()
    n._panel.valid, n._panel.mode_auto, n._panel.estop_ok = True, False, True
    n._panel_t = n.clock[0]
    out = n._srv_destination(req, Destination.Response())
    assert not out.ok and "not AUTO" in out.message  # MANUAL: refused with the reason
    n._panel.mode_auto = True
    out = n._srv_destination(req, Destination.Response())
    assert out.ok and n.loaded == [newest] and n.fsm.state == fsm.EXECUTING
    req.index = 9
    out = n._srv_destination(req, Destination.Response())
    assert not out.ok and "no destination 9" in out.message
