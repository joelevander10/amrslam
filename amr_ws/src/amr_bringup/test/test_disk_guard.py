"""W3: the disk guard. A save that runs out of space mid-bundle is the ugliest
failure in the stack, so new work is refused early - but a vehicle that is already
moving is never stopped over free space."""

import types

from agv_core import disk

from amr_bringup import mode_fsm as fsm
from amr_bringup import readiness as rd
from amr_bringup.supervisor_node import Supervisor


class Log:
    def info(self, *_):
        pass

    warn = error = info


def fake_usage(mapping):
    def usage(path):
        return types.SimpleNamespace(free=mapping[path] * 1024 * 1024, total=0, used=0)

    return usage


def test_the_levels_are_free_space_thresholds(monkeypatch):
    monkeypatch.setattr(disk.shutil, "disk_usage", fake_usage({"/s": 5000}))
    assert disk.status(["/s"]).level == disk.OK
    monkeypatch.setattr(disk.shutil, "disk_usage", fake_usage({"/s": 800}))
    assert disk.status(["/s"]).level == disk.WARN
    monkeypatch.setattr(disk.shutil, "disk_usage", fake_usage({"/s": 150}))
    st = disk.status(["/s"])
    assert st.level == disk.STOP and st.blocks_new_work and st.free_mb == 150


def test_the_fullest_filesystem_wins(monkeypatch):
    monkeypatch.setattr(disk.shutil, "disk_usage", fake_usage({"/state": 5000, "/maps": 300}))
    st = disk.status(["/state", "/maps"])
    assert st.path == "/maps" and st.level == disk.WARN


def test_an_unreadable_path_never_blocks_the_vehicle(monkeypatch):
    def boom(_path):
        raise OSError("no such filesystem")

    monkeypatch.setattr(disk.shutil, "disk_usage", boom)
    st = disk.status(["/gone"])
    assert st.level == disk.UNKNOWN and not st.blocks_new_work  # reported, not a refusal (audit R19)


def test_readiness_still_exposes_the_helper():
    """The supervisor reads it through readiness; the web through agv_core directly."""
    assert rd.disk_status is disk.status
    assert (rd.DISK_OK, rd.DISK_WARN, rd.DISK_STOP) == (disk.OK, disk.WARN, disk.STOP)


def sv():
    s = types.SimpleNamespace()
    s.state_dir, s.maps_dir = "/state", "/maps"
    s.events = []
    s._event = lambda level, code, text: s.events.append((code, text))
    s.get_logger = Log
    s.disk = disk.DiskStatus(-1, disk.OK, "")
    s._disk_level = disk.OK
    s._check_disk = types.MethodType(Supervisor._check_disk, s)
    return s


def test_the_supervisor_reports_each_edge_once(monkeypatch):
    s = sv()
    monkeypatch.setattr(disk.shutil, "disk_usage", fake_usage({"/state": 5000, "/maps": 5000}))
    s._check_disk()
    assert s.events == []  # healthy: silence
    monkeypatch.setattr(disk.shutil, "disk_usage", fake_usage({"/state": 500, "/maps": 5000}))
    s._check_disk()
    s._check_disk()
    assert [c for c, _ in s.events] == ["DISK_LOW"]  # the EDGE, not every poll
    monkeypatch.setattr(disk.shutil, "disk_usage", fake_usage({"/state": 100, "/maps": 5000}))
    s._check_disk()
    assert s.events[-1][0] == "DISK_FULL" and "100 MB free" in s.events[-1][1]
    monkeypatch.setattr(disk.shutil, "disk_usage", fake_usage({"/state": 9000, "/maps": 9000}))
    s._check_disk()
    assert "no longer low" in s.events[-1][1]


def survey_sv(level):
    s = types.SimpleNamespace()
    s.disk = disk.DiskStatus(120, level, "/maps")
    s.mode = fsm.IDLE
    s.admit_params = None
    s._lock = __import__("threading").RLock()
    s.book = types.SimpleNamespace(get=lambda _i: None, _by_request={})
    s._check_expected = lambda i, g: ""
    s._conditions = lambda: fsm.Conditions(now=1.0)
    s._srv_survey = types.MethodType(Supervisor._srv_survey, s)
    return s


def test_a_full_disk_refuses_a_new_survey_with_the_number_in_the_message(monkeypatch):
    from amr_interfaces.srv import RequestSurvey  # noqa: PLC0415

    monkeypatch.setattr(fsm, "admit", lambda *a, **k: types.SimpleNamespace(ok=True, reason=""))
    s = survey_sv(disk.STOP)
    req = RequestSurvey.Request(
        request_id="r1",
        expected_instance="",
        expected_generation=0,
        operation=RequestSurvey.Request.START,
        map_id="hall",
    )
    res = s._srv_survey(req, RequestSurvey.Response())
    assert not res.accepted and "disk full: 120 MB free" in res.message


def test_a_low_disk_still_lets_a_survey_start(monkeypatch):
    from amr_interfaces.srv import RequestSurvey  # noqa: PLC0415

    monkeypatch.setattr(fsm, "admit", lambda *a, **k: types.SimpleNamespace(ok=True, reason=""))
    s = survey_sv(disk.WARN)
    op = types.SimpleNamespace(operation_id="op-1")
    s.book.submit = lambda *a, **k: (op, True)
    s._begin_transaction = lambda *a, **k: None
    s._start_survey = lambda *a, **k: None
    req = RequestSurvey.Request(
        request_id="r2",
        expected_instance="",
        expected_generation=0,
        operation=RequestSurvey.Request.START,
        map_id="hall",
    )
    res = s._srv_survey(req, RequestSurvey.Response())
    assert "disk full" not in res.message


def test_running_work_is_never_gated_on_the_disk():
    """A full disk must not stop a vehicle that is already moving: only START and SAVE
    are refused, and the source is the place to prove it."""
    import pathlib  # noqa: PLC0415

    src = (pathlib.Path(__file__).resolve().parents[1] / "amr_bringup" / "supervisor_node.py").read_text()
    guard = src.split("self.disk.blocks_new_work", 1)[0].splitlines()[-3:]
    assert "REQ_SURVEY_START" in "".join(guard) and "REQ_SURVEY_SAVE" in "".join(guard)
    assert "REQ_NAVIGATION" not in "".join(guard) and "REQ_LINE" not in "".join(guard)
