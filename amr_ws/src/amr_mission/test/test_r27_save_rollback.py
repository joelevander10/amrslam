"""Review R27: a save that fails before staging (e.g. unwritable maps dir) must end in
RETURN_REVIEW with a failed answer, not leave the coordinator stuck in SAVING."""

import types

import pytest
from amr_mission.mapping_session_node import MappingSession

from amr_interfaces.msg import MappingState
from amr_interfaces.srv import SaveMap
from amr_mission import map_bundle as mb


class Log:
    def error(self, *_):
        pass


def test_revision_allocation_failure_returns_to_review(monkeypatch, tmp_path):
    def boom(*_a, **_k):
        raise PermissionError("maps dir not writable")

    monkeypatch.setattr(mb, "next_revision", boom)
    sv = types.SimpleNamespace(
        _state=MappingState.RETURN_REVIEW, maps_dir=str(tmp_path), _map_id="hall", states=[], paused=[]
    )
    sv._readiness = lambda: []
    sv.get_logger = Log

    def _set(state, message):
        sv._state, sv._message = state, message
        sv.states.append(state)

    sv._set = _set
    sv._pause_slam = lambda p: sv.paused.append(p)
    res = MappingSession._srv_save(sv, SaveMap.Request(note="n"), SaveMap.Response())
    assert not res.ok and "not writable" in res.message and "nothing was staged" in res.message
    assert sv.states == [MappingState.SAVING, MappingState.RETURN_REVIEW]
    assert sv.paused == [False]  # tracked toggle: a no-op when SLAM was never paused
    assert not any(p.name.startswith(".") for p in tmp_path.rglob("*"))


if __name__ == "__main__":
    pytest.main([__file__])
