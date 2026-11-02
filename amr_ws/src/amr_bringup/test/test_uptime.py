"""W1: the marker that tells a power cut from a crash from a clean stop.

The marker is the ONLY evidence a power cut leaves behind, so these tests are about
what the operator is told, and about the one ordering rule that makes the evidence
true: the marker is cleared last, after the children are down.
"""

import json
import os
import pathlib
import re
import time

from amr_bringup import uptime


def test_a_clean_stop_leaves_no_verdict(tmp_path):
    uptime.write(str(tmp_path), "inst", "IDLE")
    assert os.path.exists(uptime.path(str(tmp_path)))
    uptime.clear(str(tmp_path))
    assert uptime.verdict(str(tmp_path)) is None
    uptime.clear(str(tmp_path))  # clearing twice is not an error


def test_a_marker_from_before_this_boot_is_a_power_loss(tmp_path):
    uptime.write(str(tmp_path), "inst", "NAVIGATION", "op-7", "line-a-to-b")
    now = time.time()
    v = uptime.verdict(str(tmp_path), now_wall=now, boot_wall=now + 10)  # boot AFTER the write
    assert v.kind == uptime.POWER_LOSS
    assert v.mode == "NAVIGATION" and v.run_id == "line-a-to-b"
    s = v.sentence()
    assert "Power was lost" in s and "mission line-a-to-b" in s
    assert "switched off and on" in s  # the drives latched 8130h: say the action


def test_a_marker_from_this_boot_is_a_crash_and_needs_no_drive_action(tmp_path):
    uptime.write(str(tmp_path), "inst", "IDLE")
    now = time.time()
    v = uptime.verdict(str(tmp_path), now_wall=now, boot_wall=now - 3600)  # boot BEFORE the write
    assert v.kind == uptime.SERVICE_CRASH
    assert "restarted itself" in v.sentence() and "power" not in v.sentence().lower()


def test_an_operation_is_named_when_no_mission_was_running(tmp_path):
    uptime.write(str(tmp_path), "i", "MAPPING", "op-save-3")
    v = uptime.verdict(str(tmp_path), boot_wall=time.time() + 1)
    assert "operation op-save-3" in v.sentence()


def test_a_corrupt_or_missing_marker_invents_nothing(tmp_path):
    assert uptime.verdict(str(tmp_path)) is None  # missing
    pathlib.Path(uptime.path(str(tmp_path))).write_text('{"written_at": ')  # cut mid-write
    assert uptime.verdict(str(tmp_path)) is None
    pathlib.Path(uptime.path(str(tmp_path))).write_text("{}")  # valid json, no timestamp
    assert uptime.verdict(str(tmp_path)) is None


def test_writing_never_raises_on_a_bad_state_dir(tmp_path):
    bad = tmp_path / "file"
    bad.write_text("not a directory")
    uptime.write(str(bad / "under-a-file"), "i", "IDLE")  # must not raise
    assert uptime.verdict(str(bad / "under-a-file")) is None


def test_the_marker_is_atomic_and_leaves_no_tmp_behind(tmp_path):
    uptime.write(str(tmp_path), "i", "IDLE")
    uptime.write(str(tmp_path), "i", "NAVIGATION")
    assert sorted(p.name for p in tmp_path.iterdir()) == [uptime.MARKER]
    body = json.loads(pathlib.Path(uptime.path(str(tmp_path))).read_text())
    assert body["mode"] == "NAVIGATION" and body["pid"] == os.getpid()


def test_boot_time_is_readable_on_this_machine():
    bt = uptime.boot_time()
    assert bt is not None and 0 < bt < time.time()


def test_the_marker_is_cleared_last_in_shutdown():
    """If anything ran after the clear and hung, a kill would look like a clean stop."""
    src = (pathlib.Path(__file__).resolve().parents[1] / "amr_bringup" / "supervisor_node.py").read_text()
    body = src.split("def shutdown(self)", 1)[1].split("\ndef ", 1)[0]
    lines = [ln.strip() for ln in body.splitlines() if ln.strip() and not ln.strip().startswith("#")]
    assert lines[-1] == "uptime.clear(self.state_dir)", lines[-3:]


def test_the_supervisor_reports_ready_to_systemd_whatever_the_boot_outcome():
    """W5's caveat: READY=1 must not sit inside a success branch, or systemd kills the
    service at TimeoutStartSec and the late-base recovery path never runs."""
    src = (pathlib.Path(__file__).resolve().parents[1] / "amr_bringup" / "supervisor_node.py").read_text()
    loop = src.split("def _loop(self)", 1)[1].split("\n    def ", 1)[0]
    ready = re.search(r"^(\s*)self\._notify\.ready\(\)", loop, re.M)
    assert ready, "the supervisor never tells systemd it is ready"
    assert len(ready.group(1)) == 8, "ready() is nested in a branch; it must run on every path"


def test_a_boot_event_waits_for_a_listener():
    """/amr/events is volatile and the web is spawned BY the supervisor: publishing the
    power-loss verdict at boot put it in an empty room (caught in sim, 2026-09-22)."""
    import types  # noqa: PLC0415

    from amr_bringup.supervisor_node import Supervisor  # noqa: PLC0415

    sv = types.SimpleNamespace()
    sv.published, sv.logged, sv._deferred, sv.now = [], [], [], 100.0
    sv.subs = 0
    sv._now = lambda: sv.now
    sv._event = lambda lvl, code, text: sv.published.append(code)
    sv._pub_event = types.SimpleNamespace(get_subscription_count=lambda: sv.subs)
    sv.get_logger = lambda: types.SimpleNamespace(warn=lambda s: sv.logged.append(s), info=print, error=print)
    for name in ("_event_when_heard", "_flush_deferred"):
        setattr(sv, name, types.MethodType(getattr(Supervisor, name), sv))

    sv._event_when_heard(2, "UNCLEAN_SHUTDOWN", "power was lost")
    assert sv.published == [] and len(sv._deferred) == 1  # nobody listening yet: held
    sv.now += 5.0
    sv._flush_deferred()
    assert sv.published == []
    sv.subs = 1  # the web finished starting and subscribed
    sv._flush_deferred()
    assert sv.published == ["UNCLEAN_SHUTDOWN"] and sv._deferred == []
    sv._flush_deferred()
    assert sv.published == ["UNCLEAN_SHUTDOWN"]  # exactly once


def test_a_boot_event_is_given_up_on_rather_than_held_for_ever():
    import types  # noqa: PLC0415

    from amr_bringup.supervisor_node import DEFER_EVENT_S, Supervisor  # noqa: PLC0415

    sv = types.SimpleNamespace()
    sv.published, sv.logged, sv._deferred, sv.now = [], [], [], 0.0
    sv._now = lambda: sv.now
    sv._event = lambda lvl, code, text: sv.published.append(code)
    sv._pub_event = types.SimpleNamespace(get_subscription_count=lambda: 0)
    sv.get_logger = lambda: types.SimpleNamespace(warn=lambda s: sv.logged.append(s))
    for name in ("_event_when_heard", "_flush_deferred"):
        setattr(sv, name, types.MethodType(getattr(Supervisor, name), sv))

    sv._event_when_heard(1, "BOOT_READY", "ready")
    sv.now = DEFER_EVENT_S + 1
    sv._flush_deferred()
    assert sv.published == [] and sv._deferred == []
    assert "BOOT_READY not delivered" in sv.logged[0]  # it still reaches the log
