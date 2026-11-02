"""W2: role logs that cannot eat the disk, and per-generation state that does not pile up.

`base.log` was 43 MB on the vehicle (2026-09-22) holding every service start since the
install, and `~/.amr` held a costmap footprint per control generation for ever. A full
disk is a lost-write problem like a power cut, which is why it sits in this plan.
"""

import os
import types

from amr_bringup import logs
from amr_bringup.process_supervisor import Group


def test_a_spawn_starts_a_fresh_log_and_keeps_the_last_five(tmp_path):
    p = tmp_path / "base.log"
    for run in range(8):
        p.write_text(f"run {run}\n")
        logs.rotate(str(p))
    assert not p.exists()  # rotated away, ready for the new child to create
    assert (tmp_path / "base.log.1").read_text() == "run 7\n"  # newest kept
    assert (tmp_path / "base.log.5").read_text() == "run 3\n"  # oldest kept
    assert not (tmp_path / "base.log.6").exists()  # and nothing beyond `keep`


def test_rotating_a_log_that_is_not_there_yet_is_not_an_error(tmp_path):
    logs.rotate(str(tmp_path / "never-written.log"))
    assert list(tmp_path.iterdir()) == []


def test_a_live_log_is_truncated_with_a_marker_and_children_keep_appending(tmp_path):
    p = tmp_path / "base.log"
    p.write_text("x" * 2048)
    # a child holds the file open in append mode, exactly as Group.spawn does
    child = open(p, "ab", buffering=0)  # noqa: SIM115
    try:
        assert logs.cap(str(p), max_bytes=1024) is True
        child.write(b"after the cap\n")  # O_APPEND: lands at the NEW end, not at offset 2048
        body = p.read_text()
        assert body.startswith("[") and "truncated here" in body
        assert body.endswith("after the cap\n")
        assert len(body) < 200  # no 2 kB hole
    finally:
        child.close()
    assert logs.cap(str(p), max_bytes=1024) is False  # under the cap now: left alone


def test_capping_scans_only_live_logs(tmp_path):
    (tmp_path / "base.log").write_text("y" * 200)
    (tmp_path / "web.log").write_text("z" * 10)
    (tmp_path / "base.log.1").write_text("y" * 200)  # a rotated file is evidence: never touched
    assert logs.cap_dir(str(tmp_path), max_bytes=100) == ["base.log"]
    assert len((tmp_path / "base.log.1").read_text()) == 200


def test_the_generation_sweep_keeps_only_the_live_layers_file(tmp_path):
    for gen in (0, 2, 3, 7):
        (tmp_path / f"costmap_footprint_gen{gen}.yaml").write_text("polygon: []")
    (tmp_path / "operations.jsonl").write_text("{}")
    assert logs.sweep_generation_files(str(tmp_path), keep_generation=3) == [
        "costmap_footprint_gen0.yaml",
        "costmap_footprint_gen2.yaml",
        "costmap_footprint_gen7.yaml",
    ]
    assert (tmp_path / "costmap_footprint_gen3.yaml").exists()
    assert (tmp_path / "operations.jsonl").exists()  # nothing else is swept


def test_at_boot_every_generation_file_is_stale(tmp_path):
    (tmp_path / "costmap_footprint_gen4.yaml").write_text("polygon: []")
    assert logs.sweep_generation_files(str(tmp_path)) == ["costmap_footprint_gen4.yaml"]
    assert list(tmp_path.iterdir()) == []


def test_a_missing_state_dir_sweeps_nothing_and_raises_nothing(tmp_path):
    assert logs.sweep_generation_files(str(tmp_path / "gone")) == []
    assert logs.cap_dir(str(tmp_path / "gone")) == []


def test_group_spawn_rotates_before_it_opens(tmp_path, monkeypatch):
    """The rotation must happen in spawn(), not in the caller: every path that starts a
    child (boot, layer transition, web respawn) then gets a fresh file."""
    p = tmp_path / "web.log"
    p.write_text("previous run\n")
    seen = {}

    class FakeProc:
        pid = 4321

        def poll(self):
            return None

    def fake_popen(argv, **kw):
        seen["stdout_size"] = os.fstat(kw["stdout"].fileno()).st_size
        return FakeProc()

    monkeypatch.setattr("amr_bringup.process_supervisor.subprocess.Popen", fake_popen)
    Group.spawn("web", ["ros2", "launch", "x"], env={}, log_path=str(p))
    assert seen["stdout_size"] == 0  # the child writes into an empty file
    assert (tmp_path / "web.log.1").read_text() == "previous run\n"


def test_the_supervisor_sweeps_at_boot_and_per_layer(tmp_path):
    """Source check: both call sites must exist, since the boot sweep clears everything
    and the launch sweep protects a running layer's own file."""
    import pathlib  # noqa: PLC0415

    src = (pathlib.Path(__file__).resolve().parents[1] / "amr_bringup" / "supervisor_node.py").read_text()
    assert "logs.sweep_generation_files(self.state_dir)" in src  # boot: all of them
    assert "logs.sweep_generation_files(self.state_dir, self.generation)" in src  # launch: keep live
    assert "logs.cap_dir(self._logs)" in src


def test_types_are_what_the_supervisor_passes():
    """cap_dir/sweep take plain paths; a Namespace slipping through would be silent."""
    ns = types.SimpleNamespace(_logs="/nonexistent")
    assert logs.cap_dir(ns._logs) == []
