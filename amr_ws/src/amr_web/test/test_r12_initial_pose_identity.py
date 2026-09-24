"""Review R12: an initial pose drawn on one map must never seed localization on another."""

from amr_web.adapter import initial_pose_mismatch

MODE = {
    "mode_name": "NAVIGATION",
    "active_map_id": "hall_a",
    "active_map_revision": 3,
    "active_map_sha256": "abc",
    "generation": 12,
}


def test_pose_on_the_active_map_under_the_same_generation_is_applied():
    assert initial_pose_mismatch(MODE, "hall_a", 3, 12, "abc") == ""
    assert initial_pose_mismatch(MODE, "hall_a", 3, 12) == ""  # no sha offered: id + rev + generation


def test_pose_drawn_on_another_map_with_the_same_frame_is_refused():
    assert "hall_b rev3" in initial_pose_mismatch(MODE, "hall_b", 3, 12)
    assert "rev2" in initial_pose_mismatch(MODE, "hall_a", 2, 12)
    assert "sha256" in initial_pose_mismatch(MODE, "hall_a", 3, 12, "different")


def test_active_map_switched_during_the_drag_is_refused():
    assert "changed mode" in initial_pose_mismatch(MODE, "hall_a", 3, 11)
    assert initial_pose_mismatch(dict(MODE, mode_name="TRANSITIONING"), "hall_a", 3, 12)
    assert initial_pose_mismatch(dict(MODE, mode_name="IDLE", active_map_id=""), "hall_a", 3, 12)
    assert initial_pose_mismatch(None, "hall_a", 3, 12)
