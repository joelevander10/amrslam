"""MLS marker intake (mls-marker-plan 6, 2026-10-09): the cursor, and that a marker never
changes the job's state - nor, without a high zone in the mission, its wheel command."""

from amr_line.marker_reader import MarkerReader

from amr_line import autopilot
from amr_line import job as lj

from .test_line_authority import inputs, run


def snap(encounters, gen=0, base=0, ok=True, status="ok"):
    return {"ok": ok, "status": status, "generation": gen, "base": base, "encounters": encounters}


def test_each_marker_is_read_once_in_order():
    r = MarkerReader()
    assert r.scan(snap([(1, 2, 0, -3, True)])) == [(2, 0, -3, 0, True)]
    assert r.scan(snap([(1, 2, 0, -3, True), (2, 1, 0, 4, True)])) == [(1, 0, 4, 0, True)]
    assert r.scan(snap([(1, 2, 0, -3, True), (2, 1, 0, 4, True)])) == []
    assert r.count == 2 and r.last == {"code": 1, "direction": 0, "lcp2_mm": 4}


def test_a_sequence_gap_is_counted_as_missed():
    r = MarkerReader()
    assert r.scan(snap([(1, 2, 0, 0, True), (4, 3, 0, 0, True)])) == [(2, 0, 0, 0, True), (3, 0, 0, 2, True)]
    assert r.missed == 2


def test_joining_a_running_stream_does_not_count_old_markers_as_missed():
    r = MarkerReader()
    assert r.scan(snap([], base=17)) == []
    assert r.scan(snap([(18, 2, 0, 0, True)], base=17)) == [(2, 0, 0, 0, True)] and r.missed == 0


def test_a_new_generation_restarts_the_cursor():
    r = MarkerReader()
    r.scan(snap([(1, 2, 0, 0, True), (2, 2, 0, 0, True)]))
    assert r.scan(snap([(1, 3, 0, 0, True)], gen=1)) == [(3, 0, 0, 0, True)] and r.missed == 0


def test_an_entry_without_line_good_is_not_good():
    assert MarkerReader().scan(snap([(1, 2, 0, 0)])) == [(2, 0, 0, 0, False)]


def test_no_snapshot_is_nothing():
    assert MarkerReader().scan(None) == []


def test_a_marker_changes_neither_the_state_nor_the_wheel_command():
    plain, marked = run(lj.FollowJob(autopilot.LineFollower())), run(lj.FollowJob(autopilot.LineFollower()))
    for k in range(1, 40):
        t = 101.0 + 0.02 * k
        enc = [(1, 2, 0, 3, True)] if k >= 10 else []
        a = plain.tick(inputs(now=t))
        b = marked.tick(inputs(now=t, markers=snap(enc)))
        assert a == b and plain.state == marked.state == lj.RUNNING
    assert marked.marker_snapshot() == {"markers_ok": True, "last_marker": 2, "marker_count": 1,
                                        "marker_missed": 0}
    log = marked.drain_marker_events()
    assert ("info", "marker 2 at +3 mm") in log and marked.drain_marker_events() == []


def test_the_stream_going_down_is_logged_once():
    job = lj.FollowJob(autopilot.LineFollower())
    job.tick(inputs(markers=snap([])))
    job.tick(inputs(markers=snap([], ok=False, status="no marker data")))
    job.tick(inputs(markers=snap([], ok=False, status="no marker data")))
    assert job.drain_marker_events() == [("info", "markers up: ok"), ("warn", "markers down: no marker data")]
