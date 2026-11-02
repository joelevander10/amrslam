"""amr_base.marker_events: MLS marker codes -> events (mls-marker-plan 5.3, 2026-10-09)."""

from agv_core.drivers.canbus import read_mls

from amr_base.marker_events import ABORT_GRACE_FRAMES, MarkerDetector


def frame(code=0, reading=False, lcp2=5, nlcp=2, intro=False):
    byte6 = nlcp | (0x08 if intro else 0) | (code << 4)
    status = 0x01 | (0x40 if reading else 0)
    return read_mls.decode_tpdo1(bytes([0, 0, lcp2 & 0xFF, (lcp2 >> 8) & 0xFF, 0, 0, byte6, status]), False)


def run(det, frames, source="tpdo", ok=True):
    return [e for k, f in enumerate(frames) if (e := det.feed(0.01 * k, f, source, ok))]


def test_one_marker_is_one_event():
    det = MarkerDetector([1, 2, 3])
    ev = run(det, [frame(), frame(reading=True), frame(2, lcp2=-4), frame(2), frame(2), frame()])
    assert [(e.code, e.seq, e.lcp2_mm, e.direction) for e in ev] == [(2, 1, -4, 0)]
    assert det.aborts == 0 and det.rejects == 0


def test_a_code_shown_for_a_single_frame_is_kept():
    det = MarkerDetector([1, 2, 3])
    assert [e.code for e in run(det, [frame(), frame(3), frame()])] == [3]


def test_a_change_between_codes_is_two_events():
    # only possible without FailSafe (a skewed pass): still reported, the consumer decides
    det = MarkerDetector([1, 2, 3])
    assert [e.code for e in run(det, [frame(), frame(1), frame(3), frame()])] == [1, 3]


def test_codes_not_on_the_floor_are_rejects_not_events():
    det = MarkerDetector([1, 2, 3])
    assert run(det, [frame(), frame(5), frame(), frame(7), frame()]) == []
    assert det.rejects == 2


def test_the_raw_field_is_carried():
    det = MarkerDetector([2])
    (e,) = run(det, [frame(), frame(2, intro=True)])
    assert e.raw == (2 << 1) | 1


def test_sdo_samples_and_untrusted_samples_make_nothing():
    det = MarkerDetector([1, 2, 3])
    assert run(det, [frame(), frame(2), frame()], source="sdo") == []
    assert run(det, [frame(), frame(2), frame()], ok=False) == []


def test_a_code_already_showing_when_trust_returns_is_not_new():
    det = MarkerDetector([1, 2, 3])
    det.feed(0.0, frame(), "tpdo", True)
    det.feed(0.01, frame(2), "tpdo", False)  # misconfigured mid-marker
    assert det.feed(0.02, frame(2), "tpdo", True) is None
    assert det.feed(0.03, frame(), "tpdo", True) is None
    assert det.feed(0.04, frame(1), "tpdo", True).code == 1


def test_an_uncoded_reading_episode_is_an_abort_after_the_grace():
    det = MarkerDetector([1, 2, 3])
    run(det, [frame(), frame(reading=True), frame(reading=True)] + [frame()] * (ABORT_GRACE_FRAMES + 1))
    assert det.aborts == 1


def test_a_code_reported_just_after_the_bit_drops_is_not_an_abort():
    det = MarkerDetector([1, 2, 3])
    ev = run(det, [frame(), frame(reading=True), frame(), frame(), frame(2)] + [frame()] * 8)
    assert [e.code for e in ev] == [2] and det.aborts == 0


def test_seq_is_monotonic_and_a_new_generation_restarts_it():
    det = MarkerDetector([1, 2, 3])
    ev = run(det, [frame(), frame(1), frame(), frame(2), frame()])
    assert [e.seq for e in ev] == [1, 2] and {e.generation for e in ev} == {0}
    det.new_generation()
    ev = run(det, [frame(), frame(3), frame()])
    assert [(e.seq, e.generation) for e in ev] == [(1, 1)]


def test_drive_node_checks_exactly_what_the_bench_tool_writes():
    from amr_base.drive_node import marker_expectation

    written = {(i, s): v for i, s, _size, v, _label in read_mls.MARKER_SETTINGS}
    assert marker_expectation(False, True) == written
    i, s, _size, v, _label = read_mls.POLARITY_LOCK
    assert marker_expectation(True, True) == {**written, (i, s): v}
    assert (0x2029, 0) not in marker_expectation(False, False)
