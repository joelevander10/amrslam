"""amr_base.mls_track: read-only MLS track acquisition and its diagnostic (line-follow plan §1.1)."""

import struct

import pytest

from amr_base import mls_track
from amr_base.mls_track import ERROR, OK, WARN, MlsTrack

VARIANT = (0x2006, 1)
TPDO1 = (0x1800, 1)


class FakeLink:
    """SDO reads from a dict; records every read, route and NMT command. No write
    method at all, so an object write attempt would raise."""

    def __init__(self, objects):
        self.objects = objects
        self.router = self
        self.routes = {}
        self.reads = []
        self.nmts = []

    def nmt(self, command, node=0):
        self.nmts.append((command, node))

    def read(self, node, index, sub=0, timeout=0.4):
        self.reads.append((index, sub))
        return self.objects.get((index, sub))

    def add(self, cob, handler):
        self.routes[cob] = handler


class Frame:
    def __init__(self, data):
        self.data = data


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def objs(tpdo=0x18A, variant=0, lcp2=0xFFDB, nlcp=2, status=0x03):
    return {
        VARIANT: variant,
        TPDO1: tpdo,
        (0x2021, 1): 0,
        (0x2021, 2): lcp2,  # -37 mm as the unsigned word SDO returns
        (0x2021, 3): 0,
        (0x2021, 4): nlcp,
        (0x2022, 0): status,
    }


def make(o, mode="auto", **kw):
    got, clk = [], Clock()
    link = FakeLink(o)
    t = MlsTrack(link, 10, got.append, mode=mode, clock=clk, log=lambda s: None, **kw)
    return t, link, got, clk


def test_tpdo_registers_0x18a_and_keeps_the_latest_sample_with_its_age():
    t, link, got, clk = make(objs(), mode="tpdo")
    assert t.start() == "tpdo" and 0x18A in link.routes
    assert got[0].source == "sdo" and got[0].lcp_mm[1] == -37  # the start snapshot
    clk.t = 100.5
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 12, 0, 2, 0x01)))
    assert t.last.source == "tpdo" and t.last.lcp_mm[1] == 12 and t.last.nlcp == 2
    assert t.samples == 2 and t.tpdo_frames == 1
    assert t.age(100.53) == pytest.approx(0.03)
    link.routes[0x18A](Frame(b"\x00" * 5))  # short frame: ignored
    assert t.samples == 2


def test_only_reads_and_nmt_start_to_the_mls_nothing_is_written():
    t, link, _, _ = make(objs())
    t.start()
    for k in range(30):
        t.poll(100.0 + 1.1 + k * 0.02)
    assert not hasattr(link, "write")
    assert {r[0] for r in link.reads} <= {0x2006, 0x1800, 0x2021, 0x2022}
    # NMT Start (0x01) addressed to node 10 only - never a broadcast (node 0),
    # never a drive, never another command
    assert link.nmts and set(link.nmts) == {(0x01, 10)}


def test_tpdo_mode_starts_the_mls_and_asks_again_every_2_s_while_the_stream_is_missing():
    t, link, _, clk = make(objs(), mode="tpdo")
    assert t.start() == "tpdo" and link.nmts == [(0x01, 10)]  # once at start
    for k in range(50):  # 100.0 .. 105.0, no frames at all
        t.poll(100.0 + k * 0.1)
    assert t.mode == "tpdo"  # tpdo mode does not fall back
    assert link.nmts == [(0x01, 10)] * 3  # start, then at ~102.0 and ~104.0
    assert t.nmt_starts == 3
    clk.t = 105.0
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 7, 0, 2, 1)))
    for k in range(50):  # frames flowing: no more asking
        t.poll(105.0 + k * 0.1)
        clk.t = 105.0 + k * 0.1
        link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 7, 0, 2, 1)))
    assert t.nmt_starts == 3


def test_a_stream_that_dies_after_frames_falls_back_and_is_restarted():
    """The vehicle case of 2026-09-21: the MLS boots Operational, frames arrive,
    then a Pre-operational reaches it and the stream dies. The old fallback only
    looked at tpdo_frames == 0 and left the reading stale for good."""
    t, link, got, clk = make(objs(lcp2=20), sdo_hz=10.0)
    assert t.start() == "tpdo"
    for k in range(8):
        clk.t = 100.1 + k * 0.01
        link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 7, 0, 2, 1)))
    assert t.tpdo_frames == 8 and t.mode == "tpdo"
    n_nmt = len(link.nmts)
    t.poll(100.5)
    assert t.mode == "tpdo"
    t.poll(101.3)  # > tpdo_wait_s (1.0) since the last frame at 100.17
    assert t.mode == "sdo" and len(link.nmts) == n_nmt  # the start's ask is < 2 s old
    for k in range(1, 30):
        t.poll(101.3 + k * 0.1)
    assert got[-1].source == "sdo" and got[-1].lcp_mm[1] == 20  # reading kept alive by SDO
    assert link.nmts[n_nmt:] == [(0x01, 10)] * 2  # asked again at ~102.0 and ~104.0
    clk.t = 104.3
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 9, 0, 2, 1)))
    assert t.mode == "tpdo" and t.last.source == "tpdo" and t.last.lcp_mm[1] == 9


def test_disabled_tpdo1_start_reports_off_in_tpdo_mode():
    t, link, _, clk = make(objs(tpdo=0x8000018A), mode="tpdo")
    assert t.start() == "off" and not link.routes
    level, message, _ = t.diagnostic(clk.t)
    assert (level, message) == (ERROR, "off")


def test_absent_sensor_is_off():
    t, _, got, _ = make({}, mode="auto")
    assert t.start() == "off" and not got


def test_auto_falls_back_to_sdo_polling_one_read_per_tick_and_back_to_tpdo_on_a_frame():
    t, link, got, clk = make(objs(lcp2=20), sdo_hz=10.0)
    assert t.start() == "tpdo"
    n0 = len(link.reads)
    t.poll(100.5)
    assert t.mode == "tpdo" and len(link.reads) == n0  # still waiting for frames
    clk.t = 101.0
    t.poll(101.0)
    assert t.mode == "sdo" and len(link.reads) == n0 + 1  # fell back; ONE read this tick
    for k in range(1, 5):
        t.poll(101.0 + k * 0.02)
    assert len(link.reads) == n0 + 5 and got[-1].source == "sdo" and got[-1].lcp_mm[1] == 20
    t.poll(101.09)  # a new cycle waits for the sdo period (started at 101.0)
    assert len(link.reads) == n0 + 5
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 7, 0, 2, 1)))
    assert t.mode == "tpdo" and t.last.source == "tpdo"
    t.poll(101.3)
    assert len(link.reads) == n0 + 5  # no polling once frames flow


def test_variant_mismatch_is_counted_and_combi_is_decoded():
    word = 100 | ((12 << 1) << 8)  # Combi: 100 mm, width 12
    t, _, got, _ = make(objs(variant=1, lcp2=word))
    t.start()
    assert t.variant_mismatches == 1 and t.combi and got[0].lcp_mm[1] == 100


def test_diagnostic_keys_and_levels():
    t, link, _, clk = make(objs(), mode="tpdo")
    t.start()
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, -37, 0, 2, 0x01 | (4 << 1))))
    level, message, kv = t.diagnostic(clk.t + 0.02)
    keys = dict(kv)
    assert level == OK and message == "one track"
    for k in (
        "nlcp",
        "lcp1_mm",
        "lcp2_mm",
        "lcp3_mm",
        "line_good",
        "track_level",
        "polarity",
        "marker",
        "age_ms",
        "samples",
        "variant",
    ):
        assert k in keys, k
    assert keys["lcp2_mm"] == "-37" and keys["lcp1_mm"] == "" and keys["nlcp"] == "2 one track"
    assert keys["track_level"] == "4" and keys["age_ms"] == "20" and keys["variant"] == "0 Standard"

    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 0, 0, 0, 0)))
    assert t.diagnostic(clk.t + 0.01)[:2] == (WARN, "no track")
    assert t.diagnostic(clk.t + 0.5)[:2] == (ERROR, "stale")  # TPDO stale after 0.1 s
    t.poll(clk.t + 0.5)
    assert t.stale_periods == 1


def test_sdo_mode_is_stale_on_its_own_slower_clock():
    t, _, _, clk = make(objs(), mode="sdo", sdo_hz=10.0)
    assert t.start() == "sdo"
    assert t.diagnostic(clk.t + 0.2)[0] == OK  # 0.2 s is fresh for 10 Hz polling
    assert t.diagnostic(clk.t + 0.4)[:2] == (ERROR, "stale")


def test_bad_mode_is_refused():
    with pytest.raises(ValueError):
        MlsTrack(FakeLink({}), 10, lambda s: None, mode="nmt")
    assert mls_track.MODES == ("tpdo", "sdo", "auto", "off")


# ---- F08: a sensor absent at boot is picked up later -----------------------


def test_absent_sensor_is_rediscovered_and_streams():
    t, link, got, clk = make({}, mode="auto")
    assert t.start() == "off"
    n = len(link.reads)
    t.poll(clk.t + 1.0)
    assert len(link.reads) == n, "polled an absent sensor before the period"
    t.poll(clk.t + 5.0)
    assert len(link.reads) == n + 1 and t.mode == "off"
    t.poll(clk.t + 6.0)
    assert len(link.reads) == n + 1, "more than one probe per period"
    link.objects.update(objs())
    clk.t += 10.0
    t.poll(clk.t)
    assert t.mode == "tpdo" and t.restarts == 1 and 0x18A in link.routes
    assert link.nmts[-1] == (0x01, 10)  # NMT Start still to the MLS only
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, -37, 0, 2, 0x01 | (4 << 1))))
    assert got and got[-1].source == "tpdo"


def test_a_failed_tpdo1_read_at_start_is_rediscovered_later():
    o = objs()
    del o[TPDO1]
    t, link, _, clk = make(o, mode="auto")
    assert t.start() == "sdo"  # unknown TPDO1: poll meanwhile
    o[TPDO1] = 0x18A
    clk.t += 5.0
    t.poll(clk.t)
    assert t.mode == "tpdo" and t.restarts == 1 and 0x18A in link.routes


def test_a_configured_off_mode_stays_off():
    t, link, _, clk = make(objs(), mode="off")
    assert t.start() == "off"
    n = len(link.reads)
    for k in range(4):
        t.poll(clk.t + 5.0 * (k + 1))
    assert t.mode == "off" and len(link.reads) == n


# -- marker configuration check (mls-marker-plan 5.2, 2026-10-09) --------------------

EXPECT = {(0x2028, 1): 1, (0x2028, 2): 1, (0x2028, 3): 1, (0x2029, 0): 1}


def configured(over=None, **kw):
    o = objs(variant=3, **kw)
    o.update(EXPECT)
    o.update(over or {})
    return o


def start_streaming(o, **kw):
    t, link, got, clk = make(o, mode="tpdo", expected_variant=3, markers=EXPECT, **kw)
    t.start()
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 12, 0, 2, 0x01)))
    return t, link, clk


def test_the_marker_check_reads_one_object_per_poll_and_never_in_start():
    t, link, clk = start_streaming(configured())
    start_reads = list(link.reads)
    assert not any(k in EXPECT for k in start_reads), "start() must not read marker objects"
    assert t.marker_status == "checking" and not t.markers_ok
    for _ in EXPECT:
        before = len(link.reads)
        t.poll(clk.t)
        assert len(link.reads) - before == 1
    assert t.marker_status == "ok" and t.markers_ok and t.marker_checks == 1


def test_a_wrong_value_is_misconfigured_and_names_it():
    t, link, clk = start_streaming(configured({(0x2028, 3): 0}))
    for _ in EXPECT:
        t.poll(clk.t)
    assert t.marker_status.startswith("misconfigured") and "2028h:03 = 0 (want 1)" in t.marker_status
    assert not t.markers_ok


def test_an_unanswered_read_is_unverified_and_retried_later_not_at_once():
    o = configured()
    del o[(0x2028, 2)]
    t, link, clk = start_streaming(o)
    for _ in range(3):
        t.poll(clk.t)
    assert t.marker_status.startswith("unverified") and not t.markers_ok
    before = len(link.reads)
    t.poll(clk.t)
    assert len(link.reads) == before  # waits MARKER_RETRY_S
    o[(0x2028, 2)] = 1
    clk.t += MlsTrack.MARKER_RETRY_S
    for _ in range(len(EXPECT) + 1):
        t.poll(clk.t)
    assert t.markers_ok


def test_a_stream_gap_makes_the_markers_untrusted_until_re_read():
    t, link, clk = start_streaming(configured())
    for _ in EXPECT:
        t.poll(clk.t)
    assert t.markers_ok
    clk.t += 2.0  # TPDO1 stops: the poll asks node 10 to start again
    t.poll(clk.t)
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 12, 0, 2, 0x01)))
    assert t.marker_status == "checking" and not t.markers_ok


def test_markers_off_in_the_profile_read_nothing():
    t, link, got, clk = make(configured(), mode="tpdo", expected_variant=3)
    t.start()
    link.routes[0x18A](Frame(struct.pack("<hhhBB", 0, 12, 0, 2, 0x01)))
    for _ in range(10):
        t.poll(clk.t)
    assert not any(k in EXPECT for k in link.reads)
    assert t.marker_status_text() == "off" and not t.markers_ok


def test_the_sdo_fallback_reports_no_stream_and_no_markers():
    t, link, got, clk = make(configured(tpdo=0x8000018A), mode="auto", expected_variant=3, markers=EXPECT)
    assert t.start() == "sdo"
    assert t.marker_status_text() == "no stream (sdo)" and not t.markers_ok


# -- event detection and temperature (mls-diagnostics-plan M1/M2, 2026-10-18) ---------

EVENT_FIELD = {(0x2080, 2): 2, (0x2080, 4): 1, (0x2080, 5): 0}
EVENT_KEYS = {(0x2080, 2), (0x2080, 4), (0x2080, 5)}
FLAG = 0x80


class SensorLink(FakeLink):
    """A link that can write the line sensor: records (index, sub, value), or fails."""

    def __init__(self, objects, ok=True):
        super().__init__(objects)
        self.writes = []
        self.ok = ok

    def write_sensor(self, node, index, sub, value, size, timeout=0.05):
        from agv_core.drivers.canbus import guard

        guard.check_sensor(index, sub)
        self.writes.append((index, sub, value))
        if self.ok:
            self.objects[(0x2022, 0)] = 0x01
        return self.ok, "" if self.ok else "abort"


def frame(lcp2=12, nlcp=2, status=0x01):
    return Frame(struct.pack("<hhhBB", 0, lcp2, 0, nlcp, status))


def streaming(o=None, link_cls=FakeLink, **kw):
    o = {**objs(variant=3), **(o or {})}
    got, ev, clk = [], [], Clock()
    link = link_cls(o)
    t = MlsTrack(link, 10, got.append, mode="tpdo", expected_variant=3, clock=clk, log=lambda s: None,
                 on_event=ev.append, **kw)
    t.start()
    link.routes[0x18A](frame())
    return t, link, ev, clk


def test_a_rising_event_flag_reads_the_cause_one_object_per_poll_and_reports_a_field_event():
    t, link, ev, clk = streaming(EVENT_FIELD)
    link.routes[0x18A](frame(lcp2=-4, status=0x01 | (2 << 1) | FLAG))
    for _ in range(3):
        before = len(link.reads)
        t.poll(clk.t)
        assert len(link.reads) - before == 1
    assert {k for k in link.reads if k[0] == 0x2080} == EVENT_KEYS
    assert t.events_field == 1 and len(ev) == 1
    assert ev[0]["field"] and ev[0]["lcp2_mm"] == -4 and ev[0]["track_level"] == 2
    assert dict(t.diagnostic(clk.t)[2])["events_field"] == "1"


def test_a_flag_held_high_is_one_event_and_nothing_is_written_without_event_clear():
    t, link, ev, clk = streaming(EVENT_FIELD)
    for _ in range(20):
        link.routes[0x18A](frame(status=0x01 | FLAG))
        t.poll(clk.t)
        clk.t += 0.5
    assert t.events_field == 1 and not hasattr(link, "write_sensor")
    assert dict(t.diagnostic(clk.t)[2])["event_clear"] == "off"
    link.routes[0x18A](frame(status=0x01))  # reset elsewhere (power cycle): the next rise is new
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(3):
        t.poll(clk.t)
    assert t.events_field == 2


def test_leaving_the_tape_is_not_a_weak_tape_event():
    t, link, ev, clk = streaming(EVENT_FIELD)
    link.routes[0x18A](frame(nlcp=0, status=0))
    link.routes[0x18A](frame(nlcp=0, status=FLAG))
    for _ in range(3):
        t.poll(clk.t)
    assert t.events_field == 0 and t.events_no_track == 1 and not ev


def test_a_temperature_event_reads_the_temperature_at_once():
    o = {(0x2080, 2): 4, (0x2080, 4): 0, (0x2080, 5): 2, (0x2070, 1): 75, (0x2070, 6): 70, (0x2070, 7): 0}
    t, link, ev, clk = streaming(o)
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(6):
        t.poll(clk.t)
    assert t.events_temp == 1 and not ev
    assert t.temp == {"temp_c": 75, "hi_c": 70, "lo_c": 0, "out": True}


def test_the_temperature_is_read_periodically_and_a_silent_object_is_counted():
    t, link, _, clk = streaming({(0x2070, 1): 0xF6, (0x2070, 6): 70, (0x2070, 7): 0xF6})
    t.poll(clk.t + 5.0)
    assert not any(k[0] == 0x2070 for k in link.reads)  # TEMP_FIRST_S
    for _ in range(3):
        t.poll(clk.t + MlsTrack.TEMP_FIRST_S)
    assert t.temp["temp_c"] == -10 and not t.temp["out"]
    assert dict(t.diagnostic(clk.t)[2])["temp_c"] == "-10"
    del link.objects[(0x2070, 1)]
    t.poll(clk.t + MlsTrack.TEMP_FIRST_S + MlsTrack.TEMP_PERIOD_S)
    assert t.temp_unread == 1


def test_an_unanswered_cause_is_counted_and_retried_later():
    t, link, ev, clk = streaming()
    link.routes[0x18A](frame(status=0x01 | FLAG))
    t.poll(clk.t)
    assert t.events_unread == 1 and not t._jobs
    link.objects.update(EVENT_FIELD)
    link.routes[0x18A](frame(status=0x01 | FLAG))
    t.poll(clk.t)
    assert t.events_unread == 1 and not t._jobs  # not before EVENT_RETRY_S
    clk.t += MlsTrack.EVENT_RETRY_S
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(3):
        t.poll(clk.t)
    assert t.events_field == 1


def test_a_marker_check_and_an_event_share_the_queue_one_read_per_poll():
    o = {**configured(), **EVENT_FIELD}
    got, ev, clk = [], [], Clock()
    link = FakeLink(o)
    t = MlsTrack(link, 10, got.append, mode="tpdo", expected_variant=3, markers=EXPECT, clock=clk,
                 log=lambda s: None, on_event=ev.append)
    t.start()
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(len(EXPECT) + 3):
        before = len(link.reads)
        t.poll(clk.t)
        assert len(link.reads) - before == 1
    assert t.markers_ok and t.events_field == 1


def test_the_sdo_fallback_reads_no_cause():
    t, link, _, clk = make({**objs(tpdo=0x8000018A), **EVENT_FIELD}, mode="auto")
    assert t.start() == "sdo"
    t.poll(clk.t + 0.2)
    assert not any(k[0] == 0x2080 for k in link.reads)


# M2: the latched flag is reset through the guard, after the cause.

def test_with_event_clear_the_reset_is_written_after_the_cause_and_the_next_rise_is_new():
    t, link, ev, clk = streaming(EVENT_FIELD, link_cls=SensorLink, event_clear=0)
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(3):
        t.poll(clk.t)
    assert not link.writes and t.events_field == 1  # the cause first
    t.poll(clk.t)
    assert link.writes == [(0x2080, 1, 0)] and t.clears == 1
    clk.t += 0.2
    link.routes[0x18A](frame(status=0x01))
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(4):
        t.poll(clk.t)
    assert t.events_field == 2 and len(link.writes) == 2


def test_a_flag_still_high_after_a_clear_is_a_new_event_at_most_once_per_clear_min_s():
    t, link, ev, clk = streaming(EVENT_FIELD, link_cls=SensorLink, event_clear=1)
    for _ in range(40):  # 4 s with the flag up the whole time (the weak spot is under the sensor)
        link.routes[0x18A](frame(status=0x01 | FLAG))
        t.poll(clk.t)
        clk.t += 0.1
    assert 3 <= t.events_field <= 4 and len(link.writes) == t.events_field
    assert all(w == (0x2080, 1, 1) for w in link.writes)


def test_a_failed_reset_turns_the_clearing_off_and_keeps_reporting():
    t, link, ev, clk = streaming(EVENT_FIELD, link_cls=SensorLink, event_clear=0)
    link.ok = False
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(4):
        t.poll(clk.t)
    assert t.event_clear is None and t.clear_state == "failed" and t.events_field == 1
    assert dict(t.diagnostic(clk.t)[2])["event_clear"] == "failed"
    clk.t += 5.0
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(4):
        t.poll(clk.t)
    assert len(link.writes) == 1 and t.events_field == 1  # held high, no clear: no new event


def test_a_link_without_a_sensor_write_turns_the_clearing_off():
    t, link, ev, clk = streaming(EVENT_FIELD, event_clear=0)
    link.routes[0x18A](frame(status=0x01 | FLAG))
    for _ in range(4):
        t.poll(clk.t)
    assert t.clear_state == "failed" and t.events_field == 1


def test_only_2080h_01_can_be_written_through_the_sensor_guard():
    from agv_core.drivers.canbus import guard

    link = SensorLink({})
    for key in ((0x2080, 2), (0x2070, 6), (0x2027, 0), (0x1010, 1), (0x60FF, 0)):
        with pytest.raises(guard.ForbiddenWrite):
            link.write_sensor(10, *key, 1, 1)
    assert link.write_sensor(10, 0x2080, 1, 0, 1)[0] and link.writes == [(0x2080, 1, 0)]


def test_an_event_clear_outside_0_1_is_off():
    t, _, _, _ = streaming(event_clear=-1)
    assert t.event_clear is None and t.clear_state == "off"


def test_the_rate_limit_reports_once_per_period_and_counts_the_ones_held_back():
    r = mls_track.RateLimit(2.0)
    assert r.allow(10.0) == 0
    assert r.allow(10.5) is None and r.allow(11.9) is None
    assert r.allow(12.0) == 2
    assert r.allow(20.0) == 0
