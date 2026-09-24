"""amr_base.mls_track: read-only MLS track acquisition and its diagnostic (line-follow plan §1.1)."""

import struct

import pytest

from amr_base import mls_track
from amr_base.mls_track import ERROR, OK, WARN, MlsTrack

VARIANT = (0x2006, 1)
TPDO1 = (0x1800, 1)


class FakeLink:
    """SDO reads from a dict; records every read and route. No write method at all,
    so a write attempt would raise."""

    def __init__(self, objects):
        self.objects = objects
        self.router = self
        self.routes = {}
        self.reads = []

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


def test_only_reads_nothing_is_written():
    t, link, _, _ = make(objs())
    t.start()
    for k in range(30):
        t.poll(100.0 + 1.1 + k * 0.02)
    assert not hasattr(link, "write") and not hasattr(link, "nmt")
    assert {r[0] for r in link.reads} <= {0x2006, 0x1800, 0x2021, 0x2022}


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
