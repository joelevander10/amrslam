"""gls_node (gls-wait-coding-plan §B.4, P2) without a ROS graph or a camera.

fill() and the helpers are pure; _report and _tick run on a node built with __new__ and only
the state they read; the constructor runs with rclpy.Node's own methods stubbed and GlsLink
replaced, so the owner-lock order is checked without any I/O."""

import math
from types import SimpleNamespace

import pytest
from agv_core import config, ownerlock
from agv_core.drivers import gls
from agv_core.drivers.gls_link import REQUIRED
from builtin_interfaces.msg import Time
from rclpy.node import Node

from amr_base import gls_node as gn
from amr_interfaces.msg import Event, GlsReading


def frame(
    valid=True, x=120.0, y=-40.0, angle=90.0, spread=3.0, spread_deg=0.4, label="x0100y0200", reason=""
):
    used = ("A1", "B1", "C1", "D1", "A2")
    return gls.Frame(946687420249, label, 5, 1, x, y, angle, spread, spread_deg, used, valid, reason)


SNAP_OK = {"comms_ok": True, "config_ok": True, "fps": 14.6, "detail": "connected, settings verified"}


# ---- fill ------------------------------------------------------------------------------


def test_fill_nothing_in_view():
    m = gn.fill(GlsReading(), None, SNAP_OK, 1e-4)
    assert not m.valid and m.label_id == ""
    assert m.comms_ok and m.config_ok and m.calibrated
    assert m.fps == pytest.approx(14.6) and m.detail == "connected, settings verified"
    assert m.n_codes == 0 and m.x_counts == 0.0 and m.device_ts_ms == 0


def test_fill_invalid_frame_says_why_and_keeps_raw_counts():
    m = gn.fill(GlsReading(), frame(valid=False, reason="1 code(s), need 2"), SNAP_OK, 0.0)
    assert not m.valid and m.label_id == ""
    assert m.detail == "1 code(s), need 2"
    assert m.x_counts == 120.0 and m.y_counts == -40.0 and m.n_codes == 5 and m.n_outliers == 1


def test_fill_config_not_ok_is_never_valid():
    for cfg in (False, None):
        snap = dict(SNAP_OK, config_ok=cfg, detail="settings differ: GlsCodeSel = '1', expected '0'")
        m = gn.fill(GlsReading(), frame(), snap, 1e-4)
        assert not m.valid and not m.config_ok and m.label_id == ""
        assert "GlsCodeSel" in m.detail
    m = gn.fill(GlsReading(), frame(), dict(SNAP_OK, comms_ok=False), 1e-4)
    assert m.valid and not m.comms_ok, "comms_ok is reported, a delivered frame is still a frame"


def test_fill_uncalibrated_publishes_raw_counts_only():
    m = gn.fill(GlsReading(), frame(), SNAP_OK, 0.0, signs=(-1, -1, -1), angle_offset_rad=1.0)
    assert m.valid and m.label_id == "x0100y0200" and not m.calibrated
    assert (m.x_m, m.y_m, m.yaw_rad, m.spread_xy_m, m.spread_yaw_rad) == (0.0, 0.0, 0.0, 0.0, 0.0)
    assert (m.x_counts, m.y_counts, m.angle_deg, m.spread_counts) == (120.0, -40.0, 90.0, 3.0)
    assert m.device_ts_ms == 946687420249 and m.n_codes == 5


def test_fill_calibrated_applies_scale_signs_and_offset():
    scale = 2.5e-4
    m = gn.fill(GlsReading(), frame(), SNAP_OK, scale, signs=(-1, 1, -1), angle_offset_rad=math.pi / 4)
    assert m.valid and m.calibrated
    assert m.x_m == pytest.approx(-120.0 * scale) and m.y_m == pytest.approx(-40.0 * scale)
    assert m.yaw_rad == pytest.approx(-math.pi / 2 + math.pi / 4)
    want = gls.to_camera_pose(frame(), scale, -1, 1, -1, math.pi / 4)
    assert (m.x_m, m.y_m, m.yaw_rad) == pytest.approx(want)
    assert m.spread_xy_m == pytest.approx(3.0 * scale)
    assert m.spread_yaw_rad == pytest.approx(math.radians(0.4))
    assert m.x_counts == 120.0, "raw counts stay alongside the pose"


def test_fill_swap_xy_takes_x_from_the_device_y_before_the_signs():
    """gls.swap_xy (shipped true, bench 2026-10-30): label x from device Y, y from X; signs after."""
    scale = 2.5e-4
    m = gn.fill(GlsReading(), frame(), SNAP_OK, scale, signs=(-1, 1, 1), swap_xy=True)
    assert m.valid and m.calibrated
    assert m.x_m == pytest.approx(40.0 * scale) and m.y_m == pytest.approx(120.0 * scale)
    assert m.yaw_rad == pytest.approx(math.pi / 2)
    assert (m.x_m, m.y_m, m.yaw_rad) == pytest.approx(gls.to_camera_pose(frame(), scale, -1, 1, 1, 0.0, True))
    assert (m.x_counts, m.y_counts) == (120.0, -40.0), "the raw counts are never swapped"
    m = gn.fill(GlsReading(), frame(), SNAP_OK, 0.0, swap_xy=True)
    assert not m.calibrated and (m.x_m, m.y_m) == (0.0, 0.0), "uncalibrated: no pose, swapped or not"


def test_tick_passes_the_profile_swap_to_fill(monkeypatch):
    n = _node()
    n.link = _Link()
    n._report = lambda s: None
    n.scale = 1e-4
    for swap in (True, False):
        monkeypatch.setattr(config, "GLS_SWAP_XY", swap)
        n.published.clear()
        n.link.queue = [(gn.time.monotonic(), frame())]
        n._tick()
        (m,) = n.published
        assert (m.x_m, m.y_m) == pytest.approx((-0.004, 0.012) if swap else (0.012, -0.004))


# ---- helpers ---------------------------------------------------------------------------


def test_outlier_counts():
    assert gn.outlier_counts(0.005, 0.0) is None, "no tolerance in counts while the scale is unmeasured"
    assert gn.outlier_counts(0.0, 1e-4) is None, "0 = off"
    assert gn.outlier_counts(0.0, 0.0) is None
    assert gn.outlier_counts(0.005, 2.5e-4) == pytest.approx(20.0)


def test_expected_settings_are_the_profile_template_and_required():
    want = gn.expected_settings()
    assert want == {"OFtplstr1": config.GLS_EXPECT_TEMPLATE, **REQUIRED}
    assert REQUIRED == {"GlsCodeSel": "0", "DCDMVerISO15415": "0"}
    assert "[$XMCL]" in want["OFtplstr1"] and "[$YMCL]" in want["OFtplstr1"]


# ---- _report: one event per transition -------------------------------------------------


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, s, *_a, **_k):
        self.lines.append(("info", s))

    def warn(self, s, *_a, **_k):
        self.lines.append(("warn", s))

    def error(self, s, *_a, **_k):
        self.lines.append(("error", s))


def _node():
    n = gn.GlsNode.__new__(gn.GlsNode)
    n.events, n.published, n.log = [], [], _Logger()
    n.get_logger = lambda: n.log
    n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    n._pub_event = SimpleNamespace(publish=n.events.append)
    n._pub = SimpleNamespace(publish=n.published.append)
    n._event_seq = 0
    n._comms = n._config = None
    n._last_frame_t, n._last_invalid_t = None, 0.0
    n.scale, n.signs = 0.0, (1, 1, 1)
    return n


def snap(comms, cfg, detail="d"):
    return {"comms_ok": comms, "config_ok": cfg, "detail": detail, "fps": 0.0}


def codes(n):
    return [e.code for e in n.events]


def test_boot_that_never_connects_raises_nothing():
    n = _node()
    for _ in range(100):
        n._report(snap(False, None, "cannot connect to 192.168.4.1:2112: timed out"))
    assert n.events == []


def test_first_connect_after_boot_is_not_answering_again():
    n = _node()
    n._report(snap(False, None, "starting"))  # the link connects asynchronously: tick 1 is before any reply
    n._report(snap(True, True, "connected, settings verified"))
    assert codes(n) == []


def test_first_tick_already_answering_raises_nothing():
    n = _node()
    for _ in range(10):
        n._report(snap(True, True))
    assert n.events == []


def test_loss_and_recovery_are_one_event_each():
    n = _node()
    n._report(snap(True, True))
    for _ in range(50):
        n._report(snap(False, True, "no reply for 9.0 s: reconnecting"))
    assert codes(n) == ["GLS_DOWN"]
    e = n.events[0]
    assert e.level == Event.WARN and e.source == "gls_node" and "no reply" in e.text and e.seq == 1
    for _ in range(50):
        n._report(snap(True, True))
    assert codes(n) == ["GLS_DOWN", "GLS"]
    assert n.events[1].level == Event.INFO and n.events[1].seq == 2
    n._report(snap(False, True))
    n._report(snap(True, True))
    assert codes(n) == ["GLS_DOWN", "GLS", "GLS_DOWN", "GLS"]


def test_config_mismatch_is_raised_once():
    n = _node()
    n._report(snap(True, None, "connected: verifying settings"))
    for _ in range(100):
        n._report(snap(True, False, "settings differ: GlsCodeSel = '1', expected '0'"))
    assert codes(n) == ["GLS_CONFIG_MISMATCH"]
    assert n.events[0].level == Event.ERROR and "GlsCodeSel" in n.events[0].text
    assert ("error", "GLS settings: settings differ: GlsCodeSel = '1', expected '0'") in n.log.lines


def test_a_silent_camera_does_not_repeat_the_mismatch_every_reconnect():
    n = _node()
    for _ in range(5):  # what GlsLink.snapshot() reads through each connect / verify-timeout cycle
        n._report(snap(False, None, "connected: verifying settings"))
        n._report(snap(False, False, "no reply to OFtplstr1, GlsCodeSel, DCDMVerISO15415"))
        n._report(snap(False, False, "no reply for 9.0 s: reconnecting"))
    assert codes(n).count("GLS_CONFIG_MISMATCH") == 1


def test_verified_settings_raise_no_event():
    n = _node()
    n._report(snap(True, None))
    for _ in range(10):
        n._report(snap(True, True))
    assert n.events == [] and ("info", "GLS settings verified") in n.log.lines


# ---- _tick: per-frame readings and the valid=false cadence ------------------------------


class _Link:
    def __init__(self):
        self.snap = dict(SNAP_OK)
        self.queue = []

    def snapshot(self):
        return dict(self.snap)

    def pop_frames(self):
        out, self.queue = self.queue, []
        return out


def test_tick_publishes_every_frame_then_invalid_at_5_hz_once_stale(monkeypatch):
    t = [100.0]
    monkeypatch.setattr(gn.time, "monotonic", lambda: t[0])
    n = _node()
    n.link = _Link()
    n._report = lambda s: None
    n.link.queue = [(99.9, frame()), (99.95, frame(x=121.0))]
    n._tick()
    assert [m.valid for m in n.published] == [True, True]
    assert [m.x_counts for m in n.published] == [120.0, 121.0]
    assert all(m.header.frame_id == "gls_frame" for m in n.published)
    # 50 Hz ticks with nothing arriving: nothing until stale_s, then valid=false at 5 Hz
    n.published.clear()
    for i in range(1, 101):  # 2 s
        t[0] = 100.0 + i * 0.02
        n._tick()
    assert n.published and all(not m.valid for m in n.published)
    assert len(n.published) == pytest.approx((2.0 - config.GLS_STALE_S) * gn.INVALID_HZ, abs=1)
    # a frame resets staleness
    n.published.clear()
    n.link.queue = [(t[0], frame())]
    n._tick()
    for _ in range(1, int(config.GLS_STALE_S / 0.02)):
        t[0] += 0.02
        n._tick()
    assert [m.valid for m in n.published] == [True], "no invalid reading inside stale_s of a frame"


def test_tick_before_any_frame_publishes_invalid_at_5_hz(monkeypatch):
    t = [50.0]
    monkeypatch.setattr(gn.time, "monotonic", lambda: t[0])
    n = _node()
    n.link = _Link()
    n.link.snap = snap(False, None, "starting")
    n._report = lambda s: None
    for i in range(50):  # 1 s at 50 Hz
        t[0] = 50.0 + i * 0.02
        n._tick()
    assert len(n.published) == 5 and not any(m.valid or m.comms_ok for m in n.published)
    assert n.published[0].detail == "starting"


# ---- construction: the owner lock comes first ------------------------------------------


@pytest.fixture
def no_graph(monkeypatch, tmp_path):
    """Node's own methods stubbed, GlsLink replaced, locks in a temp dir."""
    monkeypatch.setenv("AMR_LOCK_DIR", str(tmp_path))
    built = []

    class FakeLink:
        def __init__(self, host, port, expect, **kw):
            built.append(
                SimpleNamespace(host=host, port=port, expect=expect, kw=kw, holder=ownerlock.holder("gls"))
            )
            self.started = False

        def start(self):
            self.started = True

    monkeypatch.setattr(gn, "GlsLink", FakeLink)
    monkeypatch.setattr(Node, "__init__", lambda self, name, **kw: None)
    monkeypatch.setattr(Node, "declare_parameter", lambda self, *a, **k: None)
    monkeypatch.setattr(Node, "get_parameter", lambda self, name: SimpleNamespace(value=50.0))
    pub = SimpleNamespace(publish=lambda m: None)
    monkeypatch.setattr(Node, "create_publisher", lambda self, *a, **k: pub)
    monkeypatch.setattr(Node, "create_timer", lambda self, *a, **k: None)
    monkeypatch.setattr(Node, "get_logger", lambda self: _Logger())
    return built


def test_node_takes_the_gls_lock_before_building_the_link(no_graph):
    node = gn.GlsNode()
    try:
        assert len(no_graph) == 1
        b = no_graph[0]
        assert b.holder and b.holder.startswith("gls_node"), "the lock was held before the link existed"
        assert (b.host, b.port) == (config.GLS_HOST, config.GLS_PORT)
        assert b.expect == gn.expected_settings()
        assert b.kw["keepalive_s"] == config.GLS_KEEPALIVE_S and b.kw["min_codes"] == config.GLS_MIN_CODES
        want = gn.outlier_counts(config.GLS_OUTLIER_M, config.GLS_XY_SCALE_M_PER_COUNT)
        assert b.kw["outlier_counts"] == want
        assert node.link.started
    finally:
        node._gls_lock.release()


def test_node_refuses_while_another_owner_holds_the_camera(no_graph):
    held = ownerlock.acquire("gls", "gls_probe")
    try:
        with pytest.raises(ownerlock.OwnerBusy, match="gls_probe"):
            gn.GlsNode()
        assert no_graph == [], "no link, so no socket, without the lock"
    finally:
        held.release()
