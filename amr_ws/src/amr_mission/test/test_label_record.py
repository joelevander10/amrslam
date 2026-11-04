"""Recording floor-label poses (gls-wait-coding-plan §C.2/§C.3, P3): label_record's
observation and write rules, the executor's "Record label here" service and run-time label
check, and HOME at survey start. No ROS graph: nodes are built with __new__ and stubs."""

import collections
import math
import threading
from types import SimpleNamespace

import pytest
from agv_core import config, glspose
from amr_maps.generate_sim_factory import build
from amr_mission.mapping_session_node import MappingSession
from amr_navigation.route import Limits, MapRef, Route, StartPose, Step
from test_route_executor_logic import _clock_stub, make_node

from amr_interfaces.msg import Event, GlsReading, LocalizationState, MappingState
from amr_interfaces.srv import RecordLabel, SaveMap, StartSurvey
from amr_mission import label_record as lr
from amr_mission import map_bundle as mb
from amr_mission import run_fsm as fsm
from amr_navigation import footprint as fpmod
from amr_navigation import store

N = config.GLS_FIX_FRAMES
STILL = 100.0  # the wheels came to rest here
NOW = STILL + config.GLS_STATIONARY_S + 0.5


def reading(label="x0100y0000", x=0.002, y=-0.001, yaw=0.03, n_codes=5, calibrated=True):
    m = GlsReading()
    m.valid, m.calibrated, m.label_id = True, calibrated, label
    m.x_m, m.y_m, m.yaw_rad, m.n_codes = x, y, yaw, n_codes
    return m


def samples(n=N, t_end=NOW - 0.01, t_mb=(4.0, 1.0, 0.5), dt=0.06, **kw):
    """n samples ending at t_end, ~14.6 frames/s apart."""
    return [lr.Sample(t_end - dt * (n - 1 - k), reading(**kw), t_mb) for k in range(n)]


@pytest.fixture()
def mount(monkeypatch):
    monkeypatch.setattr(config, "GLS_MOUNT_X_M", 0.4)
    monkeypatch.setattr(config, "GLS_MOUNT_Y_M", -0.1)
    monkeypatch.setattr(config, "GLS_MOUNT_YAW_RAD", 0.2)
    return 0.4, -0.1, 0.2


# ---- observe ----------------------------------------------------------------------------


def test_observe_refusals():
    s = samples()
    assert lr.observe(s, None, NOW)[1] == "the wheels are moving"
    assert "stand still" in lr.observe(s, NOW - config.GLS_STATIONARY_S + 0.01, NOW)[1]
    # every sample before the wheels came to rest: none usable
    assert "no floor-label reading" in lr.observe(s, NOW - 0.005, NOW + config.GLS_STATIONARY_S)[1]
    # the newest is older than gls.stale_s
    old = samples(t_end=NOW - config.GLS_STALE_S - 0.01)
    assert "no floor-label reading" in lr.observe(old, STILL, NOW)[1]
    assert lr.observe([], STILL, NOW)[1].startswith("no floor-label reading")
    assert "not calibrated" in lr.observe(samples(calibrated=False), STILL, NOW)[1]  # glspose's own
    assert "only 1 codes" in lr.observe(samples(n_codes=1), STILL, NOW)[1]


def test_observe_ignores_samples_before_the_wheels_came_to_rest():
    after = samples(n=N - 3)
    before = [lr.Sample(STILL - 0.1 * k - 0.01, reading(), (4.0, 1.0, 0.5)) for k in range(10)]
    obs, why = lr.observe(before + after, STILL, NOW)
    assert obs is None and f"{N - 3} of {N}" in why
    # moving-time samples of ANOTHER label do not spoil an observation made after it
    moving = [lr.Sample(STILL - 0.5, reading(label="x0900y0900", x=0.4), None)]
    obs, why = lr.observe(moving + samples(), STILL, NOW)
    assert why is None and obs.label_id == "x0100y0000" and obs.n_frames == N


def test_observe_needs_a_map_pose_for_every_frame_used_only():
    s = samples()
    s[-1] = lr.Sample(s[-1].t, s[-1].reading, None)
    obs, why = lr.observe(s, STILL, NOW)
    assert obs is None and "no map pose" in why
    # a pose-less sample older than the window does not matter
    s = [lr.Sample(STILL + 0.01, reading(), None)] + samples()
    assert lr.observe(s, STILL, NOW)[1] is None
    # need_pose=False never needs one
    s = [lr.Sample(x.t, x.reading, None) for x in samples()]
    assert lr.observe(s, STILL, NOW, need_pose=False)[1] is None


def test_observe_t_ml_is_the_averaged_tmb_mount_and_tlc(mount):
    poses = [(4.0 + 0.001 * (k % 3), 1.0, 0.5 + 0.0005 * (k % 2)) for k in range(N)]
    s = [lr.Sample(x.t, x.reading, poses[k]) for k, x in enumerate(samples())]
    obs, why = lr.observe(s, STILL, NOW)
    assert why is None
    t_mb = glspose.average(poses).pose
    want = glspose.label_in_map(t_mb, mount, (0.002, -0.001, 0.03))
    assert obs.t_ml == pytest.approx(want, abs=1e-12)
    assert obs.label_id == "x0100y0000" and obs.n_frames == N
    assert obs.spread_xy_m == pytest.approx(0.0, abs=1e-12)


def test_need_pose_false_then_at_equals_need_pose_true(mount):
    t_mb = (-3.2, 7.5, -2.8)
    a, why = lr.observe(samples(t_mb=t_mb), STILL, NOW)
    assert why is None
    b, why = lr.observe(samples(t_mb=None), STILL, NOW, need_pose=False)
    assert why is None
    c = lr.at(b, t_mb)
    assert c.t_ml == pytest.approx(a.t_ml, abs=1e-12)
    assert (c.label_id, c.n_frames, c.spread_xy_m, c.spread_yaw_rad) == (
        a.label_id,
        a.n_frames,
        a.spread_xy_m,
        a.spread_yaw_rad,
    )


# ---- record -----------------------------------------------------------------------------


def obs(label="x0100y0000", t_ml=(5.0, 1.0, 0.25)):
    return lr.Observation(label, t_ml, 15, 0.0008, 0.003)


MAP = ("hall", 1, "mapsha")


def test_record_rules(tmp_path):
    d = str(tmp_path)
    with pytest.raises(store.StoreError, match="role must be"):
        lr.record(d, MAP, obs(), "station", "UBC", "amcl")
    assert store.list_labels(d, "hall") == []
    rev, text, gone = lr.record(d, MAP, obs("x0000y0000", (1.0, 2.0, math.pi / 2)), "home", "  ", "survey")
    assert (rev, gone) == (1, "") and text == "HOME = x0000y0000 at (1.000, 2.000) m, 90.0 deg"
    doc, _ = store.latest_labels(d, "hall", 1)
    (home,) = doc["labels"]
    assert (home["name"], home["role"], home["method"], home["n_frames"]) == ("HOME", "home", "survey", 15)
    assert doc["map"] == {"id": "hall", "revision": 1, "sha256": "mapsha"}
    for name in ("", "   ", None):
        with pytest.raises(store.StoreError, match="needs a name"):
            lr.record(d, MAP, obs(), "dest", name, "amcl")
    rev, text, gone = lr.record(d, MAP, obs(), "dest", " UBC ", "amcl", "hitch")
    assert (rev, gone) == (2, "") and text.startswith("UBC = x0100y0000 at (5.000, 1.000) m")
    doc, _ = store.latest_labels(d, "hall", 1)
    assert [(e["name"], e["label_id"], e["note"]) for e in doc["labels"]] == [
        ("HOME", "x0000y0000", ""),
        ("UBC", "x0100y0000", "hitch"),
    ]
    # the same label re-recorded under a new name replaces the old entry, and says so
    rev, _, gone = lr.record(d, MAP, obs(t_ml=(5.01, 1.0, 0.25)), "dest", "UBC-2", "amcl")
    assert rev == 3 and gone == "UBC (x0100y0000)"
    # a new HOME replaces the old one
    rev, _, gone = lr.record(d, MAP, obs("x0900y0900"), "home", "", "amcl")
    assert rev == 4 and gone == "HOME (x0000y0000)"
    doc, _ = store.latest_labels(d, "hall", 1)
    assert sorted(e["label_id"] for e in doc["labels"]) == ["x0100y0000", "x0900y0900"]


def test_record_starts_afresh_on_another_bundle_or_map_revision(tmp_path):
    d = str(tmp_path)
    lr.record(d, MAP, obs("x0000y0000"), "home", "", "survey")
    lr.record(d, MAP, obs(), "dest", "UBC", "amcl")
    # same map id and revision, another bundle sha: the old labels are not carried over
    rev, _, gone = lr.record(d, ("hall", 1, "othersha"), obs("x0200y0000"), "dest", "UBD", "amcl")
    assert rev == 3 and gone == ""
    doc, _ = store.latest_labels(d, "hall", 1)
    assert [e["label_id"] for e in doc["labels"]] == ["x0200y0000"] and doc["map"]["sha256"] == "othersha"
    rev, _, gone = lr.record(d, ("hall", 1, "othersha"), obs("x0000y0000"), "home", "", "amcl")
    assert rev == 4 and gone == ""
    doc, _ = store.latest_labels(d, "hall", 1)
    assert [e["label_id"] for e in doc["labels"]] == ["x0200y0000", "x0000y0000"]
    # another map revision never inherits rev1's set: its first set holds just the new dest
    rev, _, _ = lr.record(d, ("hall", 2, "sha2"), obs(), "dest", "UBC", "amcl")
    doc, _ = store.latest_labels(d, "hall", 2)
    assert rev == 5 and [(e["label_id"], e["role"]) for e in doc["labels"]] == [("x0100y0000", "dest")]
    assert doc["map"] == {"id": "hall", "revision": 2, "sha256": "sha2"}


def test_record_a_dest_with_no_label_set_then_home_later(tmp_path):
    # a map surveyed without a HOME label (owner 2026-11-02): a dest is the first entry
    d = str(tmp_path)
    rev, text, gone = lr.record(d, MAP, obs(), "dest", "UBC", "amcl")
    assert (rev, gone) == (1, "") and text.startswith("UBC = x0100y0000 at (5.000, 1.000) m")
    doc, _ = store.latest_labels(d, "hall", 1)
    assert doc["revision"] == 1 and doc["map"] == {"id": "hall", "revision": 1, "sha256": "mapsha"}
    assert [(e["label_id"], e["role"], e["name"]) for e in doc["labels"]] == [("x0100y0000", "dest", "UBC")]
    # another dest onto the destinations-only set
    rev, _, gone = lr.record(d, MAP, obs("x0200y0000"), "dest", "UBD", "amcl")
    assert (rev, gone) == (2, "")
    # HOME recorded later is added, nothing replaced
    rev, _, gone = lr.record(d, MAP, obs("x0000y0000"), "home", "", "amcl")
    assert (rev, gone) == (3, "")
    doc, _ = store.latest_labels(d, "hall", 1)
    assert [(e["label_id"], e["role"]) for e in doc["labels"]] == [
        ("x0100y0000", "dest"),
        ("x0200y0000", "dest"),
        ("x0000y0000", "home"),
    ]
    # a second HOME replaces the first
    rev, _, gone = lr.record(d, MAP, obs("x0900y0900"), "home", "", "amcl")
    assert (rev, gone) == (4, "HOME (x0000y0000)")
    doc, _ = store.latest_labels(d, "hall", 1)
    homes = [e["label_id"] for e in doc["labels"] if e["role"] == "home"]
    assert homes == ["x0900y0900"] and len(doc["labels"]) == 3


# ---- the executor's "Record label here" -------------------------------------------------


class Log:
    def info(self, *_a, **_k):
        pass

    warn = error = info


def executor(tmp_path, active=MAP):
    n = make_node([SimpleNamespace(id="s1", type="straight")])
    _clock_stub(n)
    n.maps_dir = str(tmp_path)
    n.active_map = active
    n._gls = collections.deque(maxlen=64)
    n.events = []
    n._event_pub = SimpleNamespace(publish=n.events.append)
    n._event_seq = 0
    # the wheels have been still since STILL; the clock is NOW
    n.clock[0] = STILL
    n.wheels(0.0)
    n.clock[0] = NOW
    n.wheels(0.0)
    n._loc_t = NOW
    for s in samples(t_mb=(4.0, 1.0, 0.5)):
        n._gls.append(s)
    return n


def call(n, role="home", name="", note=""):
    return n._srv_record_label(RecordLabel.Request(role=role, name=name, note=note), RecordLabel.Response())


def test_record_label_refusals_in_order(tmp_path):
    n = executor(tmp_path, active=("", 0, ""))
    assert "no active map" in call(n).message
    n = executor(tmp_path, active=("hall", 1, ""))
    assert "no active map" in call(n).message
    n = executor(tmp_path)
    n.fsm.state = fsm.EXECUTING
    n._loc = None  # a later refusal too: the order says which is reported
    assert "a run is driving" in call(n).message
    n = executor(tmp_path)
    n._loc = None
    assert "no localisation state" in call(n).message
    n = executor(tmp_path)
    n._loc_t = NOW - n.loc_age - 0.01
    assert "no localisation state" in call(n).message
    n = executor(tmp_path)
    n._loc.state = LocalizationState.CHECKING
    n._wheels_valid = False
    assert "not READY" in call(n).message
    n = executor(tmp_path)
    n._wheels_t = NOW - n.wheels_age - 0.01
    assert "no wheel feedback" in call(n).message
    n = executor(tmp_path)
    n.wheels(0.0, valid=False)
    assert "no wheel feedback" in call(n).message
    n = executor(tmp_path)
    n.wheels(0.5)  # moving now
    n.wheels(0.0)  # and only just stopped
    assert "stand still" in call(n).message
    n = executor(tmp_path)
    n._gls.clear()
    assert "no floor-label reading" in call(n).message
    n = executor(tmp_path)
    r = call(n, "dest")  # record()'s refusals come back as text
    assert "needs a name" in r.message
    assert not r.ok and r.message.startswith("label not recorded:") and r.label_id == "" and r.revision == 0
    r = call(n, "station", "UBC")
    assert not r.ok and "role must be" in r.message
    assert n.events == [] and store.list_labels(str(tmp_path), "hall") == []


def test_record_label_success_fields_and_one_event(tmp_path, mount):
    n = executor(tmp_path)
    n.fsm.state = fsm.PAUSED  # stopped mid-run is allowed: it only writes a file
    r = call(n, "home")
    assert r.ok and r.label_id == "x0100y0000" and r.revision == 1
    want = glspose.label_in_map((4.0, 1.0, 0.5), mount, (0.002, -0.001, 0.03))
    assert (r.x_m, r.y_m, r.yaw_rad) == pytest.approx(want, abs=1e-12)
    assert r.message.startswith("recorded HOME = x0100y0000") and "label set rev1" in r.message
    (e,) = n.events
    assert (e.code, e.level, e.source) == ("LABEL_RECORDED", Event.INFO, "route_executor")
    assert e.text == r.message and e.seq == 1
    doc, _ = store.latest_labels(str(tmp_path), "hall", 1)
    assert doc["labels"][0]["method"] == "amcl" and doc["labels"][0]["x_m"] == pytest.approx(want[0])
    # re-recorded as a dest of the same id: replaces the home; a set with no home is valid
    r = call(n, "dest", "UBC")
    assert r.ok and r.revision == 2 and "replaces HOME (x0100y0000)" in r.message and len(n.events) == 2
    doc, _ = store.latest_labels(str(tmp_path), "hall", 1)
    assert [(e["label_id"], e["role"], e["name"]) for e in doc["labels"]] == [("x0100y0000", "dest", "UBC")]


def test_record_label_a_dest_with_no_home(tmp_path, mount):
    n = executor(tmp_path)  # no label set on the active map yet
    r = call(n, "dest", "UBC", "hitch")
    assert r.ok and r.label_id == "x0100y0000" and r.revision == 1, r.message
    want = glspose.label_in_map((4.0, 1.0, 0.5), mount, (0.002, -0.001, 0.03))
    assert (r.x_m, r.y_m, r.yaw_rad) == pytest.approx(want, abs=1e-12)
    assert r.message.startswith("recorded UBC = x0100y0000") and "label set rev1" in r.message
    (e,) = n.events
    assert (e.code, e.level) == ("LABEL_RECORDED", Event.INFO) and e.text == r.message
    doc, _ = store.latest_labels(str(tmp_path), "hall", 1)
    assert [(x["role"], x["name"], x["method"], x["note"]) for x in doc["labels"]] == [
        ("dest", "UBC", "amcl", "hitch")
    ]


def test_on_gls_buffers_valid_readings_with_the_pose_and_wheels_track_rest(tmp_path):
    n = executor(tmp_path)
    n._gls.clear()
    n._pose = lambda: (1.0, 2.0, 3.0)
    bad = reading()
    bad.valid = False
    n._on_gls(bad)
    n._on_gls(reading())
    assert [(s.t, s.t_mb) for s in n._gls] == [(NOW, (1.0, 2.0, 3.0))]
    for _ in range(70):
        n._on_gls(reading())
    assert len(n._gls) == 64
    assert n._still_since == STILL
    n.wheels(0.5)
    assert n._still_since is None
    n.clock[0] += 1.0
    n.wheels(0.0)
    assert n._still_since == NOW + 1.0
    n.wheels(0.0, valid=False)
    assert n._still_since is None


# ---- _srv_run: the mission's bound label set -----------------------------------------------


def _bundle(maps):
    g = build()
    stage = mb.staging_dir(maps, "sim_factory", 1)
    m = mb.Manifest(
        "sim_factory",
        1,
        mb.now_iso(),
        "map",
        0.05,
        [-3.0, -10.0, 0.0],
        g.width,
        g.height,
        {"x_m": 0, "y_m": 0, "yaw_rad": 0, "description": "mark"},
        {"dx_m": 0, "dy_m": 0, "dyaw_rad": 0, "note": "t"},
    )
    mb.stage_bundle(stage, g, None, m, required_posegraph=False)
    mb.verify(stage)
    mb.publish(stage, maps, "sim_factory", 1)
    return mb.load(maps, "sim_factory", 1)[0]


def _home():
    return store.label_entry("x0000y0000", "home", "HOME", (0.0, 0.0, 0.0), "survey", 15, 0.0, 0.0)


def test_srv_run_checks_the_bound_label_set(tmp_path):
    maps = str(tmp_path)
    man = _bundle(maps)
    steps = [
        Step("s1", "straight", to=(15.0, 0.0)),
        Step("w1", "wait", name="UBC", label_id="x0100y0000"),
        Step("s2", "rotate", direction="ccw", angle_deg=180),
        Step("s3", "straight", to=(2.0, 0.0)),
    ]
    route = Route("r1", MapRef("sim_factory", 1, man.sha256), StartPose(0.0, 0.0, 0.0), steps, Limits())
    _, _, rsha = store.save_route(maps, route)
    dest = store.label_entry("x0100y0000", "dest", "UBC", (15.0, 0.0, 0.0), "amcl", 15, 0.0, 0.0)
    _, _, lsha1 = store.save_labels(maps, "sim_factory", 1, man.sha256, [_home()])  # no UBC
    _, _, lsha2 = store.save_labels(maps, "sim_factory", 1, man.sha256, [_home(), dest])
    _, _, lsha3 = store.save_labels(maps, "sim_factory", 1, man.sha256, [dest])  # no HOME
    for mid, lab in (
        ("unknown", {"revision": 1, "sha256": lsha1}),
        ("badsha", {"revision": 2, "sha256": lsha1}),
        ("good", {"revision": 2, "sha256": lsha2}),
        ("dests", {"revision": 3, "sha256": lsha3}),
        ("none", None),
    ):
        store.save_mission(maps, mid, "sim_factory", 1, man.sha256, "r1", 1, rsha, lab)

    n = make_node([SimpleNamespace(id="s1", type="straight")])
    n.maps_dir, n.active_map = maps, ("sim_factory", 1, man.sha256)
    n.fp = fpmod.Footprint(((-0.5, -0.35), (1.1, -0.35), (1.1, 0.35), (-0.5, 0.35)), 0.20)

    def run(mid):
        return n._srv_run(SimpleNamespace(mission_id=mid), SimpleNamespace(accepted=None, message=""))

    r = run("badsha")
    assert not r.accepted and "label set hash differs" in r.message
    r = run("unknown")
    assert not r.accepted and "route invalid" in r.message and "x0100y0000" in r.message
    r = run("good")
    assert r.accepted, r.message
    r = run("dests")  # a destinations-only set (owner 2026-11-02) binds like any other
    assert r.accepted, r.message
    r = run("none")  # no set bound: the wait label is logged only, never a refusal
    assert r.accepted, r.message


# ---- mapping session: HOME at survey start ----------------------------------------------


def session(tmp_path, gls=(), still_since=STILL):
    sv = MappingSession.__new__(MappingSession)
    sv._lock = threading.Lock()
    sv._state, sv._message = MappingState.IDLE, ""
    sv.maps_dir = str(tmp_path)
    sv._now = lambda: NOW
    sv._readiness = lambda: []
    sv._pose_in_map = lambda: (2.0, -1.0, math.pi / 2)
    sv._gls = collections.deque(gls, maxlen=64)
    sv._still_since = still_since
    sv.events = []
    sv.event = lambda code, text: sv.events.append((code, text))
    sv.get_logger = lambda: Log()
    sv._pause_slam = lambda p: None

    def _set(state, message):
        sv._state, sv._message = state, message

    sv._set = _set
    return sv


def start(sv):
    return sv._srv_start(StartSurvey.Request(map_id="hall", description="mark A"), StartSurvey.Response())


def test_survey_start_with_and_without_home_never_refuses(tmp_path, mount):
    sv = session(tmp_path, samples(t_mb=None))
    r = start(sv)
    assert r.accepted and "HOME label x0100y0000 seen" in r.message
    want = glspose.label_in_map((2.0, -1.0, math.pi / 2), mount, (0.002, -0.001, 0.03))
    assert sv._home.t_ml == pytest.approx(want, abs=1e-12)
    for gls, still in (
        ((), STILL),
        (samples(t_mb=None), None),
        (samples(t_mb=None, calibrated=False), STILL),
    ):
        sv = session(tmp_path, gls, still)
        r = start(sv)
        assert r.accepted and "; mapping without a HOME label (" in r.message and sv._home is None
        assert sv._state == MappingState.MAPPING
    sv = session(tmp_path, samples(t_mb=None))
    start(sv)
    sv._srv_abort(None, SimpleNamespace(success=None, message=""))
    assert sv._home is None


def _saved(sv, tmp_path):
    """_srv_save with the transaction stubbed: the bundle 'published' as rev1."""
    sv._state = MappingState.RETURN_REVIEW
    sv._save_locked = lambda req, holder: (
        1,
        str(tmp_path / "hall" / "rev1"),
        SimpleNamespace(sha256="b" * 64),
    )
    return sv._srv_save(SaveMap.Request(note="n"), SaveMap.Response())


def test_save_writes_home_as_label_set_rev1(tmp_path):
    sv = session(tmp_path, samples(t_mb=None))
    start(sv)
    r = _saved(sv, tmp_path)
    assert r.ok and r.revision == 1 and sv._state == MappingState.SAVED and "HOME = x0100y0000" in r.message
    doc, _ = store.latest_labels(str(tmp_path), "hall", 1)
    assert doc["revision"] == 1 and doc["map"] == {"id": "hall", "revision": 1, "sha256": "b" * 64}
    assert [(e["role"], e["name"], e["method"]) for e in doc["labels"]] == [("home", "HOME", "survey")]
    assert [c for c, _ in sv.events] == ["LABEL_RECORDED"]


def test_no_home_saves_the_map_and_no_label_set(tmp_path):
    sv = session(tmp_path)
    start(sv)
    r = _saved(sv, tmp_path)
    assert r.ok and r.message.endswith("; saved without a HOME label") and sv.events == []
    assert r.revision == 1 and sv._state == MappingState.SAVED
    assert store.list_labels(str(tmp_path), "hall") == []


def test_a_failed_label_write_keeps_the_save(tmp_path, monkeypatch):
    sv = session(tmp_path, samples(t_mb=None))
    start(sv)

    def boom(*_a, **_k):
        raise OSError("disk full")

    monkeypatch.setattr(store, "save_labels", boom)
    r = _saved(sv, tmp_path)
    assert r.ok and r.revision == 1 and sv._state == MappingState.SAVED
    assert "HOME label NOT saved: disk full" in r.message
    assert [c for c, _ in sv.events] == ["LABEL_RECORD_FAILED"] and "disk full" in sv.events[0][1]


# ---- known gap -----------------------------------------------------------------------------


@pytest.mark.xfail(
    strict=True,
    reason="BUG route_executor_node.py _srv_record_label: only store.StoreError is caught around "
    "label_record.record(); an OSError from the label write (disk full, read-only maps dir) "
    "escapes the service callback instead of answering ok=false",
)
def test_record_label_answers_a_write_failure(tmp_path, monkeypatch):
    n = executor(tmp_path)

    def boom(*_a, **_k):
        raise OSError("disk full")

    monkeypatch.setattr(store, "save_labels", boom)
    r = call(n, "home")
    assert not r.ok and "disk full" in r.message
