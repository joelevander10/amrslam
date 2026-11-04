"""Floor-label sets in the map store and their use by missions and validation
(gls-wait-coding-plan §C.1, P3). Every test works in its own tmp_path store."""

import math
import os
import shutil

import pytest
import yaml
from amr_maps.generate_sim_factory import build
from amr_navigation.route import Limits, MapRef, Route, StartPose, Step
from amr_navigation.validate import INFO, validate

from amr_navigation import footprint as fpmod
from amr_navigation import store

FP = fpmod.Footprint(((-0.5, -0.35), (1.1, -0.35), (1.1, 0.35), (-0.5, 0.35)), 0.20)


def entry(lid="x0000y0000", role="home", name="HOME", pose=(1.0, 2.0, 0.5), method="survey", **kw):
    e = store.label_entry(lid, role, name, pose, method, 15, 0.001, 0.004, "")
    e.update(kw)
    return e


def good_set():
    return [entry(), entry("x0100y0000", "dest", "UBC", (5.0, 0.0, 0.0), "amcl")]


def dests_only():
    """A map surveyed without a HOME label (owner 2026-11-02): destinations only."""
    return [
        entry("x0100y0000", "dest", "UBC", (5.0, 0.0, 0.0), "amcl"),
        entry("x0200y0000", "dest", "UBD", (9.0, 1.0, 0.0), "amcl"),
    ]


# ---- save / load / list / latest ------------------------------------------------------


def test_save_load_list_and_latest_across_two_map_revisions(tmp_path):
    d = str(tmp_path)
    assert store.list_labels(d, "hall") == [] and store.latest_labels(d, "hall", 1) is None
    r1, p1, sha1 = store.save_labels(d, "hall", 1, "mapsha1", good_set())
    assert (r1, p1) == (1, store.labels_path(d, "hall", 1)) and p1.endswith(
        os.path.join("hall", "labels", "rev1.yaml")
    )
    r2, _p2, sha2 = store.save_labels(d, "hall", 2, "mapsha2", [entry()])  # map rev 2's first set
    r3, _p3, sha3 = store.save_labels(d, "hall", 1, "mapsha1", [entry(pose=(1.1, 2.0, 0.5))])
    assert (r2, r3) == (2, 3) and store.list_labels(d, "hall") == [1, 2, 3]
    doc, sha = store.load_labels(d, "hall", 1)
    assert sha == sha1 == store.sha256_file(p1)
    assert doc["revision"] == 1 and doc["map"] == {"id": "hall", "revision": 1, "sha256": "mapsha1"}
    assert [e["label_id"] for e in doc["labels"]] == ["x0000y0000", "x0100y0000"]
    # the newest set FOR THAT MAP REVISION, whatever else was written in between
    doc, sha = store.latest_labels(d, "hall", 1)
    assert (doc["revision"], sha) == (3, sha3) and doc["labels"][0]["x_m"] == 1.1
    doc, sha = store.latest_labels(d, "hall", 2)
    assert (doc["revision"], sha) == (2, sha2)
    assert store.latest_labels(d, "hall", 3) is None
    assert store.latest_labels(d, "other", 1) is None
    # immutable: later saves never rewrite an earlier revision
    assert store.sha256_file(p1) == sha1
    with pytest.raises(store.StoreError, match="no such label set"):
        store.load_labels(d, "hall", 9)


def test_save_refuses_a_bad_set_or_map_revision_and_writes_nothing(tmp_path):
    d = str(tmp_path)
    with pytest.raises(store.StoreError, match="at most one home"):
        store.save_labels(d, "hall", 1, "s", good_set() + [entry("x0200y0000", "home", "HOME2")])
    with pytest.raises(store.StoreError, match="at least one label"):
        store.save_labels(d, "hall", 1, "s", [])
    for bad in (0, True, "1"):
        with pytest.raises(store.StoreError, match="revision"):
            store.save_labels(d, "hall", bad, "s", good_set())
    with pytest.raises(store.StoreError, match="plain name"):
        store.save_labels(d, "../x", 1, "s", good_set())
    assert store.list_labels(d, "hall") == []


# ---- check_labels: one rule each --------------------------------------------------------


def _with(i, **kw):
    s = good_set()
    s[i].update(kw)
    return s


@pytest.mark.parametrize(
    "labels, why",
    [
        ({"x": 1}, "must be a list"),
        ([entry(), "x0100y0000"], "not a mapping"),
        (_with(1, label_id="X0100Y0000"), "label_id must look like"),
        (_with(1, label_id="x100y0000"), "label_id must look like"),
        (_with(1, label_id=100), "label_id must look like"),
        (_with(1, name="  "), "name must not be empty"),
        (_with(1, name=None), "name must not be empty"),
        (_with(1, role="station"), "role must be one of"),
        (_with(1, method="guess"), "method must be one of"),
        (_with(1, x_m=math.nan), "x_m must be a finite"),
        (_with(1, y_m=math.inf), "y_m must be a finite"),
        (_with(1, yaw_rad="0.1"), "yaw_rad must be a finite"),
        (_with(1, spread_xy_m=True), "spread_xy_m must be a finite"),
        (_with(1, spread_yaw_rad=None), "spread_yaw_rad must be a finite"),
        (_with(1, n_frames=0), "n_frames must be a positive"),
        (_with(1, n_frames=True), "n_frames must be a positive"),
        (_with(1, n_frames=15.0), "n_frames must be a positive"),
        (_with(1, label_id="x0000y0000"), "appears twice"),
        (_with(1, name="HOME"), "appears twice"),
        ([], "at least one label"),
        (good_set() + [entry("x0200y0000", "home", "HOME2")], "at most one home label, has 2"),
        (
            [entry(), entry("x0200y0000", "home", "HOME2"), entry("x0300y0000", "home", "HOME3")],
            "at most one home label, has 3",
        ),
    ],
)
def test_check_labels_rules(labels, why):
    assert why in (store.check_labels(labels) or "")


@pytest.mark.xfail(
    strict=True,
    reason="BUG store.py:204/236: LABEL_ID_RE '^x\\d{4}y\\d{4}$' with .match() lets a trailing "
    "newline through ('$' matches before a final \\n) and \\d takes non-ASCII digits",
)
@pytest.mark.parametrize("lid", ["x0100y0000\n", "x\u0661\u0662\u0663\u0664y0000"])
def test_check_labels_refuses_a_trailing_newline_or_non_ascii_digits(lid):
    assert "label_id must look like" in (store.check_labels(_with(1, label_id=lid)) or "")


def test_check_labels_accepts_a_good_set_and_integer_poses():
    assert store.check_labels(good_set()) is None
    assert store.check_labels([entry(x_m=1, y_m=0, yaw_rad=0)]) is None


def test_check_labels_accepts_a_set_with_no_home():
    # a map may be surveyed and used without a HOME label (owner 2026-11-02)
    assert store.check_labels(dests_only()) is None
    assert store.check_labels([entry("x0100y0000", "dest", "UBC")]) is None


# ---- identity: the file must be the set it is asked for ----------------------------------


def test_a_renamed_or_copied_label_file_is_refused(tmp_path):
    d = str(tmp_path)
    store.save_labels(d, "hall", 1, "s", good_set())
    ldir = store.labels_dir(d, "hall")
    shutil.copy(os.path.join(ldir, "rev1.yaml"), os.path.join(ldir, "rev2.yaml"))  # copied
    with pytest.raises(store.StoreError, match="is not hall labels rev2"):
        store.load_labels(d, "hall", 2)
    os.rename(os.path.join(ldir, "rev2.yaml"), os.path.join(ldir, "rev5.yaml"))  # renamed
    with pytest.raises(store.StoreError, match="is not hall labels rev5"):
        store.load_labels(d, "hall", 5)
    # a set copied in from another map id
    store.save_labels(d, "dock", 1, "s", good_set())
    os.unlink(os.path.join(ldir, "rev5.yaml"))
    shutil.copy(os.path.join(store.labels_dir(d, "dock"), "rev1.yaml"), os.path.join(ldir, "rev2.yaml"))
    with pytest.raises(store.StoreError, match="is not hall labels rev2"):
        store.load_labels(d, "hall", 2)


def test_load_refuses_a_bad_map_revision_or_entries(tmp_path):
    d = str(tmp_path)
    _, p, _ = store.save_labels(d, "hall", 1, "s", good_set())
    doc = yaml.safe_load(open(p))
    os.chmod(p, 0o644)
    doc["map"]["revision"] = "1"
    open(p, "w").write(yaml.safe_dump(doc))
    with pytest.raises(store.StoreError, match="bad map revision"):
        store.load_labels(d, "hall", 1)
    doc["map"]["revision"] = 1
    doc["labels"][1]["role"] = "home"
    open(p, "w").write(yaml.safe_dump(doc))
    with pytest.raises(store.StoreError, match="at most one home"):
        store.load_labels(d, "hall", 1)
    doc["labels"] = []
    open(p, "w").write(yaml.safe_dump(doc))
    with pytest.raises(store.StoreError, match="at least one label"):
        store.load_labels(d, "hall", 1)


def test_a_corrupt_newest_set_is_an_error_not_a_fallback(tmp_path):
    d = str(tmp_path)
    store.save_labels(d, "hall", 1, "s", good_set())
    _, p2, _ = store.save_labels(d, "hall", 1, "s", [entry()])
    for junk in ("labels: [unclosed\n", "just a string\n", ""):
        open(p2, "w").write(junk)
        with pytest.raises(store.StoreError):
            store.latest_labels(d, "hall", 1)
    # a newest set whose entries break the rules is an error too, not the older set
    doc = {"revision": 2, "map": {"id": "hall", "revision": 1, "sha256": "s"}, "labels": []}
    open(p2, "w").write(yaml.safe_dump(doc))
    with pytest.raises(store.StoreError, match="at least one label"):
        store.latest_labels(d, "hall", 1)
    doc["labels"] = [entry(), entry("x0200y0000", "home", "HOME2")]
    open(p2, "w").write(yaml.safe_dump(doc))
    with pytest.raises(store.StoreError, match="at most one home"):
        store.latest_labels(d, "hall", 1)


def test_a_destinations_only_set_saves_loads_and_is_the_latest(tmp_path):
    d = str(tmp_path)
    r1, p1, sha1 = store.save_labels(d, "hall", 1, "mapsha1", dests_only())
    assert r1 == 1 and store.list_labels(d, "hall") == [1]
    doc, sha = store.load_labels(d, "hall", 1)
    assert sha == sha1 == store.sha256_file(p1)
    assert [(e["label_id"], e["role"]) for e in doc["labels"]] == [
        ("x0100y0000", "dest"),
        ("x0200y0000", "dest"),
    ]
    doc, sha = store.latest_labels(d, "hall", 1)
    assert (doc["revision"], sha) == (1, sha1) and not [e for e in doc["labels"] if e["role"] == "home"]


# ---- with_label --------------------------------------------------------------------------


def test_with_label_replacement_rules():
    s = good_set()
    out = store.with_label(s, entry("x0200y0000", "dest", "UBD"))
    assert [e["name"] for e in out] == ["HOME", "UBC", "UBD"] and len(s) == 2  # a copy
    out = store.with_label(s, entry("x0100y0000", "dest", "UBC-2"))  # same id
    assert [(e["label_id"], e["name"]) for e in out] == [("x0000y0000", "HOME"), ("x0100y0000", "UBC-2")]
    out = store.with_label(s, entry("x0300y0000", "dest", "UBC"))  # same name
    assert [(e["label_id"], e["name"]) for e in out] == [("x0000y0000", "HOME"), ("x0300y0000", "UBC")]
    out = store.with_label(s, entry("x0900y0900", "home", "Gate"))  # a new home replaces the old
    assert [(e["label_id"], e["role"]) for e in out] == [("x0100y0000", "dest"), ("x0900y0900", "home")]
    assert store.check_labels(out) is None
    out = store.with_label(s, entry("x0100y0000", "home", "HOME"))  # a dest re-recorded as home
    assert [(e["label_id"], e["role"]) for e in out] == [("x0100y0000", "home")]
    assert store.with_label([], entry())[0]["label_id"] == "x0000y0000"


def test_with_label_on_a_destinations_only_set():
    s = dests_only()
    out = store.with_label(s, entry("x0300y0000", "dest", "UBE"))  # another dest: appended
    assert [e["name"] for e in out] == ["UBC", "UBD", "UBE"] and len(s) == 2
    assert store.check_labels(out) is None
    out = store.with_label(s, entry())  # HOME added later: nothing replaced
    assert [(e["label_id"], e["role"]) for e in out] == [
        ("x0100y0000", "dest"),
        ("x0200y0000", "dest"),
        ("x0000y0000", "home"),
    ]
    assert store.check_labels(out) is None
    out = store.with_label(out, entry("x0900y0900", "home", "Gate"))  # a second HOME replaces the first
    assert [(e["label_id"], e["role"]) for e in out] == [
        ("x0100y0000", "dest"),
        ("x0200y0000", "dest"),
        ("x0900y0900", "home"),
    ]
    out = store.with_label(s, entry("x0100y0000", "dest", "UBC-2"))  # same id still replaces
    assert [(e["label_id"], e["name"]) for e in out] == [("x0200y0000", "UBD"), ("x0100y0000", "UBC-2")]
    assert store.check_labels(out) is None


# ---- missions bind a label set -----------------------------------------------------------


def test_save_mission_with_and_without_labels(tmp_path):
    d = str(tmp_path)
    args = ("hall", 1, "mapsha", "r1", 1, "rsha")
    lab = {"revision": 1, "sha256": "labsha"}
    p = store.save_mission(d, "with", *args, lab)
    before = open(p, "rb").read()
    assert store.load_mission(d, "with")["labels"] == lab
    assert store.save_mission(d, "with", *args, dict(lab)) == p  # idempotent retry
    for other in (None, {"revision": 2, "sha256": "labsha"}, {"revision": 1, "sha256": "other"}):
        with pytest.raises(store.StoreConflict):
            store.save_mission(d, "with", *args, other)
    assert open(p, "rb").read() == before
    q = store.save_mission(d, "without", *args)
    assert "labels" not in store.load_mission(d, "without")
    assert store.save_mission(d, "without", *args, None) == q
    with pytest.raises(store.StoreConflict):
        store.save_mission(d, "without", *args, lab)  # adding a label set is a different mission
    for bad in ({"revision": 0, "sha256": "x"}, {"revision": True, "sha256": "x"}, {"sha256": "x"}):
        with pytest.raises(store.StoreError, match="revision"):
            store.save_mission(d, "bad", *args, bad)
    assert not os.path.exists(os.path.join(store.missions_dir(d), "bad.yaml"))


@pytest.mark.parametrize(
    "labels",
    [
        {"revision": 0, "sha256": "x"},
        {"revision": "1", "sha256": "x"},
        {"revision": True, "sha256": "x"},
        {"revision": 1},
        {"revision": 1, "sha256": 7},
        [1, "x"],
        "rev1",
    ],
)
def test_load_mission_refuses_a_bad_labels_reference(tmp_path, labels):
    d = str(tmp_path)
    p = store.save_mission(d, "m", "hall", 1, "mapsha", "r1", 1, "rsha", {"revision": 1, "sha256": "x"})
    doc = yaml.safe_load(open(p))
    doc["labels"] = labels
    os.chmod(p, 0o644)
    open(p, "w").write(yaml.safe_dump(doc))
    with pytest.raises(store.StoreError, match="bad labels reference"):
        store.load_mission(d, "m")


def test_mission_labels_checks_the_hash_and_the_map(tmp_path):
    d = str(tmp_path)
    _, _, sha1 = store.save_labels(d, "hall", 1, "mapsha1", good_set())
    _, _, sha2 = store.save_labels(d, "hall", 2, "mapsha2", [entry()])

    def mission(mid, rev, msha, lab):
        store.save_mission(d, mid, "hall", rev, msha, "r1", 1, "rsha", lab)
        return store.load_mission(d, mid)

    assert store.mission_labels(d, mission("none", 1, "mapsha1", None)) is None
    doc = store.mission_labels(d, mission("ok", 1, "mapsha1", {"revision": 1, "sha256": sha1}))
    assert doc["revision"] == 1 and len(doc["labels"]) == 2
    with pytest.raises(store.StoreError, match="hash differs"):
        store.mission_labels(d, mission("badsha", 1, "mapsha1", {"revision": 1, "sha256": sha2}))
    with pytest.raises(store.StoreError, match="is for map hall rev1"):  # a rev1 set on a rev2 mission
        store.mission_labels(d, mission("badrev", 2, "mapsha2", {"revision": 1, "sha256": sha1}))
    with pytest.raises(store.StoreError, match="is for map hall rev2"):  # same revision, other bundle
        store.mission_labels(d, mission("badmap", 2, "othersha", {"revision": 2, "sha256": sha2}))
    with pytest.raises(store.StoreError, match="no such label set"):
        store.mission_labels(d, mission("gone", 1, "mapsha1", {"revision": 9, "sha256": sha1}))


def test_mission_labels_returns_a_destinations_only_set(tmp_path):
    d = str(tmp_path)
    _, _, sha = store.save_labels(d, "hall", 1, "mapsha1", dests_only())
    store.save_mission(d, "m", "hall", 1, "mapsha1", "r1", 1, "rsha", {"revision": 1, "sha256": sha})
    doc = store.mission_labels(d, store.load_mission(d, "m"))
    assert doc["revision"] == 1 and [e["role"] for e in doc["labels"]] == ["dest", "dest"]


# ---- validate(): a wait's label against the bound set --------------------------------------


class Manifest:  # the fields validate() reads from amr_mission.map_bundle.Manifest
    map_id, revision, sha256 = "sim_factory", 1, "abc123"


def wait_route(label="x0100y0000"):
    steps = [
        Step("s1", "straight", to=(15.0, 0.0)),
        Step("w1", "wait", name="UBC hitch", label_id=label),
        Step("s2", "rotate", direction="ccw", angle_deg=180),
        Step("s3", "straight", to=(2.0, 0.0)),
    ]
    return Route("r1", MapRef("sim_factory", 1, "abc123"), StartPose(0.0, 0.0, 0.0), steps, Limits())


def test_validate_without_a_set_is_info_only():
    v = validate(wait_route(), Manifest, build(), FP)
    assert v.ok, [i.to_dict() for i in v.issues]
    lab = [i for i in v.issues if i.code == "label_set"]
    assert (
        len(lab) == 1
        and lab[0].severity == INFO
        and lab[0].step_id == "w1"
        and "x0100y0000" in lab[0].message
    )
    v = validate(wait_route(label=None), Manifest, build(), FP)  # no label: nothing to say
    assert v.ok and not [i for i in v.issues if i.code.startswith("label")]


def test_validate_against_a_set_refuses_an_unknown_label_and_accepts_a_known_one():
    doc = {
        "revision": 4,
        "map": {"id": "sim_factory", "revision": 1, "sha256": "abc123"},
        "labels": good_set(),
    }
    v = validate(wait_route("x0100y0000"), Manifest, build(), FP, labels=doc)
    assert v.ok and not [i for i in v.issues if i.code.startswith("label")], [i.to_dict() for i in v.issues]
    v = validate(wait_route("x0700y0000"), Manifest, build(), FP, labels=doc)
    assert not v.ok
    (e,) = [i for i in v.issues if i.code == "label_unknown"]
    assert e.severity != INFO and e.step_id == "w1" and "x0700y0000" in e.message and "rev4" in e.message
    assert not [i for i in v.issues if i.code == "label_set"]


def test_validate_against_a_destinations_only_set(tmp_path):
    d = str(tmp_path)
    _, _, sha = store.save_labels(d, "sim_factory", 1, "abc123", dests_only())
    store.save_mission(d, "m", "sim_factory", 1, "abc123", "r1", 1, "rsha", {"revision": 1, "sha256": sha})
    doc = store.mission_labels(d, store.load_mission(d, "m"))  # the set the mission binds
    v = validate(wait_route("x0100y0000"), Manifest, build(), FP, labels=doc)
    assert v.ok and not [i for i in v.issues if i.code.startswith("label")], [i.to_dict() for i in v.issues]
    v = validate(wait_route("x0000y0000"), Manifest, build(), FP, labels=doc)  # no HOME in this set
    assert not v.ok and [i for i in v.issues if i.code == "label_unknown"]
