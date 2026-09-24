import os

import numpy as np
import pytest
from amr_maps.grid import Grid, GridMeta

from amr_maps import edit
from amr_maps import grid as gridio
from amr_mission import map_bundle as mb


def small_grid() -> Grid:
    d = np.full((20, 30), -1, dtype=np.int8)
    d[5:15, 5:25] = 0
    d[5, 5:25] = 100
    return Grid(d, GridMeta(0.05, -1.0, -0.5))


def manifest(rev: int = 1) -> mb.Manifest:
    return mb.Manifest(
        map_id="line_section",
        revision=rev,
        created=mb.now_iso(),
        frame_id="map",
        resolution=0.05,
        origin=[-1.0, -0.5, 0.0],
        width=30,
        height=20,
        start={"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "description": "floor mark A"},
        review={"dx_m": 0.02, "dy_m": -0.01, "dyaw_rad": 0.01, "note": "seams ok"},
    )


def fake_posegraph(tmp_path) -> str:
    stem = str(tmp_path / "pg")
    for ext in (".posegraph", ".data"):
        with open(stem + ext, "wb") as fh:
            fh.write(b"\x00\x01" * 100)
    return stem


def test_stage_verify_publish_load(tmp_path):
    maps = str(tmp_path / "maps")
    stage = mb.staging_dir(maps, "line_section", 1)
    m = mb.stage_bundle(stage, small_grid(), fake_posegraph(tmp_path), manifest())
    assert set(m.files) == {"map.pgm", "map.yaml", "posegraph.posegraph", "posegraph.data"}
    assert mb.list_revisions(maps, "line_section") == []  # still staging
    mb.verify(stage)
    dest = mb.publish(stage, maps, "line_section", 1)
    assert dest.endswith("rev1") and not os.path.exists(stage)
    assert mb.list_revisions(maps, "line_section") == [1]
    assert mb.next_revision(maps, "line_section") == 2
    m2, g = mb.load(maps, "line_section", 1)
    assert m2.sha256 == m.sha256
    assert np.array_equal(g.data, small_grid().data)
    assert (g.meta.origin_x, g.meta.origin_y) == (-1.0, -0.5)


def test_missing_posegraph_fails_before_publish(tmp_path):
    maps = str(tmp_path / "maps")
    stage = mb.staging_dir(maps, "m", 1)
    with pytest.raises(mb.BundleError, match="serialisation missing"):
        mb.stage_bundle(stage, small_grid(), str(tmp_path / "nope"), manifest())
    with pytest.raises(mb.BundleError, match="no pose graph"):
        mb.stage_bundle(stage, small_grid(), None, manifest())
    mb.discard(stage)
    assert mb.list_revisions(maps, "m") == []


def test_corruption_is_caught_by_verify(tmp_path):
    maps = str(tmp_path / "maps")
    stage = mb.staging_dir(maps, "m", 1)
    mb.stage_bundle(stage, small_grid(), fake_posegraph(tmp_path), manifest())
    os.remove(os.path.join(stage, "posegraph.data"))
    with pytest.raises(mb.BundleError, match="missing"):
        mb.verify(stage)
    stage = mb.staging_dir(maps, "m", 2)
    mb.stage_bundle(stage, small_grid(), fake_posegraph(tmp_path), manifest(2))
    with open(os.path.join(stage, "map.pgm"), "ab") as fh:
        fh.write(b"x")
    with pytest.raises(mb.BundleError, match="hash mismatch"):
        mb.verify(stage)


def test_revisions_are_immutable(tmp_path):
    maps = str(tmp_path / "maps")
    stage = mb.staging_dir(maps, "m", 1)
    mb.stage_bundle(stage, small_grid(), fake_posegraph(tmp_path), manifest())
    mb.publish(stage, maps, "m", 1)
    stage2 = mb.staging_dir(maps, "m", 1)
    mb.stage_bundle(stage2, small_grid(), fake_posegraph(tmp_path), manifest())
    with pytest.raises(mb.BundleError, match="already exists"):
        mb.publish(stage2, maps, "m", 1)


def test_empty_file_refused(tmp_path):
    maps = str(tmp_path / "maps")
    stage = mb.staging_dir(maps, "m", 1)
    stem = str(tmp_path / "pg")
    open(stem + ".posegraph", "wb").close()
    open(stem + ".data", "wb").close()
    with pytest.raises(mb.BundleError, match="empty"):
        mb.stage_bundle(stage, small_grid(), stem, manifest())


def test_partial_directory_without_manifest_is_not_a_revision(tmp_path):
    maps = str(tmp_path / "maps")
    os.makedirs(os.path.join(maps, "m", "rev3"))
    assert mb.list_revisions(maps, "m") == []
    assert mb.next_revision(maps, "m") == 1


def test_world_fixture_is_surfaces_only(tmp_path):
    import numpy as np
    from amr_maps.generate_sim_factory import build
    from amr_mission.fixtures import surfaces_only, write_world_as_bundle

    world = build()
    surf = surfaces_only(world)
    solid = world.data >= 65
    assert (surf.data >= 65).sum() < solid.sum() * 0.6  # interiors gone
    r, c = world.world_to_cell(10.0, 2.4)  # deep inside rack row C
    assert world.data[r, c] == 100 and surf.data[r, c] == -1
    r, c = world.world_to_cell(10.0, 1.82)  # its south face
    assert surf.data[r, c] == 100
    assert np.array_equal(surf.data == 0, world.data == 0)  # free space untouched

    dest = write_world_as_bundle(str(tmp_path), "w")
    assert dest.endswith("rev1")
    m, g = mb.load(str(tmp_path), "w", 1)
    assert "surfaces" in m.review["note"]
    assert np.array_equal(g.data, surf.data)


def test_r19_rotated_grid_in_a_bundle_is_a_bundle_error(tmp_path):
    import numpy as np  # noqa: PLC0415
    import pytest  # noqa: PLC0415
    import yaml  # noqa: PLC0415

    from amr_maps import grid as gridio  # noqa: PLC0415
    from amr_mission import map_bundle as mb  # noqa: PLC0415

    grid = gridio.Grid(np.zeros((4, 4), dtype=np.int8), gridio.GridMeta(0.05, 0.0, 0.0))
    gridio.write(grid, str(tmp_path / "map"))
    y = tmp_path / "map.yaml"
    doc = yaml.safe_load(y.read_text())
    doc["origin"] = [0.0, 0.0, 0.5]
    y.write_text(yaml.safe_dump(doc))
    with pytest.raises(mb.BundleError, match="map grid"):
        mb._read_grid(str(y))


# ---- Q14: the bundle identity covers every consumed byte ----


def _staged(tmp_path):
    maps = str(tmp_path / "maps")
    stage = mb.staging_dir(maps, "q14", 1)
    mb.stage_bundle(stage, small_grid(), fake_posegraph(tmp_path), manifest())
    mb.verify(stage)
    return stage


def _relist(stage):
    """Recompute the file table's listed hashes and the bundle hash (an attacker who can
    rewrite the manifest can do this; the point is what the table CANNOT cover)."""
    import yaml  # noqa: PLC0415

    p = os.path.join(stage, mb.MANIFEST)
    doc = yaml.safe_load(open(p))
    doc["files"] = {n: mb.sha256_file(os.path.join(stage, n)) for n in doc["files"]}
    doc["sha256"] = mb.bundle_hash(doc["files"])
    yaml.safe_dump(doc, open(p, "w"))


def test_q14_external_image_cannot_hide_behind_a_verified_bundle(tmp_path):
    stage = _staged(tmp_path)
    outside = tmp_path / "elsewhere.pgm"
    outside.write_bytes(open(os.path.join(stage, "map.pgm"), "rb").read())
    y = os.path.join(stage, "map.yaml")
    txt = open(y).read().replace("image: map.pgm", f"image: {outside}")
    open(y, "w").write(txt)
    _relist(stage)
    with pytest.raises(mb.BundleError, match="plain file name"):
        mb.verify(stage)


def test_q14_traversal_and_symlink_members_and_unlisted_keepout_are_refused(tmp_path):
    import yaml  # noqa: PLC0415

    stage = _staged(tmp_path)
    p = os.path.join(stage, mb.MANIFEST)
    doc = yaml.safe_load(open(p))
    doc["files"]["../escape.pgm"] = "0" * 64
    yaml.safe_dump(doc, open(p, "w"))
    with pytest.raises(mb.BundleError, match="contained|plain file"):
        mb.verify(stage)
    stage = _staged(tmp_path / "b")
    os.remove(os.path.join(stage, "map.pgm"))
    real = tmp_path / "real.pgm"
    real.write_bytes(b"P5\n2 2\n255\n" + bytes([254, 254, 0, 0]))
    os.symlink(str(real), os.path.join(stage, "map.pgm"))
    _relist(stage)
    with pytest.raises(mb.BundleError, match="symlink"):
        mb.verify(stage)
    stage = _staged(tmp_path / "c")
    open(os.path.join(stage, "keepout.yaml"), "w").write("image: keepout.pgm\n")
    with pytest.raises(mb.BundleError, match="keepout.yaml is present but not listed"):
        mb.verify(stage)


def test_q14_manifest_geometry_must_match_the_grid(tmp_path):
    import yaml  # noqa: PLC0415

    stage = _staged(tmp_path)
    p = os.path.join(stage, mb.MANIFEST)
    doc = yaml.safe_load(open(p))
    doc["width"] = doc["width"] + 1
    yaml.safe_dump(doc, open(p, "w"))
    with pytest.raises(mb.BundleError, match="geometry"):
        mb.verify(stage)


# ---- dynamic-mapping plan §1.1/§1.3: derive_edit, dynamic.* and edits.json -------------------


def _published(tmp_path) -> str:
    maps = str(tmp_path / "maps")
    stage = mb.staging_dir(maps, "line_section", 1)
    mb.stage_bundle(stage, small_grid(), fake_posegraph(tmp_path), manifest())
    mb.publish(stage, maps, "line_section", 1)
    return maps


def _rect(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def test_derive_edit_writes_a_new_revision_with_the_mask_and_the_edit_record(tmp_path):
    import json  # noqa: PLC0415

    maps = _published(tmp_path)
    parent = mb.verify(mb.revision_dir(maps, "line_section", 1))
    ops = [
        {"op": "dynamic", "polygon": _rect(-0.5, -0.3, 0.0, 0.0)},
        {"op": "paint", "value": "unknown", "polygon": _rect(0.1, -0.3, 0.25, -0.15)},
    ]
    path, rev, res = mb.derive_edit(maps, "line_section", 1, ops, note="trolley row")
    assert rev == 2 and path.endswith("rev2") and res.dynamic_cells > 0
    m = mb.verify(path)
    # every consumed file is listed and hashed; the pose graph is carried over unchanged
    assert {"dynamic.pgm", "dynamic.yaml", "edits.json", "posegraph.posegraph"} <= set(m.files)
    assert m.files["posegraph.posegraph"] == parent.files["posegraph.posegraph"]
    assert m.sha256 != parent.sha256
    doc = json.load(open(os.path.join(path, "edits.json")))
    assert doc["parent"] == {"revision": 1, "sha256": parent.sha256}
    assert doc["note"] == "trolley row" and [o["op"] for o in doc["ops"]] == ["dynamic", "paint"]
    _, g = mb.load(maps, "line_section", 2)
    dyn = mb.load_mask(path, "dynamic")
    assert dyn is not None and int((dyn.data >= 65).sum()) == res.dynamic_cells
    assert g.data[g.world_to_cell(0.2, -0.225)] == -1  # the wall piece was erased to unknown
    # the parent is untouched
    _, g1 = mb.load(maps, "line_section", 1)
    assert g1.data[g1.world_to_cell(0.2, -0.225)] == 100
    assert mb.load_mask(mb.revision_dir(maps, "line_section", 1), "dynamic") is None


def test_derive_edit_inherits_clears_and_refuses_no_change(tmp_path):
    maps = _published(tmp_path)
    mb.derive_edit(maps, "line_section", 1, [{"op": "dynamic", "polygon": _rect(-0.5, -0.3, 0.0, 0.0)}])
    # a child of rev2 keeps its mask; its edits.json is its own, never the parent's
    p3, _, res3 = mb.derive_edit(
        maps, "line_section", 2, [{"op": "dynamic", "polygon": _rect(0.1, 0.1, 0.3, 0.3)}]
    )
    assert mb.load_edits(p3)["parent"]["revision"] == 2 and len(mb.load_edits(p3)["ops"]) == 1
    assert res3.dynamic_cells > mb.load_edits(p3)["cells"][0]  # rev2's area + the new one
    # clearing every mark leaves the child without dynamic files at all
    p4, _, res4 = mb.derive_edit(
        maps, "line_section", 3, [{"op": "undynamic", "polygon": _rect(-1.0, -0.5, 0.5, 0.5)}]
    )
    assert res4.dynamic_cells == 0 and not os.path.exists(os.path.join(p4, "dynamic.pgm"))
    assert "dynamic.yaml" not in mb.verify(p4).files
    with pytest.raises(edit.EditError, match="change nothing"):
        mb.derive_edit(maps, "line_section", 4, [{"op": "undynamic", "polygon": _rect(-1.0, -0.5, 0.5, 0.5)}])
    with pytest.raises(edit.EditError, match="change nothing"):
        mb.derive_edit(maps, "line_section", 4, [{"op": "dynamic", "polygon": _rect(50, 50, 51, 51)}])
    assert mb.list_revisions(maps, "line_section") == [1, 2, 3, 4]  # refusals publish nothing


def _list_all(stage):
    """Like _relist, but also LISTS every file in the directory (a well-formed manifest)."""
    import yaml  # noqa: PLC0415

    p = os.path.join(stage, mb.MANIFEST)
    doc = yaml.safe_load(open(p))
    doc["files"] = {
        n: mb.sha256_file(os.path.join(stage, n)) for n in sorted(os.listdir(stage)) if n != mb.MANIFEST
    }
    doc["sha256"] = mb.bundle_hash(doc["files"])
    yaml.safe_dump(doc, open(p, "w"))


def test_stray_or_misaligned_dynamic_files_are_refused(tmp_path):
    import shutil  # noqa: PLC0415

    maps = _published(tmp_path)
    path, _, _ = mb.derive_edit(
        maps, "line_section", 1, [{"op": "dynamic", "polygon": _rect(-0.5, -0.3, 0.0, 0.0)}]
    )
    # an unlisted dynamic.pgm next to a revision without one
    stage = _staged(tmp_path / "stray")
    shutil.copy(os.path.join(path, "dynamic.pgm"), os.path.join(stage, "dynamic.pgm"))
    with pytest.raises(mb.BundleError, match="dynamic.pgm is present but not listed"):
        mb.verify(stage)
    # a listed dynamic.pgm without its yaml
    os.remove(os.path.join(stage, "dynamic.pgm"))
    stage = _staged(tmp_path / "noyaml")
    shutil.copy(os.path.join(path, "dynamic.pgm"), os.path.join(stage, "dynamic.pgm"))
    _list_all(stage)
    with pytest.raises(mb.BundleError, match="without dynamic.yaml"):
        mb.verify(stage)
    # a mask in another geometry
    stage = _staged(tmp_path / "misaligned")
    gridio.write(
        Grid(np.zeros((3, 3), dtype=np.int8), GridMeta(0.05, -1.0, -0.5)), os.path.join(stage, "dynamic")
    )
    _list_all(stage)
    with pytest.raises(mb.BundleError, match="not aligned"):
        mb.verify(stage)
