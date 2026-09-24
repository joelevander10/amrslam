"""R13/R31 at the node boundary, without spinning ROS: evidence files and lease filtering."""

import json
import os
import types

import pytest

from amr_base import commissioning_node as cn


def test_r31_evidence_files_are_contained_and_unique(tmp_path):
    d = tmp_path / "commissioning"
    paths = {cn.write_evidence(str(d), "straight-1m", {"n": k}) for k in range(3)}  # same second
    assert len(paths) == 3
    for p in paths:
        assert os.path.dirname(p) == os.path.realpath(d)
    for bad in ("../x", "/tmp/x", "a/b", "..", ""):
        with pytest.raises(ValueError):
            cn.write_evidence(str(d), bad, {})
    assert len(os.listdir(d)) == 3


def test_r31_write_failure_is_reported_and_leaves_no_partial_file(tmp_path):
    d = tmp_path / "commissioning"
    with pytest.raises(TypeError):
        cn.write_evidence(str(d), "p", {"bad": object()})
    assert os.listdir(d) == []


def test_r31_node_reports_failure_not_an_old_path(tmp_path):
    job = types.SimpleNamespace(reason="done", plan_id="p", evidence=lambda: {"plan_id": "p"})
    errors = []
    node = types.SimpleNamespace(
        evidence_dir=str(tmp_path / "c"),
        job=job,
        _generation=2,
        _results_path="",
        get_logger=lambda: types.SimpleNamespace(error=errors.append),
    )
    cn.CommissioningNode._write_evidence(node)
    good = node._results_path
    assert good and json.load(open(good))["generation"] == 2
    os.chmod(tmp_path / "c", 0o500)
    try:
        if os.access(tmp_path / "c", os.W_OK):
            pytest.skip("running as a user that ignores directory permissions")
        cn.CommissioningNode._write_evidence(node)
    finally:
        os.chmod(tmp_path / "c", 0o700)
    assert node._results_path == "" and "NOT written" in job.reason and errors


def _lease(instance, generation, seq, allowed=1):
    return types.SimpleNamespace(instance=instance, generation=generation, seq=seq, allowed=allowed)


def test_r13_stale_or_replayed_leases_are_ignored():
    node = types.SimpleNamespace(_lease=None, _lease_t=None, _generation=0)
    on = cn.CommissioningNode._on_lease
    on(node, _lease("a", 3, 10))
    on(node, _lease("a", 3, 9))  # reordered
    on(node, _lease("a", 3, 10))  # duplicate
    on(node, _lease("a", 2, 50))  # older generation of the same supervisor
    assert (node._lease.generation, node._lease.seq, node._generation) == (3, 10, 3)
    on(node, _lease("a", 4, 1))  # generation rotation restarts seq
    assert (node._generation, node._lease.seq) == (4, 1)
    on(node, _lease("b", 1, 1))  # a restarted supervisor is a new identity
    assert (node._lease.instance, node._generation) == ("b", 1)
