"""U8 / P1-P2 attachment (unified plan §12.2): a plan upload cannot start motion;
a fresh physical Start can authorise exactly one job; authority loss aborts it."""

import json

import pytest

from amr_base import commissioning as cj

CPR = 1_080_000.0
PLAN = json.dumps(
    {"id": "t", "segments": [{"kind": "straight", "distance_m": 0.5}], "speed": {"speed_mps": 0.1}}
)
COMM = cj.LEASE_COMMISSIONING
AUTH = ("sup-a", 3)


def inputs(
    t, counts=(0, 0), start=False, manual=True, lease=COMM, stopped=True, auth=AUTH, cpr=CPR, gyro=None
):
    return cj.Inputs(
        now=t,
        dt=0.02,
        counts=counts,
        counts_per_rev=cpr,
        stopped=stopped,
        panel_valid=True,
        panel_manual=manual,
        start_edge=start,
        lease_allowed=lease,
        gyro_yaw_rad_s=gyro,
        authority=auth,
        start_edge_t=t if start else None,
    )


def plan(job, spec=PLAN, cpr=CPR, now=0.5, auth=AUTH, lease=1, stopped=True, mode="IDLE"):
    ident = auth or AUTH  # a mode snapshot under the lease identity unless the test says otherwise
    snap = None if mode is None else (ident[0], ident[1], mode) if isinstance(mode, str) else mode
    return job.plan(spec, cpr, now=now, authority=auth, lease_allowed=lease, stopped=stopped, mode=snap)


def test_q16_plan_admission_needs_a_fresh_idle_mode_under_the_lease_identity():
    job = cj.Job()
    for bad, msg in (
        (None, "no fresh supervisor mode"),
        ("MAPPING", "in MAPPING"),
        ("TRANSITIONING", "in TRANSITIONING"),
        ("FAULT", "in FAULT"),
        ("NAVIGATION", "in NAVIGATION"),
        (("sup-b", 3, "IDLE"), "disagree"),
        (("sup-a", 4, "IDLE"), "disagree"),
    ):
        with pytest.raises(ValueError, match=msg):
            plan(job, mode=bad)
        assert job.phase == cj.IDLE
    # an inhibited lease (allowed=0) in IDLE is still not a plan-time mover: admitted, no motion
    plan(job, lease=0)
    assert job.phase == cj.PREPARED and job.tick(inputs(1.0)) == (0.0, 0.0)


def test_plan_upload_moves_nothing_and_needs_a_scale():
    job = cj.Job()
    with pytest.raises(ValueError, match="scale"):
        plan(job, cpr=0.0)
    assert job.phase == cj.IDLE
    planned = json.loads(plan(job))
    assert job.phase == cj.PREPARED and planned["segments"][0]["counts"][0] > 0
    for _ in range(50):  # no edge, no motion, whatever else is true
        assert job.tick(inputs(1.0)) == (0.0, 0.0)
    assert job.phase == cj.PREPARED
    with pytest.raises(ValueError, match="number"):
        plan(
            job,
            json.dumps(
                {"segments": [{"kind": "straight", "distance_m": "far"}], "speed": {"speed_mps": 0.1}}
            ),
        )


def test_fresh_start_under_authority_runs_exactly_one_job():
    job = cj.Job()
    plan(job)
    # Start under AUTO, or without the COMMISSIONING class, or while rolling: ignored
    job.tick(inputs(1.0, start=True, manual=False))
    assert job.phase == cj.PREPARED and "panel" in job.reason
    job.tick(inputs(1.0, start=True, lease=1))
    assert job.phase == cj.PREPARED and "supervisor" in job.reason
    job.tick(inputs(1.0, start=True, stopped=False))
    assert job.phase == cj.PREPARED and "rest" in job.reason
    job.tick(inputs(1.0, start=True, stopped=None))
    assert job.phase == cj.PREPARED and "rest" in job.reason
    # the real thing
    assert job.tick(inputs(1.0, start=True)) == (0.0, 0.0)  # baseline tick
    assert job.phase == cj.RUNNING
    wl, wr = job.tick(inputs(1.02))
    assert wl > 0 and wr > 0  # forward, vehicle terms
    # a held Start (edge again) changes nothing; a second edge cannot start a second job
    job.tick(inputs(1.04, start=True))
    assert job.phase == cj.RUNNING and job.run is not None
    run = job.run
    job.tick(inputs(1.06, start=True))
    assert job.run is run
    job.clear()
    assert job.phase == cj.ABORTED and job.tick(inputs(1.08, start=True)) == (0.0, 0.0)
    assert job.phase == cj.ABORTED  # DONE/ABORTED need a new plan before any Start does anything


def test_selector_or_lease_loss_aborts_a_running_job():
    for kind, kw in (("selector", {"manual": False}), ("lease", {"lease": 1}), ("counts", {"counts": None})):
        job = cj.Job()
        plan(job)
        job.tick(inputs(1.0, start=True))
        assert job.tick(inputs(1.02))[0] > 0
        assert job.tick(inputs(1.04, **kw)) == (0.0, 0.0), kind
        assert job.phase == cj.ABORTED, kind
        assert job.snapshot()["phase"] == "ABORTED"
        ev = job.evidence()
        assert ev["started_counts"] == [0, 0] and ev["plan"] is not None


# ---- R13: plans are bound to authority and scale ----


def test_r13_plan_admission_needs_lease_scale_and_known_stillness():
    for kw, why in (
        ({"auth": None}, "lease"),
        ({"lease": 3}, "NAVIGATION"),
        ({"stopped": None}, "rest"),
        ({"stopped": False}, "rest"),
        ({"cpr": float("nan")}, "scale"),
    ):
        job = cj.Job()
        with pytest.raises(ValueError, match=why):
            plan(job, **kw)
        assert job.phase == cj.IDLE, kw


def test_r13_authority_change_drops_prepared_plan():
    for auth in (("sup-a", 4), ("sup-b", 3)):  # new generation, restarted supervisor
        job = cj.Job()
        plan(job)
        job.tick(inputs(1.0, auth=None, lease=0))  # stale lease alone keeps the plan
        assert job.phase == cj.PREPARED
        job.tick(inputs(1.1, auth=auth))
        assert job.phase == cj.IDLE and "changed" in job.reason
        job.tick(inputs(1.2, start=True, auth=auth))
        assert job.phase == cj.IDLE and job.run is None  # never relabelled for the new authority


def test_r13_scale_change_drops_plan_and_aborts_running_job():
    job = cj.Job()
    plan(job)
    job.tick(inputs(1.0, cpr=CPR * 2))
    assert job.phase == cj.IDLE and "scale" in job.reason
    job = cj.Job()
    plan(job)
    job.tick(inputs(1.0, start=True))
    assert job.tick(inputs(1.02))[0] > 0
    assert job.tick(inputs(1.04, cpr=CPR * 2)) == (0.0, 0.0)
    assert job.phase == cj.ABORTED and "scale" in job.reason


def test_r13_authority_change_aborts_running_job():
    job = cj.Job()
    plan(job)
    job.tick(inputs(1.0, start=True))
    assert job.tick(inputs(1.02))[0] > 0
    assert job.tick(inputs(1.04, auth=("sup-a", 4))) == (0.0, 0.0)
    assert job.phase == cj.ABORTED
    ev = job.evidence()
    assert ev["supervisor_instance"] == "sup-a" and ev["supervisor_generation"] == 3


def test_r13_start_edge_must_be_newer_than_plan_acceptance():
    job = cj.Job()
    plan(job, now=1.0)
    i = inputs(1.02, start=True)
    i.start_edge_t = 0.99  # pressed just before the plan was accepted, consumed by a later tick
    job.tick(i)
    assert job.phase == cj.PREPARED and "before the plan" in job.reason
    job.tick(inputs(1.04, start=True))
    assert job.phase == cj.RUNNING


# ---- R31: evidence identity, containment and lifecycle ----


def test_r31_plan_id_must_be_a_plain_identifier():
    for bad in ("../../etc/x", "/tmp/x", "a/b", ".hidden", "..", 7, ["x"], {"a": 1}, "x" * 65, "a b"):
        job = cj.Job()
        spec = json.loads(PLAN)
        spec["id"] = bad
        with pytest.raises(ValueError, match="plan id"):
            plan(job, json.dumps(spec))
        assert job.phase == cj.IDLE
    job = cj.Job()
    spec = json.loads(PLAN)
    spec["id"] = "straight-1m_v2.1"
    plan(job, json.dumps(spec))
    assert job.plan_id == "straight-1m_v2.1"


def test_r31_clear_during_segment_returns_evidence_of_the_aborted_segment():
    job = cj.Job()
    plan(job)
    job.tick(inputs(1.0, start=True))
    for k in range(5):
        job.tick(inputs(1.02 + 0.02 * k, counts=(1000 * k, -1000 * k)))
    ev = job.clear()
    assert ev is not None and ev["phase"] == "ABORTED" and ev["reason"] == "cleared by operator"
    assert ev["plan"] is not None and ev["aborted_segment"]["segment"] == 1
    assert ev["aborted_segment"]["elapsed_s"] > 0
    assert job.clear() is None  # nothing running any more


def test_r31_stale_gyro_intervals_are_marked():
    job = cj.Job()
    plan(job)
    job.tick(inputs(1.0, start=True))
    job.tick(inputs(1.02, gyro=0.0))
    job.tick(inputs(1.04))
    job.tick(inputs(1.06))
    job.tick(inputs(1.08, gyro=0.0))
    job.tick(inputs(1.10))
    gaps = job.evidence()["gyro_gaps_s"]
    assert len(gaps) == 2
    assert gaps[0][0] == pytest.approx(0.02) and gaps[0][1] == pytest.approx(0.06)
    assert gaps[1][1] == pytest.approx(0.10)


def test_r31_new_plan_resets_per_job_evidence():
    job = cj.Job()
    plan(job)
    job.tick(inputs(1.0, start=True))
    job.tick(inputs(1.02))
    job.clear()
    plan(job, now=2.0)
    ev = job.evidence()
    assert ev["aborted_segment"] is None and ev["started_counts"] is None and ev["gyro_gaps_s"] == []
