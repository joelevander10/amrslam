"""The commissioning job's pp backend: admitted only while the drive owner reports pp
available, one segment, speed caps, a run id held per Start, and every end - done,
halted, stale status, an owner that never takes the move, authority loss, clear -
ends the hold (hold=false for a few ticks)."""

import json

import pytest

from amr_base import commissioning as cj
from amr_base import pp

CPR = 1_080_000.0
AUTH = ("sup-a", 3)
MODE = ("sup-a", 3, "IDLE")


def spec(backend="pp", speed=0.2, segments=None):
    return json.dumps(
        {
            "id": "pp1",
            "backend": backend,
            "segments": segments or [{"kind": "straight", "distance_m": 1.0}],
            "speed": {"speed_mps": speed},
        }
    )


def view(t, available=True, run_id="", outcome="", reason="", targets=(0, 0), actuals=(0, 0)):
    return cj.PpView(t, available, reason, "idle", run_id, outcome, targets, actuals)


def admit(job, s=None, status="ok", now=0.5):
    st = view(now) if status == "ok" else status
    return job.plan(
        s or spec(), CPR, now=now, authority=AUTH, lease_allowed=1, stopped=True, mode=MODE, pp_status=st
    )


def inputs(t, counts=(0, 0), start=False, manual=True, status=None, lease=cj.LEASE_COMMISSIONING):
    return cj.Inputs(
        now=t,
        dt=0.02,
        counts=counts,
        counts_per_rev=CPR,
        stopped=True,
        panel_valid=True,
        panel_manual=manual,
        start_edge=start,
        lease_allowed=lease,
        authority=AUTH,
        start_edge_t=t if start else None,
        pp_status=status,
    )


def running(job=None):
    job = job or cj.Job()
    admit(job)
    assert job.tick(inputs(1.0, start=True, status=view(1.0))) == (0.0, 0.0)
    assert job.phase == cj.RUNNING and job.run_id
    return job


def test_pp_plan_carries_the_per_wheel_move_and_moves_nothing():
    job = cj.Job()
    planned = json.loads(admit(job))
    assert planned["backend"] == "pp" and job.phase == cj.PREPARED
    assert planned["pp"]["left"]["delta_counts"] == planned["segments"][0]["counts"][0]
    assert job.pp_hold() is None  # nothing is held before a Start


@pytest.mark.parametrize(
    "status, fragment",
    [
        (None, "no fresh pp status"),
        (view(0.0 - 1.0), "no fresh pp status"),
        (view(0.5, available=False, reason="pp locked in the profile"), "locked"),
    ],
)
def test_pp_plan_refused_unless_the_owner_reports_pp_available(status, fragment):
    job = cj.Job()
    with pytest.raises(ValueError, match=fragment):
        admit(job, status=status)
    assert job.phase == cj.IDLE


def test_pp_plan_is_one_segment():
    two = [{"kind": "straight", "distance_m": 1.0}, {"kind": "pivot", "angle_deg": 90}]
    with pytest.raises(ValueError, match="exactly one segment"):
        admit(cj.Job(), spec(segments=two))


def test_speed_caps_apply_to_both_backends():
    with pytest.raises(ValueError, match="blind_run.max_speed_mps"):
        admit(cj.Job(), spec(backend="pv", speed=0.85))
    with pytest.raises(ValueError, match="backend must be"):
        admit(cj.Job(), spec(backend="velocity"))


def test_start_refused_while_pp_became_unavailable():
    job = cj.Job()
    admit(job)
    job.tick(inputs(1.0, start=True, status=view(1.0, available=False, reason="drives fault")))
    assert job.phase == cj.PREPARED and "drives fault" in job.reason


def test_start_holds_a_run_id_every_tick_and_never_moves_wheels_itself():
    job = running()
    for k in range(5):
        t = 1.02 + 0.02 * k
        assert job.tick(inputs(t, status=view(t, run_id=job.run_id))) == (0.0, 0.0)
        assert job.pp_hold() == (job.run_id, True)


def test_done_from_the_owner_records_targets_actuals_and_ends_the_hold():
    job = running()
    rid = job.run_id
    job.tick(
        inputs(
            4.0,
            counts=(999_990, 1_000_004),
            status=view(
                4.0, run_id=rid, outcome=pp.DONE, targets=(1_000_000, 1_000_000), actuals=(999_990, 1_000_004)
            ),
        )
    )
    assert job.phase == cj.DONE
    assert job.results[0]["error_counts"] == [-10, 4]
    assert job.pp_hold() is None  # a completed move needs no release
    ev = job.evidence()
    assert ev["backend"] == "pp" and ev["pp_result"]["outcome"] == pp.DONE and ev["pp_move"]["left"]
    assert ev["pp_config"]["vendor_ref"] == cj.config.PP_VENDOR_REF  # whatever the profile records


@pytest.mark.parametrize(
    "make, fragment",
    [
        (
            lambda job, t: inputs(t, status=view(t, run_id=job.run_id, outcome=pp.HALTED, reason="x")),
            "pp halted",
        ),
        (lambda job, t: inputs(t, status=None), "status stale"),
        (lambda job, t: inputs(t, status=view(t - 1.0, run_id=job.run_id)), "status stale"),
        (lambda job, t: inputs(t, manual=False, status=view(t, run_id=job.run_id)), "MANUAL"),
        (lambda job, t: inputs(t, lease=0, status=view(t, run_id=job.run_id)), "commissioning"),
    ],
)
def test_every_early_end_releases_the_hold(make, fragment):
    job = running()
    job.tick(make(job, 1.1))
    assert job.phase == cj.ABORTED and fragment in job.reason
    releases = [job.pp_hold() for _ in range(cj.PP_RELEASE_TICKS + 2)]
    assert releases[: cj.PP_RELEASE_TICKS] == [(job.run_id, False)] * cj.PP_RELEASE_TICKS
    assert releases[-1] is None


def test_owner_that_never_takes_the_move_aborts():
    job = running()
    job.tick(inputs(1.5, status=view(1.5, run_id="someone-else")))
    assert job.phase == cj.RUNNING  # grace period
    job.tick(inputs(2.1, status=view(2.1, run_id="someone-else", reason="pp locked")))
    assert job.phase == cj.ABORTED and "did not take" in job.reason


def test_clear_mid_move_releases_and_keeps_evidence():
    job = running()
    ev = job.clear()
    assert ev is not None and ev["phase"] == "ABORTED" and ev["backend"] == "pp"
    assert job.pp_hold() == (job.run_id, False)


def test_a_new_start_needs_a_new_plan_and_gets_a_new_run_id():
    job = running()
    first = job.run_id
    job.tick(inputs(3.0, status=view(3.0, run_id=first, outcome=pp.DONE)))
    job.tick(inputs(3.5, start=True, status=view(3.5)))
    assert job.phase == cj.DONE  # no replay on another Start
    admit(job, now=4.0)
    job.tick(inputs(4.5, start=True, status=view(4.5)))
    assert job.phase == cj.RUNNING and job.run_id != first


def test_pv_backend_unchanged_by_default():
    job = cj.Job()
    planned = json.loads(
        job.plan(
            json.dumps({"segments": [{"kind": "straight", "distance_m": 0.5}], "speed": {"speed_mps": 0.1}}),
            CPR,
            now=0.5,
            authority=AUTH,
            lease_allowed=1,
            stopped=True,
            mode=MODE,
        )
    )
    assert planned["backend"] == "pv" and "pp" not in planned and job.pp_hold() is None
