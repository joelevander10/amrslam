import math

from amr_localization import readiness as rd

FRESH = {"scan": 0.01, "wheels": 0.01, "imu": 0.05, "tf": 0.05}


def converged(r: rd.Readiness, t: float) -> None:
    r.on_amcl_pose(t, 0.01, 0.01, 0.002)


def aligned(r: rd.Readiness, t: float) -> None:
    r.on_scan_match(t, match=0.9, long=0.0)


def test_starts_unlocalized_and_ignores_good_data():
    r = rd.Readiness()
    converged(r, 0.0)
    assert r.evaluate(1.0, FRESH) == rd.UNLOCALIZED
    assert not r.confirm()


def test_initialpose_then_settle_then_confirm():
    r = rd.Readiness(rd.Limits(settle_s=2.0))
    r.on_initialpose(0.0)
    assert r.state == rd.CHECKING
    assert r.evaluate(0.1, FRESH) == rd.CHECKING and "AMCL" in r.reason
    converged(r, 0.5)
    aligned(r, 0.5)
    r.evaluate(0.5, FRESH)
    assert not r.can_confirm
    aligned(r, 2.0)
    r.evaluate(2.4, FRESH)
    assert not r.can_confirm  # 1.9 s settled
    r.evaluate(2.6, FRESH)
    assert r.can_confirm
    assert not r.confirmed
    assert r.confirm()
    assert r.state == rd.READY and r.confirmed


def test_cannot_confirm_before_settled():
    r = rd.Readiness()
    r.on_initialpose(0.0)
    converged(r, 0.1)
    r.evaluate(0.2, FRESH)
    assert not r.confirm()
    assert r.state == rd.CHECKING


def test_not_converged_blocks():
    r = rd.Readiness(rd.Limits(cov_xy_max=0.05))
    r.on_initialpose(0.0)
    for t in (0.2, 3.0, 6.0):
        r.on_amcl_pose(t, 0.5, 0.5, 0.2)
        r.evaluate(t, FRESH)
    assert not r.can_confirm and "not converged" in r.reason


def test_jump_during_grace_is_ignored_then_counted():
    r = rd.Readiness(rd.Limits(initial_grace_s=1.5, settle_s=1.0))
    r.on_initialpose(0.0)
    r.on_map_odom(0.1, 0.0, 0.0, 0.0)
    r.on_map_odom(0.2, 0.5, 0.0, 0.0)  # snapping to the initial pose
    assert r.last_jump is None
    r.on_map_odom(2.0, 0.5, 0.0, 0.0)
    r.on_map_odom(2.1, 0.9, 0.0, 0.0)  # 0.4 m after grace
    assert r.last_jump is not None and r.last_jump.dist_m > 0.15
    assert r.state == rd.CHECKING and "corrected" in r.reason


def _ready(settle=1.0) -> rd.Readiness:
    r = rd.Readiness(rd.Limits(settle_s=settle, initial_grace_s=0.5))
    r.on_initialpose(0.0)
    r.on_map_odom(0.1, 0.0, 0.0, 0.0)
    converged(r, 0.6)
    aligned(r, 0.6)
    r.evaluate(0.6, FRESH)
    aligned(r, 2.0)
    r.evaluate(2.0, FRESH)
    assert r.confirm()
    return r


def test_ready_lost_on_jump_and_recover_stopped():
    r = _ready()
    r.on_map_odom(3.0, 0.0, 0.0, 0.0)
    r.on_map_odom(3.1, 0.05, 0.05, math.radians(1.0))  # small correction: fine
    assert r.state == rd.READY
    r.on_map_odom(3.2, 0.05, 0.05, math.radians(7.0))  # 6 deg: over the trigger
    assert r.state == rd.LOST and "corrected" in r.reason
    assert r.evaluate(3.3, FRESH) == rd.LOST  # nothing automatic brings it back
    r.on_initialpose(4.0)
    assert r.state == rd.CHECKING


def test_ready_lost_on_stale_stream():
    r = _ready()
    assert r.evaluate(3.0, {**FRESH, "scan": 0.3}) == rd.LOST
    assert "scan" in r.reason


def test_safety_stop_wheels_excused_while_torque_off_and_graced_briefly():
    """Auto-resume plan 2026-09-19: an STO stops wheel feedback; READY must survive it."""
    r = _ready()
    # the drive report trails the wheel timeout: 0.4 s of grace for wheels alone ...
    assert r.evaluate(3.0, {**FRESH, "wheels": 0.3}) == rd.READY
    # ... then excused for as long as the drives say torque off
    assert r.evaluate(3.1, {**FRESH, "wheels": 30.0}, frozenset({"wheels"})) == rd.READY
    aligned(r, 3.2)
    assert r.evaluate(3.2, FRESH) == rd.READY  # wheels back: nothing to re-confirm
    # not excused and past the grace: lost, as before
    assert r.evaluate(3.3, {**FRESH, "wheels": 0.6}) == rd.LOST and "wheels" in r.reason
    # the grace never covers another stream
    r = _ready()
    assert r.evaluate(3.0, {**FRESH, "wheels": 0.3, "scan": 0.3}) == rd.LOST


def test_ready_lost_on_sustained_covariance_growth_only():
    r = _ready()
    aligned(r, 3.0)  # scan verification keeps coming throughout (Q07): only covariance is at issue
    r.on_amcl_pose(3.0, 0.2, 0.2, 0.05)
    assert r.evaluate(3.0, FRESH) == rd.READY  # one wide sample: a transient
    r.on_amcl_pose(3.5, 0.01, 0.01, 0.002)
    assert r.evaluate(3.5, FRESH) == rd.READY  # recovered, timer resets
    aligned(r, 4.0)
    r.on_amcl_pose(4.0, 0.2, 0.2, 0.05)
    r.evaluate(4.0, FRESH)
    aligned(r, 5.1)
    r.on_amcl_pose(5.1, 0.2, 0.2, 0.05)
    assert r.evaluate(5.1, FRESH) == rd.LOST and "covariance" in r.reason


def test_checking_stale_blocks_but_does_not_lose():
    r = rd.Readiness()
    r.on_initialpose(0.0)
    converged(r, 0.1)
    assert r.evaluate(0.2, {**FRESH, "wheels": None}) == rd.CHECKING
    assert not r.can_confirm and "wheels" in r.reason


def test_reset():
    r = _ready()
    r.reset()
    assert r.state == rd.UNLOCALIZED and not r.confirmed
    assert r.evaluate(5.0, FRESH) == rd.UNLOCALIZED


def test_jump_is_measured_at_the_robot_not_the_odom_origin():
    r = _ready()
    far = (20.0, 0.0, 0.0)  # robot 20 m from the odom origin
    r.on_map_odom(3.0, 0.0, 0.0, 0.0, far)
    # A 0.5 deg yaw correction of the odom frame moves the FRAME's translation
    # by 0.17 m at the robot only if the frame also rotates about the origin:
    # here the map->odom translation compensates, so the robot barely moves.
    d = math.radians(0.5)
    tx, ty = 20.0 - 20.0 * math.cos(d), -20.0 * math.sin(d)  # keeps the robot's map pose at (20, 0)
    r.on_map_odom(3.1, tx, ty, d, far)
    assert r.state == rd.READY, r.reason
    # Whereas the same frame rotation WITHOUT compensation really moves the robot 0.17 m.
    r.on_map_odom(3.2, 0.0, 0.0, 2 * d, far)
    assert r.state == rd.LOST and "corrected" in r.reason


def test_scan_consistency():
    r = rd.Readiness(rd.Limits(settle_s=1.0, initial_grace_s=0.5, match_hold_s=1.0))
    r.on_initialpose(0.0)
    converged(r, 0.6)
    r.on_scan_match(0.6, match=0.4, long=0.0)  # cluttered spot: blocks confirm, nothing more
    r.evaluate(0.6, FRESH)
    r.evaluate(2.0, FRESH)
    assert not r.can_confirm and "match" in r.reason
    r.on_scan_match(2.1, match=0.9, long=0.0)
    r.evaluate(2.1, FRESH)
    r.on_scan_match(3.0, match=0.9, long=0.0)
    r.evaluate(3.2, FRESH)
    assert r.can_confirm and r.confirm()
    r.on_scan_match(4.0, match=0.3, long=0.05)  # heavy clutter while READY: not a loss
    r.on_scan_match(5.5, match=0.3, long=0.05)
    assert r.state == rd.READY
    r.on_scan_match(6.0, match=0.5, long=0.4)  # beams through walls: kidnapped
    assert r.state == rd.READY  # one sample
    r.on_scan_match(6.5, match=0.5, long=0.4)
    assert r.state == rd.READY
    r.on_scan_match(7.1, match=0.5, long=0.4)
    assert r.state == rd.LOST and "through" in r.reason


def test_long_beams_block_confirm():
    r = rd.Readiness(rd.Limits(settle_s=0.5))
    r.on_initialpose(0.0)
    converged(r, 0.1)
    r.on_scan_match(0.2, match=0.9, long=0.5)
    r.evaluate(0.2, FRESH)
    r.evaluate(2.0, FRESH)
    assert not r.can_confirm and "through" in r.reason


# ---- R11: confirmation needs fresh scan-consistency evidence from this attempt ----


def _settled_without_scan(r: rd.Readiness) -> None:
    r.on_initialpose(0.0)
    converged(r, 0.1)
    for t in (0.2, 1.0, 2.0, 3.0):
        r.evaluate(t, FRESH)


def test_r11_no_comparison_cannot_confirm():
    r = rd.Readiness(rd.Limits(settle_s=0.5))
    _settled_without_scan(r)
    assert not r.can_confirm and not r.confirm()
    assert "scan-consistency" in r.reason


def test_r11_tf_failure_after_one_good_comparison_expires():
    r = rd.Readiness(rd.Limits(settle_s=0.5, match_age_max_s=2.0))
    r.on_initialpose(0.0)
    converged(r, 0.1)
    aligned(r, 0.1)
    r.evaluate(0.2, FRESH)
    r.evaluate(0.8, FRESH)
    assert r.can_confirm
    # comparisons stop (TF lookups failing) while scans keep arriving
    r.evaluate(2.2, FRESH)
    assert not r.can_confirm and not r.confirm()
    aligned(r, 2.3)  # fresh evidence restarts the settle period
    r.evaluate(2.3, FRESH)
    assert not r.can_confirm
    r.evaluate(2.9, FRESH)
    assert r.can_confirm


def test_r11_non_finite_comparison_is_unavailable_not_good():
    r = rd.Readiness(rd.Limits(settle_s=0.5))
    r.on_initialpose(0.0)
    converged(r, 0.1)
    aligned(r, 0.1)
    r.on_scan_match(0.2, match=float("nan"), long=0.0)
    r.evaluate(0.3, FRESH)
    r.evaluate(1.0, FRESH)
    assert not r.can_confirm and r.scan_match is None


def test_r11_new_seed_clears_old_covariance_and_scan_evidence():
    r = _ready()
    r.on_initialpose(10.0)
    assert r.cov is None and r.scan_match is None and r.scan_match_t is None
    for t in (10.5, 12.0, 14.0):
        r.evaluate(t, FRESH)
    assert not r.can_confirm and "AMCL" in r.reason
    converged(r, 14.0)
    for t in (14.0, 15.0, 17.0):
        r.evaluate(t, FRESH)
    assert not r.can_confirm and "scan-consistency" in r.reason


def test_r11_delayed_sample_from_earlier_attempt_is_ignored():
    r = rd.Readiness(rd.Limits(settle_s=0.5))
    r.on_initialpose(5.0)
    converged(r, 5.1)
    r.on_scan_match(5.2, match=0.9, long=0.0, stamp=4.9)  # scan taken before the seed
    r.evaluate(5.3, FRESH)
    r.evaluate(6.0, FRESH)
    assert r.scan_match_t is None and not r.can_confirm
    r.on_scan_match(6.1, match=0.9, long=0.0, stamp=6.05)
    r.evaluate(6.1, FRESH)
    r.evaluate(6.7, FRESH)
    assert r.can_confirm


def test_r11_reset_drops_all_proof():
    r = _ready()
    r.reset("map changed")
    assert r.state == rd.UNLOCALIZED and r.reason == "map changed"
    assert r.cov is None and r.scan_match_t is None and r.last_jump is None
    r.on_initialpose(6.0)
    r.evaluate(9.0, FRESH)
    assert not r.can_confirm


# ---- Q07: READY is held on continuing evidence ----


def test_q07_ready_expires_when_scan_verification_stops_but_not_when_amcl_is_quiet():
    r = _ready()
    # a stationary, healthy filter: raw streams fresh, comparisons keep coming, AMCL silent
    for t in (3.0, 10.0, 60.0, 100.0):
        aligned(r, t)
        assert r.evaluate(t, FRESH) == rd.READY, t
    # raw scans keep flowing but the gated comparison stops (gate / transform failure)
    assert r.evaluate(102.5, FRESH) == rd.LOST and "scan-consistency" in r.reason


def test_q07_invalid_covariance_and_pre_seed_samples_are_not_evidence():
    r = _ready()
    aligned(r, 3.0)
    r.on_amcl_pose(3.0, float("nan"), 0.01, 0.002)
    assert r.cov is None
    assert r.evaluate(3.0, FRESH) == rd.READY  # a transient: covariance must stay bad cov_hold_s
    aligned(r, 4.5)
    assert r.evaluate(4.5, FRESH) == rd.LOST and "covariance" in r.reason
    r2 = rd.Readiness(rd.Limits())
    r2.on_initialpose(10.0)
    r2.on_amcl_pose(10.5, 0.01, 0.01, 0.002, stamp=9.0)  # from the previous attempt
    assert r2.cov is None
    r2.on_amcl_pose(10.5, -1.0, 0.01, 0.002)
    assert r2.cov is None
