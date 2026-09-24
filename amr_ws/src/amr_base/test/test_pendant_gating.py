"""The jog pendant at the mux: levels in the panel image become a body twist."""

import pytest

from amr_base.gating import (
    FOLLOW,
    LEASE_AUTONOMOUS,
    LEASE_COMMISSIONING,
    LEASE_MANUAL,
    MANUAL,
    NONE,
    PENDANT,
    TELEOP,
    Drives,
    Lease,
    Manual,
    Panel,
    Params,
    Permit,
    Stamped,
    Wheels,
    select,
)

SUP = Params(require_supervisor=True, teleop_enabled=True)
DRIVES = Drives(t_recv=10.0, operational=True)


def panel(t=10.0, auto=False, valid=True, **held):
    return Panel(t_recv=t, valid=valid, auto=auto, **held)


def lease(allowed=LEASE_MANUAL, t=10.0):
    return Lease(t_recv=t, instance="i1", generation=3, seq=1, allowed=allowed)


def browser(v=0.1):
    return Manual(t=10.0, v=v, w=0.0, instance="i1", generation=3, session="s", seq=1, valid_for_s=0.2)


def test_directions_map_to_a_twist_at_the_configured_speeds():
    p = Params(pendant_v=0.25, pendant_w=0.4, pendant_turn_ratio=0.75, track_m=0.5)
    s = select(10.0, None, None, None, None, panel(fwd=True), p)
    assert (s.source, s.v, s.w, s.reason) == (PENDANT, 0.25, 0.0, "pendant")
    s = select(10.0, None, None, None, None, panel(rvs=True), p)
    assert (s.source, s.v, s.w) == (PENDANT, -0.25, 0.0)
    # LEFT / RIGHT alone: a spin in place at pendant_w
    s = select(10.0, None, None, None, None, panel(left=True), p)
    assert (s.source, s.v, s.w) == (PENDANT, 0.0, 0.4)
    s = select(10.0, None, None, None, None, panel(right=True), p)
    assert (s.source, s.v, s.w) == (PENDANT, 0.0, -0.4)
    # driving + turning: an arc with the slow wheel at 75 % of the fast one, the fast wheel
    # at pendant_v (fast 0.25, slow 0.1875 -> v 0.21875, w = 0.0625 / track)
    s = select(10.0, None, None, None, None, panel(fwd=True, left=True), p)
    assert (s.source, s.v, s.w) == (PENDANT, pytest.approx(0.21875), pytest.approx(0.125))
    s = select(10.0, None, None, None, None, panel(fwd=True, right=True), p)
    assert (s.v, s.w) == (pytest.approx(0.21875), pytest.approx(-0.125))
    s = select(10.0, None, None, None, None, panel(rvs=True, left=True), p)
    assert (s.v, s.w) == (pytest.approx(-0.21875), pytest.approx(0.125))  # same lever sense as before


def test_pendant_defaults_are_the_2026_09_19_manual_speeds():
    from amr_base.diff_drive import Geometry, inverse

    p = Params()
    assert (p.pendant_v, p.pendant_w, p.pendant_turn_ratio) == (0.50, 0.39, 0.75)
    g = Geometry(0.09, p.track_m)
    s = select(10.0, None, None, None, None, panel(fwd=True, left=True), p)
    wl, wr = (x * g.wheel_radius_m for x in inverse(g, s.v, s.w))
    assert wr == pytest.approx(0.50) and wl == pytest.approx(0.375)  # no wheel above the manual speed


def test_no_direction_means_no_pendant_source():
    s = select(10.0, None, None, None, None, panel())
    assert s.source == NONE and s.reason == "MANUAL, no fresh command"


def test_pendant_outranks_browser_jog_and_teleop():
    s = select(10.0, Stamped(10.0, 0.3, 0.0), None, None, None, panel(fwd=True))
    assert s.source == PENDANT
    s = select(10.0, Stamped(10.0, 0.3, 0.0), None, None, None, panel())
    assert s.source == TELEOP
    s = select(10.0, None, None, None, None, panel(fwd=True), SUP, lease(), browser(), DRIVES)
    assert s.source == PENDANT
    s = select(10.0, None, None, None, None, panel(), SUP, lease(), browser(), DRIVES)
    assert s.source == MANUAL


def test_pendant_needs_the_same_authority_as_any_manual_stream():
    assert select(10.0, None, None, None, None, panel(fwd=True, t=9.7)).source == NONE  # stale image
    assert select(10.0, None, None, None, None, panel(fwd=True, valid=False)).source == NONE
    s = select(10.0, None, None, None, Permit(10.0, FOLLOW, True), panel(fwd=True, auto=True))
    assert s.source == NONE  # selector AUTO: the pendant is not an authority
    s = select(10.0, None, None, None, None, panel(fwd=True), SUP, lease(0), None, DRIVES)
    assert s.source == NONE and s.inhibited
    s = select(10.0, None, None, None, None, panel(fwd=True), SUP, lease(LEASE_AUTONOMOUS), None, DRIVES)
    assert s.source == NONE and s.reason == "MANUAL not allowed by supervisor"
    s = select(10.0, None, None, None, None, panel(fwd=True), SUP, lease(), None, Drives(10.0, False))
    assert s.source == NONE and s.inhibited
    s = select(10.0, None, None, None, None, panel(fwd=True), SUP, lease(), None, None)
    assert s.source == NONE and s.inhibited


def test_commissioning_substate_excludes_the_pendant():
    wheels = Wheels(10.0, 1.0, 1.0, generation=3)
    s = select(
        10.0, None, None, None, None, panel(fwd=True), SUP, lease(LEASE_COMMISSIONING), None, DRIVES, wheels
    )
    assert s.source != PENDANT and s.wheels
