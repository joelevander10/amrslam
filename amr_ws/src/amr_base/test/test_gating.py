from amr_base.gating import FOLLOW, NONE, ROTATE, TELEOP, Panel, Permit, Stamped, select

MANUAL = Panel(t_recv=10.0, valid=True, auto=False)
AUTO = Panel(t_recv=10.0, valid=True, auto=True)


def test_no_panel_no_motion():
    assert select(10.0, Stamped(10.0, 0.3, 0.0), None, None, None, None).source == NONE
    stale = Panel(t_recv=9.7, valid=True, auto=False)
    assert select(10.0, Stamped(10.0, 0.3, 0.0), None, None, None, stale).source == NONE
    invalid = Panel(t_recv=10.0, valid=False, auto=False)
    assert select(10.0, Stamped(10.0, 0.3, 0.0), None, None, None, invalid).source == NONE


def test_teleop_only_under_manual():
    s = select(10.0, Stamped(9.9, 0.3, 0.1), None, None, None, MANUAL)
    assert (s.source, s.v, s.w) == (TELEOP, 0.3, 0.1)
    s = select(10.0, Stamped(9.9, 0.3, 0.1), None, None, None, AUTO)
    assert s.source == NONE and s.v == 0.0


def test_teleop_window_does_not_extend_motion():
    # 0.35 s old: still owns the mux (0.5 s window) but the command has timed out (0.2 s)
    s = select(
        10.0, Stamped(9.65, 0.3, 0.0), Stamped(10.0, 0.3, 0.0), None, Permit(10.0, FOLLOW, True), MANUAL
    )
    assert s.source == TELEOP and s.v == 0.0
    # 0.6 s old: window gone, and under MANUAL the follow permit is not honoured either
    s = select(
        10.0, Stamped(9.4, 0.3, 0.0), Stamped(10.0, 0.3, 0.0), None, Permit(10.0, FOLLOW, True), MANUAL
    )
    assert s.source == NONE


def test_auto_needs_a_fresh_enabled_permit_matching_the_source():
    follow, rotate = Stamped(10.0, 0.3, 0.0), Stamped(10.0, 0.0, 0.3)
    assert select(10.0, None, follow, rotate, None, AUTO).source == NONE
    assert select(10.0, None, follow, rotate, Permit(9.6, FOLLOW, True), AUTO).source == NONE  # expired
    assert select(10.0, None, follow, rotate, Permit(10.0, FOLLOW, False), AUTO).source == NONE
    s = select(10.0, None, follow, rotate, Permit(10.0, FOLLOW, True), AUTO)
    assert (s.source, s.v) == (FOLLOW, 0.3)
    s = select(10.0, None, follow, rotate, Permit(10.0, ROTATE, True), AUTO)
    assert (s.source, s.w) == (ROTATE, 0.3)
    s = select(10.0, None, Stamped(9.7, 0.3, 0.0), rotate, Permit(10.0, FOLLOW, True), AUTO)
    assert s.source == NONE and "fresh" in s.reason


def test_teleop_command_is_ignored_under_auto_even_with_permit():
    s = select(10.0, Stamped(10.0, 0.5, 0.0), Stamped(10.0, 0.3, 0.0), None, Permit(10.0, FOLLOW, True), AUTO)
    assert (s.source, s.v) == (FOLLOW, 0.3)


def test_q04_route_caps_in_the_permit_clamp_the_permitted_source():
    from amr_base.gating import capped

    follow = Stamped(10.0, 0.30, 0.20)
    rotate = Stamped(10.0, 0.0, 0.30)
    s = select(10.0, None, follow, rotate, Permit(10.0, FOLLOW, True, v_max=0.10, w_max=0.15), AUTO)
    assert (s.source, s.v, s.w) == (FOLLOW, 0.10, 0.15)
    s = select(10.0, None, follow, rotate, Permit(10.0, ROTATE, True, v_max=0.10, w_max=0.15), AUTO)
    assert (s.source, s.v, s.w) == (ROTATE, 0.0, 0.15)
    # no cap (0), a negative or nonfinite cap: the command passes unchanged (hardware limits apply later)
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        assert capped(0.3, bad) == 0.3
    s = select(10.0, None, follow, rotate, Permit(10.0, FOLLOW, True), AUTO)
    assert (s.v, s.w) == (0.30, 0.20)
    # a reversed command is clamped symmetrically
    assert capped(-0.3, 0.1) == -0.1


def test_autonomous_caps_040_024_and_a_slower_route_wins():
    from amr_base.gating import capped

    fast = Stamped(10.0, 0.9, 0.9)
    s = select(10.0, None, fast, fast, Permit(10.0, FOLLOW, True, v_max=0.40, w_max=0.24), AUTO)
    assert (s.v, s.w) == (0.40, 0.24)
    s = select(
        10.0, None, Stamped(10.0, -0.9, -0.9), fast, Permit(10.0, FOLLOW, True, v_max=0.40, w_max=0.24), AUTO
    )
    assert (s.v, s.w) == (-0.40, -0.24)  # sign kept
    s = select(10.0, None, fast, fast, Permit(10.0, FOLLOW, True, v_max=0.15, w_max=0.10), AUTO)
    assert (s.v, s.w) == (0.15, 0.10)  # a route authored slower than the vehicle cap stays slower
    assert capped(0.9, 0.24) == 0.24 and capped(-0.9, 0.24) == -0.24


def test_names_cover_every_source_and_the_lease_bits_are_distinct():
    """gating.NAMES is subscripted in the 50 Hz mux tick and MuxState.msg mirrors these
    numbers, so a source added without a name is a KeyError on the control path."""
    import amr_base.gating as g

    # Discovered, not listed: a source added later must fail this without anyone
    # remembering to extend the test. Every uppercase int global that is not a lease
    # bit IS a source constant (gating.py has no other integer constants).
    sources = {
        v
        for k, v in vars(g).items()
        if k.isupper() and not k.startswith("LEASE_") and isinstance(v, int) and not isinstance(v, bool)
    }
    assert sources == set(g.NAMES), f"sources without a NAMES entry: {sorted(sources - set(g.NAMES))}"
    assert g.PENDANT == 6  # MuxState.msg pins these numbers
    # LINE (7) and LEASE_LINE (bit 8) are retired with the tracked product
    assert not hasattr(g, "LINE") and not hasattr(g, "LEASE_LINE")
    assert len(set(g.NAMES.values())) == len(g.NAMES)  # no two sources share a name
    # the lease classes are single, distinct bits of ControlLease.allowed
    bits = [g.LEASE_MANUAL, g.LEASE_AUTONOMOUS, g.LEASE_COMMISSIONING]
    assert bits == [1, 2, 4]
    assert all(b and not (b & (b - 1)) for b in bits)  # each is one bit


def test_estop_inhibits_every_source_ahead_of_the_lease():
    """Emergency plan §4.3: not NORMAL is NONE, inhibited, code ESTOP, whatever is asking."""
    from amr_base.gating import LEASE_AUTONOMOUS, LEASE_MANUAL, PENDANT, Drives, Lease, Manual, Params

    sup = Params(require_supervisor=True)
    lease = Lease(10.0, "i", 1, 1, LEASE_MANUAL | LEASE_AUTONOMOUS)
    drives = Drives(10.0, True)
    for emg in (1, 2, 3, 4):
        manual = Panel(10.0, True, False, fwd=True, emergency=emg)
        s = select(10.0, Stamped(10.0, 0.3, 0.0), None, None, None, manual, sup, lease=lease,
                   manual=Manual(10.0, 0.2, 0.0, "i", 1, "s", 1, 0.5), drives=drives)
        assert (s.source, s.inhibited, s.code) == (NONE, True, "ESTOP"), emg
        auto = Panel(10.0, True, True, emergency=emg)
        s = select(10.0, None, Stamped(10.0, 0.3, 0.0), None, Permit(10.0, FOLLOW, True, "i", 1), auto, sup,
                   lease=Lease(10.0, "i", 1, 1, 0), drives=Drives(10.0, False))
        assert s.code == "ESTOP", "ahead of INHIBITED / DRIVES_NOT_READY"
    # NORMAL: the pendant drives again (the re-press rule is the node's, below)
    pendant = Panel(10.0, True, False, fwd=True)
    s = select(10.0, None, None, None, None, pendant, sup, lease=lease, drives=drives)
    assert s.source == PENDANT
    # a stale image is PANEL_STALE's, not a false emergency
    s = select(10.0, None, None, None, None, Panel(9.0, False, False, emergency=1))
    assert s.code == "PANEL_STALE"


def test_repress_guard_needs_one_release_after_normal():
    from amr_base.gating import RepressGuard

    g = RepressGuard()
    assert not g.update(0, True), "a pendant held with no emergency is just driving"
    assert g.update(1, True)
    assert g.update(2, False), "released during the emergency does not count"
    assert g.update(0, True), "held through the Reset: nothing moves"
    assert not g.update(0, False)
    assert not g.update(0, True), "a fresh press after the release drives"
