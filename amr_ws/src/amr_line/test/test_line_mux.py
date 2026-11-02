"""The mux's LINE branch: that the tape command wins, and only when it should.

`amr_base.gating.select()` is the last software arbitration point before the
drive owner, and it is pure, so this runs without a ROS graph.

The branch sits on the AUTO side, ABOVE the LEASE_AUTONOMOUS check, because a
LINE lease carries no AUTONOMOUS bit and would otherwise be refused outright.
It sits above the permit machinery too: the follower holds no MotionPermit, it
holds LEASE_LINE and publishes a twist. Both of those are easy to break by
moving the branch, so they are pinned here.
"""
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_base")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from amr_base import gating  # noqa: E402

NOW = 1000.0
GEN = 7
# Production params. The default leaves require_supervisor False for the bench
# wrappers, and every lease-gated branch - LINE included - is skipped entirely
# in that mode, so a test that forgets this silently exercises nothing.
SUPERVISED = gating.Params(require_supervisor=True)


def lease(allowed=gating.LEASE_LINE, generation=GEN, t=NOW):
    return gating.Lease(t_recv=t, instance="sup-1", generation=generation, seq=1, allowed=allowed)


def panel(auto=True, valid=True, t=NOW):
    return gating.Panel(
        t_recv=t, valid=valid, auto=auto,
        fwd=False, rvs=False, left=False, right=False,
    )


def drives(t=NOW, operational=True):
    return gating.Drives(t_recv=t, operational=operational)


def select(**over):
    kw = dict(
        now=NOW,
        teleop=None,
        follow=None,
        rotate=None,
        permit=None,
        panel=panel(),
        lease=lease(),
        drives=drives(),
        line=gating.Stamped(NOW, 0.2, 0.1),
        p=SUPERVISED,
    )
    kw.update(over)
    return gating.select(**kw)


def test_a_fresh_line_command_is_selected():
    sel = select()
    assert sel.source == gating.LINE
    assert (sel.v, sel.w) == (0.2, 0.1)
    assert sel.reason == "line"
    assert sel.generation == GEN


def test_it_is_a_body_twist_not_per_wheel():
    """Unlike COMMISSIONING, which overloads v/w as left/right wheel rad/s."""
    assert select().wheels is False


def test_no_motion_permit_is_needed():
    """MotionPermit.LINE is reserved and unused: the branch must sit above the
    permit machinery, not inside it."""
    assert select(permit=None).source == gating.LINE


def test_a_stale_line_command_selects_nothing_rather_than_falling_through():
    """An explicit refusal. Falling through would reach the AUTONOMOUS check
    and produce a misleading 'AUTO not allowed by supervisor'."""
    sel = select(line=gating.Stamped(NOW - 5.0, 0.2, 0.1))
    assert sel.source == gating.NONE
    assert sel.reason == "line: no fresh command"
    assert sel.generation == GEN


def test_a_missing_line_command_selects_nothing():
    sel = select(line=None)
    assert sel.source == gating.NONE
    assert sel.reason == "line: no fresh command"


def test_without_the_lease_bit_the_branch_is_not_taken():
    """A lease that does not grant LINE must not reach the branch at all - it
    falls through to the ordinary AUTO path and is refused there."""
    sel = select(lease=lease(allowed=gating.LEASE_MANUAL))
    assert sel.source == gating.NONE
    assert sel.reason != "line: no fresh command"


def test_the_selector_must_be_in_auto():
    """LINE is an AUTO-side source. In MANUAL the branch is unreachable, which
    is what keeps a pendant and a tape follower from ever both being live."""
    sel = select(panel=panel(auto=False))
    assert sel.source != gating.LINE


def test_a_stale_panel_refuses_everything():
    sel = select(panel=panel(t=NOW - 10.0))
    assert sel.source == gating.NONE
    assert sel.reason == "no panel authority"


def test_drives_must_be_operational():
    sel = select(drives=drives(operational=False))
    assert sel.source == gating.NONE
    assert sel.inhibited is True


def test_an_inhibited_lease_beats_the_line_branch():
    """allowed == 0 is the supervisor saying nothing may move. It is checked
    before any source, and LINE must not be an exception to that."""
    sel = select(lease=lease(allowed=0))
    assert sel.source == gating.NONE
    assert sel.inhibited is True


def test_the_topic_is_generation_private():
    """This is what makes freshness a sufficient generation check inside the
    branch: a replaced layer publishes on its own topic, which the mux is no
    longer subscribed to, so its stream cannot look fresh."""
    assert gating.nav_topic("/amr/line_cmd", 0) == "/amr/line_cmd"
    assert gating.nav_topic("/amr/line_cmd", 3) == "/amr/layers/g3/amr/line_cmd"
    assert gating.nav_topic("/amr/line_cmd", 3) != gating.nav_topic("/amr/line_cmd", 4)


def test_line_has_a_name_in_the_mux_tick():
    """gating.NAMES is subscripted in the 50 Hz tick. A missing entry there is
    a KeyError on the hot path."""
    assert gating.NAMES.get(gating.LINE) == "line"


def test_the_lease_bit_is_exclusive_of_the_others():
    """LEASE_LINE is granted alone, so it must not collide with another bit."""
    bits = (gating.LEASE_MANUAL, gating.LEASE_AUTONOMOUS,
            gating.LEASE_COMMISSIONING, gating.LEASE_LINE)
    assert len(set(bits)) == 4
    for b in bits[:-1]:
        assert b & gating.LEASE_LINE == 0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))


# ---- 2026-10-02: no LINE ceiling at the mux; the follower's profile speeds rule --------


def test_the_line_branch_passes_the_followers_command_through():
    """Tracked AUTO cruises at 0.85 m/s (autopilot.auto_rpm). The F07 ceiling of
    0.30 m/s is gone; only the motor limit and the field rules apply after select."""
    sel = select(line=gating.Stamped(NOW, 0.85, 0.4))
    assert sel.source == gating.LINE
    assert (sel.v, sel.w) == (0.85, 0.4)
    assert not hasattr(gating, "line_cap")
