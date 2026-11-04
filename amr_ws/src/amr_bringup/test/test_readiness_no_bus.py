"""2026-10-27: a missing CAN adapter is named as such at boot, not as "drives".

"drives" and "wheel feedback" both tell the operator to press the cabinet safety
reset, which does nothing for an unplugged USB adapter.
"""

import time
from types import SimpleNamespace

from amr_bringup import readiness as rd


def _status(link_state):
    return SimpleNamespace(
        operational=False, link_state=link_state, left_state=link_state, right_state=link_state
    )


def test_no_bus_is_reported_as_the_can_bus():
    s = rd.Snapshots()
    s.on_drives(_status(rd.NO_BUS))
    missing = rd.base_missing(s, time.monotonic())
    assert "CAN bus" in missing
    assert "drives" not in missing and "wheel feedback" not in missing


def test_a_linked_base_without_wheels_still_says_wheel_feedback():
    s = rd.Snapshots()
    s.on_drives(_status("standby"))
    missing = rd.base_missing(s, time.monotonic())
    assert "wheel feedback" in missing and "CAN bus" not in missing
