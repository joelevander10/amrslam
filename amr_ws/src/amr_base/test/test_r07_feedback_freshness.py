"""R07: fresh TPDO1 and fresh TPDO2 are required independently; heartbeats and
SDO replies keep a drive 'alive' but never make its feedback valid."""

import struct
import time
from types import SimpleNamespace

import can
from test_canopen import FakeBus, _link

from amr_base.canopen import ARMED, WheelPosition, wheel_feedback

WHEELS = ((1, True), (2, False))
AGE = 0.05


def _armed():
    bus = FakeBus()
    link = _link(bus)
    link.arm(20, None, 0, 30.0, False, False)
    return bus, link


def _feed(link, now, status=True, position=True, nodes=(1, 2)):
    for n in nodes:
        t = link.telemetry[n]
        if status:
            t.statusword, t.rpm, t.t_status = 0x1637, 0, now
        if position:
            t.position, t.error_register, t.t_position = 1000, 0, now


def test_heartbeat_and_sdo_replies_do_not_hide_a_missing_tpdo():
    bus, link = _armed()
    now = time.monotonic()
    link.t_armed = now - 5.0  # well past the post-arm grace
    _feed(link, now - 1.0)
    _feed(link, now, status=False, nodes=(1,))  # node 1: TPDO2 only
    _feed(link, now, nodes=(2,))
    bus.rx.append(can.Message(arbitration_id=0x701, data=bytes([0x05])))
    link.router.pump(0.01)
    link.read(1, 0x1001)
    now = time.monotonic()
    assert link.silent_nodes(now, 0.6) == []  # transport alive...
    assert link.missing_feedback(now, 0.6) == [1]  # ...but TPDO1 is gone
    assert link.feedback_fresh(1, now, AGE) == (False, True)


def test_missing_feedback_counts_from_the_arm():
    _, link = _armed()
    t0 = link.t_armed
    assert link.missing_feedback(t0 + 0.5, 0.6) == []  # grace: no frame yet, but just armed
    assert link.missing_feedback(t0 + 0.7, 0.6) == [1, 2]
    _feed(link, t0 + 0.65, status=True, position=False)
    assert link.missing_feedback(t0 + 0.7, 0.6) == [1, 2]
    _feed(link, t0 + 0.65)
    assert link.missing_feedback(t0 + 0.7, 0.6) == []


def test_wheel_states_invalid_when_either_tpdo_is_stale_or_not_new():
    _, link = _armed()
    pos = {1: WheelPosition(), 2: WheelPosition()}
    now = time.monotonic()
    _feed(link, now)
    fb = wheel_feedback(link, now, AGE, WHEELS, {1: True, 2: True}, pos)
    assert fb[1][2] and fb[2][2]
    link.telemetry[1].t_status = now - 1.0  # stale status, fresh position
    link.telemetry[2].t_position = now - 1.0  # fresh status, stale position
    fb = wheel_feedback(link, now, AGE, WHEELS, {1: True, 2: True}, pos)
    assert not fb[1][2] and not fb[2][2]
    assert fb[1][1] == 0.0  # no velocity from a stale statusword
    _feed(link, now)
    assert not wheel_feedback(link, now, AGE, WHEELS, {1: False, 2: True}, pos)[1][2]
    link.scale = type(link.scale)(30.0, False, False)  # scale unknown
    assert not any(v for _, _, v in wheel_feedback(link, now, AGE, WHEELS, {1: True, 2: True}, pos).values())


def test_drive_status_operational_needs_fresh_feedback():
    from rclpy.clock import Clock

    from amr_base.drive_node import DriveNode

    _, link = _armed()
    out = []
    node = SimpleNamespace(
        feedback_ms=20,
        get_clock=Clock,
        _safe_publish=lambda pub, m: out.append(m),
        _pub_status=None,
        _status_snapshot={},
        _log=lambda s: None,
    )
    _feed(link, time.monotonic() - 1.0)  # cached Operation enabled, but old
    DriveNode._publish_status(node, link, None)
    assert link.state == ARMED and out[-1].operational is False
    _feed(link, time.monotonic())
    DriveNode._publish_status(node, link, None)
    assert out[-1].operational is True


def test_tpdo_frames_set_their_own_clocks_only():
    bus, link = _armed()
    link.telemetry[1].t_status = link.telemetry[1].t_position = None
    bus.rx.append(can.Message(arbitration_id=0x281, data=struct.pack("<iB", 7, 0)))
    link.router.pump(0.01)
    t = link.telemetry[1]
    assert t.t_position is not None and t.t_status is None
