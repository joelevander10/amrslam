"""amr_base.canopen without a bus: PDO layout, scaling, the arm policy table,
and the arm/disarm sequences against a scripted fake bus."""

import math
import struct

import can
import pytest

from amr_base import canopen
from amr_base.canopen import (
    ARMED,
    DISARMED,
    FAULT,
    DriveLink,
    MonitorCursor,
    Router,
    WheelScale,
    counts_per_wheel_rev,
    decide,
    decode_tpdo1,
    decode_tpdo2,
    heartbeat_consumer_value,
    tpdo_configuration_steps,
    unwrap_i32,
)

import guard  # noqa: E402  (repo module via agv_repo)

# ---------------------------------------------------------------- layout


def test_tpdo_steps_disable_remap_enable_and_pass_the_guard():
    steps = tpdo_configuration_steps(1, 20)
    # Two PDOs, each: disable, count=0, 2 entries, count=2, type, inhibit, timer, enable.
    assert len(steps) == 18
    for pdo, (comm, mapping, cob) in enumerate(((0x1800, 0x1A00, 0x181), (0x1801, 0x1A01, 0x281))):
        s = steps[pdo * 9 : pdo * 9 + 9]
        assert s[0][:3] == (comm, 1, canopen.COB_DISABLED | cob)
        assert s[1][:3] == (mapping, 0, 0)
        assert s[4][:3] == (mapping, 0, 2)
        assert s[5][:3] == (comm, 2, 255)
        assert s[6][:3] == (comm, 3, 200)  # inhibit 20 ms in 100 us units
        assert s[7][:3] == (comm, 5, 20)
        assert s[-1][:3] == (comm, 1, cob)
    mapped = {(v >> 16) for i, sub, v, _, _ in steps if 0x1A00 <= i <= 0x1A01 and sub}
    assert mapped == {0x6041, 0x606C, 0x6064, 0x1001}
    for index, sub, value, _, _ in steps:
        guard.check(index, value, sub)  # would raise
    with pytest.raises(ValueError):
        tpdo_configuration_steps(1, 0)


def test_tpdo_decoders_match_the_mapping_order():
    sw, rpm = decode_tpdo1(struct.pack("<Hi", 0x0637, -1200))
    assert (sw, rpm) == (0x0637, -1200)
    pos, err = decode_tpdo2(struct.pack("<iB", -5, 0x81))
    assert (pos, err) == (-5, 0x81)
    with pytest.raises(ValueError):
        decode_tpdo1(b"\x00\x00")


def test_consumer_heartbeat_value_and_pc_heartbeat_frame():
    assert heartbeat_consumer_value(100, 500) == (100 << 16) | 500
    with pytest.raises(ValueError):
        heartbeat_consumer_value(0, 500)
    m = canopen.pc_heartbeat_message(100)
    assert m.arbitration_id == 0x764 and bytes(m.data) == b"\x05"
    guard.check(0x1016, heartbeat_consumer_value(100, 500))  # allowed since T9


def test_unwrap_i32():
    assert unwrap_i32(0x7FFFFFF0, -0x7FFFFFF0) == 32
    assert unwrap_i32(-0x7FFFFFF0, 0x7FFFFFF0) == -32
    assert unwrap_i32(10, 3) == -7


# ---------------------------------------------------------------- scaling


def test_wheel_scale_round_trips_and_applies_gear_once():
    s = WheelScale(gear_ratio=30.0, invert_left=False, invert_right=True, counts_per_wheel_rev=30000.0)
    # 1 wheel rad/s -> 30 motor rad/s -> 286.5 r/min
    assert s.motor_rpm(1.0, left=True) == pytest.approx(30.0 * 60.0 / (2 * math.pi))
    assert s.motor_rpm(1.0, left=False) == pytest.approx(-30.0 * 60.0 / (2 * math.pi))
    assert s.wheel_rad_s(s.motor_rpm(0.7, left=False), left=False) == pytest.approx(0.7)
    assert s.wheel_rad(30000, left=True) == pytest.approx(2 * math.pi)
    assert s.wheel_rad(30000, left=False) == pytest.approx(-2 * math.pi)
    assert WheelScale(30.0, False, False).wheel_rad(1, left=True) is None


def test_counts_per_wheel_rev_refuses_disagreement():
    table = {(1, 0x608F, 1): 1000, (1, 0x608F, 2): 1, (1, 0x6091, 1): 1, (1, 0x6091, 2): 1}
    table.update({(2, 0x608F, 1): 1000, (2, 0x608F, 2): 1, (2, 0x6091, 1): 1, (2, 0x6091, 2): 1})
    rd = lambda n, i, s: table.get((n, i, s))  # noqa: E731
    assert counts_per_wheel_rev(rd, (1, 2), 30.0) == pytest.approx(30000.0)
    table[(2, 0x608F, 1)] = 2000
    assert counts_per_wheel_rev(rd, (1, 2), 30.0) is None
    table[(2, 0x608F, 1)] = 1000
    table[(1, 0x6091, 1)] = 7  # a gear the profile does not know
    assert counts_per_wheel_rev(rd, (1, 2), 30.0) is None
    assert counts_per_wheel_rev(lambda *a: None, (1, 2), 30.0) is None


def test_target_rpm_watchdog_and_clamp():
    from amr_base.canopen import target_rpm

    s = WheelScale(30.0, False, False)
    assert target_rpm(None, 10.0, 0.2, s, 4000) == (0, 0)
    assert target_rpm((9.7, 1.0, 1.0), 10.0, 0.2, s, 4000) == (0, 0)  # 0.3 s old: watchdog
    assert target_rpm((9.9, 1.0, -1.0), 10.0, 0.2, s, 4000) == (286, -286)
    assert target_rpm((9.9, 100.0, 100.0), 10.0, 0.2, s, 4000) == (4000, 4000)  # clamped


# ---------------------------------------------------------------- policy


def test_decide_table():
    assert decide(DISARMED, False, 0, 0, [], [], []).action == "none"
    assert decide(ARMED, False, 0, 0, [], [], []).action == "disarm"
    assert decide(FAULT, False, 0, 0, [], [], []).action == "disarm"
    assert decide(DISARMED, True, 10.0, 12.0, [], [], []).action == "none"  # backoff
    assert decide(DISARMED, True, 12.0, 12.0, [], [], []).action == "arm"
    # Q02: owed cleanup is retried on the backoff and blocks arming, wanted or not
    assert decide(DISARMED, True, 12.0, 12.0, [], [], [], cleanup_owed=True).action == "cleanup"
    assert decide(DISARMED, False, 12.0, 12.0, [], [], [], cleanup_owed=True).action == "cleanup"
    assert decide(DISARMED, True, 10.0, 12.0, [], [], [], cleanup_owed=True).action == "none"
    assert decide(ARMED, True, 0, 0, [], [], []).action == "none"
    assert decide(ARMED, True, 0, 0, [1], [], []).action == "fault"
    assert decide(ARMED, True, 0, 0, [], [], [2]).action == "fault"
    # ETO drop-out: disarm and re-arm with zero, not a latched fault
    assert decide(ARMED, True, 0, 0, [], [2], []).action == "disarm"
    assert decide(FAULT, True, 99.0, 0, [], [], []).action == "none"  # latched until ack


# ---------------------------------------------------------------- fake bus


class FakeBus:
    """Answers SDOs from a table, records writes, and can be told to push frames."""

    def __init__(self, statusword=0x0637):
        self.sent: list[can.Message] = []
        self.rx: list[can.Message] = []
        self.objects = {}
        self.writes = []
        self.statusword = statusword
        for n in (1, 2):
            self.objects[(n, 0x1000, 0)] = 0x00020192
            self.objects[(n, 0x1001, 0)] = 0
            self.objects[(n, 0x608F, 1)], self.objects[(n, 0x608F, 2)] = 1000, 1
            self.objects[(n, 0x6091, 1)], self.objects[(n, 0x6091, 2)] = 1, 1

    def _reply(self, node, data):
        self.rx.append(can.Message(arbitration_id=0x580 + node, data=data, is_extended_id=False))

    def send(self, m):
        self.sent.append(m)
        if 0x601 <= m.arbitration_id <= 0x67F:
            node = m.arbitration_id - 0x600
            cs, idx, sub = m.data[0], m.data[1] | (m.data[2] << 8), m.data[3]
            if cs == 0x40:  # upload
                if (idx, sub) == (0x6041, 0):
                    val = self.statusword
                else:
                    val = self.objects.get((node, idx, sub))
                if val is None:
                    self._reply(node, bytes([0x80, idx & 0xFF, idx >> 8, sub]) + b"\x00\x00\x02\x06")
                else:
                    self._reply(node, bytes([0x4B, idx & 0xFF, idx >> 8, sub]) + struct.pack("<I", val))
            else:  # download
                size = {0x2F: 1, 0x2B: 2, 0x27: 3, 0x23: 4}[cs]
                self.writes.append((node, idx, sub, int.from_bytes(bytes(m.data[4 : 4 + size]), "little")))
                self._reply(node, bytes([0x60, idx & 0xFF, idx >> 8, sub, 0, 0, 0, 0]))

    def recv(self, timeout=None):
        return self.rx.pop(0) if self.rx else None

    def shutdown(self):
        pass


def _link(bus):
    router = Router(bus)
    return DriveLink(router, {1: "left", 2: "right"}, {"accel": 2400, "decel": 3200}, log=lambda s: None)


def test_arm_configures_pdos_guard_and_enables_both_drives():
    bus = FakeBus()
    link = _link(bus)
    link.arm(20, 100, 500, 30.0, False, False)
    assert link.state == ARMED
    assert link.scale.counts_per_wheel_rev == pytest.approx(30000.0)
    w = bus.writes
    idx = [i for _, i, _, _ in w]
    # RPDO1 (disable, type, enable) + TPDO1/TPDO2 (disable, type, inhibit, timer, enable) per node
    assert idx.count(0x1400) == 6 and idx.count(0x1800) == 10 and idx.count(0x1801) == 10
    assert (1, 0x1016, 1, (100 << 16) | 500) in w and (2, 0x1016, 1, (100 << 16) | 500) in w
    cws = [(n, v) for n, i, _, v in w if i == 0x6040]
    # 0x000F: motion extension (bit 13 = 0), required for the 400 W geared motor
    assert cws == [(1, 0x06), (1, 0x07), (1, 0x000F), (2, 0x06), (2, 0x07), (2, 0x000F)]
    assert (1, 0x60FF, 0, 0) in w  # armed with a zero target
    # PDO config happened while pre-operational (NMT 0x80 broadcast came before)
    nmt = [(m.data[0], m.data[1]) for m in bus.sent if m.arbitration_id == 0]
    assert nmt[0] == (0x80, 0) and (0x01, 1) in nmt and (0x01, 2) in nmt


def test_failed_enable_rolls_back_both_drives():
    bus = FakeBus(statusword=0x0240)  # Switch on disabled, never reaches 0x27
    link = _link(bus)
    with pytest.raises(RuntimeError):
        link.arm(20, None, 0, 30.0, False, False)
    assert link.state == DISARMED
    disable = [(n, v) for n, i, _, v in bus.writes if i == 0x6040 and v in (0x06, 0x00)]
    assert (1, 0x00) in disable and (2, 0x00) in disable


def test_preflight_refuses_a_fault_or_remote_clear():
    bus = FakeBus(statusword=0x0008)  # FAULT, remote clear
    ok, report = _link(bus).preflight()
    assert not ok and any("FAULT" in r for r in report) and any("Remote" in r for r in report)


def test_send_target_is_rpdo_and_never_carries_fault_reset():
    bus = FakeBus()
    link = _link(bus)
    link.state = ARMED
    link.send_target(1200.4, -300)
    f = [m for m in bus.sent if m.arbitration_id in (0x201, 0x202)]
    assert [(m.arbitration_id, struct.unpack("<Hi", bytes(m.data))) for m in f] == [
        (0x201, (0x000F, 1200)),
        (0x202, (0x000F, -300)),
    ]
    assert link.applied == (1200, -300)


def test_router_decodes_pushed_frames_and_tracks_liveness():
    bus = FakeBus()
    link = _link(bus)
    bus.rx.append(can.Message(arbitration_id=0x181, data=struct.pack("<Hi", 0x0637, 150)))
    bus.rx.append(can.Message(arbitration_id=0x282, data=struct.pack("<iB", 4242, 0)))
    bus.rx.append(can.Message(arbitration_id=0x081, data=bytes([0x10, 0x31, 0x04, 0, 0, 0, 0, 0])))
    bus.rx.append(can.Message(arbitration_id=0x702, data=bytes([0x05])))
    link.router.pump(0.01)
    assert link.telemetry[1].statusword == 0x0637 and link.telemetry[1].rpm == 150
    assert link.telemetry[1].operation_enabled is True
    assert link.telemetry[2].position == 4242 and link.telemetry[2].nmt == "Operational"
    assert link.telemetry[1].alarm is not None and 1 in link.faulted_nodes()
    assert link.silent_nodes(now=1e9, timeout_s=0.6) == [1, 2]


def test_fault_zeroes_at_once_and_disarm_de_energises():
    bus = FakeBus()
    link = _link(bus)
    link.arm(20, None, 0, 30.0, False, False)
    bus.sent.clear()
    link.fault("drive silent")
    assert link.state == FAULT
    rpdos = [m for m in bus.sent if m.arbitration_id in (0x201, 0x202)]
    assert all(struct.unpack("<Hi", bytes(m.data))[1] == 0 for m in rpdos) and len(rpdos) == 2
    link.disarm()
    assert link.state == DISARMED
    assert [(n, v) for n, i, _, v in bus.writes if i == 0x6040][-2:] == [(2, 0x06), (2, 0x00)]


def test_disarm_retires_the_pc_loss_guard_first():
    """A clean exit must not look like a crash to the drives.

    Found on the vehicle 2026-09-16: drive_node exited cleanly, 1016h stayed
    set, both drives raised 8130h half a second later and agv_controller could
    not arm (40C0h is deny-listed, so only a power cycle clears it).
    """
    bus = FakeBus()
    link = _link(bus)
    link.arm(20, 100, 500, 30.0, False, False)
    assert link.pc_guard_set
    bus.writes.clear()
    link.disarm()
    hb = [(n, v) for n, i, sub, v in bus.writes if i == 0x1016 and sub == 1]
    assert hb == [(1, 0), (2, 0)]
    # ...and before the zero setpoint / controlword sequence, while our own
    # heartbeat is still fresh, not after a speed-zero wait the drives may not survive.
    first_cw = next(k for k, (n, i, _, _) in enumerate(bus.writes) if i in (0x60FF, 0x6040))
    assert all(bus.writes[k][1] == 0x1016 for k in range(len(hb))) and first_cw >= len(hb)
    assert not link.pc_guard_set

    # Without the guard armed (pc_loss_ms=0) nothing is written to 1016h at all.
    bus2 = FakeBus()
    link2 = _link(bus2)
    link2.arm(20, None, 0, 30.0, False, False)
    bus2.writes.clear()
    link2.disarm()
    assert not any(i == 0x1016 for _, i, _, _ in bus2.writes)


def test_monitor_cursor_reads_one_object_per_node_per_call_and_keeps_timeouts_apart():
    """U7: the diagnostic slot is bounded (one SDO per call) and a timeout is
    stored as None, never a plausible number."""
    import canmon  # repo module via agv_repo

    poller = canmon.MonitorPoller({1: "left", 2: "right"}, objects=canmon.OBJECTS[:2])
    cur = MonitorCursor(poller, [1, 2])
    calls = []

    def read(nid, idx):
        calls.append((nid, idx))
        if nid == 2:
            return None  # right drive times out
        return 0xFFFF & 0x01F4  # 500 raw -> 50.0 V for an i16 x0.1 object

    out = [cur.step(read, canmon._decode) for _ in range(4)]
    assert len(calls) == 4  # exactly one read per call
    assert [c[0] for c in calls] == [1, 2, 1, 2]  # alternating nodes
    assert calls[0][1] == calls[1][1] == canmon.OBJECTS[0][0]  # same object for both nodes
    assert calls[2][1] == canmon.OBJECTS[1][0]  # then the next object
    assert out[0][2] == 500 and out[1][2] is None
    snap = poller.snapshot({})["nodes"]
    assert snap["1"]["bus_v"]["value"] == 50.0 and snap["2"]["bus_v"]["value"] is None
    assert cur.reads == 4 and cur.timeouts == 2
