"""DriveLink's pp entry/exit against a scripted bus: verify-before-write, the mode
readback, set-points only for moving wheels, and pv restored on every failure."""

import struct
import time

import pytest
from test_canopen import FakeBus

from amr_base import pp
from amr_base.canopen import ARMED, DriveLink, Router

EXPECT = {"max_torque_permille": 500, "halt_option": 1}
OBJECTS = {"max_torque_permille": 0x6072, "halt_option": 0x605D}


class ModeBus(FakeBus):
    """6061h follows 6060h, as a drive's does (unless told to stick)."""

    def __init__(self, stick_mode=None, **kw):
        super().__init__(**kw)
        self.stick_mode = stick_mode
        for n in (1, 2):
            self.objects[(n, 0x6061, 0)] = pp.MODE_PV
            self.objects[(n, 0x6072, 0)] = 500
            self.objects[(n, 0x605D, 0)] = 1

    def send(self, m):
        super().send(m)
        if 0x601 <= m.arbitration_id <= 0x67F and m.data[0] != 0x40:
            node, idx = m.arbitration_id - 0x600, m.data[1] | (m.data[2] << 8)
            if idx == 0x6060 and self.stick_mode is None:
                self.objects[(node, 0x6061, 0)] = m.data[4]


def armed(bus, rpm=0, position=(1000, -2000)):
    link = DriveLink(Router(bus), {1: "left", 2: "right"}, {"accel": 2400, "decel": 3200}, log=lambda s: None)
    link.state = ARMED
    now = time.monotonic()
    for nid, pos in zip((1, 2), position, strict=True):
        t = link.telemetry[nid]
        t.statusword, t.rpm, t.position, t.t_status, t.t_position = 0x0637, rpm, pos, now, now
    return link


SPEC = pp.MoveSpec(pp.WheelMove(0, 0, 0, 0), pp.WheelMove(500_000, 1273, 800, 800), 3.0)


def idx_writes(bus, index):
    return [(n, v) for n, i, _, v in bus.writes if i == index]


def test_enter_verifies_switches_and_writes_only_the_moving_wheel():
    bus = ModeBus()
    link = armed(bus)
    targets = link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    assert targets == (1000, 498_000)
    assert idx_writes(bus, 0x6060) == [(1, 1), (2, 1)]
    assert idx_writes(bus, 0x607A) == [(2, 498_000)]  # the still wheel gets no set-point
    assert idx_writes(bus, 0x6081) == [(2, 1273)] and idx_writes(bus, 0x6083) == [(2, 800)]
    # nothing on the pp safety configuration was written, only read
    assert not [w for w in bus.writes if w[1] in (0x6072, 0x6065, 0x6067, 0x605D, 0x605E, 0x6085)]


def test_enter_refuses_on_config_mismatch_before_any_write():
    bus = ModeBus()
    bus.objects[(2, 0x6072, 0)] = 1000
    link = armed(bus)
    with pytest.raises(RuntimeError, match="right 6072h"):
        link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    assert bus.writes == []


def test_enter_refuses_unconfigured_profile():
    link = armed(ModeBus())
    with pytest.raises(RuntimeError, match="not configured"):
        link.pp_enter(SPEC, dict(EXPECT, halt_option=None), OBJECTS, max_age=0.5)


def test_enter_refuses_while_turning_or_disarmed():
    with pytest.raises(RuntimeError, match="turning"):
        armed(ModeBus(), rpm=12).pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    link = armed(ModeBus())
    link.state = "disarmed"
    with pytest.raises(RuntimeError, match="not armed"):
        link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)


def test_mode_readback_failure_restores_pv():
    bus = ModeBus(stick_mode=True)  # 6061h never leaves pv
    link = armed(bus)
    link.MODE_READBACK_TRIES = 2
    with pytest.raises(RuntimeError, match="6061h did not read mode 1"):
        link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    # it tried pp on both, then put pv back on both
    assert idx_writes(bus, 0x6060) == [(1, 1), (2, 1), (1, 3), (2, 3)]
    assert not idx_writes(bus, 0x607A)


def test_exit_restores_pv_ramps_and_zero():
    bus = ModeBus()
    link = armed(bus)
    link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    bus.writes.clear()
    link.pp_exit()
    assert idx_writes(bus, 0x6060) == [(1, 3), (2, 3)]
    assert (1, 2400) in idx_writes(bus, 0x6083) and (2, 3200) in idx_writes(bus, 0x6084)
    assert (1, 0) in idx_writes(bus, 0x60FF) and (2, 0) in idx_writes(bus, 0x60FF)
    assert link.applied == (0, 0)


def test_controlwords_go_out_as_rpdo1_with_zero_velocity():
    bus = ModeBus()
    link = armed(bus)
    bus.sent.clear()
    link.send_controlwords(pp.CW_HALTED, pp.CW_START)
    frames = [(m.arbitration_id, struct.unpack("<Hi", bytes(m.data))) for m in bus.sent]
    assert frames == [(0x201, (pp.CW_HALTED, 0)), (0x202, (pp.CW_START, 0))]


# ---- the pp-mode hazards in the ordinary stop paths --------------------------------


def rpdo_frames(bus):
    return [
        (m.arbitration_id, struct.unpack("<Hi", bytes(m.data)))
        for m in bus.sent
        if 0x201 <= m.arbitration_id <= 0x202
    ]


def test_fault_stop_frames_carry_halt_while_in_pp():
    bus = ModeBus()
    link = armed(bus)
    link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    bus.sent.clear()
    link.fault("test")
    # a plain 0x000F would let the halted pp move resume towards its target
    assert rpdo_frames(bus) and all(cw == pp.CW_HALTED for _, (cw, _v) in rpdo_frames(bus))


def test_fault_confirmation_in_pp_uses_actual_speed_not_bit12():
    bus = ModeBus()
    link = armed(bus)
    link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    link.fault("test")
    later = link.t_fault + link.STOP_CONFIRM_S + 0.01
    for nid in (1, 2):
        t = link.telemetry[nid]
        # bit 12 is SET (in pp: set-point acknowledged), but the wheel still turns
        t.statusword, t.rpm, t.t_status = 0x1637, 300 if nid == 2 else 0, later
    link.fault_tick(later, max_age=0.5)
    assert any("node 2" in u and "turning" in u for u in link.stop_unconfirmed)
    assert not any("node 1" in u for u in link.stop_unconfirmed)


def test_exit_holds_halt_until_pv_and_zero_velocity():
    bus = ModeBus()
    link = armed(bus)
    link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    bus.sent.clear()
    bus.writes.clear()
    link.pp_exit()
    assert rpdo_frames(bus)[:2] == [(0x201, (pp.CW_HALTED, 0)), (0x202, (pp.CW_HALTED, 0))]
    order = [i for n, i, _, _ in bus.writes if n == 1]
    assert order.index(0x6060) < order.index(0x60FF) < order.index(0x6083)
    assert link.in_pp is False and link.stop_controlword == 0x000F


def test_failed_exit_keeps_pp_stop_semantics():
    bus = ModeBus()
    link = armed(bus)
    link.pp_enter(SPEC, EXPECT, OBJECTS, max_age=0.5)
    bus.stick_mode = True  # 6061h stays pp
    link.MODE_READBACK_TRIES = 2
    with pytest.raises(RuntimeError, match="did not read mode 3"):
        link.pp_exit()
    assert link.in_pp is True and link.stop_controlword == pp.CW_HALTED
