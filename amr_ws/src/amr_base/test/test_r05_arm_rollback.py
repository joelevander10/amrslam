"""R05: arming rolls back per node wherever it fails, disarm does not trust a
software DISARMED state while hardware cleanup is owed, and a deliberate exit
zeroes before retiring 1016h without giving up the measured order (1016h
before the speed-zero wait, commit 6515799)."""

import pytest
from test_canopen import FakeBus, _link

from amr_base.canopen import ARMED, DISARMED

HB = (100 << 16) | 500


class AbortBus(FakeBus):
    """SDO downloads matching `abort(node, index, sub, value)` get an abort reply."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.abort = lambda n, i, s, v: False
        self.order = []  # ("rpdo"|"sdo"|"hb", node, index)

    def send(self, m):
        cob = m.arbitration_id
        if 0x201 <= cob <= 0x27F:
            self.order.append(("rpdo", cob - 0x200, None))
        elif cob == 0x764:
            self.order.append(("hb", None, None))
        elif 0x601 <= cob <= 0x67F and m.data[0] != 0x40:
            node, idx, sub = cob - 0x600, m.data[1] | (m.data[2] << 8), m.data[3]
            size = {0x2F: 1, 0x2B: 2, 0x27: 3, 0x23: 4}[m.data[0]]
            val = int.from_bytes(bytes(m.data[4 : 4 + size]), "little")
            self.order.append(("sdo", node, idx))
            if self.abort(node, idx, sub, val):
                self.sent.append(m)
                self._reply(node, bytes([0x80, idx & 0xFF, idx >> 8, sub]) + b"\x00\x00\x09\x06")
                return
        super().send(m)


def _disabled(bus):
    return {n for n, i, _, v in bus.writes if i == 0x6040 and v == 0x00}


def test_second_node_guard_failure_rolls_back_the_first_nodes_guard():
    bus = AbortBus(statusword=0x1637)
    bus.abort = lambda n, i, s, v: (n, i, v) == (2, 0x1016, HB)
    link = _link(bus)
    with pytest.raises(RuntimeError, match="1016h"):
        link.arm(20, 100, 500, 30.0, False, False)
    assert (1, 0x1016, 1, HB) in bus.writes
    assert (1, 0x1016, 1, 0) in bus.writes  # tracked as soon as it landed, so retired
    assert not link.pc_guard_set and link.state == DISARMED and not link.cleanup_owed
    assert _disabled(bus) == {1, 2}


def test_pdo_setup_failure_is_inside_the_rollback():
    bus = AbortBus(statusword=0x1637)
    bus.abort = lambda n, i, s, v: (n, i) == (2, 0x1801)
    link = _link(bus)
    with pytest.raises(RuntimeError):
        link.arm(20, 100, 500, 30.0, False, False)
    assert _disabled(bus) == {1, 2} and link.state == DISARMED
    assert not any(i == 0x1016 for _, i, _, _ in bus.writes)  # never reached, nothing to retire


def test_post_enable_read_failure_is_inside_the_rollback():
    bus = FakeBus(statusword=0x1637)
    link = _link(bus)
    real = link.read

    def read(n, i, s=0, timeout=0.4):
        if i == 0x608F:
            raise OSError("bus gone mid-scale-read")
        return real(n, i, s, timeout)

    link.read = read
    with pytest.raises(OSError):
        link.arm(20, 100, 500, 30.0, False, False)
    assert link.state == DISARMED and _disabled(bus) == {1, 2}
    assert [(n, v) for n, i, _, v in bus.writes if i == 0x1016 and v == 0] == [(1, 0), (2, 0)]


def test_disarm_retries_owed_cleanup_even_when_software_says_disarmed():
    bus = AbortBus(statusword=0x1637)
    link = _link(bus)
    link.arm(20, 100, 500, 30.0, False, False)
    bus.abort = lambda n, i, s, v: (n, i, v) == (2, 0x1016, 0)
    assert link.disarm() is False
    assert link.state == DISARMED and link.cleanup_owed and link.pc_guard_nodes == {2}
    assert any("1016h" in f for f in link.cleanup_failures)
    bus.abort = lambda *a: False
    bus.writes.clear()
    assert link.disarm() is True  # not an early return on DISARMED
    assert (2, 0x1016, 1, 0) in bus.writes and _disabled(bus) == {1, 2}
    assert not link.cleanup_owed and not link.pc_guard_set
    bus.writes.clear()
    assert link.disarm() is True and bus.writes == []  # nothing owed: now it is a no-op


def test_exit_zeroes_before_retiring_the_guard_and_keeps_the_measured_order():
    bus = AbortBus(statusword=0x1637)
    link = _link(bus)
    link.arm(20, 100, 500, 30.0, False, False)
    link.send_target(500, 500)
    bus.order.clear()
    link.disarm()
    seq = [(k, n, i) for k, n, i in bus.order if k != "hb"]
    first_guard = seq.index(("sdo", 1, 0x1016))
    assert seq[:first_guard] == [("rpdo", 1, None), ("rpdo", 2, None)]  # zero RPDO first
    first_60ff = seq.index(("sdo", 1, 0x60FF))
    assert seq[first_guard:first_60ff] == [("sdo", 1, 0x1016), ("sdo", 2, 0x1016)]  # then 1016h


def test_heartbeat_is_kept_alive_through_the_arm_and_disarm_transactions():
    bus = AbortBus(statusword=0x1637)
    link = _link(bus)
    link.pc_node, link.pc_heartbeat_s = 100, 0.0  # "due" before every SDO
    link.arm(20, 100, 500, 30.0, False, False)
    assert link.state == ARMED
    sdo = [k for k in bus.order if k[0] == "sdo"]
    assert sum(k[0] == "hb" for k in bus.order) >= len(sdo)
    guard_at = bus.order.index(("sdo", 1, 0x1016))
    assert ("hb", None, None) in bus.order[guard_at + 1 :]
    bus.order.clear()
    link.disarm()
    assert bus.order.count(("hb", None, None)) >= sum(k[0] == "sdo" for k in bus.order)
