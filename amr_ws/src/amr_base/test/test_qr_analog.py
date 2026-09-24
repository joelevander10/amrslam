"""CKDA08ETH analog link: register layout, one write for adjacent channels, failure handling."""

from amr_base.qr_analog import AnalogOut


class Resp:
    def __init__(self, err=False):
        self.err = err

    def isError(self):
        return self.err


class FakeClient:
    def __init__(self, log, fail=False, up=True):
        self.log, self.fail, self.up = log, fail, up

    def connect(self):
        return self.up

    def write_registers(self, addr, values, device_id=0):
        self.log.append(("regs", addr, list(values)))
        return Resp(self.fail)

    def write_register(self, addr, value, device_id=0):
        self.log.append(("reg", addr, value))
        return Resp(self.fail)

    def close(self):
        self.log.append(("close",))


def make(log, t, ch=(1, 0), **kw):
    return AnalogOut(
        "10.0.0.1",
        502,
        1,
        0.05,
        64,
        1000.0,
        10.0,
        ch[0],
        ch[1],
        client_factory=lambda: FakeClient(log, **kw),
        clock=lambda: t[0],
    )


def test_adjacent_channels_are_one_transaction_in_register_order():
    log, t = [], [0.0]
    ao = make(log, t)  # left ch1, right ch0 (the QR wiring)
    assert ao.write(1.5, 0.25)
    assert log == [("regs", 64, [250, 1500])]  # register 64 = ch0 = right
    assert ao.volts == (1.5, 0.25)


def test_clamped_unchanged_values_are_not_resent_until_refresh():
    log, t = [], [0.0]
    ao = make(log, t)
    ao.write(12.0, -1.0)
    assert log[-1] == ("regs", 64, [0, 10000])
    t[0] = 0.1
    ao.write(12.0, -1.0)
    assert len(log) == 1
    t[0] = 0.7
    ao.write(12.0, -1.0)
    assert len(log) == 2


def test_separate_channels_and_failure_backoff():
    log, t = [], [0.0]
    ao = make(log, t, ch=(1, 5), fail=True)
    assert not ao.write(1.0, 1.0)
    assert not ao.ok and ao.errors == 1 and ("close",) in log
    n = len(log)
    t[0] = 0.2
    assert not ao.write(1.0, 1.0)  # inside reconnect_s: no new attempt
    assert len(log) == n
    t[0] = 0.6
    ao.write(1.0, 1.0)
    assert ("reg", 65, 1000) in log


def test_alive_through_the_skipped_writes_of_an_unchanged_value():
    # the AMR QR on 2026-09-24: armed at 0 V, the unchanged value is only re-sent every
    # 0.5 s, and a 0.3 s freshness check on the last write faulted the drives each start
    log, t = [], [0.0]
    ao = make(log, t)
    assert not ao.alive(0.0, 0.3)  # nothing written yet
    assert ao.write(0.0, 0.0)
    for step in range(1, 40):  # 2 s of 50 Hz ticks at the same value
        t[0] = step * 0.05
        assert ao.write(0.0, 0.0)
        assert ao.alive(t[0], 0.3), t[0]
    assert len([e for e in log if e[0] == "regs"]) < 10  # still skipping, not writing each tick
    # a failed write is dead at once, not after the grace
    log2, t2 = [], [0.0]
    bad = make(log2, t2, fail=True)
    assert not bad.write(1.0, 1.0) and not bad.alive(0.0, 0.3)
    # and a link that stopped being written goes dead after refresh + grace
    assert not ao.alive(t[0] + ao.refresh_s + 0.31, 0.3)
