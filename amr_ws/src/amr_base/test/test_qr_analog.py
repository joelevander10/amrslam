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
