"""Header internet indicator: the robot's own reachability, probed at most once per ttl."""

import pytest
from amr_web.server import create_app

from amr_web import netcheck


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Sock:
    def close(self):
        pass


def test_first_reachable_target_wins_and_no_dns_is_involved():
    tried = []

    def connect(addr, timeout):
        tried.append(addr)
        if addr[0] == "1.1.1.1":
            raise OSError("unreachable")
        return Sock()

    p = netcheck.InternetProbe(connect=connect, clock=Clock())
    r = p.read()
    assert r["online"] is True and r["via"] == "8.8.8.8"
    assert tried == [("1.1.1.1", 443), ("8.8.8.8", 443)]  # IP literals: no resolver needed


def test_offline_when_nothing_answers_and_the_result_is_cached_for_the_ttl():
    calls = []
    clk = Clock()

    def connect(addr, timeout):
        calls.append(addr)
        raise OSError("no route")

    p = netcheck.InternetProbe(connect=connect, clock=clk, ttl_s=4.0)
    assert p.read()["online"] is False and len(calls) == 2
    clk.t += 3.9
    assert p.read()["online"] is False and len(calls) == 2  # cached: 0.25 Hz at most
    clk.t += 0.2
    p.read()
    assert len(calls) == 4


def test_a_caller_during_a_running_probe_gets_the_last_result_not_a_wait():
    p = netcheck.InternetProbe(connect=lambda a, timeout: Sock(), clock=Clock())
    assert p._lock.acquire()
    try:
        assert p.read()["online"] is None  # nothing known yet, and no blocking
    finally:
        p._lock.release()


def test_api_internet_serves_the_probe(tmp_path):
    from test_jog import Stub  # noqa: PLC0415

    probe = netcheck.InternetProbe(connect=lambda a, timeout: Sock(), clock=Clock())
    c = create_app(Stub(), str(tmp_path), internet_probe=probe).test_client()
    r = c.get("/api/internet")
    assert r.status_code == 200 and r.json["online"] is True and r.json["via"] == "1.1.1.1"
    assert b'id="net"' in c.get("/manual").data


def test_wifi_reader_without_the_field_network_reads_no_link(tmp_path):
    from amr_web import wifi  # noqa: PLC0415

    proc = tmp_path / "wireless"
    proc.write_text("Inter-| sta-|   Quality        |\n face | tus | link level noise |\n")
    r = wifi.WifiReader("wlp1s0", proc_path=str(proc)).read()
    assert r == {"iface": "wlp1s0", "connected": False, "ssid": None, "dbm": None, "bars": 0}
    assert pytest.approx(0) == r["bars"]
