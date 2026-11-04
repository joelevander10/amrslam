"""GLS621 link (gls-wait-coding-plan §B.1, P2): the thread that owns the camera's socket.

Offline and deterministic. GlsLink._run is driven on the test's own thread with a fake socket,
a fake clock (advanced only by the fake socket's recv timeout) and a fake stop event whose
wait() records the backoff instead of sleeping. Only the last check starts the real thread,
against a socket that answers nothing, to prove stop() ends it.
"""
import collections
import threading
import time

from helpers import check

from agv_core.drivers import gls
from agv_core.drivers.gls_link import KEEPALIVE_VAR, REQUIRED, RESEND_S, GlsLink, value_matches

TEMPLATE = "\\x02[iec _GoodReadCond][f][$BC];[$XMCL];[$YMCL];[$AN2P];[$FAMS][e][e]\\x03"
# what the camera reads back: strings carry their length in hex first
GOOD = {"OFtplstr1": "49 " + TEMPLATE, "GlsCodeSel": "0", "DCDMVerISO15415": "0"}
EXPECT = {"OFtplstr1": TEMPLATE, **REQUIRED}
# every telegram any fake socket was ever sent, across all the tests below
SENT_ALL = []


def tel(p):
    return bytes([gls.STX]) + p + bytes([gls.ETX])


def result(pos, x, y, ts, label=b"x0000y0000"):
    return pos + label + b";%d;%d;0.5;%d" % (x, y, ts)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeStop:
    """threading.Event's interface for _run, without sleeping: wait() records the delay."""

    def __init__(self):
        self.flag, self.waits = False, []

    def is_set(self):
        return self.flag

    def set(self):
        self.flag = True

    def wait(self, d):
        self.waits.append(d)
        return self.flag


class FakeSock:
    """Answers each `sRN name` from `values` (a name missing from it, or answer=False, is
    silence); `script` is [(t, bytes)] pushed when the clock reaches t; `at` is {t: fn(sock)}.
    recv() returns queued bytes, else advances the clock by dt and times out. At `end` it
    stops the link (end_how="stop"), closes from the peer side ("eof") or fails ("oserror")."""

    def __init__(self, clock, link, values=None, script=(), at=None, end=30.0, end_how="stop", dt=0.01):
        self.clock, self.link, self.dt = clock, link, dt
        self.values = dict(GOOD if values is None else values)
        self.script = sorted(script, key=lambda s: s[0])
        self.at = dict(at or {})
        self.end, self.end_how = end, end_how
        self.answer = True
        self.rx = collections.deque()
        self.sent = []  # (t, bytes)
        self.closed = False
        self.timeout = None

    def settimeout(self, t):
        self.timeout = t

    def sendall(self, b):
        self.sent.append((self.clock.t, b))
        SENT_ALL.append(b)
        name = b[1:-1].decode("ascii").split(" ", 1)[1]
        if self.answer and name in self.values:
            self.rx.append(tel(f"sRA {name} {self.values[name]}".encode("ascii")))

    def names_sent(self, name):
        return [t for t, b in self.sent if b == gls.command(f"sRN {name}")]

    def recv(self, n):
        while self.script and self.script[0][0] <= self.clock.t:
            self.rx.append(self.script.pop(0)[1])
        for t in sorted(k for k in self.at if k <= self.clock.t):
            self.at.pop(t)(self)
        if self.rx:
            return self.rx.popleft()
        if self.clock.t >= self.end:
            if self.end_how == "eof":
                return b""
            if self.end_how == "oserror":
                raise ConnectionResetError("reset by peer")
            self.link._stop.set()
        self.clock.t += self.dt
        raise TimeoutError

    def close(self):
        self.closed = True


def make_link(socks, expect=EXPECT, connects=None, **kw):
    """A link whose factory hands out `socks` (each a FakeSock builder, or an OSError to
    raise) in order; when they run out it stops the link and refuses the connection."""
    clock = Clock()
    kw.setdefault("keepalive_s", 2.0)
    kw.setdefault("verify_timeout_s", 3.0)
    kw.setdefault("frame_gap_s", 0.1)
    kw.setdefault("backoff_s", (0.5, 5.0))
    link = GlsLink("192.0.2.1", 2112, expect, clock=clock, **kw)
    link._stop = FakeStop()
    made, queue = [], list(socks)

    def factory(addr, timeout):
        if connects is not None:
            connects.append((addr, timeout))
        if not queue:
            link._stop.set()
            raise ConnectionRefusedError("no more fake sockets")
        item = queue.pop(0)
        if isinstance(item, BaseException):
            raise item
        s = item(clock, link)
        made.append(s)
        return s

    link._factory = factory
    return link, clock, made


def frames_script(t0, labels=(b"x0000y0000",), n=3, codes=3, period=0.07, ts0=1000):
    """n images, `codes` codes each, one telegram per code, starting at t0."""
    out = []
    for i in range(n):
        for c in range(codes):
            pos = b"ABCD"[c % 4:c % 4 + 1] + b"1"
            label = labels[i % len(labels)]
            out.append((t0 + i * period, tel(result(pos, 10 + c, -5 + c, ts0 + 70 * i, label))))
    return out


def test_value_matches():
    print("\ngls link: value_matches")
    check("a number reads back bare", value_matches("0", "0") and not value_matches("1", "0"))
    check("a string's hex length prefix is accepted",
          value_matches("49 " + TEMPLATE, TEMPLATE) and value_matches(TEMPLATE, TEMPLATE))
    check("a different string is a mismatch, prefix or not",
          not value_matches("49 " + TEMPLATE.replace("[$XMCL]", "[$XCL]"), TEMPLATE)
          and not value_matches("1E", "0"))


def test_verify_success_delivers_frames():
    print("\ngls link: verify, then frames")
    connects = []
    script = frames_script(0.5, n=3)
    # the same image split across two reads, and two results in ONE telegram
    joined = tel(result(b"A1", 20, 0, 5000) + result(b"B1", 22, 2, 5000))
    split = tel(result(b"C1", 24, 4, 5000))
    script += [(1.0, joined), (1.0, split[:7]), (1.01, split[7:])]
    link, clock, made = make_link(
        [lambda c, ln: FakeSock(c, ln, script=script, end=2.0)], connects=connects, connect_timeout_s=1.5)
    link._run()
    s = made[0]
    snap = link.snapshot()
    check("the socket is opened at host:port with the connect timeout",
          connects[0] == (("192.0.2.1", 2112), 1.5), str(connects))
    check("at connect every expected variable is read once, before anything else",
          sorted(b for _, b in s.sent[:3]) == sorted(gls.command(f"sRN {n}") for n in EXPECT)
          and all(t == 0.0 for t, _ in s.sent[:3]), str(s.sent[:3]))
    check("the readback verifies (string length prefix accepted)",
          snap["config_ok"] is True and snap["values"] == GOOD, f"{snap['config_ok']} {snap['detail']}")
    frames = link.pop_frames()
    fs = [f for _, f in frames]
    check("every image is delivered once: three closed by the next timestamp or the gap, one more",
          [f.ts_ms for f in fs] == [1000, 1070, 1140, 5000], str([f.ts_ms for f in fs]))
    check("a frame holds the codes of its image", fs[0].n_codes == 3 and fs[0].valid
          and fs[0].label_id == "x0000y0000", str(fs[0]))
    check("two results in one telegram and a telegram split across reads aggregate into one image",
          fs[3].n_codes == 3 and fs[3].x == 22 and fs[3].positions == ("A1", "B1", "C1"), str(fs[3]))
    check("frames carry their receive time", all(0.5 <= t <= 2.0 for t, _ in frames),
          str([t for t, _ in frames]))
    check("pop_frames drains", link.pop_frames() == [])
    check("counters: frames counted, none dropped, the label remembered",
          snap["frames"] == 4 and snap["dropped"] == 0 and snap["last_label"] == "x0000y0000"
          and snap["connects"] == 1, str(snap))
    check("after stop the link is not connected and says why",
          snap["connected"] is False and snap["detail"] == "stopped" and s.closed, snap["detail"])


def test_mismatch_drops_frames():
    print("\ngls link: a mismatch drops every frame")
    bad = dict(GOOD, GlsCodeSel="1")
    seen = {}
    link, clock, made = make_link([lambda c, ln: FakeSock(
        c, ln, values=bad, script=frames_script(0.5, n=4), end=2.0,
        at={1.5: lambda s: seen.update(link.snapshot())})])
    link._run()
    check("a value that differs fails the verification",
          seen.get("config_ok") is False and seen.get("connected") is True, str(seen.get("config_ok")))
    check("the detail names the variable, what it read and what was expected",
          "GlsCodeSel" in seen.get("detail", "") and "'1'" in seen["detail"] and "'0'" in seen["detail"]
          and "OFtplstr1" not in seen["detail"], seen.get("detail", ""))
    snap = link.snapshot()
    check("no frame is delivered, every one is counted as dropped",
          link.pop_frames() == [] and snap["frames"] == 0 and snap["dropped"] == 4, str(snap))
    check("the camera still answers: comms_ok was true during the mismatch", seen.get("comms_ok") is True)

    # a factory-reset camera: the as-found template without the label-centre outputs
    as_found = dict(GOOD, OFtplstr1="40 \\x02[iec _GoodReadCond][f][$BC];[$XCL];[$YCL][e][e]\\x03")
    seen = {}
    link, _, _ = make_link([lambda c, ln: FakeSock(c, ln, values=as_found, script=frames_script(0.5), end=1.5,
                                                   at={1.4: lambda s: seen.update(link.snapshot())})])
    link._run()
    check("a different template is a mismatch naming OFtplstr1",
          seen["config_ok"] is False and "OFtplstr1" in seen["detail"] and link.snapshot()["dropped"] == 3,
          seen["detail"])


def test_frames_before_verify_are_dropped():
    print("\ngls link: nothing is believed before the readback")
    # the camera streams results from the first byte, before any reply has come back
    link, _, made = make_link([lambda c, ln: FakeSock(
        c, ln, values={}, script=frames_script(0.0, n=3, period=0.05), end=0.6,
        at={0.4: lambda s: (setattr(s, "values", dict(GOOD)), [s.sendall(gls.command(f"sRN {n}"))
                                                              for n in EXPECT])})])
    link._run()
    snap = link.snapshot()
    check("frames completed while settings are unverified are dropped, then verification succeeds",
          snap["dropped"] == 3 and snap["frames"] == 0 and snap["config_ok"] is True, str(snap))


def test_no_reply_within_verify_timeout():
    print("\ngls link: verify timeout")
    seen = {}
    link, _, made = make_link([lambda c, ln: FakeSock(
        c, ln, values={"GlsCodeSel": "0"}, script=frames_script(3.5), end=5.0,
        at={4.5: lambda s: seen.update(link.snapshot())})])
    link._run()
    s = made[0]
    check("missing replies fail the verification after verify_timeout_s",
          seen.get("config_ok") is False and "no reply" in seen.get("detail", "")
          and "OFtplstr1" in seen["detail"] and "DCDMVerISO15415" in seen["detail"]
          and "GlsCodeSel" not in seen["detail"], seen.get("detail", ""))
    resends = s.names_sent("OFtplstr1")
    check("a missing read is resent every RESEND_S until the timeout, then no more",
          3 <= len(resends) <= 4
          and all(abs(b - a - RESEND_S) < 0.05 for a, b in zip(resends, resends[1:], strict=False))
          and resends[-1] <= 3.0 + 0.02, str(resends))
    check("the answered variable is not resent by the verify loop",
          [t for t in s.names_sent("GlsCodeSel") if t < 1.9] == [0.0], str(s.names_sent("GlsCodeSel")))
    check("frames are dropped after a verify timeout too",
          link.snapshot()["frames"] == 0 and link.snapshot()["dropped"] == 3)

    # an error reply to a read (sFA) is a reply, but not a value
    def sfa(clock, ln):
        s = FakeSock(clock, ln, values={k: v for k, v in GOOD.items() if k != "DCDMVerISO15415"}, end=4.0,
                     at={3.9: lambda s: seen.update(sfa=link.snapshot())})
        orig = s.sendall

        def sendall(b):
            orig(b)
            if b == gls.command("sRN DCDMVerISO15415"):
                s.rx.append(tel(b"sFA 5"))
        s.sendall = sendall
        return s
    link, _, _ = make_link([sfa])
    link._run()
    snap = seen["sfa"]
    check("an error reply never verifies a variable",
          snap["config_ok"] is False and "no reply to DCDMVerISO15415" == snap["detail"], snap["detail"])


def test_keepalive_and_comms():
    print("\ngls link: keepalive and health")
    seen = {}

    def mute(s):
        s.answer = False

    link, clock, made = make_link([lambda c, ln: FakeSock(
        c, ln, end=60.0, at={
            5.0: lambda s: seen.update(ok=link.snapshot()),
            9.0: mute,  # the last answered keepalive is the one at 8 s
            13.5: lambda s: seen.update(late=link.snapshot()),
            14.5: lambda s: seen.update(lost=link.snapshot()),
        })])
    ended = []
    orig = link._session

    def session(sock):
        why = orig(sock)
        ended.append((clock.t, why))
        return why
    link._session = session
    link._run()
    s = made[0]
    keeps = [t for t in s.names_sent(KEEPALIVE_VAR) if t > 0.0]
    check("a keepalive read is sent every keepalive_s",
          len(keeps) >= 7 and all(abs(b - a - 2.0) < 0.05 for a, b in zip(keeps, keeps[1:], strict=False))
          and abs(keeps[0] - 2.0) < 0.05, str(keeps))
    check("after the connect readback, the keepalive is the only thing sent",
          all(b == gls.command(f"sRN {KEEPALIVE_VAR}") for t, b in s.sent if t > 0.0))
    ok = seen["ok"]
    check("answering: comms_ok, a fresh reply age", ok["comms_ok"] is True and ok["reply_age_s"] < 2.1,
          str(ok["reply_age_s"]))
    late = seen["late"]
    check("silent for under 3 x keepalive_s: still comms_ok", late["comms_ok"] is True,
          str(late["reply_age_s"]))
    lost = seen["lost"]
    check("silent for over 3 x keepalive_s: comms_ok false, still connected",
          lost["comms_ok"] is False and lost["connected"] is True and lost["reply_age_s"] > 6.0,
          str(lost["reply_age_s"]))
    last_answered = max(t for t in keeps if t < 9.0)
    check("a link silent for 3 x keepalive_s + verify_timeout_s is closed",
          ended and abs(ended[0][0] - (last_answered + 6.0 + 3.0)) < 0.05
          and ended[0][1].startswith("no reply for")
          and s.closed, str(ended))


def test_reconnect_and_backoff():
    print("\ngls link: reconnect")

    def silent_after(t):
        return lambda c, ln: FakeSock(c, ln, at={t: lambda s: setattr(s, "answer", False)}, end=1e9)

    link, clock, made = make_link([silent_after(3.0), lambda c, ln: FakeSock(c, ln, end=clock.t + 1.0)])
    link._run()
    snap = link.snapshot()
    check("a silent link is reopened: a second connect, verified again",
          snap["connects"] == 2 and made[0].closed and made[1].closed
          and len(made[1].names_sent("OFtplstr1")) == 1 and snap["config_ok"] is True, str(snap))
    check("a reconnect after a session waits the minimum backoff",
          link._stop.waits[0] == 0.5, str(link._stop.waits))

    # peer close
    link, clock, made = make_link([
        lambda c, ln: FakeSock(c, ln, end=1.0, end_how="eof"),
        lambda c, ln: FakeSock(c, ln, end=clock.t + 0.5)])
    seen = []
    orig = link._session

    def session(sock):
        why = orig(sock)
        seen.append((why, link.snapshot()))
        return why
    link._session = session
    link._run()
    check("the camera closing the connection ends the session and it reconnects",
          seen[0][0] == "the camera closed the connection" and link.snapshot()["connects"] == 2
          and made[0].closed, seen[0][0])

    # a socket error mid-session
    link, clock, made = make_link([lambda c, ln: FakeSock(c, ln, end=1.0, end_how="oserror")])
    seen = []
    orig = link._session
    link._session = lambda sock: seen.append(orig(sock)) or seen[-1]
    link._run()
    check("a socket error ends the session with its reason, and the socket is closed",
          seen == ["link error: reset by peer"] and made[0].closed, str(seen))

    # connect failures back off, doubling to the maximum, and reset after a connect
    errs = [OSError("network unreachable")] * 6
    link, clock, made = make_link(errs + [lambda c, ln: FakeSock(c, ln, end=clock.t + 0.5, end_how="eof"),
                                          OSError("again")])
    seen = []
    orig_factory = link._factory

    def factory(addr, timeout):
        try:
            return orig_factory(addr, timeout)
        except OSError:
            seen.append(link.snapshot())
            raise
    link._factory = factory
    link._run()
    w = link._stop.waits
    check("a refused connect is reported, not connected",
          seen and seen[0]["connected"] is False
          and seen[1]["detail"].startswith("cannot connect to 192.0.2.1:2112")
          and "network unreachable" in seen[1]["detail"], seen[1]["detail"] if len(seen) > 1 else "")
    check("connect failures back off 0.5, 1, 2, 4 then hold at the 5 s maximum",
          w[:6] == [0.5, 1.0, 2.0, 4.0, 5.0, 5.0], str(w))
    check("a successful connect resets the backoff to the minimum",
          w[6:8] == [0.5, 0.5], str(w))


def test_counters():
    print("\ngls link: junk, unknown, fps")
    script = [(0.5, b"\r\nxx"), (0.5, tel(b"hello")), (0.5, tel(b"A1x0000y0000;1;2;garbled"))]
    script += frames_script(1.0, n=11, codes=2, period=0.1, ts0=2000)
    link, clock, _ = make_link([lambda c, ln: FakeSock(c, ln, script=script, end=3.0)])
    link._run()
    snap = link.snapshot()
    check("bytes outside telegrams are junk, unparseable telegrams unknown",
          snap["junk"] == 4 and snap["unknown"] == 2, f"{snap['junk']} {snap['unknown']}")
    check("fps is measured from the frame receive times",
          abs(snap["fps"] - 10.0) < 0.3 and snap["frames"] == 11, f"{snap['fps']} {snap['frames']}")
    check("rx and frame ages are relative to the clock",
          snap["frame_age_s"] is not None and 0.9 < snap["frame_age_s"] < 2.0 and snap["rx_age_s"] < 2.1,
          f"{snap['frame_age_s']} {snap['rx_age_s']}")
    many = frames_script(0.5, n=80, codes=1, period=0.02)
    link, _, _ = make_link([lambda c, ln: FakeSock(c, ln, script=many, end=3.0)])
    link._run()
    check("an undrained queue keeps only the last 64 frames",
          len(link.pop_frames()) == 64 and link.snapshot()["frames"] == 80)


def test_stop_ends_the_thread():
    print("\ngls link: the real thread")

    class Silent:
        def settimeout(self, t):
            pass

        def sendall(self, b):
            SENT_ALL.append(b)

        def recv(self, n):
            time.sleep(0.005)
            raise TimeoutError

        def close(self):
            pass

    link = GlsLink("192.0.2.1", 2112, EXPECT, socket_factory=lambda a, t: Silent(), backoff_s=(0.01, 0.02))
    link.start()
    deadline = time.monotonic() + 1.0
    while link.snapshot()["connects"] < 1 and time.monotonic() < deadline:
        time.sleep(0.005)
    t0 = time.monotonic()
    link.stop(join_s=1.0)
    check("stop() ends the thread promptly",
          link._thread is not None and not link._thread.is_alive() and time.monotonic() - t0 < 0.5,
          f"{time.monotonic() - t0:.3f} s")
    check("the thread is named and a daemon", link._thread.name == "gls" and link._thread.daemon)
    check("no thread named gls is left behind",
          not any(t.name == "gls" and t.is_alive() for t in threading.enumerate()))


def test_only_reads_are_ever_sent():
    print("\ngls link: read-only")
    allowed = {gls.command(f"sRN {n}") for n in [*EXPECT, KEEPALIVE_VAR]}
    check("every telegram sent by every link above is an sRN of an expected variable",
          len(SENT_ALL) > 50 and set(SENT_ALL) <= allowed,
          str(sorted(set(SENT_ALL) - allowed)[:3]))


TESTS = [
    test_value_matches,
    test_verify_success_delivers_frames,
    test_mismatch_drops_frames,
    test_frames_before_verify_are_dropped,
    test_no_reply_within_verify_timeout,
    test_keepalive_and_comms,
    test_reconnect_and_backoff,
    test_counters,
    test_stop_ends_the_thread,
    test_only_reads_are_ever_sent,
]


if __name__ == "__main__":
    for fn in TESTS:
        fn()
