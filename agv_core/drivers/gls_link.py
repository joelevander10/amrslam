"""The GLS621 camera's TCP link, on its own thread (gls-wait-coding-plan §B.1).

One socket, to the results port (2112): CoLa A command replies are answered on it too, mixed
into the result stream, and agv_core.drivers.gls tells them apart. The diagnostics port (2111)
is never opened: its per-code dump buries replies (bench, 2026-10-30).

READ-ONLY. The only thing ever sent is `sRN` (read variable). Configuration lives in the
device (SOPAS, saved permanently); this link checks it, never writes it:

  * At every connect it reads back the settings the integration relies on (`expect`). Until
    they have all answered as expected, frames are COUNTED AND DROPPED, never delivered: a
    camera reset to factory settings outputs each code's own position, 27 mm apart, and that
    must not become a position. A mismatch stays a mismatch until the next connect.
  * It reads one of them again every keepalive_s. The camera sends nothing on a no-read, so
    with no label in view silence is normal - the keepalive is what proves the camera alive.

HEALTH IS DATA FLOW, as for the DIO island: comms_ok means a reply arrived within
3 x keepalive_s, whatever the socket believes. A link that stops answering for longer than
that is closed and reopened with a backoff.

Free of config and ROS: the node builds it from the profile, the tests from a fake socket.
"""

from __future__ import annotations

import collections
import socket
import threading
import time

from agv_core.drivers import gls

# What the integration relies on besides the template (the template comes from the profile,
# gls.expect_template): the multi-code label application, and no print grading per image.
REQUIRED = {"GlsCodeSel": "0", "DCDMVerISO15415": "0"}
KEEPALIVE_VAR = "GlsCodeSel"
RESEND_S = 1.0  # a verify read that has not answered is sent again this often
FPS_WINDOW = 30  # frames over which the rate is measured


def value_matches(raw: str, want: str) -> bool:
    """A CoLa A sRA value against the expected one. String variables come with their length
    in hex first ("49 \\x02[iec ..."), numbers without ("0", "1E")."""
    return raw == want or raw.split(" ", 1)[-1] == want


class GlsLink:
    """Owns the GLS socket on its own thread. start() once; read snapshot(), drain pop_frames()."""

    def __init__(
        self,
        host: str,
        port: int,
        expect: dict[str, str],
        *,
        connect_timeout_s: float = 2.0,
        keepalive_s: float = 2.0,
        verify_timeout_s: float = 3.0,
        frame_gap_s: float = 0.1,
        min_codes: int = 1,
        outlier_counts: float | None = None,
        backoff_s: tuple[float, float] = (0.5, 5.0),
        socket_factory=None,
        clock=time.monotonic,
    ):
        self.host, self.port = host, int(port)
        self.expect = dict(expect)
        self.connect_timeout_s, self.keepalive_s = connect_timeout_s, keepalive_s
        self.verify_timeout_s, self.frame_gap_s = verify_timeout_s, frame_gap_s
        self.min_codes, self.outlier_counts = min_codes, outlier_counts
        self.backoff_s = backoff_s
        self._factory = socket_factory or self._default_socket
        self._clock = clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frames: collections.deque = collections.deque(maxlen=64)
        self._frame_ts: collections.deque = collections.deque(maxlen=FPS_WINDOW)
        # health and counters, under _lock
        self._connected = False
        self._config_ok: bool | None = None  # None until every expected value has answered
        self._detail = "starting"
        self._last_reply: float | None = None
        self._last_rx: float | None = None
        self._last_frame: float | None = None
        self._last_label = ""
        self._n_frames = self._n_dropped = self._n_unknown = self._n_junk = self._n_connects = 0
        self._values: dict[str, str] = {}

    # ---- public ------------------------------------------------------------------------

    @staticmethod
    def _default_socket(addr, timeout):
        return socket.create_connection(addr, timeout=timeout)

    def start(self) -> None:
        if self._thread:
            return
        self._thread = threading.Thread(target=self._run, name="gls", daemon=True)
        self._thread.start()

    def stop(self, join_s: float = 1.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(join_s)

    def pop_frames(self) -> list[tuple[float, gls.Frame]]:
        """Frames completed since the last call, with their local receive time. Only frames from
        a camera whose settings verified; never more than the last 64."""
        with self._lock:
            out = list(self._frames)
            self._frames.clear()
            return out

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            reply_age = None if self._last_reply is None else now - self._last_reply

            def age(t):
                return None if t is None else now - t

            ts = list(self._frame_ts)
            fps = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0
            return {
                "connected": self._connected,
                "comms_ok": self._connected and reply_age is not None and reply_age <= 3 * self.keepalive_s,
                "config_ok": self._config_ok,
                "detail": self._detail,
                "reply_age_s": reply_age,
                "rx_age_s": age(self._last_rx),
                "frame_age_s": age(self._last_frame),
                "fps": fps,
                "last_label": self._last_label,
                "values": dict(self._values),
                "frames": self._n_frames,
                "dropped": self._n_dropped,
                "unknown": self._n_unknown,
                "junk": self._n_junk,
                "connects": self._n_connects,
            }

    # ---- the thread --------------------------------------------------------------------

    def _set(self, **kw) -> None:
        with self._lock:
            for k, v in kw.items():
                setattr(self, "_" + k, v)

    def _run(self) -> None:
        delay = self.backoff_s[0]
        while not self._stop.is_set():
            try:
                sock = self._factory((self.host, self.port), self.connect_timeout_s)
            except OSError as e:
                self._set(connected=False, detail=f"cannot connect to {self.host}:{self.port}: {e}")
                self._stop.wait(delay)
                delay = min(self.backoff_s[1], delay * 2)
                continue
            delay = self.backoff_s[0]
            with self._lock:
                self._connected, self._config_ok, self._values = True, None, {}
                self._last_reply = None
                self._n_connects += 1
                self._detail = "connected: verifying settings"
            why = self._session(sock)
            try:
                sock.close()
            except OSError:
                pass
            self._set(connected=False, detail=why)
            if not self._stop.is_set():
                self._stop.wait(delay)

    def _send(self, sock, name: str) -> None:
        sock.sendall(gls.command(f"sRN {name}"))

    def _session(self, sock) -> str:
        """One connection, until it fails or stop(). Returns why it ended."""
        sock.settimeout(0.02)
        agg = gls.FrameAggregator(self.min_codes, outlier_xy=self.outlier_counts)
        start = self._clock()
        sent = keep = start
        buf = b""
        try:
            for name in self.expect:
                self._send(sock, name)
            while not self._stop.is_set():
                now = self._clock()
                with self._lock:
                    values, verdict, last_reply = dict(self._values), self._config_ok, self._last_reply
                if verdict is None:
                    missing = [n for n in self.expect if n not in values]
                    if not missing:
                        self._judge(values)
                    elif now - start > self.verify_timeout_s:
                        self._set(config_ok=False, detail=f"no reply to {', '.join(missing)}")
                    elif now - sent >= RESEND_S:
                        for name in missing:
                            self._send(sock, name)
                        sent = now
                if now - keep >= self.keepalive_s:
                    self._send(sock, KEEPALIVE_VAR)
                    keep = now
                quiet_since = last_reply if last_reply is not None else start
                if now - quiet_since > 3 * self.keepalive_s + self.verify_timeout_s:
                    return f"no reply for {now - quiet_since:.1f} s: reconnecting"
                try:
                    data = sock.recv(4096)
                except TimeoutError:
                    data = None
                if data == b"":
                    return "the camera closed the connection"
                if data:
                    buf = self._feed(buf + data, agg, now)
                f = agg.flush(self._clock(), self.frame_gap_s)
                if f:
                    self._deliver(f, self._clock())
        except OSError as e:
            return f"link error: {e}"
        return "stopped"

    def _feed(self, buf: bytes, agg: gls.FrameAggregator, now: float) -> bytes:
        tels, rest, junk = gls.split(buf)
        with self._lock:
            self._n_junk += junk
            if tels:
                self._last_rx = now
        for p in tels:
            r = gls.parse(p)
            if isinstance(r, list):
                for read in r:
                    f = agg.add(read, now)
                    if f:
                        self._deliver(f, now)
            elif isinstance(r, gls.Reply):
                with self._lock:
                    self._last_reply = now
                    if r.kind == "sRA" and r.name in self.expect:
                        self._values[r.name] = r.value
            else:
                with self._lock:
                    self._n_unknown += 1
        return rest

    def _judge(self, values: dict[str, str]) -> None:
        bad = [n for n, want in self.expect.items() if not value_matches(values[n], want)]
        if bad:
            got = "; ".join(f"{n} = {values[n]!r}, expected {self.expect[n]!r}" for n in bad)
            self._set(config_ok=False, detail=f"settings differ: {got}")
        else:
            self._set(config_ok=True, detail="connected, settings verified")

    def _deliver(self, f: gls.Frame, now: float) -> None:
        with self._lock:
            if self._config_ok is not True:
                self._n_dropped += 1
                return
            self._frames.append((now, f))
            self._frame_ts.append(now)
            self._n_frames += 1
            self._last_frame = now
            if f.valid:
                self._last_label = f.label_id
