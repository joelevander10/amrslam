"""RFID station-tag reader (Chafon CF821, UHF EPC Gen2, TCP 2022).

A dedicated thread owns the socket. Nothing here is ever called from the 50 Hz
control tick - the tick reads snapshot(), exactly the way it already consumes
sensor_age_s from the MLS. A blocking socket call on the tick would blow the
20 ms budget, and that budget is already tight because of the 125 kbps CAN bus.

WHY THE TCP OPTIONS BELOW ARE NOT OPTIONAL
------------------------------------------
The failure that will actually bite is the reader losing power, or the cable
parting mid-run. A stock socket reports ESTABLISHED indefinitely: recv() simply
never returns, no exception is raised, and at 0.5 m/s the vehicle covers several
kilometres still believing it is connected. The keepalive block declares the peer
dead in ~3 s and TCP_USER_TIMEOUT bounds unacked data at 300 ms (~15 cm of
travel). They are set PER SOCKET - a sysctl would drag the wifi and USB-tether
sockets along with it.

Faster still: on a direct point-to-point link, /sys/class/net/<if>/carrier drops
within milliseconds of a cable parting, long before TCP can conclude anything.
That is why the link is deliberately built with no switch (manuals/rfid-setup):
insert one and a yanked reader-side cable leaves our carrier up and lying to us.

TWO FAULTS, NOT ONE
-------------------
  rfid_comms_lost   - the CONNECTION is gone. Reported here. Consumers map it
                      onto the existing sensor_lost path: we no longer know where
                      the route markers are, so hold straight and stop.
                      Connection state is only a trustworthy health signal
                      BECAUSE of the socket options above. KIM2A has no keepalive
                      and calls _record_rx() on every recv timeout, so on that
                      vehicle a dead-but-connected reader stays "healthy" forever
                      and only raises a dashboard note after 60 s.
  rfid_tag_overdue  - comms fine, but a tag was MISSED (dirt, misalignment). Needs
                      an expected-tag-sequence route model and is NOT implemented
                      here; it is not a comms problem.

Silence is NOT a fault. The reader pushes only when a tag is in the field, so a
quiet link is the normal state of a vehicle between stations. Prolonged silence
raises a soft "silent" flag for the dashboard (antenna knocked, reader wedged) -
it never stops the vehicle.

PROTOCOL PROVENANCE
-------------------
This is the SAME READER HARDWARE as the KIM2A vehicle, on the same TCP port, so
the framing below is not inferred - it is lifted from a system that has been
reading tags in production (agv-kim2a-controller/drivers/rfid_tcp.py, and the
init command from its profiles/*.json).

  init      CF FF 00 70 00 24 15   sent once per connection
  frame     17 bytes, header 0xCF, fixed length, no checksum
  tag       bytes [13:15] -> 4 hex chars
  ignore    "3130" - see below; kept only as a backstop

The init has a REPLY: a ~160 byte banner naming the firmware, e.g.
"CP-203910_V1.13 / UHF Even Reader V1.0 / RN370MU-910". KIM2A never reads it, so
their 17-byte splitter chews the banner's first frame and extracts bytes 13:15 -
the ASCII "10" out of "CP-2039_10_" - which is exactly the mysterious "3130" they
hardcode as an ignored startup artifact. Consuming the banner deliberately, and
recording it as the device identity, removes the need for that magic entirely.

*** The init command is why the reader appears silent. *** It does not answer
unsolicited queries; it streams continuously once started. Do not add polling.
"""
import os
import socket
import threading
import time

import config
import events


class ChafonCFCodec:
    """Fixed-length 0xCF frames. Push-only: the reader streams after the init.

    KIM2A parses this by splitting the hex STRING on "CF", which loses any frame
    whose payload happens to contain 0xCF - their own comment calls out "stray CF
    in payload" and their length check then discards it. Scanning bytes for the
    header and taking a fixed span instead keeps those frames, and cannot
    mis-split.
    """

    HEADER = 0xCF

    def __init__(self, init=b"", frame_len=17, tag_offset=13, tag_len=2,
                 ignore=()):
        self._init = init
        self.frame_len = frame_len
        self.tag_offset = tag_offset
        self.tag_len = tag_len
        self.ignore = {t.upper() for t in ignore}

    def handshake(self):
        """Sent once on connect. Without it the reader never says anything."""
        return self._init or None

    def poll(self):
        return None                     # push-only; there is nothing to ask for

    def decode(self, buf):
        """(frames, bytes_consumed). Never raises, never blocks."""
        frames, i, n = [], 0, len(buf)
        while True:
            start = buf.find(self.HEADER, i)
            if start < 0:
                return frames, n                    # no header left; drop junk
            if n - start < self.frame_len:
                return frames, start                # partial frame, keep it
            frames.append(buf[start:start + self.frame_len])
            i = start + self.frame_len

    def tag_of(self, frame):
        """Frame -> tag id as uppercase hex, or None if it is not a real tag."""
        end = self.tag_offset + self.tag_len
        if len(frame) < end:
            return None
        tag = frame[self.tag_offset:end].hex().upper()
        return None if tag in self.ignore else tag


def _hardened(sock):
    """Bound how long a dead peer can masquerade as a live one."""
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for name, value in (("TCP_KEEPIDLE", 1), ("TCP_KEEPINTVL", 1),
                        ("TCP_KEEPCNT", 2), ("TCP_USER_TIMEOUT", 300)):
        opt = getattr(socket, name, None)
        if opt is not None:                 # Linux-only; skip elsewhere
            sock.setsockopt(socket.IPPROTO_TCP, opt, value)
    return sock


class RfidLink:
    """Owns the reader socket on its own thread. start() once, read snapshot()."""

    def __init__(self, codec=None):
        self.codec = codec or ChafonCFCodec(
            init=bytes.fromhex(config.RFID_INIT_HEX or ""),
            frame_len=config.RFID_FRAME_LEN,
            tag_offset=config.RFID_TAG_OFFSET,
            tag_len=config.RFID_TAG_LEN,
            ignore=config.RFID_IGNORE_TAGS)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sock = None
        self._connected = False
        self._last_tag = None
        self._last_tag_at = None            # monotonic
        self._last_rx = None                # monotonic, any bytes
        self._tags_seen = 0
        self._identity = None
        self._detail = "disabled" if not config.RFID_ENABLED else "starting"

    # ---- public ----------------------------------------------------------

    def start(self):
        if not config.RFID_ENABLED or self._thread:
            return
        self._thread = threading.Thread(target=self._run, name="rfid",
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        sock, self._sock = self._sock, None
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def carrier(self):
        """True/False from the NIC, or None when the interface is unknown.

        On a direct link this is the fastest possible proof the reader is
        physically present - milliseconds, versus seconds for any TCP mechanism.
        """
        path = f"/sys/class/net/{config.RFID_INTERFACE}/carrier"
        try:
            with open(path) as fh:
                return fh.read().strip() == "1"
        except OSError:
            return None                     # no such interface; not a fault

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            tag_age = (now - self._last_tag_at) if self._last_tag_at else None
            rx_age = (now - self._last_rx) if self._last_rx else None
            # Hold the tag briefly so a 5 Hz UI poll cannot miss one that a
            # 10 Hz reader poll saw. Beyond the hold it is history, not state.
            live = (self._last_tag
                    if tag_age is not None and tag_age <= config.RFID_TAG_HOLD_S
                    else None)
            return {
                "enabled": config.RFID_ENABLED,
                "connected": self._connected,
                "carrier": self.carrier(),
                "comms_ok": self._comms_ok(),
                # Soft flag only. Never stops the vehicle.
                "silent": bool(self._connected and rx_age is not None
                               and rx_age > config.RFID_SILENT_WARN_S),
                "tag": live,
                "last_tag": self._last_tag,
                "tag_age_s": tag_age,
                "rx_age_s": rx_age,
                "tags_seen": self._tags_seen,
                "identity": self._identity,
                "detail": self._detail,
            }

    def _comms_ok(self):
        """Health is CONNECTION state, not data flow.

        Deliberately not "have we heard bytes recently": between stations there
        are no tags and therefore no bytes, and treating that as a fault would
        stop the vehicle on every straight. The socket options make connection
        state meaningful - a dead peer errors the socket within ~3 s, and carrier
        drops in milliseconds if the cable parts.
        """
        if not config.RFID_ENABLED:
            return None                     # not a fault; simply not in use
        if self.carrier() is False:
            return False
        return self._connected

    # ---- thread ----------------------------------------------------------

    def _run(self):
        while not self._stop.is_set():
            try:
                self._session()
            except Exception as e:          # noqa: BLE001 - thread must survive
                self._set_detail(f"{type(e).__name__}: {e}")
            self._drop()
            if self._stop.wait(config.RFID_RECONNECT_PERIOD_S):
                return

    def _session(self):
        if self.carrier() is False:
            self._set_detail("no carrier - cable or reader power")
            return

        sock = _hardened(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
        # Bound the connect. An unreachable host would otherwise hold this
        # thread for the OS TCP timeout, 30-120 s.
        sock.settimeout(2.0)
        sock.connect((config.RFID_IP, config.RFID_PORT))
        sock.settimeout(config.RFID_RECV_TIMEOUT_S)
        self._sock = sock

        now = time.monotonic()
        with self._lock:
            self._connected = True
            self._last_rx = now
            self._detail = "connected"
        events.info(f"RFID connected {config.RFID_IP}:{config.RFID_PORT}")

        hello = self.codec.handshake()
        if hello:
            sock.sendall(hello)
            self._read_banner(sock)

        buf = b""
        while not self._stop.is_set():
            # No polling. The reader streams after the init command; asking it
            # for anything is neither necessary nor supported.
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                # Normal: no tag in the antenna field. Not a fault, and the
                # recv timeout is only here so the loop can notice _stop.
                continue
            if not chunk:
                raise ConnectionResetError("reader closed the connection")

            buf += chunk
            frames, used = self.codec.decode(buf)
            buf = buf[used:]
            if len(buf) > 8192:             # never grow without bound on junk
                buf = b""
            self._absorb(frames)

    @staticmethod
    def _printable(blob):
        """Longest printable runs in the banner, joined - the firmware strings."""
        runs, cur = [], []
        for b in blob:
            if 32 <= b < 127:
                cur.append(chr(b))
            else:
                if len(cur) >= 6:
                    runs.append("".join(cur))
                cur = []
        if len(cur) >= 6:
            runs.append("".join(cur))
        return " / ".join(runs)

    def _read_banner(self, sock):
        """Consume the init reply so it is never mistaken for tag data.

        This is the whole reason KIM2A needs its "3130" special case. Reading it
        also gets us the reader's firmware identity for free, which is worth
        having on the dashboard when a unit is swapped.
        """
        blob = b""
        sock.settimeout(config.RFID_BANNER_WAIT_S)
        try:
            while len(blob) < 4096:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                blob += chunk
        except socket.timeout:
            pass
        finally:
            sock.settimeout(config.RFID_RECV_TIMEOUT_S)
        if not blob:
            return
        ident = self._printable(blob)
        with self._lock:
            self._identity = ident
            self._last_rx = time.monotonic()
        if ident:
            events.info(f"RFID reader: {ident}")

    def _absorb(self, frames):
        now = time.monotonic()
        tags = [t for t in (self.codec.tag_of(f) for f in frames) if t]
        with self._lock:
            self._last_rx = now
            if not tags:
                return
            tag = tags[-1]
            new = tag != self._last_tag
            self._last_tag = tag
            self._last_tag_at = now
            self._tags_seen += 1
        # Edge-triggered: a tag sitting in the field re-reads many times a
        # second and would otherwise flush the event ring in seconds.
        if new:
            events.info(f"RFID tag {tag}")

    def _drop(self):
        was = self._connected
        sock, self._sock = self._sock, None
        if sock:
            try:
                sock.close()
            except OSError:
                pass
        with self._lock:
            self._connected = False
        if was:
            events.warn("RFID link lost")

    def _set_detail(self, text):
        with self._lock:
            self._detail = text
