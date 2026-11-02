"""RFID station-tag reader (Chafon CF821 "UHF Even Reader", EPC Gen2, TCP 2022).

A dedicated thread owns the socket. Nothing here is ever called from the 50 Hz
control tick - the tick reads snapshot(), exactly the way it already consumes
sensor_age_s from the MLS. A blocking socket call on the tick would blow the
20 ms budget, and that budget is already tight because of the 125 kbps CAN bus.

PROTOCOL (captured from this unit 2026-10-08, CRC-verified)
-----------------------------------------------------------
Vendor manual: "UHF Even Reader user manual-EN", appendix A. Every frame is

    CF | Addr | Cmd(2) | Len | [Status] | Data | CRC16 (high byte first)

Len counts Status+Data. Commands we send carry no Status; every response does.
CRC is CRC-16/MCRF4XX (poly 0x8408 reflected, init 0xFFFF) over CF..Data.
This unit answers from address 0x01; we send to the broadcast address 0xFF.

  0x0001  tag read, pushed by the reader in active mode:
          Status | RSSI(2, signed, 0.1 dBm) | Ant | Channel | EpcLen | EPC
          The manual says RSSI is dBm; the captured values (-720..-760) are 0.1 dBm.
          Status 0x12 is "inventory round done", not a tag.
  0x0050  "is the device online" - empty reply, Status 0. Used as the heartbeat.
  0x0070  device info: hardware/firmware strings of the CP board and RF module.
  0x0072  all basic parameters: work mode, interface, region, power, ...

The reader streams tags because it is in ACTIVE work mode (0x0072 WorkMode=1),
not because of anything we send. 0x0070 and 0x0072 are queries; this driver
sends no command that changes reader settings - that is done with the vendor
tool, on purpose. Never put two commands in one TCP segment: on 2026-10-08 that
left the reader silent (no replies, no tags) for the whole connection.

Our tags carry a 4-byte EPC; the tag id is the LAST tag_len bytes of the EPC
as uppercase hex ("0020"), the convention the route tables use.

Earlier versions split the stream into fixed 17-byte frames and took bytes
13:15. That only worked because every tag here has a 4-byte EPC; a 12-byte EPC
tag, or the 160-byte 0x0070 reply, misframed the stream (the "3130" that KIM2A
ignores is the ASCII "10" out of that reply). Length + CRC framing removes both.

WHY THE TCP OPTIONS BELOW ARE NOT OPTIONAL
------------------------------------------
The failure that will actually bite is the reader losing power, or the cable
parting mid-run. A stock socket reports ESTABLISHED indefinitely: recv() simply
never returns, no exception is raised. The keepalive block declares the peer
dead in ~3 s and TCP_USER_TIMEOUT bounds unacked data at 300 ms (~15 cm of
travel). They are set PER SOCKET - a sysctl would drag the wifi and USB-tether
sockets along with it.

Faster still: on a direct point-to-point link, /sys/class/net/<if>/carrier drops
within milliseconds of a cable parting, long before TCP can conclude anything.
That is why the link is deliberately built with no switch: insert one and a
yanked reader-side cable leaves our carrier up and lying to us.

What neither catches is a reader whose firmware wedges while its TCP stack
keeps answering keepalives. rfid.heartbeat_s > 0 sends 0x0050 periodically; once
the reader has answered one on a connection, an unanswered gap longer than
heartbeat_timeout_s drops and reconnects the link. A reader that never answers
is reported and left alone - it is not proof of a fault.

TWO FAULTS, NOT ONE
-------------------
  rfid_comms_lost   - the CONNECTION is gone. Reported here. Consumers map it
                      onto the existing sensor_lost path: we no longer know where
                      the route markers are, so hold straight and stop.
  rfid_tag_overdue  - comms fine, but a tag was MISSED (dirt, misalignment). Needs
                      an expected-tag-sequence route model and is NOT implemented
                      here; it is not a comms problem.

Silence is NOT a fault. The reader pushes only when a tag is in the field, so a
quiet link is the normal state of a vehicle between stations. Prolonged silence
raises a soft "silent" flag for the dashboard - it never stops the vehicle.
"""
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass

from agv_core import config, events

HEADER = 0xCF
BROADCAST = 0xFF

CMD_TAG = 0x0001
CMD_ONLINE = 0x0050
CMD_DEVICE_INFO = 0x0070
CMD_GET_PARAMS = 0x0072

# Gap between the connect-time queries when the first gets no reply. The reader
# must never see two commands in one TCP segment (see _session).
PARAMS_AFTER_S = 0.5

STATUS_OK = 0x00
STATUS_INVENTORY_DONE = 0x12

WORK_MODES = {0: "answer", 1: "active", 2: "trigger"}
INTERFACES = {0x80: "RS232", 0x40: "RS485", 0x20: "RJ45", 0x10: "WiFi"}
REGIONS = {0: "custom", 1: "US", 2: "Korea", 3: "EU", 4: "Japan", 5: "Malaysia",
           6: "EU3", 7: "China band 1", 8: "China band 2"}


def crc16(data):
    """CRC-16/MCRF4XX: reflected 0x1021 (0x8408), init 0xFFFF, no final xor."""
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
    return c


def command(cmd, data=b"", addr=BROADCAST):
    """A host->reader frame. Commands carry no Status byte."""
    body = bytes([HEADER, addr, cmd >> 8, cmd & 0xFF, len(data)]) + bytes(data)
    c = crc16(body)
    return body + bytes([c >> 8, c & 0xFF])


@dataclass(frozen=True)
class Frame:
    addr: int
    cmd: int
    status: int | None          # None only for an empty (Len 0) frame
    data: bytes                 # after Status


@dataclass(frozen=True)
class TagRead:
    tag: str                    # last tag_len EPC bytes, uppercase hex
    epc: str
    rssi_dbm: float
    antenna: int
    channel: int


class CfDecoder:
    """Len-delimited, CRC-checked 0xCF frames. Never raises, never blocks."""

    OVERHEAD = 7                # CF Addr Cmd(2) Len ... CRC(2)

    def __init__(self):
        self.bad_frames = 0     # CRC failures; each costs one byte of resync

    def decode(self, buf):
        """(frames, bytes_consumed). A partial frame is left for the next call."""
        frames, i, n = [], 0, len(buf)
        while True:
            start = buf.find(HEADER, i)
            if start < 0:
                return frames, n                    # no header left; drop junk
            if n - start < 5:
                return frames, start                # header seen, Len not yet
            total = self.OVERHEAD + buf[start + 4]
            if n - start < total:
                return frames, start                # partial frame, keep it
            raw = buf[start:start + total]
            if crc16(raw[:-2]) != (raw[-2] << 8 | raw[-1]):
                # A 0xCF inside some other frame's payload, or line noise.
                # Resync one byte on; a real frame further on is still found.
                self.bad_frames += 1
                i = start + 1
                continue
            body = raw[5:-2]
            frames.append(Frame(addr=raw[1], cmd=raw[2] << 8 | raw[3],
                                status=body[0] if body else None,
                                data=bytes(body[1:])))
            i = start + total


def parse_tag(frame, tag_len):
    """0x0001 Status 0 -> TagRead, else None (round-done, short EPC, junk)."""
    if frame.cmd != CMD_TAG or frame.status != STATUS_OK:
        return None
    d = frame.data
    if len(d) < 5:
        return None
    epc_len = d[4]
    epc = d[5:5 + epc_len]
    if len(epc) != epc_len or epc_len < tag_len:
        return None
    return TagRead(tag=epc[-tag_len:].hex().upper(),
                   epc=epc.hex().upper(),
                   rssi_dbm=int.from_bytes(d[0:2], "big", signed=True) / 10.0,
                   antenna=d[2],
                   channel=d[3])


def _field(blob):
    return blob.split(b"\0", 1)[0].decode("ascii", "replace").strip()


def parse_device_info(frame):
    """0x0070 -> "CP hw / CP fw / RF hw / RF fw", or None.

    Layout per the manual: HardVer(32) FirmVer(32) SN(12), once for the CP board
    and once for the RF module. The SN fields are binary on this unit and the RF
    hardware string overruns into the CP SN field, so only the strings that
    decode cleanly are joined.
    """
    if frame.cmd != CMD_DEVICE_INFO or frame.status != STATUS_OK:
        return None
    d = frame.data
    if len(d) < 140:
        return None
    parts = [_field(d[a:a + 32]) for a in (0, 32, 76, 108)]
    return " / ".join(p for p in parts if p) or None


def parse_params(frame):
    """0x0072 -> dict of the reader's basic parameters, or None."""
    if frame.cmd != CMD_GET_PARAMS or frame.status != STATUS_OK:
        return None
    d = frame.data
    if len(d) < 25:
        return None
    start_mhz = int.from_bytes(d[8:10], "big") + int.from_bytes(d[10:12], "big") / 1000.0
    step_khz = int.from_bytes(d[12:14], "big")
    channels = d[14]
    return {
        "addr": d[0],
        "rfid_protocol": d[1],
        "work_mode": d[2],
        "work_mode_name": WORK_MODES.get(d[2], f"0x{d[2]:02X}"),
        "interface": d[3],
        "interface_name": INTERFACES.get(d[3], f"0x{d[3]:02X}"),
        "baudrate_code": d[4],
        "wiegand": d[5],
        "antennas": d[6],
        "region": d[7],
        "region_name": REGIONS.get(d[7], f"0x{d[7]:02X}"),
        "start_mhz": start_mhz,
        "step_khz": step_khz,
        "channels": channels,
        "end_mhz": start_mhz + step_khz * channels / 1000.0,
        "power_dbm": d[15],
        "inquiry_area": d[16],
        "q_value": d[17],
        "session": d[18],
        "filter_time_s": d[21],
        "trigger_time_s": d[22],
        "buzzer_10ms": d[23],
        "polling_10ms": d[24],
    }


def params_summary(params):
    """One line for logs and the HMI: "active RS232 US 902.75-927.25 MHz 30 dBm"."""
    if not params:
        return ""
    return (f"{params['work_mode_name']} {params['interface_name']} {params['region_name']} "
            f"{params['start_mhz']:.2f}-{params['end_mhz']:.2f} MHz {params['power_dbm']} dBm")


def param_mismatches(params):
    """What the reader reports vs what the profile expects. Warnings, never stops."""
    out = []
    for key, want in (("work_mode", config.RFID_EXPECT_WORK_MODE),
                      ("region", config.RFID_EXPECT_REGION),
                      ("power_dbm", config.RFID_EXPECT_POWER_DBM)):
        have = params.get(key)
        if have != want:
            out.append(f"{key}: reader {have}, profile expects {want}")
    return out


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


class Heartbeat:
    """0x0050 liveness, clock-fed so it is testable without a socket.

    Arms on the first reply of a connection. Armed + no reply for timeout_s is a
    wedged reader. Never armed after timeout_s means the firmware does not answer
    0x0050 while streaming: reported once, then no more requests are sent.
    """

    def __init__(self, period_s, timeout_s):
        self.period_s = period_s
        self.timeout_s = timeout_s
        self.reset(None)

    def reset(self, now):
        self._connected_at = now
        self._sent_at = None
        self._reply_at = None
        self.armed = False
        self.unanswered = False

    @property
    def enabled(self):
        return self.period_s > 0

    def state(self):
        if not self.enabled:
            return "off"
        if self.armed:
            return "ok"
        return "unanswered" if self.unanswered else "waiting"

    def due(self, now):
        """True when a request should go out now."""
        if not self.enabled or self.unanswered:
            return False
        return self._sent_at is None or now - self._sent_at >= self.period_s

    def sent(self, now):
        self._sent_at = now

    def reply(self, now):
        self._reply_at = now
        self.armed = True

    def check(self, now):
        """'dead' when an armed heartbeat has gone quiet, 'gave_up' once when it
        never armed, else None."""
        if not self.enabled or self._connected_at is None:
            return None
        if self.armed:
            return "dead" if now - self._reply_at > self.timeout_s else None
        if not self.unanswered and now - self._connected_at > self.timeout_s:
            self.unanswered = True
            return "gave_up"
        return None


class PassStats:
    """Read statistics per tag pass, clock-fed so it is testable without a socket.

    The consumer needs one read per pass, so a pass that produced one read and a
    pass that produced none differ only by luck. Reads per pass and the RSSI spread
    are the read MARGIN that a "tag missed" report cannot show (2026-10-09).

    A pass of a tag ends once it has not been read for clear_s (rfid.tag_clear_s,
    the encounter rule). Passes of different tags overlap independently.
    """

    def __init__(self, clear_s, keep=64):
        self.clear_s = clear_s
        self._open = {}                     # tag -> [first, last, reads, sum, peak, low]
        self.closed = deque(maxlen=keep)    # (seq, summary dict), oldest first
        self.seq = 0

    def read(self, tag, rssi_dbm, now):
        p = self._open.get(tag)
        if p is not None and now - p[1] >= self.clear_s:
            self._close(tag)
            p = None
        if p is None:
            self._open[tag] = [now, now, 1, rssi_dbm, rssi_dbm, rssi_dbm]
            return
        p[1] = now
        p[2] += 1
        p[3] += rssi_dbm
        p[4] = max(p[4], rssi_dbm)
        p[5] = min(p[5], rssi_dbm)

    def expire(self, now, force=False):
        """Close the passes that ended by now (all of them with force: a reconnect)."""
        for tag in [t for t, p in self._open.items() if force or now - p[1] >= self.clear_s]:
            self._close(tag)

    def _close(self, tag):
        first, last, n, total, peak, low = self._open.pop(tag)
        self.seq += 1
        self.closed.append((self.seq, {
            "tag": tag, "reads": n, "duration_s": last - first,
            "rssi_peak": peak, "rssi_mean": total / n, "rssi_min": low}))


def pass_summary(p):
    """One log line per pass. tools/bag_report.py parses it: change both together."""
    return (f"RFID pass {p['tag']}: {p['reads']} reads in {p['duration_s']:.2f} s, "
            f"RSSI peak {p['rssi_peak']:.1f} mean {p['rssi_mean']:.1f} min {p['rssi_min']:.1f} dBm")


class RfidLink:
    """Owns the reader socket on its own thread. start() once, read snapshot()."""

    def __init__(self):
        self.decoder = CfDecoder()
        self.tag_len = config.RFID_TAG_LEN
        self.ignore = {t.upper() for t in config.RFID_IGNORE_TAGS}
        self.heartbeat = Heartbeat(config.RFID_HEARTBEAT_S, config.RFID_HEARTBEAT_TIMEOUT_S)
        self.passes = PassStats(config.RFID_TAG_CLEAR_S)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sock = None
        self._connected = False
        self._last_tag = None
        self._last_tag_at = None            # monotonic
        self._last_read = None              # TagRead of the latest tag frame
        self._last_rx = None                # monotonic, any valid frame
        self._tags_seen = 0
        self._identity = None
        self._params = None
        self._identity_seen = False         # this connection's 0x0070 reply is in
        self._mismatch = []
        self._detail = "disabled" if not config.RFID_ENABLED else "starting"
        # Encounter stream (gy-demo, restored 2026-10-02 for the line layer): one
        # numbered entry per DISTINCT pass of a tag, so a consumer can process each
        # exactly once and see a gap (seq) or a reconnect (generation).
        self._encounters = deque(maxlen=256)
        self._encounter_seq = 0
        self._encounter_tag = None
        self._encounter_at = None
        self._rebaseline = False
        self._generation = 0

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

    def snapshot(self, encounters=False):
        now = time.monotonic()
        with self._lock:
            tag_age = (now - self._last_tag_at) if self._last_tag_at else None
            rx_age = (now - self._last_rx) if self._last_rx else None
            # Hold the tag briefly so a 5 Hz UI poll cannot miss one that a
            # 10 Hz reader poll saw. Beyond the hold it is history, not state.
            live = (self._last_tag
                    if tag_age is not None and tag_age <= config.RFID_TAG_HOLD_S
                    else None)
            read = self._last_read
            params = dict(self._params) if self._params else None
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
                "epc": read.epc if read else None,
                "rssi_dbm": read.rssi_dbm if read else None,
                "antenna": read.antenna if read else None,
                "channel": read.channel if read else None,
                "freq_mhz": (params["start_mhz"] + read.channel * params["step_khz"] / 1000.0
                             if read and params else None),
                "identity": self._identity,
                "reader": params,
                "config_mismatch": list(self._mismatch),
                "heartbeat": self.heartbeat.state(),
                "bad_frames": self.decoder.bad_frames,
                "detail": self._detail,
                "encounter_seq": self._encounter_seq,
                "generation": self._generation,
                "encounters": list(self._encounters) if encounters else [],
                "passes": list(self.passes.closed) if encounters else [],
            }

    def _comms_ok(self):
        """Health is CONNECTION state, not data flow.

        Deliberately not "have we heard bytes recently": between stations there
        are no tags and therefore no bytes, and treating that as a fault would
        stop the vehicle on every straight. The socket options make connection
        state meaningful - a dead peer errors the socket within ~3 s, and carrier
        drops in milliseconds if the cable parts. An armed heartbeat that goes
        quiet drops the connection, so it reaches this flag the same way.
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
        recv_timeout = config.RFID_RECV_TIMEOUT_S
        if self.heartbeat.enabled:
            recv_timeout = min(recv_timeout, self.heartbeat.period_s)
        sock.settimeout(recv_timeout)
        self._sock = sock

        now = time.monotonic()
        self.heartbeat.reset(now)
        self._identity_seen = False
        with self._lock:
            self._connected = True
            self._last_rx = now
            self._detail = "connected"
        events.info(f"RFID connected {config.RFID_IP}:{config.RFID_PORT}")

        # Read-only queries, ONE COMMAND PER TCP SEGMENT. Both in a single
        # segment (2026-10-08) left the reader silent for the whole connection -
        # no replies, no tag pushes. 0x0070 alone is what every working
        # connection has sent; 0x0072 follows once its reply is in, or after
        # PARAMS_AFTER_S. TCP_NODELAY keeps separate sendall()s separate.
        # Replies arrive interleaved with tag frames; _absorb() sorts them.
        sock.sendall(command(CMD_DEVICE_INFO))
        params_due = now + PARAMS_AFTER_S

        buf = b""
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                self.passes.expire(now)
            if params_due is not None and (now >= params_due or self._identity_seen):
                sock.sendall(command(CMD_GET_PARAMS))
                params_due = None
            if self.heartbeat.due(now):
                sock.sendall(command(CMD_ONLINE))
                self.heartbeat.sent(now)
            verdict = self.heartbeat.check(now)
            if verdict == "dead":
                raise TimeoutError(f"reader stopped answering 0x0050 for "
                                   f">{self.heartbeat.timeout_s:g} s")
            if verdict == "gave_up":
                events.warn("RFID reader does not answer the 0x0050 heartbeat; "
                            "relying on TCP keepalive and carrier only")
            try:
                chunk = sock.recv(4096)
            except TimeoutError:
                continue                    # no tag in the field: normal
            if not chunk:
                raise ConnectionResetError("reader closed the connection")

            buf += chunk
            frames, used = self.decoder.decode(buf)
            buf = buf[used:]
            if len(buf) > 8192:             # never grow without bound on junk
                buf = b""
            self._absorb(frames)

    def _absorb(self, frames):
        if not frames:
            return
        now = time.monotonic()
        reads = []
        for f in frames:
            if f.cmd == CMD_TAG:
                r = parse_tag(f, self.tag_len)
                if r and r.tag not in self.ignore:
                    reads.append(r)
            elif f.cmd == CMD_ONLINE and f.status == STATUS_OK:
                self.heartbeat.reply(now)
            elif f.cmd == CMD_DEVICE_INFO:
                self._on_identity(parse_device_info(f))
            elif f.cmd == CMD_GET_PARAMS:
                self._on_params(parse_params(f))
        with self._lock:
            self._last_rx = now
            if not reads:
                return
            for r in reads:
                self.passes.read(r.tag, r.rssi_dbm, now)
                # A reconnect first establishes a baseline. It is not evidence
                # of departure, and must not synthesize a second station visit.
                if self._rebaseline:
                    self._rebaseline = False
                elif (r.tag != self._encounter_tag
                      or self._encounter_at is None
                      or now - self._encounter_at >= config.RFID_TAG_CLEAR_S):
                    self._encounter_seq += 1
                    self._encounters.append((self._encounter_seq, r.tag))
                self._encounter_tag, self._encounter_at = r.tag, now
            last = reads[-1]
            new = last.tag != self._last_tag
            self._last_tag = last.tag
            self._last_read = last
            self._last_tag_at = now
            self._tags_seen += len(reads)
        # Edge-triggered: a tag sitting in the field re-reads many times a
        # second and would otherwise flush the event ring in seconds.
        if new:
            events.info(f"RFID tag {last.tag} ({last.rssi_dbm:.1f} dBm)")

    def _on_identity(self, ident):
        if not ident:
            return
        self._identity_seen = True
        with self._lock:
            changed = ident != self._identity
            self._identity = ident
        if changed:
            events.info(f"RFID reader: {ident}")

    def _on_params(self, params):
        if not params:
            return
        mismatch = param_mismatches(params)
        with self._lock:
            self._params = params
            self._mismatch = mismatch
        events.info(f"RFID reader config: {params_summary(params)}")
        for m in mismatch:
            events.warn(f"RFID reader config: {m}")

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
            self._generation += 1
            self._encounters.clear()
            self.passes.expire(time.monotonic(), force=True)
            self._rebaseline = True
        if was:
            events.warn("RFID link lost")

    def _set_detail(self, text):
        with self._lock:
            self._detail = text
