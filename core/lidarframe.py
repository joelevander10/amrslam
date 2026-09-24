"""Decode the SICK nanoScan3 UDP data telegram. Bytes in, dicts out.

*** THIS IS NOT A SAFETY FUNCTION. *** The stop comes from the scanner's OSSD
pair, hardwired into the FX3, which drops the drives' STO. The manual is explicit
that the Ethernet data "must not be used for safety-related applications".
Everything decoded here is for navigation, display and logging.

PURE COMPUTATION
----------------
No sockets, no config, no threads - the same stance core/panel.py and
core/branch.py take, and for the same reason: the whole decode is then testable
from a captured telegram with no scanner, no network and no privileges. The one
stateful thing here is Reassembler, which is fed a clock rather than reading one.

WHY THIS IS HAND-ROLLED, WHEN THE BRIEF SAYS NOT TO
---------------------------------------------------
lidar_brief.md sec 2.3 points at sick_safetyscanners_base / sick_safetyscanners2.
Both open a TCP CoLa 2 session to port 2122 and WRITE the UDP output
configuration to the device on startup - which sec 0 rule 2 of that same brief
forbids in as many words ("Do not change the scanner's configuration. Not the IP,
not the fields, not the data output settings."). They are also C++/ROS 2, in a
repo that is standard library plus Flask, python-can and pymodbus.

The scanner is already configured to stream continuously to this PC, so none of
that machinery is needed: 172 datagram/s arrive with nothing bound to the port.
A passive listener is simpler AND strictly more compliant.

What sec 2.3 is really warning against is guessing at byte offsets, and that
warning is honoured here rather than ignored:

  * every offset below is DERIVED from the telegram's own block table, never
    hardcoded - which is also what sec 2.4 demands of the angular range;
  * every field is either backed by the evidence recorded against it, or left
    unnamed in a `raw` dict. A field graduates out of `raw` when a human has
    confirmed it against SICK document 8022706 ("microScan3, outdoorScan3,
    nanoScan3: Data output via UDP and TCP/IP"), which is not in this repo.
    `drivers/lidar_scan.py raw` exists to make that check a five-minute job.

WHAT WAS CONFIRMED ON THE WIRE, AND HOW
---------------------------------------
Measured against the live scanner at 192.168.3.10, and against the captured
telegram in tests/fixtures/:

  header      24 bytes, marker b"MS3 ", protocol 0x444D, version 1.0. The
              telegram is fragmented by the SCANNER, not by IP: one 6812-byte
              telegram arrives as 5 datagrams of 1460/1460/1460/1460/1092.
  identification
              increments by exactly +1 per telegram over 103 consecutive
              samples, so it is a telegram counter and gives sec 3.5 sequence-gap
              detection with no dependence on document 8022706.
  block table 7 (offset, size) u16 pairs at telegram +32. Offsets are relative
              to telegram[4:]; proof is that the last block, (6744, 64), then
              ends at exactly 6812 - the telegram length - and the measurement
              block tiles the space before it with no gap.
  start angle int32 at derived+4 reads -199229440, and -47.5 deg * 2**22 is
              -199229440 exactly. The brief records the configured range as
              -47.5 deg. That is not a coincidence, so angles are int32 in units
              of 2**-22 degrees.
  resolution  int32 at derived+8 reads 699050 = 0.16667 deg, against the 0.17 deg
              the brief records.
  point size  the measurement block is 6608 bytes with NO count prefix - its
              first four bytes are already a plausible point (79 mm, rssi 121)
              and 6608/4 = 1652 divides exactly.
  status bit0 1651 of the 1652 points carry it; the single point that does not
              has distance 0. So bit 0 is validity - and the datasheet's "1651
              measured values" is the count of VALID beams, while the wire
              carries 1652 slots.

THE COUNT COMES FROM THE BLOCK SIZE, NEVER FROM A CONSTANT
----------------------------------------------------------
This is the trap sec 2.4 sets, and the numbers above are exactly it: build
against the datasheet's 1651 and every scan is silently one point short, with the
error landing at the end of the sector where nobody looks. There is deliberately
no 1651 anywhere in this file.
"""
import struct

MARKER = b"MS3 "

# marker, protocol, major, minor, total_length, identification,
# fragment_offset, reserved.
_HEADER = struct.Struct("<4sHBBIIII")
HEADER_LEN = _HEADER.size                       # 24

_TABLE_AT = 32                                  # block table, within the telegram
_TABLE_ENTRIES = 7
_BLOCK_BASE = 4                                 # block offsets are from telegram[4:]
_ENTRY = struct.Struct("<HH")
_POINT = struct.Struct("<HBB")                  # distance_mm, rssi, status
POINT_LEN = _POINT.size

# Block table slots. Named by what the wire shows them to CARRY, not by the
# names in a document nobody here has read - slot 2 is called measurement
# because it decodes as 1652 plausible points, and that is the whole claim.
BLK_STATUS = 0          # 16 B present; contents NOT yet validated - see zones()
BLK_DERIVED = 1         # 24 B; start angle and resolution confirmed above
BLK_MEASUREMENT = 2     # 6608 B = 1652 points
BLK_INTRUSION = 3       # absent - "Object detection" is off, per the brief
BLK_APPLICATION = 4     # absent - "Application Data" is off, per the brief
BLK_LOCAL_IO = 5        # 64 B; contents not yet validated
BLK_SPARE = 6           # absent

# Angles are int32 in units of 2**-22 degrees. See the header block above.
ANGLE_LSB = 1.0 / 4194304.0

STATUS_VALID = 0x01     # confirmed; every other bit stays unnamed
NO_ECHO_MM = 40000      # the 40 m measurement range, reported for "no echo"


def parse_datagram(data):
    """One UDP datagram -> its header fields and payload, or None if foreign.

    Returns None rather than raising for anything that is not a telegram
    fragment: the socket is bound to a port, not to a promise, and a stray
    broadcast must not take the reader's thread down.
    """
    if len(data) < HEADER_LEN:
        return None
    marker, proto, major, minor, total, ident, offset, _res = \
        _HEADER.unpack_from(data, 0)
    if marker != MARKER:
        return None
    return {"protocol": proto, "version": (major, minor), "total_length": total,
            "identification": ident, "fragment_offset": offset,
            "payload": data[HEADER_LEN:]}


def block_table(tel):
    """The telegram's own directory: slot -> (offset, size), or None if absent.

    An absent block is (0, 0) on the wire. Returning None for it matters: read
    literally, (0, 0) is a zero-length block AT offset 0, and code that treats
    the two the same will happily decode the telegram header as an empty
    intrusion block instead of noticing the feature is switched off.
    """
    if len(tel) < _TABLE_AT + _TABLE_ENTRIES * _ENTRY.size:
        return [None] * _TABLE_ENTRIES
    out = []
    for i in range(_TABLE_ENTRIES):
        offset, size = _ENTRY.unpack_from(tel, _TABLE_AT + i * _ENTRY.size)
        start = offset + _BLOCK_BASE
        # A block whose extent runs past the telegram is a decode error, not a
        # short read to be papered over with a slice that silently truncates.
        if size == 0 or start + size > len(tel):
            out.append(None)
        else:
            out.append((start, size))
    return out


def block(tel, table, slot):
    """Bytes of one block, or None when it is absent."""
    ext = table[slot] if slot < len(table) else None
    if ext is None:
        return None
    start, size = ext
    return tel[start:start + size]


def derived_values(tel, table):
    """Angular geometry, taken from the telegram rather than from the datasheet.

    sec 2.4: "Angular span matches the Configuration of Data Output block, not
    what you assumed. The manual warns data may be output from a slightly larger
    angle range than configured - take the range from the telegram, never
    hardcode it." That is what this is for.
    """
    b = block(tel, table, BLK_DERIVED)
    if b is None or len(b) < 24:
        return None
    f0, start, res, f3, f4, f5 = struct.unpack_from("<IiiIII", b, 0)
    return {
        "start_angle_deg": start * ANGLE_LSB,
        "resolution_deg": res * ANGLE_LSB,
        # f0 reads 30, matching the 30 ms scan cycle the brief records, but one
        # matching integer is a coincidence until 8022706 says otherwise.
        "raw": {"f0": f0, "f3": f3, "f4": f4, "f5": f5},
    }


def beam_count(table):
    """How many points this telegram carries. From the block size. Always."""
    ext = table[BLK_MEASUREMENT] if len(table) > BLK_MEASUREMENT else None
    return 0 if ext is None else ext[1] // POINT_LEN


def measurement(tel, table, step=1):
    """The point array: distances in mm, RSSI, and the raw status byte.

    Parallel lists rather than a list of dicts - 1652 dicts per telegram at 34 Hz
    is 56k allocations a second to render a picture.

    `step` decimates for the browser. It samples rather than averaging: a
    minimum would bias every rendered scan toward the nearest return and make
    the picture look worse than the scan, and a mean would invent surfaces
    between two real ones.
    """
    b = block(tel, table, BLK_MEASUREMENT)
    if b is None:
        return None
    step = max(1, int(step))
    dist, rssi, status = [], [], []
    for d, r, s in _POINT.iter_unpack(b[:len(b) - len(b) % POINT_LEN]):
        dist.append(d)
        rssi.append(r)
        status.append(s)
    if step > 1:
        dist, rssi, status = dist[::step], rssi[::step], status[::step]
    return {"dist_mm": dist, "rssi": rssi, "status": status, "step": step}


def valid_mask(status, dist):
    """Which points are worth drawing: bit 0 set, and an actual echo.

    NO_ECHO_MM is the top of the measurement range and is what the scanner
    reports when nothing came back. Plotting those puts a solid 40 m arc across
    the picture where the room is simply empty.
    """
    return [bool(s & STATUS_VALID) and 0 < d < NO_ECHO_MM
            for s, d in zip(status, dist)]


def zones(tel, table, spec):
    """Cut-off path states - PROVISIONAL, and deliberately hard to trust.

    sec 2.5 is blunt: "Do not assume the ordering - confirm it." The status
    block decodes to 16 bytes whose layout is not in this repo, so this reads
    whatever bytes the profile names and reports `validated` alongside, straight
    from the profile's own flag.

    Until a human has walked an object inward through all three rings and set
    lidar.zones_validated true, every caller must treat the result as unknown.
    The UI does: unvalidated renders as its own state, never as "clear". That is
    the sec 4 rule applied to our own ignorance rather than to a dead cable.

    spec: (block_slot, [byte offsets], active_low, validated)
    """
    slot, offsets, active_low, validated = spec
    b = block(tel, table, slot)
    out = {"validated": bool(validated), "paths": [], "readable": b is not None}
    if b is None:
        return out
    for off in offsets:
        if not 0 <= off < len(b):
            out["paths"].append(None)            # unreadable: never "clear"
            continue
        raw = b[off]
        out["paths"].append(bool(raw) if active_low else not raw)
    out["raw"] = list(b)
    return out


def decode(tel, zone_spec, step=1, points=True):
    """Whole telegram -> the dict the rest of the system reads."""
    table = block_table(tel)
    out = {
        "length": len(tel),
        "blocks": [None if e is None else {"offset": e[0], "size": e[1]}
                   for e in table],
        "beams": beam_count(table),
        "derived": derived_values(tel, table),
        "zones": zones(tel, table, zone_spec),
        # Named nothing, carried anyway: this is what a human diffs against
        # document 8022706 to promote a field.
        "raw": {"status_block": list(block(tel, table, BLK_STATUS) or b""),
                "local_io_block": list(block(tel, table, BLK_LOCAL_IO) or b"")},
    }
    if points:
        out["measurement"] = measurement(tel, table, step)
    return out


class Reassembler:
    """Fragments in, whole telegrams out.

    The scanner fragments at the application layer - five datagrams carrying one
    6812-byte telegram, each stamped with its offset - so UDP hands us five
    independent, individually droppable, potentially reordered pieces.

    Keyed by identification, so a fragment of the NEXT telegram arriving while
    this one is still short does not corrupt it. A partial telegram older than
    `timeout_s` is dropped and counted rather than kept: holding it would let a
    fragment lost at 34 Hz pair up with a same-offset fragment from a telegram
    30 ms later and emit a scan stitched out of two different instants.
    """

    def __init__(self, timeout_s=0.2):
        self.timeout_s = timeout_s
        self.dropped = 0            # partial telegrams abandoned
        self.foreign = 0            # datagrams that were not telegram fragments
        self._parts = {}            # identification -> {offset: payload}
        self._first = {}            # identification -> monotonic time
        self._total = {}            # identification -> expected length

    def push(self, data, now):
        """Feed one datagram. Returns (identification, telegram) or None."""
        head = parse_datagram(data)
        if head is None:
            self.foreign += 1
            return None

        ident = head["identification"]
        self._expire(now, keep=ident)
        parts = self._parts.setdefault(ident, {})
        if not parts:
            self._first[ident] = now
            self._total[ident] = head["total_length"]
        parts[head["fragment_offset"]] = head["payload"]

        if sum(len(p) for p in parts.values()) < self._total[ident]:
            return None

        tel = b"".join(parts[o] for o in sorted(parts))
        self._forget(ident)
        # Trim rather than trust: a duplicated fragment would otherwise hand the
        # decoder a telegram longer than its own header claims.
        return ident, tel[:head["total_length"]]

    def _expire(self, now, keep=None):
        for ident, t in list(self._first.items()):
            if ident != keep and now - t > self.timeout_s:
                self.dropped += 1
                self._forget(ident)

    def _forget(self, ident):
        self._parts.pop(ident, None)
        self._first.pop(ident, None)
        self._total.pop(ident, None)
