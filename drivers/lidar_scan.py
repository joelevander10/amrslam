#!/usr/bin/env python3
"""Read the nanoScan3 UDP stream. Read-only - the scanner is never written to.

    lidar_scan.py             one telegram, decoded and summarised
    lidar_scan.py watch       live rate, gaps and zone flags; prints on CHANGE
    lidar_scan.py raw         hex dump of the header and every block
    lidar_scan.py capture F   save one telegram's datagrams to F, for the tests

`raw` is the important one. lidar_brief.md sec 2.3 says not to guess at byte
offsets and to work from SICK document 8022706 ("microScan3, outdoorScan3,
nanoScan3: Data output via UDP and TCP/IP"), which is not in this repo. This mode
lays the telegram out block by block, with offsets and sizes taken from the
telegram's OWN directory, so confirming a field against that document is reading
two things side by side rather than writing a script.

Nothing here opens a CoLa 2 session, and nothing here can change the scanner's
configuration - which is rule 2 of the brief. It binds a socket and listens.
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "core"))
import config  # noqa: E402
import lidarframe as lf  # noqa: E402

OK, BAD, WARN = "\033[32mOK\033[0m", "\033[31mFAIL\033[0m", "\033[33mWARN\033[0m"

BLOCK_NAMES = {
    lf.BLK_STATUS: "status (contents NOT validated)",
    lf.BLK_DERIVED: "derived values (start angle + resolution confirmed)",
    lf.BLK_MEASUREMENT: "measurement data",
    lf.BLK_INTRUSION: "object detection",
    lf.BLK_APPLICATION: "application data",
    lf.BLK_LOCAL_IO: "local I/O (contents NOT validated)",
    lf.BLK_SPARE: "spare",
}


def open_socket():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(3.0)
    s.bind((config.LIDAR_HOST_IP, config.LIDAR_PORT))
    return s


def one_telegram(sock, keep_datagrams=False):
    """-> (identification, telegram, [datagrams]). Raises on timeout."""
    asm = lf.Reassembler(config.LIDAR_REASSEMBLY_TIMEOUT_S)
    seen, seen_ident = [], None
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        data, addr = sock.recvfrom(65535)
        if addr[0] != config.LIDAR_SENSOR_IP:
            continue
        if keep_datagrams:
            head = lf.parse_datagram(data)
            # Restart the collection whenever a new telegram begins, so the
            # saved set is the fragments of ONE scan and not a mixture of two.
            if head and head["identification"] != seen_ident:
                seen, seen_ident = [], head["identification"]
            seen.append(data)
        done = asm.push(data, time.monotonic())
        if done is not None:
            return done[0], done[1], seen
    raise TimeoutError(f"no complete telegram from {config.LIDAR_SENSOR_IP} "
                       f"in 5 s (dropped {asm.dropped}, foreign {asm.foreign})")


def zone_spec():
    return (config.LIDAR_ZONE_BLOCK, config.LIDAR_ZONE_BYTES,
            config.LIDAR_ZONE_ACTIVE_LOW, config.LIDAR_ZONES_VALIDATED)


def show_zones(z):
    if not z.get("validated"):
        print(f"    {WARN} cut-off paths are UNVALIDATED - the byte mapping in "
              f"the profile is a guess.")
        print( "         Walk an object inward through warn -> slow -> stop "
               "(brief sec 2.5), confirm")
        print( "         the order, then set lidar.zones_validated true. Until "
               "then nothing may")
        print( "         treat these as 'clear'.")
    for name, state in zip(("stop", "slow", "warn"), z.get("paths") or []):
        mark = "?" if not z.get("validated") else ("OCCUPIED" if state else "clear")
        print(f"      path {name:<5} raw={state}   {mark}")


def scan(sock):
    ident, tel, _ = one_telegram(sock)
    dec = lf.decode(tel, zone_spec(), step=1)
    d = dec["derived"] or {}
    m = dec["measurement"] or {}
    n = dec["beams"]

    print(f"    telegram {ident}, {len(tel)} bytes, {n} beams\n")
    print( "    blocks (offset and size from the telegram's own table):")
    for i, b in enumerate(dec["blocks"]):
        label = BLOCK_NAMES.get(i, f"slot {i}")
        if b is None:
            print(f"      {i}  {'absent':>14}   {label}")
        else:
            print(f"      {i}  {b['offset']:6d} +{b['size']:<6d} {label}")

    if d:
        start, res = d["start_angle_deg"], d["resolution_deg"]
        print(f"\n    geometry, READ FROM THE TELEGRAM (brief sec 2.4):")
        print(f"      start {start:+.3f} deg, resolution {res:.5f} deg")
        print(f"      {n} beams -> span {n * res:.2f} deg, "
              f"ending {start + n * res:+.3f} deg")

    if m:
        dist, status = m["dist_mm"], m["status"]
        mask = lf.valid_mask(status, dist)
        good = [x for x, k in zip(dist, mask) if k]
        print(f"\n    points: {len(dist)} total, {sum(mask)} valid with an echo")
        if good:
            print(f"      distance min {min(good)} mm, max {max(good)} mm")
        print(f"      no-echo ({lf.NO_ECHO_MM} mm): "
              f"{sum(1 for x in dist if x >= lf.NO_ECHO_MM)}")
        print(f"      status bytes seen: "
              f"{sorted({hex(s) for s in status})}")

    print(f"\n    cut-off paths:")
    show_zones(dec["zones"])
    print(f"\n    {OK}: decoded {len(tel)} bytes.")
    return 0


def raw(sock):
    ident, tel, _ = one_telegram(sock)
    table = lf.block_table(tel)
    print(f"    telegram {ident}, {len(tel)} bytes.")
    print( "    Read this next to SICK document 8022706. A field stays in "
           "lidarframe's `raw`")
    print( "    dict until somebody has confirmed it here.\n")

    def dump(label, buf, base=0):
        print(f"    {label}")
        for o in range(0, len(buf), 16):
            chunk = buf[o:o + 16]
            words = struct.unpack_from("<%dI" % (len(chunk) // 4), chunk) \
                if len(chunk) % 4 == 0 else ()
            print(f"      +{base + o:04d}  {chunk.hex(' '):<48}"
                  + ("  u32 " + " ".join(str(w) for w in words) if words else ""))
        print()

    dump("telegram header and block table (+000..+079)", tel[:80])
    for i, ext in enumerate(table):
        label = BLOCK_NAMES.get(i, f"slot {i}")
        if ext is None:
            print(f"    block {i}: absent   {label}\n")
            continue
        start, size = ext
        body = tel[start:start + size]
        if i == lf.BLK_MEASUREMENT:
            # 6608 bytes of points is not a hex dump anyone reads; show the
            # shape and the first few points instead.
            pts = list(lf._POINT.iter_unpack(body[:32]))
            print(f"    block {i}: +{start} size {size}   {label}")
            print(f"      {size // lf.POINT_LEN} points x {lf.POINT_LEN} B "
                  f"(u16 distance_mm, u8 rssi, u8 status)")
            print(f"      first 8: " + " ".join(f"{d}mm/r{r}/s{s:#04x}"
                                                for d, r, s in pts))
            print()
            continue
        dump(f"block {i}: +{start} size {size}   {label}", body, start)
    return 0


def watch(sock):
    # Line-buffer explicitly - same reason as modbus_io.watch: block buffering
    # means the change you are waiting for never appears while piped.
    say = lambda s: print(s, flush=True)                      # noqa: E731
    say(f"watching {config.LIDAR_SENSOR_IP}. Rate once a second; zone changes as "
        f"they happen. Ctrl-C to stop.\n")

    asm = lf.Reassembler(config.LIDAR_REASSEMBLY_TIMEOUT_S)
    prev_ident = prev_zones = None
    gaps = n = 0
    t0 = time.monotonic()
    while True:
        try:
            data, addr = sock.recvfrom(65535)
        except TimeoutError:
            say(f"    {time.strftime('%H:%M:%S')}  {BAD} NO DATA for 3 s")
            continue
        if addr[0] != config.LIDAR_SENSOR_IP:
            continue
        done = asm.push(data, time.monotonic())
        if done is None:
            continue
        ident, tel = done
        n += 1
        if prev_ident is not None:
            step = (ident - prev_ident) & 0xFFFFFFFF
            if 1 < step < 0x80000000:
                gaps += step - 1
                say(f"    {time.strftime('%H:%M:%S')}  {WARN} {step - 1} "
                    f"telegram(s) LOST (counter {prev_ident} -> {ident})")
        prev_ident = ident

        z = lf.decode(tel, zone_spec(), points=False)["zones"]
        paths = z.get("paths")
        if paths != prev_zones:
            tag = "" if z.get("validated") else "  [UNVALIDATED mapping]"
            say(f"    {time.strftime('%H:%M:%S')}  zones {paths}{tag}")
            prev_zones = paths

        span = time.monotonic() - t0
        if span >= 1.0:
            say(f"    {time.strftime('%H:%M:%S')}  {n / span:5.1f} Hz   "
                f"{gaps} gap(s), {asm.dropped} partial, {asm.foreign} foreign")
            t0, n = time.monotonic(), 0


def capture(sock, path):
    """Save one telegram's datagrams, length-prefixed, for the offline tests."""
    ident, tel, dgs = one_telegram(sock, keep_datagrams=True)
    with open(path, "wb") as f:
        for d in dgs:
            f.write(struct.pack("<I", len(d)))
            f.write(d)
    print(f"    {OK}: telegram {ident}, {len(dgs)} datagram(s), "
          f"{len(tel)} bytes -> {path}")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Read the SICK nanoScan3 UDP stream (read-only).")
    ap.add_argument("mode", nargs="?", default="scan",
                    choices=("scan", "watch", "raw", "capture"))
    ap.add_argument("file", nargs="?", help="output path for `capture`")
    args = ap.parse_args()

    if args.mode == "capture" and not args.file:
        ap.error("capture needs an output path")
    if not config.LIDAR_ENABLED:
        print(f"{WARN} lidar.enabled is false in profile {config.PROFILE_NAME} - "
              f"listening anyway.")

    try:
        sock = open_socket()
    except OSError as e:
        print(f"    {BAD}: cannot bind {config.LIDAR_HOST_IP}:{config.LIDAR_PORT}"
              f" - {e}")
        # The single most common cause, and not obvious from the errno.
        print( "         If the controller is running, it already holds this "
               "port. Stop it first.")
        return 2
    print(f"listening on {config.LIDAR_HOST_IP}:{config.LIDAR_PORT} for "
          f"{config.LIDAR_SENSOR_IP}\n")

    try:
        if args.mode == "watch":
            return watch(sock) or 0
        if args.mode == "raw":
            return raw(sock)
        if args.mode == "capture":
            return capture(sock, args.file)
        return scan(sock)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    except Exception as e:                    # noqa: BLE001 - report, don't trace
        print(f"    {BAD}: {e}")
        return 1
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
