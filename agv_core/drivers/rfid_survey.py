"""RFID read-quality survey: per-channel read counts and RSSI, read-only.

    python3 -m agv_core.drivers.rfid_survey                 # 60 s, profile reader
    python3 -m agv_core.drivers.rfid_survey --seconds 300 --csv reads.csv

Hold or park a tag at the antenna (fixed distance and angle) and let it run.
The table answers two questions the reader's own settings cannot:
  - margin: how far the RSSI sits above the reader's floor at this geometry
  - band:   whether some channels read worse (interference, tag tuning) -
            the comparison to make before and after a region change.

Read-only by construction: it sends only 0x0070 (device info) and 0x0072
(parameters), exactly what the driver sends on connect, then listens. It never
sends a command that changes reader settings, so there is no --go.

Takes the "rfid" owner lock and refuses while rfid_node holds it - the reader
serves a second client, but two listeners make the event log and this table
disagree about who saw what. Stop amr.service first.
"""
import argparse
import csv
import socket
import statistics
import sys
import time
from collections import defaultdict

from agv_core import config
from agv_core.drivers import rfid

BAD = "\033[31mFAIL\033[0m"


def survey(seconds, csv_path=None, out=sys.stdout):
    sock = rfid._hardened(socket.create_connection((config.RFID_IP, config.RFID_PORT), timeout=2.0))
    sock.settimeout(0.5)
    # One command per TCP segment - the reader goes silent on two (see rfid._session).
    sock.sendall(rfid.command(rfid.CMD_DEVICE_INFO))
    time.sleep(rfid.PARAMS_AFTER_S)
    sock.sendall(rfid.command(rfid.CMD_GET_PARAMS))
    dec = rfid.CfDecoder()
    params, ident = None, None
    by_ch = defaultdict(list)
    rows = []
    t0 = time.monotonic()
    buf = b""
    try:
        while time.monotonic() - t0 < seconds:
            try:
                chunk = sock.recv(4096)
            except TimeoutError:
                continue
            if not chunk:
                raise ConnectionResetError("reader closed the connection")
            buf += chunk
            frames, used = dec.decode(buf)
            buf = buf[used:]
            for f in frames:
                if f.cmd == rfid.CMD_GET_PARAMS:
                    params = rfid.parse_params(f) or params
                elif f.cmd == rfid.CMD_DEVICE_INFO:
                    ident = rfid.parse_device_info(f) or ident
                elif (r := rfid.parse_tag(f, config.RFID_TAG_LEN)) is not None:
                    t = time.monotonic() - t0
                    by_ch[r.channel].append(r.rssi_dbm)
                    rows.append((round(t, 3), r.tag, r.epc, r.rssi_dbm, r.antenna, r.channel))
    finally:
        sock.close()

    elapsed = time.monotonic() - t0
    print(f"reader  {ident or '(no device info)'}", file=out)
    print(f"config  {rfid.params_summary(params) or '(no parameters)'}", file=out)
    for m in rfid.param_mismatches(params) if params else ():
        print(f"WARN    {m}", file=out)
    tags = sorted({r[1] for r in rows})
    print(f"reads   {len(rows)} in {elapsed:.1f} s ({len(rows) / elapsed:.1f}/s), "
          f"tags {', '.join(tags) or 'none'}, bad frames {dec.bad_frames}\n", file=out)
    if rows:
        print(f"{'ch':>3} {'MHz':>8} {'reads':>6} {'min':>7} {'median':>7} {'max':>7}", file=out)
        for ch in sorted(by_ch):
            v = by_ch[ch]
            mhz = (f"{params['start_mhz'] + ch * params['step_khz'] / 1000.0:8.3f}"
                   if params else f"{'?':>8}")
            print(f"{ch:>3} {mhz} {len(v):>6} {min(v):>7.1f} {statistics.median(v):>7.1f} "
                  f"{max(v):>7.1f}", file=out)
        if len(by_ch) == 1 and len(rows) >= 20:
            print("\nNOTE    every read reported one channel: the reader is not hopping, or this "
                  "field is not the hop channel on this firmware.", file=out)
    else:
        print("no tag read - is a tag at the antenna?", file=out)

    if csv_path:
        with open(csv_path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "tag", "epc", "rssi_dbm", "antenna", "channel"])
            w.writerows(rows)
        print(f"\nwrote {len(rows)} reads to {csv_path}", file=out)
    return rows, params


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--csv", help="also write every read to this CSV")
    args = ap.parse_args(argv)

    from agv_core import ownerlock  # noqa: PLC0415
    try:
        lock = ownerlock.acquire("rfid", "rfid_survey")  # noqa: F841 - held until exit
    except ownerlock.OwnerBusy as e:
        print(f"{BAD}: the RFID reader is owned ({e}). Stop it first: sudo systemctl stop amr.service")
        return 2
    try:
        survey(args.seconds, args.csv)
    except OSError as e:
        print(f"{BAD}: {config.RFID_IP}:{config.RFID_PORT}: {e}")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
