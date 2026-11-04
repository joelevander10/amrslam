"""Bench tool: read the GLS621 floor-label camera, read-only.

    python3 -m agv_core.drivers.gls_probe --verify             # the settings the integration relies on
    python3 -m agv_core.drivers.gls_probe --frames -t 30       # one aggregated line per camera image
    python3 -m agv_core.drivers.gls_probe --raw gls.log -t 60  # every telegram, timestamped (bench §7)

For the bench tests in manuals/gls6-qr/gls6-system.md §7 (units, signs, angle, repeatability,
rate, capture window): --raw keeps every telegram with its receive time so the analysis can be
redone later, --frames shows what the aggregation makes of them as you move the label.

READ-ONLY. The only thing it ever sends is CoLa A `sRN` (read variable); configuration lives in
the device (SOPAS, saved permanently) and this tool cannot change it. Takes the "gls" owner
lock, so it refuses to run while the vehicle's own GLS node owns the camera.
"""

import argparse
import socket
import sys
import time

from agv_core import ownerlock
from agv_core.drivers import gls

HOST, PORT = "192.168.4.1", 2112  # results port; commands are answered on it too
# What the integration depends on, as read back on 2026-10-30 (gls-wait-coding-plan §2).
VERIFY = ("OFtplstr1", "GlsCodeSel", "DCDMVerISO15415", "DC2DContrast")
RESEND_S = 1.0


def _read_only(name: str) -> bytes:
    if not name.replace("_", "").isalnum():
        raise ValueError(f"not a variable name: {name!r}")
    return gls.command(f"sRN {name}")


def _fmt(f: gls.Frame) -> str:
    if not f.valid:
        return f"{f.ts_ms}  INVALID  {f.reason}"
    return (
        f"{f.ts_ms}  {f.label_id}  n={f.n_codes:2d}{f' (-{f.n_outliers})' if f.n_outliers else ''}"
        f"  x={f.x:7.1f}  y={f.y:7.1f}  a={f.angle_deg:7.2f}"
        f"  spread {f.spread_xy:5.1f} cnt / {f.spread_deg:4.2f} deg  {','.join(f.positions)}"
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("-t", "--seconds", type=float, default=10.0, help="how long to listen (default 10)")
    ap.add_argument("--verify", action="store_true", help=f"read back {', '.join(VERIFY)}")
    ap.add_argument("--frames", action="store_true", help="print one line per aggregated image")
    ap.add_argument("--raw", metavar="FILE", help="append every telegram, with its receive time, to FILE")
    ap.add_argument("--gap", type=float, default=0.05, help="close a frame after this silence, s")
    ap.add_argument("--min-codes", type=int, default=1)
    ap.add_argument("--outlier", type=float, default=None, help="drop codes this many counts off the median")
    args = ap.parse_args(argv)
    if not (args.verify or args.frames or args.raw):
        args.frames = True

    try:
        lock = ownerlock.acquire("gls", "gls_probe")  # noqa: F841 - held until exit
    except ownerlock.OwnerBusy as e:
        sys.exit(f"GLS is in use ({e}). Stop amr.service first.")
    try:
        sock = socket.create_connection((args.host, args.port), timeout=3.0)
    except OSError as e:
        sys.exit(f"cannot reach the GLS at {args.host}:{args.port}: {e}")
    sock.settimeout(0.02)
    raw = open(args.raw, "a", encoding="ascii", errors="replace") if args.raw else None  # noqa: SIM115
    agg = gls.FrameAggregator(args.min_codes, outlier_xy=args.outlier)
    want = {n: None for n in VERIFY} if args.verify else {}
    sent_t = 0.0
    n_tel = n_frames = n_unknown = n_junk = n_invalid = 0
    buf = b""
    start = time.monotonic()
    # --verify alone ends as soon as every reply is in (or the time is up)
    end = start + args.seconds
    try:
        while (now := time.monotonic()) < end:
            missing = [n for n, v in want.items() if v is None]
            if missing and now - sent_t >= RESEND_S:
                for name in missing:
                    sock.sendall(_read_only(name))
                sent_t = now
            elif args.verify and not missing and not (args.frames or args.raw):
                break
            try:
                data = sock.recv(4096)
            except TimeoutError:
                data = None
            if data == b"":
                print("the GLS closed the connection")
                break
            frames = []
            if data:
                tels, buf, junk = gls.split(buf + data)
                n_junk += junk
                for p in tels:
                    n_tel += 1
                    if raw:
                        raw.write(f"{time.time():.4f}\t{p.decode('ascii', 'replace')}\n")
                    r = gls.parse(p)
                    if isinstance(r, list):
                        for read in r:
                            f = agg.add(read, now)
                            if f:
                                frames.append(f)
                    elif isinstance(r, gls.Reply):
                        if r.name in want and want[r.name] is None:
                            want[r.name] = f"{r.kind} {r.value}"
                            print(f"{r.name:16s} {r.kind} {r.value}")
                    else:
                        n_unknown += 1
            f = agg.flush(time.monotonic(), args.gap)
            if f:
                frames.append(f)
            for f in frames:
                n_frames += 1
                n_invalid += not f.valid
                if args.frames:
                    print(_fmt(f), flush=True)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        sock.close()
        if raw:
            raw.close()
    took = max(1e-6, time.monotonic() - start)
    for name, v in want.items():
        if v is None:
            print(f"{name:16s} NO REPLY")
    print(
        f"{took:.1f} s: {n_tel} telegrams, {n_frames} frames ({n_frames / took:.1f}/s, {n_invalid} invalid), "
        f"{n_unknown} unrecognised, {n_junk} junk bytes"
    )
    return 1 if any(v is None for v in want.values()) else 0


if __name__ == "__main__":
    sys.exit(main())
