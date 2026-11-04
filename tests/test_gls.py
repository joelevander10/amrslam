"""GLS621 floor-label camera: telegram framing, parsing, frame aggregation, conversion, and
the read-only bench probe (gls-wait-coding-plan §B.1/§B.2). No network: the telegrams below
are copied from the vehicle's camera, 2026-10-30."""
import math
import os
import tempfile

from helpers import check

from agv_core import ownerlock
from agv_core.drivers import gls, gls_probe

T1 = b"A2x0000y0000;11;9;0.593822;946687420249"
T2 = b"D2x0000y0000;4;9;0.292423;946687420249"
# two results in ONE telegram, no separator (about 3 % of telegrams on the vehicle)
JOINED = b"C2x0000y0000;7;-33;0.584781;946692806114D2x0000y0000;5;-33;0.590747;946692806114"


def tel(p):
    return bytes([gls.STX]) + p + bytes([gls.ETX])


def test_split():
    print("\ngls: framing")
    out, rest, junk = gls.split(tel(T1) + tel(T2))
    check("two telegrams in one read are two", out == [T1, T2] and rest == b"" and junk == 0, str(out))
    a = tel(T1)
    out1, rest1, _ = gls.split(a[:10])
    out2, rest2, _ = gls.split(rest1 + a[10:])
    check("a telegram split across reads is kept and completed",
          out1 == [] and out2 == [T1] and rest2 == b"")
    out, _, junk = gls.split(b"xx" + tel(T1) + b"\r\n")
    check("bytes outside STX..ETX are junk, counted, never a telegram", out == [T1] and junk == 4, str(junk))
    out, _, junk = gls.split(bytes([gls.STX]) + b"lost etx" + tel(T2))
    check("a second STX before an ETX restarts the telegram", out == [T2] and junk == 9, f"{out} {junk}")
    out, rest, junk = gls.split(bytes([gls.STX]) + b"x" * (gls.MAX_TELEGRAM + 1))
    check("an overlong telegram with no ETX is dropped, not buffered forever",
          out == [] and rest == b"" and junk == gls.MAX_TELEGRAM + 2)


def test_parse():
    print("\ngls: parse")
    r = gls.parse(T1)
    check("a result is one code read with its fields",
          r == [gls.CodeRead("A2", "x0000y0000", 11, 9, 0.593822, 946687420249)], str(r))
    r = gls.parse(JOINED)
    check("two results back to back in one telegram are two code reads",
          isinstance(r, list) and [c.pos for c in r] == ["C2", "D2"] and r[1].x == 5 and r[1].y == -33
          and r[0].ts_ms == r[1].ts_ms == 946692806114, str(r))
    r = gls.parse(b"sRA OFtplstr1 49 \\x02[iec _GoodReadCond][f][$BC];[$XMCL]")
    check("a CoLa reply keeps its value verbatim",
          r == gls.Reply("sRA", "OFtplstr1", "49 \\x02[iec _GoodReadCond][f][$BC];[$XMCL]"), str(r))
    check("an error reply is a reply", gls.parse(b"sFA 5") == gls.Reply("sFA", "5", ""))
    bad = [b"", b"hello", b"a2x0000y0000;1;2;3.0;4", b"A2x0000y0000;1;2;3.0", b"A2x0000y0000;1;2;nan;4",
           T1 + b"garbage", b"zz" + T1, b"A2x0000y0000;1.5;2;3.0;4", b"A2x0000y0000;1;2;3.0;4\xff"]
    check("anything not entirely well-formed results is None, never a partial list",
          all(gls.parse(b) is None for b in bad), str([gls.parse(b) for b in bad]))


def reads(*codes, ts=1000, label="x0000y0000"):
    return [gls.CodeRead(p, label, x, y, a, ts) for p, x, y, a in codes]


def test_frames():
    print("\ngls: frames")
    agg = gls.FrameAggregator()
    got = [agg.add(r, 0.0) for r in reads(("A1", 10, 0, 1.0), ("B1", 12, 2, 1.0), ("C1", 14, 4, 1.0))]
    check("codes of one image do not close a frame", got == [None, None, None])
    f = agg.add(reads(("A1", 0, 0, 0.0), ts=1060)[0], 0.06)
    check("the next image's timestamp closes the previous frame",
          f is not None and f.ts_ms == 1000 and f.n_codes == 3 and f.valid, str(f))
    check("x/y are medians, the spread is the furthest code",
          f.x == 12 and f.y == 2 and math.isclose(f.spread_xy, math.hypot(2, 2)), str(f))
    check("flush waits for the gap", agg.flush(0.08, 0.05) is None)
    f = agg.flush(0.12, 0.05)
    check("then closes the last frame before a no-read", f is not None and f.ts_ms == 1060)
    check("and has nothing left", agg.flush(1.0, 0.0) is None)

    f = gls.aggregate(reads(("A1", 1, 1, 359.99), ("B1", 1, 1, 0.29), ("C1", 1, 1, 0.2)))
    check("angles straddling 0/360 have a circular median near 0, not 120",
          min(f.angle_deg, 360 - f.angle_deg) < 0.3 and f.spread_deg < 0.4, str(f.angle_deg))
    f = gls.aggregate(reads(("A1", 1, 1, 0.0)) + reads(("B1", 1, 1, 0.0), label="x0100y0000"))
    check("two labels in one image are refused, not averaged",
          not f.valid and f.label_id == "" and "mixed" in f.reason)
    f = gls.aggregate(reads(("A1", 1, 1, 0.0)), min_codes=2)
    check("fewer codes than min_codes is not valid", not f.valid and "need 2" in f.reason)
    rs = reads(("A1", 10, 0, 1.0), ("B1", 11, 0, 1.0), ("C1", 12, 0, 1.0), ("D1", 60, 0, 1.0))
    f = gls.aggregate(rs, outlier_xy=5)
    check("a code beyond the outlier tolerance is dropped and counted",
          f.n_codes == 3 and f.n_outliers == 1 and "D1" not in f.positions and f.x == 11, str(f))
    f = gls.aggregate(rs)
    check("without a tolerance nothing is dropped", f.n_codes == 4 and f.n_outliers == 0)


def test_conversion():
    print("\ngls: conversion")
    f = gls.aggregate(reads(("A1", 100, -50, 90.0)))
    x, y, yaw = gls.to_camera_pose(f, 0.0001)
    check("counts times scale, degrees to radians",
          math.isclose(x, 0.01) and math.isclose(y, -0.005) and math.isclose(yaw, math.pi / 2),
          f"{x} {y} {yaw}")
    x, y, yaw = gls.to_camera_pose(f, 0.0001, x_sign=-1, y_sign=-1, angle_sign=-1, angle_offset_rad=math.pi)
    check("signs and the angle offset apply, yaw wrapped",
          math.isclose(x, -0.01) and math.isclose(y, 0.005) and math.isclose(yaw, math.pi / 2),
          f"{x} {y} {yaw}")
    x, y, yaw = gls.to_camera_pose(f, 0.0001, swap_xy=True)
    check("swap_xy takes the label x from the device Y and y from X",
          math.isclose(x, -0.005) and math.isclose(y, 0.01) and math.isclose(yaw, math.pi / 2),
          f"{x} {y} {yaw}")
    x, y, yaw = gls.to_camera_pose(f, 0.0001, x_sign=-1, y_sign=1, swap_xy=True)
    check("the signs apply after the swap (x_sign negates the device Y)",
          math.isclose(x, 0.005) and math.isclose(y, 0.01), f"{x} {y} {yaw}")
    refused = []
    for kw in ({"m_per_count": 0.0}, {"m_per_count": float("nan")}, {"m_per_count": 1e-4, "x_sign": 0}):
        try:
            gls.to_camera_pose(f, **kw)
        except ValueError:
            refused.append(kw)
    check("no scale, a bad scale or a bad sign is refused, never guessed", len(refused) == 3, str(refused))


def test_probe_is_read_only_and_owns_the_camera():
    print("\ngls: probe")
    check("commands are CoLa A framed", gls.command("sRN GlsCodeSel") == b"\x02sRN GlsCodeSel\x03")
    check("the probe only ever reads a variable",
          gls_probe._read_only("OFtplstr1") == b"\x02sRN OFtplstr1\x03")
    try:
        gls_probe._read_only("OFtplstr1 1")
        smuggled = True
    except ValueError:
        smuggled = False
    check("a name cannot smuggle in a value or a second command", not smuggled)
    old = os.environ.get("AMR_LOCK_DIR")
    with tempfile.TemporaryDirectory() as d:
        os.environ["AMR_LOCK_DIR"] = d
        try:
            held = ownerlock.acquire("gls", "gls_node")
            try:
                gls_probe.main(["--verify", "--host", "127.0.0.1", "--port", "9"])
                why = ""
            except SystemExit as e:
                why = str(e)
            held.release()
        finally:
            if old is None:
                os.environ.pop("AMR_LOCK_DIR", None)
            else:
                os.environ["AMR_LOCK_DIR"] = old
    check("the probe refuses while the camera's owner holds the gls lock",
          "GLS is in use" in why and "gls_node" in why, why)


TESTS = [
    test_split,
    test_parse,
    test_frames,
    test_conversion,
    test_probe_is_read_only_and_owns_the_camera,
]
