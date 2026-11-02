"""The station-tag reader: framing, parsing, heartbeat, link state.

Fixtures are frames captured from agv-01's reader on 2026-10-08 (tcpdump of the
live service plus read-only queries), not frames shaped from a spec.
"""
import os

from helpers import check

from agv_core import config

# 0x0001 tag reads of tag 0020, as the reader pushed them (RSSI -76.0 .. -72.0 dBm).
TAG_FRAMES = [
    "CF0100010A00FD08010004000000" "20B387",
    "CF0100010A00FD12010004000000" "20CE2C",
    "CF0100010A00FD30010004000000" "2073B3",
    "CF0100010A00FD26010004000000" "20357A",
    "CF0100010A00FD1C010004000000" "20FE21",
]
# 0x0070 device info reply (160 bytes).
DEVICE_INFO = (
    "CF0100709900" "43502D3230333931305F56312E3133" + "00" * 17 +
    "554846204576656E205265616465722056312E3020323032362E30342E323220"
    "4B413BCCA46130524E333730" "4D552D3931305F56312E332D4D" + "00" * 19 +
    "5548462053656E696F72205265616465722056312E342E31322E3236" "00000000"
    "220700183532391446" "4D3936" "DDED")
# 0x0072 all-parameters reply: active, RS232, US 902.75 + 49 x 500 kHz, 30 dBm.
PARAMS = "CF0100721A00" "0100018004000101" "038602EE01F431" "1E0104000000000301" "00" "FB46"


def hx(s):
    return bytes.fromhex(s.replace(" ", ""))


def test_rfid():
    import copy
    import json
    import random
    import shutil
    import tempfile

    from agv_core.drivers import rfid
    print("\nrfid link")

    # -- framing ------------------------------------------------------------
    check("CRC matches the captured init command",
          rfid.crc16(hx("CFFF007000")) == 0x2415)
    check("command() rebuilds the device-info query KIM2A sends",
          rfid.command(rfid.CMD_DEVICE_INFO) == hx("CFFF0070002415"))
    check("command() rebuilds the captured get-params query",
          rfid.command(rfid.CMD_GET_PARAMS) == hx("CFFF00720017A5"))
    check("command() carries data and its CRC",
          rfid.command(0x005F, b"\x02") == hx("CFFF005F0102343E"))

    dec = rfid.CfDecoder()
    stream = b"".join(hx(f) for f in TAG_FRAMES)
    frames, used = dec.decode(stream)
    check("five captured tag frames decode", len(frames) == 5 and used == len(stream),
          f"{len(frames)} frames, used {used}")
    r = rfid.parse_tag(frames[0], 2)
    check("tag id is the last 2 EPC bytes", r is not None and r.tag == "0020", repr(r))
    check("full EPC is kept", r.epc == "00000020", r.epc)
    check("RSSI is 0.1 dBm, signed", r.rssi_dbm == -76.0, str(r.rssi_dbm))
    check("antenna and channel are parsed", (r.antenna, r.channel) == (1, 0))
    check("RSSI spread matches the capture",
          [rfid.parse_tag(f, 2).rssi_dbm for f in frames] == [-76.0, -75.0, -72.0, -73.0, -74.0])

    dec = rfid.CfDecoder()
    got = []
    buf = b""
    for b in stream:                                # worst case: one byte per recv
        buf += bytes([b])
        fr, u = dec.decode(buf)
        buf = buf[u:]
        got += fr
    check("a frame split across reads decodes once", len(got) == 5 and buf == b"")

    frames, used = rfid.CfDecoder().decode(b"\x00\x11" + hx(TAG_FRAMES[0]))
    check("junk before the header is skipped", len(frames) == 1 and used == 19)
    frames, used = rfid.CfDecoder().decode(hx(TAG_FRAMES[0])[:9])
    check("a partial frame is kept, not consumed", frames == [] and used == 0, f"used={used}")

    bad = bytearray(hx(TAG_FRAMES[0]))
    bad[-1] ^= 0xFF
    dec = rfid.CfDecoder()
    frames, _ = dec.decode(bytes(bad) + hx(TAG_FRAMES[1]))
    check("a bad CRC is dropped and the next frame still found",
          len(frames) == 1 and rfid.parse_tag(frames[0], 2).rssi_dbm == -75.0
          and dec.bad_frames >= 1, f"{len(frames)} frames, bad={dec.bad_frames}")

    def resp(cmd, status, data, addr=0x01):
        body = bytes([0xCF, addr, cmd >> 8, cmd & 0xFF, 1 + len(data), status]) + data
        c = rfid.crc16(body)
        return body + bytes([c >> 8, c & 0xFF])

    embedded = resp(rfid.CMD_TAG, 0, b"\xFF\x10\x01\x05\x04" + b"\xCF\xCF\x00\x21")
    frames, _ = rfid.CfDecoder().decode(embedded)
    check("a payload full of 0xCF still decodes as ONE frame",
          len(frames) == 1 and rfid.parse_tag(frames[0], 2).tag == "0021", f"{len(frames)}")
    long_epc = resp(rfid.CMD_TAG, 0, b"\xFE\x00\x01\x07\x0C" + bytes(range(1, 13)))
    frames, _ = rfid.CfDecoder().decode(long_epc)
    t = rfid.parse_tag(frames[0], 2)
    check("a 12-byte EPC (25-byte frame) decodes; id is its tail",
          len(long_epc) == 25 and t.tag == "0B0C" and t.epc.endswith("0B0C"), repr(t))
    short_epc = resp(rfid.CMD_TAG, 0, b"\xFE\x00\x01\x00\x01\xAA")
    check("an EPC shorter than tag_len is not a tag",
          rfid.parse_tag(rfid.CfDecoder().decode(short_epc)[0][0], 2) is None)
    done = rfid.CfDecoder().decode(resp(rfid.CMD_TAG, rfid.STATUS_INVENTORY_DONE, b""))[0][0]
    check("status 0x12 (round done) is not a tag", rfid.parse_tag(done, 2) is None)

    # -- the device-info reply: what used to become the "3130" tag ------------
    info_bytes = hx(DEVICE_INFO)
    check("device-info fixture is the captured 160 bytes", len(info_bytes) == 160)
    frames, used = rfid.CfDecoder().decode(info_bytes + hx(TAG_FRAMES[0]))
    check("device-info reply + tag decode as two frames, CRC ok",
          [f.cmd for f in frames] == [rfid.CMD_DEVICE_INFO, rfid.CMD_TAG], str(frames)[:80])
    check("the device-info reply yields no tag (no 3130 artifact)",
          [rfid.parse_tag(f, 2) for f in frames][0] is None)
    ident = rfid.parse_device_info(frames[0])
    check("identity names the CP and RF firmware",
          ident is not None and "CP-203910_V1.13" in ident
          and "UHF Senior Reader V1.4.12.26" in ident, str(ident))

    p = rfid.parse_params(rfid.CfDecoder().decode(hx(PARAMS))[0][0])
    check("parameters reply parses", p is not None)
    check("work mode is active", (p["work_mode"], p["work_mode_name"]) == (1, "active"))
    check("interface is reported as RS232", p["interface_name"] == "RS232")
    check("region US, 902.75-927.25 MHz, 49 x 500 kHz",
          (p["region_name"], p["start_mhz"], p["end_mhz"], p["step_khz"], p["channels"])
          == ("US", 902.75, 927.25, 500, 49), str(p))
    check("power 30 dBm, Q 4, session 0",
          (p["power_dbm"], p["q_value"], p["session"]) == (30, 4, 0))
    check("the profile's expectations match the captured unit", rfid.param_mismatches(p) == [],
          str(rfid.param_mismatches(p)))
    check("the summary line reads as the HMI shows it",
          rfid.params_summary(p) == "active RS232 US 902.75-927.25 MHz 30 dBm", rfid.params_summary(p))
    check("a different region is a mismatch",
          len(rfid.param_mismatches(dict(p, region=0))) == 1)

    random.seed(11)
    ok = True
    for _ in range(400):
        blob = bytes(random.randrange(256) for _ in range(random.randrange(64)))
        blob = (b"\xCF" + blob) if random.random() < 0.5 else blob
        try:
            fr, u = rfid.CfDecoder().decode(blob)
            assert 0 <= u <= len(blob)
            for f in fr:
                rfid.parse_tag(f, 2)
                rfid.parse_device_info(f)
                rfid.parse_params(f)
        except Exception as e:                      # noqa: BLE001
            check("decode never raises on random bytes", False, repr(e))
            ok = False
            break
    if ok:
        check("decode and the parsers never raise on random bytes", True)

    # -- heartbeat (clock-fed) -----------------------------------------------
    hb = rfid.Heartbeat(0.0, 3.0)
    hb.reset(0.0)
    check("heartbeat 0 is off and never due", hb.state() == "off" and not hb.due(0.0)
          and hb.check(100.0) is None)
    hb = rfid.Heartbeat(1.0, 3.0)
    hb.reset(0.0)
    check("first request is due at connect", hb.due(0.0) and hb.state() == "waiting")
    hb.sent(0.0)
    check("not due again inside the period", not hb.due(0.5))
    hb.reply(0.1)
    check("a reply arms it", hb.state() == "ok" and hb.check(3.0) is None)
    check("armed and quiet past the timeout is dead", hb.check(3.2) == "dead")
    hb = rfid.Heartbeat(1.0, 3.0)
    hb.reset(10.0)
    check("never answered: gives up once, after the timeout",
          hb.check(12.0) is None and hb.check(13.5) == "gave_up" and hb.check(20.0) is None)
    check("after giving up nothing more is sent, nothing is dead",
          not hb.due(30.0) and hb.state() == "unanswered")
    hb.reset(40.0)
    check("a reconnect starts the heartbeat fresh", hb.state() == "waiting" and hb.due(40.0))

    # -- pass statistics (clock-fed) ---------------------------------------------
    ps = rfid.PassStats(0.5)
    for t, r in ((0.0, -72.0), (0.1, -70.0), (0.3, -76.0)):
        ps.read("0040", r, t)
    check("reads inside clear_s are one open pass", not ps.closed)
    ps.expire(0.7)
    check("a pass stays open until clear_s after its last read", not ps.closed)
    ps.expire(0.8)
    p = ps.closed[-1][1] if ps.closed else {}
    check("a closed pass carries reads, duration and the RSSI spread",
          (p.get("tag"), p.get("reads"), round(p.get("duration_s", 0), 2), p.get("rssi_peak"),
           round(p.get("rssi_mean", 0), 2), p.get("rssi_min")) == ("0040", 3, 0.3, -70.0, -72.67, -76.0),
          str(p))
    ps.read("0060", -70.0, 1.0)
    ps.read("0040", -71.0, 1.1)
    ps.read("0060", -69.0, 1.4)
    ps.expire(1.7)
    check("overlapping passes of two tags close independently",
          [c[1]["tag"] for c in ps.closed] == ["0040", "0040"] and ps.seq == 2,
          str([c[1]["tag"] for c in ps.closed]))
    ps.expire(1.8, force=True)
    check("a forced expiry (reconnect) closes every open pass, numbered on",
          [(c[0], c[1]["tag"], c[1]["reads"]) for c in ps.closed][-1] == (3, "0060", 2))
    check("the pass log line keeps the format bag_report parses",
          rfid.pass_summary(ps.closed[0][1])
          == "RFID pass 0040: 3 reads in 0.30 s, RSSI peak -70.0 mean -72.7 min -76.0 dBm",
          rfid.pass_summary(ps.closed[0][1]))

    # -- link state from captured frames (no socket) ---------------------------
    link = rfid.RfidLink()
    snap = link.snapshot()
    check("carrier is read from the real NIC, or None if absent",
          snap["carrier"] in (True, False, None), str(snap["carrier"]))
    check("silence is not a fault while connected", snap["silent"] is False)
    link._absorb(rfid.CfDecoder().decode(hx(PARAMS) + info_bytes)[0])
    link._absorb(rfid.CfDecoder().decode(stream)[0])
    snap = link.snapshot(encounters=True)
    check("five reads of one tag are ONE encounter",
          snap["encounters"] == [(1, "0020")] and snap["tags_seen"] == 5, str(snap["encounters"]))
    link.passes.expire(float("inf"))
    snap = link.snapshot(encounters=True)
    check("the five captured reads are one pass with their RSSI spread",
          [(c[1]["reads"], c[1]["rssi_peak"], c[1]["rssi_min"]) for c in snap["passes"]]
          == [(5, -72.0, -76.0)], str(snap["passes"]))
    check("snapshot carries RSSI, channel and frequency of the last read",
          (snap["rssi_dbm"], snap["channel"], snap["freq_mhz"]) == (-74.0, 0, 902.75),
          f"{snap['rssi_dbm']} {snap['channel']} {snap['freq_mhz']}")
    check("snapshot carries the reader's parameters and identity",
          snap["reader"]["region_name"] == "US" and "CP-203910" in (snap["identity"] or ""))
    check("no config mismatch on the captured unit", snap["config_mismatch"] == [])
    link.ignore = {"0020"}
    before = link.snapshot()["tags_seen"]
    link._absorb(rfid.CfDecoder().decode(hx(TAG_FRAMES[0]))[0])
    check("an ignored id never counts", link.snapshot()["tags_seen"] == before)
    link.heartbeat = rfid.Heartbeat(1.0, 3.0)
    link.heartbeat.reset(0.0)
    link._absorb(rfid.CfDecoder().decode(resp(rfid.CMD_ONLINE, 0, b""))[0])
    check("a 0x0050 reply arms the link heartbeat", link.heartbeat.state() == "ok")

    # -- profile refusals --------------------------------------------------------
    base = json.load(open(config.profile_path()))

    def refuses(name, mutate, expect):
        d = copy.deepcopy(base)
        mutate(d)
        # Under its profile name - the loader checks the stem, so a temp name
        # would be refused for the wrong reason. See test_config_profile().
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, "agv-01.json")
        with open(path, "w") as fh:
            json.dump(d, fh)
        try:
            config.load(path)
            check(name, False, "accepted!")
        except config.ConfigError as e:
            check(name, expect in str(e), str(e)[:70])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
            config.load()

    refuses("tag_len 0 is refused", lambda d: d["rfid"].update(tag_len=0), "1..12")
    refuses("tag_len beyond a 96-bit EPC is refused", lambda d: d["rfid"].update(tag_len=13), "1..12")
    refuses("the retired frame_len key is refused",
            lambda d: d["rfid"].update(frame_len=17), "unknown key")
    refuses("an ignore_tags entry of the wrong width is refused",
            lambda d: d["rfid"].update(ignore_tags=["31"]), "hex chars")
    refuses("silent_warn below recv_timeout is refused",
            lambda d: d["rfid"].update(silent_warn_s=0.5), "silent_warn_s")
    refuses("a negative heartbeat is refused",
            lambda d: d["rfid"].update(heartbeat_s=-1.0), "heartbeat_s must be >= 0")
    refuses("a heartbeat timeout inside one period is refused",
            lambda d: d["rfid"].update(heartbeat_s=2.0, heartbeat_timeout_s=2.0),
            "must exceed heartbeat_s")
    refuses("an unknown work mode is refused",
            lambda d: d["rfid"].update(expect_work_mode=3), "expect_work_mode")
    refuses("an unknown region is refused",
            lambda d: d["rfid"].update(expect_region=9), "expect_region")
    refuses("power above 30 dBm is refused",
            lambda d: d["rfid"].update(expect_power_dbm=31), "expect_power_dbm")


def test_rfid_survey():
    """The survey tool against a fake reader replaying the captured frames, and
    its refusal while rfid_node owns the reader."""
    import contextlib
    import io
    import shutil
    import socket
    import tempfile
    import threading

    from agv_core import ownerlock
    from agv_core.drivers import rfid_survey
    print("\nrfid survey (read-only)")

    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    got = []

    def reader():
        c, _ = srv.accept()
        got.append(c.recv(64))                      # the two queries
        c.sendall(hx(DEVICE_INFO) + hx(PARAMS))
        for f in TAG_FRAMES * 5:                    # split mid-frame on purpose
            c.sendall(hx(f)[:7])
            c.sendall(hx(f)[7:])
        c.close()

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    saved = config.RFID_IP, config.RFID_PORT
    config.RFID_IP, config.RFID_PORT = "127.0.0.1", srv.getsockname()[1]
    out = io.StringIO()
    try:
        rows, params = rfid_survey.survey(1.0, out=out)
    except ConnectionResetError:
        rows, params = None, None
    finally:
        config.RFID_IP, config.RFID_PORT = saved
        srv.close()
    th.join(2.0)
    from agv_core.drivers import rfid
    check("the survey opens with the device-info query ALONE in its segment",
          got and got[0] == rfid.command(rfid.CMD_DEVICE_INFO), got[0].hex() if got else "nothing")
    check("a reader closing the link is reported, not swallowed", rows is None)

    # Same replay, but the fake reader stays open until the survey window ends.
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    hold = threading.Event()

    def reader_open():
        c, _ = srv.accept()
        c.recv(64)
        c.sendall(hx(DEVICE_INFO) + hx(PARAMS) + b"".join(hx(f) for f in TAG_FRAMES * 5))
        hold.wait(3.0)
        c.close()

    th = threading.Thread(target=reader_open, daemon=True)
    th.start()
    config.RFID_IP, config.RFID_PORT = "127.0.0.1", srv.getsockname()[1]
    out = io.StringIO()
    try:
        rows, params = rfid_survey.survey(0.8, out=out)
    finally:
        hold.set()
        config.RFID_IP, config.RFID_PORT = saved
        srv.close()
    text = out.getvalue()
    check("every replayed read is counted", len(rows) == 25, str(len(rows)))
    check("the table shows channel 0 at 902.750 MHz with the RSSI spread",
          "902.750" in text and "-76.0" in text and "-72.0" in text, text[-200:])
    check("one channel across 25 reads is called out", "not hopping" in text)
    check("reader identity and config are printed",
          "CP-203910_V1.13" in text and "active RS232 US" in text)

    tmp = tempfile.mkdtemp()
    old_env = os.environ.get("AMR_LOCK_DIR")
    os.environ["AMR_LOCK_DIR"] = tmp
    try:
        held = ownerlock.acquire("rfid", "rfid_node")
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = rfid_survey.main(["--seconds", "0.1"])
        check("the survey refuses while rfid_node owns the reader",
              rc == 2 and "rfid_node" in out.getvalue(), out.getvalue()[:80])
        held.release()
    finally:
        if old_env is None:
            os.environ.pop("AMR_LOCK_DIR", None)
        else:
            os.environ["AMR_LOCK_DIR"] = old_env
        shutil.rmtree(tmp, ignore_errors=True)


def test_rfid_link_session():
    """The real session loop against a fake reader that, like agv-01's, must
    never receive two commands in one TCP segment (2026-10-08: that left the
    reader silent for the whole connection)."""
    import socket
    import threading
    import time

    from agv_core.drivers import rfid
    print("\nrfid link session")

    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    segments = []
    done = threading.Event()

    def reader():
        c, _ = srv.accept()
        c.settimeout(0.1)
        replied = set()
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
            try:
                seg = c.recv(256)
            except TimeoutError:
                continue
            if not seg:
                break
            segments.append(seg)
            if seg == rfid.command(rfid.CMD_DEVICE_INFO) and "info" not in replied:
                replied.add("info")
                c.sendall(hx(DEVICE_INFO))
                for f in TAG_FRAMES:
                    c.sendall(hx(f))
            elif seg == rfid.command(rfid.CMD_GET_PARAMS) and "params" not in replied:
                replied.add("params")
                c.sendall(hx(PARAMS))
        done.set()
        c.close()

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    saved = config.RFID_IP, config.RFID_PORT, config.RFID_INTERFACE
    config.RFID_IP, config.RFID_PORT = "127.0.0.1", srv.getsockname()[1]
    config.RFID_INTERFACE = "lo"
    link = rfid.RfidLink()
    try:
        link.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not (link.snapshot()["reader"] and link.snapshot()["tag"]):
            time.sleep(0.05)
        snap = link.snapshot(encounters=True)
    finally:
        link.stop()
        config.RFID_IP, config.RFID_PORT, config.RFID_INTERFACE = saved
        done.wait(3.0)
        srv.close()
    check("the first segment is the device-info query alone",
          segments[:1] == [rfid.command(rfid.CMD_DEVICE_INFO)], [x.hex() for x in segments][:2])
    check("no segment ever carries two commands",
          all(len(rfid.CfDecoder().decode(x)[0]) <= 1 and x[4] + 7 == len(x) for x in segments),
          [x.hex() for x in segments])
    check("parameters are queried after the identity reply",
          rfid.command(rfid.CMD_GET_PARAMS) in segments[1:2], [x.hex() for x in segments])
    check("identity, parameters and the tag all arrive over the session",
          bool(snap["identity"]) and snap["reader"] and snap["reader"]["region_name"] == "US"
          and snap["tag"] == "0020" and snap["encounters"] == [(1, "0020")], str(snap)[:120])


TESTS = [
    test_rfid,
    test_rfid_survey,
    test_rfid_link_session,
]
