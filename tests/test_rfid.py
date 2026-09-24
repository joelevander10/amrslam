"""The station-tag reader: framing, codec, link health."""
import math
import os
import pathlib
import struct
import sys
import threading

from helpers import FAIL, ROOT, check

import config
import kinematics
import motion

def test_rfid():
    """Protocol is not inferred - it is the same reader hardware as KIM2A, so the
    framing is checked against frames shaped exactly like the ones their
    production parser handles."""
    import copy
    import json
    import tempfile
    import rfid
    print("\nrfid link")

    cx = rfid.ChafonCFCodec(init=bytes.fromhex("CFFF0070002415"),
                            frame_len=17, tag_offset=13, tag_len=2,
                            ignore=("3130",))

    check("init command is the one the working vehicle sends",
          cx.handshake() == bytes.fromhex("CFFF0070002415"),
          cx.handshake().hex(" ").upper())
    check("push-only: there is nothing to poll", cx.poll() is None)

    def frame(tag_hex, fill=b"\x00"):
        f = bytearray(b"\xCF" + fill * 16)
        f[13:15] = bytes.fromhex(tag_hex)
        return bytes(f)

    good = frame("1234")
    frames, used = cx.decode(good)
    check("a 17-byte frame decodes", len(frames) == 1 and used == 17)
    check("tag comes from bytes 13:15", cx.tag_of(frames[0]) == "1234",
          str(cx.tag_of(frames[0])))

    check("the 3130 startup echo is not a tag",
          cx.tag_of(frame("3130")) is None)

    frames, used = cx.decode(b"\x00\x11" + good)
    check("junk before the header is skipped", len(frames) == 1 and used == 19)

    frames, used = cx.decode(good + good)
    check("two frames in one read both decode", len(frames) == 2)

    frames, used = cx.decode(good[:9])
    check("a partial frame is kept, not consumed",
          frames == [] and used == 0, f"used={used}")
    frames, used = cx.decode(good + good[:9])
    check("trailing partial survives a whole frame",
          len(frames) == 1 and used == 17, f"used={used}")

    # KIM2A splits the hex STRING on "CF" and length-checks the pieces, so a
    # frame whose payload contains 0xCF is torn in two and dropped. Scanning for
    # the header and taking a fixed span keeps it.
    embedded = bytearray(good)
    embedded[5] = 0xCF
    frames, _ = cx.decode(bytes(embedded))
    check("a payload containing 0xCF still decodes as ONE frame",
          len(frames) == 1 and cx.tag_of(frames[0]) == "1234",
          f"{len(frames)} frame(s)")

    import random
    random.seed(11)
    ok = True
    for _ in range(400):
        blob = bytes(random.randrange(256) for _ in range(random.randrange(48)))
        try:
            fr, u = cx.decode(blob)
            assert 0 <= u <= len(blob)
            for f in fr:
                cx.tag_of(f)
        except Exception as e:                      # noqa: BLE001
            check("decode never raises on random bytes", False, repr(e))
            ok = False
            break
    if ok:
        check("decode and tag_of never raise on random bytes", True)

    link = rfid.RfidLink()
    snap = link.snapshot()
    check("carrier is read from the real NIC, or None if absent",
          snap["carrier"] in (True, False, None), str(snap["carrier"]))
    check("silence is not a fault while connected",
          snap["silent"] is False, str(snap["silent"]))

    base = json.load(open(config.profile_path()))

    def refuses(name, mutate, expect):
        import shutil
        d = copy.deepcopy(base)
        mutate(d)
        # Under its profile name - the loader checks the stem, so a temp name
        # would be refused for the wrong reason. See test_config_profile().
        tmp = tempfile.mkdtemp()
        p = os.path.join(tmp, "agv-01.json")
        with open(p, "w") as fh:
            json.dump(d, fh)
        try:
            config.load(p)
            check(name, False, "accepted!")
        except config.ConfigError as e:
            check(name, expect in str(e), str(e)[:70])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
            config.load()

    refuses("a tag slice outside the frame is refused",
            lambda d: d["rfid"].update(tag_offset=16), "must fit inside")
    refuses("a non-hex init command is refused",
            lambda d: d["rfid"].update(init_hex="zz"), "init_hex")
    refuses("enabling with no init command is refused",
            lambda d: d["rfid"].update(init_hex=""), "stay silent")
    refuses("an ignore_tags entry of the wrong width is refused",
            lambda d: d["rfid"].update(ignore_tags=["31"]), "hex chars")
    refuses("silent_warn below recv_timeout is refused",
            lambda d: d["rfid"].update(silent_warn_s=0.5), "silent_warn_s")


TESTS = [
    test_rfid,
]
