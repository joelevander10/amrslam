"""The platform key and the AMR QR profile (platform qr_analog, section qr_base)."""
import copy
import json
import os
import shutil
import tempfile

from helpers import check

import config


def _doc(name):
    with open(config.profile_path(name)) as fh:
        return json.load(fh)


def _load_with(base, mutate, name):
    d = copy.deepcopy(base)
    mutate(d)
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, f"{name}.json")
    with open(path, "w") as fh:
        json.dump(d, fh)
    try:
        config.load(path)
        return None
    except config.ConfigError as e:
        return str(e)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        config.load()  # always restore the real profile


def test_qr_profile():
    print("\nplatform key and the AMR QR profile")
    qr, agv = _doc("amr-qr-01"), _doc("agv-01")

    def refuses(name, base, mutate, expect, profile):
        msg = _load_with(base, mutate, profile)
        check(name, msg is not None and expect in msg, msg or "accepted!")

    def qr_refuses(name, mutate, expect):
        refuses(name, qr, mutate, expect, "amr-qr-01")

    def qb(**kw):
        return lambda d: d["qr_base"].update(kw)

    msg = _load_with(qr, lambda d: None, "amr-qr-01")
    check("amr-qr-01 loads", msg is None, msg or "")
    config.load(config.profile_path("amr-qr-01"))
    try:
        check("amr-qr-01 is platform qr_analog", config.PLATFORM == config.PLATFORM_QR)
        check("track and wheel are the QR's", (config.TRACK_M, config.WHEEL_DIA_M) == (0.37652, 0.18))
        check("the right motor is mirrored (QR wiring)", config.INVERT_RIGHT and not config.INVERT_LEFT)
        check("top speed derives from motor_max_rpm, under the voltage cap",
              0.1 < config.MAX_SPEED_MPS < 0.5, f"{config.MAX_SPEED_MPS:.3f} m/s")
        check("mux accel ceiling is 0.5 m/s^2 from the software ramp",
              abs(config.ACCEL_RPM_S * config.MPS_PER_RPM - 0.5) < 0.01)
        names = [s["name"] for s in config.describe()]
        check("the params page shows platform and qr_base", "platform" in names and "qr_base" in names)
        notes = config.tuning_notes()
        check("qr_base.* and platform.* notes are parsed", "qr_base.*" in notes and "platform.*" in notes)
    finally:
        config.load()
    check("agv-01 is platform blvr_canopen", config.PLATFORM == config.PLATFORM_BLVR)

    refuses("a profile without platform is refused", agv, lambda d: d.pop("platform"),
            "missing top-level key(s): ['platform']", "agv-01")
    refuses("an unknown platform is refused", agv, lambda d: d.update(platform="kinco"),
            "platform: expected one of", "agv-01")
    refuses("qr_base in a blvr_canopen profile is refused", agv,
            lambda d: d.update(qr_base=copy.deepcopy(qr["qr_base"])), "qr_base belongs only", "agv-01")
    qr_refuses("a qr_analog profile without qr_base is refused", lambda d: d.pop("qr_base"),
               "missing top-level key(s): ['qr_base']")
    qr_refuses("a typo'd qr_base key is refused",
               lambda d: d["qr_base"].update(kp=d["qr_base"].pop("kp_v_per_rad_s")), "unknown key")
    qr_refuses("motor_max_rpm beyond the feedforward headroom is refused",
               lambda d: d["vehicle"].update(motor_max_rpm=3000.0), "needs")
    qr_refuses("v_max above the module's full scale is refused", qb(v_max_v=12.0), "v_max_v")
    qr_refuses("two motor coils on one channel are refused", qb(do_left_rev=4), "must be distinct")
    qr_refuses("a motor coil shared with the horn is refused",
               lambda d: (d["horn"].update(enabled=True, do_channel=1)), "also a motor coil")
    qr_refuses("the two analog channels must differ", qb(ao_ch_left=0), "two distinct channels")
    qr_refuses("analog and digital modules on one address are refused", qb(ao_ip="192.168.1.40"),
               "copy-paste")
    qr_refuses("E-stop on the START input is refused", qb(di_estop=1), "collides")
    qr_refuses("an unknown encoder mode is refused", qb(enc_mode="canopen"), "enc_mode")
    qr_refuses("an encoder timeout under 3 event periods is refused", qb(enc_timeout_s=0.03), "event periods")
    qr_refuses("a coil hold shorter than two DIO scans is refused", qb(coil_hold_s=0.04), "coil_hold_s")
    qr_refuses("a direction dwell shorter than a DIO scan is refused", qb(dir_dwell_s=0.01), "dir_dwell_s")
    qr_refuses("profile position on analog drivers is refused",
               lambda d: d["pp"].update(enabled=True, vendor_ref="x",
                                        expect={k: 1 for k in d["pp"]["expect"]}), "pp.enabled must be false")
    qr_refuses("drive monitoring (BLV-R objects) is refused", lambda d: d["monitor"].update(enabled=True),
               "monitor.enabled")
    qr_refuses("a nonstandard IMU baud rate is refused", qb(imu_baud=12345), "imu_baud")
    qr_refuses("a gyro sign other than +-1 is refused", qb(imu_gyro_sign=0.5), "imu_gyro_sign")
    check("the real profile is restored afterwards", config.PROFILE_NAME == "agv-01")


TESTS = [test_qr_profile]
