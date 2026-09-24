"""The MLS IMU: scaling, wrap-safe timestamps, and the TPDO slot rules.

The scaling constants are the interesting part, and they are asserted two ways:
against the manual's stated LSBs, and against the FULL SCALES those LSBs imply.
The second is what catches a mis-read table - a wrong LSB gives a full scale that
is not a number any IMU ships with, whereas the LSB alone looks plausible.

The measured noise figures from 2026-09-03 are used as fixtures rather than as
constants in the module: they describe one unit on one day, and the point of
`bias` mode is to re-measure them.
"""
import math

from helpers import check

import read_imu as imu


def test_scaling():
    print("\nMLS IMU: scaling constants and their implied full scales")

    check("gyro LSB is 125 * 2**-11 deg/s",
          abs(imu.GYRO_LSB_DPS - 0.06103515625) < 1e-12,
          f"{imu.GYRO_LSB_DPS}")
    check("...which implies a standard +-2000 deg/s full scale",
          round(32767 * imu.GYRO_LSB_DPS) == 2000,
          f"{32767 * imu.GYRO_LSB_DPS:.1f}")
    check("accel LSB is 2**-11 g",
          abs(imu.ACCEL_LSB_G - 1 / 2048) < 1e-12)
    check("...which implies a standard +-16 g full scale",
          round(32767 * imu.ACCEL_LSB_G, 1) == 16.0)
    check("euler LSB is 1e-4 rad", imu.EULER_LSB_RAD == 1e-4)
    check("...and +-pi rad just fits an int16 at that LSB",
          31416 < 32767 and round(math.pi / imu.EULER_LSB_RAD) == 31416)

    # s16 is the whole reason a negative rate reads as negative. The SDO layer
    # returns unsigned words, so without it a clockwise turn reads as ~2000 dps.
    check("a raw word above 0x8000 is negative", imu.s16(0xFFFF) == -1)
    check("a raw word below 0x8000 is positive", imu.s16(0x7FFF) == 32767)
    check("s16 passes None through", imu.s16(None) is None)

    check("a full-scale negative rate decodes to -2000 deg/s",
          abs(imu.decode_gyro((0x8000,))[0] + 2000.0) < 0.1,
          f"{imu.decode_gyro((0x8000,))[0]:.2f}")
    check("the measured +0.0574 deg/s bias is about one LSB",
          abs(0.0574 / imu.GYRO_LSB_DPS - 0.94) < 0.02)

    g = imu.decode_accel((0, 0, 2026))
    check("a stationary z-axis reading decodes to about 1 g",
          abs(g[2] - 0.989) < 0.002, f"{g[2]:.4f} g")
    check("an unanswered axis stays None rather than becoming zero",
          imu.decode_gyro((None, 5, None)) == (None, 5 * imu.GYRO_LSB_DPS, None))


def test_quaternion():
    print("\nMLS IMU: the quaternion normalises itself")

    # The manual does not state the quaternion's LSB, so it is derived from the
    # norm - which is correct whatever the LSB is, because a quaternion is a
    # unit vector by definition.
    q = imu.decode_quaternion((10000, 0, 0, 0))
    check("a w-only quaternion normalises to identity",
          q == (1.0, 0.0, 0.0, 0.0), str(q))
    q = imu.decode_quaternion((1000, 1000, 1000, 1000))
    check("any scale normalises to unit magnitude",
          abs(math.sqrt(sum(v * v for v in q)) - 1.0) < 1e-9)
    check("a zero quaternion is None, not a divide by zero",
          imu.decode_quaternion((0, 0, 0, 0)) is None)
    check("a missing component yields None rather than a partial rotation",
          imu.decode_quaternion((1, 2, None, 4)) is None)


def test_timestamp_wrap():
    print("\nMLS IMU: the timestamp wraps every 65.536 s")

    check("the counter is a u16 of milliseconds",
          imu.STAMP_WRAP == 65536 and imu.STAMP_WRAP / 1000 == 65.536)
    check("an ordinary delta is a subtraction",
          imu.stamp_delta_ms(1500, 1480) == 20)
    # *** This is the one that matters. *** A plain subtraction goes negative
    # about once a minute, and a negative dt in a rate calculation is an
    # enormous transient in whatever consumes it.
    check("a delta ACROSS the wrap stays positive",
          imu.stamp_delta_ms(10, 65530) == 16,
          str(imu.stamp_delta_ms(10, 65530)))
    check("a wrap of exactly one period reads as zero, not as 65536",
          imu.stamp_delta_ms(100, 100) == 0)
    check("a missing stamp yields None rather than a bogus delta",
          imu.stamp_delta_ms(None, 5) is None
          and imu.stamp_delta_ms(5, None) is None)


def test_gravity_check():
    print("\nMLS IMU: gravity as a free scale check")

    check("a perfect 1 g reads as zero error",
          abs(imu.gravity_error((0.0, 0.0, 1.0))) < 1e-9)
    check("the recorded 0.9894 g reads as 1.1 % low",
          abs(imu.gravity_error((0.0, 0.0, 0.9894)) + 0.0106) < 1e-6,
          f"{imu.gravity_error((0.0, 0.0, 0.9894)):+.4f}")
    check("the magnitude is used, not one axis - tilt does not matter",
          abs(imu.gravity_error((0.6, 0.0, 0.8))) < 1e-9)
    check("an incomplete reading yields None",
          imu.gravity_error((0.0, None, 1.0)) is None)


def test_bias_summary():
    print("\nMLS IMU: what a bias run concludes")

    lsb = imu.GYRO_LSB_DPS
    # Reproduce the 2026-09-03 result: bias +0.0574, sigma 0.0291 - sigma below
    # one LSB, so the sensor is quantisation-limited rather than noise-limited.
    quantised = [0.0574 + (lsb if i % 2 else -lsb) * 0.48 for i in range(200)]
    s = imu.summarise_bias(quantised)
    check("the bias is recovered", abs(s["bias_dps"] - 0.0574) < 1e-6,
          f"{s['bias_dps']:+.4f}")
    check("drift is reported per minute, matching the recorded 3.4 deg/min",
          abs(s["drift_deg_min"] - 3.44) < 0.02, f"{s['drift_deg_min']:.2f}")
    check("sigma below one LSB is reported as quantisation-limited",
          s["quantisation_limited"], f"sigma {s['sigma_dps']:.4f}")

    noisy = [0.0574 + (5 * lsb if i % 2 else -5 * lsb) for i in range(200)]
    s = imu.summarise_bias(noisy)
    check("a noisy run is NOT reported as quantisation-limited",
          not s["quantisation_limited"], f"sigma {s['sigma_dps']:.4f}")
    check("the bias survives the extra noise",
          abs(s["bias_dps"] - 0.0574) < 1e-6)
    check("one sample cannot produce a sigma, so it produces nothing",
          imu.summarise_bias([0.05]) is None)
    check("no samples yield None rather than a divide by zero",
          imu.summarise_bias([]) is None)


def test_tpdo_rules():
    print("\nMLS IMU: TPDO slot and COB-ID rules")

    check("the sensor's four fixed-purpose slots are named",
          set(imu.TPDO_SLOTS) == {"euler", "quaternion", "acceleration",
                                  "gyro"})
    check("yaw rate is 1806h, the slot heading actually needs",
          imu.TPDO_SLOTS["gyro"][0] == 0x1806)
    check("Euler is 1803h, the slot already carrying a COB-ID",
          imu.TPDO_SLOTS["euler"][0] == 0x1803)

    # Only four bases are valid. Writing any other value is refused by the
    # sensor late and unhelpfully, so it is refused here first.
    check("a valid base yields base + node",
          imu.cob_id_for(0x280, 10) == 0x28A)
    check("the four valid bases all resolve",
          [imu.cob_id_for(b, 10) for b in imu.VALID_COB_BASES]
          == [0x18A, 0x28A, 0x38A, 0x48A])
    raised = False
    try:
        imu.cob_id_for(0x200, 10)
    except ValueError:
        raised = True
    check("an invalid COB-ID base raises before anything is written", raised)

    # The probed Euler slot is 0x8000048A: addressed, but disabled by bit 31.
    # Enabling it is therefore clearing one bit, not writing a new COB-ID -
    # which is exactly what the reconciliation document says.
    check("bit 31 is the PDO-does-not-exist bit",
          imu.COB_DISABLED == 0x80000000)
    check("the probed Euler entry decodes as addressed-but-disabled",
          (0x8000048A & 0x7FF) == 0x48A and bool(0x8000048A & imu.COB_DISABLED))
    check("clearing bit 31 leaves the COB-ID untouched",
          (0x8000048A & ~imu.COB_DISABLED) == 0x0000048A)
    check("this sensor allows at most four active TPDOs",
          imu.MAX_ACTIVE_TPDOS == 4)


def test_lever_arm_recorded():
    print("\nMLS IMU: the mounting offset is recorded, not assumed")

    # Yaw rate needs no lever-arm correction - a rigid body rotates at the same
    # rate about every point - but accelerometer use does, and imu_frame must be
    # the real mounting point rather than a copy of base_link.
    check("the offset from the MLS LED edge is carried in the module",
          imu.IMU_OFFSET_FROM_LED_MM == (89.1, 2.5, -9.4))
    check("the module says yaw rate needs no correction but accel does",
          "lever-arm" in imu.__doc__ or "lever-arm" in
          open(imu.__file__, encoding="utf-8").read())
    check("the module refuses to import the vehicle profile",
          "import config" not in open(imu.__file__, encoding="utf-8").read())


TESTS = [test_scaling, test_quaternion, test_timestamp_wrap,
         test_gravity_check, test_bias_summary, test_tpdo_rules,
         test_lever_arm_recorded]
