#!/usr/bin/env python3
"""The IMU inside the SICK MLS: read it, measure its bias, put it on a TPDO.

Reads are safe and work in Pre-operational. The `tpdo` mode WRITES to the sensor
and refuses to run without --go, the same rule tune_mls.py follows.

WHY THIS MATTERS MORE THAN IT LOOKS
-----------------------------------
This IMU is the mitigation for a structural weakness in the vehicle's odometry.
The wheel encoders are on the MOTOR side of a 30:1 gearhead, so backlash and any
motor-to-wheel compliance are unobservable - odometry sees commanded shaft
motion, not achieved wheel motion, and the error lands as heading error on every
direction reversal. The standard fix is gyro-odometry: let the gyro own heading
and wheel odometry own translation only.

  see manuals/slam-generalized-plan/hardware-reconciliation.md, D-2 and D-3

*** So the gyro is a NAVIGATION component that happens to live inside the line
sensor. *** Retiring tape-following retires the MLS's primary feature but must
NOT retire the hardware, or the mitigation goes with it. That is a BOM note as
much as a code note.

WHAT IS MEASURED, AND WHAT IS ASSUMED
-------------------------------------
The scaling below comes from the MLS operating instructions. It is worth stating
that it is also self-consistent, because that is the cheapest evidence available
without a turntable:

    gyro    125 * 2**-11 deg/s per LSB  -> full scale +-2000 deg/s
    accel          2**-11 g per LSB     -> full scale +-16 g
    euler          1e-4 rad per LSB     -> +-3.2767 rad, and pi is 31416 counts,
                                           so +-pi just fits an int16
    stamp   1 ms, u16                   -> wraps at 65.536 s

Three standard full scales and an Euler range that exactly fills its word is not
what a mis-read table looks like.

The NOISE figures are measured on this unit, 661 samples, vehicle stationary,
2026-09-03:

    gyro z bias   +0.0574 deg/s   = 0.94 LSB   -> 3.4 deg/min heading drift
    gyro z noise   sigma 0.0291   = 0.48 LSB   -> BELOW one LSB
    gravity        0.9894 g                    -> 1.1 % scale error, uncalibrated

Noise below one LSB means the gyro is quantisation-limited, not noise-limited,
which is a better result than the generic plan assumed for a hobby-grade IMU.
The bias is the thing to correct, and `bias` mode below re-measures it - do that
rather than trusting the number above, which is one unit on one day.

*** Yaw rate only. *** The 3.4 deg/min drift makes ABSOLUTE yaw useless over a
shift, and the manual is explicit that yaw is a relative measurement. Over a
100 ms scan interval the same drift is 0.006 deg, which is nothing. Fuse the
RATE and let the lidar own absolute heading.

Deliberately free of `config`, like the rest of canbus/: every vehicle-specific
value is a parameter defaulting to a module constant.
"""
import argparse
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drive_forward import sdo_write  # noqa: E402
from verify_drivers import (BAD, OK, WARN, claim_bus, open_bus,  # noqa: E402
                            sdo_read)

SENSOR_NODE = 10

# Object dictionary. Sub-indices 1..3 are x, y, z throughout.
OBJ_ENABLE = (0x2006, 2)        # MEMS enable; IMU and thermometer share it
OBJ_LPF = (0x202D, 4)           # orientation low-pass filter
OBJ_EULER = 0x2030              # sub 1-3, roll/pitch/yaw
OBJ_QUAT = 0x2031               # sub 1-4, w/x/y/z
OBJ_YAW_RESET = (0x2032, 1)     # write to zero the yaw reference
OBJ_ACCEL = 0x2033              # sub 1-3
OBJ_GYRO = 0x2034               # sub 1-3, the one that matters
OBJ_STAMP = (0x2035, 0)         # milliseconds, u16, wraps at 65.536 s

# Scaling. See the docstring for why these are believed.
GYRO_LSB_DPS = 125.0 * 2 ** -11         # 0.06103515625 deg/s
ACCEL_LSB_G = 2.0 ** -11                # 0.00048828125 g
EULER_LSB_RAD = 1e-4
STAMP_WRAP = 1 << 16                    # u16 milliseconds

AXES = ("x", "y", "z")

# The IMU is NOT at base_link. It sits inside the sensor head, which is mounted
# at the front lookahead position, and the offset below is from the MLS LED edge.
#
# Yaw rate needs no correction for this - a rigid body rotates at the same rate
# about every point - but ANY accelerometer use needs lever-arm compensation,
# and imu_frame in the TF tree must be the real mounting point rather than a
# copy of base_link. That is the mistake this constant exists to prevent.
IMU_OFFSET_FROM_LED_MM = (89.1, 2.5, -9.4)

# TPDO slots, as probed on this sensor. The MLS allows at most FOUR TPDOs active
# at once, and only these COB-ID bases are valid: 0x180, 0x280, 0x380, 0x480,
# each plus the node id. TPDO1 (0x18A) carries track data and stays: line
# following is back as an engineering feature (read_mls.py, amr_base/mls_track.py).
#
#   1803h TPDO4  Euler         COB-ID 0x48A, DISABLED (raw 0x8000048A)
#   1804h TPDO5  quaternion    COB-ID 0x000, disabled
#   1805h TPDO6  acceleration  COB-ID 0x000, disabled
#   1806h TPDO7  yaw rate      COB-ID 0x000, disabled
TPDO_SLOTS = {
    "euler": (0x1803, "Euler angles"),
    "quaternion": (0x1804, "quaternion"),
    "acceleration": (0x1805, "acceleration"),
    "gyro": (0x1806, "yaw rate"),
}
VALID_COB_BASES = (0x180, 0x280, 0x380, 0x480)
MAX_ACTIVE_TPDOS = 4
# Bit 31 of the COB-ID entry is "this PDO does not exist". Enabling an already
# addressed PDO is therefore clearing one bit, not writing a new COB-ID.
COB_DISABLED = 1 << 31


def s16(v):
    """A raw 16-bit object as signed. The SDO layer hands back unsigned."""
    return v - 0x10000 if v is not None and v >= 0x8000 else v


def decode_gyro(raw):
    """Three raw words -> deg/s per axis. None passes through as None."""
    return tuple(None if v is None else s16(v) * GYRO_LSB_DPS for v in raw)


def decode_accel(raw):
    """Three raw words -> g per axis."""
    return tuple(None if v is None else s16(v) * ACCEL_LSB_G for v in raw)


def decode_euler(raw):
    """Three raw words -> radians per axis."""
    return tuple(None if v is None else s16(v) * EULER_LSB_RAD for v in raw)


def decode_quaternion(raw):
    """Four raw words -> a unit quaternion, or None if any component is missing.

    The manual does not state the scale, so it is DERIVED from the norm rather
    than assumed: a quaternion is a unit vector by definition, so dividing by its
    own magnitude is correct whatever the LSB turns out to be. A zero-norm
    reading means the IMU is off, not that the vehicle is at an odd attitude.
    """
    vals = [s16(v) for v in raw]
    if any(v is None for v in vals):
        return None
    norm = math.sqrt(sum(v * v for v in vals))
    if norm == 0:
        return None
    return tuple(v / norm for v in vals)


def stamp_delta_ms(new, old):
    """Milliseconds between two timestamps, wrap-safe.

    The counter is a u16 of milliseconds and wraps every 65.536 s, so a plain
    subtraction goes negative roughly once a minute - which, fed to a rate
    calculation, is an enormous transient in whatever consumes it. Nothing here
    can tell a wrap from a 65 s gap, which is why this is only meaningful when
    sampled far faster than the wrap period.
    """
    if new is None or old is None:
        return None
    return (new - old) % STAMP_WRAP


def gravity_error(accel_g):
    """How far the accelerometer's magnitude is from 1 g, as a fraction.

    A stationary IMU measures exactly gravity, so this is a free scale check
    that needs no reference equipment - it is where the recorded 0.9894 g
    (1.1 % low) came from. Only meaningful while the vehicle is STOPPED.
    """
    vals = [v for v in accel_g if v is not None]
    if len(vals) != 3:
        return None
    return math.sqrt(sum(v * v for v in vals)) - 1.0


def summarise_bias(samples_dps):
    """Bias, noise and what they mean, from stationary gyro samples.

    Returned rather than printed so a test can assert on it, and so a future
    calibration step can subtract the bias without re-deriving it.
    """
    if len(samples_dps) < 2:
        return None
    mean = statistics.fmean(samples_dps)
    sigma = statistics.stdev(samples_dps)
    return {
        "n": len(samples_dps),
        "bias_dps": mean,
        "sigma_dps": sigma,
        "bias_lsb": mean / GYRO_LSB_DPS,
        "sigma_lsb": sigma / GYRO_LSB_DPS,
        "drift_deg_min": mean * 60.0,
        # Below one LSB means the gyro is quantisation-limited rather than
        # noise-limited, which is a better sensor than the generic plan assumed.
        "quantisation_limited": sigma < GYRO_LSB_DPS,
    }


def cob_id_for(base, node):
    """A COB-ID the MLS will accept, or a ValueError naming the ones it will.

    The sensor accepts only four bases. Writing any other value is refused by
    the sensor, but late and unhelpfully, so it is refused here first.
    """
    if base not in VALID_COB_BASES:
        raise ValueError(
            f"{base:#05x} is not a COB-ID base this sensor accepts "
            f"({', '.join(f'{b:#05x}' for b in VALID_COB_BASES)})")
    return base + node


# ---- bus access -----------------------------------------------------------

def rd(bus, node, index, sub):
    st, val, _, _ = sdo_read(bus, node, index, sub)
    if not st or not val:
        return None
    return int.from_bytes(val, "little")


def read_triple(bus, node, index):
    return tuple(rd(bus, node, index, s) for s in (1, 2, 3))


def read_all(bus, node=SENSOR_NODE):
    """One snapshot of everything the IMU exposes. Read-only."""
    return {
        "enabled": rd(bus, node, *OBJ_ENABLE),
        "lpf": rd(bus, node, *OBJ_LPF),
        "stamp_ms": rd(bus, node, *OBJ_STAMP),
        "euler_rad": decode_euler(read_triple(bus, node, OBJ_EULER)),
        "gyro_dps": decode_gyro(read_triple(bus, node, OBJ_GYRO)),
        "accel_g": decode_accel(read_triple(bus, node, OBJ_ACCEL)),
        "quaternion": decode_quaternion(
            tuple(rd(bus, node, OBJ_QUAT, s) for s in (1, 2, 3, 4))),
    }


def read_tpdo_state(bus, node=SENSOR_NODE):
    """Which IMU TPDOs exist and which are disabled. Read-only."""
    out = {}
    for name, (index, label) in TPDO_SLOTS.items():
        raw = rd(bus, node, index, 1)
        out[name] = {
            "index": index, "label": label, "raw": raw,
            "cob_id": None if raw is None else raw & 0x7FF,
            "enabled": None if raw is None else not (raw & COB_DISABLED),
        }
    return out


# ---- modes ----------------------------------------------------------------

def _fmt3(vals, unit, scale=1.0, width=9, prec=4):
    return "  ".join(
        f"{a}={'--':>{width}}" if v is None else f"{a}={v * scale:{width}.{prec}f}"
        for a, v in zip(AXES, vals)) + f"  {unit}"


def show(bus, node=SENSOR_NODE):
    r = read_all(bus, node)
    if r["enabled"] == 0:
        print(f"{WARN} 2006h:02 MEMS = 0 - the IMU and the thermometer are OFF.")
        print("      Every reading below will be zero or stale. Enable it in")
        print("      the sensor configuration before measuring anything.")
    elif r["enabled"] is None:
        print(f"{BAD} the sensor did not answer 2006h:02 - is node "
              f"{node} on the bus and powered?")
        return 1

    print(f"    enabled (2006h:02)   {r['enabled']}")
    print(f"    orientation LPF      {r['lpf']}")
    print(f"    timestamp (2035h)    {r['stamp_ms']} ms"
          f"  (u16, wraps every {STAMP_WRAP / 1000:.3f} s)")
    print(f"    gyro   (2034h)  {_fmt3(r['gyro_dps'], 'deg/s')}")
    print(f"    accel  (2033h)  {_fmt3(r['accel_g'], 'g')}")
    print(f"    euler  (2030h)  {_fmt3(r['euler_rad'], 'rad')}")
    if r["quaternion"]:
        print("    quat   (2031h)  "
              + "  ".join(f"{a}={v:9.5f}"
                          for a, v in zip("wxyz", r["quaternion"])))

    g_err = gravity_error(r["accel_g"])
    if g_err is not None:
        mark = OK if abs(g_err) < 0.05 else WARN
        print(f"\n    {mark} |accel| is {1 + g_err:.4f} g "
              f"({g_err * 100:+.1f} % vs 1 g) - a free scale check, valid only "
              f"while STOPPED")

    print("\n    TPDO slots (at most "
          f"{MAX_ACTIVE_TPDOS} active at once on this sensor):")
    for name, st in read_tpdo_state(bus, node).items():
        if st["raw"] is None:
            print(f"      {st['index']:04X}h {st['label']:<14} no reply")
            continue
        state = "ENABLED " if st["enabled"] else "disabled"
        print(f"      {st['index']:04X}h {st['label']:<14} "
              f"COB-ID 0x{st['cob_id']:03X}  {state}  (raw 0x{st['raw']:08X})")
    return 0


def measure_bias(bus, node=SENSOR_NODE, seconds=30.0, axis=2):
    """Average the gyro while the vehicle is stationary. Read-only.

    *** The vehicle must be STOPPED and undisturbed. *** Nothing here can tell a
    genuine bias from somebody leaning on the chassis, so this refuses to report
    a number it cannot stand behind: a sample spread far above the recorded
    noise floor means something moved, and that is reported instead of averaged.
    """
    print(f"    sampling gyro {AXES[axis]} for {seconds:.0f} s - "
          f"DO NOT DISTURB THE VEHICLE")
    samples, end = [], time.time() + seconds
    while time.time() < end:
        raw = rd(bus, node, OBJ_GYRO, axis + 1)
        if raw is not None:
            samples.append(s16(raw) * GYRO_LSB_DPS)
    stats = summarise_bias(samples)
    if stats is None:
        print(f"{BAD} only {len(samples)} samples - the sensor is not answering.")
        return 1

    print(f"\n    n              {stats['n']}")
    print(f"    bias           {stats['bias_dps']:+.4f} deg/s "
          f"({stats['bias_lsb']:+.2f} LSB)")
    print(f"    sigma          {stats['sigma_dps']:.4f} deg/s "
          f"({stats['sigma_lsb']:.2f} LSB)")
    print(f"    heading drift  {stats['drift_deg_min']:+.2f} deg/min")
    if stats["quantisation_limited"]:
        print(f"\n    {OK} sigma is below one LSB ({GYRO_LSB_DPS:.4f} deg/s) - "
              f"quantisation-limited,")
        print("      which is the best this sensor can report and better than "
              "the plan assumed.")
    else:
        print(f"\n    {WARN} sigma is above one LSB. Either the vehicle moved "
              f"during the run, or")
        print("      this unit is noisier than the one measured on 2026-09-03 "
              "(sigma 0.0291).")
    print(f"\n    Subtract the bias in the EKF; do NOT use absolute yaw - "
          f"{stats['drift_deg_min']:+.2f} deg/min")
    print("    makes it useless over a shift. See the module docstring.")
    return 0


def enable_tpdo(bus, node, slot, base, go=False):
    """Put one IMU quantity on a TPDO. WRITES - needs go=True.

    Only the COB-ID entry is touched. The mapping is whatever the sensor ships
    with for that slot, which is the point of these being fixed-purpose slots
    rather than freely mappable ones.
    """
    index, label = TPDO_SLOTS[slot]
    cob = cob_id_for(base, node)

    state = read_tpdo_state(bus, node)
    active = [n for n, s in state.items() if s["enabled"]]
    if slot not in active and len(active) >= MAX_ACTIVE_TPDOS:
        print(f"{BAD} {len(active)} TPDOs are already active "
              f"({', '.join(active)}) and this sensor allows "
              f"{MAX_ACTIVE_TPDOS}.")
        print("      Disable one before enabling another.")
        return 1

    clash = [n for n, s in state.items()
             if n != slot and s["enabled"] and s["cob_id"] == cob]
    if clash:
        print(f"{BAD} COB-ID 0x{cob:03X} is already used by {clash[0]}. Two "
              f"PDOs on one COB-ID")
        print("      is two decoders reading each other's frames.")
        return 1

    print(f"    {index:04X}h {label:<14} -> COB-ID 0x{cob:03X} (0x{cob:08X})")
    if not go:
        print(f"\n{WARN} this writes to the sensor. Re-run with --go. "
              f"Nothing has been changed.")
        return 2
    ok, detail = sdo_write(bus, node, index, 1, cob, 4)
    if not ok:
        print(f"{BAD} write refused: {detail}")
        return 1
    print(f"    {OK} enabled. The sensor must be RESTARTED for this to take "
          f"effect,")
    print("      and stored if it is to survive a power cycle.")
    return 0


# ---- CLI ------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Read the IMU inside the SICK MLS, measure its bias, or "
                    "put a quantity on a TPDO.",
        epilog="Reads are safe. `tpdo` writes and needs --go.")
    ap.add_argument("--node", type=int, default=SENSOR_NODE)
    sub = ap.add_subparsers(dest="mode", required=True)

    sub.add_parser("show", help="read-only: one snapshot of everything")
    b = sub.add_parser("bias", help="read-only: measure gyro bias, stationary")
    b.add_argument("--seconds", type=float, default=30.0)
    b.add_argument("--axis", type=int, default=2, choices=(0, 1, 2),
                   help="0=x 1=y 2=z; z is the one heading needs")
    t = sub.add_parser("tpdo", help="put a quantity on a TPDO (writes)")
    t.add_argument("slot", choices=sorted(TPDO_SLOTS))
    t.add_argument("--cob-base", type=lambda s: int(s, 0), default=0x280,
                   help="0x180 / 0x280 / 0x380 / 0x480 (default 0x280)")
    t.add_argument("--go", action="store_true", help="actually write")

    args = ap.parse_args(argv)

    if args.mode == "tpdo":
        try:
            cob_id_for(args.cob_base, args.node)
        except ValueError as e:
            print(f"{BAD}: {e}")
            return 2
        if not args.go:
            print(f"{WARN} the control service must not be running - it owns "
                  f"can0.\n")

    lock = claim_bus("read_imu")
    if lock is None:
        return 2
    try:
        bus, how = open_bus()
    except Exception as e:                      # noqa: BLE001 - a bench tool
        print(f"{BAD}: {e}")
        return 2
    print(f"connected via {how}\n")
    try:
        if args.mode == "show":
            return show(bus, args.node)
        if args.mode == "bias":
            return measure_bias(bus, args.node, args.seconds, args.axis)
        if args.mode == "tpdo":
            return enable_tpdo(bus, args.node, args.slot, args.cob_base,
                               args.go)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
