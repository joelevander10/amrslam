"""qr_calibrate: AMR QR bench tool - read the sensors, and measure the wheel feedforward.

    ros2 run amr_base qr_calibrate io                 DI/DO image, prints on change (read-only)
    ros2 run amr_base qr_calibrate enc                encoder counts, wheel speed, source (read-only)
    ros2 run amr_base qr_calibrate imu                WitMotion frame types, rates, yaw rate (read-only)
    ros2 run amr_base qr_calibrate imu-setup --rate 50 --baud 115200 --go
                                                      configure the WitMotion unit: output rate,
                                                      content (acc+gyro+angle) and baud, saved in it
    ros2 run amr_base qr_calibrate brake-off          0 V, FWD/REV low, brakes released (vehicle free)
    ros2 run amr_base qr_calibrate breakaway --wheel left --go
                                                      ramp 0 -> v until the wheel turns: ff_offset_v
    ros2 run amr_base qr_calibrate ff --wheel both --volts 0.8,1.2,1.6,2.0 --go
                                                      steady speed per voltage, least-squares line:
                                                      ff_motor_rpm_per_volt and ff_offset_v

No ROS node, no mux, no panel authority: the motion commands here drive the coils and the
analog outputs DIRECTLY. That is why they need --go, a typed confirmation that the drive
wheels are OFF THE GROUND, the E-stop released (read from the DI image and re-checked
every sample), and why they refuse to run beside qr_base_node (the same owner locks).
Every exit path - normal, Ctrl-C, exception - writes 0 V, drops FWD/REV, brakes until
the wheels stop and then RELEASES the brakes, so the vehicle can be pushed afterwards.
`qr_calibrate brake-off` does the same on its own (e.g. after a crash left a brake on).

Voltages are capped at qr_base.v_max_v from the profile, forward direction only by default
(--reverse for the other). Uses AGV_PROFILE like every node.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import amr_base.agv_repo  # noqa: F401
from amr_base import canopen, wit_imu
from amr_base.agv_repo import config
from amr_base.qr_analog import AnalogOut
from amr_base.qr_encoders import EncoderPair
from amr_base.wit_imu import ACC, ANGLE, GYRO, WitParser, YawRateSource

import ownerlock  # noqa: E402
from verify_drivers import open_bus  # noqa: E402

SIDES = ("left", "right")


def _need_qr() -> None:
    if config.PLATFORM != config.PLATFORM_QR:
        sys.exit(f"profile {config.PROFILE_NAME} is platform {config.PLATFORM}; set AGV_PROFILE=amr-qr-01")


def _dio_client():
    from pymodbus.client import ModbusTcpClient  # noqa: PLC0415

    c = ModbusTcpClient(config.DIO_IP, port=config.DIO_PORT, timeout=0.2)
    if not c.connect():
        sys.exit(f"no DIO module at {config.DIO_IP}:{config.DIO_PORT}")
    return c


def _read_bits(c, fn, base, count):
    r = fn(base, count=count, device_id=config.DIO_DEVICE_ID)
    if r.isError():
        raise OSError(str(r))
    bits = [bool(b) for b in r.bits[:count]]
    return [not b for b in bits] if (fn == c.read_discrete_inputs and config.DIO_DI_FLIPPED) else bits


def _estop_active(di: list[bool]) -> bool:
    raw = di[config.QR_DI_ESTOP]
    return (not raw) if config.QR_ESTOP_ACTIVE_LOW else raw


# ------------------------------------------------------------------ read-only


def cmd_io(_args) -> int:
    c = _dio_client()
    last = None
    print(f"DIO {config.DIO_IP}: printing on change, Ctrl-C to stop")
    try:
        while True:
            di = _read_bits(c, c.read_discrete_inputs, config.DIO_DI_BASE, config.DIO_NUM_DI)
            do = _read_bits(c, c.read_coils, config.DIO_DO_BASE, config.DIO_NUM_DO)
            if (di, do) != last:
                last = (di, do)
                on_di = [f"DI{i:02d} {config.DIO_DI_NAMES[i]}" for i, b in enumerate(di) if b]
                on_do = [f"DO{i:02d} {config.DIO_DO_NAMES[i]}" for i, b in enumerate(do) if b]
                print(f"{time.strftime('%H:%M:%S')}  E-STOP {'ACTIVE' if _estop_active(di) else 'released'}")
                print("   DI high: " + (", ".join(on_di) or "-"))
                print("   DO high: " + (", ".join(on_do) or "-"))
            time.sleep(0.05)
    except KeyboardInterrupt:
        return 0
    finally:
        c.close()


def _encoders():
    raw, how = open_bus(config.CAN_BITRATE, config.CAN_CHANNEL, config.CAN_ADAPTER_SERIAL)
    router = canopen.Router(raw)
    encs = EncoderPair(
        router,
        config.LEFT,
        config.RIGHT,
        config.QR_ENC_COUNTS_PER_REV,
        config.QR_ENC_RANGE_COUNTS,
        config.QR_ENC_INVERT_LEFT,
        config.QR_ENC_INVERT_RIGHT,
        config.QR_ENC_MODE,
        config.QR_ENC_EVENT_MS,
    )
    encs.start()
    print(f"encoders on {how}")
    return router, encs


def _pump(router, encs, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        router.pump(0.004)
        encs.poll()


def cmd_enc(_args) -> int:
    router, encs = _encoders()
    print("push the vehicle FORWARD by hand (or turn each wheel forward): both speeds must read POSITIVE.")
    print("A negative one -> flip qr_base.enc_invert_<side>.  Ctrl-C to stop.")
    try:
        while True:
            _pump(router, encs, 0.2)
            cols = []
            for name, e in zip(SIDES, encs.wheels, strict=True):
                sp = e.rad_s
                speed = "" if sp is None else f"{sp:+7.3f} rad/s {sp * config.WHEEL_DIA_M / 2:+6.3f} m/s"
                cols.append(f"{name}: raw {e.raw!s:>9} counts {e.counts!s:>9} {speed} [{e.source}]")
            print("   ".join(cols))
    except KeyboardInterrupt:
        return 0
    finally:
        router.shutdown()


def cmd_imu(_args) -> int:
    import serial  # noqa: PLC0415

    port = serial.Serial(config.QR_IMU_PORT, config.QR_IMU_BAUD, timeout=0.05)
    parser, src = WitParser(), YawRateSource()
    last_wz = None
    counts = {ACC: 0, GYRO: 0, ANGLE: 0}
    t0 = time.monotonic()
    print(
        f"IMU {config.QR_IMU_PORT} @ {config.QR_IMU_BAUD}. Spin the vehicle CCW (left): wz must go POSITIVE."
    )
    try:
        while True:
            data = port.read(max(1, port.in_waiting))
            now = time.monotonic()
            for f in parser.feed(data, now):
                counts[f.kind] = counts.get(f.kind, 0) + 1
                s = src.offer(f)
                if s is not None:
                    last_wz = s.wz_rad_s
            if now - t0 >= 1.0:
                dt, t0 = now - t0, now
                rates = "  ".join(f"0x{k:02X}:{v / dt:5.1f} Hz" for k, v in sorted(counts.items()))
                yaw = "-" if src.yaw_deg is None else f"{src.yaw_deg:+7.2f} deg"
                wz = "-" if last_wz is None else f"{math.degrees(last_wz):+7.2f} deg/s"
                print(
                    f"{rates}   mode {src.mode:10s} wz {wz}  yaw {yaw}   bad checksum {parser.bad_checksum}"
                )
                counts = {k: 0 for k in counts}
    except KeyboardInterrupt:
        return 0
    finally:
        port.close()


def _imu_rates(port, seconds: float = 2.0) -> dict[int, float]:
    """Frames per second by type over `seconds` on an open port."""
    parser, counts = WitParser(), {}
    port.reset_input_buffer()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        for f in parser.feed(port.read(max(1, port.in_waiting)), time.monotonic()):
            counts[f.kind] = counts.get(f.kind, 0) + 1
    dt = time.monotonic() - t0
    return {k: v / dt for k, v in counts.items()}


def _fmt_rates(r: dict[int, float]) -> str:
    return "  ".join(f"0x{k:02X}:{v:5.1f} Hz" for k, v in sorted(r.items())) or "no frames"


def cmd_imu_setup(args) -> int:
    """Write output rate, content and baud to the WitMotion unit and save them in it.

    Nothing moves; the change persists in the IMU. Sequence (WitMotion standard protocol,
    ~0.1 s between frames): at the CURRENT baud unlock, content, rate, save; then, if the
    baud changes, unlock + baud, reopen at the new baud, unlock + save. Verified by
    counting frames at the new settings; a unit that ignored the commands is reported,
    not assumed.
    """
    import serial  # noqa: PLC0415

    if not args.go:
        sys.exit("refusing to reconfigure the IMU without --go")
    if args.rate not in wit_imu.RATE_CODES:
        sys.exit(f"--rate must be one of {sorted(wit_imu.RATE_CODES)}")
    if args.baud not in wit_imu.BAUD_CODES:
        sys.exit(f"--baud must be one of {sorted(wit_imu.BAUD_CODES)}")
    need = wit_imu.frame_bytes_per_s(args.rate)
    if need > 0.8 * args.baud:
        sys.exit(f"{args.rate} Hz x 3 frames needs {need} bit/s - too much for {args.baud} baud")

    dev = config.QR_IMU_PORT
    cur = None
    for baud in [config.QR_IMU_BAUD] + [b for b in (9600, 115200) if b != config.QR_IMU_BAUD]:
        with serial.Serial(dev, baud, timeout=0.05) as p:
            r = _imu_rates(p, 1.5)
        print(f"  {dev} @ {baud}: {_fmt_rates(r)}")
        if r.get(GYRO) or r.get(ANGLE):
            cur = baud
            break
    if cur is None:
        sys.exit("no WitMotion frames at 9600 or 115200 - check the cable, or use the vendor tool")

    def send(p, frame: bytes, what: str) -> None:
        p.write(frame)
        p.flush()
        print(f"  -> {frame.hex(' ').upper():15s} {what}")
        time.sleep(0.15)

    content = wit_imu.RSW_ACC | wit_imu.RSW_GYRO | wit_imu.RSW_ANGLE
    with serial.Serial(dev, cur, timeout=0.05) as p:
        send(p, wit_imu.UNLOCK, "unlock")
        send(p, wit_imu.command(wit_imu.REG_RSW, content), "content: acc + gyro + angle")
        send(p, wit_imu.UNLOCK, "unlock")
        send(p, wit_imu.command(wit_imu.REG_RRATE, wit_imu.RATE_CODES[args.rate]), f"rate {args.rate} Hz")
        send(p, wit_imu.SAVE, "save")
        if args.baud != cur:
            send(p, wit_imu.UNLOCK, "unlock")
            send(p, wit_imu.command(wit_imu.REG_BAUD, wit_imu.BAUD_CODES[args.baud]), f"baud {args.baud}")
    if args.baud != cur:
        time.sleep(0.3)
        with serial.Serial(dev, args.baud, timeout=0.05) as p:
            send(p, wit_imu.UNLOCK, "unlock (new baud)")
            send(p, wit_imu.SAVE, "save (new baud)")
    time.sleep(0.5)

    with serial.Serial(dev, args.baud, timeout=0.05) as p:
        r = _imu_rates(p, 2.0)
    print(f"\nnow {dev} @ {args.baud}: {_fmt_rates(r)}")
    got = r.get(GYRO, 0.0)
    if got >= 0.8 * args.rate:
        print(f'OK. Set in profiles/amr-qr-01.json:  "imu_baud": {args.baud}')
        print("Then power-cycle the IMU once and run `qr_calibrate imu` to confirm the setting was saved.")
        return 0
    with serial.Serial(dev, cur, timeout=0.05) as p:
        back = _imu_rates(p, 1.5)
    print(f"still at {cur}? {_fmt_rates(back)}")
    print(
        "The unit did not take the settings. Its firmware may use another command set: "
        "configure it with the WitMotion PC software (rate, content, baud, save) instead."
    )
    return 1


def cmd_brake_off(_args) -> int:
    """All motor outputs to their free state: 0 V, FWD/REV low, brakes released."""
    dio = _dio_client()
    ao = AnalogOut(
        config.QR_AO_IP,
        config.QR_AO_PORT,
        config.QR_AO_DEVICE_ID,
        0.2,
        config.QR_AO_REGISTER_BASE,
        config.QR_AO_COUNTS_PER_VOLT,
        config.QR_AO_FULL_SCALE_V,
        config.QR_AO_CH_LEFT,
        config.QR_AO_CH_RIGHT,
    )
    ok = ao.write(0.0, 0.0)
    print(f"  analog 0 V: {'ok' if ok else 'FAILED ' + ao.detail}")
    chans = [
        (config.QR_DO_LEFT_FWD, "left FWD"),
        (config.QR_DO_LEFT_REV, "left REV"),
        (config.QR_DO_RIGHT_FWD, "right FWD"),
        (config.QR_DO_RIGHT_REV, "right REV"),
        (config.QR_DO_LEFT_BRK, "left BRAKE"),
        (config.QR_DO_RIGHT_BRK, "right BRAKE"),
    ]
    for ch, name in chans:  # direction coils first, brakes last
        r = dio.write_coil(config.DIO_DO_BASE + ch, False, device_id=config.DIO_DEVICE_ID)
        ok = ok and not r.isError()
        print(f"  DO{ch:02d} {name:11s} -> off {'' if not r.isError() else 'FAILED ' + str(r)}")
    ao.close()
    dio.close()
    return 0 if ok else 1


# ------------------------------------------------------------------ motion (on blocks)


class Rig:
    """Direct outputs for one or both wheels, with the E-stop re-checked on every sample."""

    def __init__(self, sides: tuple[str, ...], reverse: bool) -> None:
        self.sides = sides
        self.dio = _dio_client()
        self.ao = AnalogOut(
            config.QR_AO_IP,
            config.QR_AO_PORT,
            config.QR_AO_DEVICE_ID,
            0.2,
            config.QR_AO_REGISTER_BASE,
            config.QR_AO_COUNTS_PER_VOLT,
            config.QR_AO_FULL_SCALE_V,
            config.QR_AO_CH_LEFT,
            config.QR_AO_CH_RIGHT,
        )
        self.router, self.encs = _encoders()
        self.coils = {
            "left": (config.QR_DO_LEFT_FWD, config.QR_DO_LEFT_REV, config.QR_DO_LEFT_BRK),
            "right": (config.QR_DO_RIGHT_FWD, config.QR_DO_RIGHT_REV, config.QR_DO_RIGHT_BRK),
        }
        # vehicle-forward per wheel, through the profile's invert (right is mirrored on the QR)
        self.drv_dir = {
            "left": (-1 if config.INVERT_LEFT else 1) * (-1 if reverse else 1),
            "right": (-1 if config.INVERT_RIGHT else 1) * (-1 if reverse else 1),
        }
        self.reverse = reverse

    def _coil(self, ch: int, val: bool) -> None:
        r = self.dio.write_coil(config.DIO_DO_BASE + ch, val, device_id=config.DIO_DEVICE_ID)
        if r.isError():
            raise OSError(f"coil {ch}: {r}")

    def check_estop(self) -> None:
        di = _read_bits(self.dio, self.dio.read_discrete_inputs, config.DIO_DI_BASE, config.DIO_NUM_DI)
        if _estop_active(di):
            raise RuntimeError("E-STOP active - aborted")

    def stop(self, release: bool = True, settle_s: float = 1.5) -> None:
        """0 V, FWD/REV low, brake ON until the wheels are still - then brake OFF.

        The coils LATCH in the module: whatever this leaves is what the vehicle keeps after
        the tool exits. Braking stops the wheels quickly; releasing afterwards leaves the
        vehicle free to push, as the QR controller's shutdown_system() and qr_base_node's
        clean exit do. release=False keeps the brake on (not used by any command).
        """
        self._safe(lambda: self.ao.write(0.0, 0.0))
        for side in SIDES:
            fwd, rev, brk = self.coils[side]
            self._safe(lambda f=fwd: self._coil(f, False))
            self._safe(lambda r=rev: self._coil(r, False))
            self._safe(lambda b=brk: self._coil(b, True))
        if not release:
            return
        end = time.monotonic() + settle_s
        try:
            while time.monotonic() < end:  # wait for rest (or the time limit), then free them
                try:
                    _pump(self.router, self.encs, 0.1)
                    still = all(
                        e.rad_s is not None and abs(e.rad_s) < config.QR_ZERO_RAD_S for e in self.encs.wheels
                    )
                except Exception:  # noqa: BLE001 - no encoders: just wait the time out
                    still = False
                if still:
                    break
        except KeyboardInterrupt:  # a second Ctrl-C must not leave the brakes latched on
            pass
        for side in SIDES:
            _fwd, _rev, brk = self.coils[side]
            self._safe(lambda b=brk: self._coil(b, False))
        print("  outputs off: 0 V, FWD/REV low, brakes released (the vehicle can be pushed)")

    @staticmethod
    def _safe(fn) -> bool:
        try:
            fn()
            return True
        except Exception as e:  # noqa: BLE001
            print(f"  (stop step failed: {e})")
            return False

    def engage(self) -> None:
        """Brake off, direction on, 0 V - for the selected wheels only."""
        self.ao.write(0.0, 0.0)
        for side in self.sides:
            fwd, rev, brk = self.coils[side]
            self._coil(fwd, False)
            self._coil(rev, False)
        time.sleep(max(0.1, config.QR_DIR_DWELL_S))
        for side in self.sides:
            fwd, rev, brk = self.coils[side]
            self._coil(brk, False)
            self._coil(fwd if self.drv_dir[side] > 0 else rev, True)

    def volts(self, v: float) -> None:
        v = min(v, config.QR_V_MAX_V)
        vl = v if "left" in self.sides else 0.0
        vr = v if "right" in self.sides else 0.0
        if not self.ao.write(vl, vr):
            raise OSError(f"analog write failed: {self.ao.detail}")

    def speed(self, seconds: float) -> dict[str, float | None]:
        """Mean wheel speed (vehicle terms, rad/s) over `seconds`, E-stop checked every 0.1 s."""
        start = {}
        _pump(self.router, self.encs, 0.05)
        for side, e in zip(SIDES, self.encs.wheels, strict=True):
            start[side] = (e.t, e.counts)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check_estop()
            _pump(self.router, self.encs, 0.1)
        out = {}
        for side, e in zip(SIDES, self.encs.wheels, strict=True):
            t0, c0 = start[side]
            if None in (t0, c0, e.t, e.counts) or e.t <= t0:
                out[side] = None
            else:
                out[side] = (e.counts - c0) / (e.t - t0) * 2 * math.pi / config.QR_ENC_COUNTS_PER_REV
        return out

    def close(self) -> None:
        self.stop()
        self._safe(self.dio.close)
        self._safe(self.ao.close)
        self._safe(self.router.shutdown)


def _confirm(args) -> None:
    if not args.go:
        sys.exit("refusing to move anything without --go")
    print("\n*** This drives the wheels directly, bypassing the ROS stack. ***")
    print("    The drive wheels must be OFF THE GROUND (vehicle on blocks), the area clear,")
    print("    and a hand on the E-stop.  Type YES to continue: ", end="", flush=True)
    if sys.stdin.readline().strip() != "YES":
        sys.exit("not confirmed")


def _sides(arg: str) -> tuple[str, ...]:
    return SIDES if arg == "both" else (arg,)


def _to_motor_rpm(w: float) -> float:
    return abs(w) * config.GEAR_RATIO * 60.0 / (2 * math.pi)


def cmd_breakaway(args) -> int:
    _need_qr()
    _confirm(args)
    rig = Rig(_sides(args.wheel), args.reverse)
    found: dict[str, float] = {}
    try:
        rig.check_estop()
        rig.engage()
        v = 0.0
        while v <= min(args.max_v, config.QR_V_MAX_V) + 1e-9 and len(found) < len(rig.sides):
            rig.volts(v)
            sp = rig.speed(args.step_s)
            for side in rig.sides:
                w = sp.get(side)
                if side not in found and w is not None and abs(w) > config.QR_ZERO_RAD_S * 2:
                    found[side] = v
                    print(f"  {side}: turns at {v:.2f} V ({w:+.3f} rad/s)")
            print(f"  {v:.2f} V  " + "  ".join(f"{s}: {sp.get(s)}" for s in rig.sides))
            v += args.step_v
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        rig.close()
    for side in rig.sides:
        print(f"{side}: breakaway {found.get(side, 'not reached')}")
    if found:
        print(f"suggested qr_base.ff_offset_v ~ {max(found.values()):.2f} (the larger wheel, then check ff)")
    return 0


def cmd_ff(args) -> int:
    _need_qr()
    _confirm(args)
    volts = [float(v) for v in args.volts.split(",")]
    if max(volts) > config.QR_V_MAX_V:
        sys.exit(f"--volts above qr_base.v_max_v ({config.QR_V_MAX_V} V)")
    rig = Rig(_sides(args.wheel), args.reverse)
    rows: dict[str, list[tuple[float, float]]] = {s: [] for s in rig.sides}
    try:
        rig.check_estop()
        rig.engage()
        for v in volts:
            rig.volts(v)
            rig.speed(args.settle_s)
            sp = rig.speed(args.measure_s)
            line = []
            for side in rig.sides:
                w = sp.get(side)
                if w is None:
                    line.append(f"{side}: no encoder")
                    continue
                if (w < 0) != args.reverse and abs(w) > config.QR_ZERO_RAD_S:
                    raise RuntimeError(
                        f"{side} turned the WRONG way ({w:+.3f} rad/s): fix vehicle.invert_{side} "
                        f"or qr_base.enc_invert_{side} before anything else"
                    )
                rows[side].append((v, _to_motor_rpm(w)))
                line.append(
                    f"{side}: {w:+.3f} rad/s = {_to_motor_rpm(w):7.1f} motor r/min "
                    f"({abs(w) * config.WHEEL_DIA_M / 2:.3f} m/s)"
                )
            print(f"  {v:.2f} V   " + "   ".join(line))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        rig.close()

    print("\nfit  motor_rpm = rpm_per_volt * (V - offset_v)")
    fits = {}
    for side, pts in rows.items():
        pts = [(v, r) for v, r in pts if r > 1.0]
        if len(pts) < 2:
            print(f"  {side}: not enough moving points")
            continue
        n = len(pts)
        mv = sum(v for v, _ in pts) / n
        mr = sum(r for _, r in pts) / n
        sxx = sum((v - mv) ** 2 for v, _ in pts)
        if sxx <= 0:
            continue
        slope = sum((v - mv) * (r - mr) for v, r in pts) / sxx
        offset = mv - mr / slope
        fits[side] = (slope, offset)
        print(f"  {side}: ff_motor_rpm_per_volt {slope:.1f}   ff_offset_v {offset:.3f}")
    if fits:
        slope = min(s for s, _ in fits.values())
        offset = max(o for _, o in fits.values())
        print(
            f"\nprofile (the weaker wheel, so the PI adds rather than removes): "
            f'"ff_motor_rpm_per_volt": {slope:.1f}, "ff_offset_v": {max(0.0, offset):.2f}'
        )
        top = slope * (0.9 * config.QR_V_MAX_V - max(0.0, offset))
        print(
            f"with v_max_v {config.QR_V_MAX_V} V the config check allows vehicle.motor_max_rpm <= {top:.0f} "
            f"({top / config.RPM_PER_MPS:.2f} m/s)"
        )
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="qr_calibrate", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("io")
    sub.add_parser("enc")
    sub.add_parser("imu")
    sub.add_parser("brake-off")
    ims = sub.add_parser("imu-setup")
    ims.add_argument("--rate", type=int, default=50, help="Hz: 10, 20, 50, 100, 200")
    ims.add_argument("--baud", type=int, default=115200)
    ims.add_argument("--go", action="store_true")
    b = sub.add_parser("breakaway")
    b.add_argument("--wheel", choices=("left", "right", "both"), default="both")
    b.add_argument("--step-v", type=float, default=0.05)
    b.add_argument("--step-s", type=float, default=0.5)
    b.add_argument("--max-v", type=float, default=1.5)
    b.add_argument("--reverse", action="store_true")
    b.add_argument("--go", action="store_true")
    f = sub.add_parser("ff")
    f.add_argument("--wheel", choices=("left", "right", "both"), default="both")
    f.add_argument("--volts", default="0.8,1.2,1.6,2.0,2.4")
    f.add_argument("--settle-s", type=float, default=1.5)
    f.add_argument("--measure-s", type=float, default=2.0)
    f.add_argument("--reverse", action="store_true")
    f.add_argument("--go", action="store_true")
    args = ap.parse_args(argv)
    _need_qr()
    locks = []
    try:
        if args.cmd in ("io", "brake-off", "breakaway", "ff"):
            locks.append(ownerlock.acquire("dio", f"qr_calibrate {args.cmd}"))
        if args.cmd in ("enc", "breakaway", "ff"):
            locks.append(ownerlock.acquire("can", f"qr_calibrate {args.cmd}"))
        if args.cmd in ("imu", "imu-setup"):
            locks.append(ownerlock.acquire("imu", f"qr_calibrate {args.cmd}"))
    except ownerlock.OwnerBusy as e:
        sys.exit(f"{e} - stop amr.service / qr_base_node first")
    handler = {
        "io": cmd_io,
        "enc": cmd_enc,
        "imu": cmd_imu,
        "imu-setup": cmd_imu_setup,
        "brake-off": cmd_brake_off,
        "breakaway": cmd_breakaway,
        "ff": cmd_ff,
    }[args.cmd]
    return handler(args)


if __name__ == "__main__":
    sys.exit(main())
