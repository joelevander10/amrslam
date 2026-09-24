"""CANopen side of the drive node (T9): PDO layout, unit scaling, the CiA-402
arm/disarm sequence and the drive-side response to losing the PC.

No rclpy here. Everything that touches the bus goes through an injected
`bus` (python-can) so the sequences can be asserted without hardware, and every
write goes through drivers/canbus/guard.py, the same deny-list canworker used.
The SDO helpers, RPDO1 packing and the CiA-402 constants are the repo's own
(drivers/canbus), imported bare via amr_base.agv_repo.

Units (spec §3.2, reconciliation D-1/D-2): 60FFh and 606Ch are signed MOTOR
r/min; 6064h is motor-side encoder counts (608Fh counts per 6091h-geared
revolution). ROS speaks WHEEL rad/s. The gearbox ratio and the profile's
invert flags are applied exactly once, here.

PDO layout (spec §3.2: statusword + velocity + position do not fit one frame):

    RPDO1  0x200+n  6040h controlword u16 | 60FFh target velocity i32   (PC -> drive, 50 Hz)
    TPDO1  0x180+n  6041h statusword  u16 | 606Ch velocity actual i32   (drive -> PC, event timer)
    TPDO2  0x280+n  6064h position    i32 | 1001h error register u8     (drive -> PC, event timer)

TPDOs are transmission type 255 with an event timer, so the drive pushes them
at a fixed period whether or not the value changed, and an inhibit time equal
to that period so a changing value cannot push them faster.

PC-loss response (spec §3.3): the drive is told to CONSUME a heartbeat that
this process produces (1016h = PC node id + timeout). If the PC dies, the
drive raises 8130h (heartbeat error) and applies its fault reaction (605Eh,
default 2 = quick-stop ramp), independent of anything on the PC. Clearing that
alarm is an operator act (40C0h is on the deny-list), which is the intended
behaviour: a vehicle that stopped because its controller died is not restarted
by the controller coming back.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass, field

import can

import amr_base.agv_repo  # noqa: F401  (puts the repo layer dirs on sys.path)
from amr_base import pp

import guard  # repo module
import rpdo  # noqa: E402
from alarms import decode_emcy, decode_nmt  # noqa: E402
from bus_health import decode_state  # noqa: E402
from drive_forward import (  # noqa: E402
    CW_DISABLE_VOLTAGE,
    CW_ENABLE,
    CW_SHUTDOWN,
    CW_SWITCH_ON,
    SW_FAULT,
    SW_REMOTE,
    SW_SPEED_IS_ZERO,
    sdo_write,
)
from verify_drivers import sdo_read, u32  # noqa: E402

# ---------------------------------------------------------------- PDO layout

TPDO1_COB_BASE = 0x180
TPDO2_COB_BASE = 0x280
TPDO1_COMM, TPDO1_MAP = 0x1800, 0x1A00
TPDO2_COMM, TPDO2_MAP = 0x1801, 0x1A01
COB_DISABLED = 1 << 31
TRANSMISSION_EVENT = 255

MAP_STATUSWORD = (0x6041 << 16) | 0x0010
MAP_VELOCITY_ACTUAL = (0x606C << 16) | 0x0020
MAP_POSITION_ACTUAL = (0x6064 << 16) | 0x0020
MAP_ERROR_REGISTER = (0x1001 << 16) | 0x0008

_TPDO1 = struct.Struct("<Hi")  # statusword, velocity r/min
_TPDO2 = struct.Struct("<iB")  # position counts, error register

HEARTBEAT_COB_BASE = 0x700
NMT_OPERATIONAL = 0x05
CW_OPERATION_ENABLED = rpdo.CW_OPERATION_ENABLED
OPERATION_ENABLED_MASK, OPERATION_ENABLED = 0x6F, 0x27


def tpdo_configuration_steps(node: int, period_ms: int) -> list[tuple[int, int, int, int, str]]:
    """The SDO writes that put statusword+velocity on TPDO1 and position+error on TPDO2.

    Same disable → remap → enable ordering rpdo.configuration_steps uses (BLV-R
    manual §4.7.1). Returned as data so it can be checked against the deny-list
    and asserted in tests before a frame is sent.
    """
    if not 1 <= period_ms <= 65535:
        raise ValueError(f"TPDO event timer {period_ms} ms out of range 1..65535")
    steps = []
    for comm, mapping, cob_base, entries, label in (
        (TPDO1_COMM, TPDO1_MAP, TPDO1_COB_BASE, (MAP_STATUSWORD, MAP_VELOCITY_ACTUAL), "TPDO1"),
        (TPDO2_COMM, TPDO2_MAP, TPDO2_COB_BASE, (MAP_POSITION_ACTUAL, MAP_ERROR_REGISTER), "TPDO2"),
    ):
        cob = cob_base + node
        steps.append((comm, 1, COB_DISABLED | cob, 4, f"disable {label} before remapping"))
        steps.append((mapping, 0, 0, 1, f"{label}: clear the mapping entry count"))
        for i, entry in enumerate(entries, start=1):
            steps.append((mapping, i, entry, 4, f"{label}: map {entry >> 16:04X}h ({entry & 0xFF} bit)"))
        steps.append((mapping, 0, len(entries), 1, f"{label}: {len(entries)} mapped objects"))
        steps.append((comm, 2, TRANSMISSION_EVENT, 1, f"{label}: transmission type 255 (event)"))
        # Inhibit = period: type 255 also transmits on every VALUE CHANGE, and a
        # servo-locked position counter dithering by one count fired TPDO2 on
        # every drive cycle (~1200 f/s measured 2026-09-16, 3x the budget).
        steps.append((comm, 3, period_ms * 10, 2, f"{label}: inhibit time {period_ms} ms"))
        steps.append((comm, 5, period_ms, 2, f"{label}: event timer {period_ms} ms"))
        steps.append((comm, 1, cob, 4, f"enable {label} on 0x{cob:03X}"))
    for index, sub, value, _size, _what in steps:
        guard.check(index, value, sub)
    return steps


def decode_tpdo1(data: bytes) -> tuple[int, int]:
    """TPDO1 payload -> (statusword, velocity actual r/min)."""
    if len(data) < _TPDO1.size:
        raise ValueError(f"TPDO1 payload is {_TPDO1.size} bytes, got {len(data)}")
    return _TPDO1.unpack_from(bytes(data), 0)


def decode_tpdo2(data: bytes) -> tuple[int, int]:
    """TPDO2 payload -> (position actual counts, error register)."""
    if len(data) < _TPDO2.size:
        raise ValueError(f"TPDO2 payload is {_TPDO2.size} bytes, got {len(data)}")
    return _TPDO2.unpack_from(bytes(data), 0)


def heartbeat_consumer_value(producer_node: int, timeout_ms: int) -> int:
    """1016h:01 = producer node id in bits 16..23, timeout in ms in bits 0..15."""
    if not 1 <= producer_node <= 127:
        raise ValueError(f"producer node id {producer_node} not in 1..127")
    if not 0 <= timeout_ms <= 0xFFFF:
        raise ValueError(f"consumer heartbeat {timeout_ms} ms out of range")
    return (producer_node << 16) | timeout_ms


def pc_heartbeat_message(pc_node: int) -> can.Message:
    """The heartbeat this process produces so the drives can miss it."""
    return can.Message(
        arbitration_id=HEARTBEAT_COB_BASE + pc_node, data=[NMT_OPERATIONAL], is_extended_id=False
    )


def unwrap_i32(prev: int, new: int) -> int:
    """Signed delta between two INT32 counter readings, wrap-safe."""
    d = (new - prev) & 0xFFFFFFFF
    return d - (1 << 32) if d >= (1 << 31) else d


class WheelPosition:
    """Continuous wheel angle from the wrapping INT32 6064h counter (R06).

    Absolute counts / scale would turn the signed 32-bit rollover (about 2 000
    wheel turns at 1 080 000 counts per turn) into kilometres of travel. This
    accumulates wrap-safe count deltas instead, and only between two VALID
    samples under the same arm epoch and scale: an invalid sample, a re-arm or
    a scale change drops the baseline, and the next valid sample becomes the new
    one without moving the output. A discontinuity costs at most the motion
    since the last valid sample; it never shows up as displacement.
    """

    def __init__(self) -> None:
        self.rad = 0.0
        self._raw: int | None = None
        self._key = None

    def update(self, raw: int | None, valid: bool, scale: WheelScale | None, left: bool, epoch: int) -> float:
        if not valid or raw is None or scale is None or not scale.counts_per_wheel_rev:
            self._raw = None
            return self.rad
        key = (epoch, scale)
        if self._raw is not None and key == self._key:
            self.rad += scale.wheel_rad(unwrap_i32(self._raw, raw), left)
        self._raw, self._key = raw, key
        return self.rad


# ---------------------------------------------------------------- scaling

RAD_S_PER_RPM = 2.0 * 3.141592653589793 / 60.0


@dataclass(frozen=True)
class WheelScale:
    """Motor-side units -> wheel-side SI, per wheel. Applied exactly once."""

    gear_ratio: float
    invert_left: bool
    invert_right: bool
    counts_per_wheel_rev: float | None = None  # 608Fh/6091h at arm time; None = positions unknown

    def _sign(self, left: bool) -> float:
        return -1.0 if (self.invert_left if left else self.invert_right) else 1.0

    def motor_rpm(self, wheel_rad_s: float, left: bool) -> float:
        return wheel_rad_s / RAD_S_PER_RPM * self.gear_ratio * self._sign(left)

    def wheel_rad_s(self, motor_rpm: float, left: bool) -> float:
        return motor_rpm * RAD_S_PER_RPM / self.gear_ratio * self._sign(left)

    def wheel_rad(self, counts: int, left: bool) -> float | None:
        if not self.counts_per_wheel_rev:
            return None
        return counts / self.counts_per_wheel_rev * 2.0 * 3.141592653589793 * self._sign(left)


def counts_per_wheel_rev(read, nodes, gear_ratio: float) -> float | None:
    """Port of canworker._read_encoder_scale: 608Fh counts per motor rev × gear.

    `read(node, index, sub)` returns an unsigned int or None. Refuses (None)
    when the two drives disagree or 6091h shows a gear the profile does not,
    because a position scale guessed wrong is odometry that is wrong by 30×.
    """
    scales = set()
    for nid in nodes:
        inc, revs = read(nid, 0x608F, 1), read(nid, 0x608F, 2)
        gm, gs = read(nid, 0x6091, 1), read(nid, 0x6091, 2)
        if not (inc and revs and gm and gs):
            return None
        drive_gear = gm / gs
        if not (abs(drive_gear - 1.0) < 1e-9 or abs(drive_gear - gear_ratio) < 1e-9):
            return None
        scales.add(inc / revs * gear_ratio)
    return scales.pop() if len(scales) == 1 else None


# ---------------------------------------------------------------- router


class Router:
    """Bus wrapper that hands UNSOLICITED frames to decoders before the SDO
    helpers can discard them (the canworker.TpdoTap shape).

    sdo_read/sdo_write drain and filter on 0x580+node, so any pushed frame not
    routed here is lost while a transfer is in flight. Handlers run on the bus
    thread and must never raise; an exception is swallowed so an SDO transfer
    in progress is not broken by a decoder bug.
    """

    def __init__(self, bus):
        self._bus = bus
        self._route: dict[int, callable] = {}

    def add(self, cob_id: int, handler) -> None:
        self._route[cob_id] = handler

    def send(self, msg) -> None:
        self._bus.send(msg)

    def recv(self, timeout=None):
        deadline = None if timeout is None else time.perf_counter() + timeout
        while True:
            remaining = None if deadline is None else max(0.0, deadline - time.perf_counter())
            m = self._bus.recv(timeout=remaining)
            if m is None:
                return None
            h = self._route.get(m.arbitration_id)
            if h is None:
                return m
            try:
                h(m)
            except Exception:  # noqa: BLE001 - see class docstring
                pass
            if deadline is not None and time.perf_counter() >= deadline:
                return None

    def pump(self, seconds: float) -> None:
        """Route everything that arrives for `seconds`; SDO replies to nobody are dropped."""
        end = time.perf_counter() + seconds
        while True:
            left = end - time.perf_counter()
            if left <= 0:
                return
            if self.recv(timeout=left) is None:
                return

    def shutdown(self) -> None:
        self._bus.shutdown()


# ---------------------------------------------------------------- drive link

DISARMED, ARMED, FAULT = "disarmed", "armed", "fault"


@dataclass
class DriveTelemetry:
    statusword: int | None = None
    rpm: int | None = None
    position: int | None = None
    error_register: int | None = None
    t_status: float | None = None  # monotonic receive time of the last TPDO1
    t_position: float | None = None  # ... TPDO2
    t_alive: float | None = None  # any frame or SDO reply from this node
    nmt: str | None = None
    alarm: dict | None = None

    @property
    def state(self) -> str:
        return "-" if self.statusword is None else decode_state(self.statusword)

    @property
    def operation_enabled(self) -> bool | None:
        if self.statusword is None:
            return None
        return (self.statusword & OPERATION_ENABLED_MASK) == OPERATION_ENABLED

    @property
    def faulted(self) -> bool:
        return bool(self.statusword is not None and self.statusword & SW_FAULT) or bool(self.error_register)


@dataclass
class DriveLink:
    """Owns the two BLV-R drives over one Router. Blocking, bus-thread only.

    State: DISARMED (drives de-energised, or not yet touched), ARMED (both in
    Operation enabled, RPDO1 live), FAULT (a drive faulted or fell silent while
    armed; cleared only by disarm()). Every transition that can fail part-way
    rolls back to a known state: arm() de-energises BOTH drives on any error.
    """

    router: Router
    nodes: dict[int, str]
    ramp: dict  # {"accel": rpm/s, "decel": rpm/s}
    log: callable = print
    state: str = DISARMED
    fault_reason: str | None = None
    telemetry: dict[int, DriveTelemetry] = field(default_factory=dict)
    scale: WheelScale | None = None
    applied: tuple[int, int] | None = None
    pc_guard_nodes: set = field(default_factory=set)  # 1016h written on these; cleared on disarm
    # PC heartbeat kept alive INSIDE blocking arm/disarm transactions (R05): the
    # loop cannot produce it while an SDO sequence or the speed-zero wait runs.
    pc_node: int | None = None
    pc_heartbeat_s: float = 0.1
    heartbeat_withheld: bool = False  # R04 fallback: let the drives' 1016h trip
    cleanup_owed: bool = False  # arm touched the drives; a disarm has not fully undone it
    cleanup_failures: list = field(default_factory=list)
    arm_epoch: int = 0  # bumped by every arm; WheelPosition rebaselines on it
    t_armed: float | None = None
    # R04: nodes a fault-stop zero has not reached yet, and the retry/confirm budget.
    stop_pending: set = field(default_factory=set)
    stop_unconfirmed: list = field(default_factory=list)
    t_fault: float | None = None
    _stop_tries: int = 0
    _t_hb: float = 0.0
    # The drives may be in pp (6060h = 1). In pp, statusword bit 12 is "set-point
    # acknowledge", NOT "speed is zero", and an RPDO without Halt lets a halted move
    # RESUME towards its target - so every stop frame and standstill test differs.
    in_pp: bool = False

    STOP_RETRIES = 5  # extra loop ticks a failed zero send is retried
    STOP_CONFIRM_S = 3.0  # decel 3200 rpm/s from 4000 r/min is 1.25 s

    @property
    def pc_guard_set(self) -> bool:
        return bool(self.pc_guard_nodes)

    @property
    def stop_controlword(self) -> int:
        """What a zero/stop RPDO carries: Halt while in pp (a plain 0x0F would let a
        halted pp move continue), plain Operation enabled in pv."""
        return pp.CW_HALTED if self.in_pp else CW_OPERATION_ENABLED

    def _standstill_status(self, t: DriveTelemetry) -> bool:
        """Positive standstill from a TPDO1: 606Ch = 0 in pp, the speed-zero bit in pv."""
        if self.in_pp:
            return t.rpm == 0
        return bool(t.statusword & SW_SPEED_IS_ZERO)

    def _standstill_sdo(self, nid: int) -> bool:
        if self.in_pp:
            return self.read(nid, 0x606C, timeout=0.2) == 0
        return bool((self.read(nid, 0x6041, timeout=0.2) or 0) & SW_SPEED_IS_ZERO)

    def __post_init__(self):
        for nid in self.nodes:
            self.telemetry[nid] = DriveTelemetry()
            self.router.add(TPDO1_COB_BASE + nid, self._make_tpdo1(nid))
            self.router.add(TPDO2_COB_BASE + nid, self._make_tpdo2(nid))
            self.router.add(0x080 + nid, self._make_emcy(nid))
            self.router.add(HEARTBEAT_COB_BASE + nid, self._make_heartbeat(nid))

    # -- pushed frames (bus thread, inside recv) --

    def _make_tpdo1(self, nid):
        def on(m):
            sw, rpm = decode_tpdo1(m.data)
            t = self.telemetry[nid]
            t.statusword, t.rpm = sw & 0xFFFF, rpm
            t.t_status = t.t_alive = time.monotonic()

        return on

    def _make_tpdo2(self, nid):
        def on(m):
            pos, err = decode_tpdo2(m.data)
            t = self.telemetry[nid]
            t.position, t.error_register = pos, err
            t.t_position = t.t_alive = time.monotonic()

        return on

    def _make_emcy(self, nid):
        def on(m):
            a = decode_emcy(bytes(m.data))
            t = self.telemetry[nid]
            t.t_alive = time.monotonic()
            t.alarm = None if a["cleared"] else a
            label = self.nodes[nid]
            if a["cleared"]:
                self.log(f"node {nid} ({label}) alarms cleared")
            else:
                self.log(f"node {nid} ({label}) ALARM {a['hex']} - {a['name']}: {a['note']}")

        return on

    def _make_heartbeat(self, nid):
        def on(m):
            t = self.telemetry[nid]
            t.t_alive = time.monotonic()
            t.nmt = decode_nmt(m.data[0] if m.data else 0)

        return on

    # -- SDO (blocking) --

    def keepalive(self) -> None:
        """PC heartbeat if one is due. Same thread as every other frame, so no second CAN owner."""
        if self.pc_node is None or self.heartbeat_withheld:
            return
        if time.monotonic() - self._t_hb >= self.pc_heartbeat_s:
            try:
                self.send_pc_heartbeat(self.pc_node)
            except Exception:  # noqa: BLE001 - the caller's own write will report the bus
                pass

    def read(self, node, index, sub=0, timeout=0.4):
        self.keepalive()
        st, val, _, _ = sdo_read(self.router, node, index, sub, timeout=timeout, collision_window=0.0)
        if st:
            self._alive(node)
            return u32(val)
        return None

    def _alive(self, node):
        t = self.telemetry.get(node)  # the MLS (node 10) is read through here too
        if t is not None:
            t.t_alive = time.monotonic()

    def write(self, node, index, sub, value, size, what):
        guard.check(index, value, sub)
        self.keepalive()
        ok, detail = sdo_write(self.router, node, index, sub, value, size)
        if not ok:
            raise RuntimeError(f"node {node}: {what} ({index:04X}h) failed: {detail}")
        self._alive(node)

    def nmt(self, command, node=0):
        self.router.send(can.Message(arbitration_id=0x000, data=[command, node], is_extended_id=False))
        self.router.pump(0.05)

    # -- configuration --

    def enable_heartbeat(self, ms: int) -> None:
        """Drive PRODUCER heartbeat (1017h): what tells a dead drive from an idle one."""
        if not ms:
            return
        for nid in self.nodes:
            try:
                self.write(nid, 0x1017, 0, ms, 2, "producer heartbeat time")
            except Exception as e:  # noqa: BLE001
                self.log(f"node {nid} refused a heartbeat interval ({e}); liveness falls back to TPDOs")

    def set_pc_loss_guard(self, pc_node: int, timeout_ms: int) -> None:
        """Drive CONSUMER heartbeat (1016h): the drive's own response to losing us."""
        for nid in self.nodes:
            self.write(
                nid, 0x1016, 1, heartbeat_consumer_value(pc_node, timeout_ms), 4, "consumer heartbeat time"
            )
            # Per node, as soon as its write lands: a failure on the second node
            # must still leave the first one tracked for the rollback (R05).
            self.pc_guard_nodes.add(nid)

    def clear_pc_loss_guard(self) -> list[int]:
        """Retire 1016h on a DELIBERATE exit, while our heartbeat is still fresh.

        The guard exists for the PC dying, not for the PC leaving. Left set, the
        drives fault with 8130h half a second after a clean shutdown - which
        looks identical to a crash, cannot be cleared over CAN (40C0h is
        deny-listed) and needs a drive power cycle. Found 2026-09-16: both drives
        in FAULT after drive_node exited, and agv_controller could not arm.

        Returns the nodes still carrying the guard; they stay tracked, so a
        later disarm or exit tries again.
        """
        failed = []
        for nid in [n for n in self.nodes if n in self.pc_guard_nodes]:
            try:
                self.write(nid, 0x1016, 1, 0, 4, "consumer heartbeat time = 0")
                self.pc_guard_nodes.discard(nid)
            except Exception as e:  # noqa: BLE001 - de-energising still proceeds
                self.log(f"node {nid}: could not clear 1016h ({e}); it will fault when we stop")
                failed.append(nid)
        return failed

    def configure_pdos(self, feedback_period_ms: int) -> None:
        """RPDO1 (setpoint) and TPDO1/2 (feedback). Pre-operational only (CiA 301)."""
        for nid in self.nodes:
            rpdo.configure(self.router, nid, self._sdo_write_tuple)
            for index, sub, value, size, what in tpdo_configuration_steps(nid, feedback_period_ms):
                self.write(nid, index, sub, value, size, what)

    def _sdo_write_tuple(self, bus, node, index, sub, value, size):
        try:
            self.write(node, index, sub, value, size, "RPDO1 setup")
        except Exception as e:  # noqa: BLE001
            return False, str(e)
        return True, ""

    # -- state machine --

    def preflight(self) -> tuple[bool, list[str]]:
        ok, report = True, []
        for nid, label in self.nodes.items():
            st, _, note, _ = sdo_read(self.router, nid, 0x1000, 0)
            if st is None:
                report.append(f"node {nid} ({label}): not responding")
                ok = False
                continue
            if "COLLISION" in note:
                report.append(f"node {nid} ({label}): {note}")
                ok = False
                continue
            err, sw = self.read(nid, 0x1001), self.read(nid, 0x6041)
            if err is None or sw is None:
                report.append(f"node {nid} ({label}): diagnostics unreadable")
                ok = False
                continue
            report.append(
                f"node {nid} ({label}): error reg 0x{err:02X}, statusword 0x{sw:04X} ({decode_state(sw)})"
            )
            if err:
                report.append(f"  node {nid}: driver reports a fault - clear it first")
                ok = False
            if not sw & SW_REMOTE:
                report.append(
                    f"  node {nid}: Remote bit clear - controlword ignored (S-ON active, or MEXE02)"
                )
                ok = False
            if sw & SW_FAULT:
                report.append(f"  node {nid}: FAULT state")
                ok = False
        return ok, report

    def arm(
        self,
        feedback_period_ms: int,
        pc_node: int | None,
        pc_loss_ms: int,
        gear_ratio: float,
        invert_left: bool,
        invert_right: bool,
    ) -> list[str]:
        """Energise both drives with zero targets. Raises, and rolls back, on any failure.

        Everything after preflight (the first write to a drive) is inside the
        rollback: NMT, PDO mapping, each node's 1016h, the enable sequence and
        the post-enable scale reads (R05). A failure anywhere de-energises both
        drives and retires whichever guards were already written.
        """
        ok, report = self.preflight()
        if not ok:
            raise RuntimeError("preflight failed: " + "; ".join(report))
        self.arm_epoch += 1
        self.cleanup_owed = True
        self._reset_stop()
        try:
            # PDO mapping and NMT error control belong in PRE-OPERATIONAL.
            self.nmt(0x80)
            self.configure_pdos(feedback_period_ms)
            if pc_node is not None and pc_loss_ms:
                self.set_pc_loss_guard(pc_node, pc_loss_ms)
            self._enable_sequence()
            cprev = counts_per_wheel_rev(lambda n, i, s: self.read(n, i, s), tuple(self.nodes), gear_ratio)
        except BaseException:
            self.disarm(force=True)
            raise
        self.scale = WheelScale(gear_ratio, invert_left, invert_right, cprev)
        if cprev is None:
            self.log("encoder scale (608Fh/6091h) unreadable or inconsistent - wheel positions invalid")
        else:
            self.log(f"encoder scale {cprev:.0f} counts per wheel turn")
        self.applied = None
        self.in_pp = False  # _enable_sequence wrote 6060h = pv on both drives
        self.t_armed = time.monotonic()
        self.state, self.fault_reason = ARMED, None
        return report

    def _enable_sequence(self) -> None:
        for nid in self.nodes:
            self.nmt(0x01, nid)
        for nid in self.nodes:
            self.write(nid, 0x6060, 0, 3, 1, "modes of operation = pv")
            self.write(nid, 0x6083, 0, int(self.ramp["accel"]), 4, "profile acceleration")
            self.write(nid, 0x6084, 0, int(self.ramp["decel"]), 4, "profile deceleration")
            self.write(nid, 0x60FF, 0, 0, 4, "target velocity = 0")
            for cw, name in (
                (CW_SHUTDOWN, "Shutdown"),
                (CW_SWITCH_ON, "Switch On"),
                (CW_ENABLE, "Enable Operation"),
            ):
                self.write(nid, 0x6040, 0, cw, 2, name)
                self.router.pump(0.05)
            sw = self.read(nid, 0x6041) or 0
            if (sw & OPERATION_ENABLED_MASK) != OPERATION_ENABLED:
                raise RuntimeError(
                    f"node {nid} did not reach Operation enabled (statusword 0x{sw:04X}, {decode_state(sw)})"
                )

    def disarm(self, force: bool = False) -> bool:
        """Zero, wait for the ramp, de-energise, Pre-operational. Never raises.

        Returns True when every cleanup write was acknowledged. Anything short of
        that leaves `cleanup_owed` set (and `cleanup_failures` saying what), so a
        later disarm or the exit path runs the teardown again instead of taking
        the software DISARMED state as proof the hardware is (R05).
        """
        was = self.state
        self.state, self.applied = DISARMED, None
        if was == DISARMED and not force and not self.cleanup_owed:
            return True
        failed = []
        # A zero RPDO before anything blocking, so a teardown interrupted during
        # the SDO writes below still leaves the drives commanding zero. Only
        # when they were ours and enabled: the RPDO carries controlword 0x0F,
        # which must not ENABLE a drive a rollback caught half-switched-on.
        if was in (ARMED, FAULT):
            failed += [f"node {n}: zero RPDO" for n in self._send_zero_each()]
        # Retire 1016h next, BEFORE the speed-zero wait. The review (R05) asks for
        # stop/de-energise first; the measured order is the other way round
        # (2026-09-16, 6515799): with the guard still set, both drives raised
        # 8130h during/after a clean exit and needed a power cycle. The heartbeat
        # keep-alive inside write()/read() now covers the wait too, but that is
        # not yet bench-proven, so the proven order stays.
        failed += [f"node {n}: 1016h" for n in self.clear_pc_loss_guard()]
        for nid in self.nodes:
            ok, detail = self._raw_write(nid, 0x60FF, 0, 0, 4)
            if not ok:
                failed.append(f"node {nid}: 60FFh=0 ({detail})")
        end = time.monotonic() + 4.0
        while time.monotonic() < end:
            try:
                if all(self._standstill_sdo(n) for n in self.nodes):
                    break
            except Exception:  # noqa: BLE001
                break
            self.router.pump(0.05)
        for nid in self.nodes:
            for cw in (CW_SHUTDOWN, CW_DISABLE_VOLTAGE):
                ok, detail = self._raw_write(nid, 0x6040, 0, cw, 2)
                if not ok:
                    failed.append(f"node {nid}: 6040h=0x{cw:04X} ({detail})")
        try:
            self.nmt(0x80)
        except Exception as e:  # noqa: BLE001
            failed.append(f"NMT pre-operational ({e})")
        self.fault_reason = None if was != FAULT else self.fault_reason
        self.cleanup_owed, self.cleanup_failures = bool(failed), failed
        self._reset_stop()
        if failed:
            self.log(f"disarm incomplete, will retry on the next disarm: {'; '.join(failed)}")
        return not failed

    def _raw_write(self, nid, index, sub, value, size) -> tuple[bool, str]:
        self.keepalive()
        try:
            ok, detail = sdo_write(self.router, nid, index, sub, value, size)
        except Exception as e:  # noqa: BLE001
            return False, str(e)
        if ok:
            self._alive(nid)
        return ok, detail

    def fault(self, reason: str) -> None:
        """Latch a fault: setpoint zero now, no ramp. Drives stay energised (servo lock).

        The zero goes to each drive independently; one failed send does not skip
        the other (R04). Nodes it did not reach are retried by fault_tick().
        """
        if self.state == ARMED:
            self.stop_pending = set(self._send_zero_each())
            self.t_fault = time.monotonic()
            self._stop_tries = 0
        self.state, self.fault_reason = FAULT, reason

    def fault_tick(self, now: float, max_age: float) -> None:
        """Once per loop in FAULT: bounded zero retries, then measured standstill.

        Unconfirmed = a zero that could not be sent after STOP_RETRIES, or, once
        STOP_CONFIRM_S has passed, a drive without POSITIVE standstill evidence: a
        fresh statusword from after the fault with speed-zero. A drive whose status
        is missing or stale is unconfirmed too (review Q01): a silent TPDO stream
        does not mean the drive cannot hear our heartbeat, and a successful socket
        send proves nothing about reception. Unconfirmed makes the node withhold
        the PC heartbeat so every drive's own 1016h trips (the documented fallback).
        """
        if self.state != FAULT or self.t_fault is None:
            return
        if self.stop_pending and self._stop_tries < self.STOP_RETRIES:
            self._stop_tries += 1
            self.stop_pending = set(self._send_zero_each(self.stop_pending))
        unconfirmed = []
        if self.stop_pending and self._stop_tries >= self.STOP_RETRIES:
            unconfirmed += [f"node {n}: zero not delivered" for n in sorted(self.stop_pending)]
        if now - self.t_fault >= self.STOP_CONFIRM_S:
            for n in self.nodes:
                t = self.telemetry.get(n)
                if t is None or t.t_status is None or t.t_status < self.t_fault:
                    unconfirmed.append(f"node {n}: no status since the fault")
                elif now - t.t_status >= max_age:
                    unconfirmed.append(f"node {n}: status stale ({now - t.t_status:.1f} s)")
                elif not self._standstill_status(t):
                    unconfirmed.append(f"node {n}: still turning ({t.rpm} r/min)")
        if unconfirmed != self.stop_unconfirmed:
            self.stop_unconfirmed = unconfirmed
            if unconfirmed:
                self.log(f"fault stop UNCONFIRMED: {'; '.join(unconfirmed)}")

    def _reset_stop(self) -> None:
        self.stop_pending, self.stop_unconfirmed, self.t_fault, self._stop_tries = set(), [], None, 0
        self.heartbeat_withheld = False

    def _send_zero_each(self, nodes=None) -> list[int]:
        """Zero RPDO1 to each node on its own; returns the nodes whose send raised."""
        failed = []
        for nid in self.nodes if nodes is None else [n for n in self.nodes if n in nodes]:
            try:
                rpdo.send(self.router, nid, self.stop_controlword, 0)
            except Exception as e:  # noqa: BLE001
                self.log(f"node {nid}: zero setpoint send failed ({e})")
                failed.append(nid)
        if not failed:
            self.applied = (0, 0)
        return failed

    def send_target(self, left_rpm: int, right_rpm: int) -> None:
        """RPDO1 to both drives. A queue append; nothing is waited for."""
        target = (int(round(left_rpm)), int(round(right_rpm)))
        for nid, rpm in zip(self.nodes, target, strict=True):
            rpdo.send(self.router, nid, CW_OPERATION_ENABLED, rpm)
        self.applied = target

    def send_pc_heartbeat(self, pc_node: int) -> None:
        self._t_hb = time.monotonic()
        self.router.send(pc_heartbeat_message(pc_node))

    # -- profile position (pp) blind moves: amr_base/pp.py decides, these act --
    #
    # Entered only from ARMED, at rest, after the pp safety configuration read back
    # from both drives matches the profile. Nothing here writes that configuration
    # (6072h/6065h/6067h/605Dh/605Eh/6085h are not on guard.ALLOWED). Every failure
    # after the first mode write puts the drives back in pv before it is reported.

    MODE_READBACK_TRIES = 10

    def _mode_readback(self, nid: int, want: int) -> None:
        for _ in range(self.MODE_READBACK_TRIES):
            got = self.read(nid, 0x6061)
            if got is not None and (got & 0xFF) == want:
                return
            self.router.pump(0.02)
        raise RuntimeError(f"node {nid}: 6061h did not read mode {want}")

    def at_rest(self, now: float, max_age: float) -> str | None:
        """None when both drives report zero speed on fresh feedback, else why not."""
        for nid, label in self.nodes.items():
            t = self.telemetry[nid]
            if t.t_status is None or now - t.t_status > max_age:
                return f"{label} feedback stale"
            if t.rpm != 0:
                return f"{label} turning ({t.rpm} r/min)"
            if t.position is None or t.t_position is None or now - t.t_position > max_age:
                return f"{label} position stale"
        return None

    def pp_enter(self, spec: pp.MoveSpec, expect: dict, objects: dict, max_age: float) -> tuple[int, int]:
        """Blocking. Verify, switch both drives to pp, write the set-points. -> absolute targets.

        Raises RuntimeError with the reason. Moves nothing by itself: motion starts
        with the new-set-point edge the controller sends on the next ticks.
        """
        if self.state != ARMED:
            raise RuntimeError(f"drives not armed ({self.state})")
        if self.faulted_nodes():
            raise RuntimeError(f"drive fault on node(s) {self.faulted_nodes()}")
        why = self.at_rest(time.monotonic(), max_age)
        if why:
            raise RuntimeError(f"not at rest: {why}")
        problems = pp.verify_config(lambda n, i: self.read(n, i), self.nodes, expect, objects)
        if problems:
            raise RuntimeError("pp configuration check failed: " + "; ".join(problems))
        actual = tuple(self.telemetry[n].position for n in self.nodes)
        targets = pp.absolute_targets(actual, spec)  # raises ValueError before anything is written
        self.in_pp = True  # from the first mode write on, stop frames carry Halt
        try:
            for nid in self.nodes:
                self.write(nid, 0x6060, 0, pp.MODE_PP, 1, "modes of operation = pp")
            for nid in self.nodes:
                self._mode_readback(nid, pp.MODE_PP)
            for (nid, label), wheel, target in zip(self.nodes.items(), spec.wheels(), targets, strict=True):
                if not wheel.moves:
                    continue  # stays in pp at rest: the position loop holds it where it is
                self.write(nid, 0x6081, 0, wheel.velocity_rpm, 4, f"{label} profile velocity")
                self.write(nid, 0x6083, 0, wheel.accel_rpm_s, 4, f"{label} profile acceleration")
                self.write(nid, 0x6084, 0, wheel.decel_rpm_s, 4, f"{label} profile deceleration")
                self.write(nid, 0x607A, 0, target, 4, f"{label} target position")
        except Exception as e:
            try:
                self.pp_exit()
            except Exception as e2:  # noqa: BLE001
                raise RuntimeError(f"{e}; and the return to pv failed: {e2}") from e
            raise RuntimeError(str(e)) from e
        self.applied = None
        return targets

    def send_controlwords(self, cw_left: int, cw_right: int) -> None:
        """pp ticks: RPDO1 carries only the controlword; the velocity field is zero."""
        for nid, cw in zip(self.nodes, (cw_left, cw_right), strict=True):
            rpdo.send(self.router, nid, cw, 0)

    def pp_exit(self) -> None:
        """Blocking. Both drives back to pv at zero, with the pv ramps restored. Raises on failure.

        Halt stays asserted until pv is confirmed and 60FFh is zero: clearing it while
        still in pp would let a halted move carry on to its target.
        """
        errors = []
        for nid in self.nodes:
            try:
                rpdo.send(self.router, nid, pp.CW_HALTED, 0)
                self.write(nid, 0x6060, 0, pp.MODE_PV, 1, "modes of operation = pv")
                self._mode_readback(nid, pp.MODE_PV)
                self.write(nid, 0x60FF, 0, 0, 4, "target velocity = 0")
                self.write(nid, 0x6083, 0, int(self.ramp["accel"]), 4, "profile acceleration")
                self.write(nid, 0x6084, 0, int(self.ramp["decel"]), 4, "profile deceleration")
            except Exception as e:  # noqa: BLE001 - try the other drive too, then report both
                errors.append(f"node {nid}: {e}")
        if errors:
            raise RuntimeError("; ".join(errors))
        self.in_pp = False
        self.applied = (0, 0)

    # -- checks --

    def silent_nodes(self, now: float, timeout_s: float) -> list[int]:
        return [n for n, t in self.telemetry.items() if t.t_alive is None or now - t.t_alive > timeout_s]

    def feedback_fresh(self, nid: int, now: float, max_age: float) -> tuple[bool, bool]:
        """(status fresh, position fresh): TPDO1 and TPDO2 each on their own (R07).

        Heartbeats, EMCY and SDO replies refresh t_alive, never these.
        """
        t = self.telemetry[nid]
        return (
            t.t_status is not None and now - t.t_status < max_age,
            t.t_position is not None and now - t.t_position < max_age,
        )

    def missing_feedback(self, now: float, timeout_s: float) -> list[int]:
        """Armed drives whose TPDO1 or TPDO2 stopped, even if they still answer
        heartbeats/SDOs. Counted from the arm, so frames from before it do not count."""
        since = self.t_armed if self.t_armed is not None else now
        out = []
        for n, t in self.telemetry.items():
            ts = max(t.t_status or 0.0, since)
            tp = max(t.t_position or 0.0, since)
            if now - ts > timeout_s or now - tp > timeout_s:
                out.append(n)
        return out

    def dropped_out(self) -> list[int]:
        """Drives that left Operation enabled while we believe they are armed."""
        return [n for n, t in self.telemetry.items() if t.operation_enabled is False]

    def faulted_nodes(self) -> list[int]:
        return [n for n, t in self.telemetry.items() if t.faulted or t.alarm is not None]


def wheel_feedback(
    link: DriveLink, now: float, max_age: float, wheels, is_new: dict, positions: dict
) -> dict:
    """Per wheel (pos_rad, vel_rad_s, valid) for one WheelStates (R06/R07).

    `wheels` is ((node, left), ...); `is_new[node]` says both TPDO1 and TPDO2
    arrived since the last publish; `positions[node]` is that wheel's
    WheelPosition. Valid needs a fresh statusword AND a fresh position, each on
    its own clock, a new sample of both, a known scale, no fault, and ARMED.
    """
    out = {}
    scale = link.scale
    for n, left in wheels:
        t = link.telemetry[n]
        status_ok, position_ok = link.feedback_fresh(n, now, max_age)
        valid = bool(
            status_ok
            and position_ok
            and is_new.get(n)
            and t.position is not None
            and scale is not None
            and scale.counts_per_wheel_rev
            and not t.faulted
            and link.state == ARMED
        )
        pos = positions[n].update(t.position, valid, scale, left, link.arm_epoch)
        vel = scale.wheel_rad_s(t.rpm, left) if (scale and status_ok and t.rpm is not None) else 0.0
        out[n] = (pos, vel, valid)
    return out


def target_rpm(cmd, now: float, timeout_s: float, scale: WheelScale, max_rpm: float) -> tuple[int, int]:
    """The setpoint for this tick. `cmd` is (t_recv, left_rad_s, right_rad_s) or None.

    The drive node's OWN command watchdog (spec §3.5): a command older than
    timeout_s is a zero setpoint regardless of what the mux last said, and the
    result is clamped to the motor's rating per wheel.
    """
    # A negative age (the caller's clock behind the receipt time) is not "fresh": it is a
    # clock error, and the safe setpoint for a clock error is zero (review Q03).
    if cmd is None or not (0.0 <= now - cmd[0] <= timeout_s):
        return 0, 0
    # R03: max/min would turn NaN into full scale. Anything nonfinite - the
    # command, the scaled result (1e308 x gear) or the limit - is zero.
    if not (math.isfinite(max_rpm) and max_rpm >= 0.0):
        return 0, 0
    left, right = scale.motor_rpm(cmd[1], left=True), scale.motor_rpm(cmd[2], left=False)
    if not (math.isfinite(left) and math.isfinite(right)):
        return 0, 0
    left = max(-max_rpm, min(max_rpm, left))
    right = max(-max_rpm, min(max_rpm, right))
    return int(round(left)), int(round(right))


@dataclass(frozen=True)
class ArmDecision:
    action: str  # "none" | "arm" | "fault" | "disarm"
    reason: str = ""


def decide(
    state: str,
    want_armed: bool,
    now: float,
    retry_at: float,
    silent: list,
    dropped: list,
    faulted: list,
    cleanup_owed: bool = False,
) -> ArmDecision:
    """The arm/fault policy as a pure function, so it can be tabled in a test.

    - not wanted: disarm if anything is energised.
    - DISARMED and wanted: arm, subject to the retry backoff.
    - ARMED: a silent or alarmed drive is a FAULT (latched; the setpoint goes
      to zero at once). FAULT is left only by an explicit ack/disarm - a
      vehicle that stopped itself is not restarted by the thing that stopped it.
    - ARMED but a drive left Operation enabled (the safety chain took it to
      ETO): DISARM and let the backoff re-arm with a ZERO target, the same
      level-held behaviour canworker._hold_arm_state has. Re-arming moves
      nothing: motion needs a MotionPermit and a Start edge upstream (T8).
    - FAULT: stay there.
    - DISARMED with cleanup owed (a disarm left a drive energised, guarded or
      in the wrong NMT state): retry the teardown on the backoff, wanted or not,
      and never arm over it (review Q02) - the software state is not proof.
    """
    if state == DISARMED and cleanup_owed:
        return ArmDecision("cleanup" if now >= retry_at else "none", "cleanup owed")
    if not want_armed:
        return ArmDecision("disarm" if state != DISARMED else "none", "not wanted")
    if state == DISARMED:
        return ArmDecision("arm" if now >= retry_at else "none", "backoff" if now < retry_at else "wanted")
    if state == ARMED:
        if silent:
            return ArmDecision("fault", f"drive silent: {silent}")
        if faulted:
            return ArmDecision("fault", f"drive fault: {faulted}")
        if dropped:
            return ArmDecision("disarm", f"drive left Operation enabled (ETO?): {dropped}")
        return ArmDecision("none", "armed")
    return ArmDecision("none", "fault latched")


# ---------------------------------------------------------------- monitoring slot


class MonitorCursor:
    """One bounded diagnostic SDO read per call (unified plan §7.1), round-robin
    over canmon.OBJECTS x nodes. `read(node, index)` is the link's SDO read
    (None on timeout). Pure bookkeeping around the repo's MonitorPoller so the
    slot is testable without a bus; the caller decides WHEN to call it."""

    def __init__(self, poller, nodes) -> None:
        self.poller = poller
        self.nodes = list(nodes)
        self._i = 0
        self._current = None
        self.reads = 0
        self.timeouts = 0

    def step(self, read, decode) -> tuple[int, str, object] | None:
        if self._i == 0 or self._current is None:
            self._current = self.poller.next_object()
            if self._current is None:
                return None
        index, key, ctype = self._current
        nid = self.nodes[self._i]
        self._i = (self._i + 1) % len(self.nodes)
        raw = read(nid, index)
        self.reads += 1
        if raw is None:
            self.timeouts += 1
            self.poller.store(nid, key, None)
            return nid, key, None
        value = decode(ctype, int(raw).to_bytes(4, "little"))
        self.poller.store(nid, key, value)
        return nid, key, value
