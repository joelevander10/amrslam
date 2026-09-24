"""End-to-end smoke test of qr_base_node with STUBBED ROS and simulated hardware.

Runs without a ROS installation: rclpy and the message packages are replaced by minimal
stand-ins, so this checks the node's own wiring (threads, gating, panel, wheel loop,
coils, shutdown), not DDS. test_qr_base_node_sim.py runs it in a subprocess, because the
stubs replace modules process-wide.

    python3 tools/qr_base_node_sim.py      # exit 0 = every check passed

Simulated: CK5162E (DI image + coil readback), CKDA08ETH (volts), two CANopen encoders
emitting TPDO1 from a first-order motor model, no IMU. Checks: arming, panel virtual
selector, a forward command reaching speed with the right coils, reversal interlock,
E-stop, command watchdog, clean shutdown.
"""

import math
import os
import struct
import sys
import tempfile
import threading
import time
import types

os.environ["AGV_PROFILE"] = "amr-qr-01"
os.environ["AMR_LOCK_DIR"] = tempfile.mkdtemp(prefix="amr_lock_")
# amr_base package source (this file lives in amr_base/tools/)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


# ---------------------------------------------------------------- ROS stubs
def mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Stamp:
    def __init__(self, t=0.0):
        self.sec, self.nanosec = int(t), int((t % 1) * 1e9)


class Header:
    def __init__(self):
        self.stamp = Stamp()
        self.frame_id = ""


def msgclass(name, fields):
    def init(self, **kw):
        self.header = Header()
        for k, v in fields.items():
            setattr(self, k, v() if callable(v) else v)
        self.__dict__.update(kw)

    return type(name, (), {"__init__": init})


class Duration:
    def __init__(self, seconds=0.0):
        self.s = seconds


class Time:
    def __init__(self, t):
        self.t = t

    def __sub__(self, d):
        return Time(self.t - d.s)

    def to_msg(self):
        return Stamp(self.t)


class Clock:
    def now(self):
        return Time(time.time())


class Logger:
    def __init__(self, log):
        self.log = log

    def _p(self, lvl):
        def f(msg, **kw):
            self.log.append((lvl, msg))
            print(f"  [{lvl}] {msg}")

        return f

    def __getattr__(self, lvl):
        return self._p(lvl)


class Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m)


class Param:
    def __init__(self, v):
        self.value = v


class Node:
    def __init__(self, name):
        self._params, self.pubs, self.subs, self.srvs, self.logs = {}, {}, {}, {}, []

    def declare_parameter(self, n, v):
        self._params[n] = v

    def get_parameter(self, n):
        return Param(self._params[n])

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = Pub(topic)
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_service(self, typ, name, cb):
        self.srvs[name] = cb

    def create_timer(self, period, cb):
        pass

    def get_logger(self):
        return Logger(self.logs)

    def get_clock(self):
        return Clock()

    def destroy_node(self):
        pass


mod("rclpy", ok=lambda: True, init=lambda args=None: None, spin=lambda n: None, try_shutdown=lambda: None)
mod("rclpy.node", Node=Node)
mod("rclpy.duration", Duration=Duration)
mod("rclpy.executors", ExternalShutdownException=RuntimeError)
mod(
    "rclpy.qos",
    QoSProfile=lambda **k: None,
    QoSDurabilityPolicy=Msg(VOLATILE=0),
    QoSReliabilityPolicy=Msg(BEST_EFFORT=0, RELIABLE=1),
)
mod("diagnostic_msgs")
mod(
    "diagnostic_msgs.msg",
    DiagnosticArray=msgclass("DiagnosticArray", {"status": list}),
    DiagnosticStatus=type("DS", (Msg,), {"OK": 0, "WARN": 1, "ERROR": 2}),
    KeyValue=Msg,
)
mod("sensor_msgs")
mod(
    "sensor_msgs.msg",
    Imu=msgclass(
        "Imu",
        {
            "orientation_covariance": lambda: [0.0] * 9,
            "angular_velocity": lambda: Msg(x=0.0, y=0.0, z=0.0),
            "angular_velocity_covariance": lambda: [0.0] * 9,
            "linear_acceleration_covariance": lambda: [0.0] * 9,
        },
    ),
)
mod("std_srvs")
mod("std_srvs.srv", Trigger=object)
mod("amr_interfaces")
EV = msgclass("Event", {})
EV.INFO, EV.WARN, EV.ERROR = 0, 1, 2
mod(
    "amr_interfaces.msg",
    ControlLease=Msg,
    DriveStatus=msgclass("DriveStatus", {}),
    Event=EV,
    IoImage=msgclass("IoImage", {}),
    PanelState=msgclass("PanelState", {}),
    WheelStates=msgclass("WheelStates", {"left_counts": 0, "right_counts": 0, "counts_valid": False}),
    WheelVelocities=Msg,
)

# ---------------------------------------------------------------- hardware sims
import can  # noqa: E402

from amr_base import qr_base_node as qbn  # noqa: E402
from amr_base.agv_repo import config  # noqa: E402

state = {
    "volts": [0.0, 0.0],
    "do": [False] * 16,
    "di": [False] * 16,
    "w": [0.0, 0.0],
    "pos": [1000.0, 2000.0],
}
state["di"][0] = True  # E-stop NC closed = released
lock = threading.Lock()


class FakeDio:
    def __init__(self):
        self.want = {}
        self.t0 = time.monotonic()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            now = time.monotonic()
            with lock:
                for ch, (v, until) in list(self.want.items()):
                    state["do"][ch] = bool(v) and now < until
                # interlock check: FWD and REV of a wheel never both high
                for f, r in ((4, 5), (1, 2)):
                    assert not (state["do"][f] and state["do"][r]), "FWD and REV both high!"
            time.sleep(0.03)

    def set_coil(self, ch, v, hold):
        self.want[ch] = (v, time.monotonic() + hold)

    def snapshot(self):
        with lock:
            return {
                "comms_ok": True,
                "di": list(state["di"]),
                "do": list(state["do"]),
                "rx_age_s": 0.01,
                "di_names": config.DIO_DI_NAMES,
                "do_names": config.DIO_DO_NAMES,
                "commanded": {},
                "scans": 1,
                "errors": 0,
                "writes": 0,
                "detail": "sim",
            }

    def stop(self):
        with lock:
            for ch in self.want:
                state["do"][ch] = False


class FakeModbus:
    """The analog module's Modbus side; the REAL AnalogOut runs on top of it (its skip /
    refresh logic is what the node's I/O interlock has to live with)."""

    class _Ok:
        @staticmethod
        def isError():
            return False

    def connect(self):
        return True

    def close(self):
        pass

    def _set(self, addr, val):
        ch = addr - config.QR_AO_REGISTER_BASE
        i = 0 if ch == config.QR_AO_CH_LEFT else 1
        with lock:
            state["volts"][i] = val / config.QR_AO_COUNTS_PER_VOLT

    def write_registers(self, addr, vals, device_id=None):
        for k, v in enumerate(vals):
            self._set(addr + k, v)
        return self._Ok()

    def write_register(self, addr, val, device_id=None):
        self._set(addr, val)
        return self._Ok()


class FakeAO(qbn.AnalogOut):
    def __init__(self, *a, **k):
        super().__init__(*a, client_factory=FakeModbus, **k)


class FakeBus:
    """Encoders: TPDO1 every 20 ms after NMT start; SDO downloads acked."""

    def __init__(self):
        self.rx, self.started, self.t = [], set(), time.monotonic()
        threading.Thread(target=self._plant, daemon=True).start()

    def _plant(self):
        # left wheel: FWD coil 4 / REV 5, ch left; right: FWD 1 / REV 2, mirrored (invert_right).
        # Integrated over the REAL elapsed time: a fixed dt drifts from the wall clock the
        # node's encoder speed is measured against as soon as sleep() runs long (slow VM).
        k = 1000.0 * 2 * math.pi / 60 / 30  # the AMR QR motor: ~970 r/min per V (ff, 2026-09-24)
        t_last = time.monotonic()
        t_tpdo = t_last
        while True:
            time.sleep(0.005)
            now = time.monotonic()
            dt, t_last = now - t_last, now
            with lock:
                for i, (f, r, drv) in enumerate(((4, 5, 1.0), (1, 2, -1.0))):
                    d = 1 if state["do"][f] else -1 if state["do"][r] else 0
                    ss = drv * d * max(0.0, state["volts"][i] - 0.22) * k
                    state["w"][i] += (ss - state["w"][i]) * min(1.0, dt / 0.15)
                    # encoder raw counts as mounted: a mirrored unit (enc_invert_*) counts
                    # backwards for vehicle-forward - the real right one does
                    sign = -1.0 if (config.QR_ENC_INVERT_LEFT, config.QR_ENC_INVERT_RIGHT)[i] else 1.0
                    state["pos"][i] += sign * state["w"][i] * dt * 8192 / (2 * math.pi)
                if now - t_tpdo >= 0.02:  # TPDO1 event timer
                    t_tpdo = now
                    for node, i in ((1, 0), (2, 1)):
                        if node in self.started:
                            raw = int(state["pos"][i]) % (1 << 24)
                            self.rx.append(
                                can.Message(arbitration_id=0x180 + node, data=struct.pack("<I", raw))
                            )

    def send(self, msg):
        if msg.arbitration_id == 0 and msg.data[0] == 1:
            self.started.add(msg.data[1])
        elif 0x600 < msg.arbitration_id < 0x680:
            d = bytes(msg.data)
            with lock:
                self.rx.append(
                    can.Message(
                        arbitration_id=0x580 + msg.arbitration_id - 0x600,
                        data=bytes([0x60]) + d[1:4] + bytes(4),
                    )
                )

    def recv(self, timeout=None):
        end = time.monotonic() + (timeout or 0)
        while True:
            with lock:
                if self.rx:
                    return self.rx.pop(0)
            if time.monotonic() >= end:
                return None
            time.sleep(0.001)

    def shutdown(self):
        pass


qbn.dio.DioLink = FakeDio
qbn.AnalogOut = FakeAO
qbn.open_bus = lambda *a: (FakeBus(), "sim")
qbn.refuse_if_legacy_running = lambda w: None

config.IMU_ENABLED = False
node = qbn.QrBaseNode()


def last(topic):
    msgs = node.pubs[topic].msgs
    return msgs[-1] if msgs else None


def press(ch, dur=0.2):
    with lock:
        state["di"][ch] = True
    time.sleep(dur)
    with lock:
        state["di"][ch] = False
    time.sleep(0.2)


def cmd(wl, wr, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        node._on_cmd(Msg(left_rad_s=wl, right_rad_s=wr, generation=0))
        time.sleep(0.02)


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (f"   {detail}" if detail else ""))
    if not cond:
        check.failed += 1


check.failed = 0

# idle at 0 V for longer than the analog refresh (0.5 s): the real AnalogOut skips
# unchanged writes, and the I/O interlock must not read that as a lost module
time.sleep(1.5)
st = last("/drives/status")
check(
    "armed and operational after start (idle past the analog refresh)",
    st is not None and st.operational,
    st and st.left_state,
)
ps = last("/amr/panel_state")
check("panel valid, MANUAL at boot", ps.valid and not ps.mode_auto)

cmd(2.0, 2.0, 2.5)
with lock:
    w, do, v = list(state["w"]), list(state["do"]), list(state["volts"])
check("left reaches 2 rad/s forward", abs(w[0] - 2.0) < 0.15, f"{w[0]:.3f}")
check(
    "right reaches 2 rad/s forward (mirrored: REV coil)",
    abs(w[1] - 2.0) < 0.15 and do[2] and not do[1],
    f"{w[1]:.3f} do1={do[1]} do2={do[2]}",
)
check("left FWD coil, brakes released", do[4] and not do[5] and not do[6] and not do[3])
ws = last("/wheel_states")
check(
    "wheel_states valid and forward",
    ws.left_valid and ws.right_valid and ws.left_vel_rad_s > 1.8 and ws.right_vel_rad_s > 1.8,
    f"{ws.left_vel_rad_s:.2f} {ws.right_vel_rad_s:.2f}",
)
check(
    "counts in driver terms (right negated)",
    ws.right_counts < 0 < ws.left_counts,
    f"{ws.left_counts} {ws.right_counts}",
)

cmd(-1.0, -1.0, 2.5)
with lock:
    w = list(state["w"])
check("reversal reaches -1 rad/s", abs(w[0] + 1.0) < 0.15 and abs(w[1] + 1.0) < 0.15, f"{w}")

time.sleep(0.5)  # watchdog: command stops -> rest with brake
with lock:
    do, v = list(state["do"]), list(state["volts"])
check("command timeout -> 0 V, both brakes", v == [0.0, 0.0] and do[6] and do[3], f"{v} {do[:7]}")

press(1)
check("START in MANUAL -> AUTO", last("/amr/panel_state").mode_auto)
n0 = sum(m.start_edge for m in node.pubs["/amr/panel_state"].msgs)
press(1)
n1 = sum(m.start_edge for m in node.pubs["/amr/panel_state"].msgs)
check("START in AUTO -> one Start edge", n1 - n0 == 1)
press(2)
check("STOP in AUTO -> MANUAL", not last("/amr/panel_state").mode_auto)

with lock:
    state["di"][0] = False  # E-stop pressed (NC opens)
threading.Thread(target=cmd, args=(2.0, 2.0, 1.0)).start()
time.sleep(0.5)
st = last("/drives/status")
with lock:
    v = list(state["volts"])
check(
    "E-stop: not operational, 0 V despite a command",
    not st.operational and st.left_state == "E-STOP" and v == [0.0, 0.0],
)
time.sleep(0.6)
with lock:
    state["di"][0] = True
time.sleep(0.4)
check("E-stop released: operational again", last("/drives/status").operational)

node.stop()
with lock:
    do, v = list(state["do"]), list(state["volts"])
check("shutdown: 0 V and every coil low", v == [0.0, 0.0] and not any(do), f"{v} {do[:7]}")
check("diagnostics published", len(node.pubs["/diagnostics"].msgs) > 0)
errs = [m for lvl, m in node.logs if lvl in ("error", "fatal")]
check("no errors logged", not errs, str(errs[:3]))
print(f"\n{check.failed} failed")
sys.exit(1 if check.failed else 0)
