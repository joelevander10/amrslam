#!/usr/bin/env python3
"""P7 (unified plan §12.2): one unified workflow and a fresh restart of a layer.

Opt-in (AMR_SIM_TESTS=1): boots the supervisor in simulation on its own
domain and drives IDLE -> survey -> returned -> save -> IDLE -> NAVIGATION on
the saved revision -> IDLE -> survey -> abort -> IDLE, then kills a required
node of a running layer and recovers. Asserts unchanged base PIDs, generation
rotation, the lease at every stable mode, and zero descendants after SIGINT.
~3 min on the N97.
"""

import os
import re
import signal
import subprocess

import pytest

if os.environ.get("AMR_SIM_TESTS") != "1":
    pytest.skip("simulation supervisor test; set AMR_SIM_TESTS=1", allow_module_level=True)
os.environ["ROS_DOMAIN_ID"] = "68"
os.environ["AMR_SUPERVISOR_SIM"] = "1"
import math
import sys
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import SetBool, Trigger

from amr_interfaces.msg import (
    ControlLease,
    LocalizationState,
    ManualCommand,
    ModeState,
    MuxState,
    RunState,
    WheelVelocities,
)
from amr_interfaces.srv import GetOperation, RequestMode, RequestSurvey, RunMission, SetPose2D

R1 = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
NAMES = ["STARTING", "IDLE", "MAPPING", "NAVIGATION", "TRANSITIONING", "FAULT", "STOPPING"]


class Probe(Node):
    def __init__(self):
        super().__init__("p7_probe")
        self.mode = None
        self.lease = None
        self.mux = None
        self.wheel = 0.0
        self.create_subscription(ModeState, "/amr/mode_state", lambda m: setattr(self, "mode", m), LATCHED)
        self.create_subscription(ControlLease, "/amr/control_lease", lambda m: setattr(self, "lease", m), R1)
        self.create_subscription(MuxState, "/amr/mux_state", lambda m: setattr(self, "mux", m), R1)
        self.create_subscription(
            WheelVelocities, "/cmd_wheel_vel", lambda m: setattr(self, "wheel", m.left_rad_s), R1
        )
        self.jog = self.create_publisher(ManualCommand, "/amr/manual_command", R1)
        self.c_mode = self.create_client(RequestMode, "/amr/mode/request")
        self.c_survey = self.create_client(RequestSurvey, "/amr/supervisor/survey")
        self.c_op = self.create_client(GetOperation, "/amr/operations/get")
        self.c_recover = self.create_client(Trigger, "/amr/supervisor/recover")
        self.seq = 0
        self.loc = None
        self.run = None
        self.create_subscription(
            LocalizationState, "/amr/localization_state", lambda m: setattr(self, "loc", m), LATCHED
        )
        self.create_subscription(RunState, "/amr/run_state", lambda m: setattr(self, "run", m), LATCHED)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.c_confirm = self.create_client(Trigger, "/amr/localization/confirm")
        self.c_panel_mode = self.create_client(SetBool, "/sim/panel/set_mode")
        self.c_start = self.create_client(Trigger, "/sim/panel/press_start")
        self.c_teleport = self.create_client(SetPose2D, "/sim/set_pose")
        self.c_run = self.create_client(RunMission, "/amr/run_mission")
        self.c_ack = self.create_client(Trigger, "/amr/ack_fault")

    def wait_until(self, pred, timeout, what):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        raise AssertionError(f"timeout: {what}")

    def spin(self, s):
        t = time.monotonic()
        while time.monotonic() - t < s:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_mode(self, want, timeout):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.mode is not None and self.mode.mode == want:
                return True
            if self.mode is not None and self.mode.mode == 5:
                print(f"   FAULT: {self.mode.fault_code} {self.mode.reason}")
                return False
        now = NAMES[self.mode.mode] if self.mode else None
        print(f"   timeout waiting for {NAMES[want]}; now {now} phase={self.mode.phase if self.mode else ''}")
        return False

    def call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{client.srv_name} unavailable")
        f = client.call_async(req)
        t = time.monotonic()
        while not f.done() and time.monotonic() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return f.result()

    def op(self, oid):
        r = self.call(self.c_op, GetOperation.Request(operation_id=oid))
        return r.state if r.found else None

    def wait_op(self, oid, timeout):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            st = self.op(oid)
            if st is not None and st.status != 0:
                return st
            self.spin(0.2)
        return self.op(oid)

    def survey(self, operation, map_id="", desc=""):
        req = RequestSurvey.Request()
        req.request_id = uuid.uuid4().hex
        req.expected_instance = self.mode.instance
        req.expected_generation = self.mode.generation
        req.operation = operation
        req.map_id = map_id
        req.description = desc
        r = self.call(self.c_survey, req)
        print(f"   survey op {operation}: accepted={r.accepted} {r.message} op={r.operation_id}")
        return r

    def request_mode(self, target, map_id="", rev=0):
        req = RequestMode.Request()
        req.request_id = uuid.uuid4().hex
        req.expected_instance = self.mode.instance
        req.expected_generation = self.mode.generation
        req.target = target
        req.map_id = map_id
        req.map_revision = rev
        r = self.call(self.c_mode, req)
        print(f"   mode {NAMES[target]}: accepted={r.accepted} {r.message} op={r.operation_id}")
        return r

    def jog_for(self, seconds, v=0.2, w=0.0):
        t = time.monotonic()
        peak = 0.0
        while time.monotonic() - t < seconds:
            m = ManualCommand()
            m.header.stamp = self.get_clock().now().to_msg()
            m.instance = self.lease.instance
            m.generation = self.lease.generation
            m.session = "p7"
            self.seq += 1
            m.seq = self.seq
            m.valid_for_s = 0.2
            m.v, m.w = v, w
            self.jog.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)
            peak = max(peak, abs(self.wheel))
        return peak


BASE_PATTERN = "fake_base_node|cmd_mux_kinematics_node --ros|diff_drive_odom_node --ros|ekf_node"


def _proc_stat(pid: int) -> tuple[int, int] | None:
    """-> (ppid, session) from /proc, None if the process is gone."""
    try:
        with open(f"/proc/{pid}/stat", encoding="utf-8", errors="replace") as f:
            rest = f.read().rsplit(")", 1)[1].split()  # the comm field may contain spaces
        return int(rest[1]), int(rest[3])
    except (OSError, IndexError, ValueError):
        return None


def _cmdline(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def _owned(pid: int, root: int, table: dict[int, tuple[int, int]]) -> bool:
    """A descendant of the fixture's process, or still in the session it created.

    The session catches orphans after the root exits (the fixture starts the
    supervisor with start_new_session=True, so session id == root pid).
    """
    if pid == os.getpid() or pid not in table:
        return False
    if table[pid][1] == root:
        return True
    seen = set()
    while pid in table and pid not in seen and pid > 1:
        if pid == root:
            return True
        seen.add(pid)
        pid = table[pid][0]
    return False


def owned_pids(root: int, pattern: str) -> set[int]:
    """PIDs owned by the fixture's process tree whose command line matches `pattern`.

    Replaces host-wide pgrep/pkill: another stack on this host (another domain,
    or the vehicle) matching the same pattern is never seen or signalled.
    """
    table = {}
    for name in os.listdir("/proc"):
        if name.isdigit():
            st = _proc_stat(int(name))
            if st is not None:
                table[int(name)] = st
    rx = re.compile(pattern)
    return {pid for pid in table if _owned(pid, root, table) and rx.search(_cmdline(pid))}


def kill_owned(root: int, pattern: str, sig: int = signal.SIGKILL) -> set[int]:
    """Signal only fixture-owned matches, re-verifying ownership just before each signal."""
    killed = set()
    for pid in owned_pids(root, pattern):
        if pid in owned_pids(root, pattern):
            try:
                os.kill(pid, sig)
                killed.add(pid)
            except ProcessLookupError:
                pass
    return killed


def base_pids(root: int) -> set[int]:
    return owned_pids(root, BASE_PATTERN)


@pytest.fixture(scope="module")
def stack(tmp_path_factory):
    state = tmp_path_factory.mktemp("state")
    maps = state / "maps"
    maps.mkdir()
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "amr_bringup.supervisor_node",
            "--ros-args",
            "-p",
            "real:=false",
            "-p",
            "web:=false",
            "-p",
            "foxglove:=false",
            "-p",
            f"state_dir:={state}",
            "-p",
            f"maps_dir:={maps}",
        ],
        stdout=open(state / "supervisor.log", "wb"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    p = Probe()
    yield p, proc, state
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=40)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
    rclpy.shutdown()


def settled_lease(p: Probe, expect_allowed: int):
    p.spin(0.5)  # the lease lags a mode change by up to one loop period
    assert p.lease is not None and p.lease.allowed == expect_allowed, (p.lease.allowed, expect_allowed)
    assert p.lease.generation == p.mode.generation


def test_unified_workflow_and_layer_restart(stack):
    p, proc, state = stack
    assert p.wait_mode(1, 60), "IDLE"
    settled_lease(p, 1)
    pids0 = base_pids(proc.pid)
    assert len(pids0) >= 4
    gen0 = p.mode.generation

    # survey -> returned -> save
    r = p.survey(0, "p7_map", "probe mark")
    assert r.accepted and p.wait_mode(2, 60)
    settled_lease(p, 1)
    assert p.wait_op(r.operation_id, 5).status == 1
    assert p.mode.generation == gen0 + 1
    assert p.jog_for(3.0, 0.25, 0.0) > 1.0  # the base moves under MAPPING/MANUAL
    p.jog_for(4.0, 0.0, 0.5)
    p.jog_for(3.0, 0.25, 0.0)
    p.spin(2.0)
    r = p.survey(1)
    st = p.wait_op(r.operation_id, 10)
    assert st.status == 1 and "closure" in st.message
    r = p.survey(2, desc="note")
    assert r.accepted
    st = p.wait_op(r.operation_id, 90)
    assert st.status == 1 and st.map_revision == 1 and st.map_sha256, st.message
    assert p.wait_mode(1, 40)
    settled_lease(p, 1)
    assert (p.mode.last_survey_map_id, p.mode.last_survey_revision) == ("p7_map", 1)

    # navigation on that exact revision, then idle
    r = p.request_mode(3, "p7_map", 1)
    assert r.accepted and p.wait_mode(3, 90)
    settled_lease(p, 3)
    assert (p.mode.active_map_id, p.mode.active_map_revision) == ("p7_map", 1)
    r = p.request_mode(1)
    assert r.accepted and p.wait_mode(1, 40)
    settled_lease(p, 1)
    assert p.mode.active_map_id == ""

    # one short route on the verified world bundle, under the supervisor's lease:
    # executor permit (instance + generation) -> mux -> fake base
    from amr_mission.fixtures import write_world_as_bundle  # noqa: PLC0415
    from amr_navigation.route import Limits, MapRef, Route, StartPose, Step  # noqa: PLC0415

    from amr_mission import map_bundle as mb  # noqa: PLC0415
    from amr_navigation import store  # noqa: PLC0415

    maps = str(state / "maps")
    write_world_as_bundle(maps)
    manifest = mb.verify(mb.revision_dir(maps, "sim_factory", 1))
    route = Route(
        "p7",
        MapRef("sim_factory", 1, manifest.sha256),
        StartPose(0.0, 0.0, 0.0),
        [Step("s1", "straight", to=(2.0, 0.0))],
        Limits(),
        repeat_count=1,
    )
    rrev, _, rsha = store.save_route(maps, route)
    store.save_mission(maps, "m_p7", "sim_factory", 1, manifest.sha256, "p7", rrev, rsha)
    r = p.request_mode(3, "sim_factory", 1)
    assert r.accepted and p.wait_mode(3, 90)
    settled_lease(p, 3)
    p.call(p.c_teleport, SetPose2D.Request(x_m=0.0, y_m=0.0, yaw_rad=0.0))
    p.spin(1.0)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = "map"
    m.pose.pose.orientation.w = 1.0
    m.pose.covariance[0] = m.pose.covariance[7] = 0.3**2
    m.pose.covariance[35] = math.radians(10) ** 2
    for _ in range(3):
        p.init_pub.publish(m)
        p.spin(0.2)
    p.jog_for(1.2, 0.15, 0.0)
    p.spin(0.5)
    p.wait_until(
        lambda: p.loc is not None and p.loc.generation == p.mode.generation and p.loc.can_confirm,
        40,
        "can_confirm",
    )
    assert p.call(p.c_confirm, Trigger.Request()).success
    p.jog_for(1.2, -0.15, 0.0)
    p.spin(1.0)
    r = p.call(p.c_run, RunMission.Request(mission_id="m_p7"))
    assert r.accepted, r.message
    p.wait_until(lambda: p.run is not None and p.run.state == 1, 20, "READY")
    p.call(p.c_panel_mode, SetBool.Request(data=True))  # AUTO
    p.spin(0.5)
    p.call(p.c_start, Trigger.Request())
    p.wait_until(lambda: p.run is not None and p.run.state == 2, 20, "EXECUTING")
    moved = p.wait_until(lambda: abs(p.wheel) > 0.5, 20, "wheels turning under the executor permit")
    assert moved
    # Mechanism, not tolerance (sim tests are mechanism checks; tolerances are set on
    # hardware): the run must END - DONE, or FAULT from the executor's own tolerance
    # guards (e.g. "passed the endpoint", seen once in simulation when AMCL's along-track
    # estimate plus the stopping distance exceeded 0.10 m). Either way the output is zero
    # afterwards and a FAULT is acknowledged before the next mode change.
    p.wait_until(lambda: p.run is not None and p.run.state in (5, 6), 90, "run ended (DONE or FAULT)")
    if p.run.state == 5:
        tolerance_faults = ("passed the endpoint", "endpoint missed", "cross-track", "heading")
        assert any(t in p.run.reason for t in tolerance_faults), f"unexpected executor fault: {p.run.reason}"
        print(f"   route ended in a tolerance FAULT (accepted in sim): {p.run.reason}")
        p.wait_until(lambda: abs(p.wheel) < 1e-6, 5, "zero output after the fault")
        assert p.call(p.c_ack, Trigger.Request()).success
        p.wait_until(lambda: p.run is not None and p.run.state == 0, 10, "IDLE after acknowledge")
    p.call(p.c_panel_mode, SetBool.Request(data=False))  # back to MANUAL for the next transition
    p.spin(1.0)
    r = p.request_mode(1)
    assert r.accepted and p.wait_mode(1, 40)

    # a second survey is a fresh layer; abort returns to idle
    r = p.survey(0, "p7_map", "again")
    assert r.accepted and p.wait_mode(2, 60)
    r = p.survey(3)
    assert r.accepted and p.wait_mode(1, 40)

    # required-node failure in a live layer -> FAULT, inhibited; recover -> IDLE
    r = p.survey(0, "p7_map", "kill")
    assert r.accepted and p.wait_mode(2, 60)
    assert kill_owned(proc.pid, "async_slam_toolbox_node"), "no fixture-owned slam_toolbox to kill"
    assert p.wait_mode(5, 20)
    settled_lease(p, 0)
    assert p.mode.fault_code == "LAYER_EXITED"
    r = p.call(p.c_recover, Trigger.Request())
    assert r.success and p.wait_mode(1, 40)
    settled_lease(p, 1)

    assert base_pids(proc.pid) == pids0, "base layer must survive every transition"
    assert p.mode.generation > gen0 + 6


def test_shutdown_leaves_no_descendants(stack):
    p, proc, state = stack
    proc.send_signal(signal.SIGINT)
    proc.wait(timeout=40)
    p.spin(1.0)
    assert base_pids(proc.pid) == set()
    assert not owned_pids(proc.pid, "slam_toolbox|amcl|map_server|base.launch")
