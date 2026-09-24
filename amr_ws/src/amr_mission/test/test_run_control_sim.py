"""T8 mechanism tests (spec §7, §8 "Permission", "Pause/abort/resume", "Obstacles", "Failures", "Watchdogs").

One nav.launch.py sim, a short route, several scenarios in sequence. Each checks
a mechanism, not a tolerance:
  1. permission: no Start -> no motion; Start in MANUAL ignored; AUTO + Start runs
  2. pause mid-straight -> wheels zero fast, permit NONE; prepare-resume + Start -> continues
  3. obstacle on the line -> BLOCKED; removal alone does not resume; prepare + Start does
  4. pause mid-turn -> resumes the REMAINING angle; total travel is the drawn magnitude
  5. selector to MANUAL mid-run -> abort (IDLE)
  6. panel lost mid-run -> FAULT; acknowledge -> IDLE; a mission with a wrong hash is refused
"""

import os

os.environ["ROS_DOMAIN_ID"] = "67"

import pytest  # noqa: E402

if os.environ.get("AMR_SIM_TESTS") != "1":
    pytest.skip("simulation launch test; set AMR_SIM_TESTS=1", allow_module_level=True)

import math  # noqa: E402
import shutil  # noqa: E402
import tempfile  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import unittest  # noqa: E402

import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.markers  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from amr_mission.fixtures import write_world_as_bundle  # noqa: E402
from amr_navigation.route import Limits, MapRef, Route, StartPose, Step  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist  # noqa: E402
from launch import LaunchDescription  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.launch_description_sources import AnyLaunchDescriptionSource  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402

from amr_interfaces.msg import LocalizationState, MotionPermit, RunState, WheelStates  # noqa: E402
from amr_interfaces.srv import AddObstacle, RunMission, SetPose2D  # noqa: E402
from amr_mission import map_bundle as mb  # noqa: E402
from amr_navigation import store  # noqa: E402

MAPS_DIR = tempfile.mkdtemp(prefix="amr_ctl_test_")
SENSOR_DATA = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
S = {
    RunState.IDLE: "IDLE",
    RunState.READY: "READY",
    RunState.EXECUTING: "EXECUTING",
    RunState.PAUSED: "PAUSED",
    RunState.BLOCKED: "BLOCKED",
    RunState.FAULT: "FAULT",
    RunState.DONE: "DONE",
}


def make_route(sha: str) -> Route:
    """One long straight and one long turn, so there is room to pause inside each."""
    s = [
        Step("s1", "straight", to=(8.0, 0.0)),
        Step("s2", "rotate", direction="ccw", angle_deg=270),  # E -> S the long way
        Step("s3", "rotate", direction="ccw", angle_deg=90),  # S -> E
    ]
    return Route("ctl", MapRef("sim_factory", 1, sha), StartPose(0.0, 0.0, 0.0), s, Limits(), repeat_count=1)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    rev_dir = write_world_as_bundle(MAPS_DIR, "sim_factory")
    manifest = mb.verify(rev_dir)
    rev, path, sha = store.save_route(MAPS_DIR, make_route(manifest.sha256))
    store.save_mission(MAPS_DIR, "m_ctl", "sim_factory", 1, manifest.sha256, "ctl", rev, sha)
    # a mission whose manifest lies about the route hash
    bad = os.path.join(store.missions_dir(MAPS_DIR), "m_bad.yaml")
    with open(bad, "w") as fh:
        yaml.safe_dump(
            {
                "mission_id": "m_bad",
                "map": {"id": "sim_factory", "revision": 1, "sha256": manifest.sha256},
                "route": {"id": "ctl", "revision": rev, "sha256": "0" * 64},
            },
            fh,
        )
    nav = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("amr_bringup"), "launch", "nav.launch.py")
        ),
        launch_arguments={
            "sim": "true",
            "maps_dir": MAPS_DIR,
            "map_id": "sim_factory",
            "revision": "1",
            "slip_noise_std": "0.02",
            "clutter_count": "0",
            "web": "false",
        }.items(),
    )
    return LaunchDescription([nav, launch_testing.actions.ReadyToTest()]), {}


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class TestRunControl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("ctl_test")
        cls.lock = threading.RLock()  # predicates in _wait re-enter it via _truth()
        cls.truth = None
        cls.loc = None
        cls.run_state = None
        cls.permit = None
        cls.wheels = None
        cls.wheels_t = 0.0
        cls.log = []

        def keep(attr):
            def cb(m):
                with cls.lock:
                    setattr(cls, attr, m)
                    if attr == "wheels":
                        cls.wheels_t = time.time()
                    if attr == "run_state" and (not cls.log or cls.log[-1][0] != m.state):
                        cls.log.append((m.state, m.reason, time.time()))

            return cb

        cls.node.create_subscription(Odometry, "/sim/ground_truth", keep("truth"), SENSOR_DATA)
        cls.node.create_subscription(LocalizationState, "/amr/localization_state", keep("loc"), LATCHED)
        cls.node.create_subscription(RunState, "/amr/run_state", keep("run_state"), LATCHED)
        cls.node.create_subscription(MotionPermit, "/amr/motion_permit", keep("permit"), 1)
        cls.node.create_subscription(WheelStates, "/wheel_states", keep("wheels"), SENSOR_DATA)
        cls.init_pub = cls.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        cls.teleop = cls.node.create_publisher(Twist, "/cmd_vel_teleop", 1)
        cls.cli = {
            "confirm": cls.node.create_client(Trigger, "/amr/localization/confirm"),
            "mode": cls.node.create_client(SetBool, "/sim/panel/set_mode"),
            "valid": cls.node.create_client(SetBool, "/sim/panel/set_valid"),
            "start": cls.node.create_client(Trigger, "/sim/panel/press_start"),
            "run": cls.node.create_client(RunMission, "/amr/run_mission"),
            "pause": cls.node.create_client(Trigger, "/amr/pause"),
            "resume": cls.node.create_client(Trigger, "/amr/resume"),
            "abort": cls.node.create_client(Trigger, "/amr/abort"),
            "ack": cls.node.create_client(Trigger, "/amr/ack_fault"),
            "add": cls.node.create_client(AddObstacle, "/sim/add_obstacle"),
            "clear": cls.node.create_client(Trigger, "/sim/clear_obstacles"),
            "teleport": cls.node.create_client(SetPose2D, "/sim/set_pose"),
        }
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(MAPS_DIR, ignore_errors=True)

    # ---- helpers ------------------------------------------------------------------------

    def _call(self, name, req=None, timeout=20.0):
        c = self.cli[name]
        self.assertTrue(c.wait_for_service(timeout_sec=30.0), f"{c.srv_name} unavailable")
        fut = c.call_async(req or Trigger.Request())
        t0 = time.time()
        while not fut.done():
            self.assertLess(time.time() - t0, timeout, f"{c.srv_name} timed out")
            time.sleep(0.05)
        return fut.result()

    def _state(self):
        with self.lock:
            return self.run_state.state if self.run_state else None

    def _wait(self, pred, timeout, what):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if pred():
                    return time.time() - t0
            time.sleep(0.05)
        with self.lock:
            r = self.run_state
        self.fail(f"timeout: {what}; run={S.get(r.state) if r else None} {r.reason if r else ''}")

    def _wait_state(self, st, timeout, what=""):
        return self._wait(
            lambda: self.run_state is not None and self.run_state.state == st, timeout, what or S[st]
        )

    def _drive(self, v, seconds):
        tw = Twist()
        tw.linear.x = v
        for _ in range(int(seconds / 0.05)):
            self.teleop.publish(tw)
            time.sleep(0.05)
        self.teleop.publish(Twist())

    def _truth(self):
        with self.lock:
            p = self.truth.pose.pose
            return p.position.x, p.position.y, _yaw(p.orientation)

    def _wheels_moving(self):
        with self.lock:
            w = self.wheels
        return w is not None and (abs(w.left_vel_rad_s) > 0.05 or abs(w.right_vel_rad_s) > 0.05)

    def _localize_at_mark(self):
        """Sim stand-in for "reposition manually": put reality on the mark, then relocalise."""
        self._call("mode", SetBool.Request(data=False))
        self._call("teleport", SetPose2D.Request(x_m=0.0, y_m=0.0, yaw_rad=0.0))
        time.sleep(1.0)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        x, y, yaw = self._truth()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.3**2
        m.pose.covariance[35] = math.radians(10) ** 2
        for _ in range(3):
            self.init_pub.publish(m)
            time.sleep(0.2)
        self._drive(0.15, 1.2)
        time.sleep(0.5)
        self._wait(lambda: self.loc is not None and self.loc.can_confirm, 40, "can_confirm")
        r = self._call("confirm")
        self.assertTrue(r.success, r.message)
        self._drive(-0.15, 1.2)  # sim only: back onto the mark
        time.sleep(1.0)

    def _load(self, mission="m_ctl"):
        r = self._call("run", RunMission.Request(mission_id=mission))
        return r

    # ---- scenarios ------------------------------------------------------------------------

    def test_scenarios(self):
        self._wait(lambda: self.loc is not None and self.run_state is not None, 60, "monitor + executor")
        time.sleep(4.0)
        self._localize_at_mark()

        # 6b. a mission whose manifest hash does not match the route file is refused at load
        r = self._load("m_bad")
        self.assertFalse(r.accepted)
        self.assertIn("hash", r.message)

        # 1. permission
        r = self._load()
        self.assertTrue(r.accepted, r.message)
        self._wait_state(RunState.READY, 10)
        time.sleep(3.0)
        self.assertEqual(self._state(), RunState.READY, "no Start: nothing may happen")
        self.assertFalse(self._wheels_moving())
        self._call("start")  # selector is MANUAL: ignored
        time.sleep(1.5)
        self.assertEqual(self._state(), RunState.READY)
        self.assertFalse(self._wheels_moving())
        self._call("mode", SetBool.Request(data=True))
        time.sleep(0.5)
        self._call("start")
        self._wait_state(RunState.EXECUTING, 10)
        self._wait(lambda: self._wheels_moving(), 10, "wheels moving")
        print("1. permission ok")

        # 2. pause mid-straight
        self._wait(lambda: self.truth is not None and self._truth()[0] > 1.5, 30, "1.5 m along")
        r = self._call("pause")
        self.assertTrue(r.success, r.message)
        stop_dt = self._wait(lambda: not self._wheels_moving(), 3, "wheels zero after pause")
        self.assertEqual(self._state(), RunState.PAUSED)
        with self.lock:
            permit = self.permit
        self.assertEqual(permit.source, MotionPermit.NONE)
        print(f"2. paused: wheels zero {stop_dt * 1000:.0f} ms after the pause reply")
        self.assertLess(stop_dt, 1.0)  # ramp-down of the slewed setpoint after an immediate zero permit
        self._call("start")  # not prepared: ignored
        time.sleep(1.0)
        self.assertEqual(self._state(), RunState.PAUSED)
        r = self._call("resume")
        self.assertTrue(r.success, r.message)
        self._call("start")
        self._wait_state(RunState.EXECUTING, 10)
        print("2. resumed")

        # 3. obstacle on the line ahead -> BLOCKED; removal alone does not resume
        x = self._truth()[0]
        self._call("add", AddObstacle.Request(x_m=min(7.0, x + 1.2), y_m=0.0, size_m=0.6))
        self._wait_state(RunState.BLOCKED, 10)
        self._wait(lambda: not self._wheels_moving(), 3, "wheels zero when blocked")
        r = self._call("resume")
        self.assertFalse(r.success, "resume must be refused while the obstacle is there")
        self._call("clear")
        time.sleep(2.5)  # clearance must be stable 1 s; still BLOCKED without Start
        self.assertEqual(self._state(), RunState.BLOCKED)
        r = self._call("resume")
        self.assertTrue(r.success, r.message)
        self._call("start")
        self._wait_state(RunState.EXECUTING, 10)
        print("3. obstruction ok")

        # 4. pause mid-turn -> resume the remaining angle
        self._wait(
            lambda: (
                self.run_state is not None
                and self.run_state.step_index == 1
                and self.run_state.step_type == "rotate"
            ),
            60,
            "turn step",
        )
        yaw0 = self._truth()[2]
        self._wait(
            lambda: (
                abs(
                    math.degrees(
                        math.atan2(math.sin(self._truth()[2] - yaw0), math.cos(self._truth()[2] - yaw0))
                    )
                )
                > 60
            ),
            30,
            "60 deg into the turn",
        )
        self._call("pause")
        self._wait(lambda: not self._wheels_moving(), 3, "stopped in the turn")
        with self.lock:
            remaining = self.run_state.remaining_turn_rad
        print(f"4. paused in the turn with {math.degrees(remaining):.1f} deg remaining of 270")
        self.assertLess(math.degrees(remaining), 215)
        self.assertGreater(math.degrees(remaining), 100)
        r = self._call("resume")
        self.assertTrue(r.success, r.message)
        self._call("start")
        self._wait(lambda: self.run_state is not None and self.run_state.step_index >= 2, 60, "turn finished")
        yaw_end = self._truth()[2]
        turned = math.degrees(math.atan2(math.sin(yaw_end - yaw0), math.cos(yaw_end - yaw0)))
        print(f"4. net heading change {turned:+.1f} deg (expected -90 for a ccw 270)")
        self.assertLess(abs(turned - (-90.0)), 5.0)
        self._wait_state(RunState.DONE, 60)
        print("4. DONE")

        # 5a. start gate: 8 m from the route start, Start must be refused (spec §7.1)
        r = self._load()
        self.assertTrue(r.accepted, r.message)
        self._wait_state(RunState.READY, 10)
        self._call("mode", SetBool.Request(data=True))
        time.sleep(0.5)
        self._call("start")
        time.sleep(1.5)
        with self.lock:
            st = self.run_state
        self.assertEqual(st.state, RunState.READY)
        self.assertIn("Start refused", st.reason)
        print(f"5a. start gate: {st.reason}")

        # 5. manual takeover mid-run -> abort
        self._localize_at_mark()
        r = self._load()
        self.assertTrue(r.accepted, r.message)
        self._wait_state(RunState.READY, 10)
        self._call("mode", SetBool.Request(data=True))
        time.sleep(0.5)
        self._call("start")
        self._wait_state(RunState.EXECUTING, 10)
        self._wait(lambda: self._wheels_moving(), 10, "moving")
        self._call("mode", SetBool.Request(data=False))
        self._wait_state(RunState.IDLE, 5, "IDLE after manual takeover")
        self._wait(lambda: not self._wheels_moving(), 3, "stopped after takeover")
        print("5. manual takeover -> IDLE")

        # 6. panel lost mid-run -> FAULT; ack -> IDLE
        self._localize_at_mark()
        r = self._load()
        self.assertTrue(r.accepted, r.message)
        self._wait_state(RunState.READY, 10)
        self._call("mode", SetBool.Request(data=True))
        time.sleep(0.5)
        self._call("start")
        self._wait_state(RunState.EXECUTING, 10)
        self._wait(lambda: self._wheels_moving(), 10, "moving")
        self._call("valid", SetBool.Request(data=False))
        dt = self._wait_state(RunState.FAULT, 3, "FAULT on panel loss")
        self._wait(lambda: not self._wheels_moving(), 3, "stopped on fault")
        print(f"6. panel lost -> FAULT in {dt * 1000:.0f} ms")
        self._call("valid", SetBool.Request(data=True))
        time.sleep(0.5)
        self._call("start")
        time.sleep(1.0)
        self.assertEqual(self._state(), RunState.FAULT, "Start must not resume a fault")
        r = self._call("ack")
        self.assertTrue(r.success)
        self._wait_state(RunState.IDLE, 5)
        print("6. fault acknowledged -> IDLE")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
