"""T5 acceptance (spec §4.2, §4.3, §10 T5): localise on a saved bundle, detect loss, recover stopped.

nav.launch.py sim with the world written as a bundle and 12 clutter boxes the
reference map does not contain. Sequence:

  1. /initialpose at the start mark with a deliberate +0.3 m / +10 deg error -> CHECKING
  2. drive the survey loop (2 % slip); once converged and settled, confirm -> READY
  3. AMCL pose vs ground truth over the second half of the loop: p95 < 0.10 m / 2 deg
  4. teleport reality 2 m away, facing backwards; drive 1 m -> LOST within a few seconds
  5. stop, new /initialpose at the true pose, drive slowly, confirm -> READY again
"""

import os

# Every launch test gets its own DDS domain so `colcon test` may run packages in
# parallel without simulations talking to each other's /scan, /map or /cmd_vel_teleop.
# Set before rclpy and before launch forks the nodes, which inherit it.
os.environ["ROS_DOMAIN_ID"] = "65"

import pytest

if (
    os.environ.get("AMR_SIM_TESTS") != "1"
):  # ~1-5 min each on the N97: run deliberately, not on every colcon test
    pytest.skip("simulation launch test; set AMR_SIM_TESTS=1", allow_module_level=True)

import math
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from amr_mission.fixtures import write_world_as_bundle
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from amr_interfaces.msg import LocalizationState
from amr_interfaces.srv import SetPose2D

MAPS_DIR = tempfile.mkdtemp(prefix="amr_amcl_test_")
SENSOR_DATA = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    write_world_as_bundle(MAPS_DIR, "sim_factory")
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
            "clutter_count": "12",
        }.items(),
    )
    square = Node(
        package="amr_sim",
        executable="square_drive",
        name="square_drive",
        output="screen",
        parameters=[
            {
                "side_a_m": 21.0,
                "side_b_m": 4.8,
                "speed_mps": 0.6,
                "turn_rate_rad_s": 0.5,
                "settle_s": 1.0,
                "start_delay_s": 6.0,
            }
        ],
    )
    return LaunchDescription([nav, square, launch_testing.actions.ReadyToTest()]), {"square": square}


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def _stamp(h) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class TestAmcl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("amcl_test")
        cls.lock = threading.Lock()
        cls.truth_hist: list[tuple[float, float, float, float]] = []  # t, x, y, yaw
        cls.amcl_hist: list[tuple[float, float, float, float]] = []
        cls.state: LocalizationState | None = None

        def on_truth(m):
            with cls.lock:
                p = m.pose.pose
                cls.truth_hist.append((_stamp(m.header), p.position.x, p.position.y, _yaw(p.orientation)))
                if len(cls.truth_hist) > 20000:
                    del cls.truth_hist[:10000]

        def on_amcl(m):
            with cls.lock:
                p = m.pose.pose
                cls.amcl_hist.append((_stamp(m.header), p.position.x, p.position.y, _yaw(p.orientation)))

        def on_state(m):
            with cls.lock:
                cls.state = m

        cls.node.create_subscription(Odometry, "/sim/ground_truth", on_truth, SENSOR_DATA)
        cls.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl, 10)
        cls.node.create_subscription(LocalizationState, "/amr/localization_state", on_state, LATCHED)
        cls.init_pub = cls.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        cls.cmd_pub = cls.node.create_publisher(Twist, "/cmd_vel_teleop", 1)
        cls.confirm = cls.node.create_client(Trigger, "/amr/localization/confirm")
        cls.teleport = cls.node.create_client(SetPose2D, "/sim/set_pose")
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(MAPS_DIR, ignore_errors=True)

    # ---- helpers -------------------------------------------------------------

    def _call(self, client, req, timeout=20.0):
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), f"{client.srv_name} unavailable")
        fut = client.call_async(req)
        t0 = time.time()
        while not fut.done():
            self.assertLess(time.time() - t0, timeout, f"{client.srv_name} timed out")
            time.sleep(0.05)
        return fut.result()

    def _publish_initialpose(self, x, y, yaw, sigma_xy=0.5, sigma_yaw=0.26):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = sigma_xy**2
        m.pose.covariance[35] = sigma_yaw**2
        for _ in range(3):
            self.init_pub.publish(m)
            time.sleep(0.2)

    def _wait_state(self, predicate, timeout, what):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                st = self.state
            if st is not None and predicate(st):
                return st
            time.sleep(0.2)
        with self.lock:
            st = self.state
        self.fail(
            f"timeout waiting for {what}; last state {st.state if st else None}: {st.reason if st else ''}"
        )

    def _truth_at(self, t):
        with self.lock:
            hist = list(self.truth_hist)
        if not hist:
            return None
        ts = np.array([h[0] for h in hist])
        i = int(np.argmin(np.abs(ts - t)))
        return hist[i] if abs(ts[i] - t) < 0.05 else None

    def _drive(self, v, w, seconds):
        m = Twist()
        m.linear.x, m.angular.z = v, w
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.cmd_pub.publish(m)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())

    # ---- the test --------------------------------------------------------------

    def test_localize_lose_recover(self, proc_output, square):
        self._wait_state(lambda s: s.state == LocalizationState.UNLOCALIZED, 60, "monitor up")
        self.assertTrue(self.confirm.wait_for_service(timeout_sec=30.0))
        time.sleep(4.0)  # AMCL lifecycle up, IMU calibrating (the square starts after 6 s)

        # 1. deliberately wrong initial pose
        self._publish_initialpose(0.3, 0.0, math.radians(10.0))
        st = self._wait_state(lambda s: s.state == LocalizationState.CHECKING, 10, "CHECKING")
        res = self._call(self.confirm, Trigger.Request())
        self.assertFalse(res.success, "confirm must be refused before convergence and settling")

        # 2. the square drives; wait until the filter converged and settled. The operator
        #    confirms only when the scan visibly aligns; the test stands in for those eyes
        #    with ground truth, since a corridor can converge 0.3 m off along its axis.
        st = self._wait_state(lambda s: s.can_confirm, 60, "can_confirm")
        print(
            f"converged: cov xx {st.cov_xx:.4f} yy {st.cov_yy:.4f} yaw {st.cov_yaw:.5f}; "
            f"scan match {100 * st.scan_match:.0f} %; {st.reason}"
        )
        t0 = time.time()
        while True:
            with self.lock:
                last = self.amcl_hist[-1] if self.amcl_hist else None
            tr = self._truth_at(last[0]) if last else None
            if tr is not None and math.hypot(last[1] - tr[1], last[2] - tr[2]) < 0.10:
                break
            self.assertLess(time.time() - t0, 60.0, "AMCL never came within 0.10 m of truth")
            time.sleep(0.2)
        print(f"aligned with truth after {time.time() - t0:.1f} s more driving")
        with self.lock:
            st = self.state
        self.assertTrue(st.can_confirm, st.reason)
        res = self._call(self.confirm, Trigger.Request())
        self.assertTrue(res.success, res.message)
        t_ready = time.time()
        self._wait_state(lambda s: s.state == LocalizationState.READY, 5, "READY")

        proc_output.assertWaitFor("square done", process=square, timeout=200)
        time.sleep(1.0)

        # 3. accuracy over the second half of the loop (wall-clock split)
        with self.lock:
            amcl = list(self.amcl_hist)
        self.assertGreater(len(amcl), 50, "too few AMCL updates")
        mid = amcl[len(amcl) // 2][0]
        errs, yaws = [], []
        for t, x, y, yaw in amcl:
            if t < mid:
                continue
            tr = self._truth_at(t)
            if tr is None:
                continue
            errs.append(math.hypot(x - tr[1], y - tr[2]))
            yaws.append(abs(math.degrees(math.atan2(math.sin(yaw - tr[3]), math.cos(yaw - tr[3])))))
        self.assertGreater(len(errs), 20)
        p95, y95 = float(np.percentile(errs, 95)), float(np.percentile(yaws, 95))
        print(
            f"AMCL vs truth, second half: n={len(errs)} p50 {np.median(errs):.3f} m p95 {p95:.3f} m "
            f"max {max(errs):.3f} m | yaw p95 {y95:.2f} deg max {max(yaws):.2f} deg"
        )
        self.assertLess(p95, 0.10)
        self.assertLess(y95, 2.0)
        with self.lock:
            st = self.state
        self.assertEqual(st.state, LocalizationState.READY, f"lost during the loop: {st.reason}")
        self.assertGreater(time.time() - t_ready, 20.0)

        # 4. kidnap: reality moves 2 m and turns round; odometry does not. Drive so AMCL updates.
        tr = self._truth_at(time.time())
        res = self._call(self.teleport, SetPose2D.Request(x_m=2.0, y_m=0.0, yaw_rad=math.pi))
        self.assertTrue(res.ok)
        self._drive(0.3, 0.0, 4.0)
        st = self._wait_state(lambda s: s.state == LocalizationState.LOST, 15, "LOST after kidnap")
        print(
            f"LOST: {st.reason} (scan match {100 * st.scan_match:.0f} %, "
            f"jump {st.last_jump_m:.2f} m / {math.degrees(st.last_jump_rad):.1f} deg)"
        )

        # 5. recover, stopped: new initial pose at the true pose, then slow motion, then confirm
        time.sleep(1.0)
        tr = self._truth_at(time.time())
        self.assertIsNotNone(tr)
        self._publish_initialpose(tr[1], tr[2], tr[3], sigma_xy=0.3, sigma_yaw=math.radians(10))
        self._wait_state(lambda s: s.state == LocalizationState.CHECKING, 10, "CHECKING again")
        self._drive(0.2, 0.0, 6.0)
        self._wait_state(lambda s: s.can_confirm, 30, "can_confirm again")
        res = self._call(self.confirm, Trigger.Request())
        self.assertTrue(res.success, res.message)
        st = self._wait_state(lambda s: s.state == LocalizationState.READY, 5, "READY again")
        tr = self._truth_at(time.time())
        with self.lock:
            last = self.amcl_hist[-1]
        err = math.hypot(last[1] - tr[1], last[2] - tr[2])
        print(f"recovered: AMCL vs truth {err:.3f} m")
        self.assertLess(err, 0.15)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
