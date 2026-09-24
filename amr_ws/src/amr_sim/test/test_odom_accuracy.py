"""spec §8 test 1, test_odom_accuracy: fused odometry vs ground truth < 1 %.

The full sim.launch.py chain (fake base with 2 % slip, fake IMU with the D-3
bias and noise, cmd_mux, diff_drive_odom, imu_bias, ekf_local) driven round a
4 x 5 m square. The fused estimate must be within 1 % of the distance and
within 1 degree of heading; the raw wheel odometry, which has no gyro, is
reported alongside so the gain from fusion is visible in the log.
"""

import os

# Every launch test gets its own DDS domain so `colcon test` may run packages in
# parallel without simulations talking to each other's /scan, /map or /cmd_vel_teleop.
# Set before rclpy and before launch forks the nodes, which inherit it.
os.environ["ROS_DOMAIN_ID"] = "62"

import pytest

if (
    os.environ.get("AMR_SIM_TESTS") != "1"
):  # ~1-5 min each on the N97: run deliberately, not on every colcon test
    pytest.skip("simulation launch test; set AMR_SIM_TESTS=1", allow_module_level=True)

import math
import os
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("amr_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={"slip_noise_std": "0.02"}.items(),
    )
    square = Node(
        package="amr_sim",
        executable="square_drive",
        name="square_drive",
        output="screen",
        parameters=[{"speed_mps": 1.0, "turn_rate_rad_s": 1.0, "settle_s": 0.8, "start_delay_s": 3.0}],
    )
    return LaunchDescription([sim, square, launch_testing.actions.ReadyToTest()]), {"square": square}


def _yaw(o: Odometry) -> float:
    q = o.pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def _err(a: Odometry, b: Odometry) -> tuple[float, float]:
    ex = a.pose.pose.position.x - b.pose.pose.position.x
    ey = a.pose.pose.position.y - b.pose.pose.position.y
    d = _yaw(a) - _yaw(b)
    return math.hypot(ex, ey), abs(math.atan2(math.sin(d), math.cos(d)))


class TestOdomAccuracy(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("odom_accuracy_test")
        cls.lock = threading.Lock()
        cls.last = {}
        cls.truth_len = 0.0
        cls._prev = None

        def keep(key):
            def cb(m):
                with cls.lock:
                    cls.last[key] = m
                    if key == "truth":
                        if cls._prev is not None:
                            cls.truth_len += math.hypot(
                                m.pose.pose.position.x - cls._prev.pose.pose.position.x,
                                m.pose.pose.position.y - cls._prev.pose.pose.position.y,
                            )
                        cls._prev = m

            return cb

        cls.node.create_subscription(Odometry, "/sim/ground_truth", keep("truth"), SENSOR_DATA)
        cls.node.create_subscription(Odometry, "/odom_raw", keep("raw"), SENSOR_DATA)
        cls.node.create_subscription(Odometry, "/odometry/filtered", keep("fused"), 10)
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_fused_odom_within_one_percent(self, proc_output, square):
        proc_output.assertWaitFor("square done", process=square, timeout=150)
        time.sleep(1.0)
        with self.lock:
            last, dist = dict(self.last), self.truth_len
        for k in ("truth", "raw", "fused"):
            self.assertIn(k, last, f"no {k} odometry received")
        self.assertGreater(dist, 15.0, f"square barely moved: {dist:.2f} m")
        self.assertEqual(last["fused"].header.frame_id, "odom")
        self.assertEqual(last["fused"].child_frame_id, "base_footprint")

        raw_pos, raw_yaw = _err(last["raw"], last["truth"])
        fused_pos, fused_yaw = _err(last["fused"], last["truth"])
        print(
            f"distance {dist:.2f} m | raw: {raw_pos * 1000:.0f} mm ({100 * raw_pos / dist:.2f} %), "
            f"{math.degrees(raw_yaw):.2f} deg | fused: {fused_pos * 1000:.0f} mm "
            f"({100 * fused_pos / dist:.2f} %), {math.degrees(fused_yaw):.2f} deg"
        )
        self.assertLess(fused_pos / dist, 0.01)
        self.assertLess(fused_yaw, math.radians(1.0))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
