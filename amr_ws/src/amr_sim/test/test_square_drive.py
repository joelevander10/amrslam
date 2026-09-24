"""T2 integration (spec §8 test 1 precursor, and test 6 at the mux level).

fake_base + cmd_mux + diff_drive_odom, driven round a 4 x 5 m square by
square_drive. Raw wheel odometry must match ground truth to < 1 % of the
distance travelled with slip off, and the wheels must be zero shortly after
the command stream stops.
"""

import os

# Every launch test gets its own DDS domain so `colcon test` may run packages in
# parallel without simulations talking to each other's /scan, /map or /cmd_vel_teleop.
# Set before rclpy and before launch forks the nodes, which inherit it.
os.environ["ROS_DOMAIN_ID"] = "61"

import pytest

if (
    os.environ.get("AMR_SIM_TESTS") != "1"
):  # ~1-5 min each on the N97: run deliberately, not on every colcon test
    pytest.skip("simulation launch test; set AMR_SIM_TESTS=1", allow_module_level=True)

import math
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from amr_interfaces.msg import WheelStates

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    square = Node(
        package="amr_sim",
        executable="square_drive",
        name="square_drive",
        output="screen",
        parameters=[{"speed_mps": 1.0, "turn_rate_rad_s": 1.0, "settle_s": 0.8}],
    )
    return (
        LaunchDescription(
            [
                Node(
                    package="amr_sim",
                    executable="fake_base_node",
                    name="fake_base",
                    parameters=[{"slip_noise_std": 0.0}],
                ),
                Node(
                    package="amr_base",
                    executable="cmd_mux_kinematics_node",
                    name="cmd_mux_kinematics",
                ),
                Node(
                    package="amr_base",
                    executable="diff_drive_odom_node",
                    name="diff_drive_odom",
                ),
                square,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"square": square},
    )


def _yaw(o: Odometry) -> float:
    q = o.pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class TestSquareDrive(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("square_test")
        cls.lock = threading.Lock()
        cls.odom = None
        cls.truth = None
        cls.wheels = None
        cls.truth_len = 0.0
        cls._prev_truth = None

        def on_odom(m):
            with cls.lock:
                cls.odom = m

        def on_truth(m):
            with cls.lock:
                if cls._prev_truth is not None:
                    dx = m.pose.pose.position.x - cls._prev_truth.pose.pose.position.x
                    dy = m.pose.pose.position.y - cls._prev_truth.pose.pose.position.y
                    cls.truth_len += math.hypot(dx, dy)
                cls._prev_truth = m
                cls.truth = m

        def on_wheels(m):
            with cls.lock:
                cls.wheels = m

        cls.node.create_subscription(Odometry, "/odom_raw", on_odom, SENSOR_DATA)
        cls.node.create_subscription(Odometry, "/sim/ground_truth", on_truth, SENSOR_DATA)
        cls.node.create_subscription(WheelStates, "/wheel_states", on_wheels, SENSOR_DATA)
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_odom_tracks_ground_truth(self, proc_output, square):
        proc_output.assertWaitFor("square done", process=square, timeout=120)
        time.sleep(1.0)  # let the last odometry samples arrive
        with self.lock:
            odom, truth, dist = self.odom, self.truth, self.truth_len
        self.assertIsNotNone(odom, "no /odom_raw received")
        self.assertIsNotNone(truth, "no /sim/ground_truth received")
        self.assertGreater(dist, 15.0, f"square barely moved: {dist:.2f} m")

        ex = odom.pose.pose.position.x - truth.pose.pose.position.x
        ey = odom.pose.pose.position.y - truth.pose.pose.position.y
        pos_err = math.hypot(ex, ey)
        yaw_err = abs(math.atan2(math.sin(_yaw(odom) - _yaw(truth)), math.cos(_yaw(odom) - _yaw(truth))))
        print(
            f"distance {dist:.3f} m, position error {pos_err * 1000:.1f} mm "
            f"({100 * pos_err / dist:.3f} %), yaw error {math.degrees(yaw_err):.3f}°"
        )
        self.assertLess(pos_err / dist, 0.01)
        self.assertLess(yaw_err, math.radians(1.0))

    def test_2_wheels_zero_after_command_stream_stops(self, proc_output, square):
        proc_output.assertWaitFor("square done", process=square, timeout=120)
        # cmd_mux zeroes at 200 ms without input; fake_base at 250 ms without mux.
        time.sleep(0.5)
        with self.lock:
            w = self.wheels
        self.assertIsNotNone(w)
        self.assertEqual((w.left_vel_rad_s, w.right_vel_rad_s), (0.0, 0.0))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
