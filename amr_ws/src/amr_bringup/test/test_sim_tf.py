"""Composed sim launch: every TF child has exactly one parent, and the tree is §5.

Unit tests of the URDF and of the odometry node cannot see this: the conflict
only exists once both publish. Listens to /tf and /tf_static for a few seconds
and records every (child -> parent) pair seen.
"""

import os

# Every launch test gets its own DDS domain so `colcon test` may run packages in
# parallel without simulations talking to each other's /scan, /map or /cmd_vel_teleop.
# Set before rclpy and before launch forks the nodes, which inherit it.
os.environ["ROS_DOMAIN_ID"] = "63"

import pytest

if (
    os.environ.get("AMR_SIM_TESTS") != "1"
):  # ~1-5 min each on the N97: run deliberately, not on every colcon test
    pytest.skip("simulation launch test; set AMR_SIM_TESTS=1", allow_module_level=True)

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
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

EXPECTED = {
    "base_footprint": "odom",
    "base_link": "base_footprint",
    "laser_frame": "base_link",
    "imu_frame": "base_link",
    "wheel_left": "base_link",
    "wheel_right": "base_link",
}


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    import os

    sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("amr_bringup"), "launch", "sim.launch.py")
        )
    )
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


class TestSimTf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("tf_tree_test")
        cls.lock = threading.Lock()
        cls.parents: dict[str, set[str]] = {}

        def on_tf(msg: TFMessage):
            with cls.lock:
                for t in msg.transforms:
                    cls.parents.setdefault(t.child_frame_id, set()).add(t.header.frame_id)

        static_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.node.create_subscription(TFMessage, "/tf", on_tf, 100)
        cls.node.create_subscription(TFMessage, "/tf_static", on_tf, static_qos)
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_one_parent_per_frame_and_expected_tree(self):
        deadline = time.time() + 15.0
        while time.time() < deadline:
            with self.lock:
                if set(EXPECTED) <= set(self.parents):
                    break
            time.sleep(0.2)
        time.sleep(2.0)  # keep listening: a second publisher would show up here
        with self.lock:
            parents = {k: set(v) for k, v in self.parents.items()}
        print("tf tree:", {c: sorted(p) for c, p in sorted(parents.items())})
        missing = set(EXPECTED) - set(parents)
        self.assertFalse(missing, f"frames never published: {missing}")
        multi = {c: p for c, p in parents.items() if len(p) != 1}
        self.assertFalse(multi, f"frames with more than one parent: {multi}")
        for child, parent in EXPECTED.items():
            self.assertEqual(parents[child], {parent}, child)
        extra = set(parents) - set(EXPECTED)
        self.assertFalse(extra, f"unexpected frames: {extra}")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
