"""T4 acceptance (spec §8 "Survey/return", "Save/load"; §10 T4).

mapping.launch.py sim: a survey loop round rack row C with 2 % wheel slip and
a deliberate closing error, then Returned-to-start -> Save. Checks:

  * the closure evidence reports the REAL return error (from ground truth),
    not zero: confirmation neither forces pose equality nor hides the error;
  * the saved occupancy agrees with the world: occupied cells within 2 cells
    of each other in both directions, > 0.9 of them, within the observed area;
  * the bundle reloads with identical geometry and passes verification;
  * a second save of the same session is a new revision, never an overwrite.
"""

import os

# Every launch test gets its own DDS domain so `colcon test` may run packages in
# parallel without simulations talking to each other's /scan, /map or /cmd_vel_teleop.
# Set before rclpy and before launch forks the nodes, which inherit it.
os.environ["ROS_DOMAIN_ID"] = "64"

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
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from amr_interfaces.msg import MappingState
from amr_interfaces.srv import SaveMap, StartSurvey
from amr_maps import grid as gridio
from amr_mission import map_bundle as mb

MAPS_DIR = tempfile.mkdtemp(prefix="amr_survey_test_")
CLOSING_ERROR_M, CLOSING_ERROR_DEG = 0.10, 3.0
WORLD = os.path.join(get_package_share_directory("amr_maps"), "worlds", "sim_factory", "world.yaml")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mapping = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("amr_bringup"), "launch", "mapping.launch.py")
        ),
        launch_arguments={"sim": "true", "slip_noise_std": "0.02", "maps_dir": MAPS_DIR}.items(),
    )
    # East along the B/C aisle, north past the rack ends, west along the C/D
    # aisle, south back to the mark. Last leg 0.10 m short, last turn 3 deg short.
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
                "start_delay_s": 4.0,
                "closing_error_m": CLOSING_ERROR_M,
                "closing_error_deg": CLOSING_ERROR_DEG,
            }
        ],
    )
    return LaunchDescription([mapping, square, launch_testing.actions.ReadyToTest()]), {"square": square}


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def _near(occ_a: np.ndarray, occ_b: np.ndarray, cells: int) -> float:
    """Fraction of True cells in occ_a with a True cell of occ_b within `cells` (Chebyshev)."""
    dil = np.zeros_like(occ_b)
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            dil |= np.roll(np.roll(occ_b, dr, axis=0), dc, axis=1)
    n = int(occ_a.sum())
    return float((occ_a & dil).sum()) / n if n else 0.0


def _resample_world(world: gridio.Grid, target: gridio.Grid) -> np.ndarray:
    """World cells sampled at the centres of the target grid's cells (same frame)."""
    rows = np.arange(target.height)
    cols = np.arange(target.width)
    xs = target.meta.origin_x + (cols + 0.5) * target.meta.resolution
    ys = target.meta.origin_y + (rows + 0.5) * target.meta.resolution
    wc = np.floor((xs - world.meta.origin_x) / world.meta.resolution).astype(int)
    wr = np.floor((ys - world.meta.origin_y) / world.meta.resolution).astype(int)
    out = np.full((target.height, target.width), -1, dtype=np.int8)
    ok_c = (wc >= 0) & (wc < world.width)
    ok_r = (wr >= 0) & (wr < world.height)
    sub = world.data[np.ix_(wr[ok_r], wc[ok_c])]
    out[np.ix_(ok_r, ok_c)] = sub
    return out


class TestSurvey(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("survey_test")
        cls.lock = threading.Lock()
        cls.truth = None
        cls.state = None

        def on_truth(m):
            with cls.lock:
                cls.truth = m

        def on_state(m):
            with cls.lock:
                cls.state = m

        cls.node.create_subscription(
            Odometry,
            "/sim/ground_truth",
            on_truth,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )
        from rclpy.qos import QoSDurabilityPolicy

        cls.node.create_subscription(
            MappingState,
            "/amr/mapping_state",
            on_state,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.start_cli = cls.node.create_client(StartSurvey, "/amr/survey/start")
        cls.returned_cli = cls.node.create_client(Trigger, "/amr/survey/returned")
        cls.save_cli = cls.node.create_client(SaveMap, "/amr/survey/save")
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

    def _call(self, client, req, timeout=30.0):
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), f"{client.srv_name} unavailable")
        fut = client.call_async(req)
        t0 = time.time()
        while not fut.done():
            self.assertLess(time.time() - t0, timeout, f"{client.srv_name} timed out")
            time.sleep(0.05)
        return fut.result()

    def test_survey_return_save(self, proc_output, square):
        # Start: readiness includes IMU calibration, which takes ~2.3 s still.
        deadline = time.time() + 20.0
        while time.time() < deadline:
            res = self._call(self.start_cli, StartSurvey.Request(map_id="sim_factory", description="mark A"))
            if res.accepted:
                break
            time.sleep(0.5)
        self.assertTrue(res.accepted, res.message)
        print("start:", res.message)

        proc_output.assertWaitFor("square done", process=square, timeout=240)
        time.sleep(3.0)  # let the last scans match and the graph settle
        with self.lock:
            truth = self.truth
        truth_err = math.hypot(truth.pose.pose.position.x, truth.pose.pose.position.y)
        truth_dyaw = math.degrees(_yaw(truth.pose.pose.orientation))
        print(f"ground-truth return error: {truth_err:.3f} m, {truth_dyaw:+.2f} deg")
        self.assertGreater(truth_err, 0.05, "the loop should NOT close exactly in this test")

        res = self._call(self.returned_cli, Trigger.Request())
        self.assertTrue(res.success, res.message)
        print("returned:", res.message)
        time.sleep(1.0)
        with self.lock:
            st = self.state
        self.assertEqual(st.state, MappingState.RETURN_REVIEW)
        self.assertTrue(st.closure_available)
        est_err = math.hypot(st.closure_dx_m, st.closure_dy_m)
        est_dyaw = math.degrees(st.closure_dyaw_rad)
        print(f"estimated closure: {est_err:.3f} m, {est_dyaw:+.2f} deg")
        # Evidence must track reality: neither forced to zero nor wildly off.
        self.assertLess(abs(est_err - truth_err), 0.10)
        self.assertLess(abs(est_dyaw - truth_dyaw), 3.0)

        res = self._call(self.save_cli, SaveMap.Request(note="test survey"), timeout=60.0)
        self.assertTrue(res.ok, res.message)
        print("saved:", res.path)
        self.assertEqual(res.revision, 1)
        self.assertTrue(os.path.isdir(res.path))

        # --- bundle: verify, reload, compare against the world -------------
        manifest, saved = mb.load(MAPS_DIR, "sim_factory", 1)
        self.assertEqual(manifest.review["dx_m"], pytest.approx(st.closure_dx_m))
        self.assertEqual(manifest.start["description"], "mark A")
        for name in ("map.pgm", "map.yaml", "posegraph.posegraph", "posegraph.data"):
            self.assertIn(name, manifest.files)
        self.assertEqual(saved.meta.resolution, pytest.approx(0.05))

        world = gridio.read(WORLD)
        world_on_saved = _resample_world(world, saved)
        observed = saved.data != -1
        slam_occ = saved.data >= 65
        world_occ = (world_on_saved == 100) & observed
        occ_to_world = _near(slam_occ, world_on_saved == 100, 2)
        world_to_occ = _near(world_occ, slam_occ, 2)
        print(
            f"saved map {saved.width}x{saved.height}, observed {observed.mean() * 100:.0f} %, "
            f"slam occupied -> world {occ_to_world:.3f}, world occupied (observed) -> slam {world_to_occ:.3f}"
        )
        self.assertGreater(int(slam_occ.sum()), 1000)
        self.assertGreater(occ_to_world, 0.9)
        self.assertGreater(world_to_occ, 0.9)

        # --- a second save is a new revision, not an overwrite -------------
        res2 = self._call(self.save_cli, SaveMap.Request(note="again"), timeout=60.0)
        self.assertFalse(res2.ok, "save from SAVED must be refused; a new survey needs RETURN_REVIEW")
        self.assertEqual(mb.list_revisions(MAPS_DIR, "sim_factory"), [1])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
