"""scan_synth_node (spec §8): /scan raycast from ground truth against a world PGM.

Replaces the SICK driver in sim. The laser pose is ground truth (base_footprint
in the world frame) composed with the URDF laser offset, so the scan is what a
scanner mounted 0.964 m ahead of the axle would see. Gaussian range noise,
sigma 0.01 m. Optional clutter: N random rectangles added to the world at
startup, so a survey/localisation run sees objects the reference map does not.

Publishes on the frame `laser_frame`; stamps at the ground-truth sample time.
"""

import math
import os
import random

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from amr_maps.raycast import ScanGeometry, cast, nanoscan3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from amr_interfaces.srv import AddObstacle
from amr_maps import grid as gridio

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ScanSynth(Node):
    def __init__(self) -> None:
        super().__init__("scan_synth")
        default_world = os.path.join(
            get_package_share_directory("amr_maps"), "worlds", "sim_factory", "world.yaml"
        )
        self.declare_parameter("world_yaml", default_world)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("beams", 381)  # 190 deg at 0.5 deg
        # The unit is a 275 deg scanner, but its mount has a wall behind it: the
        # usable window is about +/-95 deg (operator, 2026-09-16). The real driver
        # is configured to the same window (nanoscan3.yaml angle_start/end).
        self.declare_parameter("fov_deg", 190.0)
        self.declare_parameter("range_max", 30.0)
        self.declare_parameter("range_noise_std", 0.01)
        self.declare_parameter("frame_id", "laser_frame")
        # URDF laser offset from base_footprint (x forward, yaw). Keep equal to amr.urdf.xacro.
        self.declare_parameter("laser_x", 0.964)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("laser_yaw", 0.0)
        self.declare_parameter("clutter_count", 0)
        self.declare_parameter("clutter_size_m", 1.2)
        self.declare_parameter("seed", 0)
        p = self.get_parameter

        self.world = gridio.read(p("world_yaml").value)
        self.geom = ScanGeometry(
            -math.radians(p("fov_deg").value) / 2,
            math.radians(p("fov_deg").value) / 2,
            int(p("beams").value),
            nanoscan3().range_min,
            float(p("range_max").value),
        )
        self.noise = float(p("range_noise_std").value)
        self.frame_id = p("frame_id").value
        self.lx, self.ly, self.lyaw = p("laser_x").value, p("laser_y").value, p("laser_yaw").value
        self.dt = 1.0 / p("rate_hz").value
        self._rng = np.random.default_rng(int(p("seed").value))
        self._truth: Odometry | None = None
        self._add_clutter(
            int(p("clutter_count").value), float(p("clutter_size_m").value), int(p("seed").value)
        )

        self._base_world = self.world.data.copy()  # before runtime obstacles
        self.create_service(AddObstacle, "/sim/add_obstacle", self._add_obstacle)
        self.create_service(Trigger, "/sim/clear_obstacles", self._clear_obstacles)
        self._pub = self.create_publisher(LaserScan, "/scan", SENSOR_DATA)
        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, SENSOR_DATA)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"world {os.path.basename(p('world_yaml').value)} {self.world.width}x{self.world.height} @ "
            f"{self.world.meta.resolution} m, {self.geom.beams} beams / {p('fov_deg').value} deg, "
            f"{p('rate_hz').value} Hz, clutter {p('clutter_count').value}"
        )

    def _add_clutter(self, count: int, size: float, seed: int) -> None:
        """Random boxes on free cells, away from the start mark. Deterministic per seed."""
        if count <= 0:
            return
        rng = random.Random(seed)
        g = self.world
        placed = 0
        while placed < count:
            x = rng.uniform(g.meta.origin_x, g.meta.origin_x + g.width * g.meta.resolution)
            y = rng.uniform(g.meta.origin_y, g.meta.origin_y + g.height * g.meta.resolution)
            if math.hypot(x, y) < 3.0:  # keep the start area as surveyed
                continue
            r0, c0 = g.world_to_cell(x - size / 2, y - size / 2)
            r1, c1 = g.world_to_cell(x + size / 2, y + size / 2)
            if r0 < 0 or c0 < 0 or r1 >= g.height or c1 >= g.width:
                continue
            if (g.data[r0:r1, c0:c1] != 0).any():
                continue
            g.data[r0:r1, c0:c1] = 100
            placed += 1

    def _add_obstacle(self, req: AddObstacle.Request, res: AddObstacle.Response):
        """A box in REALITY only: the saved map does not know it (spec §8 obstacle tests)."""
        g = self.world
        r0, c0 = g.world_to_cell(req.x_m - req.size_m / 2, req.y_m - req.size_m / 2)
        r1, c1 = g.world_to_cell(req.x_m + req.size_m / 2, req.y_m + req.size_m / 2)
        r0, c0 = max(r0, 0), max(c0, 0)
        r1, c1 = min(r1, g.height - 1), min(c1, g.width - 1)
        if r1 < r0 or c1 < c0:
            res.ok, res.message = False, "outside the world"
            return res
        g.data[r0 : r1 + 1, c0 : c1 + 1] = 100
        res.ok = True
        res.message = f"obstacle {req.size_m:.2f} m at ({req.x_m:.2f}, {req.y_m:.2f})"
        self.get_logger().warn(res.message)
        return res

    def _clear_obstacles(self, _req, res):
        self.world.data[:] = self._base_world
        res.success, res.message = True, "runtime obstacles cleared"
        self.get_logger().info(res.message)
        return res

    def _on_truth(self, msg: Odometry) -> None:
        self._truth = msg

    def _tick(self) -> None:
        if self._truth is None:
            return
        t = self._truth
        q = t.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        bx, by = t.pose.pose.position.x, t.pose.pose.position.y
        lx = bx + self.lx * math.cos(yaw) - self.ly * math.sin(yaw)
        ly = by + self.lx * math.sin(yaw) + self.ly * math.cos(yaw)
        ranges = cast(self.world, lx, ly, yaw + self.lyaw, self.geom)
        finite = np.isfinite(ranges)
        if self.noise > 0.0:
            ranges[finite] += self._rng.normal(0.0, self.noise, int(finite.sum()))

        m = LaserScan()
        m.header.stamp = t.header.stamp
        m.header.frame_id = self.frame_id
        m.angle_min = float(self.geom.angle_min)
        m.angle_max = float(self.geom.angle_max)
        m.angle_increment = float(self.geom.angle_increment)
        m.time_increment = 0.0
        m.scan_time = float(self.dt)
        m.range_min = float(self.geom.range_min)
        m.range_max = float(self.geom.range_max)
        m.ranges = ranges.tolist()
        self._pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSynth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A callback running while launch tears the context down raises from
        # the C layer ("Unable to convert call argument"); only real if still ok.
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
