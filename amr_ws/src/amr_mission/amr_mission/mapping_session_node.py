"""mapping_session_node: the survey coordinator (spec §6.1, §6.2).

States IDLE -> MAPPING -> RETURN_REVIEW -> SAVING -> SAVED. Services:

  /amr/survey/start     StartSurvey   readiness checks, record the start reference
  /amr/survey/returned  Trigger       compute closure evidence vs the reference
  /amr/survey/save      SaveMap       pause SLAM, serialise, stage, verify, publish
  /amr/survey/abort     Trigger       back to IDLE (SLAM keeps its graph: relaunch for a clean survey)

The closure evidence is what SLAM estimates after the operator has physically
returned; nothing here forces the final pose onto the start pose. A save that
fails keeps its staging directory as a draft and never becomes a revision.
slam_toolbox's pause service is a toggle whose reply carries no state, so the
paused state is tracked locally (unpaused at launch).
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import asdict

import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from slam_toolbox.srv import Pause, SerializePoseGraph
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from amr_interfaces.msg import MappingState, WheelStates
from amr_interfaces.srv import SaveMap, StartSurvey
from amr_maps import grid as gridio
from amr_mission import map_bundle as mb

SENSOR_DATA = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
STATE_NAMES = {
    MappingState.IDLE: "IDLE",
    MappingState.MAPPING: "MAPPING",
    MappingState.RETURN_REVIEW: "RETURN_REVIEW",
    MappingState.SAVING: "SAVING",
    MappingState.SAVED: "SAVED",
}


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MappingSession(Node):
    def __init__(self) -> None:
        super().__init__("mapping_session")
        self.declare_parameter("maps_dir", os.path.expanduser("~/amr_maps"))
        self.declare_parameter("scan_age_limit_s", 0.15)
        self.declare_parameter("imu_age_limit_s", 0.20)
        self.declare_parameter("wheels_age_limit_s", 0.10)
        self.declare_parameter("stationary_wheel_rad_s", 0.01)
        self.declare_parameter("service_timeout_s", 10.0)
        # slam_toolbox stamps map->odom with the last scan it CONSUMED (+ transform_timeout).
        # Scans flowing while that stamp stands still means SLAM is no longer taking them:
        # its executor froze (2026-09-17) or its message filter drops everything.
        self.declare_parameter("slam_stall_s", 5.0)
        self.declare_parameter("generation", 0)  # layer generation (unified plan U0)
        self.generation = int(self.get_parameter("generation").value)
        p = self.get_parameter
        self.maps_dir = os.path.expanduser(p("maps_dir").value)
        self.scan_age = p("scan_age_limit_s").value
        self.imu_age = p("imu_age_limit_s").value
        self.wheels_age = p("wheels_age_limit_s").value
        self.w_eps = p("stationary_wheel_rad_s").value
        self.srv_timeout = p("service_timeout_s").value
        self.slam_stall_s = p("slam_stall_s").value

        self._lock = threading.Lock()
        self._state = MappingState.IDLE
        self._map_id = ""
        self._description = ""
        self._revision = 0
        self._start: mb.StartReference | None = None
        self._review: mb.ReturnReview | None = None
        self._saved_path = ""
        self._message = "select New map while stopped"
        self._last_scan_t: float | None = None
        self._last_scan_stamp: float | None = None  # header stamp, slam's clock for map->odom
        self._slam_stalled_s = 0.0  # >0: seconds of scans SLAM has not consumed
        self._last_imu_t: float | None = None
        self._last_wheels: tuple[float, bool] | None = None  # (t, still)
        self._map: OccupancyGrid | None = None
        self._map_t: float | None = None
        self._slam_paused = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        io_group = ReentrantCallbackGroup()
        srv_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(LaserScan, "/scan", self._on_scan, SENSOR_DATA, callback_group=io_group)
        self.create_subscription(Imu, "/imu/data", self._on_imu, SENSOR_DATA, callback_group=io_group)
        self.create_subscription(
            WheelStates, "/wheel_states", self._on_wheels, SENSOR_DATA, callback_group=io_group
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED, callback_group=io_group)
        self._pub = self.create_publisher(MappingState, "/amr/mapping_state", LATCHED)
        self.create_timer(1.0, self._publish_state, callback_group=io_group)

        self._pause = self.create_client(
            Pause, "/slam_toolbox/pause_new_measurements", callback_group=io_group
        )
        self._serialize = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map", callback_group=io_group
        )

        self.create_service(StartSurvey, "/amr/survey/start", self._srv_start, callback_group=srv_group)
        self.create_service(Trigger, "/amr/survey/returned", self._srv_returned, callback_group=srv_group)
        self.create_service(SaveMap, "/amr/survey/save", self._srv_save, callback_group=srv_group)
        self.create_service(Trigger, "/amr/survey/abort", self._srv_abort, callback_group=srv_group)
        self.get_logger().info(f"maps_dir {self.maps_dir}; state IDLE")
        self._publish_state()

    # ---- inputs ------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_t = self._now()
        self._last_scan_stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

    def _slam_lag_s(self) -> float | None:
        """Seconds between the newest scan and the scan SLAM last consumed, from the
        map->odom stamp; None when there is no map->odom or no scan yet."""
        if self._last_scan_stamp is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        consumed = rclpy.time.Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        return self._last_scan_stamp - consumed

    def _check_slam(self) -> None:
        """Once a second while a graph is being built (not paused for review/save)."""
        if self._state != MappingState.MAPPING:
            self._slam_stalled_s = 0.0
            return
        lag = self._slam_lag_s()
        stalled = lag is not None and lag > self.slam_stall_s
        if stalled and self._slam_stalled_s == 0.0:
            self.get_logger().error(
                f"SLAM stalled: scans arrive but slam_toolbox consumed none for {lag:.0f} s "
                "(map->odom stamp frozen); abort the survey and start again"
            )
        self._slam_stalled_s = lag if stalled else 0.0
        if stalled:
            with self._lock:
                self._message = (
                    f"SLAM STALLED {lag:.0f} s: slam_toolbox stopped taking scans; abort, then start again"
                )

    def _on_imu(self, _msg: Imu) -> None:
        self._last_imu_t = self._now()

    def _on_wheels(self, msg: WheelStates) -> None:
        still = (
            msg.left_valid
            and msg.right_valid
            and abs(msg.left_vel_rad_s) < self.w_eps
            and abs(msg.right_vel_rad_s) < self.w_eps
        )
        self._last_wheels = (self._now(), still)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map, self._map_t = msg, self._now()

    def _pose_in_map(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:  # noqa: BLE001 - any TF failure means "no pose"
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _readiness(self, need_still: bool = True) -> list[str]:
        """Empty list = ready; otherwise the reasons (spec §2.4 age limits)."""
        now = self._now()
        problems = []
        if self._last_scan_t is None or now - self._last_scan_t > self.scan_age:
            problems.append("no fresh /scan")
        if self._last_imu_t is None or now - self._last_imu_t > self.imu_age:
            problems.append("IMU not calibrated or stale (/imu/data)")
        if self._last_wheels is None or now - self._last_wheels[0] > self.wheels_age:
            problems.append("no fresh /wheel_states")
        elif need_still and not self._last_wheels[1]:
            problems.append("vehicle is moving")
        if self._pose_in_map() is None:
            problems.append("no map->base_footprint transform")
        if self._slam_stalled_s > 0.0:
            problems.append(
                f"SLAM stalled for {self._slam_stalled_s:.0f} s (slam_toolbox hung); abort and restart"
            )
        return problems

    # ---- state -------------------------------------------------------------

    def _set(self, state: int, message: str) -> None:
        with self._lock:
            old, self._state, self._message = self._state, state, message
        if old != state:
            self.get_logger().info(f"{STATE_NAMES[old]} -> {STATE_NAMES[state]}: {message}")
        self._publish_state()

    def _publish_state(self) -> None:
        self._check_slam()
        with self._lock:
            m = MappingState()
            m.generation = self.generation
            m.header.stamp = self.get_clock().now().to_msg()
            m.state = self._state
            m.map_id = self._map_id
            m.revision = self._revision
            m.description = self._description
            if self._start is not None:
                m.start = Pose2D(x=self._start.x_m, y=self._start.y_m, theta=self._start.yaw_rad)
            if self._review is not None:
                m.closure_available = True
                m.closure_dx_m = self._review.dx_m
                m.closure_dy_m = self._review.dy_m
                m.closure_dyaw_rad = self._review.dyaw_rad
            m.saved_path = self._saved_path
            m.message = self._message
        self._pub.publish(m)

    # ---- services ----------------------------------------------------------

    def _srv_start(self, req: StartSurvey.Request, res: StartSurvey.Response):
        if self._state != MappingState.IDLE:
            res.accepted, res.message = (
                False,
                f"not IDLE ({STATE_NAMES[self._state]}); abort or relaunch mapping",
            )
            return res
        if not req.map_id or not mb.REV_RE.match("rev1") or "/" in req.map_id or req.map_id.startswith("."):
            res.accepted, res.message = False, "map_id must be a plain name"
            return res
        problems = self._readiness()
        if problems:
            res.accepted, res.message = False, "not ready: " + "; ".join(problems)
            self._set(self._state, res.message)
            return res
        x, y, yaw = self._pose_in_map()
        with self._lock:
            self._map_id, self._description = req.map_id, req.description
            self._start = mb.StartReference(x, y, yaw, req.description)
            self._review, self._saved_path, self._revision = None, "", 0
        self._set(
            MappingState.MAPPING,
            f"surveying '{req.map_id}' from ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg)",
        )
        res.accepted, res.message = True, self._message
        return res

    def _srv_returned(self, _req, res: Trigger.Response):
        if self._state not in (MappingState.MAPPING, MappingState.RETURN_REVIEW):
            res.success, res.message = False, f"not surveying ({STATE_NAMES[self._state]})"
            return res
        problems = self._readiness()
        if problems:
            res.success, res.message = False, "not ready: " + "; ".join(problems)
            return res
        x, y, yaw = self._pose_in_map()
        s = self._start
        review = mb.ReturnReview(x - s.x_m, y - s.y_m, _wrap(yaw - s.yaw_rad))
        with self._lock:
            self._review = review
        self._set(
            MappingState.RETURN_REVIEW,
            f"closure as estimated: dx {review.dx_m:+.3f} m, dy {review.dy_m:+.3f} m, "
            f"dyaw {math.degrees(review.dyaw_rad):+.2f} deg - review seams, then save or drive another loop",
        )
        res.success, res.message = True, self._message
        return res

    def _srv_abort(self, _req, res: Trigger.Response):
        if self._state == MappingState.SAVING:
            res.success, res.message = False, "save in progress"
            return res
        with self._lock:
            self._start, self._review = None, None
        self._set(MappingState.IDLE, "survey aborted; relaunch mapping for a clean graph")
        res.success, res.message = True, self._message
        return res

    def _srv_save(self, req: SaveMap.Request, res: SaveMap.Response):
        if self._state != MappingState.RETURN_REVIEW:
            res.ok, res.message = False, f"save needs RETURN_REVIEW (state {STATE_NAMES[self._state]})"
            return res
        problems = self._readiness()
        if problems:
            res.ok, res.message = False, "not ready: " + "; ".join(problems)
            return res
        self._set(MappingState.SAVING, "pausing SLAM and serialising")
        stage = None
        try:
            # allocation and staging are part of the transaction (review R27): an unwritable
            # maps dir must return a failed save and RETURN_REVIEW, not leave SAVING behind
            revision = mb.next_revision(self.maps_dir, self._map_id)
            stage = mb.staging_dir(self.maps_dir, self._map_id, revision)
            self._pause_slam(True)
            pause_t = self._now()
            self._call(self._serialize, SerializePoseGraph.Request(filename=os.path.join(stage, "posegraph")))
            grid_msg = self._wait_for_map(after=pause_t, timeout=2.5)
            grid = gridio.from_occupancy_grid_msg(grid_msg)
            review = mb.ReturnReview(**{**asdict(self._review), "note": req.note})
            manifest = mb.Manifest(
                map_id=self._map_id,
                revision=revision,
                created=mb.now_iso(),
                frame_id=grid_msg.header.frame_id or "map",
                resolution=grid.meta.resolution,
                origin=[grid.meta.origin_x, grid.meta.origin_y, grid.meta.origin_yaw],
                width=grid.width,
                height=grid.height,
                start=asdict(self._start),
                review=asdict(review),
                software={"slam": "slam_toolbox online_async", "profile": "slam_mapping.yaml"},
            )
            mb.stage_bundle(stage, grid, os.path.join(stage, "posegraph"), manifest)
            mb.verify(stage)
            dest = mb.publish(stage, self.maps_dir, self._map_id, revision)
        except Exception as e:  # noqa: BLE001 - any failure keeps the draft, never a revision
            draft = stage.replace(".staging-", ".draft-") if stage else None
            if stage:
                try:
                    os.rename(stage, draft)
                except OSError:
                    draft = stage
            try:
                self._pause_slam(False)  # let the operator keep surveying
            except Exception as e2:  # noqa: BLE001
                self.get_logger().error(f"could not resume SLAM after failed save: {e2}")
            kept = f"; draft kept at {draft}" if draft else "; nothing was staged"
            self._set(MappingState.RETURN_REVIEW, f"save failed: {e}{kept}")
            res.ok, res.message = False, self._message
            return res
        with self._lock:
            self._revision, self._saved_path = revision, dest
        self._set(MappingState.SAVED, f"published {dest} (bundle {manifest.sha256[:12]})")
        res.ok, res.path, res.revision, res.message = True, dest, revision, self._message
        return res

    # ---- slam_toolbox helpers ----------------------------------------------

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.srv_timeout):
            raise RuntimeError(f"{client.srv_name} unavailable")
        fut = client.call_async(request)
        deadline = time.monotonic() + self.srv_timeout
        while not fut.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f"{client.srv_name} timed out")
            time.sleep(0.02)
        if fut.exception() is not None:
            raise RuntimeError(f"{client.srv_name}: {fut.exception()}")
        return fut.result()

    def _pause_slam(self, paused: bool) -> None:
        """pause_new_measurements is a TOGGLE and its reply's `status` is only
        "call handled", not the new state, so the state is tracked here. It is
        assumed unpaused at launch, which is slam_toolbox's default."""
        if self._slam_paused == paused:
            return
        self._call(self._pause, Pause.Request())
        self._slam_paused = paused

    def _wait_for_map(self, after: float, timeout: float) -> OccupancyGrid:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._map is not None and self._map_t is not None and self._map_t >= after:
                return self._map
            time.sleep(0.05)
        if self._map is None:
            raise RuntimeError("no /map received from slam_toolbox")
        return self._map  # the latest is the final solution once paused


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingSession()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
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
