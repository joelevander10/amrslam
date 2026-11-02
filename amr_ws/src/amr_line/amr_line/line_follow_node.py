"""line_follow_node (dual-product plan, Increment 1): magnetic-tape following.

    /amr/line/clear      Trigger            abort a running follow (back to IDLE)
    /amr/line/mission    SelectMission      the next run's job: mission + destination
    /amr/line_state      LineState          latched, on change + 1 Hz
    /amr/line_cmd        Twist              body command while following,
                                            to the mux's LINE source
    /amr/line_track      LineTrack  (in)    the MLS reading from drive_node
    /amr/line/teach      LineTeach          tape-graph teaching (tracked plan 3, P1; engineer)
    /amr/line/at_home    Trigger            parked at HOME-N: /initialpose from its taught pose
    /amcl_pose, /amr/localization_state (in) AMCL and readiness, when the layer runs them

Tape localization (tracked plan 3) runs in SHADOW only (P1): amr_line.tape_runtime logs
where the locator would stop and branch, and records teach passes. Nothing it computes
reaches the job or the command.

There is no software arm (2026-10-07): the physical Start from IDLE checks the
prerequisites and runs in one step (FollowJob._start). Motor arming is drive_node's.

The vehicle follows only on a fresh physical Start edge, under a valid AUTO
panel, while the supervisor's lease carries LEASE_LINE - which it grants
EXCLUSIVELY in LINE mode, so no browser jog and no pendant can fight the tape.
Every tick re-checks that authority; the mux and the drive owner gate the
output again on their own.

*** The control law is not in this file. *** `amr_line.autopilot` is a
byte-for-byte port of the engine that ran this vehicle as a tape AGV, and
`amr_line.job` holds the authority state machine, ROS-free and unit-tested.
This node marshals messages, stamps freshness and owns the clock. Keep it that
way: the decisions worth testing must stay where a test can reach them without
a ROS graph.

*** Command shape: a body Twist, not per-wheel. *** The follower computes
(v, omega) internally and only converts to wheels at the end, so publishing
the body pair costs one exact round trip - kinematics.wheels_to_body and
body_to_wheels both apply the invert_left/invert_right rule, so it cancels -
and it puts LINE on the same accel path the mux already gives FOLLOW.
"""

from __future__ import annotations

import collections
import math
import os
import time

import rclpy
from agv_core import alarms, kinematics
from agv_core import config as vehicle_config
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from amr_interfaces.msg import (
    ControlLease,
    DriveStatus,
    Event,
    LineState,
    LineTrack,
    LocalizationState,
    MarkerEvent,
    ModeState,
    MuxState,
    PanelState,
    StationDetection,
    WheelStates,
)
from amr_interfaces.srv import LineTeach, SelectMission
from amr_line import autopilot, runtime, tape_health
from amr_line import job as lj
from amr_line import track as tk

try:
    # The protective field's real state comes from the SICK driver, the same
    # source and the same index the executor uses. Optional exactly as it is
    # there: the package is absent on a dev box, and a layer that cannot be
    # imported without the scanner driver cannot be unit-tested at all.
    from sick_safetyscanners2_interfaces.msg import OutputPaths
except ImportError:  # pragma: no cover - present on the vehicle
    OutputPaths = None

RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
SENSOR = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)

# The state machine's constants are LineState's constants. If a renumbering
# ever slips through, fail here at construction rather than let readiness.py's
# `line_state in (1, 2, 3)` quietly stop guarding a moving vehicle.
_WIRE = (
    (lj.IDLE, LineState.IDLE), (lj.ARMED, LineState.ARMED),
    (lj.RUNNING, LineState.RUNNING), (lj.HOLD, LineState.HOLD),
    (lj.DONE, LineState.DONE), (lj.FAULT, LineState.FAULT),
)


class LineFollowNode(Node):
    def __init__(self) -> None:
        super().__init__("line_follow_node")
        for mine, wire in _WIRE:
            if mine != wire:
                raise RuntimeError(
                    f"amr_line.job state {mine} does not match LineState {wire}: "
                    "readiness._line_active_locked depends on ARMED/RUNNING/HOLD "
                    "being 1..3. Fix the message or the module, not this check."
                )

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("generation", 0)
        self.declare_parameter("prereq_grace_s", 0.5)
        # -1 = the profile's auto_resume_hold_s; a value here overrides it.
        self.declare_parameter("auto_resume_clear_s", -1.0)
        self.declare_parameter("auto_resume_estop", False)
        self.declare_parameter("accept_sdo_track", False)
        self.declare_parameter("panel_fresh_s", 0.2)
        self.declare_parameter("lease_fresh_s", 0.3)
        self.declare_parameter("drives_fresh_s", 0.5)
        self.declare_parameter("field_output_index", 0)  # /output_paths status[i]
        # "scanner": the protective field is what a FRESH /output_paths says,
        # and unknown otherwise (a Start is refused and a torque loss is
        # an E-stop). "assume_clear": the sim, which has no scanner. The launch
        # file picks by the supervisor's `real`; never assume on a vehicle.
        self.declare_parameter("field_source", "scanner")
        self.declare_parameter("field_fresh_s", 0.5)  # /output_paths rides every scan (~34 Hz)
        # Mission engine inputs (2026-10-02)
        self.declare_parameter("rfid_fresh_s", 1.5)  # heartbeat is 2 Hz
        self.declare_parameter("wheels_fresh_s", 0.1)
        self.declare_parameter("still_wheel_rad_s", 0.02)
        # Speed engine inputs (tracked-speed-plan-1, 2026-10-08): the curve guard's yaw
        # rate and speed, and the mux's warning-field factor for the U-turn budget.
        self.declare_parameter("imu_fresh_s", 0.1)
        self.declare_parameter("odom_fresh_s", 0.1)
        self.declare_parameter("mux_fresh_s", 0.3)  # /amr/mux_state is 10 Hz
        # Tape localization (tracked plan 3): set by line_layer.launch.py from the profile's
        # tape_loc block and the active pointer ($AMR_STATE_DIR/line_graph.json).
        self.declare_parameter("loc_status", "off: tape_loc.enabled is false")
        self.declare_parameter("loc_map_id", "")
        self.declare_parameter("loc_map_revision", -1)
        self.declare_parameter("loc_graph_revision", -1)
        self.declare_parameter("maps_dir", "~/amr_maps")
        self.declare_parameter("state_dir", "~/.amr")

        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self._generation = int(self.get_parameter("generation").value)
        self.panel_fresh = float(self.get_parameter("panel_fresh_s").value)
        self.lease_fresh = float(self.get_parameter("lease_fresh_s").value)
        self.drives_fresh_s = float(self.get_parameter("drives_fresh_s").value)
        self.field_index = int(self.get_parameter("field_output_index").value)
        if self.field_index < 0:
            raise ValueError(f"field_output_index {self.field_index} < 0")
        self.field_source = str(self.get_parameter("field_source").value)
        if self.field_source not in ("scanner", "assume_clear"):
            raise ValueError(f"field_source {self.field_source!r}: scanner or assume_clear")
        self.field_fresh = float(self.get_parameter("field_fresh_s").value)

        # The engine is handed its constants; it never reads a profile itself.
        # `runtime` carries ONLY the names the engine reads. The layer's own
        # settings (rate floor, auto-resume window) come straight from the
        # profile: a getattr-with-default on `runtime` silently read 40 Hz
        # whatever the profile said (review finding 2026-09-21, F06).
        runtime.load_from_profile()
        auto_clear = float(self.get_parameter("auto_resume_clear_s").value)
        if auto_clear < 0:
            auto_clear = float(vehicle_config.AUTO_RESUME_HOLD_S)
        self.job = lj.FollowJob(
            autopilot.LineFollower(),
            prereq_grace_s=float(self.get_parameter("prereq_grace_s").value),
            auto_resume_clear_s=auto_clear,
            auto_resume_estop=bool(self.get_parameter("auto_resume_estop").value),
            premove_s=float(vehicle_config.AUTO_START_DELAY_S),
            premove_timeout_s=float(vehicle_config.PREMOVE_TIMEOUT_S),
        )
        # No job yet is plain line following, with the site's U-turn tags.
        _ok, msg = self.job.set_mission(self._plain_mission())
        self.get_logger().info(f"mission at start: {msg}")
        self.reader = tk.TrackReader(
            timeout_s=runtime.SENSOR_TIMEOUT_S,
            min_hz=float(vehicle_config.LINE_MIN_TRACK_HZ),
            accept_sdo=bool(self.get_parameter("accept_sdo_track").value),
        )

        self._panel = None
        self._panel_t = None
        self._start_edge_t = None
        self._reset_edge = False
        self._lease = None
        self._lease_t = None
        self._drives_t = None
        self._torque_off_msg = False
        self._field_clear: bool | None = None  # last /output_paths verdict
        self._field_t: float | None = None     # ...and when it arrived (monotonic)
        self._t_last = time.monotonic()
        self._published_zero = True
        # RFID: the encounter stream rebuilt as the driver's snapshot shape.
        self.rfid_fresh = float(self.get_parameter("rfid_fresh_s").value)
        self._rfid_t: float | None = None
        self._rfid_status = {"comms_ok": False, "encounter_seq": 0, "generation": 0, "tag_age_s": None}
        self._rfid_encounters: collections.deque = collections.deque(maxlen=256)
        # MLS markers (mls-marker-plan 6): the same snapshot shape, rebuilt from /amr/line_marker.
        self._marker_t: float | None = None
        self._marker_status = {"ok": False, "status": "no marker data", "generation": None, "base": 0}
        self._marker_events: collections.deque = collections.deque(maxlen=64)
        # wheels: encoder counts for the U-turn and a stillness test
        self.wheels_fresh = float(self.get_parameter("wheels_fresh_s").value)
        self.still_w = float(self.get_parameter("still_wheel_rad_s").value)
        self._wheels: WheelStates | None = None
        self._wheels_t: float | None = None
        self.imu_fresh = float(self.get_parameter("imu_fresh_s").value)
        self.odom_fresh = float(self.get_parameter("odom_fresh_s").value)
        self.mux_fresh = float(self.get_parameter("mux_fresh_s").value)
        self._imu_wz: float | None = None
        self._imu_t: float | None = None
        self._odom: tuple[float, float] | None = None  # (v, wz)
        self._odom_t: float | None = None
        self._mux_scale = 1.0
        self._mux_t: float | None = None

        self.create_subscription(LineTrack, "/amr/line_track", self._on_track, SENSOR)
        self.create_subscription(PanelState, "/amr/panel_state", self._on_panel, 10)
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, RELIABLE_1)
        self.create_subscription(ModeState, "/amr/mode_state", self._on_mode, LATCHED)
        self.create_subscription(DriveStatus, "/drives/status", self._on_drives, RELIABLE_1)
        rfid_qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(StationDetection, "/amr/rfid", self._on_rfid, rfid_qos)
        # Same QoS as RFID: reliable, volatile - every marker is a message, a replay never new.
        self.create_subscription(MarkerEvent, "/amr/line_marker", self._on_marker, rfid_qos)
        self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, SENSOR)
        self.create_subscription(Imu, "/imu/data", self._on_imu, SENSOR)
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, SENSOR)
        self.create_subscription(MuxState, "/amr/mux_state", self._on_mux, RELIABLE_1)
        if self.field_source == "assume_clear":
            self.get_logger().warning(
                "field_source=assume_clear: the protective field is ASSUMED clear "
                "(simulation contract). Never run a vehicle like this."
            )
        elif OutputPaths is not None:
            self.create_subscription(OutputPaths, "/output_paths", self._on_output_paths, SENSOR)
        else:
            self.get_logger().error(
                "sick_safetyscanners2_interfaces is not installed: the protective "
                "field stays UNKNOWN and this layer will not start. Install the "
                "scanner driver, or launch with field_source:=assume_clear in a sim."
            )

        self._pub_cmd = self.create_publisher(Twist, "/amr/line_cmd", RELIABLE_1)
        self._pub_state = self.create_publisher(LineState, "/amr/line_state", LATCHED)
        # Operator events on LINE-STATE changes only: the tick runs at 50 Hz.
        self._pub_event = self.create_publisher(Event, "/amr/events", 50)
        self._event_seq = 0
        self._event_state: int | None = None
        self._power_published: bool | None = None  # LineState.drive_power last sent

        self.create_service(Trigger, "/amr/line/clear", self._srv_clear)
        self.create_service(SelectMission, "/amr/line/mission", self._srv_mission)

        self.tapeloc = self._tape_runtime()
        # MLS track level per position along the taught tape (mls-diagnostics-plan M3): written
        # at the end of each run, only where the locator was HEALTHY. Display/maintenance only.
        self.tape_health = tape_health.TapeHealth()
        self._state_dir = os.path.expanduser(str(self.get_parameter("state_dir").value))
        self._pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", RELIABLE_1)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl,
                                 QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                                            durability=QoSDurabilityPolicy.VOLATILE))
        self.create_subscription(LocalizationState, "/amr/localization_state", self._on_readiness, LATCHED)
        self.create_service(LineTeach, "/amr/line/teach", self._srv_teach)
        self.create_service(Trigger, "/amr/line/at_home", self._srv_at_home)

        self.create_timer(self.dt, self._tick)
        self.create_timer(1.0, self._publish_state)
        self._publish_state()
        self.get_logger().info(
            f"line layer up, generation {self._generation}: "
            f"k_ratio={runtime.K_RATIO} kd={runtime.KD} "
            f"normal={vehicle_config.AUTO_RPM:g} rpm high={vehicle_config.AUTO_HIGH_RPM:g} rpm "
            f"rate floor={self.reader.min_hz} Hz "
            f"auto-resume clear={self.job.auto_resume_clear_s} s"
        )

    # -- tape localization (tracked plan 3, P1 shadow) -------------------------
    def _tape_runtime(self):
        """The teach/shadow runtime, or None when the site mission does not load (logged)."""
        from agv_core import mission  # noqa: PLC0415

        from amr_line import tape_locator, tape_runtime  # noqa: PLC0415

        p = self.get_parameter
        name = str(vehicle_config.TAPE_LOC_SITE_MISSION)
        try:
            site = mission.load_site(name)
        except mission.MissionError as e:
            self.get_logger().error(f"tape localization unavailable: {e}")
            return None
        status = str(p("loc_status").value)
        rev = int(p("loc_map_revision").value)
        g = int(p("loc_graph_revision").value)
        selection = None
        if not status and p("loc_map_id").value and rev >= 0:
            selection = {"map_id": str(p("loc_map_id").value), "map_revision": rev,
                         "graph_revision": g if g >= 0 else None}
        maps_dir = os.path.expanduser(str(p("maps_dir").value))
        rt = tape_runtime.TapeRuntime(
            site, tape_locator.Params.from_profile(vehicle_config), maps_dir=maps_dir,
            state_dir=os.path.expanduser(str(p("state_dir").value)), selection=selection,
            resolve_sha=lambda m, r: self._bundle_sha(maps_dir, m, r), status=status,
            shadow_circuit=str(vehicle_config.TAPE_LOC_SHADOW_CIRCUIT))
        self.get_logger().info(f"tape localization: {rt.detail}")
        return rt

    @staticmethod
    def _bundle_sha(maps_dir, map_id, revision):
        from amr_mission import map_bundle as mb  # noqa: PLC0415

        rev_dir = mb.revision_dir(maps_dir, map_id, int(revision))
        return mb.load_manifest(rev_dir, map_id, int(revision)).sha256

    def _ros_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        if self.tapeloc is None:
            return
        q = m.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c = m.pose.covariance
        self.tapeloc.amcl(self._ros_now(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                          m.pose.pose.position.x, m.pose.pose.position.y, yaw, c[0], c[1], c[7])

    def _on_readiness(self, m: LocalizationState) -> None:
        if self.tapeloc is not None and int(m.generation) == self._generation:
            self.tapeloc.readiness(self._ros_now(), int(m.state), str(m.reason))

    def _travel(self, now: float):
        """Signed forward wheel travel, m (vehicle terms, as diff_drive_odom), or None."""
        w = self._wheels
        if w is None or self._wheels_t is None or now - self._wheels_t > self.wheels_fresh:
            return None
        if not (w.left_valid and w.right_valid):
            return None
        return 0.5 * vehicle_config.WHEEL_DIA_M / 2.0 * (w.left_pos_rad + w.right_pos_rad)

    def _tapeloc_tick(self, now: float) -> None:
        rt = self.tapeloc
        if rt is None:
            return
        _counts, _per_rev, still = self._wheel_inputs(now)
        u = self.job.tape.uturn_snapshot() or {}
        rt.tick(self._ros_now(), self._travel(now), (self.reader.sensor() or {}).get("nlcp"), still,
                (u.get("phase") or "") if u.get("active") else "")
        self.tape_health.where(rt.snapshot() if rt.loc is not None else None)
        events = rt.drain_events()
        for level, text in events:
            self._event("LINE_LOC", text, Event.WARN if level == "warn" else Event.INFO)
        if events:
            self._publish_state()

    def _srv_teach(self, req, res):
        if self.tapeloc is None:
            res.ok, res.message = False, "tape localization unavailable (site mission refused: see the log)"
        else:
            res.ok, res.message = self.tapeloc.teach(self._ros_now(), req.action.strip(), req.arg)
        self.get_logger().info(f"teach {req.action} {req.arg}: {res.message}")
        self._event("LINE_LOC", f"teach {req.action}: {res.message}", Event.INFO if res.ok else Event.WARN)
        self._publish_state()
        return res

    def _srv_at_home(self, req, res):
        if self.tapeloc is None:
            res.success, res.message = False, "tape localization unavailable"
            return res
        ok, msg, pose = self.tapeloc.at_home()
        if ok:
            m = PoseWithCovarianceStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.pose.pose.position.x, m.pose.pose.position.y = float(pose[0]), float(pose[1])
            m.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
            m.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
            m.pose.covariance[35] = 0.03 ** 2
            self._pub_initialpose.publish(m)
        res.success, res.message = ok, msg
        self._event("LINE_LOC", msg, Event.INFO if ok else Event.WARN)
        self._publish_state()
        return res

    # -- intake ------------------------------------------------------------
    def _on_track(self, m: LineTrack) -> None:
        self.reader.update(m, time.monotonic())
        self.tape_health.track(m.track_level, m.nlcp, m.event_flag, m.source)

    def _on_panel(self, m: PanelState) -> None:
        self._panel, self._panel_t = m, time.monotonic()
        if m.start_edge:
            self._start_edge_t = time.monotonic()  # consumed by exactly one tick
        if m.reset_edge:
            self._reset_edge = True

    def _on_lease(self, m: ControlLease) -> None:
        cur = self._lease
        if cur is not None and cur.instance == m.instance:
            # Replayed or reordered leases never move authority backwards.
            if int(m.generation) < int(cur.generation):
                return
            if int(m.generation) == int(cur.generation) and int(m.seq) <= int(cur.seq):
                return
        self._lease, self._lease_t = m, time.monotonic()

    def _on_rfid(self, m: StationDetection) -> None:
        now = time.monotonic()
        if int(m.generation) != self._rfid_status["generation"]:
            self._rfid_encounters.clear()  # the driver cleared its buffer on reconnect
        if not m.heartbeat and m.rfid_tag:
            self._rfid_encounters.append((int(m.encounter_seq), str(m.rfid_tag)))
        self._rfid_status = {
            "comms_ok": bool(m.comms_ok),
            "encounter_seq": max(int(m.encounter_seq), self._rfid_status["encounter_seq"])
            if int(m.generation) == self._rfid_status["generation"] else int(m.encounter_seq),
            "generation": int(m.generation),
            "tag_age_s": None if m.tag_age_s < 0 else float(m.tag_age_s),
        }
        self._rfid_t = now

    def _on_marker(self, m: MarkerEvent) -> None:
        gen = int(m.generation)
        st = self._marker_status
        if gen != st["generation"]:
            # drive_node restarted its marker stream: what came before is not continuous.
            self._marker_events.clear()
            st["generation"] = gen
            st["base"] = int(m.seq) if m.heartbeat else max(0, int(m.seq) - 1)
        if not m.heartbeat and m.code:
            self._marker_events.append((int(m.seq), int(m.code), int(m.direction), int(m.lcp2_mm),
                                        bool(m.line_good)))
        st["ok"], st["status"] = bool(m.markers_ok), str(m.status)
        self._marker_t = time.monotonic()

    def _markers(self, now: float) -> dict:
        """The marker snapshot. A silent stream (no heartbeat for 2.5 s) is not ok."""
        fresh = self._marker_t is not None and now - self._marker_t <= 2.5
        st = dict(self._marker_status)
        if not fresh:
            st["ok"], st["status"] = False, "no marker data"
        st["encounters"] = list(self._marker_events)
        return st

    def _rfid(self, now: float) -> dict:
        """agv_core.drivers.rfid snapshot shape. A silent node is a link that is down."""
        fresh = self._rfid_t is not None and now - self._rfid_t <= self.rfid_fresh
        st = dict(self._rfid_status)
        st["comms_ok"] = bool(fresh and st["comms_ok"])
        if st["tag_age_s"] is not None and self._rfid_t is not None:
            st["tag_age_s"] += now - self._rfid_t
        st["encounters"] = list(self._rfid_encounters)
        return st

    def _on_wheels(self, m: WheelStates) -> None:
        self._wheels, self._wheels_t = m, time.monotonic()

    def _on_imu(self, m: Imu) -> None:
        self._imu_wz, self._imu_t = float(m.angular_velocity.z), time.monotonic()

    def _on_odom(self, m: Odometry) -> None:
        self._odom = (float(m.twist.twist.linear.x), float(m.twist.twist.angular.z))
        self._odom_t = time.monotonic()

    def _on_mux(self, m: MuxState) -> None:
        self._mux_scale, self._mux_t = float(m.speed_scale), time.monotonic()

    def _motion_inputs(self, now: float):
        """(yaw_rate, v_meas, warning_scale). Gyro first, odometry as the fallback;
        None when neither is fresh, which takes HIGH away. A stale mux reads as no
        warning: the U-turn budget then runs, which faults rather than waits."""
        odom_ok = self._odom_t is not None and now - self._odom_t <= self.odom_fresh
        imu_ok = self._imu_t is not None and now - self._imu_t <= self.imu_fresh
        v = self._odom[0] if odom_ok else None
        yaw = self._imu_wz if imu_ok else (self._odom[1] if odom_ok else None)
        mux_ok = self._mux_t is not None and now - self._mux_t <= self.mux_fresh
        return yaw, v, (self._mux_scale if mux_ok else 1.0)

    def _wheel_inputs(self, now: float):
        """(counts, counts_per_rev, still) from a FRESH /wheel_states, else nothing."""
        w = self._wheels
        if w is None or self._wheels_t is None or now - self._wheels_t > self.wheels_fresh:
            return None, 0.0, False
        still = (w.left_valid and w.right_valid and abs(w.left_vel_rad_s) <= self.still_w
                 and abs(w.right_vel_rad_s) <= self.still_w)
        counts = (int(w.left_counts), int(w.right_counts)) if w.counts_valid else None
        return counts, float(w.counts_per_wheel_rev), bool(still)

    def _on_mode(self, m: ModeState) -> None:
        self._mode = m

    def _on_drives(self, m: DriveStatus) -> None:
        # torque off = not operational AND neither statusword says Operation enabled
        self._torque_off_msg = (
            not m.operational and "Operation enabled" not in (m.left_state, m.right_state)
        )
        self._drives_t = time.monotonic()

    def _on_output_paths(self, m) -> None:
        status = getattr(m, "status", None)
        if status is None or len(status) <= self.field_index:
            return  # a malformed array is no evidence at all: the old one ages out
        self._field_clear = bool(status[self.field_index])
        self._field_t = time.monotonic()

    def _field(self, now: float) -> bool | None:
        """clear / violated / None (unknown: no sample, or the stream stalled)."""
        if self.field_source == "assume_clear":
            return True
        if self._field_t is None or now - self._field_t > self.field_fresh:
            return None
        return self._field_clear

    # -- authority ---------------------------------------------------------
    def _authority(self, now: float):
        if self._lease is None or self._lease_t is None or now - self._lease_t > self.lease_fresh:
            return None, 0
        return (str(self._lease.instance), int(self._lease.generation)), int(self._lease.allowed)

    def _inputs(self, now: float, dt: float) -> lj.Inputs:
        authority, allowed = self._authority(now)
        panel_ok = (
            self._panel is not None
            and self._panel_t is not None
            and now - self._panel_t <= self.panel_fresh
        )
        start_edge_t, self._start_edge_t = self._start_edge_t, None
        reset_edge, self._reset_edge = self._reset_edge, False
        track_ok, track_cause = self.reader.usable(now)
        counts, per_rev, still = self._wheel_inputs(now)
        yaw, v_meas, scale = self._motion_inputs(now)
        return lj.Inputs(
            now=now,
            dt=dt,
            sensor=self.reader.sensor(),
            sensor_age_s=self.reader.age_s(now) or 0.0,
            track_ok=track_ok,
            track_cause=track_cause,
            panel_valid=bool(panel_ok and self._panel.valid),
            panel_auto=bool(panel_ok and self._panel.mode_auto),
            start_edge=start_edge_t is not None,
            start_edge_t=start_edge_t,
            reset_edge=reset_edge,
            lease_allowed=allowed,
            lease_line=ControlLease.LINE,
            authority=authority,
            torque_off=bool(
                self._torque_off_msg
                and self._drives_t is not None
                and now - self._drives_t <= self.drives_fresh_s
            ),
            field_clear=self._field(now),
            drives_fresh=bool(
                self._drives_t is not None and now - self._drives_t <= self.drives_fresh_s
            ),
            rfid=self._rfid(now),
            markers=self._markers(now),
            counts=counts,
            counts_per_rev=per_rev,
            wheels_still=still,
            yaw_rate=yaw,
            v_meas=v_meas,
            warning_scale=scale,
        )

    # -- services ----------------------------------------------------------
    def _srv_clear(self, req, res):
        if self.job.clear():
            # Do not leave the last nonzero command live until the mux's
            # freshness timeout: zero it now.
            self._publish_cmd(0.0, 0.0)
        res.success, res.message = True, "cleared"
        self.get_logger().info("clear: the run is cleared")
        self._publish_state()
        return res

    def _plain_mission(self):
        """missions/empty.json (the site's U-turn tags), or None - no tags at all -
        when it is missing or refused, so a bad file never blocks line following."""
        from agv_core import mission  # noqa: PLC0415

        try:
            return mission.load("empty")
        except mission.MissionError as e:
            self.get_logger().error(f"plain line following without U-turn tags: {e}")
            return None

    def _srv_mission(self, req, res):
        from agv_core import mission  # noqa: PLC0415

        name = (req.name or "").strip()
        try:
            doc = self._plain_mission() if name in ("", "none") else mission.load(name)
        except mission.MissionError as e:
            res.ok, res.message = False, f"mission {name!r} refused: {e}"
        else:
            res.ok, res.message = self.job.set_mission(doc, (req.destination or "").strip())
        self.get_logger().info(f"mission: {res.message}")
        self._publish_state()
        return res

    # -- the tick ----------------------------------------------------------
    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._t_last = now - self._t_last, now
        before = self.job.state

        left, right = self.job.tick(self._inputs(now, min(dt, 5 * self.dt)))
        self._tapeloc_tick(now)
        engine_events = self.job.tape.drain_events()
        for level, text in engine_events:
            self._event("LINE_MISSION", text, Event.WARN if level == "warn" else Event.INFO)
        marker_events = self.job.drain_marker_events()
        for level, text in marker_events:
            self._event("LINE_MARKER", text, Event.WARN if level == "warn" else Event.INFO)
        if engine_events or marker_events:
            self._publish_state()
        if self.job.refused:
            self._event("LINE_MISSION", f"Start refused: {self.job.refused}", Event.WARN)
            self.job.refused = None
            self._publish_state()

        if self.job.state == lj.RUNNING:
            v, omega = kinematics.wheels_to_body(left, right)
            self._publish_cmd(v, omega)
        elif before == lj.RUNNING:
            # Explicit zero on the falling edge, for the same reason as above.
            self._publish_cmd(0.0, 0.0)

        if self.job.state != before:
            self.get_logger().info(
                f"line {lj.STATE_NAMES[before]} -> {lj.STATE_NAMES[self.job.state]}"
                f" {self.job.reason}"
            )
            if before in (lj.RUNNING, lj.HOLD) and self.job.state in (lj.DONE, lj.FAULT, lj.IDLE):
                self._write_tape_health(lj.STATE_NAMES[self.job.state])
            self._publish_state()
        elif self.job.wants_power() != self._power_published:
            self._publish_state()  # a hold changing cause can change drive_power: say so now

    def _write_tape_health(self, result: str) -> None:
        """End of a run: append its tape-health bins (M3). A disk problem is logged, never raised."""
        bins = self.tape_health.drain()
        try:
            meta = {"generation": self._generation, "result": result}
            path = tape_health.write_run(self._state_dir, bins, meta)
        except OSError as e:
            self.get_logger().warn(f"tape health not written: {e}")
            return
        if path:
            self.get_logger().info(f"tape health: {len(bins)} bins -> {path}")

    # -- output ------------------------------------------------------------
    def _publish_cmd(self, v: float, omega: float) -> None:
        if v == 0.0 and omega == 0.0 and self._published_zero:
            return
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(omega)
        self._pub_cmd.publish(m)
        self._published_zero = v == 0.0 and omega == 0.0

    def _publish_state(self) -> None:
        d = getattr(self.job, "diag", {}) or {}
        m = LineState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.generation = self._generation
        m.state = self.job.state
        m.hold_cause = self.job.hold_cause
        m.auto_resume = self.job.auto_resume()
        m.drive_power = bool(self.job.wants_power())
        self._power_published = m.drive_power
        m.engine_state = str(d.get("state", "idle"))
        m.has_track = bool(d.get("has_track", False))
        m.error_mm = float(d.get("e_mm") or 0.0)
        v, omega = (0.0, 0.0)
        if self.job.state == lj.RUNNING and d:
            v, omega = kinematics.wheels_to_body(d.get("n_l", 0.0), d.get("n_r", 0.0))
        m.v_mps, m.omega_rad_s = float(v), float(omega)
        m.followed_m = float(self.job.followed_m)
        m.track_hz = float(self.reader.hz() or 0.0)
        m.sample_age_s = float(self.reader.age_s(time.monotonic()) or 0.0)
        m.track_source = "" if self.reader.last is None else str(self.reader.last.source)
        m.message = self.job.reason
        m.code = self._code()
        for k, v in self.job.mission_snapshot().items():
            setattr(m, k, v)
        for k, v in self.job.marker_snapshot().items():
            setattr(m, k, v)
        if getattr(self, "tapeloc", None) is not None:
            for k, v in self.tapeloc.snapshot().items():
                setattr(m, k, v)
        self._pub_state.publish(m)
        if self.job.state != self._event_state:
            self._event_state = self.job.state
            self._event(m.code, m.message)

    def _code(self) -> str:
        """Alarm catalogue code for the current hold or fault; "" while it is driving."""
        if self.job.state in (lj.HOLD, lj.FAULT):
            if self.job.hold_cause:
                return alarms.hold_code(self.job.hold_cause)
            return "EXEC_PREREQ_LOST" if self.job.state == lj.FAULT else "WAITING_FOR_PREREQ"
        return ""

    def _event(self, code: str, detail: str, level: int | None = None) -> None:
        code = code or "MODE_CHANGE"
        row = alarms.get(code)
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source = "line_follow"
        m.level = level if level is not None else {
            alarms.ERROR: Event.ERROR, alarms.WARN: Event.WARN}.get(row.severity, Event.INFO)
        m.code = code
        m.text = f"line {lj.STATE_NAMES[self.job.state]}: {detail}"
        self._event_seq += 1
        m.seq = self._event_seq
        self._pub_event.publish(m)


def main() -> None:
    rclpy.init()
    node = LineFollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.job.clear()
            node._publish_cmd(0.0, 0.0)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
