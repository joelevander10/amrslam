"""commissioning_node (unified plan §7.2): the /blind replacement.

    /amr/commissioning/plan   PlanCommissioning   validate + hold a plan (moves nothing)
    /amr/commissioning/clear  Trigger             drop the plan / abort a running job
    /amr/commissioning_state  CommissioningState  latched, on change + 2 Hz
    /amr/commissioning_wheels WheelVelocities     per-wheel setpoints while a pv job runs,
                                                  to the mux's COMMISSIONING source
    /amr/commissioning_pp     PpMove              the held pp move while a pp job runs
                                                  (and hold=false after it ends early),
                                                  to the drive owner
    /drives/pp_status         PpStatus  (in)      pp availability and the move's outcome

A job executes only on a fresh physical Start edge under a valid MANUAL panel
while the supervisor's lease carries the COMMISSIONING class (the supervisor
grants that - and withholds MANUAL - while this node reports PREPARED/RUNNING).
Every tick re-checks that authority; the mux and drive owner gate the output
again on their own. Evidence goes to <state_dir>/commissioning/<plan>-<ts>-<ns>.json,
written when a job ends, is cleared, or the node shuts down mid-job.
"""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from amr_base import commissioning as cj
from amr_interfaces.msg import (
    CommissioningState,
    ControlLease,
    ModeState,
    PanelState,
    PpMove,
    PpStatus,
    WheelStates,
    WheelVelocities,
)
from amr_interfaces.srv import PlanCommissioning

RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)
MODE_NAMES = {
    ModeState.STARTING: "STARTING",
    ModeState.IDLE: "IDLE",
    ModeState.MAPPING: "MAPPING",
    ModeState.NAVIGATION: "NAVIGATION",
    ModeState.TRANSITIONING: "TRANSITIONING",
    ModeState.FAULT: "FAULT",
    ModeState.STOPPING: "STOPPING",
}
LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
SENSOR = QoSProfile(
    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE
)


class CommissioningNode(Node):
    def __init__(self) -> None:
        super().__init__("commissioning_node")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("state_dir", os.path.expanduser("~/.amr"))
        self.declare_parameter("counts_fresh_s", 0.1)
        self.declare_parameter("still_wheel_rad_s", 0.02)
        self.declare_parameter("gyro_fresh_s", 0.1)
        p = self.get_parameter
        self.dt = 1.0 / float(p("rate_hz").value)
        self.evidence_dir = os.path.join(os.path.expanduser(str(p("state_dir").value)), "commissioning")
        self.counts_fresh = float(p("counts_fresh_s").value)
        self.still_thr = float(p("still_wheel_rad_s").value)
        self.gyro_fresh = float(p("gyro_fresh_s").value)
        self.job = cj.Job()
        self._wheels = None
        self._wheels_t = None
        self._panel = None
        self._panel_t = None
        self._start_edge_t: float | None = None
        self._lease = None
        self._mode: ModeState | None = None
        self._mode_t: float | None = None
        self._lease_t = None
        self._gyro = None
        self._gyro_t = None
        self._generation = 0
        self._last_phase = None
        self._seq = 0
        self._pp_view: cj.PpView | None = None
        self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, SENSOR)
        self.create_subscription(PanelState, "/amr/panel_state", self._on_panel, 10)
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, RELIABLE_1)
        self.create_subscription(ModeState, "/amr/mode_state", self._on_mode, LATCHED)
        self.create_subscription(Imu, "/imu/data", self._on_imu, SENSOR)
        self.create_subscription(PpStatus, "/drives/pp_status", self._on_pp_status, RELIABLE_1)
        self._pub_pp = self.create_publisher(PpMove, "/amr/commissioning_pp", RELIABLE_1)
        self._pub_cmd = self.create_publisher(WheelVelocities, "/amr/commissioning_wheels", RELIABLE_1)
        self._pub_state = self.create_publisher(CommissioningState, "/amr/commissioning_state", LATCHED)
        self.create_service(PlanCommissioning, "/amr/commissioning/plan", self._srv_plan)
        self.create_service(Trigger, "/amr/commissioning/clear", self._srv_clear)
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.5, self._publish_state)
        self._t_last = time.monotonic()
        self.get_logger().info(f"commissioning idle; evidence -> {self.evidence_dir}")

    # -- inputs --

    def _on_wheels(self, m: WheelStates) -> None:
        self._wheels, self._wheels_t = m, time.monotonic()

    def _on_panel(self, m: PanelState) -> None:
        self._panel, self._panel_t = m, time.monotonic()
        if m.start_edge:
            self._start_edge_t = time.monotonic()  # consumed by exactly one tick

    def _on_lease(self, m: ControlLease) -> None:
        cur = self._lease
        if cur is not None and cur.instance == m.instance:
            # Replayed or reordered leases of the same supervisor never move authority back.
            if int(m.generation) < int(cur.generation):
                return
            if int(m.generation) == int(cur.generation) and int(m.seq) <= int(cur.seq):
                return
        self._lease, self._lease_t = m, time.monotonic()
        self._generation = int(m.generation)

    def _on_pp_status(self, m: PpStatus) -> None:
        self._pp_view = cj.PpView(
            time.monotonic(),
            bool(m.available),
            str(m.reason),
            str(m.state),
            str(m.run_id),
            str(m.outcome),
            (int(m.left_target), int(m.right_target)),
            (int(m.left_actual), int(m.right_actual)),
        )

    def _on_imu(self, m: Imu) -> None:
        z = float(m.angular_velocity.z)
        if math.isfinite(z):
            self._gyro, self._gyro_t = z, time.monotonic()

    def _feedback(self, now: float) -> tuple[tuple[int, int] | None, float, bool | None]:
        """-> (raw counts, encoder scale, stopped) from FRESH, VALID wheel feedback only."""
        w = self._wheels
        fresh = w is not None and self._wheels_t is not None and now - self._wheels_t <= self.counts_fresh
        if not fresh:
            return None, 0.0, None
        counts = (int(w.left_counts), int(w.right_counts)) if w.counts_valid else None
        cpr = float(w.counts_per_wheel_rev)
        cpr = cpr if w.counts_valid and math.isfinite(cpr) and cpr > 0.0 else 0.0
        stopped = None
        if w.left_valid and w.right_valid:
            stopped = abs(w.left_vel_rad_s) <= self.still_thr and abs(w.right_vel_rad_s) <= self.still_thr
        return counts, cpr, stopped

    def _on_mode(self, m: ModeState) -> None:
        self._mode, self._mode_t = m, time.monotonic()

    def _mode_snapshot(self, now: float) -> tuple[str, int, str] | None:
        """(instance, generation, mode name) when the supervisor's mode is fresh (2 Hz, 1.5 s)."""
        if self._mode is None or self._mode_t is None or now - self._mode_t > 1.5:
            return None
        m = self._mode
        return (str(m.instance), int(m.generation), MODE_NAMES.get(int(m.mode), "?"))

    def _authority(self, now: float) -> tuple[tuple[str, int] | None, int]:
        if self._lease is None or self._lease_t is None or now - self._lease_t > 0.3:
            return None, 0
        return (str(self._lease.instance), int(self._lease.generation)), int(self._lease.allowed)

    # -- services (never move anything) --

    def _srv_plan(self, req, res):
        now = time.monotonic()
        _counts, cpr, stopped = self._feedback(now)
        authority, allowed = self._authority(now)
        try:
            res.planned_json = self.job.plan(
                req.plan_json,
                cpr,
                now=now,
                authority=authority,
                lease_allowed=allowed,
                stopped=stopped,
                mode=self._mode_snapshot(now),
                pp_status=self._pp_view,
            )
        except ValueError as e:
            res.ok, res.message = False, str(e)
            return res
        self._results_path = ""
        res.ok, res.message = True, f"plan {self.job.plan_id} held: press physical Start under MANUAL to run"
        self.get_logger().info(res.message)
        self._publish_state()
        return res

    def _srv_clear(self, req, res):
        ev = self.job.clear()
        if ev is not None:
            self._publish_wheels(0.0, 0.0)
            self._publish_pp_hold()  # hold=false at once, not on the owner's stale timeout
            self._write_evidence(ev)
        res.success, res.message = True, "cleared"
        self._publish_state()
        return res

    # -- the tick --

    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._t_last = now - self._t_last, now
        counts, cpr, stopped = self._feedback(now)
        authority, allowed = self._authority(now)
        panel_ok = self._panel is not None and self._panel_t is not None and now - self._panel_t <= 0.2
        start_edge_t, self._start_edge_t = self._start_edge_t, None
        gyro_ok = self._gyro_t is not None and now - self._gyro_t <= self.gyro_fresh
        inputs = cj.Inputs(
            now=now,
            dt=min(dt, 5 * self.dt),
            counts=counts,
            counts_per_rev=cpr,
            stopped=stopped,
            panel_valid=bool(panel_ok and self._panel.valid),
            panel_manual=bool(panel_ok and not self._panel.mode_auto),
            start_edge=start_edge_t is not None,
            lease_allowed=allowed,
            gyro_yaw_rad_s=self._gyro if gyro_ok else None,
            authority=authority,
            start_edge_t=start_edge_t,
            pp_status=self._pp_view,
        )
        before = self.job.phase
        wl, wr = self.job.tick(inputs)
        self._publish_pp_hold()
        if self.job.backend == "pp":
            pass  # the drive owner moves the wheels; nothing goes to the mux
        elif self.job.phase in (cj.RUNNING, cj.SETTLING):
            self._publish_wheels(wl, wr)
        elif before in (cj.RUNNING, cj.SETTLING):
            # Do not leave the previous nonzero sample live until the mux's
            # freshness timeout when a job completes or loses authority.
            self._publish_wheels(0.0, 0.0)
        if self.job.phase != before:
            a, b = cj.PHASE_NAMES[before], cj.PHASE_NAMES[self.job.phase]
            self.get_logger().info(f"commissioning {a} -> {b} {self.job.reason}")
            if self.job.phase in (cj.DONE, cj.ABORTED):
                self._write_evidence()
            self._publish_state()

    def _publish_pp_hold(self) -> None:
        hold = self.job.pp_hold()
        if hold is None or self.job.pp_spec is None:
            return
        run_id, held = hold
        s = self.job.pp_spec
        m = PpMove()
        m.header.stamp = self.get_clock().now().to_msg()
        m.run_id, m.generation, m.hold = run_id, self._generation, held
        self._seq += 1
        m.seq = self._seq
        m.left_delta_counts, m.right_delta_counts = s.left.delta_counts, s.right.delta_counts
        m.left_velocity_rpm, m.right_velocity_rpm = s.left.velocity_rpm, s.right.velocity_rpm
        m.left_accel_rpm_s, m.right_accel_rpm_s = s.left.accel_rpm_s, s.right.accel_rpm_s
        m.left_decel_rpm_s, m.right_decel_rpm_s = s.left.decel_rpm_s, s.right.decel_rpm_s
        m.duration_s = float(s.duration_s)
        self._pub_pp.publish(m)

    def _publish_wheels(self, left: float, right: float) -> None:
        m = WheelVelocities()
        m.header.stamp = self.get_clock().now().to_msg()
        m.generation = self._generation
        m.left_rad_s, m.right_rad_s = float(left), float(right)
        self._pub_cmd.publish(m)

    def _write_evidence(self, ev: dict | None = None) -> None:
        """Write one evidence file; `_results_path` names it only if it was fully written."""
        self._results_path = ""
        data = ev if ev is not None else self.job.evidence()
        data = dict(data, generation=self._generation)
        try:
            self._results_path = write_evidence(self.evidence_dir, str(data.get("plan_id", "")), data)
        except (OSError, TypeError, ValueError) as e:
            self.job.reason = f"{self.job.reason} (evidence NOT written: {e})".strip()
            try:
                self.get_logger().error(f"evidence not written: {e}")
            except Exception:  # noqa: BLE001 - logging may already be gone at shutdown
                pass

    _results_path = ""

    def finalize(self, why: str = "commissioning node shutting down") -> None:
        """Abort a running job and keep its evidence (orderly shutdown)."""
        ev = self.job.clear(why)
        if ev is None:
            return
        try:
            self._publish_wheels(0.0, 0.0)
            self._publish_pp_hold()
        except Exception:  # noqa: BLE001 - the context may already be shut down
            pass
        self._write_evidence(ev)

    def _publish_state(self) -> None:
        s = self.job.snapshot()
        m = CommissioningState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.phase = self.job.phase
        m.generation = self._generation
        m.plan_id = s.get("plan_id", "")
        m.segment = int(s.get("segment", 0))
        m.segments = int(s.get("segments", 0))
        m.kind = str(s.get("kind", ""))
        prog = s.get("progress_m", [0.0, 0.0])
        m.progress_left_m, m.progress_right_m = float(prog[0]), float(prog[1])
        m.speed_mps = float(s.get("speed_mps", 0.0))
        pose = s.get("pose", {})
        m.pose_x_m, m.pose_y_m, m.heading_deg = (
            float(pose.get("x_m", 0.0)),
            float(pose.get("y_m", 0.0)),
            float(pose.get("heading_deg", 0.0)),
        )
        m.gyro_heading_deg = float(s.get("gyro_heading_deg", 0.0))
        m.completed = int(s.get("completed", 0))
        m.reason = str(s.get("reason", "") or "")
        m.results_path = self._results_path if self.job.phase in (cj.DONE, cj.ABORTED) else ""
        m.backend = self.job.backend
        m.run_id = self.job.run_id
        if self.job.backend == "pp" and self.job.phase in (cj.RUNNING, cj.DONE, cj.ABORTED):
            counts, _cpr, _st = self._feedback(time.monotonic())
            m.progress_left_m, m.progress_right_m = (float(x) for x in self.job.pp_progress(counts))
        self._pub_state.publish(m)


def write_evidence(directory: str, plan_id: str, data: dict) -> str:
    """Exclusively create <directory>/<plan_id>-<ts>-<ns>.json; -> its path.

    The plan id is re-validated and the resolved path must stay inside the
    directory. A partial file (e.g. disk full) is removed and the error raised.
    """
    if not cj.PLAN_ID.fullmatch(plan_id):
        raise ValueError(f"unsafe plan id {plan_id!r}")
    os.makedirs(directory, exist_ok=True)
    base = os.path.realpath(directory)
    stem = f"{plan_id}-{time.strftime('%Y%m%d-%H%M%S')}-{time.time_ns() % 1_000_000_000:09d}"
    for n in range(100):
        path = os.path.realpath(os.path.join(base, f"{stem}.json" if n == 0 else f"{stem}-{n}.json"))
        if os.path.commonpath([base, path]) != base:
            raise ValueError("evidence path escapes the evidence directory")
        try:
            f = open(path, "x", encoding="utf-8")
        except FileExistsError:
            continue
        try:
            with f:
                json.dump(data, f, indent=1)
        except BaseException:
            try:
                os.unlink(path)
            except OSError:
                pass
            raise
        return path
    raise OSError("no unique evidence file name")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommissioningNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.finalize()
        except Exception:  # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
