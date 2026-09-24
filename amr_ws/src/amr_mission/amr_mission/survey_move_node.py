"""survey_move_node: preset moves while surveying (mapping layer only), press once.

    /amr/survey/move       SurveyMove   "straight" X m (+ fwd / - rev) or "rotate" 45..180 deg
    /amr/survey/move_stop  Trigger      stop the running move
    /amr/survey_move_state SurveyMoveState, latched

The move is driven as a browser-style ManualCommand of its own session, refreshed at
rate_hz with a cmd_valid_s lifetime, so it moves only under the MANUAL authority the jog pad
has (physical selector MANUAL, the supervisor's manual lease, the mux's S-curve and survey
spin cap), and it stops within cmd_valid_s if this node dies. The pendant outranks it at the
mux; here ANY pendant button, the selector leaving MANUAL, the drives losing torque (E-stop),
another jog session or Stop, a stale input, or the survey ending aborts the move.

After a completed move the node waits settle_s for SLAM to take the last scans, then reports
how far SLAM moved the vehicle's map pose relative to odometry during the move
(survey_move.slam_correction): a heading correction over max_correction_deg flags a likely
wrong scan match while the operator is still there to redo the stretch.
"""

from __future__ import annotations

import math
import threading
import uuid

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from amr_interfaces.msg import (
    ControlLease,
    DriveStatus,
    ManualCommand,
    MappingState,
    PanelState,
    SurveyMoveState,
)
from amr_interfaces.srv import SurveyMove
from amr_mission import survey_move as sm

LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
RELIABLE_1 = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)
LEASE_MANUAL = 1


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SurveyMoveNode(Node):
    def __init__(self) -> None:
        super().__init__("survey_move")
        dp = self.declare_parameter
        dp("rate_hz", 20.0)
        dp("cmd_valid_s", 0.2)
        dp("settle_s", 1.5)
        dp("input_age_s", 0.5)
        dp("max_correction_deg", 2.0)
        dp("v_mps", 0.30)
        dp("w_rad_s", 0.20)
        p = self.get_parameter
        self.cmd_valid = float(p("cmd_valid_s").value)
        self.settle_s = float(p("settle_s").value)
        self.input_age = float(p("input_age_s").value)
        self.max_corr_deg = float(p("max_correction_deg").value)
        self.profile = sm.Profile(v_mps=float(p("v_mps").value), w_rad_s=float(p("w_rad_s").value))
        self._init_state()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10)
        self.create_subscription(ControlLease, "/amr/control_lease", self._on_lease, 10)
        self.create_subscription(PanelState, "/amr/panel_state", self._on_panel, 10)
        self.create_subscription(DriveStatus, "/drives/status", self._on_drives, 10)
        self.create_subscription(ManualCommand, "/amr/manual_command", self._on_manual, 10)
        self.create_subscription(MappingState, "/amr/mapping_state", self._on_mapping, LATCHED)
        self._cmd_pub = self.create_publisher(ManualCommand, "/amr/manual_command", RELIABLE_1)
        self._state_pub = self.create_publisher(SurveyMoveState, "/amr/survey_move_state", LATCHED)
        self.create_service(SurveyMove, "/amr/survey/move", self._srv_move)
        self.create_service(Trigger, "/amr/survey/move_stop", self._srv_stop)
        self.create_timer(1.0 / float(p("rate_hz").value), self._tick)
        self.create_timer(0.2, self._publish_state)
        self._publish_state()

    def _init_state(self) -> None:
        self._lock = threading.RLock()
        self.state = SurveyMoveState.IDLE
        self.move: sm.Move | None = None
        self.ctl: sm.MoveController | None = None
        self.move_id = ""
        self.session = ""
        self.seq = 0
        self.reason = ""
        self.check: tuple[bool, float, float] | None = None  # ok, metres, degrees
        self._settle_until = 0.0
        self._map_odom0: tuple[float, float, float] | None = None
        self._lease: ControlLease | None = None
        self._lease_t = -1e9
        self._panel: PanelState | None = None
        self._panel_t = -1e9
        self._drives_ok = False
        self._drives_t = -1e9
        self._odom: tuple[float, float, float] | None = None
        self._odom_t = -1e9
        self._mapping = MappingState.IDLE

    # ---- inputs ----

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        self._odom, self._odom_t = (m.pose.pose.position.x, m.pose.pose.position.y, _yaw(q)), self._now()

    def _on_lease(self, m: ControlLease) -> None:
        self._lease, self._lease_t = m, self._now()

    def _on_panel(self, m: PanelState) -> None:
        self._panel, self._panel_t = m, self._now()
        with self._lock:
            if self.state == SurveyMoveState.MOVING and self._pendant(m):
                self._abort("pendant button pressed")

    def _on_drives(self, m: DriveStatus) -> None:
        self._drives_ok, self._drives_t = bool(m.operational), self._now()

    def _on_manual(self, m: ManualCommand) -> None:
        with self._lock:
            if self.state == SurveyMoveState.MOVING and m.session != self.session:
                self._abort("jog pad or Stop used")

    def _on_mapping(self, m: MappingState) -> None:
        self._mapping = m.state

    @staticmethod
    def _pendant(m: PanelState) -> bool:
        return bool(m.pendant_fwd or m.pendant_rvs or m.pendant_left or m.pendant_right)

    def _map_odom(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform("map", "odom", Time())
        except Exception:  # noqa: BLE001 - no transform is a normal state early in a survey
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, _yaw(tr.rotation)

    # ---- authority ----

    def _blocker(self, now: float) -> str:
        """Why a move may not start or continue now; "" if it may."""
        if self._mapping != MappingState.MAPPING:
            return "not surveying"
        pn = self._panel
        if pn is None or now - self._panel_t > self.input_age or not pn.valid:
            return "panel stale or invalid"
        if pn.mode_auto:
            return "selector is not MANUAL"
        if self._pendant(pn):
            return "pendant button pressed"
        ls = self._lease
        if ls is None or now - self._lease_t > self.input_age or not (ls.allowed & LEASE_MANUAL):
            return "no manual lease from the supervisor"
        if now - self._drives_t > self.input_age or not self._drives_ok:
            return "drives have no torque (E-stop?)"
        if self._odom is None or now - self._odom_t > self.input_age:
            return "no odometry"
        return ""

    # ---- services ----

    def _srv_move(self, req: SurveyMove.Request, res: SurveyMove.Response):
        with self._lock:
            if self.state in (SurveyMoveState.MOVING, SurveyMoveState.SETTLING):
                res.accepted, res.message = False, "a move is already running"
                return res
            try:
                move = sm.Move.parse(req.kind, float(req.value))
            except sm.MoveError as e:
                res.accepted, res.message = False, str(e)
                return res
            now = self._now()
            why = self._blocker(now)
            if why:
                res.accepted, res.message = False, f"refused: {why}"
                return res
            self.move, self.ctl = move, sm.MoveController(move, self.profile)
            self.ctl.start(self._odom, now)
            self.move_id, self.session, self.seq = uuid.uuid4().hex[:8], "survey-" + uuid.uuid4().hex, 0
            self.reason, self.check = "", None
            self._map_odom0 = self._map_odom()
            self.state = SurveyMoveState.MOVING
            self.get_logger().info(f"move {self.move_id}: {move.label()}")
            self._publish_state()
            res.accepted, res.message, res.move_id = True, move.label(), self.move_id
            return res

    def _srv_stop(self, _req, res: Trigger.Response):
        with self._lock:
            if self.state != SurveyMoveState.MOVING:
                res.success, res.message = False, "no move running"
            else:
                self._abort("stopped by operator")
                res.success, res.message = True, "stopped"
            return res

    # ---- motion ----

    def _send(self, v: float, w: float, valid_for: float) -> None:
        ls = self._lease
        m = ManualCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.instance = ls.instance if ls is not None else ""
        m.generation = int(ls.generation) if ls is not None else 0
        m.session = self.session
        self.seq += 1
        m.seq = self.seq
        m.valid_for_s, m.v, m.w = valid_for, float(v), float(w)
        self._cmd_pub.publish(m)

    def _abort(self, why: str) -> None:
        self._send(0.0, 0.0, 0.0)  # revokes this session at the mux: zero at once
        self.state, self.reason = SurveyMoveState.ABORTED, why
        self.get_logger().warn(f"move {self.move_id} aborted: {why}")
        self._publish_state()

    def _tick(self) -> None:
        try:
            self._tick_inner()
        except Exception as e:  # noqa: BLE001 - a bug must stop the move, never kill the node
            with self._lock:
                if self.state in (SurveyMoveState.MOVING, SurveyMoveState.SETTLING):
                    self._send(0.0, 0.0, 0.0)
                    self.state, self.reason = SurveyMoveState.ABORTED, f"internal error: {e!r}"
                    self._publish_state()
            self.get_logger().error(f"survey move tick failed: {e!r}")

    def _tick_inner(self) -> None:
        with self._lock:
            now = self._now()
            if self.state == SurveyMoveState.MOVING:
                why = self._blocker(now)
                if why:
                    self._abort(why)
                    return
                v, w, done, reason = self.ctl.step(self._odom, now)
                if not done:
                    self._send(v, w, self.cmd_valid)
                    return
                self._send(0.0, 0.0, 0.0)
                if reason:
                    self._abort(reason)
                    return
                self.state, self._settle_until = SurveyMoveState.SETTLING, now + self.settle_s
                self._publish_state()
            elif self.state == SurveyMoveState.SETTLING and now >= self._settle_until:
                self._finish()

    def _finish(self) -> None:
        after = self._map_odom()
        if self._map_odom0 is None or after is None or self._odom is None:
            self.check, self.reason = None, "no map->odom transform: SLAM check skipped"
        else:
            m, rad = sm.slam_correction(self._map_odom0, after, self._odom)
            ok, self.reason = sm.check_verdict(self.move, m, rad, self.max_corr_deg)
            self.check = (ok, m, math.degrees(rad))
        self.state = SurveyMoveState.DONE
        text = (
            f"move {self.move_id} done ({self.move.label()}, achieved {self._achieved_text()}): {self.reason}"
        )
        # two call sites on purpose: rclpy refuses a severity change at one call site (2026-09-19,
        # that ValueError killed the node right after a move and left the page "checking the map")
        if self.check is None or self.check[0]:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        self._publish_state()

    def _achieved_text(self) -> str:
        a = self.ctl.achieved() if self.ctl else 0.0
        return f"{a:.3f} m" if self.move and self.move.kind == sm.STRAIGHT else f"{math.degrees(a):.1f} deg"

    def _publish_state(self) -> None:
        m = SurveyMoveState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state, m.move_id, m.reason = self.state, self.move_id, self.reason
        if self.move is not None:
            m.label, m.kind, m.target = self.move.label(), self.move.kind, self.move.target
            m.progress = self.ctl.achieved() if self.ctl else 0.0
        if self.check is not None:
            m.check_done, (m.check_ok, m.correction_m, m.correction_deg) = True, self.check
        self._state_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SurveyMoveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            with node._lock:
                if node.state == SurveyMoveState.MOVING:
                    node._send(0.0, 0.0, 0.0)
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
