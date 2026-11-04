"""gls_node: the SICK GLS621 floor-label camera, read-only (gls-wait-coding-plan §B.4).

    GLS621 at gls.host:gls.port, CoLa A over TCP (agv_core/drivers/gls_link.GlsLink thread)
        -> /gls/reading (GlsReading): one message per camera image (~14.6 Hz with a label
           in view), and valid=false at 5 Hz once nothing arrived for gls.stale_s
        -> /amr/events: GLS_DOWN when the camera stops answering, GLS when it answers
           again, GLS_CONFIG_MISMATCH when its settings are not the ones this relies on

The sole owner of the camera's socket ("gls" owner lock, taken before any I/O), like the
panel and drive nodes; the bench probe refuses while this runs. It never writes a device
setting, and it decides nothing: what a reading means for the pose is label_fix's (P4).

NOT IN THE SAFETY PATH. A camera that is unplugged, misconfigured or wrong never stops,
slows or starts anything: the node is an optional launch member and its silence is only
a GLS_DOWN warning.

Until the bench tests have measured gls.xy_scale_m_per_count the readings carry raw counts
only (calibrated=false, x_m/y_m/yaw_rad 0): nothing downstream may treat them as a pose.
"""

from __future__ import annotations

import math
import time

import rclpy
from agv_core import config, ownerlock
from agv_core.drivers import gls
from agv_core.drivers.gls_link import REQUIRED, GlsLink
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from amr_interfaces.msg import Event, GlsReading

FRAME_ID = "gls_frame"
INVALID_HZ = 5.0  # the "nothing in view" heartbeat


def expected_settings() -> dict[str, str]:
    """What the camera must read back before a frame is believed."""
    return {"OFtplstr1": config.GLS_EXPECT_TEMPLATE, **REQUIRED}


def outlier_counts(outlier_m: float, scale: float) -> float | None:
    """The aggregation's outlier tolerance in device counts, or None (off) while the scale is
    unmeasured or the tolerance is 0."""
    return outlier_m / scale if scale > 0.0 and outlier_m > 0.0 else None


def fill(m: GlsReading, frame: gls.Frame | None, snap: dict, scale: float, signs=(1, 1, 1),
         angle_offset_rad: float = 0.0, swap_xy: bool = False) -> GlsReading:
    """GlsReading fields from one frame (None = nothing in view) and the link's snapshot.
    Pure apart from the message object, so it is tested without a graph."""
    m.comms_ok = bool(snap.get("comms_ok"))
    m.config_ok = snap.get("config_ok") is True
    m.calibrated = scale > 0.0
    m.fps = float(snap.get("fps") or 0.0)
    m.detail = str(snap.get("detail") or "")
    if frame is None:
        m.valid, m.label_id = False, ""
        return m
    m.valid = bool(frame.valid and m.config_ok)
    m.label_id = frame.label_id if m.valid else ""
    m.n_codes, m.n_outliers = frame.n_codes, frame.n_outliers
    m.x_counts, m.y_counts, m.angle_deg = float(frame.x), float(frame.y), float(frame.angle_deg)
    m.spread_counts = float(frame.spread_xy)
    m.device_ts_ms = int(frame.ts_ms)
    if m.calibrated:
        m.x_m, m.y_m, m.yaw_rad = gls.to_camera_pose(frame, scale, *signs, angle_offset_rad, swap_xy)
        m.spread_xy_m = frame.spread_xy * scale
        m.spread_yaw_rad = math.radians(frame.spread_deg)
    if not frame.valid:
        m.detail = frame.reason
    return m


class GlsNode(Node):
    def __init__(self) -> None:
        super().__init__("gls_node")
        self.declare_parameter("rate_hz", 50.0)
        self._pub = self.create_publisher(GlsReading, "/gls/reading", 10)
        self._pub_event = self.create_publisher(Event, "/amr/events", 50)
        self._event_seq = 0
        self.scale = float(config.GLS_XY_SCALE_M_PER_COUNT)
        self.signs = (config.GLS_X_SIGN, config.GLS_Y_SIGN, config.GLS_ANGLE_SIGN)
        self.link = None
        if not config.GLS_ENABLED:
            self.get_logger().warn("gls.enabled is false in the profile: no camera, nothing published")
            return
        self._gls_lock = ownerlock.acquire("gls", "gls_node")  # before any I/O (plan §9.2)
        self.link = GlsLink(
            config.GLS_HOST,
            config.GLS_PORT,
            expected_settings(),
            connect_timeout_s=config.GLS_CONNECT_TIMEOUT_S,
            keepalive_s=config.GLS_KEEPALIVE_S,
            verify_timeout_s=config.GLS_VERIFY_TIMEOUT_S,
            frame_gap_s=config.GLS_FRAME_GAP_S,
            min_codes=config.GLS_MIN_CODES,
            outlier_counts=outlier_counts(config.GLS_OUTLIER_M, self.scale),
        )
        self.link.start()
        self._last_frame_t: float | None = None
        self._last_invalid_t = 0.0
        self._comms: bool | None = None
        self._config: bool | None = None
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info(
            f"GLS621 at {config.GLS_HOST}:{config.GLS_PORT}, read-only; "
            + (f"scale {self.scale:g} m/count" if self.scale > 0 else "NOT CALIBRATED: raw counts only")
        )

    def _tick(self) -> None:
        now = time.monotonic()
        snap = self.link.snapshot()
        self._report(snap)
        frames = self.link.pop_frames()
        for _t, f in frames:
            self._publish(f, snap)
        if frames:
            self._last_frame_t = now
        stale = self._last_frame_t is None or now - self._last_frame_t > config.GLS_STALE_S
        if stale and now - self._last_invalid_t >= 1.0 / INVALID_HZ:
            self._last_invalid_t = now
            self._publish(None, snap)

    def _publish(self, frame: gls.Frame | None, snap: dict) -> None:
        m = GlsReading()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = FRAME_ID
        self._pub.publish(
            fill(m, frame, snap, self.scale, self.signs, config.GLS_ANGLE_OFFSET_RAD, config.GLS_SWAP_XY)
        )

    def _report(self, snap: dict) -> None:
        """One event per health transition, never per tick."""
        comms = bool(snap["comms_ok"])
        # a boot that has not answered yet is neither a loss nor a recovery: _comms stays None
        if comms != self._comms and (comms or self._comms is not None):
            if comms:
                self.get_logger().info(f"GLS answering: {snap['detail']}")
                if self._comms is False:  # only after a GLS_DOWN
                    self._event(Event.INFO, "GLS", f"floor-label camera answering again: {snap['detail']}")
            else:
                self.get_logger().warn(f"GLS not answering: {snap['detail']}")
                self._event(Event.WARN, "GLS_DOWN", f"floor-label camera not answering: {snap['detail']}")
            self._comms = comms
        # None = re-verifying after a reconnect: the last verdict stands, so a camera that
        # never answers (verify timeout, reconnect, timeout...) raises the mismatch once
        cfg = snap["config_ok"]
        if cfg is not None and cfg != self._config:
            if cfg is False:
                self.get_logger().error(f"GLS settings: {snap['detail']}")
                self._event(Event.ERROR, "GLS_CONFIG_MISMATCH", snap["detail"])
            else:
                self.get_logger().info("GLS settings verified")
            self._config = cfg

    def _event(self, level: int, code: str, text: str) -> None:
        m = Event()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source, m.level, m.code, m.text = "gls_node", level, code, text
        self._event_seq += 1
        m.seq = self._event_seq
        self._pub_event.publish(m)

    def close(self) -> None:
        if self.link is not None:
            self.link.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
