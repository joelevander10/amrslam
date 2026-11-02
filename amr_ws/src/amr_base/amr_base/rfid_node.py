"""rfid_node: the RFID reader (Chafon CF821, TCP) -> /amr/rfid encounters.

Base layer, every mode, like the MLS track: the tape product's mission engine
consumes the encounters, and a trackless vehicle can still see tags. The reader
is TCP on enp2s0, not can0; it takes the "rfid" owner lock so the read-only
survey tool (agv_core.drivers.rfid_survey) cannot run beside it.

Wire contract is StationDetection.msg: one message per DISTINCT tag pass
(agv_core.drivers.rfid's encounter stream, numbered), plus a 2 Hz heartbeat with
the link status, on the same RELIABLE topic so a consumer sees them in order.
Both carry the driver's latest read quality (RSSI, channel) and the reader
configuration it read back on connect, for display.
Nothing here decides anything; the encounter rules (tag_clear_s, re-baseline on
reconnect) are the driver's.

Each finished tag pass is logged (rosout INFO, "RFID pass ...": reads, duration,
RSSI) so a run bag shows the read margin; tools/bag_report.py tabulates it.
"""

from __future__ import annotations

import rclpy
from agv_core import config, ownerlock
from agv_core.drivers import rfid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from amr_interfaces.msg import StationDetection

ENCOUNTERS = QoSProfile(
    depth=50, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
)


class RfidNode(Node):
    def __init__(self) -> None:
        super().__init__("rfid_node")
        self.declare_parameter("poll_hz", 50.0)
        self.declare_parameter("heartbeat_hz", 2.0)
        self.link = rfid.RfidLink()
        self._pub = self.create_publisher(StationDetection, "/amr/rfid", ENCOUNTERS)
        self._sent = 0  # highest encounter_seq published
        self._pass_logged = 0  # highest pass seq logged
        self._generation = None
        self._rfid_lock = ownerlock.acquire("rfid", "rfid_node")  # before opening the reader socket
        self.link.start()
        self.create_timer(1.0 / float(self.get_parameter("poll_hz").value), self._poll)
        self.create_timer(1.0 / float(self.get_parameter("heartbeat_hz").value), self._heartbeat)
        self.get_logger().info(
            f"RFID {config.RFID_IP}:{config.RFID_PORT} enabled={config.RFID_ENABLED} "
            f"tag_clear_s={config.RFID_TAG_CLEAR_S}"
        )

    def _msg(self, snap: dict, heartbeat: bool, seq: int, tag: str = "") -> StationDetection:
        m = StationDetection()
        m.header.stamp = self.get_clock().now().to_msg()
        m.rfid_tag = tag
        m.heartbeat = heartbeat
        m.encounter_seq = int(seq)
        m.generation = int(snap.get("generation", 0))
        m.comms_ok = bool(snap.get("comms_ok", False))
        rx, tag_age = snap.get("rx_age_s"), snap.get("tag_age_s")
        m.rx_age_s = -1.0 if rx is None else float(rx)
        m.tag_age_s = -1.0 if tag_age is None else float(tag_age)
        rssi, ch, freq = snap.get("rssi_dbm"), snap.get("channel"), snap.get("freq_mhz")
        m.rssi_dbm = 0.0 if rssi is None else float(rssi)
        m.channel = -1 if ch is None else int(ch)
        m.freq_mhz = 0.0 if freq is None else float(freq)
        m.reader = rfid.params_summary(snap.get("reader"))
        m.config_mismatch = [str(x) for x in snap.get("config_mismatch", ())]
        return m

    def _poll(self) -> None:
        snap = self.link.snapshot(encounters=True)
        if snap.get("generation", 0) != self._generation:
            # A reconnect clears the driver's buffer; numbering continues, so only
            # the newer entries are new.
            self._generation = snap.get("generation", 0)
        for seq, tag in snap.get("encounters", ()):
            if seq > self._sent:
                self._pub.publish(self._msg(snap, False, seq, tag))
                self._sent = seq
        for seq, p in snap.get("passes", ()):
            if seq > self._pass_logged:
                self.get_logger().info(rfid.pass_summary(p))
                self._pass_logged = seq

    def _heartbeat(self) -> None:
        snap = self.link.snapshot()
        self._pub.publish(self._msg(snap, True, snap.get("encounter_seq", 0)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RfidNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.link.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
