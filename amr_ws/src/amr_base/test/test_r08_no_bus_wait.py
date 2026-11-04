"""R08 (2026-10-27): no can0 at boot keeps drive_node alive and saying why.

The CANable was out at boot; amr.service was bound to can0 and never started, so
the operator had no page at all. With the bind gone, drive_node must WAIT for the
bus - not exit and take the base layer down - and report "no bus" on
/drives/status and as an event, then carry on once can0 appears.
"""

import queue
import threading
from types import SimpleNamespace

from amr_base import drive_node as dn


def _node(out, events):
    from rclpy.clock import Clock

    stop = threading.Event()
    node = SimpleNamespace(
        get_clock=Clock,
        get_logger=lambda: SimpleNamespace(error=lambda s: None),
        _safe_publish=lambda pub, m: out.append(m),
        _pub_status=None,
        _status_snapshot={"state": "starting", "reason": "", "mode": "off"},
        _edge=lambda slot, what: events.append((slot, what)),
        _requests=queue.Queue(),
        _stop=stop,
    )
    node._publish_no_bus = lambda reason: dn.DriveNode._publish_no_bus(node, reason)
    node._refuse_requests = lambda reason: dn.DriveNode._refuse_requests(node, reason)
    return node


def test_waits_for_can0_reporting_no_bus_then_opens(monkeypatch):
    out, events, tries = [], [], []

    def open_bus(*_a):
        tries.append(1)
        if len(tries) < 3:
            raise RuntimeError("no CANable found on USB, and can0 is not up")
        return "BUS", "socketcan:can0"

    monkeypatch.setattr(dn, "open_bus", open_bus)
    monkeypatch.setattr(dn, "BUS_RETRY_S", 0.0)
    monkeypatch.setattr(dn.rclpy, "ok", lambda: True)  # no rclpy.init() in a unit test
    node = _node(out, events)
    assert dn.DriveNode._wait_for_bus(node) == ("BUS", "socketcan:can0")
    assert len(tries) == 3, "retried until can0 opened, did not give up"
    assert len(out) == 2 and all(m.link_state == dn.NO_BUS and not m.operational for m in out)
    assert "can0 is not up" in out[0].power_reason, "the reason reaches the operator"
    assert events[0][0] == "bus" and events[0][1][0] == "CAN_DOWN"
    assert events[-1] == ("bus", None), "the event clears once the bus is up"


def test_requests_are_refused_at_once_while_there_is_no_bus(monkeypatch):
    out, events = [], []
    node = _node(out, events)

    def open_bus(*_a):
        node._stop.set()  # one failed try, then shutdown
        raise RuntimeError("can0 is not up")

    monkeypatch.setattr(dn, "open_bus", open_bus)
    monkeypatch.setattr(dn.rclpy, "ok", lambda: True)
    done, box = threading.Event(), {}
    node._requests.put(("arm", done, box))
    assert dn.DriveNode._wait_for_bus(node) is None, "a shutdown while waiting returns None"
    assert done.is_set() and box["ok"] is False and "no CAN bus" in box["msg"]
