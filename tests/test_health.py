"""Hardware liveness: the two-tier watchdog table."""
import math
import os
import pathlib
import struct
import sys
import threading

from helpers import FAIL, ROOT, check

import config
import kinematics
import motion

def test_health():
    """Hardware liveness: the two-tier watchdog and its edge reporting.

    The hole this closes: _poll_telemetry() keeps the last statusword when an
    SDO read returns None, so before health.py a driver that stopped answering
    looked alive for as long as the process ran.
    """
    import health
    print("\nhardware health")

    # -- a source is not a fault until it has been seen once -----------------
    src = health.HealthSource("lidar")
    mon = health.HealthMonitor([(src, 1.0)])
    r = mon.evaluate(now=100.0)
    check("a never-seen source is not reported lost",
          not r["sensor_error"] and not r["system_error"])
    check("a never-seen source reports seen=False",
          r["sources"]["lidar"]["seen"] is False)

    src.mark_rx(now=100.0)
    r = mon.evaluate(now=100.5)
    check("a fresh source is healthy", r["sources"]["lidar"]["ok"] is True)
    r = mon.evaluate(now=101.5)
    check("a source past its timeout is lost", r["sensor_error"] is True,
          r["sensor_detail"])
    src.mark_rx(now=101.6)
    r = mon.evaluate(now=101.7)
    check("a source recovers on the next read", r["sensor_error"] is False)

    # -- the two tiers route differently -------------------------------------
    drv = health.HealthSource("driver:1", critical=True, detail="node 1 (left)")
    aux = health.HealthSource("lidar")
    mon = health.HealthMonitor([(drv, 0.5), (aux, 0.5)])
    drv.mark_rx(now=0.0)
    aux.mark_rx(now=0.0)
    mon.evaluate(now=0.1)
    aux.mark_rx(now=1.0)                 # aux alive, driver silent
    r = mon.evaluate(now=1.0)
    check("a silent driver raises the CRITICAL tier",
          r["system_error"] is True and r["sensor_error"] is False,
          r["system_detail"])
    drv.mark_rx(now=2.0)                 # driver back, aux now silent
    r = mon.evaluate(now=2.0)
    check("a silent non-critical source raises the SENSOR tier only",
          r["sensor_error"] is True and r["system_error"] is False,
          r["sensor_detail"])

    # -- edges fire ONCE, not once per tick ----------------------------------
    # This is the events.py trap: at 50 Hz a per-tick emit empties the whole
    # 200-entry ring in about four seconds.
    s = health.HealthSource("x", critical=True)
    mon = health.HealthMonitor([(s, 0.5)])
    s.mark_rx(now=0.0)
    mon.evaluate(now=0.0)
    edges = tier_edges = 0
    for i in range(100):                 # 2 s of ticks with the source dead
        r = mon.evaluate(now=1.0 + i * 0.02)
        edges += len(r["changed"])
        tier_edges += r["system_edge"] is not None
    check("a source transition is reported once, not per tick", edges == 1,
          f"{edges} edge(s) over 100 ticks")
    check("a tier transition is reported once, not per tick", tier_edges == 1,
          f"{tier_edges} edge(s) over 100 ticks")

    # -- PullSource wraps an existing snapshot -------------------------------
    snap = {"comms_ok": True, "rx_age_s": 0.2, "detail": "reader"}
    pull = health.PullSource("rfid", lambda: snap)
    mon = health.HealthMonitor([(pull, 5.0)])
    check("a pull source reports its snapshot verdict",
          mon.evaluate(now=0.0)["sources"]["rfid"]["ok"] is True)
    snap["comms_ok"] = False
    check("a pull source fault reaches the auto-only tier",
          mon.evaluate(now=0.0)["sensor_error"] is True)
    # rfid.enabled false -> comms_ok is None -> not in use, never a fault.
    snap["comms_ok"] = None
    r = mon.evaluate(now=0.0)
    check("a disabled pull source never blocks a mode",
          r["sensor_error"] is False and r["sources"]["rfid"]["in_use"] is False)
    pull_raises = health.PullSource("boom", lambda: 1 / 0)
    check("a raising snapshot is absorbed, never propagated",
          health.HealthMonitor([(pull_raises, 1.0)]).evaluate(now=0.0)
          ["sensor_error"] is False)

    # -- reset clears history across a bus reopen ----------------------------
    s = health.HealthSource("y")
    s.mark_rx(now=0.0)
    mon = health.HealthMonitor([(s, 0.5)])
    mon.reset()
    check("reset() forgets a stale timestamp",
          mon.evaluate(now=999.0)["sensor_error"] is False)

    # -- health.py stays dependency-free -------------------------------------
    head = (ROOT / "core" / "health.py").read_text(encoding="utf-8")
    head = head[head.index('"""', head.index('"""') + 3):]
    imports = {ln.split()[1].split(".")[0] for ln in head.splitlines()
               if ln.startswith(("import ", "from "))}
    check("health.py imports only the standard library",
          imports <= {"time"}, str(sorted(imports)))

    # -- wired into the vehicle, on the paths that prove liveness ------------
    cw = (ROOT / "canworker.py").read_text(encoding="utf-8")
    # The MLS used to be a supervised source, fed by every decoded TPDO1 frame.
    # It went with tape following; what proves a drive alive now is its producer
    # heartbeat and its telemetry replies, which is the arrangement health.py
    # was built around in the first place.
    check("a producer heartbeat marks the driver alive",
          "src.mark_rx()" in cw[cw.index("def _on_heartbeat"):])
    check("each driver marks health on a telemetry answer",
          "self._src_node[nid].mark_rx()" in cw)
    check("a completed SDO transfer also counts as liveness",
          "src.mark_rx()" in cw[cw.index("def _mark_alive"):cw.index("def _read")])
    check("health is evaluated outside the lock",
          "hw = self._hw.evaluate(now)" in cw)
    check("a critical fault refuses an arm", "system_error" in
          cw[cw.index("def _do_arm"):cw.index("def _do_disarm")])




def test_blocking_action_does_not_fake_a_fault():
    """The regression: arming used to report both drives silent.

    Liveness was marked ONLY from the 5 Hz telemetry poll and from a producer
    heartbeat that was never actually enabled. A real arm blocks the single bus
    thread for 700-900 ms against a 0.6 s driver timeout, so the evaluate()
    immediately after the drain saw both drives stale and emitted
    "driver silent - node 1 (left), node 2 (right), stopping" - while manual
    jogging kept working, because they were never silent at all.

    Three things now prevent it, and each is checked here.
    """
    import canworker
    import health
    print("\nfalse 'driver silent' on a blocking action")

    src = (ROOT / "canworker.py").read_text(encoding="utf-8")

    # 1. The producer heartbeat is actually turned on. It was dead code: defined
    #    and never called, which left 1017h at its factory default of 0 = OFF
    #    and can.heartbeat_ms inert.
    body = src[src.index("def _run(self)"):]
    body = body[:body.find("\n    def ", 1)]
    check("_enable_heartbeat() is called from the bus-open path",
          "self._enable_heartbeat()" in body)
    check("...and it is not called from _do_arm, so liveness does not depend "
          "on having armed",
          "_enable_heartbeat" not in src[src.index("def _do_arm"):])

    # 2. Every successful driver SDO transfer counts as evidence.
    for fn in ("_read", "_read_i32", "_write"):
        seg = src[src.index(f"def {fn}(self"):]
        seg = seg[:seg.find("\n    def ", 1)]
        check(f"{fn}() marks the node alive on success",
              "self._mark_alive(" in seg)
    check("_mark_alive uses .get(), so the MLS is not mistaken for a drive",
          "self._src_node.get(node)" in src)

    # 3. A window we could not observe is not judged.
    check("a long queue drain resets the monitor instead of failing it",
          "self._hw.min_timeout()" in src and "self._hw.reset()" in src)

    # --- and the behaviour those three produce ---------------------------
    mon = health.HealthMonitor()
    drv1 = health.HealthSource("driver:1", critical=True, detail="node 1 (left)")
    drv2 = health.HealthSource("driver:2", critical=True, detail="node 2 (right)")
    mon.add(drv1, 0.6)
    mon.add(drv2, 0.6)
    check("min_timeout is the tightest deadline in the table",
          mon.min_timeout() == 0.6)

    t = 100.0
    drv1.mark_rx(now=t); drv2.mark_rx(now=t)
    check("both drives healthy to start",
          mon.evaluate(now=t)["system_error"] is False)

    # An 850 ms arm during which the drives answered every SDO: with
    # _mark_alive the gap between marks is one 50 ms sleep, never the whole arm.
    for step in range(1, 18):
        drv1.mark_rx(now=t + step * 0.05); drv2.mark_rx(now=t + step * 0.05)
    r = mon.evaluate(now=t + 0.85)
    check("an arm that talks to the drives throughout raises no fault",
          r["system_error"] is False and r["system_edge"] is None,
          f"{r['system_error']} {r['system_edge']}")

    # The other half: a stall with NO marks at all must still not fault, because
    # the loop resets the monitor first. Without the reset this is the bug.
    mon.reset()
    r = mon.evaluate(now=t + 99.0)
    check("after reset an unobserved window reads as not-seen, not as lost",
          r["system_error"] is False, str(r["system_error"]))

    # ...and a genuinely dead drive must still be caught. The fix removes false
    # positives; it must not remove true ones.
    drv1.mark_rx(now=t + 99.0); drv2.mark_rx(now=t + 99.0)
    mon.evaluate(now=t + 99.0)
    r = mon.evaluate(now=t + 100.0)
    check("a drive that really stops answering still faults",
          r["system_error"] is True and r["system_edge"] is True,
          f"{r['system_error']} {r['system_edge']}")


TESTS = [
    test_health,
    test_blocking_action_does_not_fake_a_fault,
]
