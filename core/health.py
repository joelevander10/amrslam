"""Hardware health: is each device still talking to us, and what does it cost?

*** This is NOT the browser watchdog. *** config.MANUAL_WATCHDOG_S covers an
absent OPERATOR - a closed tab, a dropped Wi-Fi link, a released button. What
lives here covers an absent DEVICE. They fail for different reasons, they
stop the vehicle for different reasons, and merging them would mean a browser
refresh could paper over a dead driver. Keep them apart.

WHY A PROTOCOL AND NOT THREE MORE IF-STATEMENTS
-----------------------------------------------
Before this module the vehicle had three unrelated liveness mechanisms - the
sensor's frame timestamp, a per-node fault dict, and the RFID link's own socket
health - and one hole where a fourth should have been: _poll_telemetry() keeps
the last statusword forever when an SDO read returns None, so a driver that
stopped answering looked healthy indefinitely. Adding a lidar would have meant a
fourth mechanism and a fourth chance to forget the timeout.

A source declares its own criticality and the monitor owns the arithmetic, so
supervising something new is one row in a table.

TWO TIERS, BECAUSE TWO FAULTS ARE NOT THE SAME FAULT
----------------------------------------------------
  critical=True   the drivers. Losing them means we cannot command or even
                  observe the wheels, so every mode stops.
  critical=False  the MLS and the RFID reader. Losing them means we no longer
                  know where the tape or the stations are, so AUTO stops - but
                  manual jogging never needed them and must not be held hostage.
                  This is the policy _do_arm() already implements for a silent
                  sensor; here it becomes a declared property rather than an
                  if-statement.

PURE COMPUTATION
----------------
No CAN, no Flask, no file I/O, standard library only - the same stance events.py
takes, and for the same reason: the bus thread and Flask both touch this, and
neither may drag the other in. evaluate() is arithmetic over timestamps, safe to
call every tick and safe to call while holding a lock (though canworker calls it
outside one anyway).

Edge detection lives HERE. evaluate() runs at 50 Hz, and events.py records that
one chatty call site empties the 200-entry ring in about four seconds, so the
monitor reports transitions rather than states and the caller emits only on a
transition.
"""
import time


class HealthSource:
    """A device whose liveness the bus thread observes directly.

    mark_rx() is called on every successful read - a decoded TPDO1 frame, an SDO
    reply that came back. Written and read on the bus thread only, so it needs
    no lock of its own; a source fed from another thread wants PullSource.
    """

    def __init__(self, name, critical=False, detail=None):
        self.name = name
        self.critical = critical
        self.detail = detail or name
        self._last_rx = 0.0
        self._ok = False              # False until the first successful read

    def mark_rx(self, now=None):
        self._last_rx = time.monotonic() if now is None else now
        self._ok = True

    def reset(self):
        """Forget history - used when the bus is reopened, so a stale timestamp
        from a previous session cannot make a device look alive."""
        self._last_rx = 0.0
        self._ok = False

    def health(self, now, timeout_s):
        age = (now - self._last_rx) if self._ok else None
        return {"name": self.name, "critical": self.critical,
                "detail": self.detail,
                "ok": self._ok and age is not None and age <= timeout_s,
                "seen": self._ok, "age_s": age, "timeout_s": timeout_s}


class PullSource:
    """A device that already tracks its own health on its own thread.

    Wraps a snapshot callable rather than having that thread write in here:
    RfidLink.snapshot() is already correct, already locked, and already the
    thing the UI reads. Copying its verdict on demand avoids a second lock and a
    second definition of what "healthy" means for that device.

    snapshot_fn must return a dict; `ok_key` names the field holding the
    verdict. A None verdict means "not in use" - not a fault, and not a source
    the monitor should judge (rfid.enabled false is exactly this case).
    """

    def __init__(self, name, snapshot_fn, critical=False, ok_key="comms_ok",
                 age_key="rx_age_s"):
        self.name = name
        self.critical = critical
        self.detail = name
        self._snapshot = snapshot_fn
        self._ok_key = ok_key
        self._age_key = age_key

    def health(self, now, timeout_s):
        try:
            snap = self._snapshot() or {}
        except Exception:                       # noqa: BLE001 - never fatal
            snap = {}
        verdict = snap.get(self._ok_key)
        return {"name": self.name, "critical": self.critical,
                "detail": snap.get("detail") or self.name,
                # None verdict = not in use. Reported as healthy so it can never
                # block a mode, and flagged so the UI can grey it out.
                "ok": True if verdict is None else bool(verdict),
                "seen": verdict is not None,
                "age_s": snap.get(self._age_key),
                "timeout_s": timeout_s, "in_use": verdict is not None}


class HealthMonitor:
    """The watchdog table. One row per supervised device.

    A source that has never been seen is NOT a fault: the drivers are silent
    until the bus is up and the sensor until it is NMT-started, and reporting
    that as a failure at boot would cry wolf before anything is even connected.
    Only a source that answered once and then stopped counts as lost.
    """

    def __init__(self, table=()):
        self._table = list(table)
        self._prev = {}               # name -> ok, for edge detection
        self._prev_system = False
        self._prev_sensor = False

    def add(self, source, timeout_s):
        self._table.append((source, timeout_s))
        return source

    def min_timeout(self):
        """The tightest deadline in the table.

        Used by the caller to decide whether a stall it just suffered was long
        enough to invalidate every source's evidence - see canworker._run().
        """
        return min((t for _s, t in self._table), default=0.0)

    def reset(self):
        for source, _ in self._table:
            if hasattr(source, "reset"):
                source.reset()
        self._prev.clear()
        self._prev_system = self._prev_sensor = False

    def evaluate(self, now=None):
        """One pass over the table. Pure arithmetic; safe at tick rate.

        Returns the two tier verdicts, every source's health, and the list of
        transitions since the previous call so the caller can emit once per
        edge instead of once per tick.
        """
        now = time.monotonic() if now is None else now
        sources = {}
        changed = []
        system_lost, sensor_lost = [], []

        for source, timeout_s in self._table:
            h = source.health(now, timeout_s)
            sources[h["name"]] = h

            # Never seen is not lost - see the class docstring.
            lost = h["seen"] and not h["ok"]
            if lost:
                (system_lost if h["critical"] else sensor_lost).append(h["detail"])

            was = self._prev.get(h["name"])
            if was is not None and was != h["ok"]:
                changed.append((h["name"], h["detail"], h["ok"]))
            self._prev[h["name"]] = h["ok"]

        system_error = bool(system_lost)
        sensor_error = bool(sensor_lost)
        result = {
            "system_error": system_error,
            "system_detail": ", ".join(system_lost),
            "sensor_error": sensor_error,
            "sensor_detail": ", ".join(sensor_lost),
            "sources": sources,
            "changed": changed,
            # Tier transitions, so the caller can act on the EDGE rather than
            # re-stopping an already-stopped vehicle 50 times a second.
            "system_edge": (None if system_error == self._prev_system
                            else system_error),
            "sensor_edge": (None if sensor_error == self._prev_sensor
                            else sensor_error),
        }
        self._prev_system, self._prev_sensor = system_error, sensor_error
        return result
