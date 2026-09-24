"""Internet reachability for the header: does THIS robot reach the internet?

A comfort indicator like the Wi-Fi readout, never an authority: nothing on the robot needs
the internet (DDS is loopback-only, the lidar and DIO are wired, the web UI is served
locally). The probe is a plain TCP connect to well-known anycast addresses on 443 - no DNS,
so a dead resolver cannot stall it and a working one cannot fake it - and the result is
cached for `ttl_s`, so however many pages poll, the robot probes at most once per ttl_s
(the header polls at 0.25 Hz). One probe at a time; a caller that arrives while one is
running gets the last result instead of waiting.
"""

from __future__ import annotations

import socket
import threading
import time

TARGETS = (("1.1.1.1", 443), ("8.8.8.8", 443))


class InternetProbe:
    def __init__(
        self,
        targets=TARGETS,
        timeout_s: float = 1.5,
        ttl_s: float = 4.0,
        connect=socket.create_connection,
        clock=time.monotonic,
    ) -> None:
        self.targets, self.timeout_s, self.ttl_s = tuple(targets), timeout_s, ttl_s
        self._connect, self._clock = connect, clock
        self._lock = threading.Lock()
        self._last: dict | None = None
        self._t: float | None = None

    def _probe(self) -> dict:
        for host, port in self.targets:
            t0 = self._clock()
            try:
                s = self._connect((host, port), timeout=self.timeout_s)
            except OSError:
                continue
            try:
                s.close()
            except OSError:
                pass
            return {"online": True, "via": host, "rtt_ms": round((self._clock() - t0) * 1000.0)}
        return {"online": False, "via": None, "rtt_ms": None}

    def read(self) -> dict:
        now = self._clock()
        if self._t is not None and now - self._t < self.ttl_s:
            return dict(self._last, age_s=round(now - self._t, 1))
        if not self._lock.acquire(blocking=False):  # a probe is running: last result, or unknown
            return dict(self._last or {"online": None, "via": None, "rtt_ms": None}, age_s=None)
        try:
            self._last = self._probe()
            self._t = self._clock()
            return dict(self._last, age_s=0.0)
        finally:
            self._lock.release()
