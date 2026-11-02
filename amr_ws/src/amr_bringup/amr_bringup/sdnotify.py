"""sd_notify in ten lines: READY=1 and WATCHDOG=1 over the systemd socket.

A supervisor whose loop is wedged is still an "alive process" to systemd, and
`Restart=on-failure` never fires. With `Type=notify` + `WatchdogSec` the loop
itself has to keep saying it is alive, so a hung loop is restarted exactly like a
crash - and the stop path is already bounded (TimeoutStopSec 55 s) with motion
inhibited throughout.

No python-systemd dependency: the protocol is one datagram to the socket named
in NOTIFY_SOCKET. Outside systemd the socket is absent and every call is a no-op,
so sim runs, tests and bench launches are unaffected.
"""

from __future__ import annotations

import os
import socket


class Notifier:
    def __init__(self) -> None:
        self.address = os.environ.get("NOTIFY_SOCKET", "")
        self.enabled = bool(self.address)
        # WatchdogSec in microseconds; systemd asks for a ping at HALF the interval.
        usec = os.environ.get("WATCHDOG_USEC", "")
        self.interval_s = (int(usec) / 2_000_000.0) if usec.isdigit() else 0.0
        self._sock: socket.socket | None = None
        if self.enabled:
            try:
                self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM | socket.SOCK_CLOEXEC)
            except OSError:
                self.enabled = False

    def _send(self, message: str) -> None:
        if not self.enabled or self._sock is None:
            return
        addr = self.address
        if addr.startswith("@"):  # abstract namespace
            addr = "\0" + addr[1:]
        try:
            self._sock.sendto(message.encode(), addr)
        except OSError:
            pass  # a lost notification must never disturb the vehicle

    def ready(self) -> None:
        self._send("READY=1")

    def watchdog(self) -> None:
        self._send("WATCHDOG=1")

    def status(self, text: str) -> None:
        self._send(f"STATUS={text[:200]}")

    def stopping(self) -> None:
        self._send("STOPPING=1")
