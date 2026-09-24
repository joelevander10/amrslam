"""Wi-Fi link readout for the header: signal in dBm from /proc/net/wireless.

The kernel row is free to read at any rate; the SSID needs nmcli, so it is
refreshed at most every SSID_TTL_S and never blocks a poll for long. Nothing
here is authority for anything - it is a comfort indicator for whoever holds
the tablet, so a missing interface simply reads "no link".
"""

from __future__ import annotations

import subprocess
import time

PROC_WIRELESS = "/proc/net/wireless"
SSID_TTL_S = 10.0

# dBm thresholds for 4..1 bars; below the last is 0 bars (still associated)
BAR_DBM = (-55, -65, -75, -85)


def bars_for(dbm: float | None) -> int:
    if dbm is None:
        return 0
    return sum(1 for t in BAR_DBM if dbm >= t)


def parse_proc(text: str, iface: str) -> float | None:
    """Signal level in dBm for iface, or None if it has no row (not associated)."""
    for line in text.splitlines():
        if ":" not in line:
            continue
        name, rest = line.split(":", 1)
        if name.strip() != iface:
            continue
        fields = rest.split()
        try:
            return float(fields[2].rstrip("."))
        except (IndexError, ValueError):
            return None
    return None


def _nmcli(*args: str) -> str | None:
    try:
        out = subprocess.run(
            ["nmcli", *args], capture_output=True, text=True, timeout=1.0, check=False
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    line = out.strip().splitlines()[0] if out.strip() else ""
    value = line.split(":", 1)[1].strip() if ":" in line else ""
    return value or None


class WifiReader:
    def __init__(self, iface: str, proc_path: str = PROC_WIRELESS) -> None:
        self.iface = iface
        self.proc_path = proc_path
        self._ssid: str | None = None
        self._ssid_t = 0.0

    def _read_dbm(self) -> float | None:
        try:
            with open(self.proc_path, encoding="ascii", errors="replace") as f:
                return parse_proc(f.read(), self.iface)
        except OSError:
            return None

    def _read_ssid(self) -> str | None:
        now = time.monotonic()
        if now - self._ssid_t < SSID_TTL_S:
            return self._ssid
        self._ssid_t = now
        # Two lookups against NetworkManager's cache: neither triggers a scan.
        conn = _nmcli("-t", "-f", "GENERAL.CONNECTION", "dev", "show", self.iface)
        ssid = _nmcli("-t", "-f", "802-11-wireless.ssid", "con", "show", conn) if conn else None
        self._ssid = ssid or conn or None
        return self._ssid

    def read(self) -> dict:
        dbm = self._read_dbm()
        return {
            "iface": self.iface,
            "connected": dbm is not None,
            "ssid": self._read_ssid() if dbm is not None else None,
            "dbm": dbm,
            "bars": bars_for(dbm),
        }
