"""Refuse to run beside the legacy controller.

`agv_controller` (python3 main.py, the Flask jog pad) owns can0, the DIO
island's coil and the panel while it runs. socketcan lets a second process
open can0 and Modbus TCP accepts a second client, so nothing in the OS stops
two owners - which is exactly the SDO-reply and competing-DO-writer hazard the
spec forbids (§3.1, §3.4). This is the mechanical check, used by every node
that touches those devices. T11 makes it a systemd Conflicts= as well.
"""

from __future__ import annotations

import subprocess


def legacy_controller_running() -> str | None:
    """A short reason if the legacy controller is up, else None. Never raises."""
    try:
        r = subprocess.run(
            ["systemctl", "is-active", "agv_controller"], capture_output=True, text=True, timeout=2
        )
        if r.stdout.strip() == "active":
            return "systemd unit agv_controller is active"
    except Exception:  # noqa: BLE001 - no systemd is not a reason to refuse
        pass
    try:
        r = subprocess.run(
            ["pgrep", "-f", r"python3 .*/agv_can/main\.py"], capture_output=True, text=True, timeout=2
        )
        if r.returncode == 0 and r.stdout.strip():
            return f"legacy main.py running (pid {r.stdout.split()[0]})"
    except Exception:  # noqa: BLE001
        pass
    try:
        # The AMR QR controller (Flask app.py + amr_controller.py, port 5050) owns the same
        # CK5162E/CKDA08ETH modules, the encoder bus and the IMU port as qr_base_node.
        r = subprocess.run(
            ["pgrep", "-f", r"python3? .*(amr_controller|gls621_qr_nav|trackless_module)\.py"],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if r.returncode == 0 and r.stdout.strip():
            return f"AMR QR controller running (pid {r.stdout.split()[0]})"
    except Exception:  # noqa: BLE001
        pass
    # `python3 app.py` does not name amr_controller on its command line; its Flask port
    # does identify it (nothing in this stack serves 5050).
    try:
        import socket  # noqa: PLC0415

        with socket.create_connection(("127.0.0.1", 5050), timeout=0.3):
            return "something serves 127.0.0.1:5050 (the AMR QR web app.py?)"
    except OSError:
        pass
    return None


def refuse_if_legacy_running(what: str) -> None:
    why = legacy_controller_running()
    if why:
        raise RuntimeError(f"{what} refuses to start: {why}. stop agv_controller / the AMR QR app.py first")
