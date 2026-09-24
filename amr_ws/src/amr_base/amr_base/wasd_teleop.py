"""WASD keyboard teleop on /cmd_vel_teleop, hold-to-drive.

    W / up     forward          S / down   backward
    A / left   turn left (CCW)  D / right  turn right (CW)
    Q / E      forward arc left / right
    Z / C      backward arc left / right
    space      stop now          + / -     linear speed up / down
    ] / [      turn speed up / down         Ctrl-C  quit (sends stop)

A terminal only sends key REPEATS while a key is held, after an initial delay (about
0.5 s). So the first press drives for FIRST_HOLD_S, and each repeat extends by
REPEAT_HOLD_S: releasing the key stops the vehicle within REPEAT_HOLD_S. The mux only
accepts /cmd_vel_teleop with the panel in MANUAL, and drops it after 0.5 s of silence.

The key handling (KeyHold, key_to_twist) is pure and tested; the node is the thin part.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

FIRST_HOLD_S = 0.7  # covers the terminal's initial repeat delay
REPEAT_HOLD_S = 0.2  # a held key repeats every ~30-50 ms
RATE_HZ = 20.0
STOP_BURST = 3  # zero commands sent after a release, then silence (no teleop authority held)

V_START, V_STEP, V_MAX = 0.10, 0.05, 0.40  # m/s
W_START, W_STEP, W_MAX = 0.30, 0.10, 1.00  # rad/s

# key -> (linear sign, angular sign)
MOTION = {
    "w": (1, 0),
    "s": (-1, 0),
    "a": (0, 1),
    "d": (0, -1),
    "q": (1, 1),
    "e": (1, -1),
    "z": (-1, -1),  # backward arcs: the tail swings the way the key points
    "c": (-1, 1),
    "UP": (1, 0),
    "DOWN": (-1, 0),
    "LEFT": (0, 1),
    "RIGHT": (0, -1),
}
ARROWS = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}


@dataclass
class Speeds:
    v: float = V_START
    w: float = W_START

    def adjust(self, key: str) -> bool:
        """Speed keys; True if the key was one."""
        if key in ("+", "="):
            self.v = min(V_MAX, round(self.v + V_STEP, 3))
        elif key in ("-", "_"):
            self.v = max(V_STEP, round(self.v - V_STEP, 3))
        elif key == "]":
            self.w = min(W_MAX, round(self.w + W_STEP, 3))
        elif key == "[":
            self.w = max(W_STEP, round(self.w - W_STEP, 3))
        else:
            return False
        return True


def key_to_twist(key: str, sp: Speeds) -> tuple[float, float] | None:
    """(linear m/s, angular rad/s) for a motion key, None otherwise."""
    m = MOTION.get(key if key in ARROWS.values() else key.lower())
    if m is None:
        return None
    lin, ang = m
    # an arc turns at half rate, so it curves rather than pivots
    w = sp.w * (0.5 if lin and ang else 1.0)
    return lin * sp.v, ang * w


class KeyHold:
    """Which motion is held, from key events and their timing. Pure."""

    def __init__(self, first_s: float = FIRST_HOLD_S, repeat_s: float = REPEAT_HOLD_S) -> None:
        self.first_s, self.repeat_s = first_s, repeat_s
        self.key: str | None = None
        self.until = 0.0
        self._last = 0.0

    def press(self, key: str, now: float) -> None:
        repeat = key == self.key and now - self._last <= self.first_s
        self.key, self._last = key, now
        self.until = now + (self.repeat_s if repeat else self.first_s)

    def stop(self) -> None:
        self.key, self.until = None, 0.0

    def held(self, now: float) -> str | None:
        return self.key if self.key is not None and now <= self.until else None


def split_keys(data: str) -> list[str]:
    """Raw terminal bytes -> key names (arrow escape sequences become UP/DOWN/LEFT/RIGHT)."""
    out, i = [], 0
    while i < len(data):
        if data[i] == "\x1b" and data[i + 1 : i + 2] in ("[", "O") and data[i + 2 : i + 3] in ARROWS:
            out.append(ARROWS[data[i + 2]])
            i += 3
            continue
        out.append(data[i])
        i += 1
    return out


def main() -> None:  # pragma: no cover - needs a terminal and ROS
    import rclpy  # noqa: PLC0415
    from geometry_msgs.msg import Twist  # noqa: PLC0415
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: PLC0415

    if not sys.stdin.isatty():
        sys.exit("wasd_teleop needs a terminal (run it directly, not from a launch file)")
    rclpy.init()
    node = rclpy.create_node("wasd_teleop")
    qos = QoSProfile(
        depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE
    )
    pub = node.create_publisher(Twist, "/cmd_vel_teleop", qos)

    def send(v: float, w: float) -> None:
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        pub.publish(t)

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    sp, hold = Speeds(), KeyHold()
    stops_left, shown = 0, None
    print(__doc__.split("\n\n")[1].replace("\n", "\r\n") + "\r")
    try:
        tty.setcbreak(fd)
        period = 1.0 / RATE_HZ
        while rclpy.ok():
            r, _, _ = select.select([fd], [], [], period)
            now = time.monotonic()
            if r:
                for k in split_keys(os.read(fd, 64).decode(errors="ignore")):
                    if k == "\x03":
                        raise KeyboardInterrupt
                    if k == " ":
                        hold.stop()
                    elif sp.adjust(k):
                        pass
                    elif key_to_twist(k, sp) is not None:
                        hold.press(k, now)
            k = hold.held(now)
            if k is not None:
                v, w = key_to_twist(k, sp)
                send(v, w)
                stops_left = STOP_BURST
            elif stops_left:
                send(0.0, 0.0)
                stops_left -= 1
                v = w = 0.0
            else:
                v = w = 0.0
            line = f"speed {sp.v:.2f} m/s  turn {sp.w:.2f} rad/s   cmd v {v:+.2f} w {w:+.2f}"
            if line != shown:
                shown = line
                sys.stdout.write("\r" + line + "   ")
                sys.stdout.flush()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        try:
            for _ in range(STOP_BURST):
                send(0.0, 0.0)
                time.sleep(0.02)
        except Exception:  # noqa: BLE001 - context may already be down
            pass
        print("\r\nstopped")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
