"""USB keypad on the AMR QR PC: hold-to-drive jog + the panel buttons, KIM2A key map.

Ported from agv-kim2a-controller (drivers/keypad_evdev.py + keyboard_control.py): the same
wireless keypad / keyboard, read straight from Linux (/dev/input/event*, python-evdev),
hot-plugged, GRABbed so its keys never reach the PC's desktop or a terminal.

    W / up      forward             S / down    reverse
    A / left    spin left (CCW)     D / right   spin right (CW)
    W+A W+D S+A S+D                 curve (the pendant's arc, slow wheel at turn_ratio)
    G           the START button    (MANUAL -> AUTO; in AUTO starts / resumes the route)
    ENTER       the STOP button     (AUTO -> MANUAL; in MANUAL = Reset, acks a route FAULT)
    Q           switch MANUAL <-> AUTO (one press)
    E           keypad EMERGENCY STOP, latched; ENTER releases it (that press does nothing else)

How it reaches the vehicle - nothing new downstream:
  * the four direction keys are a SECOND PENDANT: qr_base_node ORs them into the panel
    image's pendant levels, so the mux gives them exactly the pendant's authority (MANUAL
    only, outranks the browser jog), speeds (pendant_v_m_s / pendant_w_rad_s), S-curve, the
    survey spin cap and the after-E-stop "release once" guard;
  * G / ENTER are ORed into the START / STOP levels of the virtual selector (qr_panel.py),
    so they behave exactly like the push buttons, auto_hold_s included;
  * E is ORed into the E-stop: emergency ACTIVE, drives out, as the mushroom does. It is
    a SOFTWARE stop - not safety rated; the mushroom and the scanner OSSD are the real ones.

Release, unplug, a read error or a reader that stops making progress (heartbeat older than
STALE_S) all read as "nothing held". A wireless keyboard that goes out of range is the
receiver's business: most send the release on link loss, but the host cannot know.

Keys of the KIM2A map with nothing to do here (speed 1-5, K/L track, T/Y rotate, R,
F1-F12 cabang, the PLC trolley keys) are ignored, each logged once.

The key logic (KeypadState) is pure and tested; KeypadReader is the evdev part.
    ros2 run amr_base qr_keypad --list     which input devices would be used
    ros2 run amr_base qr_keypad --watch    print what the keys do (stop amr.service first:
                                           the service GRABs the keypad)
"""

from __future__ import annotations

import select
import threading
import time
from dataclasses import dataclass

STALE_S = 1.0  # reader heartbeat older than this: every held key reads released
SCAN_S = 2.0  # hot-plug rescan
POLL_S = 0.2  # reader wake-up (heartbeat) period

MOVE = {"w": "fwd", "s": "rvs", "a": "left", "d": "right"}
PANEL = {"g", "enter", "q", "e"}
# the rest of the KIM2A / Kayaba layout, recognised so it is logged once as unused
UNUSED = set("12345klrtyzxuiobnm") | {f"f{i}" for i in range(1, 13)}


@dataclass(frozen=True)
class KeypadImage:
    fwd: bool = False
    rvs: bool = False
    left: bool = False
    right: bool = False
    start: bool = False  # G held
    stop: bool = False  # ENTER held (not the press that released the keypad E-stop)
    toggles: int = 0  # Q presses so far (the selector acts on a change)
    estop: bool = False  # keypad E-stop latched


IDLE = KeypadImage()


class KeypadState:
    """Held keys per device, the E latch and the Q counter. Thread-safe, pure (no evdev)."""

    def __init__(self, panel_keys: bool = True, stale_s: float = STALE_S) -> None:
        self.panel_keys = bool(panel_keys)
        self.stale_s = float(stale_s)
        self._lock = threading.Lock()
        self._held: dict[str, set[str]] = {}
        self._alive_t: float | None = None
        self._estop = False
        self._toggles = 0
        self._unused_logged: set[str] = set()
        self.devices: list[str] = []
        self.detail = "no reader"

    def heartbeat(self, now: float, devices: list[str] | None = None, detail: str | None = None) -> None:
        with self._lock:
            self._alive_t = now
            if devices is not None:
                self.devices = list(devices)
            if detail is not None:
                self.detail = detail

    def key(self, device: str, name: str, value: int, now: float) -> str | None:
        """One evdev key event (value 1 press, 0 release, 2 auto-repeat). Returns a log note."""
        if value == 2:
            return None  # auto-repeat: the level is already held
        press = value == 1
        with self._lock:
            self._alive_t = now
            held = self._held.setdefault(device, set())
            if name in MOVE:
                (held.add if press else held.discard)(name)
                return None
            if name in PANEL:
                if not self.panel_keys:
                    return self._once(name, "panel keys are off (qr_base.keypad_panel_keys)")
                if name == "enter":
                    if press and self._estop:
                        # this press only releases the latch: it is never held as STOP
                        self._estop = False
                        return "keypad ENTER: keypad E-stop released"
                    (held.add if press else held.discard)(name)
                    return None
                if name == "g":
                    (held.add if press else held.discard)(name)
                    return None
                if not press:
                    return None
                if name == "q":
                    self._toggles += 1
                    return "keypad Q: switch MANUAL/AUTO"
                if not self._estop:  # "e"
                    self._estop = True
                    return "keypad E: EMERGENCY STOP (ENTER releases it)"
                return None
            if press and name in UNUSED:
                return self._once(name, "not used on the AMR QR")
            return None

    def drop(self, device: str) -> None:
        """A device went away: everything it held is released (the E latch stays)."""
        with self._lock:
            self._held.pop(device, None)

    def image(self, now: float) -> KeypadImage:
        with self._lock:
            alive = self._alive_t is not None and now - self._alive_t <= self.stale_s
            held = set().union(*self._held.values()) if (alive and self._held) else set()
            return KeypadImage(
                fwd="w" in held,
                rvs="s" in held,
                left="a" in held,
                right="d" in held,
                start="g" in held,
                stop="enter" in held,
                toggles=self._toggles,
                estop=self._estop,
            )

    def _once(self, name: str, why: str) -> str | None:
        if name in self._unused_logged:
            return None
        self._unused_logged.add(name)
        return f"keypad {name.upper()} ignored: {why}"


def name_ok(name: str, contains: str, exclude: str) -> bool:
    """Profile filters: `contains` (empty = any keyboard), `exclude` comma-separated."""
    low = (name or "").lower()
    if contains and contains.lower() not in low:
        return False
    return not any(x.strip() and x.strip().lower() in low for x in (exclude or "").split(","))


def code_map(ecodes) -> dict[int, str]:
    """evdev key code -> key name (arrows drive like WASD; numpad Enter = Enter)."""
    m = {}
    for c in "wasdgqeklrtyzxuiobnm":
        m[getattr(ecodes, f"KEY_{c.upper()}")] = c
    for d in "12345":
        m[getattr(ecodes, f"KEY_{d}")] = d
        m[getattr(ecodes, f"KEY_KP{d}")] = d
    for code, name in (("UP", "w"), ("DOWN", "s"), ("LEFT", "a"), ("RIGHT", "d")):
        m[getattr(ecodes, f"KEY_{code}")] = name
    m[ecodes.KEY_ENTER] = m[ecodes.KEY_KPENTER] = "enter"
    for i in range(1, 13):
        m[getattr(ecodes, f"KEY_F{i}")] = f"f{i}"
    return m


class KeypadReader:  # pragma: no cover - needs /dev/input
    """Hot-plug evdev reader feeding a KeypadState; run() until `stop` is set."""

    def __init__(self, state: KeypadState, contains: str, exclude: str, grab: bool, log) -> None:
        self.state, self.contains, self.exclude, self.grab, self.log = state, contains, exclude, grab, log

    def run(self, stop: threading.Event) -> None:
        try:
            import evdev  # noqa: PLC0415
            from evdev import ecodes  # noqa: PLC0415
        except ImportError:
            self.state.heartbeat(time.monotonic(), [], "python3-evdev not installed")
            self.log("warn", "USB keypad: python3-evdev is not installed (sudo apt install python3-evdev)")
            return
        codes = code_map(ecodes)
        devs: dict[str, object] = {}  # path -> InputDevice
        next_scan = 0.0
        self.log("info", f"USB keypad: watching (name contains {self.contains!r}, grab {self.grab})")
        try:
            while not stop.is_set():
                now = time.monotonic()
                if now >= next_scan:
                    next_scan = now + SCAN_S
                    self._scan(evdev, ecodes, devs)
                names = [d.name for d in devs.values()]
                self.state.heartbeat(now, names, "connected: " + ", ".join(names) if names else "no keypad")
                if not devs:
                    stop.wait(POLL_S)
                    continue
                try:
                    ready, _, _ = select.select(list(devs.values()), [], [], POLL_S)
                except (OSError, ValueError):
                    ready = list(devs.values())  # find the dead one below
                for dev in ready:
                    try:
                        for ev in dev.read():
                            if ev.type == ecodes.EV_KEY and ev.code in codes:
                                note = self.state.key(dev.path, codes[ev.code], ev.value, time.monotonic())
                                if note:
                                    self.log("info", note)
                    except (OSError, BlockingIOError) as e:
                        if isinstance(e, BlockingIOError):
                            continue
                        self.log("warn", f"USB keypad '{dev.name}' disconnected ({e})")
                        self._close(devs, dev.path)
        finally:
            for path in list(devs):
                self._close(devs, path)

    def _scan(self, evdev, ecodes, devs) -> None:
        for path in evdev.list_devices():
            if path in devs:
                continue
            try:
                dev = evdev.InputDevice(path)
            except OSError:
                continue  # no permission (group input) or vanished
            keys = dev.capabilities().get(ecodes.EV_KEY, [])
            keyboard = ecodes.KEY_W in keys or ecodes.KEY_UP in keys
            if not keyboard or not name_ok(dev.name, self.contains, self.exclude):
                dev.close()
                continue
            if self.grab:
                try:
                    dev.grab()
                except OSError as e:
                    self.log("warn", f"USB keypad: could not grab '{dev.name}' ({e})")
            devs[path] = dev
            self.log("info", f"USB keypad connected: '{dev.name}' ({path})")

    def _close(self, devs, path) -> None:
        dev = devs.pop(path, None)
        self.state.drop(path)
        if dev is not None:
            try:
                dev.close()
            except Exception:  # noqa: BLE001 - already gone
                pass


def main(argv: list[str] | None = None) -> None:  # pragma: no cover - bench tool
    import argparse  # noqa: PLC0415

    ap = argparse.ArgumentParser(description="AMR QR USB keypad: list devices or watch the keys")
    ap.add_argument("--list", action="store_true", help="input devices and whether the profile uses them")
    ap.add_argument("--watch", action="store_true", help="print what the keys do (no ROS, nothing moves)")
    a = ap.parse_args(argv)
    try:
        from agv_core import config  # noqa: PLC0415

        contains = str(getattr(config, "QR_KEYPAD_NAME_CONTAINS", ""))
        exclude = str(getattr(config, "QR_KEYPAD_EXCLUDE_CONTAINS", ""))
        panel_keys = bool(getattr(config, "QR_KEYPAD_PANEL_KEYS", True))
    except Exception:  # noqa: BLE001 - no profile: show every keyboard
        contains, exclude, panel_keys = "", "", True
    try:
        import evdev  # noqa: PLC0415
        from evdev import ecodes  # noqa: PLC0415
    except ImportError:
        raise SystemExit("python3-evdev is not installed: sudo apt install python3-evdev") from None
    if a.list or not a.watch:
        paths = evdev.list_devices()
        if not paths:
            print("no /dev/input/event* readable - is this user in group 'input'?")
        for path in paths:
            dev = evdev.InputDevice(path)
            keys = dev.capabilities().get(ecodes.EV_KEY, [])
            kb = ecodes.KEY_W in keys or ecodes.KEY_UP in keys
            use = kb and name_ok(dev.name, contains, exclude)
            print(f"{path:20s} {'USED ' if use else ('kbd  ' if kb else '     ')} {dev.name}")
        print(f"(profile: name contains {contains!r}, exclude {exclude!r})")
        return
    state = KeypadState(panel_keys)
    stop = threading.Event()
    reader = KeypadReader(state, contains, exclude, False, lambda lvl, msg: print(f"[{lvl}] {msg}"))
    threading.Thread(target=reader.run, args=(stop,), daemon=True).start()
    shown = None
    try:
        while True:
            im = state.image(time.monotonic())
            if im != shown:
                shown = im
                print(im)
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop.set()


if __name__ == "__main__":
    main()
