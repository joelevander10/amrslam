"""AMR QR USB keypad: key logic (qr_keypad.KeypadState) and its merge into the virtual selector."""

from amr_base.qr_keypad import IDLE, KeypadState, name_ok
from amr_base.qr_panel import VirtualSelector

START, STOP, ESTOP = 1, 2, 0
DEV = "/dev/input/event5"


def img(estop_ok=True, comms=True):
    di = [False] * 16
    di[ESTOP] = estop_ok
    return {"comms_ok": comms, "di": di, "detail": "test"}


class Rig:
    def __init__(self, panel_keys=True, **kw):
        self.kp = KeypadState(panel_keys)
        self.sel = VirtualSelector(START, STOP, ESTOP, True, 2, **kw)
        self.t = 0.0
        self.kp.heartbeat(self.t)

    def key(self, name, value):
        return self.kp.key(DEV, name, value, self.t)

    def press(self, name):
        self.key(name, 1)
        f, e = self.scan(3)
        self.key(name, 0)
        g, e2 = self.scan(3)
        return g, {k: e[k] + e2[k] for k in e}

    def scan(self, n=3, **kw):
        out, edges = None, {"start": 0, "reset": 0}
        for _ in range(n):
            self.kp.heartbeat(self.t)
            out = self.sel.tick(img(**kw), self.t, self.kp.image(self.t))
            self.t += 0.03
            edges["start"] += out.start_edge
            edges["reset"] += out.reset_edge
        return out, edges


def test_direction_keys_are_hold_to_drive_and_combine_into_an_arc():
    r = Rig()
    r.scan()
    r.key("w", 1)
    f, _ = r.scan()
    assert f.pendant.fwd and not f.pendant.left
    r.key("a", 1)
    r.key("w", 2)  # auto-repeat changes nothing
    f, _ = r.scan()
    assert f.pendant.fwd and f.pendant.left
    r.key("w", 0)
    r.key("a", 0)
    f, _ = r.scan()
    assert f.pendant == (False, False, False, False)


def test_opposing_keys_cancel():
    r = Rig()
    r.key("w", 1)
    r.key("s", 1)
    f, _ = r.scan()
    assert not f.pendant.fwd and not f.pendant.rvs


def test_unplug_and_a_stalled_reader_release_everything():
    r = Rig()
    r.key("d", 1)
    f, _ = r.scan()
    assert f.pendant.right
    r.kp.drop(DEV)
    f, _ = r.scan()
    assert not f.pendant.right
    r.key("d", 1)
    im = r.kp.image(r.t + 5.0)  # no heartbeat for 5 s: the reader is not making progress
    assert im == IDLE


def test_no_trusted_panel_image_means_no_keypad_motion():
    r = Rig()
    r.key("w", 1)
    f, _ = r.scan(comms=False)
    assert not f.valid and not f.pendant.fwd


def test_g_is_the_start_button_and_enter_the_stop_button():
    r = Rig()
    r.scan()
    f, e = r.press("g")
    assert f.mode_auto and e["start"] == 0  # MANUAL -> AUTO starts nothing
    f, e = r.press("g")
    assert f.mode_auto and e["start"] == 1  # in AUTO: the Start edge
    f, e = r.press("enter")
    assert not f.mode_auto and e["reset"] == 0  # AUTO -> MANUAL
    f, e = r.press("enter")
    assert not f.mode_auto and e["reset"] == 1  # STOP in MANUAL = Reset


def test_q_toggles_once_per_press_and_presses_during_a_bad_image_are_dropped():
    r = Rig()
    r.scan()
    r.key("q", 1)
    f, e = r.scan()
    assert f.mode_auto and e["start"] == 0
    r.key("q", 2)  # held: no repeat toggles
    r.key("q", 0)
    f, _ = r.scan()
    assert f.mode_auto
    r.key("q", 1)
    r.key("q", 0)
    f, _ = r.scan()
    assert not f.mode_auto
    r.key("q", 1)
    r.key("q", 0)
    f, _ = r.scan(comms=False)
    f, _ = r.scan(4)
    assert f.valid and not f.mode_auto  # not queued for later


def test_e_latches_an_estop_and_enter_releases_it_without_a_stop_press():
    r = Rig()
    r.scan()
    r.press("g")  # AUTO
    r.key("e", 1)
    r.key("e", 0)
    f = r.sel.tick(img(), r.t, r.kp.image(r.t))
    assert "E-STOP ACTIVE (keypad E)" in (f.changed or "")
    f, _ = r.scan()
    assert f.estop and f.mode_auto  # an E-stop does not move the selector
    f, e = r.press("enter")
    assert not f.estop and f.mode_auto and e["reset"] == 0  # only the latch was released
    f, _ = r.press("enter")
    assert not f.mode_auto  # the next ENTER is the STOP button again


def test_the_keypad_estop_survives_an_unplug():
    r = Rig()
    r.key("e", 1)
    r.kp.drop(DEV)
    f, _ = r.scan()
    assert f.estop


def test_physical_estop_is_not_released_by_enter():
    r = Rig()
    r.scan()
    r.key("e", 1)
    f, _ = r.scan(estop_ok=False)
    f, _ = r.press("enter")
    assert r.kp.image(r.t).estop is False
    f, _ = r.scan(estop_ok=False)
    assert f.estop  # the mushroom is still open


def test_panel_keys_off_leaves_only_the_direction_keys():
    r = Rig(panel_keys=False)
    r.scan()
    assert "panel keys are off" in r.key("e", 1)
    f, _ = r.press("g")
    r.key("q", 1)
    f, _ = r.scan()
    assert not f.mode_auto and not f.estop
    r.key("w", 1)
    f, _ = r.scan()
    assert f.pendant.fwd


def test_unused_kim2a_keys_are_logged_once():
    kp = KeypadState()
    assert "not used" in kp.key(DEV, "4", 1, 0.0)
    assert kp.key(DEV, "4", 1, 0.1) is None
    assert kp.key(DEV, "f1", 0, 0.1) is None  # releases never log


def test_device_name_filters():
    assert name_ok("XING WEI 2.4G USB Keyboard", "xing wei", "AT Translated Set 2")
    assert not name_ok("Logitech USB Keyboard", "XING WEI", "")
    assert name_ok("Logitech USB Keyboard", "", "AT Translated Set 2")
    assert not name_ok("AT Translated Set 2 keyboard", "", "AT Translated Set 2, Power Button")
