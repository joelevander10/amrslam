"""AMR QR virtual selector: START/STOP buttons -> selector level + Start/Reset edges."""

from amr_base.qr_panel import VirtualSelector

START, STOP, ESTOP = 1, 2, 0


def img(start=False, stop=False, estop_ok=True, comms=True):
    di = [False] * 16
    di[START], di[STOP], di[ESTOP] = start, stop, estop_ok
    return {"comms_ok": comms, "di": di, "detail": "test"}


class Run:
    def __init__(self, **kw):
        self.sel = VirtualSelector(START, STOP, ESTOP, True, 2, **kw)
        self.t = 0.0

    def hold(self, scans=2, **kw):
        out = None
        edges = {"start": 0, "reset": 0}
        for _ in range(scans):
            out = self.sel.tick(img(**kw), self.t)
            self.t += 0.03
            edges["start"] += out.start_edge
            edges["reset"] += out.reset_edge
        return out, edges


def test_baseline_needs_debounce_and_a_held_start_at_power_on_is_not_a_press():
    r = Run()
    f, _ = r.hold(1, start=True)
    assert not f.valid  # one scan is not debounced
    f, e = r.hold(3, start=True)
    assert f.valid and not f.mode_auto and e == {"start": 0, "reset": 0}
    f, _ = r.hold(3)  # released
    f, _ = r.hold(3, start=True)  # a real press
    assert f.mode_auto


def test_start_enters_auto_then_is_the_start_edge_and_stop_goes_back():
    r = Run()
    r.hold(3)
    f, e = r.hold(3, start=True)
    assert f.mode_auto and e["start"] == 0  # switching modes starts nothing
    r.hold(3)
    f, e = r.hold(3, start=True)
    assert f.mode_auto and e["start"] == 1
    r.hold(3)
    f, e = r.hold(3, stop=True)
    assert not f.mode_auto and e["reset"] == 0
    r.hold(3)
    f, e = r.hold(3, stop=True)
    assert not f.mode_auto and e["reset"] == 1  # STOP in MANUAL = Reset


def test_stop_wins_over_start():
    r = Run()
    r.hold(3)
    f, e = r.hold(3, start=True, stop=True)
    assert not f.mode_auto and e == {"start": 0, "reset": 1}
    f, e = r.hold(3, start=True)  # STOP released, START still held from before: spent
    assert not f.mode_auto and e["start"] == 0


def test_estop_is_reported_and_does_not_move_the_selector():
    r = Run()
    r.hold(3)
    r.hold(3, start=True)
    r.hold(3)
    f, _ = r.hold(3, estop_ok=False)
    assert f.valid and f.estop and f.mode_auto
    f, _ = r.hold(3)
    assert not f.estop


def test_comms_loss_invalidates_and_falls_back_to_manual_and_rebaselines():
    r = Run()
    r.hold(3)
    r.hold(3, start=True)
    f, _ = r.hold(2, comms=False)
    assert not f.valid and not f.mode_auto and f.estop
    f, e = r.hold(3, start=True)  # START held across the gap: baseline, not a press
    assert f.valid and not f.mode_auto and e["start"] == 0


def test_auto_hold_long_press_for_auto_short_press_is_a_manual_start_edge():
    r = Run(auto_hold_s=1.0)
    r.hold(3)
    f, e = r.hold(5, start=True)  # 0.15 s
    assert not f.mode_auto and e["start"] == 0
    f, e = r.hold(3)  # released early -> MANUAL start edge
    assert not f.mode_auto and e["start"] == 1
    f, e = r.hold(40, start=True)  # 1.2 s held -> AUTO, no edge
    assert f.mode_auto and e["start"] == 0
    f, e = r.hold(3)
    assert e["start"] == 0
    f, e = r.hold(3, start=True)
    assert e["start"] == 1  # in AUTO a press is the Start edge at once
