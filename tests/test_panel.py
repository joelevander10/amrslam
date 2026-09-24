"""Operator panel: debounced edges, and the Reset-means-READY state machine."""
import copy
import json

import time

from helpers import ROOT, check

import config
import panel
from panel import AUTO, MANUAL

R, S, A = 0, 1, 2                       # channel indices used by these tests


def image(reset=False, start=False, auto=False):
    di = [False] * 16
    di[R], di[S], di[A] = reset, start, auto
    return di


def scanner(debounce=1):
    return panel.PanelScan(R, S, A, debounce)


def run(sc, frames, comms_ok=True):
    """Feed a list of images, return the list of intents."""
    return [sc.scan(f, comms_ok) for f in frames]


def test_edges_and_tie_down():
    """A press is an edge, and a button already down at boot is not a press."""
    print("\npanel: edges")

    sc = scanner()
    out = run(sc, [image(), image(start=True), image(start=True),
                   image(start=True), image()])
    check("the first image is a baseline and emits nothing",
          not out[0].start and not out[0].reset)
    check("pressing Start is one edge", out[1].start is True)
    check("holding it is not another edge - a half-second press covers ten "
          "scans", [o.start for o in out[2:]] == [False, False, False],
          str([o.start for o in out]))

    # The hazard this exists for: a taped-down button, or a selector already
    # sitting in AUTO, must not act at power-on.
    sc = scanner()
    out = run(sc, [image(start=True, auto=True), image(start=True, auto=True)])
    check("Start held down at boot never fires", not any(o.start for o in out))
    check("and the selector is reported, not treated as a change",
          out[0].mode == AUTO and not out[0].mode_changed)

    sc = scanner()
    out = run(sc, [image(), image(reset=True), image()])
    check("Reset is edge-detected the same way",
          [o.reset for o in out] == [False, True, False])


def test_selector():
    """The selector is a level, and only its transitions matter."""
    print("\npanel: selector")

    sc = scanner()
    out = run(sc, [image(), image(auto=True), image(auto=True), image()])
    check("low reads MANUAL", out[0].mode == MANUAL)
    check("high reads AUTO", out[1].mode == AUTO)
    check("and the move is flagged once", out[1].mode_changed is True)
    check("holding the position is not a change", out[2].mode_changed is False)
    check("moving back flags again",
          out[3].mode == MANUAL and out[3].mode_changed is True)
    check("mode() reports it for the UI", sc.mode() == MANUAL)


def test_debounce():
    """A level must read the same on N consecutive scans to be believed."""
    print("\npanel: debounce")

    sc = scanner(debounce=2)
    run(sc, [image(), image()])                     # baseline
    out = run(sc, [image(start=True)])
    check("a single-scan glitch is not yet a press", out[0].start is False)
    out = run(sc, [image()])
    check("and if it goes away it never becomes one", out[0].start is False)

    out = run(sc, [image(start=True), image(start=True)])
    check("two consecutive scans are accepted",
          [o.start for o in out] == [False, True])

    sc = scanner(debounce=3)
    run(sc, [image(auto=True)] * 3)                 # settle on AUTO
    out = run(sc, [image()])                        # moving, not yet believed
    check("an unsettled contact keeps reporting the last SETTLED selector "
          "position, so the UI does not flicker mid-bounce",
          out[0].valid and out[0].mode == AUTO and not out[0].mode_changed,
          f"valid={out[0].valid} mode={out[0].mode}")


def test_comms_loss_cannot_synthesise_a_press():
    """The reconnect hazard: stale bits are not input."""
    print("\npanel: comms loss")

    sc = scanner()
    run(sc, [image(), image(auto=True)])            # baselined, selector AUTO

    out = sc.scan(image(auto=True), False)
    check("a scan with comms down is not valid", out.valid is False)
    check("and reports no edges", not out.start and not out.reset)

    # The link returns while Start happens to read high. Comparing against the
    # pre-outage image would invent a press that nobody made.
    out = run(sc, [image(start=True, auto=True), image(start=True, auto=True)])
    check("the first scan back re-baselines instead of firing Start",
          out[0].start is False)
    check("and the held button still does not fire on later scans",
          out[1].start is False)
    check("releasing and pressing again works normally",
          run(sc, [image(auto=True), image(start=True, auto=True)])[1].start
          is True)

    # A short or malformed image is not a button press either.
    check("a short image is refused", sc.scan([True, True], True).valid is False)
    check("None is refused", sc.scan(None, True).valid is False)


def test_reset_means_ready():
    """The controller's state machine, driven through its own handlers.

    The shape changed: MANUAL is an armed state the selector holds, AUTO is a
    disarmed one that Start energises. Reset no longer arms anything - it was
    being pressed reflexively before every jog, which is what this is fixing.
    """
    import canworker
    import config
    import events
    print("\npanel: MANUAL is armed, AUTO is armed by Start")

    calls = []

    class Ctl(canworker.Controller):
        """Real handlers, faked slow actions - no bus, no drivers."""
        def __init__(self):
            self._lock = __import__("threading").Lock()
            self._armed = False
            self._mode = "idle"
            self._direction = "stop"
            self._target = (0, 0)
            self._deadline = 0.0
            self._fault = None
            self._last_action = None
            self._last_stop_reason = None
            self._panel = scanner()
            self._arm_retry_at = 0.0
            self._arm_fail = None
            self._telemetry = {n: {"statusword": 0x0027} for n in config.NODES}
            # No blind-run plan set: Start in MANUAL has nothing to run.
            self._blind_plan = None
            self._blind = None
            self._blind_start_at = 0.0
            self._blind_abort_req = None
            self._counts_per_wheel_rev = None
            self.arm_fails = None

        def _do_arm(self, mode):
            calls.append(("arm", mode))
            if self.arm_fails:
                raise RuntimeError(self.arm_fails)
            self._armed, self._mode = True, mode

        def _do_disarm(self):
            calls.append(("disarm",))
            self._armed, self._mode = False, "idle"

    events.clear()
    c = Ctl()

    # ---- MANUAL: the selector is the arm command --------------------------
    c._hold_arm_state(MANUAL)
    check("the selector resting in MANUAL arms it, with nothing pressed",
          c._armed and c._mode == "manual", str(calls))
    n = len(calls)
    for _ in range(20):
        c._hold_arm_state(MANUAL)
    check("...and holding it there does not re-arm every scan",
          len(calls) == n, f"{len(calls) - n} extra call(s)")

    c._panel_start(MANUAL)
    check("Start in MANUAL with no blind-run plan does nothing - jogging is "
          "per-direction from the web pad",
          ("run", True, "panel") not in calls and not c._blind_start_at)

    # ---- AUTO is the resting state now ------------------------------------
    # *** There is nothing autonomous to start. *** Tape following was the only
    # thing Start ever launched. The button, its debounced edge and the
    # anti-tie-down rule are all still live and still tested above; what is gone
    # is the run. Start says so rather than silently doing nothing.
    c._panel_mode_changed(AUTO)
    check("the selector moving to AUTO disarms", not c._armed)
    c._hold_arm_state(AUTO)
    check("...and AUTO does not arm itself", not c._armed, str(c._mode))

    events.clear()
    c._panel_start(AUTO)
    check("Start in AUTO cannot arm anything", not c._armed)
    check("...and cannot produce motion", c._target == (0, 0), str(c._target))
    msgs = " ".join(e["msg"] for e in events.since(0)[1])
    check("...and says why, rather than being silently inert",
          "no autonomous mode" in msgs, msgs[:70])

    # ---- Reset only stops and acknowledges --------------------------------
    c._armed = True
    c._target = (500, 500)
    c._panel_reset(AUTO)
    check("Reset zeroes the setpoint", c._target == (0, 0), str(c._target))
    c._hold_arm_state(AUTO)
    check("...and AUTO returns to disarmed", not c._armed, str(c._mode))

    # ---- faults still need a human ----------------------------------------
    c3 = Ctl()
    c3._set_fault("line lost")
    c3._hold_arm_state(MANUAL)
    check("a latched fault stops MANUAL arming itself", not c3._armed)
    c3._panel_start(AUTO)
    check("...and Start is refused too", not c3._armed)
    c3._panel_reset(MANUAL)
    check("Reset clears the fault", c3._fault is None)
    c3._hold_arm_state(MANUAL)
    check("...and the selector arms it again with no second press", c3._armed)

    # ---- a failed arm is a condition, not a fault -------------------------
    c4 = Ctl()
    c4.arm_fails = "preflight failed: driver 2 not answering"
    c4._hold_arm_state(MANUAL)
    check("an auto-arm that fails does NOT latch a fault - nobody did anything "
          "wrong", c4._fault is None and not c4._armed, str(c4._fault))
    check("...it records why", c4._arm_fail is not None, str(c4._arm_fail))
    n = len(calls)
    for _ in range(50):
        c4._hold_arm_state(MANUAL)
    check("...and backs off rather than hammering the bus at tick rate",
          len(calls) == n, f"{len(calls) - n} attempt(s) in 50 scans")
    c4.arm_fails = None
    c4._arm_retry_at = 0.0
    c4._hold_arm_state(MANUAL)
    check("...then arms by itself once the cause is gone, with no Reset",
          c4._armed and c4._fault is None)

    # ---- torque taken away underneath it ----------------------------------
    c5 = Ctl()
    c5._hold_arm_state(MANUAL)
    check("armed in manual", c5._armed)
    # 0x1270 = Switch on disabled: what the drives report in ETO, which is what
    # the safety chain leaves behind.
    c5._telemetry = {n: {"statusword": 0x1270} for n in config.NODES}
    c5._arm_retry_at = 0.0
    c5._hold_arm_state(MANUAL)
    check("drives dropping out of Operation enabled triggers a re-arm",
          ("disarm",) in calls[-3:] or c5._armed, str(calls[-3:]))
    c6 = Ctl()
    c6._telemetry = {n: {"statusword": None} for n in config.NODES}
    check("an unread statusword is 'unknown', never 'not ready'",
          c6._drives_ready() is None)
    events.clear()


def test_the_manual_watchdog_does_not_latch():
    """The manual watchdog zeroes the setpoint without latching a fault.

    A manual watchdog trip is self-correcting - the setpoint is already zero and
    recovery is to hold the button again, with a hand on the control. Latching
    it made a dropped Wi-Fi packet cost a walk to the panel, and under
    panel.manual_auto_arm it would block re-arming as well.
    """
    import events
    print("\npanel: the manual watchdog stops without latching")

    src = (ROOT / "canworker.py").read_text(encoding="utf-8")
    block = src[src.index("watchdog: no keepalive"):]
    block = block[:block.index("# Device liveness")]

    # A CALL, not a mention - the comment names _set_fault() to explain why it
    # is deliberately not used here, and that comment is worth keeping.
    check("it warns instead of latching",
          "events.warn" in block and "self._set_fault(" not in block)
    check("it zeroes the setpoint outright",
          "self._target = target = (0, 0)" in block)
    # *** The latching case is not gone, only unreachable. *** Latched motion
    # that would resume on its own MUST latch a fault, and tape following did.
    # The source comment is what carries that to whoever adds navigation; this
    # pins it so it cannot be deleted as noise.
    check("the latching case is recorded for whatever navigates next",
          "autonomous mode returns" in block)

    # The emit sits on a 50 Hz path, so it is only safe because the branch
    # clears its own condition. Pin that the zeroing is still there to do it.
    warn_at = block.index("events.warn")
    zero_at = block.index("self._target = target = (0, 0)")
    check("the branch self-clears after warning, so it cannot repeat at 50 Hz",
          zero_at > warn_at)

    # And the reason still reaches the operator either way.
    check("the stop reason is recorded for the UI in both modes",
          "self._last_stop_reason = reason" in block)
    events.clear()


def test_panel_events_are_edge_only():
    """_panel_scan runs at 50 Hz; the event ring holds 200 entries."""
    import events
    print("\npanel: event discipline")

    src = (ROOT / "canworker.py").read_text(encoding="utf-8")
    # The panel is what ENTERS every state, so it has to be scanned while idle
    # - from the bus loop itself, not from anything gated on being armed.
    loop = src[src.index("while not self._stop_evt.is_set():"):]
    check("the panel is scanned from the bus loop, in every state",
          "self._panel_scan()" in loop
          and loop.index("self._panel_scan()") < loop.index("if armed"))

    class C:
        _lock = __import__("threading").Lock()
        _fault = None

    events.clear()
    c = C()
    canworker_set_fault = __import__("canworker").Controller._set_fault
    canworker_set_fault(c, "line lost")
    check("a fault emits once", len(events.since(0)[1]) == 1)
    for _ in range(200):
        canworker_set_fault(c, "line lost")
    check("and a fault that persists stays quiet",
          len(events.since(0)[1]) == 1, f"{len(events.since(0)[1])} events")
    events.clear()


def test_panel_profile():
    """A panel that cannot work must be refused by name at boot."""
    print("\npanel: profile validation")

    doc = json.load(open(str(ROOT / "profiles" / "agv-01.json")))

    def refused(name, mutate, needle):
        d = copy.deepcopy(doc)
        mutate(d)
        try:
            config._validate(config._derive(config._parse(d)))
            check(name, False, "NOT rejected")
        except config.ConfigError as e:
            check(name, needle in str(e), str(e)[:70])

    refused("two functions on one channel is refused - it would fire a Reset "
            "edge every time Start is pushed",
            lambda d: d["panel"].update(di_start=2), "distinct")
    refused("a channel past num_di is refused",
            lambda d: d["panel"].update(di_auto=99), "channel in 0..")
    refused("a negative channel is refused",
            lambda d: d["panel"].update(di_reset=-1), "channel in 0..")
    refused("debounce_scans below 1 is refused",
            lambda d: d["panel"].update(debounce_scans=0), "debounce_scans")
    refused("the panel cannot be enabled without the DI scan that feeds it",
            lambda d: d["dio"].update(enabled=False), "never respond")

    # Verified at the panel 2026-09-16 with the ROS panel_node watching the
    # DI image: DI00 is empty, Start is DI01, Reset is DI02, the selector is
    # DI03. The profile shipped as 0/1/2 until then, which read a Reset press
    # as the selector flicking to AUTO and never saw the real selector at all.
    check("the shipped profile matches the wiring: DI01 start, DI02 reset, "
          "DI03 auto",
          (config.PANEL_DI_RESET, config.PANEL_DI_START,
           config.PANEL_DI_AUTO) == (2, 1, 3))
    check("...and the DI names say the same",
          (config.DIO_DI_NAMES[1], config.DIO_DI_NAMES[2], config.DIO_DI_NAMES[3])
          == ("PB Start", "PB Reset", "SS Auto/Manual"))

    refused("a pendant channel on a panel channel is refused",
            lambda d: d["pendant"].update(di_fwd=d["panel"]["di_start"]), "collide")
    refused("two pendant functions on one channel is refused",
            lambda d: d["pendant"].update(di_left=d["pendant"]["di_right"]), "distinct")
    refused("a pendant channel past num_di is refused",
            lambda d: d["pendant"].update(di_rvs=99), "channel in 0..")
    refused("the pendant cannot be enabled without the DI scan that feeds it",
            lambda d: (d["dio"].update(enabled=False),
                       d["panel"].update(enabled=False),
                       d["horn"].update(enabled=False)), "pendant")
    check("the shipped pendant wiring: DI04 fwd, DI05 rvs, DI06 left, DI07 right",
          (config.PENDANT_DI_FWD, config.PENDANT_DI_RVS, config.PENDANT_DI_LEFT,
           config.PENDANT_DI_RIGHT) == (4, 5, 6, 7))
    check("...and the DI names say the same",
          [config.DIO_DI_NAMES[i] for i in (4, 5, 6, 7)]
          == ["Pendant FWD", "Pendant RVS", "Pendant LEFT", "Pendant RIGHT"])


def test_pendant():
    """The jog pendant: debounced direction levels, no tie-down rule."""
    print("\npanel: pendant")
    lv = panel.DebouncedLevels((4, 5, 6, 7), debounce_scans=2)

    def di(fwd=False, rvs=False, left=False, right=False):
        bits = [False] * 16
        bits[4], bits[5], bits[6], bits[7] = fwd, rvs, left, right
        return bits

    check("nothing is believed before the first debounced scan",
          lv.scan(di(fwd=True), True) is None)
    check("a level held at power-on IS reported - no anti-tie-down for a deadman",
          lv.scan(di(fwd=True), True) == (True, False, False, False))
    check("one differing scan is not yet believed; the last level holds",
          lv.scan(di(), True) == (True, False, False, False))
    check("the second identical scan is believed",
          lv.scan(di(), True) == (False, False, False, False))
    check("comms loss reports nothing", lv.scan(di(fwd=True), False) is None)
    check("...and the return of comms needs a fresh debounce",
          lv.scan(di(fwd=True), True) is None)
    check("a short image reports nothing", lv.scan([True] * 4, True) is None)

    I = panel.pendant_intent
    check("nothing held asks for nothing", I(False, False, False, False) == panel.PENDANT_IDLE)
    check("fwd", I(True, False, False, False) == (True, False, False, False))
    check("right", I(False, False, False, True) == (False, False, False, True))
    check("fwd with rvs cancels the axis, the other axis survives",
          I(True, True, True, False) == (False, False, True, False))
    check("left with right cancels the axis",
          I(True, False, True, True) == (True, False, False, False))


def test_panel_loss_removes_manual_authority():
    """R14: a jog must not outlive the panel that authorises it.

    DIO is not a critical health source, and drive() checked only the cached
    mode/armed state - so a browser still re-POSTing an arrow kept the vehicle
    moving with the panel gone. Driven through the real _panel_scan and drive()
    on a controller with a fake DI link: no bus, no socket.
    """
    import canworker
    import events
    print("\npanel: losing the panel ends manual jog")

    class FakeDio:
        def __init__(self):
            self.comms_ok, self.auto = True, False

        def snapshot(self):
            di = [False] * config.DIO_NUM_DI
            di[config.PANEL_DI_AUTO] = self.auto
            return {"di": di, "comms_ok": self.comms_ok}

        def set_coil(self, *a):
            pass

    class Ctl(canworker.Controller):
        def _do_arm(self, mode):
            self._armed, self._mode = True, mode

        def _do_disarm(self, force=False):
            self._armed, self._mode = False, "idle"

        def _drives_ready(self):
            return True

    def refused(c, needle):
        try:
            c.drive("forward")
            return "accepted"
        except RuntimeError as e:
            return "" if needle in str(e) else str(e)

    def settle(c):
        for _ in range(config.PANEL_DEBOUNCE_SCANS + 1):
            c._panel_scan()

    events.clear()
    c = Ctl()
    c._dio = FakeDio()
    settle(c)
    check("a valid panel in MANUAL arms and allows a jog",
          c._armed and c._mode == "manual" and refused(c, "") == "accepted"
          and c._target != (0, 0), f"armed={c._armed} target={c._target}")

    c._dio.comms_ok = False
    c._panel_scan()
    check("one scan without the panel zeroes the held jog",
          c._target == (0, 0) and c._direction == "stop",
          f"target={c._target}")
    check("...and says why", c._last_stop_reason == "panel input lost",
          str(c._last_stop_reason))
    why = refused(c, "panel input lost")
    check("a browser keepalive cannot restart it while the panel is gone",
          why == "" and c._target == (0, 0), why)

    c._dio.comms_ok = True
    settle(c)
    why = refused(c, "release and press again")
    check("the panel coming back (MANUAL) does not replay the held direction",
          why == "" and c._target == (0, 0), why)
    c.halt()                                    # the browser's release
    check("a release followed by a fresh press drives again",
          refused(c, "") == "accepted" and c._target != (0, 0),
          str(c._target))

    c._dio.comms_ok = False
    c._panel_scan()
    c._dio.comms_ok, c._dio.auto = True, True
    settle(c)
    why = refused(c, "not in manual mode")
    check("the panel coming back in AUTO disarms and cannot replay either",
          why == "" and not c._armed and c._target == (0, 0),
          f"{why} armed={c._armed} target={c._target}")

    src = (ROOT / "canworker.py").read_text(encoding="utf-8")
    loop = src[src.index("while not self._stop_evt.is_set():"):]
    check("the panel scan runs before the setpoint is read for the tick",
          loop.index("self._panel_scan()")
          < loop.index("armed, target, deadline, mode = ("))
    events.clear()


TESTS = [
    test_edges_and_tie_down,
    test_selector,
    test_debounce,
    test_comms_loss_cannot_synthesise_a_press,
    test_reset_means_ready,
    test_the_manual_watchdog_does_not_latch,
    test_panel_events_are_edge_only,
    test_panel_loss_removes_manual_authority,
    test_panel_profile,
    test_pendant,
]
