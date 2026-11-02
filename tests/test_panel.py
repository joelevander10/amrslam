"""Operator panel: debounced edges, and the Reset-means-READY state machine."""
import copy
import json

from helpers import ROOT, check

from agv_core import config, panel
from agv_core.panel import AUTO, MANUAL

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

    # auto_when_on=False: the same contact, wired so ON = MANUAL (agv-01).
    sc = panel.PanelScan(R, S, A, 1, auto_when_on=False)
    out = run(sc, [image(), image(auto=True), image()])
    check("inverted: low reads AUTO", out[0].mode == AUTO)
    check("inverted: high reads MANUAL",
          out[1].mode == MANUAL and out[1].mode_changed is True)
    check("inverted: back to low reads AUTO", out[2].mode == AUTO)
    check("inverted: mode() agrees", sc.mode() == AUTO)


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
    # 2026-10-02: remapped to documentation/electrical/01-schematic/io-mapping.md
    # (DI00 start, DI01 reset, DI02 selector, DI03-06 pendant), and the selector
    # reads ON = MANUAL (panel.auto_when_on false), per the operator at the panel.
    check("the shipped profile matches io-mapping.md: DI00 start, DI01 reset, "
          "DI02 selector",
          (config.PANEL_DI_RESET, config.PANEL_DI_START,
           config.PANEL_DI_AUTO) == (1, 0, 2))
    check("...and the DI names say the same",
          (config.DIO_DI_NAMES[0], config.DIO_DI_NAMES[1], config.DIO_DI_NAMES[2])
          == ("PB Start", "PB Reset", "SS Auto/Manual"))
    check("the selector reads ON = MANUAL", config.PANEL_AUTO_WHEN_ON is False)

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
    check("the shipped pendant wiring (io-mapping.md): DI03 fwd, DI04 rvs, "
          "DI05 left, DI06 right",
          (config.PENDANT_DI_FWD, config.PENDANT_DI_RVS, config.PENDANT_DI_LEFT,
           config.PENDANT_DI_RIGHT) == (3, 4, 5, 6))
    check("...and the DI names say the same",
          [config.DIO_DI_NAMES[i] for i in (3, 4, 5, 6)]
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

    intent = panel.pendant_intent
    check("nothing held asks for nothing", intent(False, False, False, False) == panel.PENDANT_IDLE)
    check("fwd", intent(True, False, False, False) == (True, False, False, False))
    check("right", intent(False, False, False, True) == (False, False, False, True))
    check("fwd with rvs cancels the axis, the other axis survives",
          intent(True, True, True, False) == (False, False, True, False))
    check("left with right cancels the axis",
          intent(True, False, True, True) == (True, False, False, False))


# The four controller-driven cases (Reset means ready, the manual watchdog,
# edge-only events, panel loss ends a jog) went with canworker at U11
# (2026-09-21); their ROS equivalents live in amr_base/test (panel_node,
# cmd_mux gating, pendant).
TESTS = [
    test_edges_and_tie_down,
    test_selector,
    test_debounce,
    test_comms_loss_cannot_synthesise_a_press,
    test_panel_profile,
    test_pendant,
]
