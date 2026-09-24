"""Modbus digital I/O: the scan engine, its health verdict, and the profile."""
import copy
import json
import time

from helpers import ROOT, check

import config


class _Reply:
    def __init__(self, bits, error=False):
        # pymodbus pads to a byte boundary, so a 16-bit ask can come back with
        # more; the driver must slice rather than trust the length.
        self.bits = list(bits)
        self._error = error

    def isError(self):
        return self._error


class FakeClient:
    """Stands in for ModbusTcpClient. No socket, no pymodbus, no hardware."""

    def __init__(self, di=None, do=None):
        self.di = di if di is not None else [False] * 16
        self.do = do if do is not None else [False] * 16
        self.fail_connect = False
        self.fail_read = False
        self.short_read = False
        self.fail_write = False
        self.reads = 0
        self.writes = []                    # (address, value), in order
        self.closed = 0

    def connect(self):
        return not self.fail_connect

    def _bits(self, src, count):
        self.reads += 1
        if self.fail_read:
            return _Reply([], error=True)
        if self.short_read:
            return _Reply(src[:count - 4])
        return _Reply(src + [False] * 3)        # byte-boundary padding

    def read_discrete_inputs(self, address, count=1, device_id=1):
        return self._bits(self.di, count)

    def read_coils(self, address, count=1, device_id=1):
        return self._bits(self.do, count)

    def write_coil(self, address, value, device_id=1):
        self.writes.append((address, bool(value)))
        if self.fail_write:
            return _Reply([], error=True)
        # The module latches it, so the next read_coils sees it. That is what
        # makes the readback a readback rather than a restatement of intent.
        self.do[address] = bool(value)
        return _Reply([bool(value)])

    def close(self):
        self.closed += 1


def _link(fake):
    import dio
    return dio.DioLink(client_factory=lambda: fake)


def test_dio_scan():
    """One scan builds a coherent input image."""
    print("\ndio: the scan")

    di = [False] * 16
    di[2] = di[11] = True
    fake = FakeClient(di=di, do=[True] + [False] * 15)
    link = _link(fake)

    s = link.snapshot()
    check("before any scan there is no image age", s["rx_age_s"] is None)
    check("and comms_ok is false - never scanned is not healthy",
          s["comms_ok"] is False)
    check("the lists are still the right length, so the page can render",
          len(s["di"]) == 16 and len(s["do"]) == 16)

    link._scan()
    s = link.snapshot()
    check("a scan reads the inputs", [i for i, b in enumerate(s["di"]) if b] == [2, 11])
    check("and the coil readback", s["do"][0] is True)
    check("byte-boundary padding is sliced off", len(s["di"]) == 16)
    check("the scan counts", s["scans"] == 1 and s["errors"] == 0)
    check("comms_ok once a scan has landed", s["comms_ok"] is True)
    check("connected", s["connected"] is True)

    # The image must be a copy. A page handler that sorted or cleared this list
    # would otherwise corrupt what every other consumer reads.
    s["di"][0] = True
    s["di_names"][0] = "clobbered"
    check("snapshot returns copies, not the live image",
          link.snapshot()["di"][0] is False)
    check("names are copied too", link.snapshot()["di_names"][0] != "clobbered")


def test_dio_flipped():
    """di_flipped inverts the input image, and only the inputs."""
    print("\ndio: inversion")

    di = [False] * 16
    di[2] = True
    fake = FakeClient(di=di, do=[True] + [False] * 15)
    link = _link(fake)

    was = config.DIO_DI_FLIPPED
    try:
        config.DIO_DI_FLIPPED = True
        link._scan()
        s = link.snapshot()
        check("flipped inverts every input",
              [i for i, b in enumerate(s["di"]) if b] ==
              [i for i in range(16) if i != 2])
        check("outputs are NOT inverted - the flag is about input wiring",
              s["do"][0] is True)
    finally:
        config.DIO_DI_FLIPPED = was


def test_dio_faults():
    """A failed scan must not blank the last known image."""
    print("\ndio: faults")

    di = [False] * 16
    di[5] = True
    fake = FakeClient(di=di)
    link = _link(fake)
    link._scan()

    fake.fail_read = True
    raised = False
    try:
        link._scan()
    except IOError:
        raised = True
    check("a Modbus error raises out of the scan", raised)

    s = link.snapshot()
    check("the last good image survives the failure - blanking it would read "
          "as 'all inputs off', which is a lie", s["di"][5] is True)

    fake.fail_read = False
    fake.short_read = True
    raised = False
    try:
        link._scan()
    except IOError:
        raised = True
    check("a short reply is refused, not zero-padded into the image", raised)

    fake.short_read = False
    fake.fail_connect = True
    link._client = None
    raised = False
    try:
        link._scan()
    except ConnectionError:
        raised = True
    check("a refused connection raises", raised)


def test_dio_health():
    """comms_ok keys on recent SUCCESSFUL scans, not on socket state.

    The opposite of RfidLink, deliberately: the reader is push-only so silence
    is normal, but this module is polled, so silence is the fault.
    """
    print("\ndio: health verdict")

    link = _link(FakeClient())
    link._scan()
    check("a fresh scan is healthy", link.snapshot()["comms_ok"] is True)

    # Age the last good scan past the window without touching the socket.
    with link._lock:
        link._last_ok = time.monotonic() - (config.DIO_SILENT_WARN_S + 0.5)
    s = link.snapshot()
    check("a stale image is unhealthy even though the socket is 'connected'",
          s["comms_ok"] is False and s["connected"] is True)

    was = config.DIO_ENABLED
    try:
        config.DIO_ENABLED = False
        check("disabled reports a None verdict, which health.py reads as "
              "'not in use' rather than as a fault",
              _link(FakeClient()).snapshot()["comms_ok"] is None)
    finally:
        config.DIO_ENABLED = was


def test_dio_health_source():
    """The PullSource wiring: non-critical, so auto stops and manual does not."""
    import health
    print("\ndio: health integration")

    link = _link(FakeClient())
    link._scan()
    mon = health.HealthMonitor([(health.PullSource("dio", link.snapshot),
                                 config.DIO_SILENT_WARN_S)])
    r = mon.evaluate(now=time.monotonic())
    check("a scanning module is healthy", r["sources"]["dio"]["ok"] is True)

    with link._lock:
        link._last_ok = time.monotonic() - 99.0
    r = mon.evaluate(now=time.monotonic())
    check("a dead module is reported", r["sources"]["dio"]["ok"] is False)
    check("as a SENSOR error - auto stops", r["sensor_error"] is True)
    check("not a system error - manual keeps jogging",
          r["system_error"] is False)


def test_dio_events_are_edge_only():
    """The scan runs 20x a second; the event ring holds 200 entries."""
    import events
    print("\ndio: event discipline")

    fake = FakeClient()
    link = _link(fake)

    events.clear()
    link._scan()
    check("connecting emits once", len(events.since(0)[1]) == 1)
    for _ in range(200):                    # 10 s of scans
        link._scan()
    check("a healthy scan does not emit",
          len(events.since(0)[1]) == 1, f"{len(events.since(0)[1])} events")

    link._drop("cable")
    check("losing it emits once", len(events.since(0)[1]) == 2)
    for _ in range(50):
        link._drop("cable")
    check("and staying lost stays quiet",
          len(events.since(0)[1]) == 2, f"{len(events.since(0)[1])} events")
    events.clear()


def test_dio_profile():
    """A profile that would mislabel or flap the module is refused by name."""
    print("\ndio: profile validation")

    doc = json.load(open(str(ROOT / "profiles" / "agv-01.json")))

    def refused(name, mutate, needle):
        d = copy.deepcopy(doc)
        mutate(d["dio"])
        try:
            config._validate(config._derive(config._parse(d)))
            check(name, False, "NOT rejected")
        except config.ConfigError as e:
            check(name, needle in str(e), str(e)[:70])

    refused("a short name list is refused - a 16-lamp page fed 12 labels would "
            "mislabel the rest",
            lambda b: b.update(di_names=[""] * 12), "must match")
    refused("a long name list is refused",
            lambda b: b.update(do_names=[""] * 20), "must match")
    refused("a non-string name is refused",
            lambda b: b.update(di_names=[0] * 16), "list of strings")
    refused("num_di of zero is refused", lambda b: b.update(num_di=0), "num_di")
    refused("a negative base is refused", lambda b: b.update(di_base=-1),
            "di_base")
    refused("a timeout longer than the scan period is refused",
            lambda b: b.update(timeout_s=0.5), "must be below scan_period_s")
    refused("silent_warn below the scan period is refused - one late scan "
            "would read as a fault",
            lambda b: b.update(silent_warn_s=0.01), "must exceed")
    refused("a reconnect wait longer than the health window is refused - one "
            "retry would always trip it",
            lambda b: b.update(reconnect_period_s=5.0), "must be below")
    refused("an out-of-range port is refused", lambda b: b.update(port=0),
            "port")

    d = copy.deepcopy(doc)
    d["dio"]["di_names"] = ["  estop  "] + [""] * 15
    ns = config._parse(d)
    check("names are stripped, so stray spaces cannot misalign the column",
          ns["DIO_DI_NAMES"][0] == "estop")
    # The panel channels are mapped, so they must be labelled - an unnamed
    # lamp on /io next to a button that arms the vehicle is a trap.
    for ch, what in ((config.PANEL_DI_RESET, "reset"),
                     (config.PANEL_DI_START, "start"),
                     (config.PANEL_DI_AUTO, "auto selector")):
        check(f"DI{ch:02d} ({what}) is named on the /io page",
              config.DIO_DI_NAMES[ch] != "", config.DIO_DI_NAMES[ch])


def test_dio_write_phase():
    """A claimed coil is driven to match intent, against the readback."""
    print("\ndio: the write phase")

    fake = FakeClient()
    link = _link(fake)

    link._scan()
    check("an unclaimed module is never written to - this driver energises "
          "nothing it was not asked to", fake.writes == [], str(fake.writes))

    link.set_coil(3, True, hold_s=10.0)
    check("set_coil does no I/O of its own - it runs on the 50 Hz CAN tick",
          fake.writes == [], str(fake.writes))

    link._scan()
    check("the next scan raises the coil",
          fake.writes == [(config.DIO_DO_BASE + 3, True)], str(fake.writes))
    check("and counts it", link.snapshot()["writes"] == 1)

    # The published image is the last thing the MODULE reported, not what we
    # asked for. One scan of lag, in exchange for a write that is acknowledged
    # but not applied showing up as a lamp that never lights.
    check("the image is not patched with our intent",
          link.snapshot()["do"][3] is False)
    link._scan()
    check("the readback catches up on the following scan",
          link.snapshot()["do"][3] is True)

    n = len(fake.writes)
    link._scan()
    link._scan()
    check("a coil already at the right level is not rewritten every scan - "
          "at 20 scans a second that would be a relay chattering",
          len(fake.writes) == n, str(fake.writes))

    # The reboot case, which is the whole reason this diffs against a readback
    # instead of tracking a shadow copy. Nothing detects the reboot.
    fake.do[3] = False
    link._scan()
    check("a module that came back with its coils clear is re-asserted",
          fake.writes[-1] == (config.DIO_DO_BASE + 3, True), str(fake.writes[-1]))

    link.set_coil(3, False, hold_s=10.0)
    link._scan()
    check("lowering the claim lowers the coil",
          fake.writes[-1] == (config.DIO_DO_BASE + 3, False))

    try:
        link.set_coil(config.DIO_NUM_DO, True, hold_s=1.0)
        check("a channel off the end of the module is refused", False,
              "accepted!")
    except ValueError:
        check("a channel off the end of the module is refused", True)


def test_dio_command_expires():
    """A claim is renewed or it falls low. The horn cannot outlive its caller."""
    print("\ndio: command expiry")

    fake = FakeClient()
    link = _link(fake)
    link.set_coil(0, True, hold_s=0.05)
    link._scan()
    check("the coil is raised while the claim is fresh",
          fake.writes[-1] == (config.DIO_DO_BASE + 0, True))
    link._scan()
    check("the module is holding it", fake.do[0] is True)

    # The failure this exists for: the CAN tick dies mid-run. Nothing calls
    # set_coil again, and nothing has to notice - the deadline does the work.
    time.sleep(0.06)
    link._scan()
    check("an unrenewed claim is driven LOW rather than left standing - a horn "
          "that outlives its control loop is how operators learn to ignore "
          "horns", fake.writes[-1] == (config.DIO_DO_BASE + 0, False))
    check("and the snapshot says so, so the page agrees with the module",
          link.snapshot()["commanded"]["0"] is False)

    # Expiry lowers the coil; it does not release it. The claim stays, so the
    # channel goes on being held low rather than reverting to whoever wrote it
    # last - which for an output is the difference between off and unknown.
    n = len(fake.writes)
    link._scan()
    check("an expired claim keeps holding the coil low, without rewriting it",
          len(fake.writes) == n and "0" in link.snapshot()["commanded"])

    link.set_coil(0, True, hold_s=10.0)
    link._scan()
    check("renewing it raises the coil again",
          fake.writes[-1] == (config.DIO_DO_BASE + 0, True))


def test_dio_write_failure_is_not_silent():
    """A write that fails is a dropped connection, exactly like a failed read."""
    print("\ndio: write faults")

    fake = FakeClient()
    link = _link(fake)
    link.set_coil(1, True, hold_s=10.0)
    fake.fail_write = True

    raised = False
    try:
        link._scan()
    except IOError:
        raised = True
    check("a refused write raises out of the scan rather than being assumed "
          "to have landed", raised)
    check("the coil's state is not counted as written",
          link.snapshot()["writes"] == 0)

    # The scan that raised also never published, so comms_ok stays false and
    # the health source reports the module as lost - the same verdict a module
    # that stopped answering reads would get.
    check("a scan that could not write is not a successful scan",
          link.snapshot()["comms_ok"] is False)

    fake.fail_write = False
    link._scan()
    check("it recovers on its own once the write goes through",
          fake.do[1] is True and link.snapshot()["comms_ok"] is True)


def test_dio_stop_de_energises():
    """Coils LATCH in hardware. Exiting must not leave the horn sounding."""
    print("\ndio: shutdown")

    fake = FakeClient()
    link = _link(fake)
    link.set_coil(0, True, hold_s=10.0)
    link._scan()
    check("the coil is up before we stop", fake.do[0] is True)

    # stop() waits on the scan thread, so drive the scans a real one would.
    import threading
    stopping = threading.Thread(target=lambda: link.stop(quiesce_s=2.0),
                                daemon=True)
    link._thread = stopping             # stop() only quiesces a live link
    stopping.start()
    deadline = time.monotonic() + 2.0
    while not link._quiet.is_set() and time.monotonic() < deadline:
        link._scan()
        time.sleep(0.005)
    stopping.join(timeout=2.0)

    check("stop() drives the claim low before closing the socket",
          fake.do[0] is False, str(fake.writes))
    check("and waits for the READBACK to prove it, not for the write to be "
          "acknowledged", link._quiet.is_set())
    check("the socket is closed afterwards", fake.closed >= 1)


TESTS = [
    test_dio_scan,
    test_dio_flipped,
    test_dio_faults,
    test_dio_write_phase,
    test_dio_command_expires,
    test_dio_write_failure_is_not_silent,
    test_dio_stop_de_energises,
    test_dio_health,
    test_dio_health_source,
    test_dio_events_are_edge_only,
    test_dio_profile,
]
