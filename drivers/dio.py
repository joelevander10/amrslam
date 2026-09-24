"""Digital I/O over Modbus TCP (16 in / 16 out) on its own thread.

The module answers at DIO_IP:502. Discrete inputs and coils both start at
address 0, and device_id is ignored - 1, 0 and 255 all reply - so the id in the
profile is documentation rather than addressing.

WHY A THREAD AND NOT THE CONTROL TICK
-------------------------------------
A read measures 2.0 ms on this link, so a scan (DI then DO) is ~4 ms. The CAN
control tick has a 20 ms budget, so scanning there would spend a fifth of it on
network I/O in the good case - and a network stall, which no timeout makes
instant, would blow the tick outright. Same reasoning as rfid.py: own thread,
own socket, publish an image the control loop reads without ever blocking.

WHAT "LIKE A PLC" ACTUALLY BUYS
------------------------------
Not the loop - the INPUT IMAGE. Every consumer sees one coherent set of bits
sampled at one instant, instead of reading a live socket at whatever moment it
happens to ask. snapshot() under a lock is exactly that, and it is the same
discipline canworker._branch_scan() already follows for the RFID reader.

The write phase is now present, and it is the ONLY thing this software
energises: the horn-and-lights coil, raised while motion is commanded. It sits
after the reads for the reason a PLC puts it there - the outputs written in a
scan are the ones decided from the inputs read in that same scan, so a coil can
never be driven from an image nobody has refreshed.

Three rules make a write path tolerable on a 150 kg vehicle, and all three are
below rather than in the caller:

  * The DECISION is not here. set_coil() takes a value someone else computed
    behind an arm-state interlock; this module owns the socket, not the policy.
    There is still no click handler on a web page, and no endpoint to call one.
  * A command EXPIRES. It carries a hold, and the caller renews it every tick.
    A control thread that dies stops renewing, and the coil falls low at the
    next scan instead of staying energised until somebody kills the process.
  * We write the DIFFERENCE against the readback, not a shadow copy. A module
    that rebooted comes back with its coils clear, and the very next scan sees
    the mismatch and re-asserts. Nothing needs to notice the reboot.

HEALTH IS THE OPPOSITE OF THE RFID READER'S
-------------------------------------------
RfidLink._comms_ok() keys on CONNECTION state and deliberately ignores data
flow, because the reader is push-only and silence between stations is normal.
Here silence IS the fault: we poll every scan_period_s, so a scan that has not
succeeded recently means the module, the cable or the switch has gone, whatever
the socket believes. Keying on the socket would report a half-open connection as
healthy for as long as the OS kept it.

TOPOLOGY
--------
The RFID reader is daisy-chained THROUGH this module's second port, so both sit
behind one cable to enp2s0. This module failing takes the reader with it, and
RfidLink.carrier() cannot see that - the PC's link partner is this module, so
carrier stays up. Two sources reporting at once is the signature of the shared
cause; either alone is the device itself.
"""
import threading
import time

import config
import events


class DioLink:
    """Owns the Modbus socket on its own thread. start() once, read snapshot()."""

    def __init__(self, client_factory=None):
        # Injectable so the tests can drive a fake without a network or a
        # pymodbus import - the same reason RfidLink takes a codec.
        self._client_factory = client_factory or self._default_client
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._client = None

        self._connected = False
        self._di = [False] * config.DIO_NUM_DI
        self._do = [False] * config.DIO_NUM_DO
        self._last_ok = None                # monotonic, last SUCCESSFUL scan
        self._scans = 0
        self._errors = 0
        self._writes = 0
        # channel -> [value, expires_at]. An entry is created by the first
        # set_coil() for that channel and then kept: once this software has
        # claimed a coil it goes on asserting it, and an expired claim asserts
        # LOW rather than releasing the channel. Releasing it would leave the
        # last value standing, which is the one outcome a hold exists to stop.
        self._want = {}
        # Set by stop() to force every claim low, cleared onto _quiet once a
        # readback proves it landed. See stop().
        self._quiesce = threading.Event()
        self._quiet = threading.Event()
        self._detail = "disabled" if not config.DIO_ENABLED else "starting"

    # ---- public ----------------------------------------------------------

    @staticmethod
    def _default_client():
        # Imported lazily: pymodbus costs ~40 ms and is dead weight in a process
        # running with dio.enabled false.
        from pymodbus.client import ModbusTcpClient
        return ModbusTcpClient(config.DIO_IP, port=config.DIO_PORT,
                               timeout=config.DIO_TIMEOUT_S)

    def start(self):
        if not config.DIO_ENABLED or self._thread:
            return
        self._thread = threading.Thread(target=self._run, name="dio",
                                        daemon=True)
        self._thread.start()

    def stop(self, quiesce_s=1.0):
        """De-energise anything we claimed, then shut the socket.

        The wait is the point. Coils LATCH in the module: a process that simply
        exits leaves the horn sounding until somebody cuts the power, and an
        alarm that outlives the thing it was warning about is how operators
        learn to ignore alarms. So the scan thread is asked to drive every
        claim low and given a bounded moment to prove it did.

        Bounded, and best-effort by design: if the module is already
        unreachable there is nothing to write and nothing to wait for, and a
        shutdown path is the last place to hang. The renewal deadline in
        set_coil() is the backstop for every exit that does not come through
        here at all.
        """
        if self._thread and self._thread.is_alive():
            with self._lock:
                claimed = bool(self._want)
            if claimed:
                self._quiesce.set()
                self._quiet.wait(quiesce_s)
        self._stop.set()
        self._close()

    def set_coil(self, channel, value, hold_s):
        """Claim an output. Renew within hold_s or it falls low on its own.

        Called from the CAN tick at 50 Hz, so it does no I/O and never blocks:
        it records an intent that the scan thread acts on within one
        scan_period_s. That lag is why the horn is driven from the COMMANDED
        setpoint rather than from measured motion - it leaves the warning
        ahead of the wheels rather than behind them.

        hold_s is a deadline, not a duration to sound for. Every call pushes it
        out; a caller that stops calling drops the coil. See config.HORN_HOLD_S.
        """
        if not 0 <= channel < config.DIO_NUM_DO:
            raise ValueError(f"coil {channel} outside 0..{config.DIO_NUM_DO - 1}")
        with self._lock:
            self._want[channel] = [bool(value), time.monotonic() + hold_s]

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            age = (now - self._last_ok) if self._last_ok else None
            return {
                "enabled": config.DIO_ENABLED,
                "connected": self._connected,
                "comms_ok": self._comms_ok(age),
                # Copies. A caller that mutated the live lists would corrupt
                # the image every other consumer is reading.
                "di": list(self._di),
                "do": list(self._do),
                "di_names": list(config.DIO_DI_NAMES),
                "do_names": list(config.DIO_DO_NAMES),
                "rx_age_s": age,
                "scans": self._scans,
                "errors": self._errors,
                "writes": self._writes,
                # Which coils this software is driving, and to what. The page
                # needs it to tell "off because nothing asked" apart from "off
                # because we are holding it off" - on an output those look
                # identical and mean opposite things. Keys are strings: this
                # goes out as JSON.
                "commanded": {str(ch): self._resolve(v, until, now)
                              for ch, (v, until) in self._want.items()},
                "scan_period_s": config.DIO_SCAN_PERIOD_S,
                "detail": self._detail,
            }

    # ---- internals -------------------------------------------------------

    def _resolve(self, value, until, now):
        """What a claim is worth at `now`: itself, or LOW once it has expired.

        Caller holds the lock.
        """
        if self._quiesce.is_set():
            return False
        return bool(value) and now < until

    def _desired(self, now):
        """channel -> the level we intend to hold right now."""
        with self._lock:
            return {ch: self._resolve(v, until, now)
                    for ch, (v, until) in self._want.items()}

    def _comms_ok(self, age):
        """Recent SUCCESSFUL scans, not socket state. See the module docstring.

        Caller holds the lock.
        """
        if not config.DIO_ENABLED:
            return None                     # not a fault; simply not in use
        if age is None:
            return False                    # never completed a scan
        return age <= config.DIO_SILENT_WARN_S

    def _set_detail(self, text):
        with self._lock:
            self._detail = text

    def _close(self):
        client, self._client = self._client, None
        if client is not None:
            try:
                client.close()
            except Exception:               # noqa: BLE001 - teardown only
                pass

    def _drop(self, why):
        """Lose the connection, once, on the edge."""
        with self._lock:
            was, self._connected = self._connected, False
            self._detail = why
        self._close()
        if was:
            events.warn(f"DIO lost: {why}")

    def _run(self):
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._scan()
            except Exception as e:          # noqa: BLE001 - thread must survive
                with self._lock:
                    self._errors += 1
                self._drop(f"{type(e).__name__}: {e}")
                if self._stop.wait(config.DIO_RECONNECT_PERIOD_S):
                    return

            # Pace on elapsed time, not a flat sleep, so a slow scan does not
            # push the period out and a fast one does not busy-wait.
            rest = config.DIO_SCAN_PERIOD_S - (time.monotonic() - started)
            if self._stop.wait(max(0.0, rest)):
                return

    def _scan(self):
        """One PLC scan: read inputs, read the coil readback, publish."""
        if self._client is None:
            self._client = self._client_factory()
        if not self._client.connect():
            raise ConnectionError(f"no answer at {config.DIO_IP}:{config.DIO_PORT}")

        di = self._read_bits(self._client.read_discrete_inputs,
                             config.DIO_DI_BASE, config.DIO_NUM_DI, "inputs")
        # Read the coils back rather than trusting a shadow copy. Nothing here
        # writes them today, but a readback is what would make a FAILED write
        # visible rather than showing the value we wished for.
        do = self._read_bits(self._client.read_coils,
                             config.DIO_DO_BASE, config.DIO_NUM_DO, "coils")

        if config.DIO_DI_FLIPPED:
            di = [not b for b in di]

        # --- write phase, after the reads. See the module docstring. --------
        wrote = self._write_phase(do)

        now = time.monotonic()
        with self._lock:
            was = self._connected
            self._di, self._do = di, do
            self._connected = True
            self._last_ok = now
            self._scans += 1
            self._writes += wrote
            self._detail = f"{config.DIO_IP}:{config.DIO_PORT}"
        # Edge only. This runs every scan_period_s; emitting per scan would
        # empty the 200-entry event ring in ten seconds.
        if not was:
            events.info(f"DIO connected {config.DIO_IP}:{config.DIO_PORT}")

    def _write_phase(self, readback):
        """Drive claimed coils to match intent. Returns how many were written.

        Difference against the READBACK, which is what makes this idempotent
        and self-healing: a coil already at the right level costs nothing, and
        a module that lost power comes back with its coils clear and is
        corrected on the very next scan without anything having to detect the
        reboot.

        `readback` is deliberately NOT patched with what we just wrote. The
        published image stays the last thing the module actually reported, so a
        write the module acknowledges but does not apply shows up as a lamp
        that never changes - which is the failure worth seeing. The cost is one
        scan of lag on the lamp, against a horn whose real evidence is that it
        is audible.
        """
        wrote = 0
        for ch, want in self._desired(time.monotonic()).items():
            if ch < len(readback) and readback[ch] == want:
                continue
            r = self._client.write_coil(config.DIO_DO_BASE + ch, want,
                                        device_id=config.DIO_DEVICE_ID)
            if r.isError():
                # Same treatment as a failed read: raise, so _run() counts the
                # error and drops the connection. A write that silently failed
                # would leave the horn's state a matter of opinion.
                raise IOError(f"coil {ch} <- {want}: {r}")
            wrote += 1

        if self._quiesce.is_set() and not wrote:
            # Nothing left to correct, so the readback already shows every
            # claim low. Only now is stop() free to close the socket.
            self._quiet.set()
        return wrote

    def _read_bits(self, fn, address, count, what):
        r = fn(address, count=count, device_id=config.DIO_DEVICE_ID)
        if r.isError():
            raise IOError(f"{what} read at {address}: {r}")
        bits = list(r.bits[:count])
        if len(bits) != count:
            # pymodbus pads to a byte boundary, so a short reply means the
            # module answered for fewer channels than the profile claims.
            raise IOError(f"{what}: asked {count}, got {len(bits)}")
        return [bool(b) for b in bits]
