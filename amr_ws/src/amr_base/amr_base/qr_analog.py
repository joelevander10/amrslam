"""CKDA08ETH analog outputs (AMR QR wheel speed voltages) over Modbus TCP.

Holding register ao_register_base + channel, value = volts x ao_counts_per_volt
(1000: millivolts, as analog_digital.CKDA08ETH_Controller wrote them). Called from the
wheel-loop thread only; blocking, bounded by ao_timeout_s.

  * Both wheels in ONE write_registers when their channels are adjacent, so the two
    setpoints always come from the same tick.
  * Written when a value changed by at least one count, and re-sent every refresh_s
    regardless - a module that rebooted to 0 V is corrected without anyone noticing.
  * A failed write drops the connection and marks the link not ok; reconnects are paced
    by reconnect_s so a dead module costs one timeout per reconnect, not one per tick.
  * zero() is the exit path: best effort, never raises.

*** The module keeps its last value when the PC goes away. *** See config.py qr_base.*.
"""

from __future__ import annotations

import time


class AnalogOut:
    def __init__(
        self,
        ip: str,
        port: int,
        device_id: int,
        timeout_s: float,
        register_base: int,
        counts_per_volt: float,
        full_scale_v: float,
        ch_left: int,
        ch_right: int,
        refresh_s: float = 0.5,
        reconnect_s: float = 0.5,
        client_factory=None,
        clock=time.monotonic,
    ) -> None:
        self.ip, self.port, self.device_id, self.timeout = ip, port, device_id, timeout_s
        self.base, self.cpv, self.full_scale = register_base, counts_per_volt, full_scale_v
        self.ch = (ch_left, ch_right)
        self.refresh_s, self.reconnect_s = refresh_s, reconnect_s
        self._factory = client_factory or self._default_client
        self.clock = clock
        self._client = None
        self._sent: tuple[int, int] | None = None
        self._t_sent = 0.0
        self._t_retry = 0.0
        self.ok = False
        self.t_ok: float | None = None
        self.errors = 0
        self.writes = 0
        self.detail = "not connected"
        self.volts = (0.0, 0.0)

    def _default_client(self):
        from pymodbus.client import ModbusTcpClient  # noqa: PLC0415 - lazy, like dio.py

        return ModbusTcpClient(self.ip, port=self.port, timeout=self.timeout)

    def counts(self, volts: float) -> int:
        v = max(0.0, min(self.full_scale, float(volts)))
        return int(round(v * self.cpv))

    def write(self, v_left: float, v_right: float) -> bool:
        """Apply both voltages. Returns the link state after this call."""
        now = self.clock()
        want = (self.counts(v_left), self.counts(v_right))
        if self.ok and want == self._sent and now - self._t_sent < self.refresh_s:
            return True
        if self._client is None:
            if now < self._t_retry:
                return False
            self._client = self._factory()
        try:
            if not self._client.connect():
                raise ConnectionError(f"no answer at {self.ip}:{self.port}")
            (cl, cr), (wl, wr) = self.ch, want
            if abs(cl - cr) == 1:
                first = min(cl, cr)
                vals = [wl, wr] if cl == first else [wr, wl]
                r = self._client.write_registers(self.base + first, vals, device_id=self.device_id)
                self._check(r, f"registers {self.base + first}..{self.base + first + 1}")
            else:
                for ch, val in ((cl, wl), (cr, wr)):
                    r = self._client.write_register(self.base + ch, val, device_id=self.device_id)
                    self._check(r, f"register {self.base + ch}")
        except Exception as e:  # noqa: BLE001 - a condition, reported and retried
            self.errors += 1
            self._drop(f"{type(e).__name__}: {e}", now)
            return False
        self._sent, self._t_sent = want, now
        self.volts = (want[0] / self.cpv, want[1] / self.cpv)
        self.writes += 1
        self.ok, self.t_ok = True, now
        self.detail = f"{self.ip}:{self.port}"
        return True

    @staticmethod
    def _check(r, what: str) -> None:
        if r is None or r.isError():
            raise OSError(f"{what}: {r}")

    def _drop(self, why: str, now: float) -> None:
        self.ok = False
        self.detail = why
        self._sent = None
        self._t_retry = now + self.reconnect_s
        client, self._client = self._client, None
        if client is not None:
            try:
                client.close()
            except Exception:  # noqa: BLE001
                pass

    def zero(self) -> bool:
        """Exit path: 0 V on both channels, best effort."""
        self._t_retry = 0.0
        try:
            return self.write(0.0, 0.0) and self._sent == (0, 0)
        except Exception:  # noqa: BLE001
            return False

    def close(self) -> None:
        self._drop("closed", self.clock())
