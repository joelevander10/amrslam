"""LineTrack message -> the sensor dict the ported engine eats, plus freshness.

The engine was written against `canworker._sensor_json()`, a plain dict:

    {"tracks": [{"index": 1, "pos_mm": -30, "width": 10}, ...],
     "has_track": True}

and "no line" is `{"tracks": [], "has_track": False}`. `_read_error` reads only
`"tracks"`, so the shape is small; this module builds it from the ROS message
and answers the two questions the node must ask before every tick.

*** Freshness and RATE are different questions, and both can fail. ***
`sensor_timeout_s` catches a stream that has STOPPED. It cannot see one that is
fresh but slow, and that is the case this vehicle actually has: the MLS SDO
fallback measured 9.8 Hz on 2026-09-19 against a 50 Hz PID. Every sample is
recent, every staleness check passes, and the follower is steering on a
quarter of the information the gains were tuned for. `line_min_track_hz` is
the second gate, and `source == "sdo"` is a third - mls_track's own docstring
says the SDO path is "too slow and too skewed for control".
"""
import collections


class TrackReader:
    """Holds the last sample, converts it, and judges whether it may be used."""

    # Enough samples to measure a rate over ~0.5 s at 50 Hz without a long
    # memory: a rate estimate that lags is worse than none, because the gate
    # it feeds is meant to stop the vehicle promptly.
    _WINDOW = 25

    def __init__(self, timeout_s, min_hz, accept_sdo=False):
        self.timeout_s = float(timeout_s)
        self.min_hz = float(min_hz)
        self.accept_sdo = bool(accept_sdo)
        self._stamps = collections.deque(maxlen=self._WINDOW)
        self.last = None          # the most recent LineTrack
        self.last_rx_mono = None  # when THIS process received it
        self.seq = None
        self.drops = 0

    # -- intake ------------------------------------------------------------
    def update(self, msg, now_mono):
        """Take a LineTrack. `now_mono` is time.monotonic() at receipt."""
        if self.seq is not None and msg.seq != ((self.seq + 1) & 0xFFFFFFFF):
            # Best-effort transport: a gap is real information, not noise.
            self.drops += 1
        self.seq = int(msg.seq)
        self.last = msg
        self.last_rx_mono = now_mono
        self._stamps.append(now_mono)

    # -- judgement ---------------------------------------------------------
    def age_s(self, now_mono):
        """Seconds since the sample was DECODED, not since it was published.

        sample_age_s is the decode-to-publish gap the driver measured; the rest
        is transport and scheduling on this side. Adding them is the only
        honest answer to "how old is what I am steering on".
        """
        if self.last is None or self.last_rx_mono is None:
            return None
        return float(self.last.sample_age_s) + (now_mono - self.last_rx_mono)

    def hz(self):
        """Measured arrival rate, or None until there are enough samples."""
        if len(self._stamps) < 3:
            return None
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0 else None

    def usable(self, now_mono):
        """(ok, cause). cause is a LineState.hold_cause value when not ok."""
        if self.last is None:
            return False, "track"
        age = self.age_s(now_mono)
        if age is None or age > self.timeout_s:
            return False, "track"
        if not self.accept_sdo and self.last.source != "tpdo":
            # Not a fault of the tape: the sensor is in the wrong mode.
            return False, "rate"
        # An unmeasured rate is not a good rate: three samples at 100 Hz is
        # 20 ms of waiting, and a stream that never gets there was never
        # usable for control.
        rate = self.hz()
        if rate is None or rate < self.min_hz:
            return False, "rate"
        return True, ""

    # -- conversion --------------------------------------------------------
    def sensor(self):
        """The engine's dict. Empty tracks when there is nothing to follow."""
        m = self.last
        if m is None:
            return NO_TRACK
        tracks = [
            {"index": i + 1, "pos_mm": int(m.lcp_mm[i]), "width": 10}
            for i in range(3)
            if m.valid[i]
        ]
        # line_good is the sensor's own verdict on signal strength. A populated
        # LCP with line_good false is a reading the engine should not act on,
        # and dropping the tracks is how it is told - `_read_error` treats an
        # empty list exactly as "no line", which is what this is.
        if not m.line_good:
            tracks = []
        # nlcp drives the branch ladder's ForkPassed contact and track_level the
        # U-turn's reacquisition test (2026-10-02, mission engine).
        return {"tracks": tracks, "has_track": bool(tracks),
                "nlcp": int(m.nlcp), "track_level": int(m.track_level)}

    def nlcp(self):
        return 0 if self.last is None else int(self.last.nlcp)


NO_TRACK = {"tracks": [], "has_track": False, "nlcp": 0, "track_level": None}
