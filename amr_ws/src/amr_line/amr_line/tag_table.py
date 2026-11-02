"""The mission's RFID tag table at run time: which reads count, and where the vehicle is.

Pure and clock-fed: no ROS, no I/O, no clock of its own. Two pieces:

TagTable - one per run (TapeRun owns it). Decides whether a read of a known tag
is ACTED ON, given the row's ignore window and the job's destination:

    ignore window   after an acted-on read the same tag is ignored for ignore_s.
                    A stop's window restarts at departure: a vehicle parked for
                    ten minutes has outlived any window measured from the read.
                    High-zone tags are windowed like any other, with at most
                    1 s (agv_core.mission): one corner reads the outer id twice.
    destination     a destination stop applies only to the job's destination,
                    and only until it has been served once - the same MRU tags are
                    passed again on the way back from the U-turn. Reaching the
                    U-turn with the destination still pending means its read was
                    missed: it is marked missed and never served from the return
                    leg (amr_line.tape_run).

TagOdometer - one per layer (FollowJob owns it; survives runs and mission
changes). The last tag encountered and the wheel travel since. It answers "is
the vehicle at Home": the reader reports a tag only while it is in the antenna
field, and a measured stop parks the vehicle stop_distance_m past it, so "the
reader sees 0010 now" is not the test. The test is: the last encounter was the
Home tag, the wheels have travelled no more than the window since, and nothing
has broken the chain - an RFID reconnect, a link drop, wheel feedback missing
for longer than a grace, or another tag read. Any of those means the position is
no longer proven and the operator jogs back over the tag.
"""

import math

from amr_line import uturn

# Home window beyond the home stop's own stop distance: the read zone's lead (the
# encounter is generated as the tag enters the field, before the antenna is over
# it) plus the stop's overshoot. Generous on purpose; the window only has to
# separate "parked at Home" from "somewhere else on the line".
HOME_MARGIN_M = 0.3
# Wheel feedback may drop a sample (BEST_EFFORT); longer than this without counts
# and the travel since the tag is unknown.
COUNTS_GRACE_S = 1.0


class TagTable:
    def __init__(self, mission, destination=None):
        self.rows = dict(mission["TAGS"]) if mission else {}
        self.home = mission["HOME"] if mission else None
        self.destinations = list(mission["DESTINATIONS"]) if mission else []
        self.destination = destination or None
        self.reached = False        # the destination stop has begun
        self.missed = False         # the U-turn came first: its read was missed
        self._window_from = {}      # tag -> time its ignore window (re)started

    def row(self, tag):
        return self.rows.get(tag)

    def window_left(self, now, tag):
        """Seconds left in this tag's ignore window, 0 when it counts again."""
        row, since = self.rows.get(tag), self._window_from.get(tag)
        if row is None or since is None:
            return 0.0
        return max(0.0, row["ignore_s"] - (now - since))

    def acted(self, now, tag):
        self._window_from[tag] = now

    def departed(self, now, tag):
        """Leaving a stop restarts its window: measured from the read it has expired."""
        if tag in self.rows:
            self._window_from[tag] = now

    def stop_applies(self, row):
        """(stops, why_not) for a stop row under this job."""
        role = row["role"]
        if role in ("home", "always"):
            return True, ""
        if self.destination is None:
            return False, "no destination for this run"
        if row["label"] != self.destination:
            return False, f"not the destination ({self.destination})"
        if self.missed:
            return False, f"{self.destination} was missed outbound; not served from the return leg"
        if self.reached:
            return False, f"{self.destination} already served"
        return True, ""

    def note_u_turn(self):
        """The U-turn tag was acted on. Returns the destination if it was missed."""
        if self.destination is not None and not self.reached and not self.missed:
            self.missed = True
            return self.destination
        return None

    def result(self):
        """One line for the operator when the run ends at Home."""
        if self.destination is None:
            return "run complete"
        if self.reached:
            return f"{self.destination} served"
        if self.missed:
            return f"{self.destination} MISSED - its tag was not read before the U-turn"
        return f"{self.destination} NOT reached"


class TagOdometer:
    def __init__(self, counts_grace_s=COUNTS_GRACE_S):
        self.counts_grace_s = float(counts_grace_s)
        self.tag = None             # last tag encountered, None when the chain broke
        self.travel_m = 0.0         # wheel travel since that encounter
        self.why = "no tag read yet"
        self._cursor = None
        self._generation = None
        self._ref = None            # counts at the last integration
        self._counts_t = None       # last time counts were available

    def _break(self, why):
        if self.tag is not None:
            self.why = why
        self.tag, self.travel_m, self._ref = None, 0.0, None

    def update(self, now, rfid, counts, counts_per_rev):
        rfid = rfid or {}
        seq = int(rfid.get("encounter_seq", 0))
        gen = rfid.get("generation", 0)
        if self._generation is not None and gen != self._generation:
            self._break("RFID link reconnected since the tag was read")
        self._generation = gen
        if not rfid.get("comms_ok", False):
            self._break("RFID link down")
            self._cursor = seq
            return
        if self._cursor is None:
            # First sight: history in the buffer is not evidence of where we are now.
            self._cursor = seq
        for n, tag in rfid.get("encounters", ()):
            if n > self._cursor:
                self.tag, self.travel_m, self._ref = tag, 0.0, None
                self.why = ""
        self._cursor = max(self._cursor, seq)
        self._integrate(now, counts, counts_per_rev)

    def _integrate(self, now, counts, counts_per_rev):
        if counts is None or not counts_per_rev:
            if self.tag is not None and self._counts_t is not None \
                    and now - self._counts_t > self.counts_grace_s:
                self._break("wheel feedback lost since the tag was read")
            return
        self._counts_t = now
        if self.tag is None:
            return
        if self._ref is not None:
            from agv_core import config as vehicle  # noqa: PLC0415

            m_per_count = math.pi * vehicle.WHEEL_DIA_M / counts_per_rev
            dl = abs(uturn.counts_delta(counts[0], self._ref[0]))
            dr = abs(uturn.counts_delta(counts[1], self._ref[1]))
            self.travel_m += 0.5 * (dl + dr) * m_per_count
        self._ref = tuple(counts)

    def at(self, tag, window_m):
        """(True, "") when `tag` is the last encounter within window_m of travel."""
        if self.tag is None:
            return False, self.why or "no tag read yet"
        if self.tag != tag:
            return False, f"last tag read was {self.tag}, not {tag}"
        if self._ref is None:
            return False, "no wheel feedback since the tag was read"
        if self.travel_m > window_m:
            return False, f"moved {self.travel_m:.2f} m since tag {tag}"
        return True, ""


def home_window_m(home_row):
    return home_row["stop_distance_m"] + HOME_MARGIN_M
