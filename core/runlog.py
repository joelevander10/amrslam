"""Per-run log of a blind run. One directory per START/STOP cycle.

Each run produces logs/NNNN-blind_YYYYmmdd_HHMMSS/run.csv, one row per tick, so
a run is a single self-contained thing to copy, attach or delete. Alongside it,
logs/blind_results.csv accumulates one row per completed segment ACROSS runs -
that table is what the encoder measurement campaign reads.

Rows are buffered and flushed about once a second. The run is driven from the
bus thread, which also has to service pushed frames and blocking SDO transfers;
a per-row write would put filesystem latency straight into that path. Nothing
here may raise: a logging problem must never stop the vehicle.

The tape-following run log (auto_*, with its plot) went with the feature. This
is what is left of it, and it is deliberately smaller: the plot renderer, the
PID columns and the station bookkeeping have no meaning without a line to
follow. When a navigation stack lands, its run log wants a new column set here
rather than a revival of the old one.
"""
import csv
import os
import re
import time

import events

# Anchored to the REPO ROOT, not to this file. runlog.py lives in core/, so
# dirname(__file__) would be core/ and every run would quietly land in
# core/logs/ - no error, just numbering restarting at 0001 beside the real runs.
# If this module ever moves again, this line moves with it.
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(_ROOT, "logs")
FLUSH_PERIOD_S = 1.0
# Rows held while the file cannot be written: a minute of 50 Hz ticks. Past it,
# new rows are dropped and counted rather than growing the buffer (and each
# retried write) without limit on the bus thread.
MAX_BUFFER_ROWS = 3000
CSV_NAME = "run.csv"

# Run directories are NNNN-<prefix>_YYYYmmdd_HHMMSS. The sequence number is
# what a human cites ("run 17"); the timestamp is what makes it findable.
SEQ_RE = re.compile(r"^(\d{4,})-")


def next_seq(directory=None):
    """Highest NNNN- prefix already in the log directory, plus one.

    Derived from what is on disk rather than held in a counter file: there is
    no separate state to fall out of step with the directory, and an archived
    or deleted run leaves a gap rather than making the next run collide with a
    name that still exists.

    Never raises. A missing log directory is the first-run case and yields 1.

    LOG_DIR is read at CALL time, not bound as a default argument - a default
    would freeze the module constant at import and silently ignore any later
    reassignment, which is exactly what a test that redirects the log directory
    does.
    """
    directory = LOG_DIR if directory is None else directory
    highest = 0
    try:
        for name in os.listdir(directory):
            m = SEQ_RE.match(name)
            if m:
                highest = max(highest, int(m.group(1)))
    except OSError:                             # no logs/ yet, or unreadable
        pass
    return highest + 1


# An encoder-only blind run: one row per tick, counts and the pose they imply.
# gyro_z_dps is the MLS IMU's yaw rate as last polled - recorded beside the
# encoder heading so the two can be compared after the fact, never used.
BLIND_COLUMNS = [
    "t", "dt", "phase", "segment", "tgt_l_m", "tgt_r_m", "prog_l_m", "prog_r_m",
    "cnt_l", "cnt_r", "n_l", "n_r", "rpm_l", "rpm_r",
    "x_m", "y_m", "heading_deg", "speed_mps", "gyro_z_dps", "loop_ms",
]

# One row per completed blind-run segment, across runs. The measured_* columns
# are left blank for the operator's tape-measure values.
RESULTS_NAME = "blind_results.csv"
RESULT_COLUMNS = [
    "time", "profile", "counts_per_wheel_rev", "log_dir", "segment", "kind",
    "spec", "speed_motor_rpm", "target_left", "target_right", "final_left",
    "final_right", "error_left", "error_right", "encoder_left_m",
    "encoder_right_m", "encoder_distance_m", "encoder_heading_deg",
    "encoder_dx_m", "encoder_dy_m", "commanded_distance_m",
    "commanded_heading_deg", "commanded_dx_m", "commanded_dy_m", "duration_s",
    "measured_distance_m", "measured_heading_deg", "notes",
]


def append_result(row, path=None):
    """Append one row to the cross-run results table. Never raises.

    Returns an error string, or None. Opened and closed per row: a segment ends
    seconds apart at most a few times a minute, and a file left open would lose
    the table's last rows to a power cut at the vehicle.
    """
    path = path or os.path.join(LOG_DIR, RESULTS_NAME)
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        new = not os.path.exists(path)
        with open(path, "a", newline="") as fh:
            out = csv.writer(fh)
            if new:
                out.writerow(RESULT_COLUMNS)
            out.writerow([_fmt(row.get(c)) for c in RESULT_COLUMNS])
        return None
    except Exception as e:                      # noqa: BLE001 - never fatal
        return str(e)


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        return f"{v:.4f}"
    return str(v)


class RunLog:
    """Open on START, close on STOP. Disabled instances are silently inert."""

    def __init__(self, enabled=True, columns=None, prefix="blind"):
        self.enabled = enabled
        self.columns = BLIND_COLUMNS if columns is None else columns
        self.prefix = prefix
        self.dir = None
        self.seq = None           # run number, assigned at open()
        self.path = None          # the CSV; the UI links this
        self.note = ""
        self._fh = None
        self._buf = []
        self._t0 = None
        self._last_flush = 0.0
        self.error = None
        self.dropped = 0          # rows discarded because the buffer was full

    def open(self, note=""):
        if not self.enabled:
            return
        try:
            self.seq = next_seq()
            self.dir = os.path.join(
                LOG_DIR,
                f"{self.seq:04d}-" + time.strftime(f"{self.prefix}_%Y%m%d_%H%M%S"))
            os.makedirs(self.dir, exist_ok=True)
            self.path = os.path.join(self.dir, CSV_NAME)
            self.note = note
            self._fh = open(self.path, "w", buffering=1)
            if note:
                self._fh.write(f"# {note}\n")
            self._fh.write(",".join(self.columns) + "\n")
            self._t0 = time.monotonic()
            self._last_flush = self._t0
            self._buf = []
            self.dropped = 0
        except Exception as e:                  # noqa: BLE001 - never fatal
            self.error = str(e)
            self._fh = None

    def write(self, diag, extra=None):
        if self._fh is None:
            return
        try:
            row = dict(diag)
            if extra:
                row.update(extra)
            row["t"] = time.monotonic() - self._t0
            if len(self._buf) >= MAX_BUFFER_ROWS:
                self.dropped += 1
                if self.dropped == 1:           # once per run, not per tick
                    events.error(
                        f"run log {self.path} cannot be written "
                        f"({self.error or 'write stalled'}) - dropping rows; "
                        f"check free space and permissions on {LOG_DIR}")
            else:
                self._buf.append(",".join(_fmt(row.get(c))
                                          for c in self.columns))
            now = time.monotonic()
            if now - self._last_flush >= FLUSH_PERIOD_S:
                # Stamped BEFORE the attempt, so a failing disk is retried once
                # per flush period rather than on every tick.
                self._last_flush = now
                self._drain()
        except Exception as e:                  # noqa: BLE001
            self.error = str(e)

    def _drain(self):
        if self._fh is None or not self._buf:
            return
        self._fh.write("\n".join(self._buf) + "\n")
        self._buf = []

    def close(self):
        """Flush and close. Never raises."""
        if self._fh is None:
            return
        try:
            self._drain()
        except Exception as e:                  # noqa: BLE001
            self.error = str(e)
        finally:
            # Closed even when the drain failed, or every failed run leaks a
            # descriptor.
            try:
                self._fh.close()
            except Exception as e:              # noqa: BLE001
                self.error = self.error or str(e)
            self._fh = None
            self._buf = []
        if self.dropped:
            events.warn(f"run log {self.path}: {self.dropped} row(s) dropped "
                        f"({self.error})")
