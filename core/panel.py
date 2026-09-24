"""Operator panel: debounced edge detection over the digital input image.

Three devices on the DIO module drive the vehicle's state:

    DI_RESET   momentary pushbutton, rising edge
    DI_START   momentary pushbutton, rising edge
    DI_AUTO    maintained selector, level. HIGH = AUTO, low = manual

This module reports what the operator ASKED FOR - a Reset edge, a Start edge,
the selector position. It deliberately does not decide what the vehicle should
do about it, because that depends on state only the controller holds. Same
division as branch.py: the mechanism here, the policy there.

Free of config, of any clock, and of I/O, so the whole thing is a table-driven
unit test rather than an integration test.

THREE WAYS A PANEL SCAN GOES WRONG, AND WHAT STOPS THEM
-------------------------------------------------------
1. A HELD BUTTON REPEATS. The image is sampled 20x a second, so a normal
   half-second press covers ten scans. Only the rising edge counts.

2. A BUTTON HELD AT POWER-ON RUNS THE VEHICLE. The first accepted image is
   captured as the baseline and produces no edges at all - so a taped-down
   Start, or a selector already sitting in AUTO, cannot act until it is
   released and pressed again. This is the classic anti-tie-down rule and it is
   the reason `_stable` starts as None rather than as all-zeros.

3. A RECONNECT SYNTHESISES A PRESS. If the DIO link drops while a button is
   down and returns after it is released - or vice versa - comparing the new
   image against the pre-outage one invents an edge that nobody produced. So
   edges are evaluated only while comms are good, and the transition back INTO
   good comms re-baselines instead of comparing. The RFID reader has the same
   hazard and canworker handles it the same way, keyed on tags_seen.

Contact bounce is largely handled by the scan period itself - 50 ms is longer
than a pushbutton bounces - but `debounce_scans` requires a level to read the
same on N consecutive scans before it is believed, which also rejects a single
corrupted Modbus response.
"""
from collections import namedtuple

AUTO, MANUAL = "auto", "manual"

# valid        - there is a trusted, debounced image this scan
# reset/start  - rising edge this scan
# mode         - selector position, or None before the first baseline
# mode_changed - the selector moved on this scan
PanelIntent = namedtuple("PanelIntent",
                         "valid reset start mode mode_changed")

IDLE_INTENT = PanelIntent(valid=False, reset=False, start=False,
                          mode=None, mode_changed=False)


class PanelScan:
    """Debounced edge detector over three channels of the DI image."""

    def __init__(self, reset_ch, start_ch, auto_ch, debounce_scans=2):
        self.channels = (int(reset_ch), int(start_ch), int(auto_ch))
        self.debounce_scans = max(1, int(debounce_scans))
        self._stable = None         # last accepted sample; None = not baselined
        self._candidate = None      # sample being debounced
        self._count = 0
        self._comms = False

    def reset(self):
        """Forget the baseline. The next accepted sample becomes the new one
        and produces no edges."""
        self._stable = None
        self._candidate = None
        self._count = 0

    def scan(self, di, comms_ok):
        """One scan of the input image. Returns a PanelIntent.

        di must be the whole DI list; the channels are indexed out of it here so
        the caller never has to know the mapping.
        """
        if not comms_ok:
            # Stale bits are not input. Drop the baseline so the return of the
            # link cannot be read as somebody pressing something.
            self._comms = False
            self.reset()
            return IDLE_INTENT
        was_comms, self._comms = self._comms, True

        try:
            sample = tuple(bool(di[c]) for c in self.channels)
        except (IndexError, TypeError):
            # A short or malformed image is not a button press.
            self.reset()
            return IDLE_INTENT

        if not was_comms:
            self.reset()            # re-baseline on reconnect, see hazard 3

        if sample == self._candidate:
            self._count += 1
        else:
            self._candidate, self._count = sample, 1

        if self._count < self.debounce_scans:
            # Not yet believed. Keep reporting the last trusted selector
            # position so the UI does not flicker while a contact settles.
            return self._steady()

        if self._stable is None:
            self._stable = sample   # baseline: no edges, see hazard 2
            return self._steady()

        was_reset, was_start, was_auto = self._stable
        now_reset, now_start, now_auto = sample
        self._stable = sample
        return PanelIntent(
            valid=True,
            reset=now_reset and not was_reset,      # rising edge, hazard 1
            start=now_start and not was_start,
            mode=AUTO if now_auto else MANUAL,
            mode_changed=now_auto != was_auto)

    def _steady(self):
        """A trusted image with no edges, or nothing if not yet baselined."""
        if self._stable is None:
            return IDLE_INTENT
        return PanelIntent(valid=True, reset=False, start=False,
                           mode=AUTO if self._stable[2] else MANUAL,
                           mode_changed=False)

    def mode(self):
        """Selector position for display, or None before the first baseline."""
        if self._stable is None:
            return None
        return AUTO if self._stable[2] else MANUAL


class DebouncedLevels:
    """Debounced LEVELS over any set of DI channels (the jog pendant).

    Same trust rules as PanelScan - a level must read the same on N consecutive
    scans, stale bits during a comms loss are not input, a malformed image is
    not input - but no anti-tie-down baseline: a pendant button is a deadman,
    and one held at power-on is meant to drive.

    scan() returns the believed levels as a tuple of bools, or None while there
    is nothing trustworthy to report.
    """

    def __init__(self, channels, debounce_scans=2):
        self.channels = tuple(int(c) for c in channels)
        self.debounce_scans = max(1, int(debounce_scans))
        self._stable = None
        self._candidate = None
        self._count = 0

    def reset(self):
        self._stable = None
        self._candidate = None
        self._count = 0

    def scan(self, di, comms_ok):
        if not comms_ok:
            self.reset()
            return None
        try:
            sample = tuple(bool(di[c]) for c in self.channels)
        except (IndexError, TypeError):
            self.reset()
            return None
        if sample == self._candidate:
            self._count += 1
        else:
            self._candidate, self._count = sample, 1
        if self._count >= self.debounce_scans:
            self._stable = sample
        return self._stable


# fwd/rvs/left/right - what the pendant is asking for
PendantIntent = namedtuple("PendantIntent", "fwd rvs left right")

PENDANT_IDLE = PendantIntent(False, False, False, False)


def pendant_intent(fwd, rvs, left, right):
    """Direction levels; an opposing pair cancels to nothing on that axis."""
    if fwd and rvs:
        fwd = rvs = False
    if left and right:
        left = right = False
    return PendantIntent(bool(fwd), bool(rvs), bool(left), bool(right))
