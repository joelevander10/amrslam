"""Direction -> (left r/min, right r/min) for the differential drive.

Wheel sign convention for THIS AGV: both drivers take a POSITIVE target
velocity (60FFh) to travel forward. That means the wheels are opposite-handed,
or the drivers already have their rotation direction reversed in MEXE02.

*** If you swap a motor, re-flash a driver, or remount a wheel, re-verify this
on blocks before trusting any button. *** Set vehicle.invert_left /
invert_right in the vehicle profile rather than editing the table - the table
stays in vehicle terms.

Speeds and the invert flags live in the profile (config.MANUAL_FULL_RPM,
MANUAL_HALF_RPM, INVERT_LEFT, INVERT_RIGHT). What stays here is the pad layout
and its labels, which are interface, not vehicle parameters.
"""
import config

# Vehicle-frame table, before INVERT_* is applied. Built per call rather than at
# import so it always reflects the loaded profile.
#   curves  : outer wheel FULL, inner wheel HALF
#   spins   : wheels equal and opposite at HALF, so the AGV turns on its axis
#   reverse : the forward arc driven backwards (both signs flipped)
#
# The profile stores full_rpm as a whole number and half_ratio as a fraction;
# config rounds the product, because 60FFh is an INT32 and a float setpoint
# would render as "480.0 / 800" on the buttons and in /api/config. The bus
# itself is safe either way - canworker._write_target() casts before packing.
def _table():
    full = config.MANUAL_FULL_RPM
    half = config.MANUAL_HALF_RPM
    return {
        "forward":       ( full,  full),
        "forward_left":  ( half,  full),
        "forward_right": ( full,  half),
        "reverse":       (-full, -full),
        "reverse_left":  (-half, -full),
        "reverse_right": (-full, -half),
        "left":          (-half,  half),   # spin on axis, counter-clockwise
        "right":         ( half, -half),   # spin on axis, clockwise
        "stop":          (    0,     0),
    }


DIRECTIONS = frozenset(_table())

# Order the 3x3 pad renders in, reading left-to-right, top-to-bottom.
PAD = [
    "forward_left",  "forward", "forward_right",
    "left",          "stop",    "right",
    "reverse_left",  "reverse", "reverse_right",
]

LABELS = {
    "forward": "FWD",            "forward_left": "FWD-L",  "forward_right": "FWD-R",
    "left": "SPIN-L",            "right": "SPIN-R",        "stop": "STOP",
    "reverse": "REV",            "reverse_left": "REV-L",  "reverse_right": "REV-R",
}

GLYPHS = {
    "forward": "↑",      "forward_left": "↖",  "forward_right": "↗",
    "left": "↶",         "right": "↷",         "stop": "■",
    "reverse": "↓",      "reverse_left": "↙",  "reverse_right": "↘",
}

# Human-readable key hints shown on each button. The browser does the actual
# matching from two axes (see static/manual.js), so the in-between directions
# come from holding two arrows at once. Display only - nothing parses these.
#
# ARROWS ONLY. The letter shortcuts (WASD, and Q/E/Z/C for the diagonals) were
# removed from the page and from manual.js together; a hint here for a key that
# no longer does anything is worse than no hint. Space stays because it is the
# stop, and it is the one key binding on this page that only ever stops.
KEYMAP = {
    "forward_left":  "↑ + ←",   "forward":  "↑",       "forward_right": "↑ + →",
    "left":          "←",       "stop":     "Space",   "right":         "→",
    "reverse_left":  "↓ + ←",   "reverse":  "↓",       "reverse_right": "↓ + →",
}


def velocities(direction):
    """(left_rpm, right_rpm) in DRIVER terms, ready to write to 60FFh."""
    left, right = _table()[direction]
    return (-left if config.INVERT_LEFT else left,
            -right if config.INVERT_RIGHT else right)


def is_direction(name):
    return name in DIRECTIONS
