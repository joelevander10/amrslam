"""The configuration namespace the tape engine is HANDED, instead of importing.

`autopilot.py` is a byte-for-byte port from `gy-demo`, where it did
`import config` and got the vehicle profile loader — a module that reads
`profiles/<AGV_PROFILE>.json` from disk at import. A ROS node, a unit test and
the sim all need the same 34 names from three different sources, and a layer
that reads a file at import cannot be constructed twice in one process with two
different vehicles. So the port aliases this module to the name `config`:

    from amr_line import runtime as config      # autopilot.py, line 34

and every `config.K_RATIO` inside the engine lands here. That single alias is
what kept the port's review surface to three lines.

**Nothing here is read at import.** The engine reads every constant at call
time (its own docstring promises this), so `load_from_profile()` need only run
before the first `update()`, not before `import autopilot`.

**This module is deliberately mutable.** `simulate()` in the test rig assigns
`config.AUTO_RPM` around a run and restores it in a `finally`; the ported
`test_control.py` does the same with `K_RATIO` and `KD`. A frozen namespace
would break the rig that proves the gains.

The strictness is in `__getattr__`: an unset name raises with the list of what
to do about it, rather than returning a silent zero. A tape follower that
reads `KD` as 0.0 because a profile key was renamed does not fail — it steers
badly, at speed, on a real vehicle.
"""

# Every name the engine reads, with the profile key it comes from. The engine
# is the authority on this list: it is `grep -o 'config\.[A-Z_]*' autopilot.py`
# and nothing else. `None` means the value is derived rather than read from a
# profile section, and `load_from_profile()` knows how.
_FROM_PROFILE = {
    # steering gains
    "K_RATIO": "K_RATIO",
    "KD": "KD",
    "KI": "KI",
    "HIGH_K_RATIO": "HIGH_K_RATIO",
    "HIGH_KD": "HIGH_KD",
    "TAU_D_S": "TAU_D_S",
    "TI_DEADBAND_MM": "TI_DEADBAND_MM",
    "I_CLAMP": "I_CLAMP",
    # speed reduction on error
    "SR_POS_FRAC": "SR_POS_FRAC",
    "SR_RATE_FRAC": "SR_RATE_FRAC",
    "SR_CAP": "SR_CAP",
    "SR_TAU_S": "SR_TAU_S",
    # speeds and the software ramp
    "AUTO_RPM": "AUTO_RPM",
    "AUTO_HIGH_RPM": "AUTO_HIGH_RPM",
    "RAMP_ACCEL_RPM_S": "RAMP_ACCEL_RPM_S",
    "RAMP_JERK_RPM_S2": "RAMP_JERK_RPM_S2",
    "INNER_WHEEL_MIN_RPM": "INNER_WHEEL_MIN_RPM",
    # sensor handling
    "INVERT_ERROR": "INVERT_ERROR",
    "BRANCH_POSITIVE_IS_LEFT": "BRANCH_POSITIVE_IS_LEFT",
    "SENSOR_MAX_MM": "SENSOR_MAX_MM",
    "SENSOR_MAX_STEP_MM": "SENSOR_MAX_STEP_MM",
    "SENSOR_TIMEOUT_S": "SENSOR_TIMEOUT_S",
    "LINE_LOSS_GRACE_M": "LINE_LOSS_GRACE_M",
    "DT_NOMINAL_S": "DT_NOMINAL_S",
    # vehicle geometry, shared with the ROS base nodes
    "SENSOR_LOOKAHEAD_M": "SENSOR_LOOKAHEAD_M",
    "MOTOR_MAX_RPM": "MOTOR_MAX_RPM",
    "MPS_PER_RPM": "MPS_PER_RPM",
    "RAD_S_PER_RPM_DIFF": "RAD_S_PER_RPM_DIFF",
}

# Derived here rather than read, with the formulas config.py used on gy-demo.
# Keeping the derivation next to the names that need it is what stops a profile
# holding a dt band that disagrees with its own dt_nominal_s.
_DERIVED = ("DT_MIN_S", "DT_MAX_S", "LINE_LOSS_GRACE_MAX_S")

REQUIRED = tuple(_FROM_PROFILE) + _DERIVED

_loaded = False


def __getattr__(name):
    """Anything the engine asks for that nobody set."""
    if name in REQUIRED:
        raise AttributeError(
            f"amr_line.runtime.{name} was never set: call "
            f"load_from_profile() (or configure({name}=...)) before the first "
            f"follower tick. The engine reads its constants at call time, so "
            f"this is a startup-ordering bug, not a missing profile key."
        )
    raise AttributeError(
        f"amr_line.runtime has no attribute {name!r}. The engine reads only "
        f"{len(REQUIRED)} names and this is not one of them; if a new constant "
        f"was added to autopilot.py, add it to _FROM_PROFILE here too."
    )


def configure(**values):
    """Set names directly. For tests and the sim, where there is no profile."""
    unknown = sorted(set(values) - set(REQUIRED))
    if unknown:
        raise ValueError(
            f"not names the engine reads: {unknown}. Allowed: {sorted(REQUIRED)}"
        )
    globals().update(values)
    return {k: globals()[k] for k in REQUIRED if k in globals()}


def load_from_profile(config=None):
    """Populate from the vehicle profile: `agv_core.config` unless overridden.

    Call once at node startup, before the first tick. Safe to call again — a
    profile reload is the only way a running layer picks up a re-tuned gain.
    """
    global _loaded
    if config is None:
        from agv_core import config  # noqa: PLC0415  (keeps import-time clean)

    missing = [key for key in _FROM_PROFILE.values() if not hasattr(config, key)]
    if missing:
        raise RuntimeError(
            "the vehicle profile is missing the autopilot section this layer "
            f"needs: {missing}. Restore the `autopilot` block to the profile "
            "(agv_core/config.py _SCHEMA carries the keys and their types)."
        )

    ns = {name: getattr(config, key) for name, key in _FROM_PROFILE.items()}

    # dt is measured on the vehicle, then clamped to [0.2x, 5x] nominal, so a
    # scheduling hiccup or a resume-after-stop gap cannot blow up I and D.
    ns["DT_MIN_S"] = 0.2 * ns["DT_NOMINAL_S"]
    ns["DT_MAX_S"] = 5.0 * ns["DT_NOMINAL_S"]

    # A standstill backstop for the line-loss budget, which is a DISTANCE: a
    # stopped vehicle accrues no distance and would otherwise wait for ever.
    # Keyed off the SLOWER cruise speed (NORMAL), the case the ceiling has to
    # tolerate.
    slowest = min(ns["AUTO_RPM"], ns["AUTO_HIGH_RPM"])
    ns["LINE_LOSS_GRACE_MAX_S"] = (
        5.0 * ns["LINE_LOSS_GRACE_M"] / max(slowest * ns["MPS_PER_RPM"], 1e-6)
    )

    globals().update(ns)
    _loaded = True
    return ns


def snapshot():
    """Every name currently set, for a diagnostic topic or a test assertion."""
    return {k: globals()[k] for k in REQUIRED if k in globals()}


def is_loaded():
    return _loaded
