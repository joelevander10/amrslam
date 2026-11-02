"""Operator event log: a bounded ring buffer of things a human needs to know.

The web UI used to keep this in the browser, which meant a page reload threw it
away - and the AGV stops itself on line loss, sensor timeout and watchdog, so the
reason for a stop could disappear before anyone read it. Keeping it server-side
makes it survive a reload, a page switch, and a second browser.

*** Only operator-relevant TRANSITIONS belong here. ***
Nothing on a per-tick path may emit: at 50 Hz a single chatty call site flushes
the entire buffer of everything meaningful within four seconds. That means no
emits from the PID tick, the loop-health window, or telemetry polling - and
anything edge-triggered (a driver fault, a lost tape) must fire once per EDGE,
not once per poll.

Deliberately dependency-free so both the bus thread and Flask can import it
without either pulling in the other.
"""
import collections
import itertools
import threading
import time

MAX_EVENTS = 200

LEVELS = ("info", "warn", "error")

_lock = threading.Lock()
_buf = collections.deque(maxlen=MAX_EVENTS)
_seq = itertools.count(1)


def emit(level, msg):
    """Record one event. Never raises - logging must not break the vehicle."""
    try:
        if level not in LEVELS:
            level = "info"
        entry = {"seq": next(_seq), "t": time.time(),
                 "level": level, "msg": str(msg)}
        with _lock:
            _buf.append(entry)
        return entry["seq"]
    except Exception:                       # noqa: BLE001
        return 0


def info(msg):
    return emit("info", msg)


def warn(msg):
    return emit("warn", msg)


def error(msg):
    return emit("error", msg)


def latest_seq():
    """Highest sequence number recorded. The UI polls this cheaply inside the
    state snapshot and only fetches events when it has fallen behind."""
    with _lock:
        return _buf[-1]["seq"] if _buf else 0


def since(seq=0):
    """(latest_seq, [entries newer than seq]) in chronological order.

    seq=0 returns the whole buffer, which is what a freshly loaded page wants.
    A caller whose seq predates the oldest retained entry simply gets everything
    still held - the gap is invisible, which is the accepted cost of a ring.
    """
    with _lock:
        items = [dict(e) for e in _buf if e["seq"] > seq]
        newest = _buf[-1]["seq"] if _buf else 0
    return newest, items


def clear():
    """Test helper. Not reachable from the UI - operators should not be able to
    erase the record of why the vehicle stopped."""
    with _lock:
        _buf.clear()
