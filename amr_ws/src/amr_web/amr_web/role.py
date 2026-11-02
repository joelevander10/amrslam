"""Operator / engineer roles for the pages.

The panel in the hall is an operator's tool: Home, Run, Alarms and nothing else.
The engineering pages (Monitor, I/O, Params, Commission, Routes, Maps) stay one
PIN away, because operators tap things and a tapped Params page is a changed
vehicle.

*** This is a MISTAKE GUARD, NOT SECURITY. *** Anyone on the vehicle's network can
reach every endpoint directly; the PIN only stops a finger. Nothing here may be
relied on to protect the vehicle - the physical panel and the supervisor's lease
do that, as they always did.

WHERE THE PIN LIVES
-------------------
Not in the repository, and not in a page: `~/.amr/web.json` (0600, created on
first use), or the AMR_WEB_ENGINEER_PIN environment variable, which wins. The
browser never receives it - it POSTs a candidate to /api/role and the server
compares. The comparison is constant-time out of habit, not because a 4-digit
PIN deserves it.
"""

from __future__ import annotations

import hmac
import json
import os
import secrets

OPERATOR, ENGINEER = "operator", "engineer"
DEFAULT_PIN = "1805"
FILENAME = "web.json"


def _path(state_dir: str) -> str:
    return os.path.join(os.path.expanduser(state_dir), FILENAME)


def load(state_dir: str) -> dict:
    """The web's own settings file. Missing or unreadable = defaults; this must never
    stop the pages from serving."""
    try:
        with open(_path(state_dir), encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def ensure(state_dir: str) -> dict:
    """Create `web.json` with a PIN and a session secret on first run, 0600."""
    data = load(state_dir)
    changed = False
    if not str(data.get("engineer_pin", "")).strip():
        data["engineer_pin"] = DEFAULT_PIN
        changed = True
    if not str(data.get("secret", "")).strip():
        data["secret"] = secrets.token_hex(32)
        changed = True
    if changed:
        path = _path(state_dir)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            # tmp + fsync + replace: a cut during the first run must not leave a
            # half-written PIN file that locks the engineer out (power-loss plan W4).
            tmp = path + ".tmp"
            fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, path)
        except OSError:
            pass  # read-only state dir: the PIN still works for this process
    return data


def pin(state_dir: str) -> str:
    env = os.environ.get("AMR_WEB_ENGINEER_PIN", "").strip()
    return env or str(ensure(state_dir).get("engineer_pin") or DEFAULT_PIN)


def secret(state_dir: str) -> str:
    return str(ensure(state_dir).get("secret") or secrets.token_hex(32))


def check(state_dir: str, candidate: str) -> bool:
    return hmac.compare_digest(str(candidate or "").strip(), pin(state_dir))
