"""Browser jog sessions (unified plan §6.3). Pure, clock-injected, no ROS.

    sessions = JogSessions(clock)
    s = sessions.press(owner, instance, generation)   -> Session(id, ticket, deadline)
    cmd = sessions.refresh(session_id, ticket, seq, v, w, instance, generation)
                                                      -> Command to publish once, plus the next ticket
    sessions.release(session_id)                      -> invalidates BEFORE the zero is sent

Rules the tests pin:
  * one owner at a time: a second press while a live session exists is a
    conflict until that session is released or its ticket expires;
  * a refresh needs the current single-use ticket (a delayed/reordered POST
    carries an old one and is rejected), a strictly increasing seq, finite
    numbers, and the supervisor instance/generation the session was issued
    under - a mode change invalidates every session;
  * after the ticket deadline (server monotonic clock) nothing revives the
    session; a new press is required. Only the remaining lifetime travels
    into the ROS command (valid_for_s), never a fresh 0.2 s per late arrival;
  * limits are the manual jog caps (0.40 m/s, 0.39 rad/s since 2026-09-19; 0.30/0.30
    before), survey included, not the vehicle's.
"""

from __future__ import annotations

import math
import secrets
from dataclasses import dataclass

V_MAX, W_MAX = 0.40, 0.39
TICKET_S = 1.0  # a refresh must arrive within this; the browser sends every 0.1 s. Wider than
# CMD_S on purpose: the ROBOT stops 0.2 s after the last refresh regardless; this only decides
# whether a still-held button may resume after a link hiccup without a new press. At 0.25 s a
# single slow Wi-Fi round trip ended 14 of 17 holds on the vehicle (2026-09-17).
CMD_S = 0.20  # lifetime a command carries at most


class JogError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class Session:
    id: str
    owner: str
    instance: str
    generation: int
    ticket: str
    deadline: float
    seq: int = 0
    released: bool = False

    def live(self, now: float) -> bool:
        return not self.released and now <= self.deadline


@dataclass(frozen=True)
class Command:
    instance: str
    generation: int
    session: str
    seq: int
    valid_for_s: float
    v: float
    w: float
    ticket: str
    deadline: float


class JogSessions:
    def __init__(self, clock) -> None:
        self._clock = clock
        self._current: Session | None = None

    @property
    def current(self) -> Session | None:
        s = self._current
        return s if s is not None and s.live(self._clock()) else None

    def press(self, owner: str, instance: str, generation: int) -> Session:
        now = self._clock()
        cur = self.current
        if cur is not None and cur.owner != owner:
            raise JogError(409, "another browser holds the jog control")
        if not instance:
            raise JogError(503, "no supervisor")
        s = Session(
            id=secrets.token_hex(8),
            owner=owner,
            instance=instance,
            generation=int(generation),
            ticket=secrets.token_hex(8),
            deadline=now + TICKET_S,
        )
        self._current = s
        return s

    def refresh(
        self, session_id: str, ticket: str, seq: int, v: float, w: float, instance: str, generation: int
    ) -> Command:
        now = self._clock()
        s = self._current
        if s is None or s.id != session_id:
            raise JogError(409, "no such jog session; press again")
        if s.released:
            raise JogError(409, "session released; press again")
        if now > s.deadline:
            s.released = True
            raise JogError(409, "jog session expired; press again")
        if ticket != s.ticket:
            raise JogError(409, "stale refresh (old ticket)")
        if not isinstance(seq, int) or seq <= s.seq:
            raise JogError(409, "sequence must increase")
        if s.instance != instance or s.generation != int(generation):
            s.released = True
            raise JogError(409, "supervisor generation changed; press again")
        try:
            v, w = float(v), float(w)
        except (TypeError, ValueError):
            raise JogError(400, "v and w must be numbers") from None
        if not (math.isfinite(v) and math.isfinite(w)):
            raise JogError(400, "v and w must be finite")
        v = max(-V_MAX, min(V_MAX, v))
        w = max(-W_MAX, min(W_MAX, w))
        s.seq = seq
        s.ticket = secrets.token_hex(8)
        s.deadline = now + TICKET_S
        remaining = min(CMD_S, s.deadline - now)
        return Command(s.instance, s.generation, s.id, seq, remaining, v, w, s.ticket, s.deadline)

    def release(self, session_id: str) -> bool:
        s = self._current
        if s is None or s.id != session_id:
            return False
        s.released = True  # BEFORE the caller sends zero: a late nonzero can no longer land
        return True

    def invalidate_all(self) -> None:
        if self._current is not None:
            self._current.released = True
