"""P3 (unified plan §12.2): a released or stale browser command cannot return."""

import pytest
from amr_web.server import create_app

from amr_web import jog


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def sessions():
    clock = Clock()
    return jog.JogSessions(clock), clock


def test_release_then_delayed_nonzero_is_refused(sessions):
    js, clock = sessions
    s = js.press("tab-A", "inst", 3)
    c = js.refresh(s.id, s.ticket, 1, 0.2, 0.0, "inst", 3)
    assert (c.v, c.session, c.seq) == (0.2, s.id, 1) and 0 < c.valid_for_s <= jog.CMD_S
    ticket = c.ticket
    assert js.release(s.id)
    with pytest.raises(jog.JogError, match="released"):
        js.refresh(s.id, ticket, 2, 0.2, 0.0, "inst", 3)  # the delayed POST from the released session


def test_old_ticket_old_generation_nonfinite_and_second_tab(sessions):
    js, clock = sessions
    s = js.press("tab-A", "inst", 3)
    first_ticket = s.ticket
    c1 = js.refresh(s.id, first_ticket, 1, 0.1, 0.0, "inst", 3)
    with pytest.raises(jog.JogError, match="old ticket"):
        js.refresh(s.id, first_ticket, 2, 0.1, 0.0, "inst", 3)  # reordered: still carries the first ticket
    with pytest.raises(jog.JogError, match="sequence"):
        js.refresh(s.id, c1.ticket, 1, 0.1, 0.0, "inst", 3)
    with pytest.raises(jog.JogError, match="finite"):
        js.refresh(s.id, c1.ticket, 2, float("nan"), 0.0, "inst", 3)
    with pytest.raises(jog.JogError, match="numbers"):
        js.refresh(s.id, c1.ticket, 2, "fast", 0.0, "inst", 3)
    with pytest.raises(jog.JogError, match="another browser"):
        js.press("tab-B", "inst", 3)
    # the supervisor rotated the generation: this session is dead, a new press is needed
    with pytest.raises(jog.JogError, match="generation"):
        js.refresh(s.id, c1.ticket, 2, 0.1, 0.0, "inst", 4)
    assert js.current is None
    s2 = js.press("tab-B", "inst", 4)  # free now
    assert s2.generation == 4


def test_expired_ticket_needs_a_new_press_and_lifetime_never_regrows(sessions):
    js, clock = sessions
    s = js.press("tab-A", "inst", 3)
    c = js.refresh(s.id, s.ticket, 1, 0.3, 0.0, "inst", 3)
    clock.t += jog.TICKET_S + 0.01
    with pytest.raises(jog.JogError, match="expired"):
        js.refresh(s.id, c.ticket, 2, 0.3, 0.0, "inst", 3)
    with pytest.raises(jog.JogError, match="press again"):
        js.refresh(s.id, c.ticket, 3, 0.3, 0.0, "inst", 3)
    s = js.press("tab-A", "inst", 3)
    c = js.refresh(s.id, s.ticket, 1, 5.0, -9.0, "inst", 3)
    assert (c.v, c.w) == (jog.V_MAX, -jog.W_MAX)  # clamped to the survey caps
    assert c.valid_for_s <= jog.CMD_S


class Stub:
    def __init__(self):
        self.identity = ("inst", 3)
        self.published = []

    def supervisor_identity(self):
        return self.identity

    def manual_publish(self, cmd):
        self.published.append(cmd)

    def __getattr__(self, name):
        return lambda *a: (True, "ok")


def test_http_press_refresh_release_publishes_exactly_one_command_per_refresh(tmp_path):
    from amr_mission.fixtures import write_world_as_bundle  # noqa: PLC0415

    write_world_as_bundle(str(tmp_path))
    stub = Stub()
    app = create_app(stub, str(tmp_path))
    c = app.test_client()
    assert c.post("/api/manual/press", json={}).status_code == 400
    r = c.post("/api/manual/press", json={"owner": "tab-A"})
    assert r.status_code == 200 and r.json["generation"] == 3
    sid, ticket = r.json["session"], r.json["ticket"]
    r = c.post("/api/manual/refresh", json={"session": sid, "ticket": ticket, "seq": 1, "v": 0.2, "w": 0})
    assert r.status_code == 200 and len(stub.published) == 1 and stub.published[0].v == 0.2
    assert c.post("/api/manual/press", json={"owner": "tab-B"}).status_code == 409
    r2 = c.post("/api/manual/refresh", json={"session": sid, "ticket": ticket, "seq": 2, "v": 0.2, "w": 0})
    assert r2.status_code == 409 and len(stub.published) == 1  # stale ticket: nothing published
    c.post("/api/manual/release", json={"session": sid})
    assert stub.published[-1].v == 0.0 and stub.published[-1].w == 0.0
    r3 = c.post(
        "/api/manual/refresh", json={"session": sid, "ticket": r.json["ticket"], "seq": 3, "v": 0.2, "w": 0}
    )
    assert r3.status_code == 409 and len(stub.published) == 2
    # a supervisor change invalidates even a fresh session
    r = c.post("/api/manual/press", json={"owner": "tab-A"})
    stub.identity = ("inst", 4)
    r = c.post(
        "/api/manual/refresh",
        json={"session": r.json["session"], "ticket": r.json["ticket"], "seq": 1, "v": 0.1, "w": 0},
    )
    assert r.status_code == 409 and "generation" in r.json["message"]


def test_r02_http_release_revokes_at_the_mux_after_refreshes(tmp_path):
    """Server -> mux path: the release the server publishes must beat the seq filter."""
    from amr_mission.fixtures import write_world_as_bundle  # noqa: PLC0415

    from amr_base.gating import (  # noqa: PLC0415
        LEASE_MANUAL,
        MANUAL,
        NONE,
        Drives,
        Lease,
        ManualIntake,
        Panel,
        Params,
        select,
    )

    write_world_as_bundle(str(tmp_path))
    stub = Stub()
    c = create_app(stub, str(tmp_path)).test_client()
    mux = ManualIntake()

    def deliver(cmd):
        mux.offer(10.0, cmd.instance, cmd.generation, cmd.session, cmd.seq, cmd.valid_for_s, cmd.v, cmd.w)

    def source():
        lease = Lease(10.0, "inst", 3, 1, LEASE_MANUAL)
        return select(
            10.0,
            None,
            None,
            None,
            None,
            Panel(10.0, True, False),
            Params(require_supervisor=True),
            lease,
            mux.current,
            Drives(10.0, True),
        ).source

    r = c.post("/api/manual/press", json={"owner": "tab-A"}).json
    sid, ticket = r["session"], r["ticket"]
    for seq in range(1, 8):
        ticket = c.post(
            "/api/manual/refresh", json={"session": sid, "ticket": ticket, "seq": seq, "v": 0.2, "w": 0}
        ).json["ticket"]
    refreshes = list(stub.published)
    for cmd in refreshes:
        deliver(cmd)
    assert source() == MANUAL
    c.post("/api/manual/release", json={"session": sid})
    deliver(stub.published[-1])
    assert source() == NONE
    # reordered delivery: an in-flight refresh arriving after the release stays dead
    deliver(refreshes[-1])
    assert source() == NONE
    # duplicate release, then a fresh press works again
    c.post("/api/manual/release", json={"session": sid})
    deliver(stub.published[-1])
    r = c.post("/api/manual/press", json={"owner": "tab-A"}).json
    body = {"session": r["session"], "ticket": r["ticket"], "seq": 1, "v": 0.1, "w": 0}
    c.post("/api/manual/refresh", json=body)
    deliver(stub.published[-1])
    assert source() == MANUAL
    c.post("/api/stop", json={})
    deliver(stub.published[-1])
    assert source() == NONE
