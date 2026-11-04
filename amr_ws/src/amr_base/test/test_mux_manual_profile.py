"""The mux tick routes each source to its speed profile: pendant / browser jog follow the
jerk-limited S-curve (manual_a_max), the autonomous sources the same S-curve on their linear
speed (a_max / d_max, auto_jerk; 2026-11-03) with a plain yaw slew, and any loss of authority
zeroes both the speed and the acceleration state at once."""

from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from amr_base import cmd_mux_kinematics_node as mux
from amr_base import gating
from amr_base.diff_drive import Geometry


class _Logger:
    def info(self, *_a, **_k):
        pass

    warn = error = info


def _node(dt=0.02):
    n = mux.CmdMuxKinematics.__new__(mux.CmdMuxKinematics)
    n.geom = Geometry(0.09, 0.487)
    n.w_max = 100.0
    n.a_max, n.alpha_max, n.d_max, n.delta_max = 0.15, 0.4, 0.5, 1.0
    n.manual_a_max, n.manual_alpha_max, n.manual_jerk, n.manual_jerk_w = 0.3, 0.4, 1.0, 2.0
    n.manual_d_max = 0.5
    n.auto_jerk = 1.0
    n.dt = dt
    n.gp = gating.Params()
    n._teleop = n._follow = n._rotate = n._permit = n._panel = n._lease = n._drives = None
    n._manual = gating.ManualIntake()
    n._commissioning = None
    n._v = n._wz = n._wl = n._wr = n._a = n._alpha = 0.0
    n._source, n._reason, n._applied_gen = "none", "", 0
    n._inhibited, n._event_seq = False, 0
    n._pub_event = SimpleNamespace(publish=lambda _m: None)
    n._last = gating.Selection(gating.NONE, 0.0, 0.0, "", 0, False)
    n._now = lambda: 10.0
    n.get_logger = lambda: _Logger()
    n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    n._pub = SimpleNamespace(publish=lambda _m: None)
    n.survey_w_max, n._surveying = 0.27, False
    return n


def _run(n, sel, ticks):
    mux.gating.select = lambda *_a, **_k: sel  # the selection under test, whatever the inputs
    for _ in range(ticks):
        n._tick()


@pytest.fixture(autouse=True)
def _restore_select():
    keep = mux.gating.select
    yield
    mux.gating.select = keep


def test_pendant_and_follow_both_follow_the_s_curve():
    n = _node()
    _run(n, gating.Selection(gating.PENDANT, 0.5, 0.0, "pendant", 0), 5)  # 0.1 s
    v_manual = n._v
    assert n._a == pytest.approx(0.1, abs=0.011)  # acceleration still building (jerk 1.0)
    assert v_manual < 0.015  # well below the 0.03 a 0.3 m/s^2 step would give in 0.1 s
    _run(n, gating.Selection(gating.PENDANT, 0.5, 0.0, "pendant", 0), 95)  # 2.0 s in total
    assert n._v == pytest.approx(0.5) and n._a == 0.0

    m = _node()
    _run(m, gating.Selection(gating.FOLLOW, 0.5, 0.0, "follow", 0), 5)
    assert m._a == pytest.approx(0.1, abs=0.011), "auto acceleration builds at auto_jerk too"
    assert m._v < 0.15 * 0.1, "below what the old 0.15 m/s^2 step gave in 0.1 s"
    _run(m, gating.Selection(gating.FOLLOW, 0.5, 0.0, "follow", 0), 300)
    assert m._v == pytest.approx(0.5) and m._a == 0.0


def test_pendant_release_brakes_at_manual_d_max_not_d_max():
    """2026-11-02: manual has its own deceleration; the autonomous d_max is untouched."""
    n = _node()
    n.manual_a_max, n.manual_d_max = 0.15, 0.25
    _run(n, gating.Selection(gating.PENDANT, 0.5, 0.0, "pendant", 0), 250)  # 5 s
    assert n._v == pytest.approx(0.5)
    peak, t = 0.0, 0
    while n._v > 0.0 and t < 500:
        v0 = n._v
        _run(n, gating.Selection(gating.PENDANT, 0.0, 0.0, "pendant", 0), 1)
        peak, t = max(peak, (v0 - n._v) / n.dt), t + 1
    assert n._v == 0.0
    assert peak == pytest.approx(0.25, abs=1e-6)  # never harder than manual_d_max
    assert 0.5 / 0.25 < t * n.dt < 0.5 / 0.25 + 0.25 / 1.0  # 2.0 s at 0.25, plus the jerk-in

    m = _node()
    m.manual_d_max = 0.25
    _run(m, gating.Selection(gating.FOLLOW, 0.5, 0.0, "follow", 0), 400)
    assert m._v == pytest.approx(0.5)
    peak = 0.0
    while m._v > 0.0:
        v0 = m._v
        _run(m, gating.Selection(gating.FOLLOW, 0.0, 0.0, "follow", 0), 1)
        peak = max(peak, (v0 - m._v) / m.dt)
    assert peak == pytest.approx(0.5, abs=1e-6), "FOLLOW still brakes up to d_max, not manual_d_max"


def test_loss_of_authority_zeroes_speed_and_acceleration_state():
    n = _node()
    _run(n, gating.Selection(gating.PENDANT, 0.5, 0.0, "pendant", 0), 20)
    assert n._v > 0.0 and n._a > 0.0
    _run(n, gating.Selection(gating.NONE, 0.0, 0.0, "MANUAL, no fresh command", 0, True), 1)
    assert (n._v, n._a, n._out) == (0.0, 0.0, (0.0, 0.0))


def test_manual_spin_is_capped_while_surveying():
    from amr_interfaces.msg import ModeState

    assert gating.survey_spin_cap(0.39, True, 0.27) == 0.27
    assert gating.survey_spin_cap(-0.39, True, 0.27) == -0.27
    assert gating.survey_spin_cap(0.39, False, 0.27) == 0.39
    assert gating.survey_spin_cap(0.2, True, 0.27) == 0.2  # arcs / diagonals unchanged
    n = _node()
    n._on_mode(ModeState(mode=ModeState.MAPPING))
    assert n._surveying
    for _ in range(300):  # 6 s: long enough to reach any target
        _run(n, gating.Selection(gating.PENDANT, 0.0, 0.39, "pendant", 0), 1)
    assert n._wz == pytest.approx(0.27)
    n._on_mode(ModeState(mode=ModeState.IDLE))
    for _ in range(300):
        _run(n, gating.Selection(gating.PENDANT, 0.0, 0.39, "pendant", 0), 1)
    assert n._wz == pytest.approx(0.39)

def test_follow_taper_is_jerk_limited_and_yaw_keeps_the_plain_slew():
    """Owner 2026-11-03: the taper before an arc (0.50 -> 0.30 m/s) builds its deceleration
    at auto_jerk instead of stepping to d_max; it lands on 0.30 without overshoot. Yaw is
    still the plain alpha_max slew (path tracking unchanged)."""
    n = _node()
    _run(n, gating.Selection(gating.FOLLOW, 0.0, 0.3, "follow", 0), 5)  # 0.1 s
    assert n._wz == pytest.approx(0.4 * 0.1)
    m = _node()
    m._v = 0.5
    _run(m, gating.Selection(gating.FOLLOW, 0.3, 0.0, "follow", 0), 1)
    assert 0.5 - m._v < 0.5 * m.dt / 2, "the first tick is not a full d_max step"
    decels, t = [], 0
    while m._v > 0.3 and t < 200:
        v0 = m._v
        _run(m, gating.Selection(gating.FOLLOW, 0.3, 0.0, "follow", 0), 1)
        decels.append((v0 - m._v) / m.dt)
        t += 1
    assert m._v == pytest.approx(0.3) and m._a == 0.0
    assert max(decels) <= 0.5 + 1e-9
    steps = [abs(b - a) / m.dt for a, b in zip(decels, decels[1:], strict=False)]
    assert max(steps[:-1]) <= 1.0 + 1e-6, "deceleration changes no faster than auto_jerk"


def test_estop_zeroes_the_ramps_and_a_pendant_held_through_reset_stays_dead():
    """Emergency plan §4.3 / §8 step 2: jog held, E-stop, release, Reset with the pendant
    still held - the vehicle must not move until the pendant has been let go once."""
    n = _node()
    n._repress = gating.RepressGuard()

    def panel(emg, fwd):
        n._panel = gating.Panel(10.0, True, False, fwd=fwd, emergency=emg)

    panel(0, True)
    for _ in range(20):
        n._tick()
    assert n._v > 0.0 and n._a > 0.0
    panel(1, True)
    n._tick()
    assert (n._v, n._a, n._out, n._last.code) == (0.0, 0.0, (0.0, 0.0), "ESTOP")
    for emg in (2, 3, 0):  # released E-stop, Reset, NORMAL - the pendant held all along
        panel(emg, True)
        for _ in range(10):
            n._tick()
        assert (n._v, n._a, n._out) == (0.0, 0.0, (0.0, 0.0)), emg
    assert "release the pendant" in n._last.reason
    panel(0, False)
    n._tick()
    panel(0, True)
    for _ in range(5):
        n._tick()
    assert n._v > 0.0 and n._last.source == gating.PENDANT


def test_a_warning_field_slowdown_is_not_softened():
    """2026-11-03: normal auto braking is d_max 0.25 with jerk 0.4, but while a warning field is
    in force the run slows with the plain slew at field_d_max (0.5, as before), never the
    gentle S-curve."""
    n = _node()
    n.d_max, n.auto_jerk, n.field_d_max = 0.25, 0.4, 0.5
    n._v = 0.6
    n._speed_ramp = gating.SpeedRamp()
    n._speed_ramp.target = 0.5  # warning field 1: x0.5
    n._speed_ramp.tick = lambda *_a, **_k: 0.5
    _run(n, gating.Selection(gating.FOLLOW, 0.3, 0.0, "follow (warning field: x0.50)", 0), 1)
    assert n._v == pytest.approx(0.6 - 0.5 * n.dt), "full 0.5 m/s^2 from the first tick"
    m = _node()
    m.d_max, m.auto_jerk, m.field_d_max = 0.25, 0.4, 0.5
    m._v = 0.6
    _run(m, gating.Selection(gating.FOLLOW, 0.3, 0.0, "follow", 0), 1)
    assert 0.6 - m._v < 0.25 * m.dt / 2, "no field: the gentle S-curve"
