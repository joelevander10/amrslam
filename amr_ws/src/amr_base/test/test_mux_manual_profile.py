"""The mux tick routes each source to its speed profile: pendant / browser jog follow the
jerk-limited S-curve (manual_a_max), the autonomous sources the plain ramp (a_max), and
any loss of authority zeroes both the speed and the acceleration state at once."""

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
    n.dt = dt
    n.gp = gating.Params()
    n._teleop = n._follow = n._rotate = n._permit = n._panel = n._lease = n._drives = None
    n._manual = gating.ManualIntake()
    n._commissioning = None
    n._v = n._wz = n._wl = n._wr = n._a = n._alpha = 0.0
    n._source, n._reason, n._applied_gen = "none", "", 0
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


def test_pendant_follows_the_s_curve_and_follow_the_plain_ramp():
    n = _node()
    _run(n, gating.Selection(gating.PENDANT, 0.5, 0.0, "pendant", 0), 5)  # 0.1 s
    v_manual = n._v
    assert n._a == pytest.approx(0.1, abs=0.011)  # acceleration still building (jerk 1.0)
    assert v_manual < 0.015  # well below the 0.03 a 0.3 m/s^2 step would give in 0.1 s
    _run(n, gating.Selection(gating.PENDANT, 0.5, 0.0, "pendant", 0), 95)  # 2.0 s in total
    assert n._v == pytest.approx(0.5) and n._a == 0.0

    m = _node()
    _run(m, gating.Selection(gating.FOLLOW, 0.5, 0.0, "follow", 0), 5)
    assert m._v == pytest.approx(0.15 * 0.1) and m._a == 0.0  # 0.15 m/s^2 from the first tick


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
