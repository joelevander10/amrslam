"""survey_move_node without a ROS graph: authority, the command stream, aborts, the SLAM check."""

import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from amr_interfaces.msg import ControlLease, DriveStatus, ManualCommand, MappingState, PanelState
from amr_interfaces.msg import SurveyMoveState as S
from amr_interfaces.srv import SurveyMove
from amr_mission import survey_move_node as smn


class Log:
    """Like rclpy's logger: one call site may not change severity between calls (that
    ValueError killed the node on the vehicle, 2026-09-19)."""

    sites: dict = {}

    def _log(self, level):
        import inspect  # noqa: PLC0415

        f = inspect.stack()[2]
        key = (f.filename, f.lineno)
        if Log.sites.setdefault(key, level) != level:
            raise ValueError("Logger severity cannot be changed between calls.")

    def info(self, *_a, **_k):
        self._log("info")

    def warn(self, *_a, **_k):
        self._log("warn")

    def error(self, *_a, **_k):
        self._log("error")


def node():
    n = smn.SurveyMoveNode.__new__(smn.SurveyMoveNode)
    n.clock = [100.0]
    n._now = lambda: n.clock[0]
    n.get_logger = lambda: Log()
    n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    n.cmd_valid, n.settle_s, n.input_age, n.max_corr_deg = 0.2, 1.5, 0.5, 2.0
    n.profile = smn.sm.Profile()
    n._init_state()
    n.sent, n.states = [], []
    n._cmd_pub = SimpleNamespace(publish=n.sent.append)
    n._state_pub = SimpleNamespace(publish=n.states.append)
    n.map_odom = (0.0, 0.0, 0.0)
    n._map_odom = lambda: n.map_odom
    feed(n)
    return n


def feed(n, auto=False, pendant=False, drives=True, mapping=MappingState.MAPPING, allowed=1):
    p = PanelState(valid=True, mode_auto=auto, pendant_fwd=pendant)
    n._panel, n._panel_t = p, n.clock[0]
    n._lease, n._lease_t = ControlLease(instance="sup", generation=7, allowed=allowed), n.clock[0]
    n._drives_ok, n._drives_t = drives, n.clock[0]
    n._mapping = mapping
    if n._odom is None:
        n._odom = (0.0, 0.0, 0.0)
    n._odom_t = n.clock[0]


def request(n, kind, value):
    return n._srv_move(SurveyMove.Request(kind=kind, value=float(value)), SurveyMove.Response())


@pytest.mark.parametrize(
    "kw, why",
    [
        ({"mapping": MappingState.IDLE}, "not surveying"),
        ({"auto": True}, "not MANUAL"),
        ({"pendant": True}, "pendant"),
        ({"allowed": 0}, "manual lease"),
        ({"drives": False}, "torque"),
    ],
)
def test_refused_without_manual_authority(kw, why):
    n = node()
    feed(n, **kw)
    r = request(n, "straight", 1.0)
    assert not r.accepted and why in r.message and n.state == S.IDLE


def test_bad_values_are_refused():
    n = node()
    assert not request(n, "rotate", 30).accepted
    assert not request(n, "straight", 12.0).accepted


def test_move_streams_a_manual_command_of_its_own_session_then_settles_and_checks():
    n = node()
    r = request(n, "straight", 0.5)
    assert r.accepted and r.message == "forward 0.50 m" and n.state == S.MOVING
    n._tick()
    m: ManualCommand = n.sent[-1]
    assert (m.instance, m.generation, m.valid_for_s) == ("sup", 7, 0.2)
    assert m.session.startswith("survey-") and m.v > 0 and m.seq == 1
    # the vehicle drives: odometry reaches the target
    n.clock[0] += 2.0
    feed(n)
    n._odom = (0.5, 0.0, 0.0)
    n._tick()
    assert n.state == S.SETTLING and n.sent[-1].valid_for_s == 0.0  # our session revoked: zero
    n.map_odom = (0.0, 0.0, math.radians(3.0))  # SLAM turned the map by 3 deg meanwhile
    n.clock[0] += 1.6
    n._tick()
    assert n.state == S.DONE and n.check is not None and not n.check[0]
    assert n.check[2] == pytest.approx(3.0) and "slipped" in n.reason
    st = n.states[-1]
    assert (
        st.state == S.DONE and st.check_done and not st.check_ok and st.correction_deg == pytest.approx(3.0)
    )


def test_a_clean_move_passes_the_check():
    n = node()
    request(n, "rotate", -90)
    n._tick()
    assert n.sent[-1].w < 0 and n.sent[-1].v == 0.0
    n._odom = (0.0, 0.0, -math.pi / 2)
    n.clock[0] += 5.0
    feed(n)
    n._tick()
    n.clock[0] += 1.6
    n._tick()
    assert n.state == S.DONE and n.check[0]


@pytest.mark.parametrize(
    "event, why",
    [
        ("pendant", "pendant"),
        ("estop", "torque"),
        ("auto", "not MANUAL"),
        ("jog", "jog pad or Stop"),
        ("stop_service", "stopped by operator"),
        ("survey_ended", "not surveying"),
    ],
)
def test_any_pendant_button_estop_selector_jog_or_stop_aborts(event, why):
    n = node()
    request(n, "straight", 3.0)
    n._tick()
    n.clock[0] += 0.05
    if event == "pendant":
        n._on_panel(PanelState(valid=True, pendant_left=True))
    elif event == "estop":
        n._on_drives(DriveStatus(operational=False))
        n._tick()
    elif event == "auto":
        feed(n, auto=True)
        n._tick()
    elif event == "jog":
        n._on_manual(ManualCommand(session="browser-1", v=0.1))
    elif event == "stop_service":
        from std_srvs.srv import Trigger  # noqa: PLC0415

        assert n._srv_stop(Trigger.Request(), Trigger.Response()).success
    elif event == "survey_ended":
        feed(n, mapping=MappingState.RETURN_REVIEW)
        n._tick()
    assert n.state == S.ABORTED and why in n.reason
    assert n.sent[-1].valid_for_s == 0.0 and n.sent[-1].session == n.session  # revoked at the mux
    count = len(n.sent)
    n._tick()
    assert len(n.sent) == count  # nothing more after an abort


def test_own_commands_echoed_back_do_not_abort_and_a_second_move_waits():
    n = node()
    request(n, "straight", 3.0)
    n._tick()
    n._on_manual(n.sent[-1])  # our own stream, seen on the topic
    assert n.state == S.MOVING
    assert not request(n, "straight", 1.0).accepted


def test_a_clean_then_a_slipped_move_log_without_crashing():
    n = node()
    for slip_deg in (0.0, 3.0, 0.0):
        n.map_odom = (0.0, 0.0, 0.0)
        feed(n)
        assert request(n, "straight", 0.5).accepted
        n._tick()
        n._odom = (n._odom[0] + 0.5, 0.0, 0.0)
        n.clock[0] += 2.0
        feed(n)
        n._tick()
        n.map_odom = (0.0, 0.0, math.radians(slip_deg))
        n.clock[0] += 1.6
        n._tick()
        assert n.state == S.DONE and n.check[0] == (slip_deg == 0.0)


def test_a_bug_in_the_loop_stops_the_move_and_the_node_lives_on():
    n = node()
    request(n, "straight", 2.0)
    n._tick()

    def boom(*_a):
        raise RuntimeError("bad")

    n.ctl.step = boom
    n._tick()  # must not raise
    assert n.state == S.ABORTED and "internal error" in n.reason
    assert n.sent[-1].valid_for_s == 0.0  # revoked: the vehicle stops
    assert request(n, "straight", 1.0).accepted  # and the next move works
