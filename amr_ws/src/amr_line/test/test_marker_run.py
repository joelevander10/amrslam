"""Marker run (mission marker_run, tracked plan 3 section 7.5): the straight in front of MRU
with no RFID. Code 2 starts the CW U-turn on the way out, code 1 ends the run at home on
the way back, a code 2 on the way back past the skip distance is the backstop (home marker
missed: FAULT). Real FollowJob, synthetic clock, wheel counts and marker stream."""
import os
import sys

HERE = os.path.dirname(__file__)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from agv_core import config as vehicle  # noqa: E402
from agv_core import mission as missions  # noqa: E402
from test_mission_engine import CENTRED, NO_TAPE, PER_REV, World  # noqa: E402

from amr_line import job as lj  # noqa: E402
from amr_line import uturn  # noqa: E402

UTURN_CODE, HOME_CODE = 2, 1


def mission():
    return missions.load("mru-straight-markers")


def started():
    w = World(mission())
    w.comms = False                     # no RFID at all: it starts and drives without the link
    w.start()
    w.drive(1.0)
    assert w.job.state == lj.RUNNING, w.job.reason
    return w


def pivot(w):
    """From the U-turn marker: creep, the tape ends, stop, pivot cw, reacquire, resume."""
    assert w.job.tape.uturn_req is not None and w.job.tape.uturn_req["direction"] == "cw"
    w.drive(2.0)
    assert abs(w.job.diag["speed_target_rpm"] - 0.1 * vehicle.RPM_PER_MPS) < 1.0
    assert w.until(lambda: w.job.tape.uturn_req["phase"] == "stopping", 3.0, sensor=NO_TAPE)
    w.still, w.integrate = True, False
    assert w.until(lambda: w.job.tape.uturn is not None, 3.0, sensor=NO_TAPE), w.job.reason
    base = w.counts
    for deg, sensor in ((40, NO_TAPE), (120, NO_TAPE), (175, CENTRED)):
        c = uturn.counts_for_angle(deg, PER_REV)
        w.counts = (base[0] + int(round(c)), base[1] + int(round(-c)))
        w.tick(sensor=sensor)
    w.drive(vehicle.U_TURN_RESUME_DELAY_S + 0.2)
    assert w.job.tape.uturn_req is None and w.job.state == lj.RUNNING, w.job.reason
    w.still, w.integrate = False, True
    assert w.job.tape.mrun_phase == "back"


def test_the_mission_loads_and_needs_no_rfid():
    m = mission()
    assert m["TAGS"] == {} and m["HOME"] is None and m["MARKER_RUN"]["home_code"] == HOME_CODE
    w = started()
    assert not w.job.tape.needs_link and w.job.tape.needs_markers


def test_a_full_lap_out_pivot_back_home():
    w = started()
    w.mark(HOME_CODE)                   # the departure passes the home marker again: ignored
    w.tick()
    assert w.job.tape.stop is None and w.job.tape.uturn_req is None
    w.drive(2.0)
    w.mark(UTURN_CODE)
    w.tick()
    pivot(w)
    w.mark(UTURN_CODE)                  # re-passed right after the pivot: skipped, no backstop
    w.tick()
    assert w.job.state == lj.RUNNING and not w.job.tape.fault
    w.drive(4.0)
    w.mark(HOME_CODE)
    w.tick()
    assert w.job.tape.stop is not None and w.job.tape.stop["where"] == "T-HOME"
    assert w.until(lambda: w.job.state == lj.DONE, 5.0), w.job.reason
    assert "T-HOME" in w.job.reason and "run complete" in w.job.reason


def test_a_missed_home_marker_faults_at_the_backstop():
    w = started()
    w.drive(1.0)
    w.mark(UTURN_CODE)
    w.tick()
    pivot(w)
    w.drive(6.0)                        # 3 m back: past the re-pass skip
    w.mark(UTURN_CODE)                  # the backstop strip past T-HOME
    left, right = w.tick()
    assert w.job.state == lj.FAULT and (left, right) == (0.0, 0.0)
    assert "home marker missed" in w.job.reason


def test_a_missed_u_turn_marker_ends_at_the_tape_end():
    w = started()
    assert w.until(lambda: w.job.state == lj.DONE, 5.0, sensor=NO_TAPE), w.job.reason
    assert "track lost" in w.job.reason


def test_markers_lost_while_driving_hold_for_a_person():
    w = started()
    w.mk_ok = False
    w.tick()
    assert w.job.state == lj.HOLD and w.job.hold_cause == "markers", w.job.reason
    w.press_start()
    assert w.job.state == lj.HOLD, "refused while the stream is down"
    w.mk_ok = True
    w.tick()
    w.start()
    assert w.job.state == lj.RUNNING
    w.mark(3, lost=1)                   # an event lost on the topic: the home marker could be it
    w.tick()
    assert w.job.state == lj.HOLD and "lost" in w.job.reason


def test_start_is_refused_without_markers():
    w = World(mission())
    w.mk_ok = False
    w.press_start()
    assert w.job.state == lj.IDLE and "markers down" in w.job.reason


def test_markers_during_the_pivot_do_nothing():
    w = started()
    w.mark(UTURN_CODE)
    w.tick()
    w.mark(HOME_CODE)                   # read while approaching the tape end: ignored
    w.tick()
    assert w.job.tape.stop is None and w.job.tape.uturn_req["phase"] == "approach"
