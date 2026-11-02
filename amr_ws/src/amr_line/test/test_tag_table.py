"""amr_line.tag_table: the per-run tag rules and the at-Home odometer. Pure, clock-fed."""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_line"), os.path.dirname(__file__)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from agv_core import config as vehicle  # noqa: E402
from amr_line.tag_table import TagOdometer, TagTable, home_window_m  # noqa: E402
from test_mission_engine import PER_REV, SITE  # noqa: E402


def counts_for(metres):
    c = int(metres / (3.141592653589793 * vehicle.WHEEL_DIA_M) * PER_REV)
    return (c, c)


def rfid(seq, enc, gen=0, ok=True):
    return {"comms_ok": ok, "encounter_seq": seq, "generation": gen, "encounters": enc}


# -- TagTable ------------------------------------------------------------------------

def test_destination_rules():
    t = TagTable(SITE, "MRU2")
    mru1, mru2, home = t.row("0110"), t.row("0120"), t.row("0010")
    assert t.stop_applies(home) == (True, "")
    assert t.stop_applies(mru1)[0] is False
    assert t.stop_applies(mru2) == (True, "")
    t.reached = True
    assert t.stop_applies(mru2)[0] is False and "already served" in t.stop_applies(mru2)[1]
    assert t.result() == "MRU2 served"


def test_no_destination_passes_every_destination_stop():
    t = TagTable(SITE, None)
    assert t.stop_applies(t.row("0110"))[0] is False
    assert t.stop_applies(t.row("0020")) == (True, "")


def test_the_u_turn_marks_a_pending_destination_missed_once():
    t = TagTable(SITE, "MRU2")
    assert t.note_u_turn() == "MRU2"
    assert t.note_u_turn() is None
    assert t.stop_applies(t.row("0120"))[0] is False
    assert "MISSED" in t.result()
    served = TagTable(SITE, "MRU2")
    served.reached = True
    assert served.note_u_turn() is None


def test_ignore_window_from_the_read_and_restarted_at_departure():
    t = TagTable(SITE, "MRU2")
    assert t.window_left(10.0, "0120") == 0.0
    t.acted(10.0, "0120")
    assert t.window_left(12.0, "0120") == 2.0
    assert t.window_left(14.0, "0120") == 0.0
    t.departed(300.0, "0120")  # parked for minutes, then left
    assert t.window_left(301.0, "0120") == 3.0


# -- TagOdometer ------------------------------------------------------------------------

WINDOW = home_window_m(SITE["HOME"])


def parked_at_home():
    o = TagOdometer()
    o.update(0.0, rfid(0, []), (0, 0), PER_REV)
    o.update(0.1, rfid(1, [(1, "0010")]), (0, 0), PER_REV)
    o.update(0.2, rfid(1, [(1, "0010")]), counts_for(0.5), PER_REV)
    return o


def test_home_is_the_last_tag_within_the_window():
    o = parked_at_home()
    assert o.at("0010", WINDOW) == (True, "")
    o.update(0.3, rfid(1, [(1, "0010")]), counts_for(0.5 + WINDOW), PER_REV)
    ok, why = o.at("0010", WINDOW)
    assert not ok and "moved" in why


def test_history_in_the_buffer_is_not_evidence():
    o = TagOdometer()
    o.update(0.0, rfid(3, [(3, "0010")]), (0, 0), PER_REV)
    assert o.at("0010", WINDOW)[0] is False, "a read from before the layer started proves nothing"


def test_another_tag_a_reconnect_or_a_link_drop_breaks_the_chain():
    o = parked_at_home()
    o.update(0.3, rfid(2, [(1, "0010"), (2, "0020")]), counts_for(0.5), PER_REV)
    assert "last tag read was 0020" in o.at("0010", WINDOW)[1]

    o = parked_at_home()
    o.update(0.3, rfid(1, [], gen=1), counts_for(0.5), PER_REV)
    assert o.at("0010", WINDOW)[0] is False

    o = parked_at_home()
    o.update(0.3, rfid(1, [(1, "0010")], ok=False), counts_for(0.5), PER_REV)
    assert o.at("0010", WINDOW)[0] is False


def test_wheel_feedback_must_be_there():
    o = TagOdometer()
    o.update(0.0, rfid(0, []), None, 0.0)
    o.update(0.1, rfid(1, [(1, "0010")]), None, 0.0)
    assert o.at("0010", WINDOW)[0] is False, "no counts since the tag: travel unknown"

    o = parked_at_home()
    o.update(0.5, rfid(1, [(1, "0010")]), None, PER_REV)  # one dropped sample is fine
    assert o.at("0010", WINDOW)[0] is True
    o.update(2.0, rfid(1, [(1, "0010")]), None, PER_REV)
    assert o.at("0010", WINDOW)[0] is False


def test_a_pivot_counts_as_travel():
    o = parked_at_home()
    c = counts_for(0.6)
    o.update(0.3, rfid(1, [(1, "0010")]), (counts_for(0.5)[0] + c[0], counts_for(0.5)[1] - c[1]), PER_REV)
    assert o.at("0010", WINDOW)[0] is False
