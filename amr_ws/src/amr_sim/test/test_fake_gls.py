"""fake_gls_node's pure parts (gls-wait-coding-plan §B.6, P2): SE(2), the label list, and the
capture window (the label centre within 0.020 m fore-aft and 0.035 m sideways of the camera,
in the CAMERA frame)."""

import math

import pytest

from amr_sim import fake_gls_node as fg

NO_MOUNT = (0.0, 0.0, 0.0)


def close(a, b, tol=1e-9):
    """Poses equal; the yaw (third) compared as an angle, so -pi and pi agree."""
    if len(a) != len(b):
        return False
    d = [x - y for x, y in zip(a, b, strict=True)]
    if len(d) == 3:
        d[2] = math.atan2(math.sin(d[2]), math.cos(d[2]))
    return all(abs(v) <= tol for v in d)


def test_compose_inverse_round_trips():
    poses = [(1.0, -2.0, 0.3), (0.0, 0.0, math.pi / 2), (-3.5, 0.25, -2.9), (0.1, 0.2, math.pi)]
    for a in poses:
        assert close(fg.compose(a, fg.inverse(a)), (0.0, 0.0, 0.0))
        assert close(fg.compose(fg.inverse(a), a), (0.0, 0.0, 0.0))
        assert close(fg.inverse(fg.inverse(a)), a)
        for b in poses:
            c = fg.compose(a, b)
            assert close(fg.compose(fg.inverse(a), c), b)
            assert -math.pi <= c[2] <= math.pi


def test_compose_is_rotate_then_translate():
    assert close(fg.compose((1.0, 0.0, math.pi / 2), (0.5, 0.0, 0.0)), (1.0, 0.5, math.pi / 2))
    assert fg.compose((0.0, 0.0, 3.0), (0.0, 0.0, 0.5))[2] == pytest.approx(3.5 - 2 * math.pi), "yaw wrapped"


def test_parse_labels():
    assert fg.parse_labels("") == []
    assert fg.parse_labels(" ; ;") == []
    got = fg.parse_labels("x0000y0000,1.0,2.0,0.0; x0100y0000 , -1.5 ,0.25, 1.5708 ;")
    assert got == [("x0000y0000", (1.0, 2.0, 0.0)), ("x0100y0000", (-1.5, 0.25, 1.5708))]
    for bad in ("x0000y0000,1.0,2.0", "a,1,2,3,4", "a,1,two,3", "a,1,2,3;b"):
        with pytest.raises(ValueError):
            fg.parse_labels(bad)


LABEL = [("L1", (1.0, 0.0, 0.0))]


def test_seen_inside_the_window():
    hit = fg.seen(LABEL, (1.0, 0.0, 0.0), NO_MOUNT)
    assert hit is not None and hit[0] == "L1" and close(hit[1], (0.0, 0.0, 0.0))
    hit = fg.seen(LABEL, (1.0195, 0.0345, 0.0), NO_MOUNT)
    assert hit is not None and close(hit[1], (0.0195, 0.0345, 0.0)), "T_LC is the camera in the label frame"
    assert fg.seen(LABEL, (0.9805, -0.0345, 0.0), NO_MOUNT) is not None


OUTSIDE = [(1.0205, 0.0, 0.0), (0.9795, 0.0, 0.0), (1.0, 0.0355, 0.0), (1.0, -0.0355, 0.0)]


@pytest.mark.parametrize("base", OUTSIDE)
def test_seen_just_outside_each_bound(base):
    assert fg.seen(LABEL, base, NO_MOUNT) is None


def test_seen_nothing_without_labels():
    assert fg.seen([], (1.0, 0.0, 0.0), NO_MOUNT) is None


def test_seen_respects_the_mount_offset_and_yaw():
    mount = (0.5, 0.1, 0.0)
    assert fg.seen(LABEL, (1.0, 0.0, 0.0), mount) is None, "the base over the label, the camera is not"
    hit = fg.seen(LABEL, (0.5, -0.1, 0.0), mount)
    assert hit is not None and close(hit[1], (0.0, 0.0, 0.0))
    # the vehicle turned 90 deg left: the camera is now 0.5 m to +y of the base
    hit = fg.seen(LABEL, (1.1, -0.5, math.pi / 2), mount)
    assert hit is not None and close(hit[1], (0.0, 0.0, math.pi / 2))
    # a yawed mount: the window turns with the camera, not the base
    yawed = (0.0, 0.0, math.pi / 2)
    assert fg.seen(LABEL, (1.03, 0.0, 0.0), yawed) is not None, "0.03 m along base x: sideways to the camera"
    assert fg.seen(LABEL, (1.0, 0.03, 0.0), yawed) is None, "0.03 m along base y is fore-aft to the camera"


def test_seen_a_rotated_label():
    labels = [("R", (2.0, 1.0, math.pi / 2))]
    hit = fg.seen(labels, (2.015, 1.0, 0.0), NO_MOUNT)
    # the camera 15 mm along map +x is 15 mm to the label's -y, yawed -90 deg against it
    assert hit is not None and hit[0] == "R" and close(hit[1], (0.0, -0.015, -math.pi / 2))
    # the window is the camera's: 30 mm along map x is fore-aft for a camera heading +x ...
    assert fg.seen(labels, (2.03, 1.0, 0.0), NO_MOUNT) is None
    # ... and sideways for one heading +y, whatever the label's own yaw
    assert fg.seen(labels, (2.03, 1.0, math.pi / 2), NO_MOUNT) is not None


def test_seen_picks_the_label_in_view():
    labels = [("A", (0.0, 0.0, 0.0)), ("B", (5.0, 0.0, 0.0))]
    assert fg.seen(labels, (5.01, 0.0, 0.0), NO_MOUNT)[0] == "B"
    assert fg.seen(labels, (0.0, 0.0, 1.0), NO_MOUNT)[0] == "A"
