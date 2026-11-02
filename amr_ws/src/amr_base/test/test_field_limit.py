"""Scanner fields gate AUTO (2026-10-02): protective zeroes, warning 1 (outer) x0.5 ramped
in 1 s, warning 2 (inner) a deceleration stop within 0.40 m (0.47 2026-10-07, 0.40 2026-10-08), the stricter
winning, unknown zeroes."""

import os

import pytest

from amr_base.diff_drive import slew_asym
from amr_base.gating import (
    FOLLOW,
    LINE,
    MANUAL,
    NONE,
    PENDANT,
    ROTATE,
    Field,
    FieldParams,
    FieldView,
    Params,
    Selection,
    SpeedRamp,
    stop_time,
    apply_scale,
    field_limit,
    field_view,
    line_uturn_exempt,
)

# agv-01 (config/scanner_fields.yaml): path 0 protective, path 2 outer warning, path 1 inner.
FP = FieldParams(protective_index=0, warning_indices=(2, 1), warning_scales=(0.5, 0.0), warning_active_level=False,
                 warning_stop_m=0.40)
SUP = Params(require_supervisor=True)


def sel(source, v=0.4, w=0.2):
    return Selection(source, v, w, "x", 3)


def test_view_on_a_walk_in():
    """Walking in reads (paths 0,1,2) 111 -> 110 -> 100 -> 000: outer, then inner, then protective."""
    assert field_view(1.0, Field(0.9, (True, True, True)), FP) == FieldView(True, True, False, 0, 1.0)
    assert field_view(1.0, Field(0.9, (True, True, False)), FP) == FieldView(True, True, True, 1, 0.5)
    assert field_view(1.0, Field(0.9, (True, False, False)), FP) == FieldView(True, True, True, 2, 0.0)
    assert field_view(1.0, Field(0.9, (False, False, False)), FP) == FieldView(True, False, True, 2, 0.0)
    # the inner field alone (something entering from the side) still asks for its stop
    assert field_view(1.0, Field(0.9, (True, False, True)), FP).warning_scale == 0.0
    inverted = FieldParams(warning_indices=(1,), warning_scales=(0.5,), warning_active_level=True)
    assert field_view(1.0, Field(0.9, (True, True)), inverted).warning_active


def test_stale_short_or_missing_is_not_clear():
    assert field_view(2.0, Field(1.0, (True, True, True)), FP) == FieldView(False, False, False)
    assert field_view(1.0, Field(0.9, (True, True)), FP).fresh is False  # path 2 missing
    assert field_view(1.0, None, FP).fresh is False
    assert field_view(1.0, None, FieldParams(assume_clear=True)) == FieldView(True, True, False)


def test_protective_zeroes_every_auto_source():
    v = FieldView(True, False, True, 2, 0.0)
    for src in (FOLLOW, ROTATE, LINE):
        out, k = field_limit(sel(src), v, SUP, FP)
        assert (out.source, out.v, out.w, out.code, k) == (NONE, 0.0, 0.0, "FIELD_PROTECTIVE", 0.0)


def test_warning_targets_and_the_scale_keeps_the_curvature():
    out, k = field_limit(sel(LINE, 0.30, 0.10), FieldView(True, True, True, 1, 0.5), SUP, FP)
    assert (out.source, out.v, out.w, k) == (LINE, 0.30, 0.10, 0.5), "the target, not yet applied"
    _, k = field_limit(sel(LINE), FieldView(True, True, True, 2, 0.0), SUP, FP)
    assert k == 0.0
    half = apply_scale(out, 0.5)
    assert (half.v, half.w, half.generation) == (0.15, 0.05, 3)


def test_every_change_ramps_in_1_s():
    """Operator, 2026-10-02: time ramps, 1.0 s per change in either direction."""
    dt = 0.025  # 40 ticks = 1 s
    r = SpeedRamp()
    for _ in range(20):
        r.tick(0.5, dt, FP)
    assert r.value == pytest.approx(0.75), "halfway to warning 1 at half the time"
    for _ in range(20):
        r.tick(0.5, dt, FP)
    assert r.value == pytest.approx(0.5)
    r.tick(0.5, dt, FP)
    assert r.value == 0.5, "it never undershoots"
    for _ in range(39):
        r.tick(0.1, dt, FP)
    assert r.value > 0.1, "warning 1 -> warning 2 also takes the full second"
    r.tick(0.1, dt, FP)
    r.tick(0.1, dt, FP)
    assert r.value == pytest.approx(0.1)
    for _ in range(39):
        r.tick(1.0, dt, FP)
    assert r.value < 1.0, "not yet back before 1.0 s"
    r.tick(1.0, dt, FP)
    r.tick(1.0, dt, FP)
    assert r.value == pytest.approx(1.0)
    straight = SpeedRamp()
    for _ in range(40):
        straight.tick(0.1, dt, FP)
    assert straight.value == pytest.approx(0.1), "clear -> warning 2 directly: 1 s too"
    step = SpeedRamp()
    assert step.tick(0.5, dt, FieldParams(warning_decel_s=0.0)) == 0.5, "a zero time is a step"


def test_unknown_fields_zero_auto_only_when_supervised():
    out, _ = field_limit(sel(FOLLOW), FieldView(False, False, False), SUP, FP)
    assert out.source == NONE and out.code == "FIELD_UNKNOWN"
    out, k = field_limit(sel(FOLLOW), FieldView(False, False, False), Params(), FP)
    assert out.source == FOLLOW and k == 1.0


def test_manual_sources_are_untouched():
    for src in (MANUAL, PENDANT, NONE):
        s = sel(src)
        assert field_limit(s, FieldView(True, False, True, 2, 0.0), SUP, FP) == (s, 1.0)


def test_the_saved_field_set_matches_the_scanner_and_feeds_these_params():
    """config/scanner_fields.yaml: the corners give the ranges the nanoScan3 reports on
    /field_data (0.64 / 1.14 / 1.92 m, 2026-10-02), and its params are FP above."""
    import importlib.util

    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "amr_bringup"))
    spec = importlib.util.spec_from_file_location("scanner_fields", os.path.join(root, "amr_bringup", "scanner_fields.py"))
    sf = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sf)
    doc = sf.load(os.path.join(root, "config", "scanner_fields.yaml"))
    reach = {r: round(sf.reach_m(doc["fields"][r]["rect"]), 2) for r in sf.ROLES}
    assert reach == {"protective": 0.64, "warning_1": 1.92, "warning_2": 1.14}
    p = sf.mux_params(doc)
    got = (p["protective_index"], tuple(p["warning_indices"]), tuple(p["warning_scales"]), p["warning_active_level"])
    assert got == (FP.protective_index, FP.warning_indices, FP.warning_scales, FP.warning_active_level)
    assert p["warning_stop_m"] == FP.warning_stop_m == 0.40
    gap = doc["fields"]["warning_2"]["rect"]["x_max"] - doc["fields"]["protective"]["rect"]["x_max"]
    assert gap - p["warning_stop_m"] == pytest.approx(0.07), "latency margin before the protective field"
    w2 = doc["fields"]["warning_2"]

    def with_w2(**kw):
        return {**doc, "fields": {**doc["fields"], "warning_2": {**w2, **kw}}}

    with pytest.raises(ValueError, match="only warning_2"):
        sf.validate({**doc, "fields": {**doc["fields"], "warning_1": {**doc["fields"]["warning_1"], "scale": 0.0, "stop_m": 0.4}}})
    with pytest.raises(ValueError, match="gap to protective"):
        sf.validate(with_w2(stop_m=0.48))  # past the protective field's front edge
    no_stop = {k: v for k, v in w2.items() if k != "stop_m"}
    with pytest.raises(ValueError, match="exactly when"):
        sf.validate({**doc, "fields": {**doc["fields"], "warning_2": no_stop}})
    with pytest.raises(ValueError, match="exactly when"):
        sf.validate(with_w2(scale=0.1))  # a slow field with a stop distance


def _run_stop(v_cmd, k0=1.0, d_max=0.5, stop_d_max=0.7540 * 4 / 3, dt=0.02):
    """The mux AUTO path: factor ramp -> scaled command -> slew (drive decel during a stop).
    Returns (distance travelled after warning 2 is seen, peak decel)."""
    r = SpeedRamp(k0)
    r.target = k0
    v = v_cmd * k0
    dist, peak = 0.0, 0.0
    for _ in range(2000):
        k = r.tick(0.0, dt, FP, v_now=v)
        nv = slew_asym(v, v_cmd * k, 0.3, max(d_max, stop_d_max) if r.stopping else d_max, dt)
        peak = max(peak, (v - nv) / dt)
        dist += 0.5 * (v + nv) * dt
        v = nv
        if v == 0.0:
            return dist, peak
    raise AssertionError("never stopped")


@pytest.mark.parametrize("v_cmd,k0", [(0.85, 1.0), (0.55, 1.0), (0.6, 1.0), (0.5, 1.0), (0.85, 0.5), (0.6, 0.5)])
def test_warning_2_stops_within_the_gap_to_the_protective_field(v_cmd, k0):
    """Operator, 2026-10-07: warning 2 front edge 0.97 m, protective 0.50 m: the stop takes
    ~0.40 m from any AUTO speed (tracked NORMAL 0.5 / HIGH 0.85, trackless 0.6), straight in or
    already slowed by warning 1; 0.07 m of the 0.47 m gap is left for detection latency."""
    dist, peak = _run_stop(v_cmd, k0)
    assert dist == pytest.approx(0.40, abs=0.01), dist
    v0 = v_cmd * k0
    assert peak == pytest.approx(v0 * v0 / (2 * 0.40), rel=0.05), "a constant decel, no harder"


def test_stop_timing_falls_back_for_a_spin_or_no_distance():
    assert stop_time(0.85, FP) == pytest.approx(2 * 0.40 / 0.85)
    assert stop_time(-0.6, FP) == pytest.approx(2 * 0.40 / 0.6), "reversing stops the same"
    assert stop_time(0.01, FP) is None, "spinning in place: warning_decel_s"
    assert stop_time(0.85, FieldParams()) is None, "no stop distance set"
    r = SpeedRamp()
    for _ in range(40):
        r.tick(0.0, 0.025, FP, v_now=0.0)
    assert r.value == pytest.approx(0.0) and r.stopping, "time-ramped to 0 in 1 s"
    for _ in range(41):
        r.tick(1.0, 0.025, FP)
    assert r.value == pytest.approx(1.0) and not r.stopping, "clear: back up in 1 s"


# -- U-turn: warning 1 does not slow it (operator, 2026-10-08) ------------------------------

W1 = FieldView(True, True, True, 1, 0.5)
W2 = FieldView(True, True, True, 2, 0.0)
PROT = FieldView(True, False, True, 2, 0.0)


def test_an_exempt_u_turn_ignores_warning_1_but_still_stops_on_warning_2_and_protective():
    pivot = sel(LINE, 0.0, -0.26)
    assert field_limit(pivot, W1, SUP, FP, slow_exempt=True) == (pivot, 1.0), "keeps the U-turn speed"
    assert field_limit(pivot, W1, SUP, FP)[1] == 0.5, "not exempt: halved as before"
    assert field_limit(pivot, W2, SUP, FP, slow_exempt=True)[1] == 0.0, "warning 2 still stops it"
    out, k = field_limit(pivot, PROT, SUP, FP, slow_exempt=True)
    assert (out.source, k) == (NONE, 0.0), "the protective field still zeroes it"


def test_the_exemption_needs_line_a_fresh_u_turn_phase_and_u_turn_speed():
    fresh = (10.0, "spin")
    assert line_uturn_exempt(sel(LINE, 0.0, -0.26), fresh, 11.0, 2.5, 0.12)
    for phase in ("approach", "stopping", "spin", "center", "settle"):
        assert line_uturn_exempt(sel(LINE, 0.10, 0.0), (10.0, phase), 10.5, 2.5, 0.12), phase
    assert not line_uturn_exempt(sel(LINE, 0.0, -0.26), (10.0, ""), 10.5, 2.5, 0.12), "not in a U-turn"
    assert not line_uturn_exempt(sel(LINE, 0.0, -0.26), fresh, 13.0, 2.5, 0.12), "stale line state"
    assert not line_uturn_exempt(sel(LINE, 0.0, -0.26), None, 10.5, 2.5, 0.12), "no line state yet"
    assert not line_uturn_exempt(sel(LINE, 0.30, 0.0), (10.0, "approach"), 10.5, 2.5, 0.12), \
        "still slowing from cruise on the approach: warning 1 applies"
    assert not line_uturn_exempt(sel(FOLLOW, 0.0, -0.26), fresh, 10.5, 2.5, 0.12), "LINE only"
    assert not line_uturn_exempt(sel(LINE, 0.0, -0.26), fresh, 10.5, 2.5, 0.0), "0 turns it off"
