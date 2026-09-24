"""ScanGate: release order, TF gating with settle, rate thinning, expiry."""

from amr_localization.scan_gate import ScanGate


def test_scan_waits_for_its_transform_plus_settle_then_releases_in_order():
    g = ScanGate(min_period_s=0.0, settle_s=0.02, hold_max_s=0.5)
    g.push(10.00, "a", now=10.001)
    g.push(10.03, "b", now=10.031)
    tf_until = 10.01  # odom TF covers up to here
    assert g.poll(10.031, lambda t: t <= tf_until) == []  # a needs TF at 10.02
    tf_until = 10.025
    assert g.poll(10.04, lambda t: t <= tf_until) == ["a"]  # b (needs 10.05) still held
    tf_until = 10.06
    assert g.poll(10.07, lambda t: t <= tf_until) == ["b"]
    assert g.stats.released == 2 and g.stats.expired == 0 and g.pending == 0


def test_rate_is_capped_but_every_released_scan_is_transformable():
    g = ScanGate(min_period_s=0.1, settle_s=0.0, hold_max_s=0.5)
    out = []
    for i in range(34):  # one second of 34 Hz scans, polled as they arrive
        g.push(i / 34, i, now=i / 34)
        out += g.poll(i / 34 + 0.01, lambda t: True)
    assert out == [0, 4, 8, 12, 16, 20, 24, 28, 32]  # every 4th at 34 Hz: >= 0.1 s apart
    assert all((b - a) / 34 >= 0.1 - 1e-9 for a, b in zip(out, out[1:], strict=False))
    assert g.stats.thinned == 34 - len(out)


def test_never_transformable_scan_expires_without_blocking_the_rest():
    g = ScanGate(min_period_s=0.0, settle_s=0.0, hold_max_s=0.5)
    g.push(5.0, "stuck", now=5.0)
    g.push(5.1, "ok", now=5.1)
    assert g.poll(5.3, lambda t: t >= 5.05) == []  # stuck blocks (order), not expired yet
    assert g.poll(5.6, lambda t: t >= 5.05) == ["ok"]
    assert g.stats.expired == 1 and g.stats.released == 1


# ---- Q13 ----


def test_q13_residence_deadline_applies_even_when_tf_is_available():
    g = ScanGate(min_period_s=0.0, settle_s=0.0, hold_max_s=0.5)
    g.push(1.0, "old", now=1.0)
    assert g.poll(2.0, lambda t: True) == []  # sat for 1 s (timer stalled): not released
    assert g.stats.expired == 1


def test_q13_source_clock_rewind_resets_the_thinning_reference():
    g = ScanGate(min_period_s=0.1, settle_s=0.0, hold_max_s=0.5)
    g.push(100.0, "a", now=100.0)
    assert g.poll(100.0, lambda t: True) == ["a"]
    g.push(1.0, "b", now=100.1)  # bag restarted at t=1
    assert g.poll(100.1, lambda t: True) == ["b"]
    # a merely out-of-order scan (within rewind_s) is thinned, the epoch is kept
    g.push(0.95, "c", now=100.2)
    assert g.poll(100.2, lambda t: True) == [] and g.stats.thinned == 1


def test_q13_queue_is_bounded_and_parameters_validated():
    import pytest

    g = ScanGate(max_pending=3, hold_max_s=10.0)
    for i in range(5):
        g.push(float(i), i, now=float(i))
    assert g.pending == 3 and g.stats.expired == 2
    for bad in ({"min_period_s": -1.0}, {"hold_max_s": float("nan")}, {"max_pending": 0}):
        with pytest.raises(ValueError):
            ScanGate(**bad)
