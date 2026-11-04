import math

import pytest
from amr_localization.gyro_bias import BiasEstimator

BIAS = math.radians(0.0574)  # reconciliation D-3


def feed(est, t0, seconds, rate, stationary, dt=0.01):
    t, out = t0, None
    for _ in range(int(seconds / dt)):
        t += dt
        out = est.update(t, rate, stationary)
    return t, out


def test_nothing_until_calibrated():
    est = BiasEstimator(window_s=2.0, settle_s=0.3)
    t, out = feed(est, 0.0, 1.0, BIAS, True)
    assert out is None and not est.calibrated


def test_calibrates_after_window_and_subtracts():
    est = BiasEstimator(window_s=2.0, settle_s=0.3)
    t, out = feed(est, 0.0, 2.5, BIAS, True)
    assert est.calibrated
    assert est.bias == pytest.approx(BIAS)
    assert out == pytest.approx(0.0, abs=1e-12)
    t, out = feed(est, t, 1.0, BIAS + 0.5, False)  # moving: pass-through minus bias
    assert out == pytest.approx(0.5)


def test_moving_resets_the_window_but_keeps_the_bias():
    est = BiasEstimator(window_s=2.0, settle_s=0.3)
    t, _ = feed(est, 0.0, 2.5, BIAS, True)
    assert est.windows == 1
    t, _ = feed(est, t, 1.0, BIAS, True)  # partial second window
    t, _ = feed(est, t, 0.1, 1.0, False)  # a move
    t, out = feed(est, t, 1.0, BIAS, True)  # not enough stillness yet
    assert est.windows == 1 and est.bias == pytest.approx(BIAS)
    assert out == pytest.approx(0.0, abs=1e-12)


def test_settle_excludes_ramp_down_samples():
    est = BiasEstimator(window_s=1.0, settle_s=0.3)
    t, _ = feed(est, 0.0, 0.29, 5.0, True)  # decelerating tail, still "stationary" by wheels
    t, _ = feed(est, t, 1.05, BIAS, True)
    assert est.calibrated
    assert est.bias == pytest.approx(BIAS)


def test_bias_tracks_drift_between_moves():
    est = BiasEstimator(window_s=1.0, settle_s=0.1)
    t, _ = feed(est, 0.0, 1.2, BIAS, True)
    t, _ = feed(est, t, 0.5, 2.0, False)
    t, _ = feed(est, t, 1.2, 2 * BIAS, True)
    assert est.windows == 2
    assert est.bias == pytest.approx(2 * BIAS)


def test_reset():
    est = BiasEstimator(window_s=1.0, settle_s=0.1)
    feed(est, 0.0, 1.2, BIAS, True)
    est.reset()
    assert not est.calibrated
    assert est.update(10.0, BIAS, True) is None
