"""Unit tests for the pure analysis functions in diagnose_forward_veer.py.

Importing diagnose_forward_veer pulls in depth_hold_bar02_test and
motor_test; all three are import-safe (main() guarded) — pytest.ini already
restricts collection to tests/test_*.py so the hardware scripts themselves
are never collected.
"""

import math

import pytest

import diagnose_forward_veer as dv


# ---------------------------------------------------------------- steady_mean

def test_steady_mean_skips_spinup():
    samples = [(0.2, 10.0, 16.0), (0.8, 9.0, 16.0),   # spin-up, skipped
               (1.5, 2.0, 15.8), (2.5, 2.2, 15.6)]
    amps, volts, n = dv.steady_mean(samples, skip_s=1.0)
    assert amps == pytest.approx(2.1)
    assert volts == pytest.approx(15.7)
    assert n == 2


def test_steady_mean_empty_window():
    assert dv.steady_mean([(0.1, 5.0, 16.0)], skip_s=1.0) == (None, None, 0)
    assert dv.steady_mean([], skip_s=1.0) == (None, None, 0)


def test_steady_mean_all_volts_unknown():
    amps, volts, n = dv.steady_mean([(2.0, 1.0, None), (3.0, 3.0, None)],
                                    skip_s=1.0)
    assert amps == pytest.approx(2.0)
    assert volts is None
    assert n == 2


# -------------------------------------------------------------- analyze_group

def test_analyze_group_flags_weak_outlier():
    med, rows = dv.analyze_group({1: 2.0, 2: 2.1, 3: 1.0, 4: 1.9})
    assert med == pytest.approx(1.95)
    assert rows[3][2] == 'WEAK'
    assert rows[1][2] == '' and rows[2][2] == '' and rows[4][2] == ''


def test_analyze_group_flags_strong_outlier():
    _med, rows = dv.analyze_group({1: 2.0, 2: 2.0, 3: 2.0, 4: 3.0})
    assert rows[4][2] == 'STRONG'


def test_analyze_group_balanced_within_threshold():
    _med, rows = dv.analyze_group({m: a for m, a in
                                   zip((5, 6, 7, 8), (2.0, 2.1, 1.9, 2.05))})
    assert all(v[2] == '' for v in rows.values())


def test_analyze_group_light_load_no_flags():
    # dry props: median below MIN_GROUP_MEDIAN_A -> ratios meaningless
    med, rows = dv.analyze_group({1: 0.02, 2: 0.01, 3: 0.04, 4: 0.02})
    assert med < dv.MIN_GROUP_MEDIAN_A
    assert all(dev is None and v == '' for _n, dev, v in rows.values())


def test_analyze_group_handles_missing_motor():
    _med, rows = dv.analyze_group({1: 2.0, 2: None, 3: 2.0, 4: 2.0})
    assert rows[2] == (None, None, '')
    assert rows[1][2] == ''


def test_analyze_group_all_missing():
    med, rows = dv.analyze_group({1: None, 2: None})
    assert med is None
    assert rows == {1: (None, None, ''), 2: (None, None, '')}


# ---------------------------------------------------------------- wrap_delta

def test_wrap_delta_plain():
    assert dv.wrap_delta(0.1, 0.3) == pytest.approx(0.2)
    assert dv.wrap_delta(0.3, 0.1) == pytest.approx(-0.2)


def test_wrap_delta_across_pi_boundary():
    # +175deg -> -175deg is a +10deg step, not -350deg
    a = math.radians(175.0)
    b = math.radians(-175.0)
    assert dv.wrap_delta(a, b) == pytest.approx(math.radians(10.0), abs=1e-6)
    assert dv.wrap_delta(b, a) == pytest.approx(math.radians(-10.0), abs=1e-6)


def test_wrap_delta_unwraps_full_turn():
    # 36 steps of +10deg through two wrap crossings sums to +360deg
    yaws = [math.radians(((i * 10) + 180) % 360 - 180) for i in range(37)]
    total = sum(dv.wrap_delta(a, b) for a, b in zip(yaws, yaws[1:]))
    assert math.degrees(total) == pytest.approx(360.0, abs=1e-6)


# -------------------------------------------------------------- pwm_symmetry

def test_pwm_symmetry_symmetric():
    spread, sym = dv.pwm_symmetry({1: -200.0, 2: -198.0, 3: 202.0, 4: 199.0})
    assert sym is True
    assert spread < 10.0


def test_pwm_symmetry_asymmetric():
    spread, sym = dv.pwm_symmetry({1: -200.0, 2: -100.0, 3: 200.0, 4: 195.0})
    assert sym is False
    assert spread > 10.0


def test_pwm_symmetry_all_at_trim():
    spread, sym = dv.pwm_symmetry({1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    assert spread is None
    assert sym is False
