"""Pure pressure→depth conversion. No MAVLink, no hardware."""
import pytest

from mavlink_thruster_control.pressure import (
    PRESSURE_TYPES, depth_from_pressure, latch_surface, pick_pressure_type,
    surface_sane)


def test_prefers_external_bar02_over_hull_baro():
    # SCALED_PRESSURE (instance 0) is the FMU's internal baro: it reads the
    # air inside the sealed hull, not the water. Never pick it over the Bar02.
    assert pick_pressure_type(
        ['SCALED_PRESSURE', 'SCALED_PRESSURE2']) == 'SCALED_PRESSURE2'
    assert pick_pressure_type(
        ['SCALED_PRESSURE3', 'SCALED_PRESSURE2']) == 'SCALED_PRESSURE2'
    assert pick_pressure_type(
        ['SCALED_PRESSURE', 'SCALED_PRESSURE3']) == 'SCALED_PRESSURE3'


def test_hull_baro_alone_is_not_a_depth_source():
    # A wrong depth is worse than no depth: the hull baro never changes with
    # depth, so ALT_HOLD would "hold" against a constant and the sub would
    # sink with the controller reporting success. Refuse it.
    assert pick_pressure_type(['SCALED_PRESSURE']) is None
    assert pick_pressure_type([]) is None


def test_latch_surface_is_the_median():
    # Median, not mean: one garbage sample from an I2C hiccup must not drag
    # the zero reference with it.
    assert latch_surface([1013.0, 1013.2, 1013.1]) == pytest.approx(1013.1)
    assert latch_surface([1013.0, 1013.0, 1013.0, 5000.0]) == pytest.approx(1013.0)
    assert latch_surface([]) is None


def test_surface_sane_rejects_implausible_latch():
    assert surface_sane(1013.25)
    assert not surface_sane(0.0)
    assert not surface_sane(5000.0)


def test_depth_from_pressure():
    # 1 m of fresh water ≈ 98.07 hPa above the surface reading.
    assert depth_from_pressure(1013.25, 1013.25) == pytest.approx(0.0)
    assert depth_from_pressure(1111.32, 1013.25) == pytest.approx(1.0, abs=1e-3)
    assert depth_from_pressure(1209.38, 1013.25) == pytest.approx(2.0, abs=1e-3)


def test_depth_above_surface_is_negative_not_clamped():
    # Bobbing at the surface legitimately reads slightly negative. Clamping it
    # to 0 would hide a bad surface latch, which is exactly what we want to see.
    assert depth_from_pressure(1003.25, 1013.25) < 0.0


def test_pressure_types_are_the_scaled_pressure_family():
    assert set(PRESSURE_TYPES) == {
        'SCALED_PRESSURE', 'SCALED_PRESSURE2', 'SCALED_PRESSURE3'}
