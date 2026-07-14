"""Unit tests for imu.imu_math — pure quaternion helpers, no ROS needed."""
import math
import os
import sys

# imu_math lives in the (unbuilt) ROS package; import it straight from source
# so this runs without a sourced colcon workspace.
_PKG = os.path.join(os.path.dirname(__file__), '..', 'src', 'imu')
sys.path.insert(0, os.path.abspath(_PKG))

from imu.imu_math import (  # noqa: E402
    normalize, quat_multiply, quat_conjugate, quat_inverse,
    quat_relative, quat_average, euler_from_quat, rotate_vector,
)

IDENT = (0.0, 0.0, 0.0, 1.0)


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_normalize_unit_stays_unit():
    assert _close(normalize(IDENT), IDENT)


def test_normalize_scales_to_unit_length():
    q = normalize((0.0, 0.0, 0.0, 2.0))
    assert _close(q, IDENT)


def test_normalize_zero_returns_identity():
    assert _close(normalize((0.0, 0.0, 0.0, 0.0)), IDENT)


def test_multiply_identity_is_noop():
    q = normalize((0.1, 0.2, 0.3, 0.9))
    assert _close(quat_multiply(q, IDENT), q)
    assert _close(quat_multiply(IDENT, q), q)


def test_conjugate_flips_vector_part():
    assert _close(quat_conjugate((0.1, 0.2, 0.3, 0.9)), (-0.1, -0.2, -0.3, 0.9))


def test_inverse_times_self_is_identity():
    q = normalize((0.1, -0.2, 0.4, 0.8))
    assert _close(quat_multiply(quat_inverse(q), q), IDENT, tol=1e-9)


def test_relative_of_equal_is_identity():
    # Zeroing: current == reference must read as no rotation.
    q = normalize((0.2, 0.1, -0.3, 0.9))
    assert _close(quat_relative(q, q), IDENT, tol=1e-9)


def test_relative_recovers_delta():
    # q_cur = q_ref ⊗ q_delta  =>  quat_relative(q_ref, q_cur) == q_delta
    q_ref = normalize((0.0, 0.0, math.sin(0.3), math.cos(0.3)))
    q_delta = normalize((0.0, 0.0, math.sin(0.5), math.cos(0.5)))
    q_cur = quat_multiply(q_ref, q_delta)
    assert _close(quat_relative(q_ref, q_cur), q_delta, tol=1e-9)


def test_average_of_identicals_is_that_quat():
    q = normalize((0.1, 0.2, 0.2, 0.95))
    assert _close(quat_average([q, q, q]), q, tol=1e-9)


def test_average_handles_sign_flips():
    # q and -q are the same rotation; averaging must not cancel to zero.
    q = normalize((0.1, 0.2, 0.2, 0.95))
    nq = tuple(-c for c in q)
    avg = quat_average([q, nq, q])
    assert _close(avg, q, tol=1e-9) or _close(avg, nq, tol=1e-9)


def test_euler_identity_is_zero():
    r, p, y = euler_from_quat(IDENT)
    assert _close((r, p, y), (0.0, 0.0, 0.0))


def test_euler_yaw_90deg():
    # +90deg about Z
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    r, p, y = euler_from_quat(q)
    assert abs(y - math.pi / 2) < 1e-6
    assert abs(r) < 1e-6 and abs(p) < 1e-6


def test_euler_pitch_clamped_at_singularity():
    # +90deg about Y — asin argument must not exceed 1.0 and blow up.
    q = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
    r, p, y = euler_from_quat(q)
    assert abs(p - math.pi / 2) < 1e-6


def test_rotate_vector_identity_noop():
    assert _close(rotate_vector(IDENT, (1.0, 2.0, 3.0)), (1.0, 2.0, 3.0))


def test_rotate_vector_yaw_90_maps_x_to_y():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    x, y, z = rotate_vector(q, (1.0, 0.0, 0.0))
    assert abs(x) < 1e-6 and abs(y - 1.0) < 1e-6 and abs(z) < 1e-6
