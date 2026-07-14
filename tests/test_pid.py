"""PID unit tests — the class is shared by autonomous_controller and
heading_lock, so its behavior is pinned here."""
import math

from control.pid import PID


def test_proportional_only():
    pid = PID(kp=2.0, ki=0.0, kd=0.0)
    # first update initializes prev_error -> derivative 0, integral tiny
    assert pid.update(0.3, 0.05) == 0.3 * 2.0


def test_output_clamped_to_limit():
    pid = PID(kp=100.0, ki=0.0, kd=0.0, limit=0.5)
    assert pid.update(1.0, 0.05) == 0.5
    assert pid.update(-1.0, 0.05) == -0.5


def test_integral_windup_clamped():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, limit=10.0, i_limit=0.2)
    for i in range(100):
        out = pid.update(1.0, 0.1)          # raw integral would reach 10.0
    assert out == 0.2                        # ki * clamped integral


def test_reset_clears_state():
    pid = PID(kp=0.0, ki=1.0, kd=1.0, limit=10.0, i_limit=5.0)
    pid.update(1.0, 0.1)
    pid.update(1.0, 0.1)
    pid.reset()
    # zero error after reset -> exactly zero output (no leftover I or D)
    assert pid.update(0.0, 0.1) == 0.0


def test_set_gains_live():
    pid = PID(kp=1.0, ki=0.0, kd=0.0, limit=10.0)
    pid.set_gains(kp=3.0)
    assert pid.update(1.0, 0.05) == 3.0


def test_nonfinite_error_neutralized():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    assert pid.update(float('nan'), 0.05) == 0.0
    assert pid.update(math.inf, 0.05) == 0.0


def test_bad_dt_neutralized():
    pid = PID(kp=1.0, ki=0.0, kd=0.0)
    assert pid.update(1.0, 0.0) == 0.0       # dt <= 0
    assert pid.update(1.0, 2.0) == 0.0       # dt > 1.0


def test_reexport_from_autonomous_controller():
    # existing code/tools reference the old location; must stay importable
    from control.autonomous_controller import PID as ReExported
    assert ReExported is PID
