"""Shared PID with output limit + integral anti-windup.

Extracted verbatim from autonomous_controller (2026-07-14) so heading_lock
can reuse it. Semantics unchanged: derivative clamped to ±10, integral
clamped to ±i_limit, output clamped to ±limit, non-finite inputs/outputs
reset the controller and return 0.
"""
import math


def _is_finite(v):
    return math.isfinite(v)


class PID:
    def __init__(self, kp, ki, kd, limit=1.0, i_limit=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.i_limit = i_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def set_gains(self, kp=None, ki=None, kd=None, limit=None, i_limit=None):
        """Live-update gains (called from the on-set-parameters callback).

        Any of kp/ki/kd/limit/i_limit may be None to leave unchanged. This
        lets you tune at the pool via `ros2 param set` without restarting.
        """
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd
        if limit is not None: self.limit = limit
        if i_limit is not None: self.i_limit = i_limit

    def update(self, error, dt):
        if dt <= 0 or dt > 1.0:
            return 0.0
        if not _is_finite(error):
            self.reset()
            return 0.0
        if not self._initialized:
            self._prev_error = error
            self._initialized = True

        self._integral += error * dt
        self._integral = max(-self.i_limit, min(self.i_limit, self._integral))

        derivative = (error - self._prev_error) / dt
        derivative = max(-10.0, min(10.0, derivative))
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        if not _is_finite(output):
            self.reset()
            return 0.0
        return max(-self.limit, min(self.limit, output))
