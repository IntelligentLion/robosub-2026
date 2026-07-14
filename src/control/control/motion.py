"""MotionController — the axis mixer, and where "who may command what" is
actually enforced.

  surge, strafe : the operator. The only axes they touch.
  yaw_rate      : the heading lock — UNLESS the operator explicitly asks for
                  yaw, i.e. is deliberately changing heading.
  heave         : the dive. Zero once ALT_HOLD owns depth.
  roll, pitch   : nobody. ALT_HOLD self-levels; commanding them would fight the
                  autopilot's attitude controller. There is no field for them
                  here, so it cannot be done by accident.

Ranges follow MovementCommand: every axis clamped to [-1, 1], +surge forward,
+strafe right, +heave down, +yaw_rate clockwise.
"""
import math
from collections import namedtuple

Axes = namedtuple('Axes', ('surge', 'strafe', 'heave', 'yaw_rate'))


def _clean(v, limit=1.0):
    """Finite and clamped. A NaN reaching MANUAL_CONTROL is undefined behaviour
    at the autopilot, so it becomes 0 here rather than being passed on."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(v):
        return 0.0
    return max(-limit, min(limit, v))


class MotionController:
    STOP = Axes(0.0, 0.0, 0.0, 0.0)

    def __init__(self, max_surge=1.0, max_strafe=1.0):
        self.max_surge = float(max_surge)
        self.max_strafe = float(max_strafe)

    def mix(self, operator_surge, operator_strafe, yaw_correction, heave,
            operator_yaw=None):
        """One control tick's worth of axes.

        `operator_yaw is None` means "I am not steering — heading lock, take the
        wheel". Any number, INCLUDING 0.0, is a real command and overrides the
        lock: summing an operator turn with a correction that opposes it would
        make the lock fight the turn it was just asked to perform.
        """
        yaw = (_clean(yaw_correction) if operator_yaw is None
               else _clean(operator_yaw))
        return Axes(
            surge=_clean(operator_surge, self.max_surge),
            strafe=_clean(operator_strafe, self.max_strafe),
            heave=_clean(heave),
            yaw_rate=yaw,
        )
