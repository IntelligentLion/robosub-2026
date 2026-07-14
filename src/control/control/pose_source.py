"""Where the vehicle is — and how much to trust that answer.

**A Bar02 and a Pixhawk IMU cannot determine XY position.** There is no DVL, no
GPS, no USBL. Integrating IMU acceleration twice produces a position that drifts
without bound within seconds, so we do not pretend otherwise: the default source
dead-reckons from the *commanded* velocity through the measured heading, and
reports `is_estimate() == True` so the visualizer can label it honestly.

Depth is the exception. It is measured, not estimated, and it is exact.

One interface, so a real localization source (DVL, EKF, visual SLAM) drops in
later as a third implementation with no change to any controller.

Pure: no ROS types in, no ROS types out. Just numbers.
"""
import math
from abc import ABC, abstractmethod


class PoseSource(ABC):
    """(x, y, z, yaw) in the odom frame. Metres and radians; +z is DOWN, to
    match depth."""

    @abstractmethod
    def pose(self):
        """(x, y, z, yaw), or None if no fix yet."""

    @abstractmethod
    def is_estimate(self):
        """True if XY is dead-reckoned and therefore drifting. The visualizer
        renders an estimate differently and says so on screen — a drifting
        position drawn as if it were measured is worse than no position."""

    def reset(self):
        """Return to the origin. Called when a new dive starts."""


class DeadReckonPose(PoseSource):
    """Integrate the COMMANDED velocity through the MEASURED heading.

    Commanded, not measured, because there is nothing to measure XY velocity
    with. `surge_scale` / `strafe_scale` convert a normalized MovementCommand
    axis (-1..1) into m/s; they are guesses about thruster authority and pool
    drag, so the resulting track is qualitative. It shows the SHAPE of the path
    — "did the heading lock keep us straight?" — not where the sub actually is.
    Current, wall wash and thrust nonlinearity all go unmodelled.

    Depth comes straight from the Bar02, so z is real even though x and y are not.
    """

    def __init__(self, surge_scale=0.5, strafe_scale=0.4):
        self.surge_scale = float(surge_scale)
        self.strafe_scale = float(strafe_scale)
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._yaw = 0.0
        self._have_fix = False

    def reset(self):
        self._x = self._y = self._z = 0.0
        self._have_fix = False

    def update(self, surge_cmd, strafe_cmd, yaw_rad, depth_m, dt):
        """One integration step. yaw_rad/depth_m may be None (stale)."""
        if dt <= 0.0 or dt > 1.0:      # a huge dt means we stalled; do not leap
            return
        if yaw_rad is not None and math.isfinite(yaw_rad):
            self._yaw = yaw_rad
        if depth_m is not None and math.isfinite(depth_m):
            self._z = depth_m
            self._have_fix = True

        vx = float(surge_cmd) * self.surge_scale
        vy = float(strafe_cmd) * self.strafe_scale
        # Body → odom. +surge is forward along the heading, +strafe is to the
        # right of it (90° clockwise), matching MovementCommand.
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._x += (vx * c + vy * s) * dt
        self._y += (vx * s - vy * c) * dt

    def pose(self):
        if not self._have_fix:
            return None
        return (self._x, self._y, self._z, self._yaw)

    def is_estimate(self):
        return True


class ZedOdomPose(PoseSource):
    """Relay a real odometry source (the ZED VSLAM already running in this
    stack). XY is measured, so is_estimate() is False."""

    def __init__(self):
        self._pose = None

    def set_pose(self, x, y, z, yaw):
        self._pose = (float(x), float(y), float(z), float(yaw))

    def reset(self):
        self._pose = None

    def pose(self):
        return self._pose

    def is_estimate(self):
        return False
