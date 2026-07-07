#!/usr/bin/env python3
"""Autonomous controller — fuses vision + localization into movement commands.

Subscribes:
  - navigation_command     (auv_msgs/NavigationCommand)  — high-level goals
  - vision/detections      (auv_msgs/ObjectDetectionArray) — camera detections
  - localization/pose      (geometry_msgs/PoseStamped)   — fused sub pose
  - depth/sub_depth        (std_msgs/Float32)            — depth below surface

Publishes:
  - movement_command       (auv_msgs/MovementCommand)    — low-level thruster cmds

Modes:
  idle          — all-stop, no active control
  station_keep  — PID hold on current position + heading
  track_object  — vision-guided center + approach on a detected object
  search        — rotate until target found, then auto-switch to track
  waypoint      — PID navigate to a world-frame position
  heading_hold  — PID hold yaw only
"""

import math
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from auv_msgs.msg import (
    MovementCommand,
    NavigationCommand,
    ObjectDetectionArray,
)
from control.centering import (
    CenteringPolicy,
    DefaultPolicy,
    TargetState,
    TargetTracker,
    centering_errors,
    clamp,
    policy_for,
    shape_surge,
)


def _angle_diff(a, b):
    d = math.atan2(math.sin(a - b), math.cos(a - b))
    return d


def _is_finite(v):
    return math.isfinite(v)


def _pf(value, default):
    """Coerce a ROS parameter value to float, falling back to default.

    rclpy's ``get_parameter(...).value`` is typed ``Unknown | None`` (and can
    be None if a param isn't declared), so a bare ``float(...)`` trips the type
    checker and could raise at runtime on a misconfigured launch. This makes
    param reads total + robust.
    """
    try:
        return float(value) if value is not None else float(default)
    except (TypeError, ValueError):
        return float(default)


MAX_DEPTH_M = 4.5
MIN_DEPTH_M = 0.0
MAX_POSITION_M = 50.0
SEARCH_TIMEOUT_S = 60.0


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


class AutonomousController(Node):
    CONTROL_HZ = 10.0
    DEFAULT_APPROACH_DIST = 1.5
    CENTER_TOL = 0.08
    SEARCH_ROTATE_SPEED = 0.25
    POSITION_REACHED_TOL = 0.3
    YAW_REACHED_TOL = 0.10
    DETECTION_STALE_S = 2.0

    def __init__(self):
        super().__init__('autonomous_controller')

        # --- Camera + tracker parameters (tunable at runtime) ---
        # ZED 2i HFOV/VFOV at HD720; used to turn image bearing + range into
        # metric lateral/vertical offsets (see centering.TargetState).
        self.declare_parameter('hfov_deg', 110.0)
        self.declare_parameter('vfov_deg', 70.0)
        self.declare_parameter('ema_alpha', 0.3)   # weight on the new sample
        self.declare_parameter('coast_s', 0.6)      # hold through dropouts (s)
        self._hfov_rad = math.radians(
            _pf(self.get_parameter('hfov_deg').value, 110.0))
        self._vfov_rad = math.radians(
            _pf(self.get_parameter('vfov_deg').value, 70.0))

        # --- State ---
        self._mode = 'idle'
        self._target_label = ''
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 0.0
        self._target_yaw = 0.0
        self._max_speed = 0.5
        self._approach_dist = self.DEFAULT_APPROACH_DIST

        # Localization state
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_z = 0.0
        self._pose_yaw = 0.0
        self._pose_received = False
        self._depth_m = -1.0

        # Vision state
        self._detections = []
        self._det_stamp = self.get_clock().now()

        # Station-keep anchors (set when entering station_keep)
        self._anchor_x = 0.0
        self._anchor_y = 0.0
        self._anchor_z = 0.0
        self._anchor_yaw = 0.0

        # Search state
        self._search_found = False
        self._search_start_time = None

        # PIDs — all gains are ROS parameters (pid_<name>.kp/ki/kd/limit/
        # i_limit) so they can be tuned LIVE at the pool via
        #   ros2 param set /autonomous_controller pid_yaw.kp 1.4
        # without recompiling or restarting. The on-set-parameters callback
        # (registered below) routes changes to the PID objects immediately.
        #
        # Position/heading (station_keep / waypoint / heading_hold):
        self._pid_x   = self._declare_pid('pid_x',   0.8, 0.05, 0.2,  limit=1.0)
        self._pid_y   = self._declare_pid('pid_y',   0.8, 0.05, 0.2,  limit=1.0)
        self._pid_z   = self._declare_pid('pid_z',   1.0, 0.1,  0.15, limit=1.0)
        self._pid_yaw = self._declare_pid('pid_yaw', 1.2, 0.08, 0.3,  limit=1.0)
        # Vision centering (track_object): yaw nulls the bearing; strafe nulls
        # the metric lateral offset; depth nulls the metric vertical offset.
        # Surge is shaped directly in centering.shape_surge().
        self._pid_vis_yaw    = self._declare_pid('pid_vis_yaw',    1.5, 0.0, 0.3, limit=0.5)
        self._pid_vis_strafe = self._declare_pid('pid_vis_strafe', 0.8, 0.0, 0.2, limit=0.4)
        self._pid_vis_depth  = self._declare_pid('pid_vis_depth',  1.0, 0.0, 0.2, limit=0.4)

        self._pid_map = {
            'pid_x': self._pid_x, 'pid_y': self._pid_y,
            'pid_z': self._pid_z, 'pid_yaw': self._pid_yaw,
            'pid_vis_yaw': self._pid_vis_yaw,
            'pid_vis_strafe': self._pid_vis_strafe,
            'pid_vis_depth': self._pid_vis_depth,
        }

        # Centering framework state (set up when entering track_object).
        self._policy: CenteringPolicy = DefaultPolicy()
        self._tracker = TargetTracker(
            alpha=_pf(self.get_parameter('ema_alpha').value, 0.3),
            coast_s=_pf(self.get_parameter('coast_s').value, 0.6),
        )
        self._converge_count = 0

        self._last_tick = self.get_clock().now()

        # --- Publishers ---
        self._cmd_pub = self.create_publisher(MovementCommand, 'movement_command', 10)

        # --- Subscribers ---
        self.create_subscription(
            NavigationCommand, 'navigation_command', self._nav_cmd_cb, 10)
        self.create_subscription(
            ObjectDetectionArray, 'vision/detections', self._vision_cb, 10)
        self.create_subscription(
            PoseStamped, 'localization/pose', self._pose_cb, 10)
        self.create_subscription(
            Float32, 'depth/sub_depth', self._depth_cb, 10)

        # --- Control loop ---
        self.create_timer(1.0 / self.CONTROL_HZ, self._control_tick)
        # Live PID/camera tuning: route `ros2 param set` changes to objects.
        self.add_on_set_parameters_callback(self._on_params_changed)

        self.get_logger().info('Autonomous controller started')

    # ─── Callbacks ──────────────────────────────────────────────────

    def _nav_cmd_cb(self, msg: NavigationCommand):
        try:
            new_mode = msg.mode.lower().strip()
            valid_modes = ('idle', 'station_keep', 'track_object',
                           'search', 'waypoint', 'heading_hold')
            if new_mode not in valid_modes:
                self.get_logger().warn(f'Unknown mode "{new_mode}" — forcing idle')
                new_mode = 'idle'

            old_mode = self._mode
            # Clear any axis state persisted from the previous mode (e.g. an
            # `axes` setpoint leaves surge/strafe engaged until overwritten).
            if old_mode != new_mode:
                self._send_stop()

            self._mode = new_mode
            self._target_label = msg.target_label
            self._target_x = max(-MAX_POSITION_M, min(MAX_POSITION_M, msg.target_x))
            self._target_y = max(-MAX_POSITION_M, min(MAX_POSITION_M, msg.target_y))
            self._target_z = max(MIN_DEPTH_M, min(MAX_DEPTH_M, msg.target_z))
            self._target_yaw = msg.target_yaw
            self._max_speed = max(0.05, min(1.0, msg.speed)) if msg.speed > 0 else 0.5
            self._approach_dist = msg.approach_dist if msg.approach_dist > 0 else self.DEFAULT_APPROACH_DIST

            if new_mode == 'station_keep' and old_mode != 'station_keep':
                self._anchor_x = self._pose_x
                self._anchor_y = self._pose_y
                self._anchor_z = self._pose_z
                self._anchor_yaw = self._pose_yaw
                self._reset_pids()

            if new_mode == 'waypoint' and old_mode != 'waypoint':
                self._reset_pids()

            if new_mode in ('search', 'track_object'):
                self._search_found = False
                self._search_start_time = self.get_clock().now()
                if new_mode == 'track_object':
                    self._enter_track_object()

            if new_mode == 'idle':
                self._send_stop()

            self.get_logger().info(
                f'Nav command: mode={new_mode} label={msg.target_label} '
                f'speed={self._max_speed:.2f}')
        except Exception as e:
            self.get_logger().error(f'Nav callback error: {e}')
            self._mode = 'idle'
            self._send_stop()

    def _vision_cb(self, msg: ObjectDetectionArray):
        try:
            self._detections = msg.detections
            self._det_stamp = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f'Vision callback error: {e}')

    def _pose_cb(self, msg: PoseStamped):
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            q = msg.pose.orientation
            if not all(_is_finite(v) for v in (x, y, z, q.w, q.x, q.y, q.z)):
                self.get_logger().warn('Non-finite pose data — ignoring')
                return
            self._pose_x = max(-MAX_POSITION_M, min(MAX_POSITION_M, x))
            self._pose_y = max(-MAX_POSITION_M, min(MAX_POSITION_M, y))
            self._pose_z = max(-MAX_DEPTH_M, min(MAX_DEPTH_M, z))
            self._pose_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._pose_received = True
        except Exception as e:
            self.get_logger().error(f'Pose callback error: {e}')

    def _depth_cb(self, msg: Float32):
        try:
            if _is_finite(msg.data):
                self._depth_m = msg.data
            else:
                self.get_logger().warn('Non-finite depth — ignoring')
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    # ─── Helpers ────────────────────────────────────────────────────

    def _declare_pid(self, prefix, kp, ki, kd, limit=1.0, i_limit=0.3):
        """Declare kp/ki/kd/limit/i_limit params for `prefix` and build a PID.

        Gains are re-read live by :meth:`_on_params_changed`, so
        ``ros2 param set /autonomous_controller pid_yaw.kp 1.4`` takes effect
        immediately without restarting the node. Defaults below are the
        hand-tuned starting points; refine them at the pool with tune_pid.py.
        """
        self.declare_parameter(f'{prefix}.kp', kp)
        self.declare_parameter(f'{prefix}.ki', ki)
        self.declare_parameter(f'{prefix}.kd', kd)
        self.declare_parameter(f'{prefix}.limit', limit)
        self.declare_parameter(f'{prefix}.i_limit', i_limit)
        return self._build_pid(prefix)

    def _build_pid(self, prefix):
        """Read the current params for `prefix` into a fresh PID."""
        def g(field, default):
            return _pf(self.get_parameter(f'{prefix}.{field}').value, default)
        return PID(
            kp=g('kp', 0.0), ki=g('ki', 0.0), kd=g('kd', 0.0),
            limit=g('limit', 1.0), i_limit=g('i_limit', 0.3),
        )

    def _on_params_changed(self, params):
        """Live-tune PID gains + camera FOV from `ros2 param set`.

        Called by rclpy whenever parameters change (via ``ros2 param set`` or
        a launch override). Routes ``pid_<name>.<field>`` to the matching PID
        and refreshes the camera FOV / tracker smoothing. Returns success so the
        new values are committed.
        """
        result = SetParametersResult(successful=True)
        for p in params:
            name = p.name
            val = p.value
            if name.startswith('pid_') and '.' in name:
                prefix, field = name.split('.', 1)
                pid = self._pid_map.get(prefix)
                if pid is None or field not in ('kp', 'ki', 'kd', 'limit', 'i_limit'):
                    continue
                try:
                    setattr(pid, field, float(val))
                except (TypeError, ValueError):
                    pass
            elif name == 'hfov_deg':
                self._hfov_rad = math.radians(_pf(val, 110.0))
            elif name == 'vfov_deg':
                self._vfov_rad = math.radians(_pf(val, 70.0))
            elif name == 'ema_alpha':
                self._tracker.alpha = max(0.01, min(1.0, _pf(val, 0.3)))
            elif name == 'coast_s':
                self._tracker.coast_s = max(0.0, _pf(val, 0.6))
        return result

    def _reset_pids(self):
        self._pid_x.reset()
        self._pid_y.reset()
        self._pid_z.reset()
        self._pid_yaw.reset()

    def _send_cmd(self, command, speed=0.0, duration=0.0):
        msg = MovementCommand()
        msg.command = command
        try:
            msg.speed = float(speed)
            msg.duration = float(duration)
        except (TypeError, ValueError):
            # Non-numeric speed/duration (e.g. an unconfigured BT input port)
            # — fall back to a safe neutral command rather than crashing the
            # publisher callback, which would stop the sub.
            msg.speed = 0.0
            msg.duration = 0.0
        self._cmd_pub.publish(msg)

    def _send_stop(self):
        self._send_cmd('stop')

    def _enter_track_object(self):
        """Set up centering state when entering track_object.

        Called from both _nav_cmd_cb (direct) and _tick_search (when a search
        acquires the target), so the policy + tracker + PIDs are consistently
        initialised on every entry into vision-guided centering.
        """
        self._pid_vis_yaw.reset()
        self._pid_vis_strafe.reset()
        self._pid_vis_depth.reset()
        self._tracker.reset()
        self._converge_count = 0
        self._policy = policy_for(self._target_label, self._approach_dist)

    def _dispatch_setpoint(self, surge, strafe, heave, yaw_rate):
        """Dispatch a simultaneous 4-axis setpoint (command='axes').

        The vision centerer uses this so yaw + strafe + depth + surge converge
        together instead of one axis per tick. Each axis is clamped to [-1, 1];
        the thruster applies all four every control tick (10 Hz).
        """
        msg = MovementCommand()
        msg.command = 'axes'
        msg.speed = 0.0
        msg.duration = 0.0
        msg.surge = clamp(surge, 1.0)
        msg.strafe = clamp(strafe, 1.0)
        msg.heave = clamp(heave, 1.0)
        msg.yaw_rate = clamp(yaw_rate, 1.0)
        self._cmd_pub.publish(msg)

    def _best_detection(self, label, min_conf=0.5):
        age = (self.get_clock().now() - self._det_stamp).nanoseconds / 1e9
        if age > self.DETECTION_STALE_S:
            return None
        best = None
        for d in self._detections:
            if d.label == label and d.confidence >= min_conf:
                if best is None or d.confidence > best.confidence:
                    best = d
        return best

    # ─── Main control loop ─────────────────────────────────────────

    def _control_tick(self):
        try:
            now = self.get_clock().now()
            dt = (now - self._last_tick).nanoseconds / 1e9
            self._last_tick = now

            if dt <= 0 or dt > 1.0:
                return

            if self._depth_m > MAX_DEPTH_M:
                self.get_logger().error(
                    f'DEPTH SAFETY: {self._depth_m:.2f}m > {MAX_DEPTH_M}m — surfacing')
                self._mode = 'idle'
                self._send_cmd('emerge', 0.6)
                return

            if self._mode == 'idle':
                return
            elif self._mode == 'station_keep':
                self._tick_station_keep(dt)
            elif self._mode == 'track_object':
                self._tick_track_object(dt)
            elif self._mode == 'search':
                self._tick_search(dt)
            elif self._mode == 'waypoint':
                self._tick_waypoint(dt)
            elif self._mode == 'heading_hold':
                self._tick_heading_hold(dt)
            else:
                self.get_logger().warn(f'Unknown mode "{self._mode}" — stopping')
                self._mode = 'idle'
                self._send_stop()
        except Exception as e:
            self.get_logger().error(f'Control tick error: {e} — stopping')
            self._send_stop()
            self._mode = 'idle'

    # ─── Station keep ──────────────────────────────────────────────

    def _tick_station_keep(self, dt):
        if not self._pose_received:
            self._send_stop()
            return

        err_yaw = _angle_diff(self._anchor_yaw, self._pose_yaw)
        yaw_cmd = self._pid_yaw.update(err_yaw, dt)

        dx = self._anchor_x - self._pose_x
        dy = self._anchor_y - self._pose_y
        cos_yaw = math.cos(self._pose_yaw)
        sin_yaw = math.sin(self._pose_yaw)
        err_fwd = dx * cos_yaw + dy * sin_yaw
        err_lat = -dx * sin_yaw + dy * cos_yaw

        surge_cmd = self._pid_x.update(err_fwd, dt)
        strafe_cmd = self._pid_y.update(err_lat, dt)

        err_z = self._anchor_z - self._pose_z
        depth_cmd = self._pid_z.update(err_z, dt)

        self._dispatch_axes(surge_cmd, strafe_cmd, depth_cmd, yaw_cmd)

    # ─── Track object (vision-guided) ──────────────────────────────

    def _tick_track_object(self, dt):
        now = time.monotonic()

        det = self._best_detection(self._target_label, min_conf=0.50)
        if det is not None:
            self._tracker.update(det, now, self._target_label)

        state = self._tracker.state(now)
        if state is None:
            # No target and coast window expired — hold position (depth-hold).
            self._dispatch_setpoint(0.0, 0.0, 0.0, 0.0)
            self._converge_count = 0
            return

        errs = centering_errors(self._policy, state,
                                self._hfov_rad, self._vfov_rad)

        lim = self._max_speed
        yaw_cmd = (self._pid_vis_yaw.update(errs.yaw_err, dt)
                   if self._policy.use_yaw else 0.0)
        strafe_cmd = (self._pid_vis_strafe.update(errs.strafe_err, dt)
                      if self._policy.use_strafe else 0.0)
        depth_cmd = (self._pid_vis_depth.update(errs.depth_err, dt)
                     if self._policy.use_depth else 0.0)
        surge_cmd = shape_surge(self._policy, errs)

        # Depth and strafe run a bit slower than surge/yaw for stability.
        yaw_cmd = clamp(yaw_cmd, lim)
        strafe_cmd = clamp(strafe_cmd, lim * 0.7)
        depth_cmd = clamp(depth_cmd, lim * 0.6)
        surge_cmd = clamp(surge_cmd, lim)

        if errs.centered:
            self._converge_count += 1
        else:
            self._converge_count = 0

        if self._converge_count >= self._policy.converge_ticks:
            # Centered + in range, confirmed for N ticks — hold here.
            self._dispatch_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        self._dispatch_setpoint(surge_cmd, strafe_cmd, depth_cmd, yaw_cmd)

    # ─── Search (rotate until found, then track) ───────────────────

    def _tick_search(self, dt):
        if self._search_start_time is not None:
            elapsed = (self.get_clock().now() - self._search_start_time).nanoseconds / 1e9
            if elapsed > SEARCH_TIMEOUT_S:
                self.get_logger().warn(
                    f'Search timeout ({SEARCH_TIMEOUT_S}s) — switching to station_keep')
                self._mode = 'station_keep'
                self._anchor_x = self._pose_x
                self._anchor_y = self._pose_y
                self._anchor_z = self._pose_z
                self._anchor_yaw = self._pose_yaw
                self._reset_pids()
                self._send_stop()
                return

        det = self._best_detection(self._target_label, min_conf=0.65)

        if det is not None:
            self._search_found = True
            self._mode = 'track_object'
            self._enter_track_object()
            self.get_logger().info(
                f'Search found {self._target_label} — switching to track')
            self._send_stop()
            return

        self._send_cmd('rotate_cw', self.SEARCH_ROTATE_SPEED)

    # ─── Waypoint navigation ──────────────────────────────────────

    def _tick_waypoint(self, dt):
        if not self._pose_received:
            self._send_stop()
            return

        dx = self._target_x - self._pose_x
        dy = self._target_y - self._pose_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Yaw to target
        desired_yaw = math.atan2(dy, dx)
        err_yaw = _angle_diff(desired_yaw, self._pose_yaw)

        # Depth
        err_z = (-self._target_z) - self._pose_z
        depth_cmd = self._pid_z.update(err_z, dt)

        # If we've arrived horizontally
        if dist < self.POSITION_REACHED_TOL:
            target_yaw_err = _angle_diff(self._target_yaw, self._pose_yaw)
            yaw_cmd = self._pid_yaw.update(target_yaw_err, dt)

            if abs(target_yaw_err) < self.YAW_REACHED_TOL and abs(err_z) < 0.2:
                self._send_stop()
                self.get_logger().info('Waypoint reached — stopping')
                self._mode = 'station_keep'
                self._anchor_x = self._pose_x
                self._anchor_y = self._pose_y
                self._anchor_z = self._pose_z
                self._anchor_yaw = self._target_yaw
                self._reset_pids()
                return

            self._dispatch_axes(0.0, 0.0, depth_cmd, yaw_cmd)
            return

        # Transform to body frame
        cos_yaw = math.cos(self._pose_yaw)
        sin_yaw = math.sin(self._pose_yaw)
        err_fwd = dx * cos_yaw + dy * sin_yaw
        err_lat = -dx * sin_yaw + dy * cos_yaw

        # Turn first if way off heading
        if abs(err_yaw) > 0.4:
            yaw_cmd = self._pid_yaw.update(err_yaw, dt)
            self._dispatch_axes(0.0, 0.0, depth_cmd, yaw_cmd)
            return

        yaw_cmd = self._pid_yaw.update(err_yaw, dt)
        surge_speed = max(0.1, min(self._max_speed, dist * 0.5))
        surge_cmd = surge_speed if err_fwd > 0 else -surge_speed
        strafe_cmd = self._pid_y.update(err_lat, dt) * 0.5

        self._dispatch_axes(surge_cmd, strafe_cmd, depth_cmd, yaw_cmd)

    # ─── Heading hold ──────────────────────────────────────────────

    def _tick_heading_hold(self, dt):
        if not self._pose_received:
            return

        err_yaw = _angle_diff(self._target_yaw, self._pose_yaw)
        yaw_cmd = self._pid_yaw.update(err_yaw, dt)

        if abs(err_yaw) > 0.05:
            if yaw_cmd > 0:
                self._send_cmd('rotate_cw', min(abs(yaw_cmd), self._max_speed))
            else:
                self._send_cmd('rotate_ccw', min(abs(yaw_cmd), self._max_speed))
        else:
            self._send_cmd('depth_hold')

    # ─── Axis dispatch ─────────────────────────────────────────────

    def _dispatch_axes(self, surge, strafe, depth, yaw):
        axes = [
            (abs(yaw), yaw, 'rotate_cw', 'rotate_ccw'),
            (abs(surge), surge, 'surge_forward', 'surge_backward'),
            (abs(strafe), strafe, 'strafe_right', 'strafe_left'),
            (abs(depth), depth, 'submerge', 'emerge'),
        ]

        axes.sort(key=lambda a: a[0], reverse=True)

        dominant = axes[0]
        mag, val, pos_cmd, neg_cmd = dominant

        if mag < 0.03:
            self._send_stop()
            return

        speed = min(mag, self._max_speed)
        if val >= 0:
            self._send_cmd(pos_cmd, speed)
        else:
            self._send_cmd(neg_cmd, speed)


def main():
    rclpy.init()
    node = None
    try:
        node = AutonomousController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f'Unhandled exception: {e}')
        else:
            print(f'[autonomous_controller] Fatal startup error: {e}')
    finally:
        if node:
            try:
                node._send_stop()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
