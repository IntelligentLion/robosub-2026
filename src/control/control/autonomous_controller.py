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
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from auv_msgs.msg import (
    MovementCommand,
    NavigationCommand,
    ObjectDetectionArray,
)


def _angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


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

    def update(self, error, dt):
        if dt <= 0:
            return 0.0
        if not self._initialized:
            self._prev_error = error
            self._initialized = True

        self._integral += error * dt
        self._integral = max(-self.i_limit, min(self.i_limit, self._integral))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
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

        # PIDs for position/heading control
        self._pid_x = PID(kp=0.8, ki=0.05, kd=0.2, limit=1.0)
        self._pid_y = PID(kp=0.8, ki=0.05, kd=0.2, limit=1.0)
        self._pid_z = PID(kp=1.0, ki=0.1, kd=0.15, limit=1.0)
        self._pid_yaw = PID(kp=1.2, ki=0.08, kd=0.3, limit=1.0)

        # PIDs for vision-based centering
        self._pid_vis_x = PID(kp=2.0, ki=0.0, kd=0.5, limit=0.5)
        self._pid_vis_y = PID(kp=1.5, ki=0.0, kd=0.4, limit=0.4)

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

        self.get_logger().info('Autonomous controller started')

    # ─── Callbacks ──────────────────────────────────────────────────

    def _nav_cmd_cb(self, msg: NavigationCommand):
        new_mode = msg.mode.lower().strip()
        old_mode = self._mode

        self._mode = new_mode
        self._target_label = msg.target_label
        self._target_x = msg.target_x
        self._target_y = msg.target_y
        self._target_z = msg.target_z
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
            self._pid_vis_x.reset()
            self._pid_vis_y.reset()

        if new_mode == 'idle':
            self._send_stop()

        self.get_logger().info(
            f'Nav command: mode={new_mode} label={msg.target_label} '
            f'speed={self._max_speed:.2f}')

    def _vision_cb(self, msg: ObjectDetectionArray):
        self._detections = msg.detections
        self._det_stamp = self.get_clock().now()

    def _pose_cb(self, msg: PoseStamped):
        self._pose_x = msg.pose.position.x
        self._pose_y = msg.pose.position.y
        self._pose_z = msg.pose.position.z
        q = msg.pose.orientation
        self._pose_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose_received = True

    def _depth_cb(self, msg: Float32):
        self._depth_m = msg.data

    # ─── Helpers ────────────────────────────────────────────────────

    def _reset_pids(self):
        self._pid_x.reset()
        self._pid_y.reset()
        self._pid_z.reset()
        self._pid_yaw.reset()

    def _send_cmd(self, command, speed=0.0, duration=0.0):
        msg = MovementCommand()
        msg.command = command
        msg.speed = float(speed)
        msg.duration = float(duration)
        self._cmd_pub.publish(msg)

    def _send_stop(self):
        self._send_cmd('stop')

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
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now

        if dt <= 0 or dt > 1.0:
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
        det = self._best_detection(self._target_label, min_conf=0.50)

        if det is None:
            self._send_stop()
            return

        ex = det.position.x - 0.5
        ey = det.position.y - 0.5
        depth_m = det.position.z

        h_centered = abs(ex) <= self.CENTER_TOL
        v_centered = abs(ey) <= self.CENTER_TOL

        # Check if close enough to stop
        close_enough = False
        if depth_m > 0 and depth_m <= self._approach_dist:
            close_enough = True
        elif depth_m <= 0 and det.bbox_width >= 0.35:
            close_enough = True

        if close_enough and h_centered and v_centered:
            self._send_stop()
            return

        # Horizontal centering via yaw/strafe
        if not h_centered:
            yaw_cmd = self._pid_vis_x.update(ex, dt)
            if abs(ex) > 0.20:
                if yaw_cmd > 0:
                    self._send_cmd('rotate_cw', min(abs(yaw_cmd), self._max_speed))
                else:
                    self._send_cmd('rotate_ccw', min(abs(yaw_cmd), self._max_speed))
            else:
                if yaw_cmd > 0:
                    self._send_cmd('strafe_right', min(abs(yaw_cmd), self._max_speed))
                else:
                    self._send_cmd('strafe_left', min(abs(yaw_cmd), self._max_speed))
            return

        # Vertical centering via depth
        if not v_centered:
            depth_cmd = self._pid_vis_y.update(ey, dt)
            if depth_cmd > 0:
                self._send_cmd('submerge', min(abs(depth_cmd), self._max_speed * 0.6))
            else:
                self._send_cmd('emerge', min(abs(depth_cmd), self._max_speed * 0.6))
            return

        # Centered — approach if not close enough
        if not close_enough:
            if depth_m > 0:
                remaining = max(0.0, depth_m - self._approach_dist)
                speed = max(0.12, min(self._max_speed,
                                      remaining / max(self._approach_dist, 1.0) * 0.5))
            else:
                gap = 0.35 - det.bbox_width
                speed = max(0.12, min(self._max_speed, gap * 3.0))
            self._send_cmd('surge_forward', speed)

    # ─── Search (rotate until found, then track) ───────────────────

    def _tick_search(self, dt):
        det = self._best_detection(self._target_label, min_conf=0.65)

        if det is not None:
            self._search_found = True
            self._mode = 'track_object'
            self._pid_vis_x.reset()
            self._pid_vis_y.reset()
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
    node = AutonomousController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
