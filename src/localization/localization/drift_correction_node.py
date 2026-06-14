#!/usr/bin/env python3
"""Drift correction node — resets VIO drift using known path marker positions.

When the bottom camera detects a path marker whose position on the course is
known, this node computes the discrepancy between the expected sub position
and the current VIO estimate, then publishes a corrected pose.

Subscribes:
  - vision/path_markers   (auv_msgs/ObjectDetectionArray) – from bottom camera
  - localization/pose     (geometry_msgs/PoseStamped)      – current fused pose

Publishes:
  - localization/correction (geometry_msgs/PoseStamped) – corrected pose for fusion

Configuration:
  Known marker positions are loaded from the 'marker_map' parameter.
  Format: "label1:x1,y1;label2:x2,y2;..."
  Example: "path_marker_a:0.0,0.0;path_marker_b:3.0,0.0;gate:6.0,0.0"

  The node computes the sub's world position from the marker's known position
  and the marker's offset from frame centre (how far off-centre the marker
  appears tells us how far the sub is from being directly above/in-front of it).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from auv_msgs.msg import ObjectDetectionArray


def _parse_marker_map(s: str) -> dict:
    """Parse 'label:x,y;label:x,y' into {label: (x, y)}."""
    result = {}
    if not s:
        return result
    for entry in s.split(';'):
        entry = entry.strip()
        if not entry or ':' not in entry:
            continue
        label, coords = entry.split(':', 1)
        parts = coords.split(',')
        if len(parts) >= 2:
            result[label.strip()] = (float(parts[0]), float(parts[1]))
    return result


class DriftCorrectionNode(Node):
    def __init__(self):
        super().__init__('drift_correction_node')

        self.declare_parameter('marker_map', '')
        self.declare_parameter('min_confidence', 0.60)
        self.declare_parameter('max_center_offset', 0.25)
        self.declare_parameter('cooldown_s', 2.0)
        self.declare_parameter('fov_h_deg', 110.0)
        # Reject corrections that would jump the sub more than this (metres).
        self.declare_parameter('max_shift_m', 5.0)

        marker_map_str = self.get_parameter('marker_map').value
        self._marker_map = _parse_marker_map(marker_map_str)
        self._min_conf = self.get_parameter('min_confidence').value
        self._max_offset = self.get_parameter('max_center_offset').value
        self._cooldown_s = self.get_parameter('cooldown_s').value
        self._fov_h = math.radians(self.get_parameter('fov_h_deg').value)
        self._max_shift_m = float(self.get_parameter('max_shift_m').value)

        self._last_correction_time = None

        # Current estimated pose (from localization node)
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_z = 0.0
        self._current_yaw = 0.0
        self._pose_received = False

        self.create_subscription(
            ObjectDetectionArray,
            'vision/path_markers',
            self._markers_cb,
            10,
        )
        self.create_subscription(
            PoseStamped,
            'localization/pose',
            self._pose_cb,
            10,
        )

        self._correction_pub = self.create_publisher(
            PoseStamped, 'localization/correction', 10)

        if self._marker_map:
            self.get_logger().info(
                f'Drift correction active — {len(self._marker_map)} markers: '
                f'{list(self._marker_map.keys())}')
        else:
            self.get_logger().warn(
                'No marker_map configured — drift correction will not fire. '
                'Set marker_map param like "path_a:0,0;path_b:3,0"')

    def _pose_cb(self, msg: PoseStamped):
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            q = msg.pose.orientation
            if not all(math.isfinite(v) for v in (x, y, z, q.w, q.x, q.y, q.z)):
                return
            self._current_x = x
            self._current_y = y
            self._current_z = z
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._current_yaw = math.atan2(siny, cosy)
            self._pose_received = True
        except Exception as e:
            self.get_logger().error(f'Pose callback error: {e}')

    def _markers_cb(self, msg: ObjectDetectionArray):
        try:
            self._markers_cb_inner(msg)
        except Exception as e:
            self.get_logger().error(f'Markers callback error: {e}')

    def _markers_cb_inner(self, msg: ObjectDetectionArray):
        if not self._pose_received or not self._marker_map:
            return

        now = self.get_clock().now()
        if self._last_correction_time is not None:
            elapsed = (now - self._last_correction_time).nanoseconds / 1e9
            if elapsed < self._cooldown_s:
                return

        best_det = None
        best_conf = 0.0
        best_world_pos = None

        for det in msg.detections:
            if det.confidence < self._min_conf:
                continue
            if det.label not in self._marker_map:
                continue

            cx_offset = det.position.x - 0.5
            cy_offset = det.position.y - 0.5

            if abs(cx_offset) > self._max_offset:
                continue
            if abs(cy_offset) > self._max_offset:
                continue

            if det.confidence > best_conf:
                best_conf = det.confidence
                best_det = det
                best_world_pos = self._marker_map[det.label]

        if best_det is None or best_world_pos is None:
            return

        marker_wx, marker_wy = best_world_pos

        cx_offset = best_det.position.x - 0.5
        cy_offset = best_det.position.y - 0.5

        # Estimate sub's position from marker's known world position.
        # The marker appears at (cx_offset, cy_offset) from frame centre.
        # For the bottom camera looking down:
        #   cx_offset > 0 means marker is right of centre → sub is left of marker
        #   cy_offset > 0 means marker is below centre → sub is above marker (forward)
        # We use depth to estimate lateral displacement:
        #   displacement ≈ depth * tan(offset * fov/2)
        depth = abs(self._current_z) if abs(self._current_z) > 0.1 else 1.0
        if best_det.position.z > 0:
            depth = best_det.position.z

        dx = -cx_offset * depth * math.tan(self._fov_h / 2.0) * 2.0
        dy = -cy_offset * depth * math.tan(self._fov_h / 2.0) * 2.0

        # Rotate displacement into world frame using current yaw
        cos_yaw = math.cos(self._current_yaw)
        sin_yaw = math.sin(self._current_yaw)
        world_dx = dx * cos_yaw - dy * sin_yaw
        world_dy = dx * sin_yaw + dy * cos_yaw

        corrected_x = marker_wx + world_dx
        corrected_y = marker_wy + world_dy

        # Never feed a non-finite or wildly out-of-range correction into the
        # localization fuser — a single bad value here drifts the whole run.
        if not all(math.isfinite(v) for v in (corrected_x, corrected_y)):
            self.get_logger().warn('Non-finite correction computed — skipping')
            return
        if abs(world_dx) > self._max_shift_m or abs(world_dy) > self._max_shift_m:
            self.get_logger().warn(
                f'Correction shift too large '
                f'(dx={world_dx:.2f} dy={world_dy:.2f}) — skipping')
            return

        correction = PoseStamped()
        correction.header.stamp = now.to_msg()
        correction.header.frame_id = 'odom'
        correction.pose.position.x = corrected_x
        correction.pose.position.y = corrected_y
        correction.pose.position.z = self._current_z

        # Keep current yaw (markers don't give us heading info)
        correction.pose.orientation.w = math.cos(self._current_yaw / 2.0)
        correction.pose.orientation.z = math.sin(self._current_yaw / 2.0)

        self._correction_pub.publish(correction)
        self._last_correction_time = now

        self.get_logger().info(
            f'Drift correction from "{best_det.label}" '
            f'(conf={best_conf:.2f}) — '
            f'corrected to ({corrected_x:.2f}, {corrected_y:.2f}), '
            f'shift=({corrected_x - self._current_x:.3f}, '
            f'{corrected_y - self._current_y:.3f})')


def main():
    rclpy.init()
    node = None
    try:
        node = DriftCorrectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f'Unhandled exception: {e}')
        else:
            print(f'[drift_correction_node] Fatal error: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
