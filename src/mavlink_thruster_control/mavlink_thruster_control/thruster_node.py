"""
Modular Thruster Controller
============================
Subscribes to ``auv_msgs/msg/MovementCommand`` on the ``movement_command``
topic and translates high-level commands (submerge, surge_forward, rotate_cw,
…) into MAVLink ``manual_control`` messages sent to a Pixhawk flight
controller.

Features
--------
* **Auto-detect serial port** – tries the configured port first, then scans
  ``/dev/ttyACM*`` and ``/dev/ttyUSB*``.
* **Simulation mode** – if no hardware is found (or ``simulate`` param is
  ``True``), the node keeps running so the rest of the stack is unaffected.
* **Duration support** – commands with ``duration > 0`` automatically stop
  after the specified time.
* **10 Hz control loop** – current axis values are continuously sent to the
  flight controller so the thrusters stay active.
* **1 Hz heartbeat** – keeps ArduSub GCS failsafe from disarming.
* **Watchdog** – auto-stops if no new commands arrive within a configurable
  timeout (default 30 s).
* **Auto-reconnect** – re-establishes MAVLink link and re-arms on serial
  errors or unexpected disarms.
"""

import glob
import math
import time as _time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from auv_msgs.msg import MovementCommand

try:
    from pymavlink import mavutil
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False


class ThrusterController(Node):
    """Modular movement controller mapping MovementCommand → MAVLink axes."""

    # ── Safety constants ────────────────────────────────────────────────
    MAX_CONSECUTIVE_ERRORS = 5      # serial errors before reconnect attempt
    MAX_RECONNECT_ATTEMPTS = 5      # give up after this many consecutive reconnects
    HEARTBEAT_INTERVAL_S = 1.0      # send heartbeat every 1 s
    ARMED_CHECK_INTERVAL_S = 5.0    # verify armed status every 5 s
    DEFAULT_WATCHDOG_S = 30.0       # stop if no command received for this long

    def __init__(self):
        super().__init__('thruster_controller')

        # ── ROS parameters ──────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('simulate', False)
        self.declare_parameter('watchdog_timeout', self.DEFAULT_WATCHDOG_S)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.simulate = self.get_parameter('simulate').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

        # ── MAVLink connection ──────────────────────────────────────────
        self.master = None
        self.connected = False
        self._consecutive_errors = 0
        self._reconnecting = False
        self._reconnect_attempts = 0

        if not HAS_MAVLINK:
            self.get_logger().warn(
                'pymavlink not installed – running in SIMULATION mode')
            self.simulate = True

        if not self.simulate:
            self._connect_mavlink()
        else:
            self.get_logger().warn(
                'Running in SIMULATION mode – no thruster output')

        # ── Movement state (MAVLink axes) ───────────────────────────────
        self.current_x = 0      # surge   (-1000 … 1000)
        self.current_y = 0      # strafe  (-1000 … 1000)
        self.current_z = 500    # depth   (0 … 1000, 500 = neutral)
        self.current_r = 0      # yaw     (-1000 … 1000)

        # ── Duration tracking ───────────────────────────────────────────
        self._stop_time = None          # auto-stop deadline
        self._last_cmd_time = None      # watchdog: last command timestamp
        self._watchdog_triggered = False
        self._loop_count = 0            # for periodic debug logging
        self._last_log_key = None       # de-dupe repeated Movement log lines

        # ── ROS subscriptions & timers ──────────────────────────────────
        self.create_subscription(
            MovementCommand, 'movement_command', self._movement_cb, 10)

        self.create_timer(0.1, self._control_loop)                # 10 Hz
        self.create_timer(self.HEARTBEAT_INTERVAL_S,
                          self._heartbeat_loop)                    #  1 Hz
        self.create_timer(self.ARMED_CHECK_INTERVAL_S,
                          self._check_armed_status)                # 0.2 Hz

        self.get_logger().info('Thruster controller initialized')

    # ─── MAVLink helpers ────────────────────────────────────────────────

    def _connect_mavlink(self):
        """Try the configured port, then auto-detect others."""
        candidates = [self.serial_port]
        for pattern in ('/dev/ttyACM*', '/dev/ttyUSB*'):
            candidates.extend(sorted(glob.glob(pattern)))

        # deduplicate, preserve order
        seen: set = set()
        unique: list = []
        for p in candidates:
            if p not in seen:
                seen.add(p)
                unique.append(p)

        for port in unique:
            try:
                self.get_logger().info(f'Trying MAVLink on {port} …')
                self.master = mavutil.mavlink_connection(
                    port, baud=self.baud_rate)
                self.master.wait_heartbeat(timeout=5)
                self.get_logger().info(
                    f'MAVLink connected on {port}  '
                    f'(sysid={self.master.target_system} '
                    f'comp={self.master.target_component})')

                # ── Set MANUAL mode (required for manual_control) ──
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    19)  # 19 = MANUAL in ArduSub
                ack = self.master.recv_match(
                    type='COMMAND_ACK', blocking=True, timeout=3)
                if ack:
                    self.get_logger().info(
                        f'Set MANUAL mode ACK: result={ack.result}')
                else: 
                    self.get_logger().warn(
                        'No ACK for set_mode – trying anyway')

                _time.sleep(0.5)

                # ── Arm ──
                self._arm_vehicle()

                self.connected = True
                self._consecutive_errors = 0
                return
            except Exception as exc:
                self.get_logger().warn(f'Failed on {port}: {exc}')

        self.get_logger().error(
            'No MAVLink device found – falling back to SIMULATION mode')
        self.simulate = True

    def _arm_vehicle(self):
        """Send the arm command and log the result."""
        if self.master is None:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
            ack = self.master.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=3)
            if ack:
                self.get_logger().info(f'Arm ACK: result={ack.result}')
                if ack.result != 0:
                    self.get_logger().warn(
                        f'Arm REJECTED (result={ack.result}) – '
                        'check pre-arm checks / safety switch')
                    return False
            else:
                self.get_logger().warn('No ACK for arm command')
                return False
            self.get_logger().info('Vehicle armed')
            return True
        except Exception as exc:
            self.get_logger().error(f'Arm failed: {exc}')
            return False

    def _disarm_vehicle(self):
        """Send the disarm command (best-effort)."""
        if self.master is None:
            return
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info('Disarm command sent')
        except Exception:
            pass

    def _reconnect_mavlink(self):
        """Close existing link and attempt a fresh connection + arm."""
        if self._reconnecting:
            return
        self._reconnect_attempts += 1
        if self._reconnect_attempts > self.MAX_RECONNECT_ATTEMPTS:
            self.get_logger().error(
                f'Exceeded {self.MAX_RECONNECT_ATTEMPTS} reconnect attempts '
                f'— falling back to simulation mode')
            self.simulate = True
            self.connected = False
            return
        self._reconnecting = True
        self.get_logger().warn(
            f'Attempting MAVLink reconnect ({self._reconnect_attempts}/'
            f'{self.MAX_RECONNECT_ATTEMPTS}) …')
        try:
            if self.master is not None:
                try:
                    self.master.close()
                except Exception:
                    pass
                self.master = None
            self.connected = False
            self._connect_mavlink()
            if self.connected:
                self._reconnect_attempts = 0
        finally:
            self._reconnecting = False

    # ─── Heartbeat (keeps ArduSub GCS-failsafe happy) ──────────────────

    def _heartbeat_loop(self):
        """Send a GCS heartbeat so ArduSub doesn't trigger GCS failsafe."""
        if self.simulate or not self.connected or self.master is None:
            return
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)
        except Exception as exc:
            self.get_logger().warn(f'Heartbeat send failed: {exc}')

    # ─── Armed-status monitor ──────────────────────────────────────────

    def _check_armed_status(self):
        """Periodically verify the vehicle is still in MANUAL mode & armed."""
        if self.simulate or not self.connected or self.master is None:
            return
        try:
            hb = None
            for _ in range(100):
                msg = self.master.recv_match(
                    type='HEARTBEAT', blocking=False)
                if msg is None:
                    break
                hb = msg

            if hb is None:
                return  # no fresh heartbeat — check next cycle

            armed = bool(hb.base_mode
                         & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            custom_mode = hb.custom_mode

            if not armed:
                self.get_logger().warn(
                    'Vehicle DISARMED unexpectedly – re-arming …')
                # Ensure MANUAL mode
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    19)
                _time.sleep(0.3)
                if self._arm_vehicle():
                    self.get_logger().info('Re-armed successfully')
                else:
                    self.get_logger().error(
                        'Re-arm FAILED – vehicle may not respond')

            elif custom_mode != 19:
                self.get_logger().warn(
                    f'Mode changed to {custom_mode} – switching back to '
                    f'MANUAL (19)')
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    19)

        except Exception as exc:
            self.get_logger().warn(f'Armed-status check error: {exc}')

    # ─── ROS callback ──────────────────────────────────────────────────

    def _movement_cb(self, msg: MovementCommand):
        try:
            cmd = msg.command.lower().strip()
            speed = msg.speed
            duration = msg.duration

            if not (isinstance(speed, (int, float)) and
                    isinstance(duration, (int, float))):
                self.get_logger().warn('Invalid speed/duration types — ignoring')
                return

            if not math.isfinite(speed) or not math.isfinite(duration):
                self.get_logger().warn('Non-finite speed/duration — ignoring')
                return

            speed = max(0.0, min(1.0, speed))
            duration = max(0.0, min(60.0, duration))

            # Log only when the command meaningfully changes. Closed-loop
            # callers (e.g. prequalification depth hold) stream the same
            # command at the control rate; logging every one floods the
            # terminal. Speed is quantised so small corrections don't re-log.
            log_key = (cmd, round(speed / 0.05) * 0.05, round(duration, 1))
            if log_key != self._last_log_key:
                self.get_logger().info(
                    f'Movement: {cmd}  speed={speed:.2f}  dur={duration:.1f}s')
                self._last_log_key = log_key

            dispatch = {
                'submerge':       lambda: self.submerge(speed),
                'emerge':         lambda: self.emerge(speed),
                'surge_forward':  lambda: self.surge(speed),
                'surge_backward': lambda: self.surge(-speed),
                'strafe_left':    lambda: self.strafe(-speed),
                'strafe_right':   lambda: self.strafe(speed),
                'rotate_cw':      lambda: self.rotate(speed),
                'rotate_ccw':     lambda: self.rotate(-speed),
                'stop':           self.stop,
                'depth_hold':     self.depth_hold,
            }

            handler = dispatch.get(cmd)
            if handler is None:
                self.get_logger().warn(f'Unknown command: "{cmd}" — stopping')
                self.stop()
                return
            handler()

            if duration > 0.0:
                self._stop_time = (self.get_clock().now()
                                   + Duration(seconds=duration))
            else:
                self._stop_time = None

            self._last_cmd_time = self.get_clock().now()
            self._watchdog_triggered = False
        except Exception as e:
            self.get_logger().error(f'Movement callback error: {e} — stopping')
            self.stop()

    # ─── Modular movement primitives ────────────────────────────────────

    def submerge(self, speed: float = 0.3):
        """Descend.  speed 0.0–1.0 → z decreases below 500."""
        self.current_z = round(500 - abs(speed) * 500)
        self.current_z = max(0, self.current_z)

    def emerge(self, speed: float = 0.3):
        """Ascend.  speed 0.0–1.0 → z increases above 500."""
        self.current_z = round(500 + abs(speed) * 500)
        self.current_z = min(1000, self.current_z)

    def surge(self, speed: float = 0.0):
        """Forward (positive) / backward (negative).  -1.0 … 1.0."""
        self.current_x = round(max(-1.0, min(1.0, speed)) * 1000)

    def strafe(self, speed: float = 0.0):
        """Right (positive) / left (negative).  -1.0 … 1.0."""
        self.current_y = round(max(-1.0, min(1.0, speed)) * 1000)

    def rotate(self, speed: float = 0.0):
        """CW (positive) / CCW (negative).  -1.0 … 1.0."""
        self.current_r = round(max(-1.0, min(1.0, speed)) * 1000)

    def stop(self):
        """Halt all thrusters (neutral on every axis)."""
        self.current_x = 0
        self.current_y = 0
        self.current_z = 500
        self.current_r = 0
        self._stop_time = None

    def depth_hold(self):
        """Set depth axis to neutral while preserving other axes."""
        self.current_z = 500
        self._stop_time = None

    # ─── 10 Hz control loop ────────────────────────────────────────────

    def _control_loop(self):
        now = self.get_clock().now()

        # ── Auto-stop when duration expires ──
        if self._stop_time is not None:
            if now >= self._stop_time:
                self.get_logger().info('Duration elapsed – stopping')
                self.stop()

        # ── Watchdog: auto-stop if no commands for a long time ──
        if (self._last_cmd_time is not None
                and not self._watchdog_triggered
                and self.watchdog_timeout > 0):
            elapsed = (now - self._last_cmd_time).nanoseconds / 1e9
            if elapsed > self.watchdog_timeout:
                self.get_logger().warn(
                    f'Watchdog: no command for {elapsed:.0f}s – stopping')
                self.stop()
                self._watchdog_triggered = True

        if self.simulate:
            return

        if not self.connected or self.master is None:
            return

        safe_x = max(-1000, min(1000, int(self.current_x)))
        safe_y = max(-1000, min(1000, int(self.current_y)))
        safe_z = max(0,     min(1000, int(self.current_z)))
        safe_r = max(-1000, min(1000, int(self.current_r)))

        try:
            self.master.mav.manual_control_send(
                self.master.target_system,
                x=safe_x,
                y=safe_y,
                z=safe_z,
                r=safe_r,
                buttons=0)
            # Reset error counter on success
            self._consecutive_errors = 0
            # Log values every 2 seconds (every 20th loop at 10 Hz)
            self._loop_count += 1
            if self._loop_count % 20 == 0:
                self.get_logger().info(
                    f'MAVLink TX: x={self.current_x} y={self.current_y} '
                    f'z={self.current_z} r={self.current_r}')
        except Exception as exc:
            self._consecutive_errors += 1
            self.get_logger().error(
                f'MAVLink send error ({self._consecutive_errors}/'
                f'{self.MAX_CONSECUTIVE_ERRORS}): {exc}')
            if self._consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
                self.get_logger().error(
                    'Too many consecutive errors – attempting reconnect')
                self.stop()  # safety: neutralise axes
                self._reconnect_mavlink()

    # ─── Cleanup ────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('Shutting down – sending stop + disarm …')
        if self.master and self.connected:
            try:
                self.stop()
                # Flush several neutral frames to ensure Pixhawk receives
                for _ in range(5):
                    self.master.mav.manual_control_send(
                        self.master.target_system,
                        x=0, y=0, z=500, r=0, buttons=0)
                    _time.sleep(0.05)
                self._disarm_vehicle()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
