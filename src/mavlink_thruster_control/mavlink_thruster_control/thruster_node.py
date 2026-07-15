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
  timeout (default 4 s), and disarms if the drought continues past a second,
  longer timeout (default 60 s).
* **Auto-reconnect** – re-establishes MAVLink link and re-arms on serial
  errors or unexpected disarms.
"""

import glob
import math
import threading
import time as _time
from typing import Any
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from auv_msgs.msg import MovementCommand
from auv_msgs.srv import SetFlightMode

from mavlink_thruster_control.pressure import (
    PRESSURE_TYPES, depth_from_pressure, latch_surface, pick_pressure_type,
    surface_sane)
from mavlink_thruster_control.thruster_params import (
    ALL_MOTORS, compare, expected_params)

# pymavlink is an optional dependency: without it the node runs in simulation.
# mavutil is typed Any so guarded uses aren't flagged; it is only ever
# accessed when HAS_MAVLINK is True (the simulate fallback prevents any
# hardware path from running otherwise).
mavutil: Any = None
try:
    from pymavlink import mavutil  # type: ignore
    from mavlink_thruster_control.mavlink_compat import (
        install_add_message_guard)
    # MUST run before any mavlink_connection(): without it, one MAVLink1 packet
    # poisons the message cache and the next MAVLink2 packet of that type kills
    # the entire receive path with a TypeError.
    install_add_message_guard()
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False

G = 9.80665                     # m/s^2 — RAW_IMU accel arrives in mg


def _quat_from_euler(roll, pitch, yaw):
    """(x, y, z, w) from roll/pitch/yaw radians — REP-103 XYZ, matching the
    convention imu/orientation_node already consumes from pix_imu."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class ThrusterController(Node):
    """Modular movement controller mapping MovementCommand → MAVLink axes."""

    # ── Safety constants ────────────────────────────────────────────────
    MAX_CONSECUTIVE_ERRORS = 5      # serial errors before reconnect attempt
    MAX_RECONNECT_ATTEMPTS = 5      # give up after this many consecutive reconnects
    HEARTBEAT_INTERVAL_S = 1.0      # send heartbeat every 1 s
    ARMED_CHECK_INTERVAL_S = 5.0    # verify armed status every 5 s
    DEFAULT_WATCHDOG_S = 4.0        # stop if no command received for this long
    DEFAULT_DISARM_WATCHDOG_S = 60.0  # disarm if the drought continues this long

    # ── Gateway ─────────────────────────────────────────────────────────
    TELEMETRY_RATE_HZ = 10.0        # imu/depth/mode republish rate
    SURFACE_SAMPLES = 10            # samples median-averaged into the zero ref
    HEARTBEAT_STALE_S = 3.0         # no HEARTBEAT for this long → mode unknown
    MODE_ACK_TIMEOUT_S = 3.0        # wait this long for a custom_mode readback
    PARAM_READ_TIMEOUT_S = 3.0

    # ── Dropper (marker release servo, AUX1/SERVO9) — see dropper.py for the
    # standalone driver and hardware notes. Reimplemented here (send-only +
    # passive master.messages verify) rather than importing Dropper directly:
    # Dropper._recv() calls master.recv_match() itself, which would violate
    # the single-serial-reader rule against the background reader thread. ──
    DROPPER_CHANNEL = 9
    DROPPER_DROP_RIGHT_PWM = 1000
    DROPPER_DROP_LEFT_PWM = 1900
    DROPPER_REST_PWM = 1500

    # ── ArduSub flight modes (custom_mode IDs) ──────────────────────────
    # ALT_HOLD is the default: the autopilot holds depth on the pressure
    # sensor while horizontal axes (surge/strafe/yaw) stay pilot-controlled
    # via manual_control, exactly like MANUAL. Use MANUAL only for dry-bench
    # arming (no water/depth sensor) or the ZED depth-hold test, where an
    # external controller must be the sole depth authority.
    DEFAULT_FLIGHT_MODE = 'ALT_HOLD'
    _MODE_IDS = {'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'MANUAL': 19}

    def __init__(self, flight_mode=None, simulate=None):
        super().__init__('thruster_controller')

        # ── ROS parameters ──────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('simulate', False)
        self.declare_parameter('watchdog_timeout', self.DEFAULT_WATCHDOG_S)
        self.declare_parameter('disarm_watchdog_timeout',
                                self.DEFAULT_DISARM_WATCHDOG_S)
        self.declare_parameter('flight_mode', self.DEFAULT_FLIGHT_MODE)
        self.declare_parameter('water_density', 1000.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        # Explicit kwarg overrides the ROS param — tests pass simulate=True so
        # constructing a node never reaches for a real serial port.
        self.simulate = (bool(simulate) if simulate is not None
                         else self.get_parameter('simulate').value)
        self._rho = float(self.get_parameter('water_density').value)

        # ── Gateway state ───────────────────────────────────────────────
        # MUST be initialised BEFORE _connect_mavlink(): it calls
        # _send_set_mode, which takes _tx_lock. (Learned the hard way — a
        # missing _tx_lock here raised AttributeError inside the connect
        # loop, which the broad `except` swallowed as "Failed on /dev/ttyACM0"
        # and silently downgraded a healthy vehicle to SIMULATION mode.)
        #
        # These are written by the reader thread and read by the timers. Every
        # value is a scalar or small immutable tuple and attribute assignment
        # is atomic under the GIL, so the read side needs no lock. Serial
        # WRITES do: the executor is multi-threaded, and two concurrent sends
        # would interleave bytes and corrupt the MAVLink frame.
        self._tx_lock = threading.Lock()
        self._reader_stop = threading.Event()
        self._reader_thread = None

        self._attitude = None           # (roll, pitch, yaw, rr, pr, yr)
        self._accel = (0.0, 0.0, 0.0)
        self._pressure_type = None      # chosen SCALED_PRESSURE* variant
        self._surface_samples = []
        self._surface_hpa = None        # zero reference, once latched
        self._depth_m = float('nan')
        self._mode_name = 'UNKNOWN'
        self._armed = False
        self._last_hb_time = 0.0
        self._last_statustext = ''

        wd = self.get_parameter('watchdog_timeout').value
        try:
            self.watchdog_timeout = (float(wd) if isinstance(wd, (int, float))
                                     else self.DEFAULT_WATCHDOG_S)
        except (TypeError, ValueError):
            self.watchdog_timeout = self.DEFAULT_WATCHDOG_S

        dwd = self.get_parameter('disarm_watchdog_timeout').value
        try:
            self.disarm_watchdog_timeout = (
                float(dwd) if isinstance(dwd, (int, float))
                else self.DEFAULT_DISARM_WATCHDOG_S)
        except (TypeError, ValueError):
            self.disarm_watchdog_timeout = self.DEFAULT_DISARM_WATCHDOG_S

        # Explicit kwarg overrides the ROS param (which defaults to ALT_HOLD).
        mode_name = (flight_mode
                     or self.get_parameter('flight_mode').value
                     or self.DEFAULT_FLIGHT_MODE).upper()
        if mode_name not in self._MODE_IDS:
            self.get_logger().warn(
                f'Unknown flight_mode "{mode_name}" — falling back to MANUAL')
            mode_name = 'MANUAL'
        self.flight_mode_name = mode_name
        self.flight_mode_id = self._MODE_IDS[mode_name]
        self.get_logger().info(f'Flight mode: {self.flight_mode_name}')

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
        # MANUAL_CONTROL MAVLink2 extension axes (Vectored-6DOF frame only;
        # ArduSub ignores them on frames without roll/pitch authority).
        self.current_s = 0      # pitch   (-1000 … 1000, +nose-up)
        self.current_t = 0      # roll    (-1000 … 1000, +right-side-down)

        # ── Duration tracking ───────────────────────────────────────────
        self._stop_time = None          # auto-stop deadline
        self._last_cmd_time = None      # watchdog: last command timestamp
        self._watchdog_triggered = False
        self._watchdog_disarmed = False
        self._loop_count = 0            # for periodic debug logging
        self._mode_revert_count = 0     # consecutive flight-mode rejections
        self._last_log_key = None       # de-dupe repeated Movement log lines

        # ── ROS subscriptions & timers ──────────────────────────────────
        self.create_subscription(
            MovementCommand, 'movement_command', self._movement_cb, 10)
        self.create_subscription(
            String, 'dropper_command', self._dropper_cb, 10)

        self._imu_pub = self.create_publisher(Imu, 'pixhawk/imu/data', 10)
        self._depth_pub = self.create_publisher(Float32, 'pixhawk/depth', 10)
        self._mode_pub = self.create_publisher(String, 'pixhawk/mode', 10)
        self._armed_pub = self.create_publisher(Bool, 'pixhawk/armed', 10)

        # ReentrantCallbackGroup + MultiThreadedExecutor: _on_set_mode blocks
        # for up to MODE_ACK_TIMEOUT_S waiting for the readback, and it must
        # not stall the 1 Hz heartbeat while it does — a stalled heartbeat is
        # what trips ArduSub's GCS failsafe and disarms us mid-dive.
        self._svc_group = ReentrantCallbackGroup()
        self.create_service(
            SetFlightMode, 'pixhawk/set_mode', self._on_set_mode,
            callback_group=self._svc_group)
        self.create_service(
            Trigger, 'pixhawk/preflight', self._on_preflight,
            callback_group=self._svc_group)
        self.create_service(
            Trigger, 'pixhawk/disarm', self._on_disarm,
            callback_group=self._svc_group)

        self.create_timer(0.1, self._control_loop)                # 10 Hz
        self.create_timer(1.0 / self.TELEMETRY_RATE_HZ,
                          self._publish_telemetry)                 # 10 Hz
        self.create_timer(self.HEARTBEAT_INTERVAL_S,
                          self._heartbeat_loop)                    #  1 Hz
        self.create_timer(self.ARMED_CHECK_INTERVAL_S,
                          self._check_armed_status)                # 0.2 Hz

        if not self.simulate and self.connected:
            self._start_reader()

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

                # ── Set flight mode (ALT_HOLD by default; MANUAL for dry-bench
                #    / ZED depth-hold). manual_control drives the horizontal
                #    axes in either mode; ALT_HOLD adds autopilot depth-hold. ──
                # Safe to recv_match here: the gateway reader thread is not
                # started until after this method returns.
                self._send_set_mode(self.flight_mode_id)
                ack = self.master.recv_match(
                    type='COMMAND_ACK', blocking=True, timeout=3)
                if ack:
                    self.get_logger().info(
                        f'Set {self.flight_mode_name} mode ACK: '
                        f'result={ack.result}')
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
            with self._tx_lock:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0)
            ack = self._wait_command_ack(timeout=3.0)
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
            with self._tx_lock:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info('Disarm command sent')
        except Exception:
            pass

    def _on_disarm(self, request, response):
        """ROS-callable disarm — the mission stack's only way to disarm
        outside of node shutdown (F11: nothing could disarm at mission end)."""
        self.stop()
        if self.simulate:
            response.success = True
            response.message = 'SIMULATION — no vehicle to disarm'
            return response
        self._disarm_vehicle()
        response.success = True
        response.message = 'Disarm command sent'
        return response

    def set_flight_mode(self, mode_name):
        """Switch ArduSub flight mode at runtime.

        Style rolls/loops need this: ALT_HOLD and STABILIZE self-level, so
        they physically fight a continuous roll/pitch — the sub stalls at a
        modest lean while the tilted depth thrusters shove it sideways.
        MANUAL (or ACRO) hands the roll/pitch axes straight through.

        Updates the node's target mode so _check_armed_status enforces the
        NEW mode instead of reverting it 5 s later. No ACK read here — an
        external streamer (Bar02DepthSource) may own the serial recv path;
        the armed-status watchdog re-sends the mode if the next heartbeat
        still shows the old one. Returns True if the request was recorded.
        """
        mode_name = mode_name.upper()
        if mode_name not in self._MODE_IDS:
            self.get_logger().warn(
                f'set_flight_mode: unknown mode "{mode_name}" — ignoring')
            return False
        self.flight_mode_name = mode_name
        self.flight_mode_id = self._MODE_IDS[mode_name]
        self._mode_revert_count = 0
        self.get_logger().info(f'Flight mode → {mode_name}')
        if self.simulate or not self.connected or self.master is None:
            return True
        try:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.flight_mode_id)
        except Exception as exc:
            self.get_logger().warn(f'set_flight_mode send failed: {exc} — '
                                   'watchdog will retry')
        return True

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
            with self._tx_lock:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0)
        except Exception as exc:
            self.get_logger().warn(f'Heartbeat send failed: {exc}')

    # ─── Armed-status monitor ──────────────────────────────────────────

    def _check_armed_status(self):
        """Periodically verify the vehicle is still in the chosen mode & armed.

        Reads the CACHED heartbeat state. The gateway reader thread owns recv
        on this port; a second reader here would race pyserial (select/read
        from two threads → "device reports readiness to read but returned no
        data") and steal its messages.
        """
        if self.simulate or not self.connected or self.master is None:
            return
        try:
            if (_time.time() - self._last_hb_time) > self.HEARTBEAT_STALE_S:
                return                    # no fresh heartbeat — check next cycle

            armed = self._armed
            custom_mode = self._MODE_IDS.get(self._mode_name, -1)

            if not armed:
                self.get_logger().warn(
                    'Vehicle DISARMED unexpectedly – re-arming …')
                # Ensure the configured flight mode before re-arming.
                self._send_set_mode(self.flight_mode_id)
                _time.sleep(0.3)
                if self._arm_vehicle():
                    self.get_logger().info('Re-armed successfully')
                else:
                    self.get_logger().error(
                        'Re-arm FAILED – vehicle may not respond')

            elif custom_mode != self.flight_mode_id:
                self._mode_revert_count += 1
                if self._mode_revert_count >= 3:
                    # Autopilot keeps refusing the mode — for ALT_HOLD this
                    # means no depth sensor (Bar02 not on I2C): NO depth hold,
                    # the sub will sink. Escalate instead of warn-spamming.
                    self.get_logger().error(
                        f'{self.flight_mode_name} REJECTED {self._mode_revert_count}x '
                        f'(vehicle stays in mode {custom_mode}). ALT_HOLD needs a '
                        'working depth sensor — check Bar02 wiring / SCALED_PRESSURE2. '
                        'Depth hold is NOT active.')
                else:
                    self.get_logger().warn(
                        f'Mode changed to {custom_mode} – switching back to '
                        f'{self.flight_mode_name} ({self.flight_mode_id})')
                self._send_set_mode(self.flight_mode_id)
            else:
                self._mode_revert_count = 0

        except Exception as exc:
            self.get_logger().warn(f'Armed-status check error: {exc}')

    # ─── Gateway: single serial reader ─────────────────────────────────

    def _reader_running(self):
        return self._reader_thread is not None and self._reader_thread.is_alive()

    def _start_reader(self):
        """One thread owns recv on this port. Two readers on one serial line
        produce the "device reports readiness to read but returned no data"
        stall — they race pyserial's select/read and steal each other's bytes.
        Everything else in this node reads from the cache this thread fills."""
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='mav-reader')
        self._reader_thread.start()
        self.get_logger().info('MAVLink gateway reader started')

    def _reader_loop(self):
        wanted = ['ATTITUDE', 'RAW_IMU', 'HEARTBEAT', 'STATUSTEXT',
                  'COMMAND_ACK', 'PARAM_VALUE']
        wanted.extend(PRESSURE_TYPES)
        while not self._reader_stop.is_set():
            if self.master is None:
                _time.sleep(0.1)
                continue
            try:
                msg = self.master.recv_match(
                    type=wanted, blocking=True, timeout=1.0)
            except Exception as exc:
                self.get_logger().warn(f'gateway recv error: {exc}')
                _time.sleep(0.1)
                continue
            if msg is None:
                continue
            try:
                self._on_mav_msg(msg)
            except Exception as exc:
                self.get_logger().warn(f'gateway decode error: {exc}')

    def _on_mav_msg(self, msg):
        """Single dispatch point for every received MAVLink message. Free of
        serial I/O so tests can drive it with fakes."""
        mtype = msg.get_type()
        if mtype == 'ATTITUDE':
            self._attitude = (msg.roll, msg.pitch, msg.yaw,
                              msg.rollspeed, msg.pitchspeed, msg.yawspeed)
        elif mtype == 'RAW_IMU':
            self._accel = (msg.xacc / 1000.0 * G,
                           msg.yacc / 1000.0 * G,
                           msg.zacc / 1000.0 * G)
        elif mtype in PRESSURE_TYPES:
            self._on_pressure(mtype, msg.press_abs)
        elif mtype == 'HEARTBEAT':
            self._last_hb_time = _time.time()
            self._armed = bool(msg.base_mode & 0x80)      # SAFETY_ARMED
            self._mode_name = self._decode_mode(msg.custom_mode)
        elif mtype == 'STATUSTEXT':
            text = (msg.text.strip() if isinstance(msg.text, str)
                    else str(msg.text))
            self._last_statustext = text
            if msg.severity <= 4:                          # WARNING or worse
                self.get_logger().error(f'ArduSub: {text}')

    def _on_pressure(self, mtype, press_abs):
        """Latch the surface reference, then convert every reading to depth."""
        if self._pressure_type is None:
            chosen = pick_pressure_type([mtype])
            if chosen is None:
                # Instance-0 FMU baro: sealed in the hull, reads cabin air.
                # Ignoring it entirely beats latching a "depth" that never
                # responds to descent — ALT_HOLD would hold against a constant
                # while the sub sinks.
                return
            self._pressure_type = chosen
            self.get_logger().info(f'Depth source: {chosen}')
        if mtype != self._pressure_type:
            return

        if self._surface_hpa is None:
            self._surface_samples.append(press_abs)
            if len(self._surface_samples) < self.SURFACE_SAMPLES:
                return
            candidate = latch_surface(self._surface_samples)
            if candidate is None or not surface_sane(candidate):
                self.get_logger().error(
                    f'Surface pressure latch {candidate} hPa is implausible — '
                    'depth stays unavailable. Is the sub at the surface?')
                self._surface_samples.clear()
                return
            self._surface_hpa = candidate
            self.get_logger().info(
                f'Surface latched at {candidate:.1f} hPa — depth live')
            return

        self._depth_m = depth_from_pressure(
            press_abs, self._surface_hpa, self._rho)

    def _decode_mode(self, custom_mode):
        for name, mid in self._MODE_IDS.items():
            if mid == custom_mode:
                return name
        return f'UNKNOWN({custom_mode})'

    def _wait_command_ack(self, timeout=3.0):
        """COMMAND_ACK, without becoming a second reader on the port.

        Once the gateway reader thread is running it owns recv, and pymavlink
        stashes the latest of every message type in master.messages regardless
        of which thread received it — so poll that instead of recv_match'ing.
        """
        if not self._reader_running():
            return self.master.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=timeout)
        self.master.messages.pop('COMMAND_ACK', None)
        deadline = _time.monotonic() + timeout
        while _time.monotonic() < deadline:
            ack = self.master.messages.get('COMMAND_ACK')
            if ack is not None:
                return ack
            _time.sleep(0.02)
        return None

    # ─── Gateway: telemetry publishers ─────────────────────────────────

    def _publish_telemetry(self):
        self._publish_imu()

        d = Float32()
        d.data = float(self._depth_m)
        self._depth_pub.publish(d)

        stale = (_time.time() - self._last_hb_time) > self.HEARTBEAT_STALE_S
        m = String()
        m.data = 'UNKNOWN' if stale else self._mode_name
        self._mode_pub.publish(m)

        a = Bool()
        a.data = bool(self._armed and not stale)
        self._armed_pub.publish(a)

    def _publish_imu(self):
        if self._attitude is None:
            return
        roll, pitch, yaw, rr, pr, yr = self._attitude
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        qx, qy, qz, qw = _quat_from_euler(roll, pitch, yaw)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.x = rr
        imu.angular_velocity.y = pr
        imu.angular_velocity.z = yr
        ax, ay, az = self._accel
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        self._imu_pub.publish(imu)

    # ─── Gateway: services ─────────────────────────────────────────────

    def _send_set_mode(self, mode_id):
        """Raw set_mode write. Split out so tests can stub the serial away."""
        with self._tx_lock:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)

    def _on_set_mode(self, request, response):
        """Set the mode, then require the HEARTBEAT to READ IT BACK.

        Sending the request is not evidence it took. ArduSub refuses ALT_HOLD
        outright when the depth sensor is missing and silently stays in its
        current mode, announcing the refusal only via STATUSTEXT. A caller that
        trusts the send would then dive with no depth hold, which is how a sub
        ends up on the bottom.
        """
        mode = (request.mode or '').upper().strip()
        if mode not in self._MODE_IDS:
            response.success = False
            response.reason = (f'unknown mode "{request.mode}" — expected one '
                               f'of {sorted(self._MODE_IDS)}')
            self.get_logger().warn(response.reason)
            return response

        # Record the target FIRST: the 0.2 Hz watchdog enforces THIS mode from
        # now on, instead of reverting us to the old one five seconds later.
        self.flight_mode_name = mode
        self.flight_mode_id = self._MODE_IDS[mode]
        self._mode_revert_count = 0

        if self.simulate or not self.connected or self.master is None:
            response.success = True
            response.reason = ''
            return response

        self._last_statustext = ''
        try:
            self._send_set_mode(self.flight_mode_id)
        except Exception as exc:
            response.success = False
            response.reason = f'set_mode send failed: {exc}'
            self.get_logger().error(response.reason)
            return response

        deadline = _time.monotonic() + self.MODE_ACK_TIMEOUT_S
        while _time.monotonic() < deadline:
            if self._mode_name == mode:
                self.get_logger().info(f'Flight mode confirmed: {mode}')
                response.success = True
                response.reason = ''
                return response
            _time.sleep(0.05)

        reason = (f'{mode} not confirmed within {self.MODE_ACK_TIMEOUT_S:.0f}s '
                  f'(vehicle still reports {self._mode_name})')
        if self._last_statustext:
            reason += f' — ArduSub: {self._last_statustext}'
        response.success = False
        response.reason = reason
        self.get_logger().error(reason)
        return response

    def _read_param(self, name, timeout=None):
        """Read one autopilot param. None on timeout.

        READ-ONLY. There is deliberately no _write_param: ad-hoc runtime param
        writes left MOT_5_DIRECTION and SERVO5/7_REVERSED drifted from the
        backup and made a vertical thruster fight the other three during a dive
        (2026-07-13). Persistent changes belong in the .param file / QGC.
        """
        timeout = self.PARAM_READ_TIMEOUT_S if timeout is None else timeout
        self.master.messages.pop('PARAM_VALUE', None)
        with self._tx_lock:
            self.master.mav.param_request_read_send(
                self.master.target_system, self.master.target_component,
                name.encode('ascii'), -1)
        deadline = _time.monotonic() + timeout
        while _time.monotonic() < deadline:
            msg = self.master.messages.get('PARAM_VALUE')
            if msg is not None and msg.param_id == name:
                return float(msg.param_value)
            _time.sleep(0.02)
        return None

    def _set_dropper_servo(self, pwm, retries=3):
        """DO_SET_SERVO on the dropper channel + passive verify via the
        reader thread's master.messages cache (F17/F14: moved off the
        recv_match-based standalone Dropper driver, into the single MAVLink
        owner). True on success."""
        if self.simulate or self.master is None:
            return True
        for _ in range(retries):
            self.master.messages.pop('SERVO_OUTPUT_RAW', None)
            with self._tx_lock:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, self.DROPPER_CHANNEL, pwm, 0, 0, 0, 0, 0)
            deadline = _time.monotonic() + 1.0
            while _time.monotonic() < deadline:
                msg = self.master.messages.get('SERVO_OUTPUT_RAW')
                if msg is not None:
                    raw = getattr(
                        msg, f'servo{self.DROPPER_CHANNEL}_raw', None)
                    if raw == pwm:
                        return True
                _time.sleep(0.02)
        self.get_logger().warn(
            f'Dropper: servo{self.DROPPER_CHANNEL} did not reach {pwm}us')
        return False

    def _dropper_prepare(self):
        """SERVO9_FUNCTION reverts to 184 (Actuator1) every FC boot; force it
        back to 0 (Disabled, RC-passthrough-free) so DO_SET_SERVO drives it.
        This is the one deliberate per-mission param write in this node —
        see dropper.py's module docstring; unlike the motor-mixer params,
        this one is meant to be rewritten every run."""
        if self.simulate or self.master is None:
            return True
        name = f'SERVO{self.DROPPER_CHANNEL}_FUNCTION'
        cur = self._read_param(name)
        if cur is not None and cur == 0:
            return self._set_dropper_servo(self.DROPPER_REST_PWM)
        with self._tx_lock:
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                name.encode('ascii'), 0,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        _time.sleep(0.5)
        chk = self._read_param(name)
        if chk != 0:
            self.get_logger().warn(f'Dropper: {name} stuck at {chk}')
            return False
        return self._set_dropper_servo(self.DROPPER_REST_PWM)

    def _dropper_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'prepare':
            self._dropper_prepare()
        elif cmd == 'drop_right':
            self._set_dropper_servo(self.DROPPER_DROP_RIGHT_PWM)
        elif cmd == 'drop_left':
            self._set_dropper_servo(self.DROPPER_DROP_LEFT_PWM)
        elif cmd == 'reset':
            self._set_dropper_servo(self.DROPPER_REST_PWM)
        else:
            self.get_logger().warn(f'Unknown dropper command: "{cmd}"')

    def _on_preflight(self, request, response):
        """Hard gate before any dive: do the live thruster params still match
        the known-good backup?

        A flipped VERTICAL makes one thruster fight the other three on heave —
        the sub rolls and will not descend. A flipped HORIZONTAL turns a pure
        forward command into a yaw torque — the sub spins instead of driving
        straight. Both have happened here. No bypass flag.
        """
        if self.simulate:
            response.success = True
            response.message = 'SIMULATION — thruster params not checked'
            return response

        expected = expected_params(ALL_MOTORS)
        live = {name: self._read_param(name) for name in expected}
        ok, problems = compare(live, expected)

        response.success = ok
        if ok:
            response.message = 'thruster config matches the known-good backup'
            self.get_logger().info(response.message)
        else:
            response.message = (
                'PREFLIGHT FAILED — live thruster config does not match '
                'pixhawk_params_4.5.7_backup_2026-07-08.param:\n  '
                + '\n  '.join(problems))
            self.get_logger().error(response.message)
        return response

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
                'pitch_up':       lambda: self.pitch(speed),
                'pitch_down':     lambda: self.pitch(-speed),
                'roll_right':     lambda: self.roll(speed),
                'roll_left':      lambda: self.roll(-speed),
                'stop':           self.stop,
                'depth_hold':     self.depth_hold,
            }

            if cmd == 'axes':
                # Closed-loop 6-axis setpoint: all axes at once. Reads the
                # signed axis fields directly (speed/duration unused).
                self.set_axes(msg.surge, msg.strafe, msg.heave, msg.yaw_rate,
                              msg.pitch_rate, msg.roll_rate)
            else:
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
            self._watchdog_disarmed = False
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

    def pitch(self, speed: float = 0.0):
        """Nose up (positive) / nose down (negative).  -1.0 … 1.0."""
        self.current_s = round(max(-1.0, min(1.0, speed)) * 1000)

    def roll(self, speed: float = 0.0):
        """Right-side down (positive) / left-side down (negative). -1.0 … 1.0."""
        self.current_t = round(max(-1.0, min(1.0, speed)) * 1000)

    def stop(self):
        """Halt all thrusters (neutral on every axis)."""
        self.current_x = 0
        self.current_y = 0
        self.current_z = 500
        self.current_r = 0
        self.current_s = 0
        self.current_t = 0
        self._stop_time = None

    def depth_hold(self):
        """Set depth axis to neutral while preserving other axes."""
        self.current_z = 500
        self._stop_time = None

    def set_axes(self, surge=0.0, strafe=0.0, heave=0.0, yaw_rate=0.0,
                 pitch_rate=0.0, roll_rate=0.0):
        """Direct 6-axis setpoint (closed-loop). All axes applied simultaneously.

        This is the native MAVLink manual_control form, used by
        autonomous_controller's track_object centering. Each field is signed
        [-1, 1] and clamped here. heave is +down / -up (0 = hold depth).
        pitch_rate/roll_rate go out on the MANUAL_CONTROL extension fields
        (Vectored-6DOF frame). A malformed command is neutralised rather than
        applying garbage thrust.
        """
        try:
            sg = max(-1.0, min(1.0, float(surge)))
            st = max(-1.0, min(1.0, float(strafe)))
            h = max(-1.0, min(1.0, float(heave)))
            r = max(-1.0, min(1.0, float(yaw_rate)))
            p = max(-1.0, min(1.0, float(pitch_rate)))
            ro = max(-1.0, min(1.0, float(roll_rate)))
        except (TypeError, ValueError):
            sg = st = h = r = p = ro = 0.0
        self.current_x = round(sg * 1000)
        self.current_y = round(st * 1000)
        # heave +down -> z decreases below 500 (matches submerge/emerge)
        self.current_z = max(0, min(1000, round(500 - h * 500)))
        self.current_r = round(r * 1000)
        self.current_s = round(p * 1000)
        self.current_t = round(ro * 1000)

    # ─── 10 Hz control loop ────────────────────────────────────────────

    def _control_loop(self):
        now = self.get_clock().now()

        # ── Auto-stop when duration expires ──
        if self._stop_time is not None:
            if now >= self._stop_time:
                self.get_logger().info('Duration elapsed – stopping')
                self.stop()

        # ── Watchdog: auto-stop if no commands for a long time, then disarm
        # if the drought continues (F22: 30s of stale commands at competition
        # speed was 10-15m of blind travel, and the old watchdog only ever
        # neutralised axes — the vehicle stayed armed holding depth forever) ──
        if self._last_cmd_time is not None:
            elapsed = (now - self._last_cmd_time).nanoseconds / 1e9
            if (not self._watchdog_triggered
                    and self.watchdog_timeout > 0
                    and elapsed > self.watchdog_timeout):
                self.get_logger().warn(
                    f'Watchdog: no command for {elapsed:.0f}s – stopping')
                self.stop()
                self._watchdog_triggered = True
            if (self._watchdog_triggered
                    and not self._watchdog_disarmed
                    and self.disarm_watchdog_timeout > 0
                    and elapsed > self.disarm_watchdog_timeout):
                self.get_logger().warn(
                    f'Watchdog: no command for {elapsed:.0f}s – disarming')
                self._disarm_vehicle()
                self._watchdog_disarmed = True

        if self.simulate:
            return

        if not self.connected or self.master is None:
            return

        try:
            safe_x = max(-1000, min(1000, int(self.current_x)))
            safe_y = max(-1000, min(1000, int(self.current_y)))
            safe_z = max(0,     min(1000, int(self.current_z)))
            safe_r = max(-1000, min(1000, int(self.current_r)))
            safe_s = max(-1000, min(1000, int(self.current_s)))
            safe_t = max(-1000, min(1000, int(self.current_t)))
        except (TypeError, ValueError):
            # Axis state somehow non-numeric — neutralise (safety).
            safe_x = safe_y = safe_r = safe_s = safe_t = 0
            safe_z = 500

        try:
            # enabled_extensions bit 0 = pitch (s), bit 1 = roll (t) — MAVLink2
            # extension fields; ArduSub maps them to the pitch/roll inputs on
            # 6DOF frames and ignores them elsewhere.
            with self._tx_lock:
                self.master.mav.manual_control_send(
                    self.master.target_system,
                    x=safe_x,
                    y=safe_y,
                    z=safe_z,
                    r=safe_r,
                    buttons=0,
                    enabled_extensions=0b11,
                    s=safe_s,
                    t=safe_t)
            # Reset error counter on success
            self._consecutive_errors = 0
            # Log values every 2 seconds (every 20th loop at 10 Hz)
            self._loop_count += 1
            if self._loop_count % 20 == 0:
                self.get_logger().info(
                    f'MAVLink TX: x={self.current_x} y={self.current_y} '
                    f'z={self.current_z} r={self.current_r} '
                    f's={self.current_s} t={self.current_t}')
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
        self._reader_stop.set()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        if self.master and self.connected:
            try:
                self.stop()
                # Flush several neutral frames to ensure Pixhawk receives
                for _ in range(5):
                    with self._tx_lock:
                        self.master.mav.manual_control_send(
                            self.master.target_system,
                            x=0, y=0, z=500, r=0, buttons=0,
                            enabled_extensions=0b11, s=0, t=0)
                    _time.sleep(0.05)
                self._disarm_vehicle()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterController()
    # MultiThreadedExecutor: _on_set_mode blocks for up to MODE_ACK_TIMEOUT_S
    # waiting for the mode readback. Under the default single-threaded executor
    # that would also stall the 1 Hz heartbeat, and a stalled heartbeat is what
    # trips ArduSub's GCS failsafe and disarms us mid-dive.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
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
