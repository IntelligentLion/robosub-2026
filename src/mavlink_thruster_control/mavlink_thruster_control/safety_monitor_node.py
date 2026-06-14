"""Safety monitor — publishes battery % and leak status for the BT to react to.

Topics published
----------------
  /safety/battery_pct    std_msgs/Float32   0..100, NaN if unknown
  /safety/leak_detected  std_msgs/Bool      true if leak GPIO trips

Sources
-------
  * **Battery**: Pixhawk MAVLink `SYS_STATUS.battery_remaining` (0..100 or -1).
    If that returns -1 (ArduSub `BATT_CAPACITY` not set / no power-module mAh
    integration), we fall back to a voltage→% lookup on
    `SYS_STATUS.voltage_battery` calibrated for the **sub's actual battery
    pack**: 2× Blue Robotics 14.8 V / 10 Ah LiPo wired in parallel (4S, 20 Ah
    total). Curve: 16.4 V→100 %, 14.8 V→50 %, 14.0 V→20 %, 13.2 V→5 %,
    12.0 V→0 %. Below 14.0 V the default `battery_critical_pct` (15 %) in
    `bt_executor` trips `critical_failure`.
    In simulate mode (no Pixhawk plugged in, or `simulate:=true`), publishes a
    nominal 100 % so the rest of the stack ticks normally on the desk.
  * **Leak**: stub `_read_leak_gpio()` returning False. Replace with the actual
    sysfs / libgpiod read when the leak sensor is wired. (No leak hardware on
    the sub today.)

Why a separate node (not folded into thruster_node)
---------------------------------------------------
Keeps safety I/O isolated from thruster command path. On real hardware the
two nodes can either share via a MAVLink UDP forward (mavproxy/mavlink-router)
or this node can be the *single* MAVLink owner with thruster_node consuming
via UDP — both patterns work and both are common on ArduSub stacks.
"""

import time as _time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    from pymavlink import mavutil
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False


class SafetyMonitor(Node):
    PUBLISH_HZ = 2.0
    SYS_STATUS_TIMEOUT_S = 5.0  # treat battery as unknown after this gap
    MAX_DRAIN_MSGS = 200        # bound on SYS_STATUS messages drained per tick

    def __init__(self):
        super().__init__('safety_monitor')

        self.declare_parameter('simulate', True)
        self.declare_parameter('serial_port', '')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('udp_endpoint', '')  # e.g. 'udp:127.0.0.1:14551'
        self.declare_parameter('nominal_battery_pct', 100.0)
        self.declare_parameter('leak_gpio_chip', '')   # e.g. '/dev/gpiochip0'
        self.declare_parameter('leak_gpio_line', -1)

        self.simulate = self.get_parameter('simulate').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.udp_endpoint = self.get_parameter('udp_endpoint').value
        self.nominal_battery_pct = float(self.get_parameter('nominal_battery_pct').value)

        if not HAS_MAVLINK and not self.simulate:
            self.get_logger().warn('pymavlink missing — forcing simulate mode')
            self.simulate = True

        self.battery_pub = self.create_publisher(Float32, '/safety/battery_pct', 10)
        self.leak_pub = self.create_publisher(Bool, '/safety/leak_detected', 10)

        self.master = None
        self._last_sys_status_t = 0.0
        self._last_battery_pct = float('nan')

        if not self.simulate:
            self._open_mavlink()

        self.create_timer(1.0 / self.PUBLISH_HZ, self._tick)
        mode = 'SIMULATE' if self.simulate else 'PIXHAWK'
        self.get_logger().info(
            f'safety_monitor up — mode={mode}, publishing /safety/battery_pct + /safety/leak_detected')

    # ─── MAVLink connection ────────────────────────────────────────────
    def _open_mavlink(self):
        endpoint = self.udp_endpoint or self.serial_port
        if not endpoint:
            self.get_logger().warn(
                'No udp_endpoint or serial_port — falling back to simulate mode')
            self.simulate = True
            return
        try:
            if self.udp_endpoint:
                self.master = mavutil.mavlink_connection(self.udp_endpoint)
            else:
                self.master = mavutil.mavlink_connection(
                    self.serial_port, baud=self.baud_rate)
            self.get_logger().info(f'Waiting for MAVLink heartbeat on {endpoint}...')
            self.master.wait_heartbeat(timeout=5)
            self.get_logger().info('MAVLink heartbeat received')
            # Ask for SYS_STATUS at 2 Hz
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                2, 1)
        except Exception as e:
            self.get_logger().error(f'MAVLink open failed ({e}) — falling back to simulate')
            self.simulate = True
            self.master = None

    def _drain_sys_status(self):
        """Pull every queued SYS_STATUS message; cache the most recent battery %."""
        if not self.master:
            return
        try:
            # NASA rule 2: hard upper bound so a flooded link can't spin here.
            for _ in range(self.MAX_DRAIN_MSGS):
                msg = self.master.recv_match(type='SYS_STATUS', blocking=False)
                if msg is None:
                    break
                # Prefer ArduSub's calibrated remaining-% (needs BATT_CAPACITY set).
                pct = float(msg.battery_remaining)
                if pct < 0:
                    # Fallback: voltage→% lookup for the sub's 2× 14.8V/10Ah pack.
                    volts = msg.voltage_battery / 1000.0 if msg.voltage_battery > 0 else 0.0
                    if volts > 0:
                        pct = self._voltage_to_pct(volts)
                if pct >= 0:
                    self._last_battery_pct = pct
                    self._last_sys_status_t = _time.time()
        except Exception as e:
            self.get_logger().warn(f'SYS_STATUS read error: {e}')

    @staticmethod
    def _voltage_to_pct(v: float) -> float:
        """Piecewise-linear discharge curve for the sub's pack
        (2× Blue Robotics 14.8 V / 10 Ah LiPo in parallel — 4S, 20 Ah total).
        Calibration points (volts → %):
            16.4 → 100   (fully charged 4S)
            15.2 → 80
            14.8 → 50    (nominal 4S resting voltage at ~half discharge)
            14.0 → 20    (recommended stop — bt_executor trips below 15 %)
            13.2 → 5
            12.0 → 0     (absolute floor — 3.0 V/cell, do NOT discharge below)
        """
        if v <= 0:
            return -1.0  # unknown
        pts = [(16.4, 100.0), (15.2, 80.0), (14.8, 50.0),
               (14.0, 20.0),  (13.2, 5.0),  (12.0, 0.0)]
        if v >= pts[0][0]:
            return 100.0
        if v <= pts[-1][0]:
            return 0.0
        for (v_hi, p_hi), (v_lo, p_lo) in zip(pts, pts[1:]):
            if v_lo <= v <= v_hi:
                t = (v - v_lo) / (v_hi - v_lo)
                return p_lo + t * (p_hi - p_lo)
        return 0.0

    # ─── Leak sensor (stub) ────────────────────────────────────────────
    def _read_leak_gpio(self) -> bool:
        """TODO: replace with actual libgpiod / sysfs read once the leak
        sensor is wired. Returns False today (no leak hardware).
        """
        return False

    # ─── Main publish tick ─────────────────────────────────────────────
    def _tick(self):
        # Battery
        if self.simulate:
            pct = self.nominal_battery_pct
        else:
            self._drain_sys_status()
            age = _time.time() - self._last_sys_status_t
            pct = (self._last_battery_pct
                   if age <= self.SYS_STATUS_TIMEOUT_S
                   else float('nan'))

        msg = Float32()
        msg.data = float(pct)
        self.battery_pub.publish(msg)

        # Leak
        leak_msg = Bool()
        leak_msg.data = bool(self._read_leak_gpio())
        self.leak_pub.publish(leak_msg)


def main():
    rclpy.init()
    node = None
    try:
        node = SafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
