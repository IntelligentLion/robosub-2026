"""Safety monitor — publishes battery % and leak status for the BT to react to.

Topics published
----------------
  /safety/battery_pct    std_msgs/Float32   0..100, NaN if unknown
  /safety/leak_detected  std_msgs/Bool      true if leak GPIO trips

Sources
-------
  * **Battery**: Pixhawk MAVLink `SYS_STATUS.battery_remaining` (0..100 or -1).
    In simulate mode (no Pixhawk plugged in, or `simulate:=true`), publishes a
    nominal 100 % at 1 Hz so the rest of the stack ticks normally on the desk.
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

import math
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
            while True:
                msg = self.master.recv_match(type='SYS_STATUS', blocking=False)
                if msg is None:
                    break
                # battery_remaining: 0..100 or -1 if unknown
                pct = float(msg.battery_remaining)
                if pct >= 0:
                    self._last_battery_pct = pct
                    self._last_sys_status_t = _time.time()
        except Exception as e:
            self.get_logger().warn(f'SYS_STATUS read error: {e}')

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
