#!/usr/bin/env python3
"""Dropper subsystem wrapper — marker release servo on AUX1 (SERVO9).

Mission use (share the mission's existing MAVLink connection — single
serial owner, same rule as the thruster link):

    from dropper import Dropper

    d = Dropper(master)
    d.prepare()        # once after connect: SERVO9_FUNCTION=0 + park at rest
    d.hold()           # before submerging: grip the markers
    ...
    d.drop()           # over the bin: release, wait, park back at rest

Standalone bench test (no arming needed — DO_SET_SERVO works disarmed):

    python3 dropper.py

Hardware/firmware notes (validated on the bench, ArduSub 4.5.7):
  - Every FC boot reverts SERVO9_FUNCTION to 184 (Actuator1): the boot
    migration Sub::update_actuators_from_jsbuttons() re-saves it whenever
    the servo is Disabled and joystick buttons map to servo_1_*. prepare()
    forces it back to 0 (takes effect immediately, no reboot).
  - The FC latches the last DO_SET_SERVO value for as long as it has power
    (it stays alive on Jetson USB even with the AUV main switch off), so
    every exit path must park the pin at REST_PWM.
  - The servo is digital: it holds position while powered even without PWM,
    so there is no software "limp" — unplug power to move it by hand.
"""

from pymavlink import mavutil
import time

CHANNEL = 9          # AUX1
HOLD_PWM = 1000      # markers retained
RELEASE_PWM = 1900   # markers drop
REST_PWM = 1500      # resting/boot position
DROP_TIME = 2.0      # seconds at RELEASE_PWM before parking back at rest


class Dropper:
    def __init__(self, master):
        self.master = master

    # -- internals ----------------------------------------------------------

    def _recv(self, mtype, timeout=3):
        # old pymavlink crashes in post_message() on some instanced messages
        # (TypeError: 'NoneType' ... _instances); retry until timeout instead
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                m = self.master.recv_match(type=mtype, blocking=True,
                                           timeout=1)
            except TypeError:
                continue
            if m:
                return m
        return None

    def _get_param(self, name):
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            name.encode('utf-8'), -1)
        return self._recv('PARAM_VALUE')

    def _set_param(self, name, value):
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            name.encode('utf-8'), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        return self._recv('PARAM_VALUE')

    def _servo_raw(self):
        # drain stale queued messages so we read the value AFTER our command
        try:
            while self.master.recv_match(type='SERVO_OUTPUT_RAW',
                                         blocking=False):
                pass
        except TypeError:
            pass
        m = self._recv('SERVO_OUTPUT_RAW', timeout=2)
        return getattr(m, f'servo{CHANNEL}_raw', None) if m else None

    def _set_servo(self, pwm, retries=3):
        """DO_SET_SERVO + verify on SERVO_OUTPUT_RAW. True on success."""
        for _ in range(retries):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, CHANNEL, pwm, 0, 0, 0, 0, 0)
            self._recv('COMMAND_ACK', timeout=1)   # ack is best-effort
            time.sleep(0.3)
            if self._servo_raw() == pwm:
                return True
        print(f'[dropper] WARNING: servo{CHANNEL} did not reach {pwm} us')
        return False

    # -- mission API ---------------------------------------------------------

    def prepare(self):
        """Once per FC boot: make AUX1 drivable, park at rest. True on success.

        Safe to call every mission start — no-op cost if already prepared.
        """
        msg = self._get_param(f'SERVO{CHANNEL}_FUNCTION')
        if msg is None:
            print('[dropper] WARNING: no PARAM_VALUE reply — link problem?')
            return False
        if msg.param_value != 0:
            self._set_param(f'SERVO{CHANNEL}_FUNCTION', 0)
            time.sleep(0.5)
            chk = self._get_param(f'SERVO{CHANNEL}_FUNCTION')
            if not chk or chk.param_value != 0:
                print(f'[dropper] WARNING: SERVO{CHANNEL}_FUNCTION stuck at '
                      f'{chk.param_value if chk else "?"}')
                return False
        ok = self.rest()
        print(f'[dropper] ready (parked at {REST_PWM})' if ok else
              '[dropper] prepare failed')
        return ok

    def hold(self):
        """Grip the markers. Call before submerging."""
        return self._set_servo(HOLD_PWM)

    def drop(self):
        """Release the markers, wait DROP_TIME, park back at rest."""
        ok = self._set_servo(RELEASE_PWM)
        time.sleep(DROP_TIME)
        self.rest()
        return ok

    def rest(self):
        """Park at the resting position (also the boot-safe pin state)."""
        return self._set_servo(REST_PWM)


# -- standalone bench test ----------------------------------------------------

if __name__ == '__main__':
    PORT = '/dev/ttyACM0'
    BAUD = 115200

    print(f'Connecting {PORT} @ {BAUD} …')
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat(timeout=10)
    print(f'Heartbeat OK (sysid={master.target_system})')

    d = Dropper(master)
    if not d.prepare():
        raise SystemExit('prepare() failed')

    try:
        print('hold()')
        d.hold()
        try:
            input('Markers loaded, servo holding. Press Enter to DROP … ')
        except EOFError:
            for s in range(10, 0, -1):
                print(f'  dropping in {s} …', flush=True)
                time.sleep(1)
        print('drop()')
        d.drop()
        print('Done — markers away, servo at rest.')
    finally:
        d.rest()
