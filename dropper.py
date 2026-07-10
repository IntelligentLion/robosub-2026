#!/usr/bin/env python3
"""Dropper subsystem wrapper — marker release servo on AUX1 (SERVO9).

Mission use (share the mission's existing MAVLink connection — single
serial owner, same rule as the thruster link):

    from dropper import Dropper

    d = Dropper(master)
    d.prepare()        # once after connect: SERVO9_FUNCTION=0 + centre servo
    ...
    d.drop_right()     # over the bin: swing to 1000 — right marker away
    d.reset()          # back to centre (1500)
    d.drop_left()      # swing to 1900 — left marker away
    d.reset()

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

CHANNEL = 9            # AUX1
DROP_RIGHT_PWM = 1000  # swing right — right marker drops
DROP_LEFT_PWM = 1900   # swing left — left marker drops
REST_PWM = 1500        # centre: both markers retained (also boot-safe)
DROP_TIME = 2.0        # seconds to stay swung before it's safe to re-centre


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
        ok = self.reset()
        print(f'[dropper] ready (centred at {REST_PWM})' if ok else
              '[dropper] prepare failed')
        return ok

    def drop_right(self):
        """Swing to 1000 us — release the RIGHT marker. Servo stays swung;
        call reset() (after ~DROP_TIME) to re-centre."""
        return self._set_servo(DROP_RIGHT_PWM)

    def drop_left(self):
        """Swing to 1900 us — release the LEFT marker. Servo stays swung;
        call reset() (after ~DROP_TIME) to re-centre."""
        return self._set_servo(DROP_LEFT_PWM)

    def reset(self):
        """Centre at 1500 us — retains remaining markers, boot-safe pin state."""
        return self._set_servo(REST_PWM)

    rest = reset   # legacy alias


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

    def _pause(prompt):
        try:
            input(prompt)
        except EOFError:
            for s in range(10, 0, -1):
                print(f'  continuing in {s} …', flush=True)
                time.sleep(1)

    try:
        _pause('Markers loaded, servo centred. Press Enter to DROP RIGHT … ')
        print('drop_right()')
        d.drop_right()
        time.sleep(DROP_TIME)
        print('reset()')
        d.reset()

        _pause('Press Enter to DROP LEFT … ')
        print('drop_left()')
        d.drop_left()
        time.sleep(DROP_TIME)
        print('Done — markers away.')
    finally:
        d.reset()
