from pymavlink import mavutil
import time

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200

def connect(port, baud):
    print(f'Connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat(timeout=10)
    print(f'Heartbeat OK (sysid={master.target_system} '
          f'compid={master.target_component})')
    return master


def recv(mtype, timeout=3):
    # old pymavlink crashes in post_message() on some instanced messages
    # (TypeError: 'NoneType' ... _instances); retry until timeout instead
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            m = master.recv_match(type=mtype, blocking=True, timeout=1)
        except TypeError:
            continue
        if m:
            return m
    return None


def set_servo(channel, pwm):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,   # 9 = AUX1
        pwm,       # microseconds
        0, 0, 0, 0, 0)
    ack = recv('COMMAND_ACK', timeout=2)
    print(f'  set_servo({channel},{pwm}) -> {ack}')


NAN = float('nan')

def set_actuator1(value):
    """Drive SERVO9_FUNCTION=184 (k_actuator1). value in [-1, 1] -> MIN..MAX pwm."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        187,   # MAV_CMD_DO_SET_ACTUATOR (constant missing in old pymavlink)
        0,
        value,                        # actuator 1
        NAN, NAN, NAN, NAN, NAN,      # actuators 2-6 untouched
        0)                            # index offset
    ack = recv('COMMAND_ACK', timeout=2)
    print(f'  set_actuator1({value}) -> {ack}')


def get_param(name):
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), -1)
    msg = recv('PARAM_VALUE')
    print(f'  {name} = {msg.param_value if msg else "NO REPLY"}')
    return msg


def set_param(name, value):
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    msg = recv('PARAM_VALUE')
    print(f'  set {name} -> {msg.param_value if msg else "NO REPLY"}')
    return msg


master = connect(DEFAULT_PORT, DEFAULT_BAUD)
print("Heartbeat OK")

# ArduSub 4.5 has no MAV_CMD_DO_SET_ACTUATOR handler (result 3 UNSUPPORTED),
# so we drive the pin with plain DO_SET_SERVO, which requires
# SERVO9_FUNCTION=0 (Disabled).
#
# Note: SERVO9_FUNCTION=0 cannot be made persistent. At every boot,
# Sub::update_actuators_from_jsbuttons() sees SERVO9 disabled while joystick
# buttons are mapped to servo_1_* functions and set_and_save's it back to
# 184 (Actuator1). Function changes take effect immediately (no reboot
# needed), so we just force 0 at the start of every run.

def show_version():
    # AUTOPILOT_VERSION: which firmware are we actually talking to
    master.mav.command_long_send(
        master.target_system, master.target_component,
        512,  # MAV_CMD_REQUEST_MESSAGE
        0, 148, 0, 0, 0, 0, 0, 0)  # 148 = AUTOPILOT_VERSION
    m = recv('AUTOPILOT_VERSION')
    if m:
        v = m.flight_sw_version
        print(f'  fw version = {(v>>24)&0xFF}.{(v>>16)&0xFF}.{(v>>8)&0xFF}')
    else:
        print('  fw version = NO REPLY')


show_version()
msg = get_param('SERVO9_FUNCTION')
get_param('SERVO9_MIN')
get_param('SERVO9_MAX')

if msg and msg.param_value != 0:
    set_param('SERVO9_FUNCTION', 0)
    time.sleep(1)
    chk = get_param('SERVO9_FUNCTION')
    if chk and chk.param_value != 0:
        raise SystemExit('SERVO9_FUNCTION refused to change — something '
                         'is actively rewriting params')

master.arducopter_arm()
while True:   # motors_armed_wait(), but immune to the pymavlink TypeError
    hb = recv('HEARTBEAT', timeout=5)
    if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        break
print('Armed')


def show_out():
    # drain stale queued messages so we report the value AFTER our command
    try:
        while master.recv_match(type='SERVO_OUTPUT_RAW', blocking=False):
            pass
    except TypeError:
        pass
    m = recv('SERVO_OUTPUT_RAW', timeout=2)
    if m:
        print(f'  servo9 raw = {m.servo9_raw} us')  # what FC actually outputs

# Marker drop test.
# Adjust these to the mechanism's real positions:
HOLD_PWM = 1000      # marker retained
RELEASE_PWM = 1900   # marker drops
REST_PWM = 1500      # resting/boot position

# The FC keeps outputting the last DO_SET_SERVO value for as long as it has
# power (it stays alive on Jetson USB even when the AUV main switch is off).
# So ALWAYS leave the pin at REST_PWM on the way out, no matter how the
# script exits — otherwise the servo snaps to a stale position the next
# time servo power comes up.
try:
    set_servo(9, HOLD_PWM); time.sleep(1); show_out()
    try:
        input('Markers loaded, servo holding. Press Enter to DROP … ')
    except EOFError:
        # no interactive stdin (run via ! prefix) — countdown instead
        for s in range(10, 0, -1):
            print(f'  dropping in {s} …', flush=True)
            time.sleep(1)
    set_servo(9, RELEASE_PWM); time.sleep(2); show_out()
finally:
    set_servo(9, REST_PWM); time.sleep(1); show_out()
    print(f'Servo parked at {REST_PWM}.')
    master.arducopter_disarm()
    print('Disarmed') 