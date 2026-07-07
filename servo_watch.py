from pymavlink import mavutil
import time

PORT = '/dev/ttyACM0'
BAUD = 115200

print(f'Connecting {PORT} @ {BAUD} …')
master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat(timeout=10)
print('Heartbeat OK')


def recv(mtype, timeout=3):
    # old pymavlink crashes in post_message() on some instanced messages;
    # retry until timeout instead of dying
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            m = master.recv_match(type=mtype, blocking=True, timeout=1)
        except TypeError:
            continue
        if m:
            return m
    return None

# Motor channel trims — if one is off 1500, that channel idles off-neutral
for i in range(1, 9):
    name = f'SERVO{i}_TRIM'
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), -1)
    msg = recv('PARAM_VALUE')
    print(f'  {name} = {msg.param_value if msg else "NO REPLY"}')

master.set_mode('MANUAL')
for _ in range(10):
    hb = recv('HEARTBEAT', timeout=2)
    if hb and hb.custom_mode == 19:
        print('Mode = MANUAL')
        break
else:
    raise SystemExit('Failed to switch to MANUAL — not arming')

master.arducopter_arm()
master.motors_armed_wait()
print('Armed — watching outputs for 10 s. Note which thruster spins.')

t0 = time.time()
while time.time() - t0 < 10:
    m = recv('SERVO_OUTPUT_RAW', timeout=2)
    if m:
        outs = [getattr(m, f'servo{i}_raw') for i in range(1, 9)]
        flagged = ' '.join(
            f'ch{i+1}:{v}{"*" if v != 1500 else ""}'
            for i, v in enumerate(outs))
        print(f'  {flagged}')
    time.sleep(0.5)

master.arducopter_disarm()
print('Disarmed')
