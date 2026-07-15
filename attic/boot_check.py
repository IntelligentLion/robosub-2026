from pymavlink import mavutil
import time

PORT = '/dev/ttyACM0'
BAUD = 115200

print(f'Connecting {PORT} @ {BAUD} …')
master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat(timeout=10)
print('Heartbeat OK')


def recv(mtype, timeout=3):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            m = master.recv_match(type=mtype, blocking=True, timeout=1)
        except TypeError:
            continue
        if m:
            return m
    return None


for name in ('SERVO9_FUNCTION', 'SERVO9_TRIM', 'SERVO9_MIN', 'SERVO9_MAX'):
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode(), -1)
    r = recv('PARAM_VALUE')
    print(f'  {name} = {r.param_value if r else "NO REPLY"}')

print('Watching servo9 output for 10 s (no arming, no changes) …')
t0 = time.time()
while time.time() - t0 < 10:
    m = recv('SERVO_OUTPUT_RAW', timeout=2)
    if m:
        print(f'  servo9 raw = {m.servo9_raw} us')
    time.sleep(0.5)
