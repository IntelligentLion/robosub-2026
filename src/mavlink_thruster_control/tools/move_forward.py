#!/usr/bin/env python3
"""Drive the sub straight forward for a fixed time, then stop and disarm.

Standalone bench/pool tool — no ROS, no behavior tree. Mirrors the MAVLink
path used by thruster_node: ALT_HOLD mode + manual_control surge (x axis), so
the autopilot holds depth (needs a depth sensor + water) while we drive forward.

  python3 move_forward.py --speed 0.4 --duration 3

  --speed     forward throttle 0..1   (x = speed*1000, positive = forward)
  --duration  seconds to hold forward
  --port      flight-controller serial (default /dev/ttyACM0)

Sequence: connect → ALT_HOLD mode → arm → hold forward at 10 Hz (with GCS
heartbeat so ArduSub doesn't trip the GCS failsafe) → ramp to neutral →
disarm. Ctrl-C at any time stops + disarms.

IMPORTANT: stop thruster_node first — single owner of the Pixhawk serial
port. THRUSTERS WILL SPIN. Clear the props, run on a tether/bench, keep the
kill switch reachable.
"""

import argparse
import sys
import time

from pymavlink import mavutil

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200
ALT_HOLD_MODE = 2         # ArduSub custom_mode for ALT_HOLD
RATE_HZ = 10              # manual_control + heartbeat send rate
NEUTRAL_Z = 500           # centred vertical stick → ALT_HOLD holds depth


def connect(port, baud):
    print(f'Connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat(timeout=10)
    print(f'Heartbeat OK (sysid={master.target_system} '
          f'compid={master.target_component})')
    return master


def set_alt_hold_mode(master):
    # ALT_HOLD: autopilot holds depth (centred z=500) while we drive surge.
    # Needs a working depth sensor + water; manual_control surge passes through.
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        ALT_HOLD_MODE)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f'ALT_HOLD mode ACK: result={ack.result}' if ack
          else 'No ACK for set_mode — continuing')


def arm(master, armed):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if armed else 0, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    label = 'Arm' if armed else 'Disarm'
    if ack is None:
        print(f'{label}: NO ACK')
        return False
    print(f'{label} ACK: result={ack.result}'
          + ('' if ack.result == 0 else '  REJECTED — check pre-arm/safety'))
    return ack.result == 0


def send_frame(master, x):
    """One manual_control surge frame + a GCS heartbeat."""
    master.mav.manual_control_send(
        master.target_system, x=int(x), y=0, z=NEUTRAL_Z, r=0, buttons=0)
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def hold(master, x, seconds):
    """Send the given surge value at RATE_HZ for `seconds`."""
    period = 1.0 / RATE_HZ
    deadline = time.time() + seconds
    while time.time() < deadline:
        send_frame(master, x)
        time.sleep(period)


def stop(master):
    """Flush several neutral frames so the Pixhawk definitely sees zero."""
    for _ in range(5):
        send_frame(master, 0)
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--speed', type=float, default=0.3,
                    help='forward throttle 0..1 (default 0.3)')
    ap.add_argument('--duration', type=float, default=20.0,
                    help='seconds to hold forward (default 2.0)')
    ap.add_argument('--yes', action='store_true',
                    help='skip the confirm prompt')
    args = ap.parse_args()

    if not 0.0 < args.speed <= 1.0:
        ap.error('--speed must be in (0, 1]')
    if args.duration <= 0:
        ap.error('--duration must be > 0')

    x = round(args.speed * 1000)
    print(f'\nWILL DRIVE FORWARD: x={x} ({args.speed:.2f}) for '
          f'{args.duration:.1f}s. THRUSTERS WILL SPIN.')
    if not args.yes:
        if input('Props clear? type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            return 1

    master = connect(args.port, args.baud)
    set_alt_hold_mode(master)
    time.sleep(0.5)

    if not arm(master, True):
        print('Arm failed — aborting, not driving.')
        return 1

    try:
        print(f'Forward for {args.duration:.1f}s …')
        hold(master, x, args.duration)
    except KeyboardInterrupt:
        print('\nInterrupted — stopping.')
    finally:
        stop(master)
        arm(master, False)
        master.close()
    print('Done. Neutral + disarmed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
