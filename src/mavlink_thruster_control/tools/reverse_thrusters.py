#!/usr/bin/env python3
"""Reverse ArduSub thruster direction via SERVOx_REVERSED params.

Two-step, verifiable workflow — no QGroundControl needed.

  STEP 1 — discover which output channel each motor is on:
      python3 reverse_thrusters.py --list

  STEP 2 — reverse the two wrong thrusters (example: channels 5 and 6):
      python3 reverse_thrusters.py --reverse 5 6

The script connects, sets SERVO<ch>_REVERSED, persists to flight-controller
EEPROM, then reads the param back to confirm the write stuck.

IMPORTANT: stop thruster_node first — the Pixhawk serial port has a single
owner. Running both clashes.

Channel = the SERVO output the thruster is wired to (1-8 for the main
outputs). --list prints, for each motor output, its channel, its
SERVOx_FUNCTION (33..40 == Motor1..Motor8) and current SERVOx_REVERSED.
"""

import argparse
import sys
import time

from pymavlink import mavutil

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200

# ArduPilot SERVOx_FUNCTION value -> human label, motor functions are 33..40.
MOTOR_FUNCTION_BASE = 33  # SERVOx_FUNCTION == 33 -> Motor 1


def connect(port, baud):
    print(f'Connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat(timeout=10)
    print(f'Heartbeat OK (sysid={master.target_system} '
          f'compid={master.target_component})')
    return master


def get_param(master, name, timeout=5.0):
    """Request one param and return its float value (or None on timeout)."""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii'), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if msg and msg.param_id.strip('\x00') == name:
            return msg.param_value
    return None


def set_param(master, name, value, timeout=5.0):
    """Set a param, then read it back to confirm. Returns the read-back value."""
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('ascii'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if msg and msg.param_id.strip('\x00') == name:
            return msg.param_value
    return None


def cmd_list(master, channels):
    print('\n ch | SERVOx_FUNCTION | SERVOx_REVERSED | note')
    print('----+-----------------+-----------------+---------------------')
    for ch in channels:
        func = get_param(master, f'SERVO{ch}_FUNCTION')
        rev = get_param(master, f'SERVO{ch}_REVERSED')
        if func is None:
            print(f' {ch:>2} | (no response)')
            continue
        func_i = int(round(func))
        note = ''
        if 33 <= func_i <= 40:
            note = f'Motor {func_i - MOTOR_FUNCTION_BASE + 1}'
        elif func_i == 0:
            note = 'disabled'
        rev_s = 'reversed' if rev and int(round(rev)) == 1 else 'normal'
        print(f' {ch:>2} | {func_i:>15} | {rev_s:>15} | {note}')
    print('\nPick the two channels whose Motor maps to your reversed '
          'thrusters, then:\n  python3 reverse_thrusters.py --reverse <chA> <chB>')


def cmd_reverse(master, channels, mode):
    """mode: 'reverse' -> set 1, 'normal' -> set 0, 'toggle' -> flip.

    'reverse'/'normal' are idempotent — running them twice leaves the same
    result, so a repeated run can never silently undo a previous one.
    """
    print(f'\n{mode} channels: {channels}')
    ok = True
    for ch in channels:
        name = f'SERVO{ch}_REVERSED'
        before = get_param(master, name)
        if before is None:
            print(f'  {name}: NO RESPONSE — channel valid? skipping')
            ok = False
            continue
        cur = int(round(before))
        if mode == 'reverse':
            target = 1
        elif mode == 'normal':
            target = 0
        else:  # toggle
            target = 0 if cur == 1 else 1
        readback = set_param(master, name, target)
        if readback is None:
            print(f'  {name}: set sent but NO read-back — UNCONFIRMED')
            ok = False
            continue
        rb = int(round(readback))
        status = 'OK' if rb == target else 'MISMATCH'
        print(f'  {name}: {cur} -> {rb}  [{status}]')
        if rb != target:
            ok = False
    if ok:
        print('\nAll writes confirmed. ArduPilot persists SERVO params to '
              'EEPROM on set — power-cycle safe.')
        print('Re-run with --list to double-check, then test heave.')
    else:
        print('\nWARNING: one or more writes unconfirmed. Re-run --list '
              'before trusting the change.')
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--list', action='store_true',
                   help='print motor->channel->reversed map')
    g.add_argument('--reverse', nargs='+', type=int, metavar='CH',
                   help='set SERVOx_REVERSED=1 for these channels (idempotent)')
    g.add_argument('--normal', nargs='+', type=int, metavar='CH',
                   help='set SERVOx_REVERSED=0 for these channels (idempotent)')
    g.add_argument('--toggle', nargs='+', type=int, metavar='CH',
                   help='flip SERVOx_REVERSED for these channels (old behaviour)')
    ap.add_argument('--channels', nargs='+', type=int,
                    default=list(range(1, 9)),
                    help='channels to scan for --list (default 1..8)')
    args = ap.parse_args()

    master = connect(args.port, args.baud)

    if args.list:
        cmd_list(master, args.channels)
        return 0
    if args.reverse is not None:
        return 0 if cmd_reverse(master, args.reverse, 'reverse') else 1
    if args.normal is not None:
        return 0 if cmd_reverse(master, args.normal, 'normal') else 1
    return 0 if cmd_reverse(master, args.toggle, 'toggle') else 1


if __name__ == '__main__':
    sys.exit(main())
