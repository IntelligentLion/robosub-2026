#!/usr/bin/env python3
"""Interactive movement tester — drive one axis at a time, ALT_HOLD holds depth.

Built on field_common: runs the production ThrusterController (arm, ALT_HOLD,
heartbeat, watchdog) and RampedDriver via ``session()``, so every move here is
driven exactly like the mission drives it. Pick moves from a menu, watch each
one in isolation, repeat.

Depth is handled by ALT_HOLD, not this script: submerge / emerge ramp the sub
down / up for a few seconds, and the moment the move stops the autopilot locks
the depth it reached (the driver streams ``depth_hold`` between moves, which is
also what keeps the failsafe fed while you read the menu). That is the "depth
hold" the submerge/emerge moves ride on — there is no separate depth loop here.

Moves (single-axis — the thruster command set has no diagonals; a diagonal
would need two axes at once, which auv_msgs/MovementCommand can't express):
    forward / backward        surge_forward / surge_backward
    strafe-left / strafe-right
    turn-left / turn-right     rotate_ccw / rotate_cw, closed-loop on ZED heading
    submerge / emerge          down / up, then ALT_HOLD holds

Menu commands:
    1..8 or a move name        run for --duration seconds (translations/verticals)
    <name> <secs>              run for a custom number of seconds
    turn-left [deg]            turn (default --turn-deg); closed-loop if ZED fix
    r                          repeat last move
    q                          surface (emerge) and quit

⚠ SAFETY (see field_common.py): this ARMS the Pixhawk and drives the REAL
thrusters. Stop thruster_node first, clear props, run on a tether, Ctrl+C →
stop + disarm. Requires the ROS 2 workspace sourced:
    source /opt/ros/humble/setup.bash && source install/setup.bash

Usage:
    python3 movement_test.py --speed 0.35 --duration 4 --turn-deg 90
"""

import argparse
import time

from field_common import (
    add_move_args, session, DepthMonitor, find_node,
)

# Menu order → the RampedDriver continuous-move method that starts it. These are
# non-blocking (streamed at 10 Hz by the driver's background thread); we bound
# them by time here and end each with stop_move() → depth_hold.
TRANSLATE = ['forward', 'backward', 'strafe-left', 'strafe-right',
             'submerge', 'emerge']
BEGIN = {
    'forward':      'move_forward',
    'backward':     'move_backward',
    'strafe-left':  'strafe_left',
    'strafe-right': 'strafe_right',
    'submerge':     'move_down',
    'emerge':       'move_up',
}
TURNS = ['turn-left', 'turn-right']
TURN_FN = {'turn-left': 'turn_left', 'turn-right': 'turn_right'}
MENU = TRANSLATE + TURNS


def run_timed(driver, name, secs, args):
    """Fire a continuous translate/vertical move for `secs`, then depth-hold."""
    getattr(driver, BEGIN[name])(args.speed, args.ramp_up)
    print(f'--- {name} @ {args.speed:.2f} for {secs:.0f}s ---')
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        time.sleep(0.05)
    driver.stop_move(args.ramp_down)
    print('--- depth-hold ---')


def run_turn(driver, name, degrees, args):
    """Closed-loop turn (timed fallback without a ZED fix); ends depth-holding."""
    print(f'--- {name} {degrees:.0f}° @ {args.speed:.2f} ---')
    ok = getattr(driver, TURN_FN[name])(degrees=degrees, speed=args.speed)
    print('✓ closed-loop verified' if ok else '⚠ timed (no ZED heading fix)')


def print_menu(args, depthmon):
    d = depthmon.depth() if depthmon else None
    dstr = f'{d:.2f} m' if d is not None else 'n/a (no ZED depth topic)'
    print(f'\n=== moves (speed={args.speed:.2f}, {args.duration:.0f}s; '
          f'depth={dstr}) ===')
    for i, name in enumerate(MENU, 1):
        extra = f'  (default {args.turn_deg:.0f}°)' if name in TURNS else ''
        print(f'  {i:2d}) {name}{extra}')
    print('  r) repeat last   q) surface & quit')
    print('  ("name secs" for custom time, e.g. "forward 6" or "turn-left 45")')


def interactive_loop(driver, args, depthmon):
    last = None
    print_menu(args, depthmon)
    while True:
        try:
            raw = input('move> ').strip().lower()
        except EOFError:
            break
        if not raw:
            continue
        if raw in ('q', 'quit', 'exit'):
            break
        if raw in ('?', 'm', 'menu', 'help'):
            print_menu(args, depthmon)
            continue

        parts = raw.split()
        key, arg = parts[0], (parts[1] if len(parts) > 1 else None)

        if key == 'r':
            if last is None:
                print('no previous move')
                continue
            name, arg = last
        elif key.isdigit():
            idx = int(key) - 1
            if not 0 <= idx < len(MENU):
                print(f'index out of range 1..{len(MENU)}')
                continue
            name = MENU[idx]
        elif key in MENU:
            name = key
        else:
            print(f'unknown: {key!r}. Type "m" for menu.')
            continue

        if name in TURNS:
            try:
                degrees = float(arg) if arg else args.turn_deg
            except ValueError:
                print(f'bad degrees: {arg!r}')
                continue
            run_turn(driver, name, degrees, args)
            last = (name, arg)
        else:
            try:
                secs = float(arg) if arg else args.duration
            except ValueError:
                print(f'bad seconds: {arg!r}')
                continue
            run_timed(driver, name, secs, args)
            last = (name, arg)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap, speed=0.35, duration=4.0)
    ap.add_argument('--turn-deg', type=float, default=90.0,
                    help='default turn angle in degrees (default 90)')
    ap.add_argument('--surface-secs', type=float, default=4.0,
                    help='emerge time on quit before disarm (default 4)')
    args = ap.parse_args()

    with session(confirm_msg='INTERACTIVE MOVEMENT TEST — will ARM and drive '
                             'each axis on demand. Thrusters WILL spin.',
                 skip_confirm=args.yes,
                 extra_factory=lambda: [DepthMonitor()]) as (driver, extra):
        depthmon = find_node(extra, DepthMonitor)
        # Hold depth from the start so the sub settles before the first move.
        driver.stop_move()
        try:
            interactive_loop(driver, args, depthmon)
        finally:
            print(f'Surfacing (emerge {args.surface_secs:.0f}s)…')
            run_timed(driver, 'emerge', args.surface_secs, args)
        # session() handles stop + disarm on exit.


if __name__ == '__main__':
    main()
