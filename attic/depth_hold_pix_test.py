#!/usr/bin/env python3
"""Closed-loop depth test — submerge a fixed depth, then hold, using the PIXHAWK.

Counterpart to ``depth_hold_test.py``, but the depth source is the flight
controller's own pressure sensor (Bar30 via ``SCALED_PRESSURE2``) and depth is
held by ArduSub's built-in **ALT_HOLD** mode — NO ZED camera, no ROS, no
behavior tree. Standalone bench/pool tool, same MAVLink path as move_forward.py.

  python3 depth_hold_pix_test.py --depth 3 --hold-duration 20

  --depth          target depth in FEET below the surface (start) baseline
  --hold-duration  seconds to hold once the target depth is reached
  --port           flight-controller serial (default /dev/ttyACM0)

How it works
------------
1. Connect, request SCALED_PRESSURE2 fast, and latch the surface pressure as
   depth-zero (median of a few samples while still at the surface).
2. Switch to ALT_HOLD (ArduSub custom_mode 2) and arm. In ALT_HOLD the vertical
   (z) stick is a *climb-rate* command and centred z (=500) makes the autopilot
   hold the current depth on the pressure sensor.
3. Run a P controller on depth error that outputs a z climb-rate:
       depth  = (press_abs - surface_press) / (rho * g)
       error  = target_depth - depth          (+ve → go deeper)
       |error| <= deadband → z = 500 (let ALT_HOLD lock the depth)
       error > 0           → z < 500 (descend)
       error < 0           → z > 500 (ascend)
   As the error shrinks the rate naturally tapers to centre, handing the hold
   to ALT_HOLD. Sent at 10 Hz with a GCS heartbeat (GCS-failsafe guard).
4. Once within tolerance it holds for --hold-duration, then surfaces (ascend
   until back near zero) and disarms. A hard abort surfaces + stops if measured
   depth ever exceeds --max-depth (default 2x target).

IMPORTANT: stop thruster_node first — single owner of the Pixhawk serial port.
THRUSTERS WILL SPIN. Clear the props, run on a tether/bench, keep the kill
switch reachable. Needs a working depth/pressure sensor on the flight
controller — ALT_HOLD will refuse/behave badly without one.
"""

import argparse
import statistics
import sys
import time

from pymavlink import mavutil

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200
ALT_HOLD_MODE = 2          # ArduSub custom_mode for ALT_HOLD
RATE_HZ = 10               # manual_control + heartbeat send rate
NEUTRAL_Z = 500            # centred vertical stick → ALT_HOLD holds depth
PRESSURE_MSG_ID = 137      # SCALED_PRESSURE2 (Bar30 external pressure)
G = 9.80665                # m/s^2
FEET_TO_M = 0.3048


def connect(port, baud):
    print(f'Connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat(timeout=10)
    print(f'Heartbeat OK (sysid={master.target_system} '
          f'compid={master.target_component})')
    return master


def request_pressure(master, hz=20):
    """Ask the FC to stream SCALED_PRESSURE2 at ~hz."""
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, PRESSURE_MSG_ID, interval_us, 0, 0, 0, 0, 0)


def read_pressure_pa(master, timeout=1.0):
    """Block for one SCALED_PRESSURE2 and return absolute pressure in Pa."""
    msg = master.recv_match(type='SCALED_PRESSURE2', blocking=True,
                            timeout=timeout)
    if msg is None:
        return None
    return msg.press_abs * 100.0           # hPa → Pa


def latch_surface(master, samples=10):
    """Median surface pressure (Pa) as depth-zero baseline."""
    vals = []
    deadline = time.time() + 5.0
    while len(vals) < samples and time.time() < deadline:
        p = read_pressure_pa(master)
        if p is not None:
            vals.append(p)
    if not vals:
        return None
    return statistics.median(vals)


def set_alt_hold(master):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        ALT_HOLD_MODE)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f'ALT_HOLD ACK: result={ack.result}' if ack
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


def send_frame(master, z):
    """One manual_control vertical frame + a GCS heartbeat."""
    master.mav.manual_control_send(
        master.target_system, x=0, y=0, z=int(z), r=0, buttons=0)
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def drain_depth(master, surface_pa, rho):
    """Pull all buffered pressure msgs, return latest depth (m) or None."""
    depth = None
    while True:
        msg = master.recv_match(type='SCALED_PRESSURE2', blocking=False)
        if msg is None:
            break
        depth = (msg.press_abs * 100.0 - surface_pa) / (rho * G)
    return depth


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--depth', type=float, default=3.0,
                    help='target depth in FEET below surface (default 3.0)')
    ap.add_argument('--hold-duration', type=float, default=20.0,
                    help='seconds to hold once target reached (default 20)')
    ap.add_argument('--kp', type=float, default=2.0,
                    help='gain: climb-rate fraction per metre of error')
    ap.add_argument('--min-speed', type=float, default=0.15,
                    help='min vertical effort while moving (0-1)')
    ap.add_argument('--max-speed', type=float, default=0.6,
                    help='max vertical effort (0-1)')
    ap.add_argument('--deadband', type=float, default=0.07,
                    help='half-width (m) of neutral hold band')
    ap.add_argument('--settle-tol', type=float, default=0.1,
                    help='error (m) under which target counts as reached')
    ap.add_argument('--max-depth', type=float, default=0.0,
                    help='abort+surface past this depth (m). 0 → 2x target')
    ap.add_argument('--water-density', type=float, default=1000.0,
                    help='kg/m^3 (fresh ~1000, salt ~1025)')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    if args.depth <= 0:
        ap.error('--depth must be > 0')
    if not 0.0 < args.min_speed <= args.max_speed <= 1.0:
        ap.error('need 0 < --min-speed <= --max-speed <= 1')

    target_m = args.depth * FEET_TO_M
    max_depth_m = args.max_depth if args.max_depth > 0 else 2.0 * target_m
    rho = args.water_density

    print(f'\nWILL SUBMERGE to {target_m:.2f} m ({args.depth:.1f} ft) via '
          f'ALT_HOLD, hold {args.hold_duration:.0f}s, then surface. '
          f'THRUSTERS WILL SPIN.')
    if not args.yes:
        if input('Props clear? type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            return 1

    master = connect(args.port, args.baud)
    request_pressure(master)

    print('Latching surface pressure (stay at surface)…')
    surface_pa = latch_surface(master)
    if surface_pa is None:
        print('No SCALED_PRESSURE2 — is the depth sensor connected? Aborting.')
        master.close()
        return 1
    print(f'Surface baseline = {surface_pa:.0f} Pa')

    set_alt_hold(master)
    time.sleep(0.5)
    if not arm(master, True):
        print('Arm failed — aborting, not driving.')
        master.close()
        return 1

    period = 1.0 / RATE_HZ
    reached_at = None          # monotonic time we first hit tolerance
    aborted = False
    depth = 0.0
    loops = 0

    try:
        while True:
            loops += 1
            d = drain_depth(master, surface_pa, rho)
            if d is not None:
                depth = d

            # Hard safety abort — runaway descend.
            if not aborted and depth > max_depth_m:
                aborted = True
                print(f'ABORT: depth {depth:.2f} m > max {max_depth_m:.2f} m '
                      f'— surfacing.')
            if aborted:
                if depth > args.deadband:
                    send_frame(master, NEUTRAL_Z
                               + round(args.min_speed * 500))   # ascend
                    time.sleep(period)
                    continue
                break

            error = target_m - depth        # +ve → need to go deeper
            mag = abs(error)
            effort = max(args.min_speed, min(args.max_speed, args.kp * mag))

            if mag <= args.deadband:
                z = NEUTRAL_Z               # ALT_HOLD locks current depth
                if reached_at is None and mag <= args.settle_tol:
                    reached_at = time.monotonic()
                    print(f'✓ Reached {depth:.2f} m — holding '
                          f'{args.hold_duration:.0f}s via ALT_HOLD.')
            elif error > 0:
                z = NEUTRAL_Z - round(effort * 500)   # descend
            else:
                z = NEUTRAL_Z + round(effort * 500)   # ascend

            send_frame(master, z)

            if loops % RATE_HZ == 0:        # ~1 Hz telemetry
                state = ('HOLD' if mag <= args.deadband
                         else 'DESCEND' if error > 0 else 'CORRECT-UP')
                print(f'[{state}] depth={depth:.2f} m target={target_m:.2f} m '
                      f'err={error:+.2f} m z={z}')

            # Done holding → surface.
            if (reached_at is not None
                    and time.monotonic() - reached_at >= args.hold_duration):
                print('Hold complete — surfacing.')
                break

            time.sleep(period)

        # Surface phase: ascend until back near the surface.
        print('Surfacing…')
        deadline = time.time() + 30.0
        while time.time() < deadline:
            d = drain_depth(master, surface_pa, rho)
            if d is not None:
                depth = d
            if depth <= args.deadband:
                break
            send_frame(master, NEUTRAL_Z + round(args.min_speed * 500))
            time.sleep(period)
    except KeyboardInterrupt:
        print('\nInterrupted — stopping.')
    finally:
        for _ in range(5):                  # flush neutral
            send_frame(master, NEUTRAL_Z)
            time.sleep(0.05)
        arm(master, False)
        master.close()
    print('Done. Neutral + disarmed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
