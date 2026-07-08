#!/usr/bin/env python3
"""Depth-hold test for Pixhawk 2.4.8 + Bar02 (MS5837-02BA) via ArduSub ALT_HOLD.

Replacement for depth_hold_pix_test.py, whose pressure detection never worked:
it passed a *tuple* of message types to pymavlink's recv_match(), and
recv_match wraps any non-list in [type] — so it compared get_type() against a
list containing one tuple and matched nothing, aborting with "No
SCALED_PRESSURE/2/3 of any kind" even while the sensor streamed happily
(which is why test_pixhawk.py saw SCALED_PRESSURE2 just fine).

This script:
  1. Requests all data streams (same REQUEST_DATA_STREAM call test_pixhawk.py
     uses, which is known to work on this firmware) plus best-effort
     SET_MESSAGE_INTERVAL for each SCALED_PRESSURE* id.
  2. Detects whichever SCALED_PRESSURE / SCALED_PRESSURE2 / SCALED_PRESSURE3
     is actually streaming by reading every message and matching get_type()
     itself — no recv_match type-filter pitfalls. On failure it prints the
     message types it DID see, so "sensor missing" vs "wrong message name" is
     obvious.
  3. Sanity-checks the surface reading. A Bar02 at the surface must read
     roughly atmospheric (~850-1100 hPa). ArduSub 4.5.x misdetects the Bar02
     as a 30BA and scales pressure ~19.6x, which makes ALT_HOLD depth garbage
     — if the surface value is absurd we abort with that diagnosis instead of
     diving on bad data. (Fixed in ArduSub 4.7.0-beta1+.)
  4. Latches surface pressure as depth-zero, switches to ALT_HOLD, arms, and
     runs a P controller on depth error that commands vertical climb-rate via
     MANUAL_CONTROL (z=500 is neutral; ALT_HOLD holds depth at neutral).
  5. Holds --hold-duration seconds inside tolerance, surfaces, disarms.
     Hard abort + surface if depth ever exceeds --max-depth (default 2x
     target) or the depth reading goes implausible mid-run.

Usage:
    python3 depth_hold_bar02_test.py --depth 3 --hold-duration 20
    --depth          target depth in FEET below surface baseline (default 3)
    --hold-duration  seconds to hold at target (default 20)
    --port           flight-controller serial (default /dev/ttyACM0)
    --dry-run        detect sensor + latch surface, then exit (no arm/motion)

IMPORTANT: stop thruster_node first — one owner of the Pixhawk serial port.
THRUSTERS WILL SPIN. Props clear, tether/bench, kill switch in reach.
"""

import argparse
import statistics
import sys
import time

from pymavlink import mavutil

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200
ALT_HOLD_MODE = 2              # ArduSub custom_mode for ALT_HOLD
RATE_HZ = 10                   # manual_control + heartbeat send rate
NEUTRAL_Z = 500                # centred vertical stick
G = 9.80665                    # m/s^2
FEET_TO_M = 0.3048

# NOTE: list, not tuple — pymavlink recv_match(type=...) wraps any non-list
# argument in [arg], so a tuple silently matches nothing.
PRESSURE_TYPES = ['SCALED_PRESSURE2', 'SCALED_PRESSURE', 'SCALED_PRESSURE3']
PRESSURE_MSG_IDS = {'SCALED_PRESSURE': 29,
                    'SCALED_PRESSURE2': 137,
                    'SCALED_PRESSURE3': 143}

# Plausible absolute pressure at the surface (hPa). Outside this the sensor is
# absent, broken, or scaled with the wrong-variant (30BA) math.
SURFACE_HPA_MIN = 850.0
SURFACE_HPA_MAX = 1100.0

# Bar02 is 2 bar ABSOLUTE full scale → ~10 m of fresh water before it
# saturates. Refuse targets near that.
BAR02_FULL_SCALE_PA = 200000.0
BAR02_MARGIN_M = 0.5


def connect(port, baud):
    print(f'Connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    hb = master.wait_heartbeat(timeout=10)
    if hb is None:
        print('No heartbeat in 10 s — is the Pixhawk on this port?')
        sys.exit(1)
    print(f'Heartbeat OK (sysid={master.target_system} '
          f'compid={master.target_component})')
    return master


def request_streams(master, hz=10):
    """Request telemetry the way test_pixhawk.py does (known-good on this
    firmware), then best-effort SET_MESSAGE_INTERVAL per pressure id."""
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, hz, 1)
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, hz, 1)
    interval_us = int(1e6 / hz)
    for msg_id in PRESSURE_MSG_IDS.values():
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, interval_us, 0, 0, 0, 0, 0)


def detect_pressure_source(master, timeout=8.0, settle=3.0):
    """Find which SCALED_PRESSURE* messages stream and pick the external baro.

    On ArduSub, SCALED_PRESSURE is baro instance 0 — the barometer on the
    flight-controller board, inside the hull (reads hull air pressure, not
    water). The external Bar02 shows up as instance 1+ (SCALED_PRESSURE2/3).
    So we collect for `settle` seconds after the first pressure message and
    prefer SCALED_PRESSURE2 > SCALED_PRESSURE3 > SCALED_PRESSURE, rather than
    latching whichever arrives first. Prints every source seen with its
    reading; prints all message types seen if nothing pressure-like arrives.
    """
    preference = ['SCALED_PRESSURE2', 'SCALED_PRESSURE3', 'SCALED_PRESSURE']
    seen = {}
    pressure = {}                       # type -> latest msg
    deadline = time.time() + timeout
    settle_deadline = None
    while time.time() < deadline:
        if settle_deadline is not None and time.time() >= settle_deadline:
            break
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        mtype = msg.get_type()
        seen[mtype] = seen.get(mtype, 0) + 1
        if mtype in PRESSURE_TYPES:
            pressure[mtype] = msg
            if settle_deadline is None:
                settle_deadline = time.time() + settle
    if not pressure:
        print(f'No pressure message within {timeout:.0f}s. Types seen '
              'instead:')
        for mtype in sorted(seen, key=seen.get, reverse=True):
            print(f'    {mtype:30s} x{seen[mtype]}')
        return None, None
    for mtype in preference:
        if mtype in pressure:
            m = pressure[mtype]
            note = (' <- internal FMU baro (hull air, NOT water depth)'
                    if mtype == 'SCALED_PRESSURE' else ' <- external baro')
            print(f'    {mtype:20s} {m.press_abs:8.1f} hPa '
                  f'{m.temperature / 100.0:5.1f} °C{note}')
    chosen = next(t for t in preference if t in pressure)
    if chosen == 'SCALED_PRESSURE' and len(pressure) == 1:
        print('WARNING: only the internal FMU baro is streaming — no '
              'external Bar02 message. Check BARO_PROBE_EXT=512 / '
              'BARO_EXT_BUS=1 and I2C wiring.')
    return chosen, pressure[chosen]


def read_pressure_hpa(master, ptype, timeout=1.0):
    msg = master.recv_match(type=[ptype], blocking=True, timeout=timeout)
    return None if msg is None else msg.press_abs


def latch_surface(master, ptype, samples=10):
    """Median surface pressure (hPa) as depth-zero baseline."""
    vals = []
    deadline = time.time() + 6.0
    while len(vals) < samples and time.time() < deadline:
        p = read_pressure_hpa(master, ptype)
        if p is not None:
            vals.append(p)
    return statistics.median(vals) if vals else None


def surface_sane(hpa):
    return SURFACE_HPA_MIN <= hpa <= SURFACE_HPA_MAX


def set_alt_hold(master):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        ALT_HOLD_MODE)
    ack = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3)
    print(f'ALT_HOLD ACK: result={ack.result}' if ack
          else 'No ACK for set_mode — continuing')


def arm(master, armed):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if armed else 0, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3)
    label = 'Arm' if armed else 'Disarm'
    if ack is None:
        print(f'{label}: NO ACK')
        return False
    print(f'{label} ACK: result={ack.result}'
          + ('' if ack.result == 0 else '  REJECTED — check pre-arm/safety'))
    return ack.result == 0


def send_frame(master, z):
    """One manual_control vertical frame + GCS heartbeat (failsafe guard)."""
    master.mav.manual_control_send(
        master.target_system, x=0, y=0, z=int(z), r=0, buttons=0)
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def drain_depth(master, surface_hpa, rho, ptype):
    """Pull all buffered pressure msgs, return latest depth (m) or None."""
    depth = None
    while True:
        msg = master.recv_match(type=[ptype], blocking=False)
        if msg is None:
            break
        depth = (msg.press_abs - surface_hpa) * 100.0 / (rho * G)
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
    ap.add_argument('--dry-run', action='store_true',
                    help='detect sensor + latch surface, then exit (no arm)')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    if args.depth <= 0:
        ap.error('--depth must be > 0')
    if not 0.0 < args.min_speed <= args.max_speed <= 1.0:
        ap.error('need 0 < --min-speed <= --max-speed <= 1')

    target_m = args.depth * FEET_TO_M
    max_depth_m = args.max_depth if args.max_depth > 0 else 2.0 * target_m
    rho = args.water_density

    bar02_limit_m = ((BAR02_FULL_SCALE_PA - 101325.0) / (rho * G)
                     - BAR02_MARGIN_M)
    if target_m > bar02_limit_m:
        ap.error(f'--depth {args.depth:.1f} ft = {target_m:.2f} m exceeds '
                 f'Bar02 usable range (~{bar02_limit_m:.1f} m)')
    if max_depth_m > bar02_limit_m:
        print(f'NOTE: clamping max depth {max_depth_m:.2f} m → '
              f'{bar02_limit_m:.2f} m (Bar02 saturation limit)')
        max_depth_m = bar02_limit_m

    master = connect(args.port, args.baud)
    request_streams(master)

    print('Detecting depth/pressure source…')
    ptype, first = detect_pressure_source(master)
    if ptype is None:
        print('No SCALED_PRESSURE/2/3 arrived. Since test_pixhawk.py saw '
              'SCALED_PRESSURE2, re-check wiring/params only if it now fails '
              'too: Bar02 on external I2C, BARO_PROBE_EXT=512, '
              'BARO_EXT_BUS=1, reboot after. Aborting.')
        master.close()
        return 1
    print(f'Using depth source: {ptype} '
          f'(first reading {first.press_abs:.1f} hPa, '
          f'{first.temperature / 100.0:.1f} °C)')

    print('Latching surface pressure (keep sub at surface, still)…')
    surface_hpa = latch_surface(master, ptype)
    if surface_hpa is None:
        print(f'{ptype} stopped streaming during latch — aborting.')
        master.close()
        return 1
    print(f'Surface baseline = {surface_hpa:.1f} hPa')

    if not surface_sane(surface_hpa):
        ratio = surface_hpa / 1013.25
        print(f'ABORT: surface pressure {surface_hpa:.0f} hPa is not '
              f'atmospheric ({ratio:.1f}x expected).')
        if 15.0 < ratio < 25.0:
            print('  ~19.6x high matches the known ArduSub 4.5.x bug that '
                  'reads the Bar02 with 30BA math. ALT_HOLD depth is garbage '
                  'on this firmware — upgrade to ArduSub 4.7.0-beta1+ (or '
                  'swap in a Bar30) before depth testing.')
        else:
            print('  Sensor mis-scaled or faulty — do not trust ALT_HOLD '
                  'until this reads ~1013 hPa in air at the surface.')
        master.close()
        return 1
    print('Surface pressure plausible — sensor scaling looks correct.')

    if args.dry_run:
        print('Dry run complete: sensor detected, baseline sane. '
              'Not arming.')
        master.close()
        return 0

    print(f'\nWILL SUBMERGE to {target_m:.2f} m ({args.depth:.1f} ft) via '
          f'ALT_HOLD, hold {args.hold_duration:.0f}s, then surface. '
          f'THRUSTERS WILL SPIN.')
    if not args.yes:
        if input('Props clear? type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            master.close()
            return 1

    set_alt_hold(master)
    time.sleep(0.5)
    if not arm(master, True):
        print('Arm failed — aborting, not driving.')
        master.close()
        return 1

    period = 1.0 / RATE_HZ
    reached_at = None
    aborted = False
    depth = 0.0
    loops = 0

    try:
        while True:
            loops += 1
            d = drain_depth(master, surface_hpa, rho, ptype)
            if d is not None:
                depth = d

            # Hard safety aborts: runaway descend, or reading gone implausible
            # (saturation / sensor fault mid-run).
            if not aborted and (depth > max_depth_m
                                or depth < -1.0
                                or depth > bar02_limit_m):
                aborted = True
                print(f'ABORT: depth {depth:.2f} m outside safe envelope '
                      f'(max {max_depth_m:.2f} m) — surfacing.')
            if aborted:
                if depth > args.deadband:
                    send_frame(master,
                               NEUTRAL_Z + round(args.min_speed * 500))
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
                print(f'[{state}] depth={depth:.2f} m '
                      f'target={target_m:.2f} m err={error:+.2f} m z={z}')

            if (reached_at is not None
                    and time.monotonic() - reached_at >= args.hold_duration):
                print('Hold complete — surfacing.')
                break

            time.sleep(period)

        print('Surfacing…')
        deadline = time.time() + 30.0
        while time.time() < deadline:
            d = drain_depth(master, surface_hpa, rho, ptype)
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
