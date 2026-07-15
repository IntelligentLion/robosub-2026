#!/usr/bin/env python3
"""Submerge, drive backward 10 ft, surface — with NO IMU dependency.

NOTE: the Pixhawk is mounted facing backward, so the autopilot's -x axis is
the direction the sub actually travels. This script sends negative x. The
compass/heading is NOT used here at all (MANUAL mode, no EKF); to fix the
compass for the backward mounting, set AHRS_ORIENTATION=4 (Yaw180) once in
QGroundControl — do not patch it in scripts.

Everything here runs in MANUAL flight mode (custom_mode 19): no EKF, no
attitude estimate, no ALT_HOLD. The only feedback sensor used is the external
Bar02 pressure sensor (SCALED_PRESSURE2), which is a barometer, not an IMU.

  * Depth is closed-loop: a P controller on Bar02 depth drives the
    MANUAL_CONTROL z channel directly (MANUAL passes the stick straight to
    the mixer, so this works with the EKF/IMU completely out of the loop).
  * Forward is open-loop: without an IMU there is no velocity or heading
    estimate, so "10 feet" is thrust x time. Distance = --forward-speed
    (ft/s, a calibration guess — measure it!) x computed duration. There is
    also NO yaw hold: if the sub veers, it veers. Trim the thrusters first.

Reuses the tested helpers from depth_hold_bar02_test.py (connect, pressure
detection, surface latch, the non-skippable vertical-thruster param preflight,
PWM watchdog, MANUAL_CONTROL framing) rather than duplicating them.

Safety (same rules as depth_hold_bar02_test.py):
  * kill switch in reach, props clear, sub in the water before you say yes
  * stop thruster_node / anything else on the Pixhawk serial port first
  * Bar02 depth going stale for >2 s aborts the run (neutral + disarm) —
    the Bar02 I2C link is known to drop intermittently
  * vertical thruster param preflight cannot be skipped

Usage:
    python3 submerge_forward_10ft.py                 # 2 ft down, 10 ft fwd
    python3 submerge_forward_10ft.py --depth 3 --forward-speed 1.5
    python3 submerge_forward_10ft.py --forward-secs 8   # override the estimate
    python3 submerge_forward_10ft.py --dry-run          # preflight only
"""

import argparse
import sys
import time

from pymavlink import mavutil

import depth_hold_bar02_test as dh   # import applies the pymavlink
                                     # add_message monkeypatch too

MANUAL_MODE = 19                     # ArduSub custom_mode for MANUAL
RATE_HZ = 10
DEPTH_STALE_ABORT_S = 2.0            # Bar02 silence before we bail out
FEET_TO_M = dh.FEET_TO_M


def set_manual(master):
    """Command MANUAL and verify via heartbeat (ACK alone proves nothing)."""
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        MANUAL_MODE)
    ack = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3)
    print(f'MANUAL ACK: result={ack.result}' if ack
          else 'No ACK for set_mode — verifying via heartbeat…')
    hb = None
    deadline = time.time() + 5.0
    while time.time() < deadline:
        hb = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
        if hb is not None and hb.custom_mode == MANUAL_MODE:
            print('Mode verified: MANUAL active (no IMU/EKF in the loop).')
            return True
    print(f'MODE VERIFY FAILED: autopilot not in MANUAL (last custom_mode='
          f'{hb.custom_mode if hb else "none received"}).')
    return False


def depth_z(err_ft, kp, min_effort, max_effort):
    """P law on depth error (ft, + means need to go deeper) -> z command.
    Uses vertical_z so the command clears the stick deadzone floor."""
    effort = dh.clamp(abs(err_ft) * kp, min_effort, max_effort)
    direction = -1 if err_ft > 0 else +1     # -1 descend, +1 ascend
    return dh.vertical_z(effort, direction)


def run_phase(master, args, surface_hpa, pwm_mon, label, duration,
              target_ft, x_cmd, deadband_ft, settle_exit_secs=None):
    """One 10 Hz control phase: hold target_ft depth (P on Bar02) while
    sending x_cmd forward thrust. Returns False on abort (stale depth /
    overdepth), True when duration elapses.

    settle_exit_secs: if set, exit True as soon as depth has stayed within
    the deadband continuously for that many seconds — and exit False if
    `duration` elapses without that ever happening (reaching depth is then
    a requirement, not a timer)."""
    period = 1.0 / RATE_HZ
    last_depth_t = time.monotonic()
    depth_ft = None
    end = time.monotonic() + duration
    last_print = 0.0
    settled_since = None
    while time.monotonic() < end:
        t0 = time.monotonic()
        depth_m, _yaw, pwm = dh.drain_depth(
            master, surface_hpa, args.water_density, args.ptype)
        if depth_m is not None:
            depth_ft = depth_m / FEET_TO_M
            last_depth_t = t0
        if t0 - last_depth_t > DEPTH_STALE_ABORT_S:
            print(f'ABORT [{label}]: Bar02 depth stale '
                  f'>{DEPTH_STALE_ABORT_S:.0f}s (intermittent I2C?). '
                  'Going neutral + disarm.')
            return False
        if depth_ft is not None and depth_ft > args.abort_depth:
            print(f'ABORT [{label}]: depth {depth_ft:.2f} ft past abort '
                  f'limit {args.abort_depth:.2f} ft.')
            return False

        if depth_ft is None:
            z = dh.NEUTRAL_Z                    # no fix yet: don't push down
            settled_since = None
        else:
            err = target_ft - depth_ft
            in_band = abs(err) < deadband_ft
            z = (dh.NEUTRAL_Z if in_band
                 else depth_z(err, args.kp, args.min_effort, args.max_effort))
            if settle_exit_secs is not None:
                if not in_band:
                    settled_since = None
                elif settled_since is None:
                    settled_since = t0
                elif t0 - settled_since >= settle_exit_secs:
                    print(f'[{label}] depth {depth_ft:.2f} ft settled within '
                          f'±{deadband_ft:.2f} ft for {settle_exit_secs:.0f}s '
                          '— target reached.')
                    return True
        dh.send_frame(master, z, x=x_cmd)
        if pwm is not None:
            pwm_mon.update(pwm, z, x_cmd=x_cmd)

        if t0 - last_print >= 1.0:
            last_print = t0
            d = f'{depth_ft:.2f}' if depth_ft is not None else '?'
            print(f'[{label}] t-{end - t0:5.1f}s depth={d} ft '
                  f'target={target_ft:.2f} z={z} x={x_cmd} {pwm_mon.fmt()}')
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    if settle_exit_secs is not None:
        print(f'ABORT [{label}]: never settled at {target_ft:.2f} ft within '
              f'{duration:.0f}s — not surging blind.')
        return False
    return True


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=dh.DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=dh.DEFAULT_BAUD)
    ap.add_argument('--depth', type=float, default=2.0,
                    help='target depth, feet (default 2)')
    ap.add_argument('--distance', type=float, default=10.0,
                    help='forward distance, feet (default 10)')
    ap.add_argument('--forward-speed', type=float, default=10.0,
                    help='ASSUMED forward speed in ft/s at --forward-effort '
                         '(default 1.0 — a guess, calibrate in the water! '
                         'Speed scales with effort: recalibrate after '
                         'changing --forward-effort.)')
    ap.add_argument('--forward-secs', type=float, default=None,
                    help='override: forward thrust time in seconds '
                         '(ignores --distance/--forward-speed)')
    ap.add_argument('--forward-effort', type=float, default=0.7,
                    help='forward thrust fraction 0..1 (default 0.7)')
    ap.add_argument('--descend-timeout', type=float, default=30.0,
                    help='max seconds to reach depth before moving on')
    ap.add_argument('--settle-secs', type=float, default=3.0,
                    help='hold at depth before starting forward (default 3)')
    ap.add_argument('--kp', type=float, default=0.6,
                    help='depth P gain, effort per ft of error (default 0.6)')
    ap.add_argument('--min-effort', type=float, default=0.15)
    ap.add_argument('--max-effort', type=float, default=0.6)
    ap.add_argument('--deadband', type=float, default=0.15,
                    help='depth deadband, feet (default 0.15)')
    ap.add_argument('--abort-depth', type=float, default=None,
                    help='hard abort depth, feet (default: target + 2)')
    ap.add_argument('--water-density', type=float, default=1000.0)
    ap.add_argument('--dry-run', action='store_true',
                    help='connect + preflight only, never arm or move')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    if args.abort_depth is None:
        args.abort_depth = args.depth + 2.0
    forward_secs = (args.forward_secs if args.forward_secs is not None
                    else args.distance / args.forward_speed)

    log_path = dh.tee_output_to_log()
    print(f'Logging to {log_path}')
    master = dh.connect(args.port, args.baud)
    dh.request_streams(master, RATE_HZ)

    # Non-skippable: all 8 thrusters must match the known-good backup.
    # Verticals: a drifted one fights the others on submerge. Horizontals:
    # one flipped direction/function turns the forward surge into a spin
    # (the observed "submerges then just turns right" failure).
    if not dh.verify_vertical_thruster_directions(master):
        return 1
    if not dh.verify_horizontal_thruster_directions(master):
        return 1

    ptype, _first = dh.detect_pressure_source(master)
    if ptype is None:
        return 1
    args.ptype = ptype
    surface_hpa = dh.latch_surface(master, ptype)
    if surface_hpa is None or not dh.surface_sane(surface_hpa):
        print(f'Surface pressure {surface_hpa} hPa not plausible — '
              'refusing to compute depth from it.')
        return 1
    print(f'Surface latched: {surface_hpa:.1f} hPa ({ptype})')

    # Bar02 full-scale sanity on the target
    max_bar02_ft = ((dh.BAR02_FULL_SCALE_PA / 100.0 - surface_hpa)
                    * 100.0 / (args.water_density * dh.G)
                    / FEET_TO_M) - dh.BAR02_MARGIN_M / FEET_TO_M
    if args.abort_depth > max_bar02_ft:
        print(f'Abort depth {args.abort_depth:.1f} ft exceeds Bar02 '
              f'range (~{max_bar02_ft:.1f} ft usable). Pick shallower.')
        return 1

    print(f'\nPlan: MANUAL mode (no IMU) — descend to {args.depth:.1f} ft, '
          f'settle {args.settle_secs:.0f}s, backward (-x, Pixhawk mounted '
          f'reversed) at effort '
          f'{args.forward_effort:.2f} for {forward_secs:.1f}s '
          f'(assumed {args.forward_speed:.2f} ft/s -> '
          f'{args.distance:.0f} ft), then neutral + disarm.')
    print('NO yaw hold, NO attitude stabilization: sub goes where the '
          'thrusters point it.')

    if args.dry_run:
        print('Dry run — stopping before mode/arm.')
        return 0
    if not args.yes:
        resp = input('Sub in water, props clear, kill switch in reach? '
                     '[yes/NO] ')
        if resp.strip().lower() != 'yes':
            print('Aborted.')
            return 1

    if not set_manual(master):
        return 1
    if not dh.arm(master, True):
        return 1

    pwm_mon = dh.PwmMonitor()
    ok = True
    try:
        print('\n— Phase 1: descend until settled at depth —')
        ok = run_phase(master, args, surface_hpa, pwm_mon, 'descend',
                       args.descend_timeout + args.settle_secs,
                       args.depth, x_cmd=0, deadband_ft=args.deadband,
                       settle_exit_secs=args.settle_secs)
        if ok:
            print('\n— Phase 2: surge (open loop) + Bar02 depth hold —')
            # Pixhawk is mounted facing backward, so the vehicle's travel
            # direction is the autopilot's -x. Negative x drives the sub
            # the way we want it to go.
            ok = run_phase(master, args, surface_hpa, pwm_mon, 'backward',
                           forward_secs, args.depth,
                           x_cmd=-int(args.forward_effort * 1000),
                           deadband_ft=args.deadband)
    finally:
        # Neutral sticks, then disarm — buoyancy brings the sub up.
        print('\nNeutral + disarm…')
        for _ in range(RATE_HZ):
            dh.send_frame(master, dh.NEUTRAL_Z)
            time.sleep(1.0 / RATE_HZ)
        dh.arm(master, False)
    print('Done.' if ok else 'Run ABORTED — see messages above.')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
