es
#!/usr/bin/env python3
"""Submerge to depth (closed-loop Bar02), then drive forward for a fixed time
while holding depth — a minimal dive+forward with NO IMU dependency.

This is the trimmed sibling of submerge_forward_10ft.py: same MANUAL-mode,
Bar02-only control, but forward is a plain --strength (% power) x --duration
(seconds) burst instead of the distance/speed estimate. Everything reuses the
tested helpers rather than duplicating them:

  * dh = depth_hold_bar02_test  (connect, pressure detect, surface latch, the
    non-skippable thruster param preflights, PWM watchdog, MANUAL_CONTROL
    framing, depth P-control building blocks)
  * sf = submerge_forward_10ft  (set_manual — verified mode switch — and
    run_phase, the 10 Hz loop that holds a target depth via a P law on Bar02
    while sending a forward x command)

Control model (identical to submerge_forward_10ft.py):
  * Depth is closed-loop the WHOLE run, including during forward: a P
    controller on Bar02 depth drives the MANUAL_CONTROL z channel.
  * Forward is open-loop thrust (no IMU -> no velocity/heading estimate). No
    yaw hold: if the sub veers, it veers. Trim/check the horizontals first
    (check_horizontal_direction.py).

Direction note: the Pixhawk is mounted facing backward, so the autopilot's -x
axis is the direction the sub actually travels. To move the sub physically
FORWARD this script sends NEGATIVE x (same as submerge_forward_10ft.py).

Safety (same rules as depth_hold_bar02_test.py):
  * kill switch in reach, props clear, sub in the water before you say yes
  * stop thruster_node / anything else on the Pixhawk serial port first
  * Bar02 depth going stale for >2 s aborts the run (neutral + disarm)
  * vertical AND horizontal thruster param preflight cannot be skipped

Usage:
    python3 submerge_forward.py                      # 2 ft down, 5 s forward
    python3 submerge_forward.py --depth 3 --strength 60 --duration 8
    python3 submerge_forward.py --dry-run            # preflight only
"""

import argparse
import sys
import time

import depth_hold_bar02_test as dh   # import applies the pymavlink
                                     # add_message monkeypatch too
import submerge_forward_10ft as sf   # set_manual + run_phase (depth-hold loop)

RATE_HZ = sf.RATE_HZ
FEET_TO_M = dh.FEET_TO_M


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=dh.DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=dh.DEFAULT_BAUD)
    ap.add_argument('--depth', type=float, default=2.0,
                    help='target depth, feet (default 2)')
    ap.add_argument('--strength', type=float, default=70.0,
                    help='forward thrust strength, percent of full power '
                         '0-100 (default 70; 100 = stick/PWM endpoint)')
    ap.add_argument('--duration', type=float, default=5.0,
                    help='forward thrust time in seconds (default 5)')
    ap.add_argument('--descend-timeout', type=float, default=30.0,
                    help='max seconds to reach depth before aborting')
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

    if not 0 < args.strength <= 100:
        ap.error('--strength is a percent of full power, in (0, 100]')
    if args.abort_depth is None:
        args.abort_depth = args.depth + 2.0

    log_path = dh.tee_output_to_log()
    print(f'Logging to {log_path}')
    master = dh.connect(args.port, args.baud)
    dh.request_streams(master, RATE_HZ)

    # Non-skippable: all 8 thrusters must match the known-good backup.
    # Verticals: a drifted one fights the others on submerge. Horizontals:
    # one flipped direction/function turns the forward surge into a spin.
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

    # Bar02 full-scale sanity on the target.
    max_bar02_ft = ((dh.BAR02_FULL_SCALE_PA / 100.0 - surface_hpa)
                    * 100.0 / (args.water_density * dh.G)
                    / FEET_TO_M) - dh.BAR02_MARGIN_M / FEET_TO_M
    if args.abort_depth > max_bar02_ft:
        print(f'Abort depth {args.abort_depth:.1f} ft exceeds Bar02 '
              f'range (~{max_bar02_ft:.1f} ft usable). Pick shallower.')
        return 1

    # Physical forward = autopilot -x (Pixhawk mounted reversed).
    x_cmd = -int(round(args.strength / 100.0 * 1000))

    print(f'\nPlan: MANUAL mode (no IMU) — descend to {args.depth:.1f} ft, '
          f'settle {args.settle_secs:.0f}s, then forward (-x, Pixhawk mounted '
          f'reversed) at {args.strength:.0f}% power for {args.duration:.1f}s '
          f'while holding depth, then neutral + disarm.')
    print('NO yaw hold, NO attitude stabilization: sub goes where the '
          'thrusters point it. Check horizontals first '
          '(check_horizontal_direction.py).')

    if args.dry_run:
        print('Dry run — stopping before mode/arm.')
        return 0
    if not args.yes:
        resp = input('Sub in water, props clear, kill switch in reach? '
                     '[yes/NO] ')
        if resp.strip().lower() != 'yes':
            print('Aborted.')
            return 1

    if not sf.set_manual(master):
        return 1
    if not dh.arm(master, True):
        return 1

    pwm_mon = dh.PwmMonitor()
    ok = True
    try:
        print('\n— Phase 1: descend until settled at depth —')
        ok = sf.run_phase(master, args, surface_hpa, pwm_mon, 'descend',
                          args.descend_timeout + args.settle_secs,
                          args.depth, x_cmd=0, deadband_ft=args.deadband,
                          settle_exit_secs=args.settle_secs)
        if ok:
            print('\n— Phase 2: forward (open loop) + Bar02 depth hold —')
            ok = sf.run_phase(master, args, surface_hpa, pwm_mon, 'forward',
                              args.duration, args.depth, x_cmd=x_cmd,
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
