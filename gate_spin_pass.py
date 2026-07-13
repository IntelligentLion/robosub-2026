#!/usr/bin/env python3
"""Task 1 — gate pass while FLAT-SPINNING (yaw-only style, no roll/pitch).

Variant of gate_task.py. Mission (everything runs THROUGH the DepthKeeper —
Bar02 depth is held closed-loop under every phase):

    1. DIVE  : closed-loop to --depth ft (open-loop timed fallback if the
               Bar02 is missing — it drops intermittently).
    2. SEARCH: yaw-sweep until the 'gate' label is seen, then yaw-center on
               it so the latched course points through the gate. Skippable
               with --no-search (blind course = whatever we face after dive).
    3. SPIN-PASS, one of two modes (--mode, default "segmented"):

       segmented (STRAIGHT-PATH GUARANTEED): alternate
           - straight leg (--leg-time s @ --forward-speed) with the PROVEN
             DepthKeeper heading latch holding the course (no crab from
             weak motor 6), then
           - full 360° closed-loop spin IN PLACE (keeper.turn — ends back
             on course), then
           - if the gate is visible, re-aim on it (kills any residual
             lateral drift every cycle — the path is vision-corrected).
         Repeats until --forward-time of cumulative leg time is spent.
         Every spin is one direction; full 360s = 4 x 90° each, no
         reversals. No sign calibration needed — use this at competition.

       continuous: translate along the world course at --forward-speed
         while yawing nonstop at --spin-speed. The forward command is
         re-projected onto the rotating body every tick (surge/strafe from
         dh — ZED heading first, gyro integration when VSLAM drops, timed
         dead-reckoning last). Straightness aids:
           - a 2.5s calibration leg before the spin measures the true
             world course vector from ZED position, then
           - a cross-track P law (--cross-kp) steers back onto the course
             line every tick, capped at --cross-max, and AUTO-DISABLES if
             the error keeps growing (sign convention wrong — see below).
         ⚠ needs --strafe-sign / --heading-sign calibrated in the water:
         wrong sign → spiral crabs sideways. Calib: --forward-time 8 leg,
         watch the track, flip --strafe-sign -1 if it crabs.

    4. REALIGN (continuous mode): finish the rotation to the nearest full
               360° so we exit facing the course (same direction — the
               last partial increments score too). --no-realign skips it.
    5. EXIT  : keep depth-holding (or --surface), report the 90° count.

Depth hold never stops — yaw spins keep the body level, so ALT_HOLD never
fights us (no MANUAL mode needed, unlike roll/pitch style).

  python3 gate_spin_pass.py --dry-run                     # Bar02 sensor check only
  python3 gate_spin_pass.py --depth 3                    # segmented, safe
  python3 gate_spin_pass.py --mode continuous --forward-time 8   # calib leg
  python3 gate_spin_pass.py --no-search --surface

⚠ --dry-run detects + sanity-checks the Bar02 pressure sensor over a direct
Pixhawk connection, then exits. Pixhawk is never armed, thrusters never
touched, no vision spawned — safe on the bench.

⚠ ARMS the Pixhawk, drives REAL thrusters. Stop thruster_node first (single
serial owner); do not run vslam_node (detector owns the ZED). Tether, kill
switch, props clear. Ctrl+C → stop + disarm.

Requires sourced workspace:
    source /opt/ros/humble/setup.bash && source install/setup.bash
"""

import argparse
import math
import time

import depth_hold_bar02_test as dhb
import field_common as fc
from gate_task import dive, dive_open_loop, search_gate


# ─── Dry run: Bar02 pressure sensor check, no arm / no thrusters ─────────────

def dry_run(args):
    """Connect direct to the Pixhawk, detect + sanity-check the Bar02
    pressure sensor (same pipeline Bar02DepthSource.setup() uses), then
    exit. Never touches ThrusterController — nothing arms, nothing spins."""
    print(f'[dry] connecting {args.port} @ {args.baud}…')
    master = dhb.connect(args.port, args.baud)
    dhb.request_streams(master)

    print('[dry] detecting depth/pressure source…')
    ptype, first = dhb.detect_pressure_source(master)
    if ptype is None:
        print('[dry] FAIL — no SCALED_PRESSURE/2/3 arrived. Check Bar02 '
              'wiring / BARO_PROBE_EXT=768 / BARO_EXT_BUS=1 / reboot.')
        master.close()
        return 1
    print(f'[dry] source {ptype}, first reading {first.press_abs:.1f} hPa, '
          f'{first.temperature / 100.0:.1f} °C')

    print('[dry] latching surface pressure (keep sub at surface, still)…')
    surface_hpa = dhb.latch_surface(master, ptype)
    if surface_hpa is None:
        print(f'[dry] FAIL — {ptype} stopped streaming during latch.')
        master.close()
        return 1
    print(f'[dry] surface baseline = {surface_hpa:.1f} hPa')

    if not dhb.surface_sane(surface_hpa):
        ratio = surface_hpa / 1013.25
        print(f'[dry] FAIL — surface pressure {surface_hpa:.0f} hPa not '
              f'atmospheric ({ratio:.1f}x expected).')
        if 15.0 < ratio < 25.0:
            print('  ~19.6x high matches the known ArduSub 4.5.x Bar02/30BA '
                  'misdetect bug — upgrade to 4.7.0-beta7+ before diving.')
        master.close()
        return 1
    print('[dry] Bar02 OK — surface pressure plausible, scaling correct.')

    limit_m = ((dhb.BAR02_FULL_SCALE_PA - 101325.0)
               / (args.water_density * dhb.G) - dhb.BAR02_MARGIN_M)
    target_m = args.depth * fc.FEET_TO_M
    print(f'[dry] usable range ~{limit_m:.1f} m, --depth {args.depth:.1f} ft '
          f'= {target_m:.2f} m — {"OK" if target_m <= limit_m else "EXCEEDS RANGE"}')

    print('[dry] sensor detected, baseline sane. Pixhawk NOT armed, '
          'thrusters untouched. Dry run complete.')
    master.close()
    return 0


# ─── Phase 2b: center the course on the gate ─────────────────────────────────

def aim_at_gate(keeper, det, args):
    """Yaw until the gate sits at --cx-target, so the latched world course
    points through it. True if centered; False on loss/timeout (course stays
    wherever we ended up)."""
    print(f'[AIM] yaw-centering gate on cx={args.cx_target:.2f} '
          f'(tol {args.center_tol:.2f})…')
    period = 1.0 / fc.RATE_HZ
    lost = 0
    t0 = time.time()
    try:
        while time.time() - t0 < args.aim_timeout:
            d = det.best(args.label, args.conf)
            if d is None:
                lost += 1
                if lost >= args.lost_frames:
                    print('  gate lost during aim — using current heading.')
                    return False
                time.sleep(period)
                continue
            lost = 0
            ex = d.position.x - args.cx_target    # +ve → gate to the right
            if abs(ex) <= args.center_tol:
                print(f'✓ aimed (ex={ex:+.2f}).')
                return True
            yaw = fc.clamp(args.aim_gain * ex, -args.aim_max, args.aim_max)
            keeper.set_move(yaw=yaw, ramp=0.3)
            time.sleep(period)
    finally:
        keeper.clear_move(ramp=0.3)
        time.sleep(0.5)              # settle so the heading latch is clean
    print('  aim timeout — using current heading.')
    return False


# ─── Phase 3 (segmented): straight legs + in-place spins ─────────────────────

def spin_pass_segmented(keeper, det, args):
    """Alternate proven straight legs with full 360° in-place spins until
    --forward-time of cumulative leg time is spent. Straightness comes from
    the DepthKeeper heading latch on each leg (the mechanism every straight
    leg in gate_task already relies on) plus a gate re-aim between cycles
    whenever the detector still sees it. Returns total degrees spun."""
    direction = +1 if args.spin_dir == 'cw' else -1
    legs = max(1, int(round(args.forward_time / args.leg_time)))
    print(f'[SPIN-PASS/seg] {legs} x ({args.leg_time:.0f}s leg @ '
          f'{args.forward_speed:.2f} + 360° {args.spin_dir} spin).')
    total_deg = 0.0
    for i in range(legs):
        print(f'  [leg {i + 1}/{legs}] straight (heading latched)…')
        keeper.set_move(surge=args.forward_speed, ramp=0.5)
        time.sleep(args.leg_time)
        keeper.clear_move(ramp=0.3)
        time.sleep(0.4)                       # settle before the spin
        print(f'  [spin {i + 1}/{legs}] 360° {args.spin_dir} in place…')
        keeper.turn(direction, 360.0, speed=args.spin_speed)
        total_deg += 360.0
        # camera is stable again — re-aim on the gate to kill any lateral
        # drift the leg/spin picked up (skewed start, residual crab)
        if det is not None and det.seen(args.label, args.conf):
            aim_at_gate(keeper, det, args)
    return total_deg


# ─── Phase 3 (continuous): spin while advancing ──────────────────────────────

def measure_course(keeper, coord, args, secs=2.5):
    """Short straight leg (heading latched) to measure the true world course
    unit vector in the ZED ground plane (Y_UP → the plane is x/z). Returns
    (p0, u) or (None, None) if there's no usable position fix/displacement."""
    if coord is None or not coord.have_fix():
        print('  no ZED position fix — cross-track correction OFF.')
        return None, None
    x0, z0 = coord.x, coord.z
    print(f'  [CALIB] {secs:.1f}s straight leg to measure the course vector…')
    keeper.set_move(surge=max(args.forward_speed, 0.2), ramp=0.5)
    time.sleep(secs)
    keeper.clear_move(ramp=0.3)
    time.sleep(0.3)
    if not coord.have_fix():
        print('  position fix lost during calib — cross-track OFF.')
        return None, None
    dx, dz = coord.x - x0, coord.z - z0
    dist = math.hypot(dx, dz)
    if dist < 0.15:
        print(f'  calib displacement too small ({dist:.2f} m) — '
              f'cross-track OFF.')
        return None, None
    u = (dx / dist, dz / dist)
    print(f'  ✓ course vector ({u[0]:+.2f}, {u[1]:+.2f}), '
          f'{dist:.2f} m measured.')
    return (coord.x, coord.z), u


def spin_pass_continuous(keeper, coord, args):
    """Translate along the world course for --forward-time s while yawing
    continuously one direction. Returns integrated rotation in degrees.

    dh (rotation since leg start) drives the surge/strafe projection each
    tick (feedback ladder mirrors DepthKeeper.turn(): ZED heading → gyro
    yawspeed integration → timed dead-reckoning). On top of that, a
    cross-track P law on the ZED position steers back onto the course line
    measured by measure_course(); if the cross-track error keeps GROWING
    while we correct, the sign convention is wrong and the correction
    auto-disables rather than push us further off.
    """
    direction = +1 if args.spin_dir == 'cw' else -1
    f = args.forward_speed
    print(f'[SPIN-PASS/cont] {args.forward_time:.0f}s @ surge {f:.2f}, '
          f'yaw {direction * args.spin_speed:+.2f} ({args.spin_dir}).')

    # course line for cross-track correction (needs translation to measure)
    p0 = u = None
    if f > 0 and args.cross_kp > 0:
        p0, u = measure_course(keeper, coord, args)
    n = (-u[1], u[0]) if u is not None else None      # course-left normal

    mon = getattr(keeper.driver, '_heading_mon', None)

    def gyro():
        return keeper.src.attitude()[1] if keeper.src is not None else None

    h_prev = mon.heading() if mon is not None else None
    use_zed = h_prev is not None
    use_gyro = not use_zed and gyro() is not None
    if use_gyro:
        print('  no ZED heading — gyro yawspeed integration.')
    elif not (use_zed or use_gyro):
        print('  NO heading feedback — timed dead-reckoning. '
              'CALIBRATE TURN_90_SECONDS.')
    # dead-reckoned yaw rate at this spin effort (rad/s), sign = command
    dr_rate = (direction * (math.pi / 2) / fc.TURN_90_SECONDS
               * (args.spin_speed / fc.TURN_SPEED))

    dh = 0.0                    # signed rotation since leg start (rad)
    turned = 0.0                # unsigned total, for the style count
    cross_on = n is not None
    cross_prev = None           # |cross| at the last growth check
    cross_grew = 0              # consecutive checks where |cross| grew
    period = 1.0 / fc.RATE_HZ
    t_prev = time.monotonic()
    t0 = time.time()
    last_print = 0.0
    last_check = time.time()
    try:
        while time.time() - t0 < args.forward_time:
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            step = None
            if use_zed:
                h = mon.heading()
                if h is None:
                    if gyro() is not None:
                        print(f'  ZED lost at {math.degrees(turned):.0f}° — '
                              f'switching to gyro integration.')
                        use_zed, use_gyro = False, True
                    else:
                        print(f'  ZED lost at {math.degrees(turned):.0f}° — '
                              f'dead-reckoning.')
                        use_zed = False
                else:
                    step = fc.wrap(h - h_prev)
                    h_prev = h
            elif use_gyro:
                rates = gyro()
                if rates is None:
                    print(f'  ATTITUDE lost at {math.degrees(turned):.0f}° — '
                          f'dead-reckoning.')
                    use_gyro = False
                else:
                    step = rates[2] * dt
            if step is None:
                step = dr_rate * dt
            dh += step
            turned += abs(step)

            # world-frame command: course + cross-track correction
            a, b = f, 0.0            # along-course / along-n components
            cross = None
            if cross_on and coord.have_fix():
                px, pz = coord.x - p0[0], coord.z - p0[1]
                cross = px * n[0] + pz * n[1]      # signed offset (m, +left)
                b = fc.clamp(-args.cross_kp * cross,
                             -args.cross_max, args.cross_max)
                # wrong-sign guard: correcting must SHRINK |cross| over time
                if time.time() - last_check >= 2.0:
                    last_check = time.time()
                    if cross_prev is not None and abs(cross) > cross_prev + 0.05:
                        cross_grew += 1
                        if cross_grew >= 3:
                            print(f'  cross-track error GROWING '
                                  f'({abs(cross):.2f} m) — sign convention '
                                  f'wrong? correction DISABLED, flip '
                                  f'--strafe-sign / --heading-sign next run.')
                            cross_on = False
                            b = 0.0
                    else:
                        cross_grew = 0
                    cross_prev = abs(cross)

            # rotate world (a·u + b·n) into the spun body frame. σ =
            # --heading-sign maps +dh onto world rotation u→n; --strafe-sign
            # maps body-lateral onto the strafe axis (both calibrated wet).
            c, s = math.cos(dh), math.sin(dh)
            sg = args.heading_sign
            surge = a * c + b * sg * s
            strafe = args.strafe_sign * (b * c - a * sg * s)
            keeper.set_move(surge=surge, strafe=strafe,
                            yaw=direction * args.spin_speed, ramp=0.3)
            if time.time() - last_print >= 2.0:
                last_print = time.time()
                d = keeper.depth()
                print(f'  spun {math.degrees(turned):5.0f}°  '
                      f'({int(turned // (math.pi / 2))} x 90°)  '
                      + (f'depth {d:.2f} m  ' if d is not None
                         else 'depth n/a  ')
                      + (f'xtrack {cross:+.2f} m' if cross is not None
                         else 'xtrack n/a'))
            time.sleep(period)
    finally:
        keeper.clear_move(ramp=0.4)

    deg = math.degrees(turned)
    # finish to the nearest full 360° — SAME direction (a reversal would
    # un-score the last increment) — so we exit facing the course.
    if not args.no_realign:
        remainder = (360.0 - (deg % 360.0)) % 360.0
        if remainder > 5.0:
            print(f'[REALIGN] +{remainder:.0f}° {args.spin_dir} to complete '
                  f'the turn.')
            keeper.turn(direction, remainder, speed=args.spin_speed)
            deg += remainder
    return deg


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    # dry run
    ap.add_argument('--dry-run', action='store_true',
                    help='detect + sanity-check the Bar02 pressure sensor '
                         'only, then exit — no arm, no thrusters, no vision')
    ap.add_argument('--port', default=dhb.DEFAULT_PORT,
                    help='Pixhawk serial port (dry-run only)')
    ap.add_argument('--baud', type=int, default=dhb.DEFAULT_BAUD,
                    help='Pixhawk serial baud (dry-run only)')
    # dive
    ap.add_argument('--depth', type=float, default=3.0,
                    help='dive depth in FEET (default 3.0; gate is 1.5 m tall)')
    ap.add_argument('--settle-tol', type=float, default=0.12,
                    help='depth error (m) counting as "at depth"')
    ap.add_argument('--depth-hold-s', type=float, default=3.0,
                    help='seconds to hold depth before moving')
    ap.add_argument('--dive-time', type=float, default=5.0,
                    help='open-loop dive seconds if Bar02 unavailable')
    ap.add_argument('--dive-heave', type=float, default=0.4,
                    help='open-loop dive effort if Bar02 unavailable')
    # depth keeper (defaults proven in depth_field_test)
    ap.add_argument('--kp', type=float, default=2.0)
    ap.add_argument('--min-speed', type=float, default=0.15)
    ap.add_argument('--max-speed', type=float, default=0.6)
    ap.add_argument('--deadband', type=float, default=0.07)
    ap.add_argument('--max-depth', type=float, default=0.0,
                    help='abort+surface past this (m); 0 → 2x dive target')
    ap.add_argument('--water-density', type=float, default=1000.0)
    ap.add_argument('--yaw-kp', type=float, default=1.0)
    ap.add_argument('--yaw-hold-max', type=float, default=0.25)
    ap.add_argument('--yaw-hold-sign', type=float, default=1.0,
                    choices=[1.0, -1.0])
    # vision / search / aim
    ap.add_argument('--label', default='gate')
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--onnx', default=None,
                    help='absolute path to the detector model '
                         '(default: vision/ffc_rs_26.onnx)')
    ap.add_argument('--no-search', action='store_true',
                    help='skip search+aim; spin-pass along the post-dive '
                         'heading (blind)')
    ap.add_argument('--search-timeout', type=float, default=30.0)
    ap.add_argument('--search-sweep', type=float, default=2.5,
                    help='seconds per sweep direction')
    ap.add_argument('--search-speed', type=float, default=0.2)
    ap.add_argument('--cx-target', type=float, default=0.5,
                    help='image-x to aim the course at (0.5 = middle; '
                         'shift to pick a side / role imagery)')
    ap.add_argument('--center-tol', type=float, default=0.10)
    ap.add_argument('--aim-gain', type=float, default=0.6,
                    help='yaw effort per unit center-x error during aim')
    ap.add_argument('--aim-max', type=float, default=0.25)
    ap.add_argument('--aim-timeout', type=float, default=20.0)
    ap.add_argument('--lost-frames', type=int, default=8)
    # spin-pass
    ap.add_argument('--mode', choices=['segmented', 'continuous'],
                    default='segmented',
                    help='segmented: straight legs (heading latched) + 360° '
                         'spins in place, gate re-aim between cycles — '
                         'straight path guaranteed (default). continuous: '
                         'spin nonstop while translating (needs sign calib)')
    ap.add_argument('--forward-time', type=float, default=20.0,
                    help='seconds of forward travel — size so the gate is '
                         'fully cleared (no rangefinder; time it generously)')
    ap.add_argument('--forward-speed', type=float, default=0.25,
                    help='translation effort (0 = spin in place, calibration)')
    ap.add_argument('--spin-speed', type=float, default=fc.TURN_SPEED,
                    help='yaw effort during spins')
    ap.add_argument('--spin-dir', choices=['cw', 'ccw'], default='cw')
    ap.add_argument('--leg-time', type=float, default=4.0,
                    help='segmented: seconds per straight leg between spins')
    ap.add_argument('--strafe-sign', type=float, default=1.0,
                    choices=[1.0, -1.0],
                    help='continuous: flip if the spiral crabs sideways '
                         '(strafe-axis sign convention unverified)')
    ap.add_argument('--heading-sign', type=float, default=1.0,
                    choices=[1.0, -1.0],
                    help='continuous: sign mapping +dh onto world rotation '
                         'for the cross-track term (unverified)')
    ap.add_argument('--cross-kp', type=float, default=0.5,
                    help='continuous: strafe effort per metre of cross-track '
                         'error (0 = correction off)')
    ap.add_argument('--cross-max', type=float, default=0.15,
                    help='continuous: cross-track correction effort cap')
    ap.add_argument('--no-realign', action='store_true',
                    help='continuous: skip completing to a full 360° at end')
    # exit
    ap.add_argument('--clear-time', type=float, default=0.0,
                    help='extra straight forward seconds after realign')
    ap.add_argument('--surface', action='store_true',
                    help='closed-loop climb to the surface at the end')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    if args.dry_run:
        return dry_run(args)

    target_m = args.depth * fc.FEET_TO_M

    confirm = (f'GATE SPIN-PASS ({args.mode}): dive {args.depth:.1f} ft, '
               f'{"aim at the gate, " if not args.no_search else ""}then '
               f'{args.forward_time:.0f}s forward @ {args.forward_speed:.2f} '
               f'with {args.spin_dir.upper()} yaw spins. '
               f'THRUSTERS WILL SPIN.')

    factory = fc.spawn_vision_factory(
        model_onnx=args.onnx,
        monitor_extra=lambda: [fc.CoordMonitor()])
    with fc.session(factory, confirm_msg=confirm,
                    skip_confirm=args.yes) as (driver, extra):
        det = fc.find_node(extra, fc.DetectionMonitor)
        coord = fc.find_node(extra, fc.CoordMonitor)

        # Bar02 depth + ATTITUDE off the shared MAVLink link. If the Bar02
        # is missing (intermittent I2C) ATTITUDE still streams, so the gyro
        # heading fallback stays alive; depth falls back to open loop.
        src = None
        limit = None
        master = getattr(driver.thrusters, 'master', None)
        if master is not None:
            src = fc.Bar02DepthSource(master, rho=args.water_density)
            limit = src.setup()
            if limit is None:
                print('Bar02 unavailable — open-loop dive + ALT_HOLD only '
                      '(ATTITUDE feedback still on).')
            src.start()
        else:
            print('No MAVLink master — simulation? continuing without depth.')
        have_depth = limit is not None

        max_depth_m = args.max_depth if args.max_depth > 0 else 2.0 * target_m
        if limit is not None:
            max_depth_m = min(max_depth_m, limit)
            if target_m > limit:
                print(f'--depth exceeds Bar02 range ({limit:.1f} m). Abort.')
                return 1

        keeper = fc.DepthKeeper(
            driver, src, max_depth_m,
            kp=args.kp, min_speed=args.min_speed, max_speed=args.max_speed,
            deadband=args.deadband, yaw_kp=args.yaw_kp,
            yaw_hold_max=args.yaw_hold_max, yaw_hold_sign=args.yaw_hold_sign)
        keeper.start()

        spun_deg = 0.0
        try:
            # 1 — dive + hold
            if have_depth:
                dive(keeper, target_m, args.settle_tol, args.depth_hold_s)
            else:
                dive_open_loop(keeper, args.dive_heave, args.dive_time)

            # 2 — search + aim the course through the gate
            if not args.no_search:
                if search_gate(keeper, det, args):
                    aim_at_gate(keeper, det, args)
                else:
                    print('No gate found — spin-passing along current '
                          'heading (better than sitting still).')

            # 3+4 — spin while advancing (straight path), realign
            if args.mode == 'segmented':
                spun_deg = spin_pass_segmented(
                    keeper, None if args.no_search else det, args)
            else:
                spun_deg = spin_pass_continuous(keeper, coord, args)

            # 5 — exit
            if args.clear_time > 0:
                print(f'[CLEAR] forward {args.clear_time:.1f}s.')
                keeper.set_move(surge=args.forward_speed
                                if args.forward_speed > 0 else 0.3, ramp=0.5)
                time.sleep(args.clear_time)
                keeper.clear_move()
            if args.surface and have_depth:
                print('[SURFACE] closed-loop climb…')
                keeper.set_target(0.0)
                deadline = time.time() + 30.0
                while time.time() < deadline:
                    d = keeper.depth()
                    if d is not None and d <= args.deadband:
                        break
                    time.sleep(0.2)
        finally:
            keeper.stop()
            if src is not None:
                src.stop()
            inc = int(spun_deg // 90)
            print(f'\nSPIN-PASS done — {spun_deg:.0f}° continuous yaw '
                  f'= {inc} x 90° style increments (one direction, no '
                  f'reversals).')

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
