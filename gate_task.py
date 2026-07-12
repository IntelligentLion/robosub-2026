#!/usr/bin/env python3
"""Task 1 — gate pass with STYLE (RoboSub 2026, section 3.2).

Mission sequence (all moves run THROUGH the DepthKeeper — Bar02 depth is
held closed-loop under every phase, heading held on straight legs):

    1. DIVE    : closed-loop to --depth ft (open-loop timed fallback if the
                 Bar02 is missing — it drops intermittently).
    2. SEARCH  : yaw-sweep left/right until the 'gate' label is seen.
    3. APPROACH: strafe-only centering on the gate (yawing at an off-center
                 gate arcs the sub in skewed — heading stays latched), surge
                 throttled by the centering error. Optional --cx-target picks
                 a side of the gate (choose your role imagery).
    4. PASS    : bbox width fraction >= --close-frac (or gate lost close-in)
                 → committed straight, timed push through at held heading.
    5. STYLE   : --style plan of continuous same-direction rotations in
                 verified 90° increments. Rules: every 90° orientation change
                 scores; returning to the previous orientation does NOT (a
                 steady spin never revisits); ROLL and PITCH are worth MORE
                 than yaw — and the Vectored-6DOF frame (FRAME_CONFIG 2) has
                 full roll/pitch authority via the MANUAL_CONTROL extension
                 axes. Default plan "roll:720,yaw:720": two full barrel
                 rolls (8 x 90° high-value, ends level) then two flat spins
                 (8 x 90°). Roll/pitch increments are closed-loop on the
                 Pixhawk ATTITUDE stream; yaw on the ZED heading.
    6. EXIT    : keep depth-holding (or --surface), report the style count.

The detector owns the ZED and publishes vision/detections + vslam/odometry,
so turns/heading-hold are closed-loop off the same camera.

  python3 gate_task.py --onnx /abs/path/ffc_rs_26.onnx --depth 3
  python3 gate_task.py --style roll:360                  # first water test
  python3 gate_task.py --style roll:720,pitch:360,yaw:720 --surface
                                                          # maximum greed

⚠ ARMS the Pixhawk, drives REAL thrusters. Stop thruster_node first (single
serial owner); do not run vslam_node (detector owns the ZED). Tether, kill
switch, props clear. Ctrl+C → stop + disarm.

Requires sourced workspace:
    source /opt/ros/humble/setup.bash && source install/setup.bash
"""

import argparse
import time

import field_common as fc


# ─── Phase 1: dive ────────────────────────────────────────────────────────────

def dive(keeper, target_m, settle_tol, hold_s, timeout=90.0):
    """Closed-loop descent to target_m; True once held inside settle_tol
    continuously for hold_s (leaving the band resets the clock)."""
    keeper.set_target(target_m)
    t0 = time.monotonic()
    reached = None
    last = 0.0
    while time.monotonic() - t0 < timeout:
        d = keeper.depth()
        err = None if d is None else target_m - d
        if err is not None and abs(err) <= settle_tol:
            if reached is None:
                reached = time.monotonic()
            if time.monotonic() - reached >= hold_s:
                print(f'✓ At depth {d:.2f} m.')
                return True
        else:
            reached = None
        if time.monotonic() - last >= 1.0:
            last = time.monotonic()
            print(f'[DIVE] tgt={target_m:.2f} m  '
                  f'depth={d:.2f} m' if d is not None else
                  f'[DIVE] tgt={target_m:.2f} m  depth=n/a')
        time.sleep(0.2)
    print('dive: timeout — proceeding at current depth.')
    return False


def dive_open_loop(keeper, heave, seconds):
    """No Bar02: timed descent, then ALT_HOLD keeps whatever depth we got."""
    print(f'[DIVE] open-loop {seconds:.1f}s @ heave {heave:.2f} (no Bar02 — '
          f'ALT_HOLD holds depth afterwards).')
    keeper.set_heave_override(heave)
    try:
        time.sleep(seconds)
    finally:
        keeper.set_heave_override(None)


# ─── Phase 2: search ──────────────────────────────────────────────────────────

def search_gate(keeper, det, args):
    """Yaw-sweep until the gate is seen. True if acquired."""
    print(f'[SEARCH] sweeping for "{args.label}" '
          f'(timeout {args.search_timeout:.0f}s)…')
    t0 = time.time()
    sweep_t0 = t0
    direction = 1
    try:
        while time.time() - t0 < args.search_timeout:
            if det.seen(args.label, args.conf):
                print('✓ Gate acquired.')
                return True
            if time.time() - sweep_t0 > args.search_sweep:
                direction *= -1
                sweep_t0 = time.time()
            keeper.set_move(yaw=direction * args.search_speed, ramp=0.5)
            time.sleep(1.0 / fc.RATE_HZ)
    finally:
        keeper.clear_move(ramp=0.3)
    print('✗ SEARCH TIMEOUT — no gate.')
    return False


# ─── Phase 3+4: approach + pass-through ──────────────────────────────────────

def approach_and_pass(keeper, det, args):
    """Strafe-center on the gate while creeping forward; commit a straight,
    timed pass once close (or once the gate is lost after acquisition)."""
    print(f'[APPROACH] center on cx={args.cx_target:.2f}, '
          f'commit at frac>={args.close_frac:.2f}.')
    period = 1.0 / fc.RATE_HZ
    lost = 0
    t0 = time.time()
    try:
        while time.time() - t0 < args.approach_timeout:
            d = det.best(args.label, args.conf)
            if d is None:
                lost += 1
                if lost >= args.lost_frames:
                    print('  gate lost after approach → committing pass.')
                    break
                # brief dropout: keep creeping straight, heading held
                keeper.set_move(surge=args.creep_speed, ramp=0.5)
                time.sleep(period)
                continue
            lost = 0
            ex = d.position.x - args.cx_target        # +ve → gate to the right
            frac = d.bbox_width                       # normalised width
            if frac >= args.close_frac and abs(ex) <= args.center_tol:
                print(f'  close+centered (frac={frac:.2f} ex={ex:+.2f}) '
                      f'→ committing pass.')
                break
            strafe = fc.clamp(args.strafe_gain * ex,
                              -args.strafe_max, args.strafe_max)
            # throttle surge down while off-center so we straighten first
            slow = fc.clamp(1.0 - abs(ex) / args.ex_slow, 0.0, 1.0)
            surge = args.creep_speed + (args.speed - args.creep_speed) * slow
            if frac >= args.close_frac:
                surge = 0.0          # at the mouth but off-center: fix it first
            keeper.set_move(surge=surge, strafe=strafe, ramp=0.5)
            time.sleep(period)
        else:
            print('  approach timeout → committing pass anyway.')
    finally:
        keeper.clear_move(ramp=0.3)

    print(f'[PASS] straight push {args.pass_time:.1f}s @ {args.pass_speed:.2f} '
          f'(depth + heading held).')
    keeper.set_move(surge=args.pass_speed, ramp=0.8)
    try:
        time.sleep(args.pass_time)
    finally:
        keeper.clear_move(ramp=0.5)


# ─── Phase 5: style ───────────────────────────────────────────────────────────

def parse_style_plan(spec):
    """'roll:720,yaw:-360' → [('roll', +1, 720.0), ('yaw', -1, 360.0)].

    Sign of the degrees picks the direction (yaw: + = CW; pitch: + = nose-up;
    roll: + = right-side down). Each entry must be a multiple of 90.
    """
    plan = []
    for item in spec.split(','):
        item = item.strip()
        if not item:
            continue
        axis, _, deg = item.partition(':')
        axis = axis.strip().lower()
        if axis not in ('roll', 'pitch', 'yaw'):
            raise ValueError(f'unknown style axis "{axis}"')
        deg = float(deg)
        if deg == 0 or abs(deg) % 90 != 0:
            raise ValueError(f'{axis}: degrees must be a nonzero multiple '
                             f'of 90 (got {deg:g})')
        plan.append((axis, +1 if deg > 0 else -1, abs(deg)))
    return plan


def style_spin(keeper, plan, args):
    """Run the style plan in verified 90° increments; returns (yaw_inc,
    rollpitch_inc) counts.

    Scoring: each 90° orientation change accumulates; going BACK to the
    previous orientation does not score — every entry spins ONE direction
    continuously and full-360° multiples end level/legal. Roll and pitch
    outscore yaw, so they run on the 6DOF extension axes with ATTITUDE
    feedback; depth-hold heave is suspended during roll/pitch (body frame
    rotated) and re-latched after each entry.
    """
    yaw_inc = rp_inc = 0
    thrusters = getattr(keeper.driver, 'thrusters', None)
    can_switch = (thrusters is not None
                  and hasattr(thrusters, 'set_flight_mode'))
    style_mode = getattr(args, 'style_mode', 'MANUAL')
    for axis, direction, degrees in plan:
        total = int(round(degrees / 90.0))
        print(f'[STYLE] {axis} {degrees:.0f}° '
              f'({"+" if direction > 0 else "-"}, {total} x 90°)…')
        # ALT_HOLD/STABILIZE self-level — they FIGHT a continuous roll/
        # pitch (2026-07-10 run: roll stalled at 31° leaned while the
        # tilted depth thrusters shoved the sub sideways). Switch to
        # --style-mode (MANUAL: raw passthrough) for the WHOLE entry:
        # restoring ALT_HOLD between 90° increments would self-level =
        # a reversal, which scores nothing. Full-360° entries end level.
        restore = None
        if axis != 'yaw' and can_switch:
            restore = thrusters.flight_mode_name
            thrusters.set_flight_mode(style_mode)
            time.sleep(0.5)                 # let the mode take
        try:
            for _ in range(total):
                if axis == 'yaw':
                    ok = keeper.turn(direction, 90.0, speed=args.style_speed)
                    yaw_inc += 1
                else:
                    ok = keeper.rotate(axis, direction, 90.0,
                                       speed=args.rotate_speed)
                    rp_inc += 1
                done = (yaw_inc + rp_inc)
                print(f'  style increment {done} done'
                      + ('' if ok else '  (unverified/incomplete)'))
                if args.style_pause > 0:
                    time.sleep(args.style_pause)
        finally:
            if restore is not None:
                thrusters.set_flight_mode(restore)
        # re-latch depth after each entry (roll/pitch suspend the heave law
        # and MANUAL has no autopilot depth hold — expect drift to re-hold)
        d = keeper.hold_here()
        if d is not None:
            print(f'  re-holding depth at {d:.2f} m')
    return yaw_inc, rp_inc


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    # dive
    ap.add_argument('--depth', type=float, default=3.0,
                    help='dive depth in FEET (default 3.0; gate is 1.5 m tall)')
    ap.add_argument('--settle-tol', type=float, default=0.12,
                    help='depth error (m) counting as "at depth"')
    ap.add_argument('--depth-hold-s', type=float, default=3.0,
                    help='seconds to hold depth before searching')
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
    # vision
    ap.add_argument('--label', default='gate')
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--onnx', default=None,
                    help='absolute path to the detector model '
                         '(default: vision/ffc_rs_26.onnx)')
    # search
    ap.add_argument('--search-timeout', type=float, default=30.0)
    ap.add_argument('--search-sweep', type=float, default=2.5,
                    help='seconds per sweep direction')
    ap.add_argument('--search-speed', type=float, default=0.2)
    # approach
    ap.add_argument('--speed', type=float, default=0.35,
                    help='max approach surge (0-1)')
    ap.add_argument('--creep-speed', type=float, default=0.15,
                    help='min surge while off-center / during dropouts')
    ap.add_argument('--strafe-gain', type=float, default=0.8,
                    help='strafe effort per unit center-x error')
    ap.add_argument('--strafe-max', type=float, default=0.35)
    ap.add_argument('--ex-slow', type=float, default=0.5,
                    help='|ex| at which surge is fully throttled to creep')
    ap.add_argument('--center-tol', type=float, default=0.10,
                    help='|ex| counting as centered for the commit')
    ap.add_argument('--cx-target', type=float, default=0.5,
                    help='image-x to center the gate on (0.5 = middle; '
                         'shift to pick a side / role imagery)')
    ap.add_argument('--close-frac', type=float, default=0.5,
                    help='normalised bbox width meaning "at the gate"')
    ap.add_argument('--lost-frames', type=int, default=8,
                    help='consecutive lost ticks (10 Hz) → commit the pass')
    ap.add_argument('--approach-timeout', type=float, default=60.0)
    # pass-through
    ap.add_argument('--pass-time', type=float, default=5.0,
                    help='seconds of committed straight push through the gate')
    ap.add_argument('--pass-speed', type=float, default=0.45)
    # style
    ap.add_argument('--style', default='roll:720,yaw:720',
                    help='comma list of axis:degrees entries, run in order; '
                         'sign = direction, each a multiple of 90. Roll/pitch '
                         'outscore yaw (6DOF frame). Default '
                         '"roll:720,yaw:720". First water test: "roll:360".')
    ap.add_argument('--style-speed', type=float, default=fc.TURN_SPEED,
                    help='yaw effort during style spins')
    ap.add_argument('--rotate-speed', type=float, default=fc.ROTATE_SPEED,
                    help='roll/pitch effort during style spins')
    ap.add_argument('--style-pause', type=float, default=0.5,
                    help='pause between 90° increments')
    ap.add_argument('--style-mode', default='MANUAL',
                    choices=['MANUAL', 'ACRO'],
                    help='flight mode during roll/pitch entries — ALT_HOLD '
                         'self-levels and fights continuous rotation '
                         '(default MANUAL: raw passthrough)')
    ap.add_argument('--no-style', action='store_true',
                    help='pass the gate only, skip the spins')
    # exit
    ap.add_argument('--clear-time', type=float, default=2.0,
                    help='seconds of forward after the style spin to clear '
                         'the gate area (0 = none)')
    ap.add_argument('--surface', action='store_true',
                    help='closed-loop climb to the surface at the end')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    try:
        style_plan = [] if args.no_style else parse_style_plan(args.style)
    except ValueError as e:
        ap.error(f'--style: {e}')

    target_m = args.depth * fc.FEET_TO_M

    style_desc = ('none' if not style_plan else ', '.join(
        f'{a} {"+" if d > 0 else "-"}{deg:.0f}°' for a, d, deg in style_plan))
    confirm = (f'TASK 1 GATE RUN: dive {args.depth:.1f} ft, find + pass the '
               f'gate, then style [{style_desc}]. ROLL/PITCH WILL ROTATE THE '
               f'HULL. THRUSTERS WILL SPIN.')

    factory = fc.spawn_vision_factory(model_onnx=args.onnx)
    with fc.session(factory, confirm_msg=confirm,
                    skip_confirm=args.yes) as (driver, extra):
        det = fc.find_node(extra, fc.DetectionMonitor)

        # Bar02 depth + ATTITUDE off the shared MAVLink link. If the Bar02 is
        # missing (intermittent I2C) the source still streams ATTITUDE, so
        # roll/pitch style stays closed-loop; depth falls back to open loop.
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

        yaw_inc = rp_inc = 0
        try:
            # 1 — dive
            if have_depth:
                dive(keeper, target_m, args.settle_tol, args.depth_hold_s)
            else:
                dive_open_loop(keeper, args.dive_heave, args.dive_time)

            # 2 — search
            if not search_gate(keeper, det, args):
                print('No gate found — committing a blind straight pass '
                      '(better than sitting still).')

            # 3+4 — approach + pass
            approach_and_pass(keeper, det, args)

            # 5 — style
            if style_plan:
                yaw_inc, rp_inc = style_spin(keeper, style_plan, args)

            # 6 — clear the area / exit
            if args.clear_time > 0:
                print(f'[CLEAR] forward {args.clear_time:.1f}s.')
                keeper.set_move(surge=args.speed, ramp=0.5)
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
            print(f'\nTASK 1 done — style increments: {rp_inc} x 90° '
                  f'roll/pitch (high value) + {yaw_inc} x 90° yaw = '
                  f'{(rp_inc + yaw_inc) * 90}° total.')

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
