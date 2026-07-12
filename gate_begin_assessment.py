#!/usr/bin/env python3
"""Tasks 0+1 — Coin Flip + Begin Assessment (gate) in ONE run (RoboSub 2026).

Single mission script (the old bt_coinflip.py is merged in — nothing runs
before this). Start at the SURFACE in the start zone, any orientation (the
coin flip randomizes it). One armed session end to end: latch surface
pressure, dive, coin-flip spin until a gate role image is seen, and flow
straight into gate centering — no disarm, no surfacing, no baseline re-latch
between tasks (surfacing mid-run is not allowed in competition).

The ffc_rs_26 model has NO 'gate' class. The gate side is located by the
ROLE IMAGE hung on it — the SAME image the coin flip stopped on, passed in
with --image (or explicit --labels):

    --image survey   →  compass + hammer_and_wrench   (Survey & Repair side)
    --image rescue   →  buoy (life ring) + sos        (Search & Rescue side)

Both symbols share one board, so fresh detections of either/both classes are
fused into one union box; its center is what we steer at, its width is the
"how close" measure.

Sequence (every move runs THROUGH the DepthKeeper — Bar02 depth held
closed-loop underneath, ZED heading latched on straight legs):

    1. DIVE     : closed-loop to --depth ft (timed fallback if no Bar02).
    2. GATE     : COIN FLIP + acquire: continuous spin in --search-dir at
                  --turn-speed until ANY role-image class is seen (the model
                  has no 'gate' class), then center on the union box —
                  squares the randomized start heading onto the structure.
                  Re-acquires use an EXPANDING alternating sweep (legs grow
                  1x, 2x, 3x… --search-sweep), never a fixed wiggle.
    3. CENTER   : acquire + center on the REQUESTED side image (--image).
                  Lost or timed out → re-acquire; after the normal retries,
                  one extended round at --extra-find-scale x timeouts.
                  Centering must genuinely succeed for style to run later.
    4. SUBMERGE : deepen by --extra-depth-ft (default 0.75 ft; or absolute
                  --gate-depth) for a clean pass; the yaw servo stays on so
                  the image stays centered while sinking.
    5. APPROACH : creep at the image — strafe + gentle yaw trim on the
                  centering error, surge throttled while off-center.
    6. PASS     : union width >= --close-frac (or image lost close-in) →
                  committed straight, timed push through that side.
    7. STYLE    : ONLY runs if step 3 genuinely centered on the requested
                  image — never on a blind/uncentered run.
                  --style plan in verified 90° increments. 2026 rules
                  (§3.2): every 90° orientation change scores; undoing the
                  previous rotation does NOT (continuous one-direction spins
                  never do); roll and pitch are worth MORE than yaw; NO cap
                  is published — more clean increments = more points, bounded
                  only by run time and reliability. Default plan
                  "roll:720,pitch:360,yaw:720" = 12 high-value roll/pitch
                  increments + 8 yaw increments, ending level after each
                  entry. pitch:360 is a full loop-the-loop — clear the gate
                  first (we style after the pass) and expect depth excursion.
    8. EXIT     : forward to clear the area, keep depth (or --surface),
                  report the increment count.

  python3 gate_begin_assessment.py --image survey --yes
  python3 gate_begin_assessment.py --image rescue --gate-depth 4.5
  python3 gate_begin_assessment.py --image survey --dry     # motors OFF
  python3 gate_begin_assessment.py --style roll:360 --image survey
                                                       # first water test

⚠ ARMS the Pixhawk, drives REAL thrusters, ROLLS AND LOOPS THE HULL during
style. Stop thruster_node first (single serial owner); do not run vslam_node
(the detector owns the ZED). Tether, kill switch, props clear. Ctrl+C →
stop + disarm.

Requires sourced workspace:
    source /opt/ros/humble/setup.bash && source install/setup.bash
"""

import argparse
import os
import time

import field_common as fc
import gate_task as gt

# Deployed forward-camera model (prebuilt ffc_rs_26.engine sits beside it).
FFC_ONNX = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'src', 'vision', 'vision', 'ffc_rs_26.onnx')

# Gate side → ffc_rs_26 classes on its role image (§3.2 role selection).
ROLE_LABELS = {
    'survey': ['compass', 'hammer_and_wrench'],   # Survey & Repair
    'rescue': ['buoy', 'sos'],                    # Search & Rescue
}

# The model has no 'gate' class — "the gate" is the union of every role-image
# class on either side. Centering on this first squares us on the structure
# before we single out the requested side.
GATE_LABELS = ROLE_LABELS['survey'] + ROLE_LABELS['rescue']


# ─── Image fusion ─────────────────────────────────────────────────────────────

def board_view(det, labels, conf):
    """Fuse fresh detections of the side-image classes into one union box.

    Both symbols live on the same board, so the union over whichever classes
    are currently visible estimates the board: returns (center_x, width,
    hit_labels) in normalized image coords, or None if nothing fresh.
    """
    lefts, rights, hits = [], [], []
    for label in labels:
        d = det.best(label, conf)
        if d is None:
            continue
        half = d.bbox_width / 2.0
        lefts.append(d.position.x - half)
        rights.append(d.position.x + half)
        hits.append(label)
    if not hits:
        return None
    left, right = min(lefts), max(rights)
    return (left + right) / 2.0, right - left, hits


# ─── Phase 2: acquire ─────────────────────────────────────────────────────────

def acquire(keeper, det, args, labels=None, timeout=None, what='image',
            spin=False):
    """Target in frame? If not, search until it is.

    spin=True  → continuous turn in --search-dir at --turn-speed: the
                 coin-flip search. After the randomized start the target can
                 be ANYWHERE, so cover the full circle (proven in the water
                 2026-07-10: found the image in ~7 s).
    spin=False → expanding alternating sweep for re-acquires: leg n lasts
                 n x --search-sweep, so the arc widens each reversal instead
                 of wiggling over the same few degrees forever (the fixed
                 ±2.5 s sweep is why the 2026-07-10 run never re-found the
                 image in 100 s).
    """
    labels = labels if labels is not None else args.labels
    timeout = timeout if timeout is not None else args.search_timeout
    if board_view(det, labels, args.conf) is not None:
        print(f'✓ {what} already in frame.')
        return True
    direction = -1 if args.search_dir == 'left' else 1
    mode = 'spinning' if spin else 'sweeping'
    speed = args.turn_speed if spin else args.search_speed
    print(f'[ACQUIRE] {mode} ({args.search_dir} first) for {labels} '
          f'(timeout {timeout:.0f}s)…')
    t0 = time.time()
    leg = 1
    leg_t0 = t0
    try:
        while time.time() - t0 < timeout:
            if board_view(det, labels, args.conf) is not None:
                print(f'✓ {what} acquired.')
                return True
            if not spin and time.time() - leg_t0 > leg * args.search_sweep:
                direction *= -1
                leg += 1
                leg_t0 = time.time()
            keeper.set_move(yaw=direction * speed, ramp=0.5)
            time.sleep(1.0 / fc.RATE_HZ)
    finally:
        keeper.clear_move(ramp=0.3)
    print('✗ ACQUIRE timeout — nothing seen.')
    return False


# ─── Phase 3: center (handles the skewed coin-flip heading) ──────────────────

def center_on_image(keeper, det, args, labels=None, timeout=None,
                    what='image'):
    """Yaw-servo the union box onto --cx-target; done once it stays inside
    --center-tol continuously for --center-hold s.

    Returns True only when genuinely centered. False on loss (caller
    re-acquires) AND on timeout — style must never run on an un-centered
    heading, so a timeout is a failure, not a shrug.
    """
    labels = labels if labels is not None else args.labels
    timeout = timeout if timeout is not None else args.center_timeout
    print(f'[CENTER {what}] yaw-servo to cx={args.cx_target:.2f} '
          f'±{args.center_tol:.2f}, hold {args.center_hold:.1f}s…')
    period = 1.0 / fc.RATE_HZ
    good_t = None
    lost = 0
    last_print = 0.0
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            view = board_view(det, labels, args.conf)
            now = time.time()
            if view is None:
                lost += 1
                good_t = None
                if lost >= args.lost_frames:
                    print(f'  {what} lost while centering.')
                    return False
                keeper.set_move(ramp=0.3)      # hold heading, wait it out
                time.sleep(period)
                continue
            lost = 0
            cx, frac, hits = view
            ex = cx - args.cx_target           # +ve → image to the right
            if abs(ex) <= args.center_tol:
                keeper.set_move(ramp=0.3)      # inside band: stop, latch
                if good_t is None:
                    good_t = now
                elif now - good_t >= args.center_hold:
                    print(f'✓ Centered (ex={ex:+.2f}, frac={frac:.2f}, '
                          f'via {",".join(hits)}).')
                    return True
            else:
                good_t = None
                yaw = fc.clamp(args.yaw_gain * ex, -args.yaw_max, args.yaw_max)
                keeper.set_move(yaw=yaw, ramp=0.3)
            if now - last_print >= 1.0:
                last_print = now
                print(f'  ex={ex:+.2f} frac={frac:.2f} via {",".join(hits)}')
            time.sleep(period)
    finally:
        keeper.clear_move(ramp=0.3)
    print(f'  center timeout — NOT centered on {what}.')
    return False


def acquire_and_center(keeper, det, args, labels, what, retries,
                       timeout_scale=1.0, spin_first=False):
    """Acquire + center retry loop on one label set. Returns True only if
    genuinely centered; timeouts and losses burn retries, never fake
    success. spin_first=True → attempt 1 searches with a continuous
    full-circle spin (coin flip); later attempts sweep locally."""
    for attempt in range(1, retries + 1):
        if not acquire(keeper, det, args, labels=labels,
                       timeout=args.search_timeout * timeout_scale,
                       what=what, spin=spin_first and attempt == 1):
            return False                    # search found nothing at all
        if center_on_image(keeper, det, args, labels=labels,
                           timeout=args.center_timeout * timeout_scale,
                           what=what):
            return True
        print(f'  re-acquiring {what} ({attempt}/{retries})…')
    return False


# ─── Phase 4: submerge to gate pass depth ────────────────────────────────────

def submerge_more(keeper, det, args, have_depth, target_m):
    """Deepen to the gate pass depth; yaw servo keeps the image centered
    while sinking (perspective shifts it). Image dropping out high in the
    frame is expected near depth — heading stays latched."""
    if have_depth:
        print(f'[SUBMERGE] → {target_m:.2f} m (gate pass depth), '
              f'image-centering stays on…')
        keeper.set_target(target_m)
    else:
        print(f'[SUBMERGE] open-loop {args.extra_dive_time:.1f}s @ heave '
              f'{args.dive_heave:.2f} (no Bar02), image-centering stays on…')
        keeper.set_heave_override(args.dive_heave)
    period = 1.0 / fc.RATE_HZ
    good_t = None
    last_print = 0.0
    t0 = time.time()
    try:
        while time.time() - t0 < args.submerge_timeout:
            view = board_view(det, args.labels, args.conf)
            if view is not None and abs(view[0] - args.cx_target) > args.center_tol:
                ex = view[0] - args.cx_target
                yaw = fc.clamp(args.yaw_gain * ex, -args.yaw_max, args.yaw_max)
                keeper.set_move(yaw=yaw, ramp=0.3)
            else:
                keeper.set_move(ramp=0.3)
            if not have_depth:
                if time.time() - t0 >= args.extra_dive_time:
                    print('✓ Timed submerge done.')
                    break
            else:
                d = keeper.depth()
                if time.time() - last_print >= 1.0:
                    last_print = time.time()
                    print(f'  depth={d:.2f} m → {target_m:.2f} m'
                          if d is not None else '  depth=n/a')
                if d is not None and abs(target_m - d) <= args.settle_tol:
                    if good_t is None:
                        good_t = time.time()
                    elif time.time() - good_t >= 1.0:
                        print(f'✓ At gate depth {d:.2f} m.')
                        break
                else:
                    good_t = None
            time.sleep(period)
        else:
            print('  submerge timeout — proceeding at current depth.')
    finally:
        if not have_depth:
            keeper.set_heave_override(None)
        keeper.clear_move(ramp=0.3)


# ─── Phases 5+6: approach + pass-through ─────────────────────────────────────

def approach_and_pass(keeper, det, args):
    """Creep at the image: strafe + gentle yaw trim on the centering error,
    surge throttled while off-center. Commit a straight timed push once the
    union box is wide enough (or once the image is lost close-in)."""
    print(f'[APPROACH] cx={args.cx_target:.2f}, '
          f'commit at frac>={args.close_frac:.2f}…')
    period = 1.0 / fc.RATE_HZ
    lost = 0
    last_print = 0.0
    t0 = time.time()
    try:
        while time.time() - t0 < args.approach_timeout:
            view = board_view(det, args.labels, args.conf)
            if view is None:
                lost += 1
                if lost >= args.lost_frames:
                    print('  image lost after approach → committing pass.')
                    break
                # brief dropout: keep creeping straight, heading held
                keeper.set_move(surge=args.creep_speed, ramp=0.5)
                time.sleep(period)
                continue
            lost = 0
            cx, frac, hits = view
            ex = cx - args.cx_target
            if frac >= args.close_frac and abs(ex) <= args.center_tol:
                print(f'  close+centered (frac={frac:.2f} ex={ex:+.2f}) '
                      f'→ committing pass.')
                break
            strafe = fc.clamp(args.strafe_gain * ex,
                              -args.strafe_max, args.strafe_max)
            yaw = 0.0
            if abs(ex) > args.center_tol:      # inside band: latch heading
                yaw = fc.clamp(args.approach_yaw_gain * ex,
                               -args.approach_yaw_max, args.approach_yaw_max)
            # throttle surge down while off-center so we straighten first
            slow = fc.clamp(1.0 - abs(ex) / args.ex_slow, 0.0, 1.0)
            surge = args.creep_speed + (args.speed - args.creep_speed) * slow
            if frac >= args.close_frac:
                surge = 0.0    # at the mouth but off-center: square up first
            keeper.set_move(surge=surge, strafe=strafe, yaw=yaw, ramp=0.5)
            if time.time() - last_print >= 1.0:
                last_print = time.time()
                print(f'  ex={ex:+.2f} frac={frac:.2f} surge={surge:.2f} '
                      f'strafe={strafe:+.2f} yaw={yaw:+.2f}')
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


# ─── Dry run: vision-only centering check, motors OFF ────────────────────────

def dry_run(args):
    """Detector + DetectionMonitor only — no Pixhawk, no arming, NO MOTORS.
    Prints the fused board view and the yaw/strafe the servos WOULD command,
    once per second. Ctrl+C to stop."""
    import threading

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    print(f'[dry] vision-only — Pixhawk untouched, motors OFF.\n'
          f'[dry] steering target: union of {args.labels} @ conf ≥ '
          f'{args.conf:.2f}, cx={args.cx_target:.2f}. Ctrl+C to quit.')
    rclpy.init()
    nodes = fc.spawn_vision_factory(model_onnx=args.onnx,
                                    conf_thres=args.conf_thres,
                                    save_frames=args.save_frames)()
    det = fc.find_node(nodes, fc.DetectionMonitor)
    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            fresh = det.fresh()
            line = ('  '.join(f'{label}:{conf:.2f}' for label, conf in fresh)
                    or '(no detections)')
            view = board_view(det, args.labels, args.conf)
            if view is not None:
                cx, frac, hits = view
                ex = cx - args.cx_target
                yaw = fc.clamp(args.yaw_gain * ex, -args.yaw_max, args.yaw_max)
                strafe = fc.clamp(args.strafe_gain * ex,
                                  -args.strafe_max, args.strafe_max)
                commit = '  → WOULD COMMIT PASS' if (
                    frac >= args.close_frac and abs(ex) <= args.center_tol) else ''
                print(f'[dry] {line}\n'
                      f'      board cx={cx:.2f} ex={ex:+.2f} frac={frac:.2f} '
                      f'via {",".join(hits)} → yaw {yaw:+.2f} '
                      f'strafe {strafe:+.2f}{commit}')
            else:
                print(f'[dry] {line}   (no board — would hold/search)')
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('\n[dry] done.')
    finally:
        # Stop the detector loop BEFORE destroying nodes — it publishes from
        # its own thread and hits InvalidHandle on dead publishers otherwise.
        from vision import detector as det_mod
        det_mod.exit_signal = True
        time.sleep(1.0)
        try:
            executor.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        for n in nodes:
            try:
                n.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    # which side image (the one the coin flip stopped on)
    ap.add_argument('--image', choices=sorted(ROLE_LABELS), default='survey',
                    help='gate side to pass through = role image detected '
                         'during the coin flip (survey: compass+'
                         'hammer_and_wrench; rescue: buoy+sos). '
                         'Default survey.')
    ap.add_argument('--labels', default=None,
                    help='override: comma-separated detection classes for '
                         'the side image (beats --image)')
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--onnx', default=FFC_ONNX,
                    help='forward-camera model .onnx; prebuilt .engine '
                         'beside it is used (default: deployed ffc_rs_26)')
    ap.add_argument('--conf-thres', type=float, default=None,
                    help='detector publish gate (its default 0.4)')
    ap.add_argument('--save-frames', default=None, metavar='DIR',
                    help='dump 1 annotated frame/s (JPEG) to DIR')
    # depths
    ap.add_argument('--depth', type=float, default=2.5,
                    help='initial dive depth in FEET (from the surface '
                         'start; the coin-flip spin runs at this depth)')
    ap.add_argument('--gate-depth', type=float, default=None,
                    help='absolute pass-through depth in FEET; overrides '
                         '--extra-depth-ft when given')
    ap.add_argument('--extra-depth-ft', type=float, default=0.75,
                    help='"submerge more" step: pass depth = --depth + this '
                         'many FEET (default 0.75)')
    ap.add_argument('--settle-tol', type=float, default=0.12,
                    help='depth error (m) counting as "at depth"')
    ap.add_argument('--depth-hold-s', type=float, default=2.0,
                    help='seconds to hold the initial depth before acquire')
    ap.add_argument('--dive-time', type=float, default=5.0,
                    help='open-loop initial dive seconds if Bar02 missing')
    ap.add_argument('--extra-dive-time', type=float, default=3.0,
                    help='open-loop "submerge more" seconds if Bar02 missing')
    ap.add_argument('--dive-heave', type=float, default=0.4,
                    help='open-loop dive effort if Bar02 missing')
    ap.add_argument('--submerge-timeout', type=float, default=45.0)
    # depth keeper (defaults proven in depth_field_test)
    ap.add_argument('--kp', type=float, default=2.0)
    ap.add_argument('--min-speed', type=float, default=0.15)
    ap.add_argument('--max-speed', type=float, default=0.6)
    ap.add_argument('--deadband', type=float, default=0.07)
    ap.add_argument('--max-depth', type=float, default=0.0,
                    help='abort+surface past this (m); 0 → 2x gate depth')
    ap.add_argument('--water-density', type=float, default=1000.0)
    ap.add_argument('--yaw-kp', type=float, default=1.0)
    ap.add_argument('--yaw-hold-max', type=float, default=0.25)
    ap.add_argument('--yaw-hold-sign', type=float, default=1.0,
                    choices=[1.0, -1.0])
    # acquire
    ap.add_argument('--search-dir', choices=['left', 'right'], default='left',
                    help='coin-flip spin direction + first sweep direction')
    ap.add_argument('--turn-speed', type=float, default=0.25,
                    help='yaw effort of the continuous coin-flip spin '
                         '(first gate search)')
    ap.add_argument('--search-timeout', type=float, default=60.0,
                    help='seconds per acquire round (first round is the '
                         'full-circle coin-flip spin)')
    ap.add_argument('--search-sweep', type=float, default=2.5,
                    help='seconds per sweep direction')
    ap.add_argument('--search-speed', type=float, default=0.2)
    # center
    ap.add_argument('--cx-target', type=float, default=0.5,
                    help='image-x to center the board on (0.5 = middle)')
    ap.add_argument('--center-tol', type=float, default=0.08,
                    help='|ex| counting as centered')
    ap.add_argument('--center-hold', type=float, default=1.0,
                    help='seconds the board must stay centered')
    ap.add_argument('--yaw-gain', type=float, default=0.9,
                    help='centering yaw effort per unit ex')
    ap.add_argument('--yaw-max', type=float, default=0.3)
    ap.add_argument('--center-timeout', type=float, default=40.0)
    ap.add_argument('--acquire-retries', type=int, default=3,
                    help='re-acquire attempts if the image is lost while '
                         'centering')
    ap.add_argument('--extra-find-scale', type=float, default=1.5,
                    dest='extra_find_scale',
                    help='after the normal retries fail, one last '
                         'acquire+center round with search/center timeouts '
                         'scaled by this factor')
    # approach
    ap.add_argument('--speed', type=float, default=0.35,
                    help='max approach surge (0-1)')
    ap.add_argument('--creep-speed', type=float, default=0.15,
                    help='min surge while off-center / during dropouts')
    ap.add_argument('--strafe-gain', type=float, default=0.8,
                    help='strafe effort per unit ex')
    ap.add_argument('--strafe-max', type=float, default=0.35)
    ap.add_argument('--approach-yaw-gain', type=float, default=0.35,
                    help='gentle yaw trim per unit ex during the approach')
    ap.add_argument('--approach-yaw-max', type=float, default=0.15)
    ap.add_argument('--ex-slow', type=float, default=0.5,
                    help='|ex| at which surge is fully throttled to creep')
    ap.add_argument('--close-frac', type=float, default=0.35,
                    help='union-box width fraction meaning "at the gate" '
                         '(the role image is smaller than the gate — lower '
                         'than gate_task\'s 0.5)')
    ap.add_argument('--lost-frames', type=int, default=8,
                    help='consecutive lost ticks (10 Hz) → treat as lost')
    ap.add_argument('--approach-timeout', type=float, default=60.0)
    # pass-through
    ap.add_argument('--pass-time', type=float, default=6.0,
                    help='seconds of committed straight push through')
    ap.add_argument('--pass-speed', type=float, default=0.45)
    # style
    ap.add_argument('--style', default='roll:720,pitch:360,yaw:720',
                    help='comma list of axis:degrees entries, run in order; '
                         'sign = direction, each a nonzero multiple of 90, '
                         'each spun ONE continuous direction (reversals '
                         'never score). Roll/pitch outscore yaw; the rules '
                         'publish NO cap — add more roll for more points if '
                         'the clock allows. Default "roll:720,pitch:360,'
                         'yaw:720". First water test: "roll:360".')
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
                    help='seconds of forward after style to clear the area')
    ap.add_argument('--surface', action='store_true',
                    help='closed-loop climb to the surface at the end')
    ap.add_argument('--dry', action='store_true',
                    help='vision-only centering check: no Pixhawk, no '
                         'arming, NO MOTORS')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    if args.labels:
        args.labels = [s.strip() for s in args.labels.split(',') if s.strip()]
    else:
        args.labels = ROLE_LABELS[args.image]

    try:
        style_plan = [] if args.no_style else gt.parse_style_plan(args.style)
    except ValueError as e:
        ap.error(f'--style: {e}')

    if args.dry:
        return dry_run(args)

    gate_ft = (args.gate_depth if args.gate_depth is not None
               else args.depth + args.extra_depth_ft)
    start_m = args.depth * fc.FEET_TO_M
    gate_m = gate_ft * fc.FEET_TO_M

    style_desc = ('none' if not style_plan else ', '.join(
        f'{a} {"+" if d > 0 else "-"}{deg:.0f}°' for a, d, deg in style_plan))
    confirm = (f'TASKS 0+1 COIN FLIP + BEGIN ASSESSMENT: latch surface, '
               f'dive {args.depth:.1f} ft, spin until a role image is seen, '
               f'center on the GATE then the {args.image} image '
               f'({"+".join(args.labels)}), submerge to {gate_ft:.1f} ft, '
               f'pass through that side, then style [{style_desc}] — ONLY '
               f'if centered. ROLL/PITCH WILL ROTATE THE HULL. THRUSTERS '
               f'WILL SPIN.')

    factory = fc.spawn_vision_factory(model_onnx=args.onnx,
                                      conf_thres=args.conf_thres,
                                      save_frames=args.save_frames)
    with fc.session(factory, confirm_msg=confirm,
                    skip_confirm=args.yes) as (driver, extra):
        det = fc.find_node(extra, fc.DetectionMonitor)

        # Bar02 depth + ATTITUDE off the shared MAVLink link. If the Bar02
        # is missing (intermittent I2C) the source still streams ATTITUDE,
        # so roll/pitch style stays closed-loop; depth goes open loop.
        src = None
        limit = None
        master = getattr(driver.thrusters, 'master', None)
        if master is not None:
            src = fc.Bar02DepthSource(master, rho=args.water_density)
            limit = src.setup()
            if limit is None:
                print('Bar02 unavailable — open-loop dives + ALT_HOLD only '
                      '(ATTITUDE feedback still on).')
            src.start()
        else:
            print('No MAVLink master — simulation? continuing without depth.')
        have_depth = limit is not None

        max_depth_m = args.max_depth if args.max_depth > 0 else 2.0 * gate_m
        if limit is not None:
            max_depth_m = min(max_depth_m, limit)
            if gate_m > limit:
                print(f'gate depth {gate_ft:.1f} ft exceeds Bar02 range '
                      f'({limit:.1f} m). Abort.')
                return 1

        keeper = fc.DepthKeeper(
            driver, src, max_depth_m,
            kp=args.kp, min_speed=args.min_speed, max_speed=args.max_speed,
            deadband=args.deadband, yaw_kp=args.yaw_kp,
            yaw_hold_max=args.yaw_hold_max, yaw_hold_sign=args.yaw_hold_sign)
        keeper.start()

        yaw_inc = rp_inc = 0
        try:
            # 1 — dive from the surface start
            if have_depth:
                gt.dive(keeper, start_m, args.settle_tol, args.depth_hold_s)
            else:
                gt.dive_open_loop(keeper, args.dive_heave, args.dive_time)

            # 2 — COIN FLIP + gate: continuous spin until any role-image
            # class is seen (start heading is randomized), then center on
            # the union box: squares us on the structure first
            gate_centered = acquire_and_center(
                keeper, det, args, GATE_LABELS, 'gate',
                retries=args.acquire_retries, spin_first=True)
            if not gate_centered:
                print('Gate not centered — extra search round '
                      f'({args.extra_find_scale:.1f}x timeouts)…')
                gate_centered = acquire_and_center(
                    keeper, det, args, GATE_LABELS, 'gate', retries=1,
                    timeout_scale=args.extra_find_scale, spin_first=True)

            # 3 — center on the REQUESTED side image (--image)
            centered = acquire_and_center(
                keeper, det, args, args.labels, args.image,
                retries=args.acquire_retries)
            if not centered:
                print(f'{args.image} image not centered — extra search '
                      f'round ({args.extra_find_scale:.1f}x timeouts)…')
                centered = acquire_and_center(
                    keeper, det, args, args.labels, args.image, retries=1,
                    timeout_scale=args.extra_find_scale)
            if not centered:
                print('Never centered — committing a blind straight run '
                      '(better than sitting still). STYLE WILL BE SKIPPED.')

            # 4 — submerge to the pass depth, image-centering still live
            submerge_more(keeper, det, args, have_depth, gate_m)

            # 5+6 — approach + pass
            approach_and_pass(keeper, det, args)

            # 7 — style: ONLY if we were genuinely centered on the requested
            # image — spinning while misaligned risks fouling the gate and
            # scores nothing verifiable
            if style_plan and centered:
                yaw_inc, rp_inc = gt.style_spin(keeper, style_plan, args)
            elif style_plan:
                print('[STYLE] skipped — never centered on the '
                      f'{args.image} image.')

            # 8 — clear the area / exit
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
            print(f'\nTASKS 0+1 (Coin Flip + Begin Assessment) done — '
                  f'style increments: '
                  f'{rp_inc} x 90° roll/pitch (high value) + {yaw_inc} x 90° '
                  f'yaw = {(rp_inc + yaw_inc) * 90}° total.')

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
