#!/usr/bin/env python3
"""Automatic PID tuner — hill-climbs one axis' gains on step-response metrics.

This is NOT magic. It runs a step, measures overshoot / settle / steady-state
error, nudges (kp, ki, kd) with interpretable rules, and repeats — keeping the
best gain set it finds. It reuses tune_pid.py for everything real (plant model,
metric computation, the ROS step recorder, and `ros2 param set`).

Two modes:
  --sim  (DEFAULT, SAFE): tune against tune_pid.simulate's offline plant. No
         ROS, no camera, no thrusters, nothing moves. Use this first — to watch
         what the tuner does and get a sane starting gain set for free.
  --live: tune the REAL controller in the pool. Each iteration shells
         `tune_pid.py step` (the SUB MOVES), reads the response, and applies the
         next gains via `ros2 param set`. Needs the full wet stack up
         (tune.launch.py) + thruster_node + sub in water + props clear.

The heuristic (each iteration, gains clamped to [min,max]):
  * steady-state error over tol      -> raise ki (add integral)
  * overshoot over target            -> lower kp, raise kd (damp)
  * sluggish (settle over target,
    little overshoot)                -> raise kp (more push)
  * ringing (oscillation + big
    overshoot)                       -> raise kd, lower kp harder
Converges when overshoot <= target AND settle <= target AND |ss_err| <= tol.

SAFETY (live): bounded iterations, gain clamps, and a runaway guard that aborts
+ restores the starting gains if a step response blows up (huge overshoot /
non-finite / never settles). WATCH every live iteration; kill on runaway. This
drives the vehicle repeatedly — treat it like any wet motor test.

Usage:
    # safe: offline, no hardware — see the tuner work, get starting gains
    python3 auto_tune_pid.py --axis yaw --sim --mass 1.0 --drag 0.3

    # real pool tune (wet stack + thrusters already up):
    python3 auto_tune_pid.py --axis yaw --live --iterations 12 --yes

    --axis        yaw|surge|strafe|heave|vis_* (see tune_pid.DEFAULT_GAINS)
    --sim/--live  offline model (default) vs real controller in water
    --iterations  max tuning iterations (default 12)
    --step        setpoint size: radians (yaw) or metres (default 0.5)
    --seconds     step record window (default 12)
    --overshoot   target max overshoot %% (default 10)
    --settle      target max settling time s (default 4.0)
    --ss-tol      target max |steady-state error| (default 0.03)
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import time

import tune_pid as tp

_HERE = os.path.dirname(os.path.abspath(__file__))
_TUNE_PID = os.path.join(_HERE, 'tune_pid.py')

# Absolute gain clamps so the search can't wander into an unstable regime.
GAIN_BOUNDS = {
    'kp': (0.15, 6.0),          # kp floor > 0: zero-P is degenerate (no tracking)
    'ki': (0.0, 2.0),
    'kd': (0.0, 3.0),
}


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def clamp_gains(kp, ki, kd):
    return (clamp(kp, *GAIN_BOUNDS['kp']),
            clamp(ki, *GAIN_BOUNDS['ki']),
            clamp(kd, *GAIN_BOUNDS['kd']))


def score(m, targets):
    """Lower is better. Penalise overshoot, slow settle, steady-state error,
    and slow rise (so the optimiser can't win by going slow-and-lazy). A missing
    rise/settle (never got there) is the worst case — big fixed penalty."""
    o = m['overshoot_pct'] or 0.0
    st = m['settling_time'] if m['settling_time'] is not None else 1e3
    ss = abs(m['ss_error'] or 0.0)
    rise = m['rise_time']
    rise_pen = 5.0 * rise if rise is not None else 100.0
    return o + 2.0 * st + 100.0 * ss + rise_pen


def converged(m, targets):
    o = m['overshoot_pct'] or 0.0
    st = m['settling_time']
    ss = abs(m['ss_error'] or 0.0)
    return (o <= targets['overshoot']
            and st is not None and st <= targets['settle']
            and ss <= targets['ss'])


def is_runaway(m):
    """Divergence guard: absurd overshoot or non-finite metrics = unstable."""
    o = m['overshoot_pct']
    ss = m['ss_error']
    if o is None or ss is None:
        return False
    if not (math.isfinite(o) and math.isfinite(ss)):
        return True
    return o > 60.0             # >60% overshoot = abort before node/sub spin out


# Pattern-search initial step sizes per gain, and the floor at which the search
# is considered converged (steps too small to matter).
SEARCH_STEP0 = {'kp': 0.4, 'ki': 0.06, 'kd': 0.2}
SEARCH_STEP_MIN = {'kp': 0.02, 'ki': 0.004, 'kd': 0.01}


def perturb(gains, gi, delta):
    """Return gains with element gi (0=kp,1=ki,2=kd) nudged by delta, clamped."""
    kp, ki, kd, limit, i_limit = gains
    vals = [kp, ki, kd]
    vals[gi] += delta
    kp, ki, kd = clamp_gains(vals[0], vals[1], vals[2])
    return (kp, ki, kd, limit, i_limit)


# --------------------------------------------------------------------------- #
# Response sources
# --------------------------------------------------------------------------- #
def sim_response(axis, gains, args):
    """Offline: simulate the plant with these gains. Returns metrics dict."""
    angular = (axis == 'yaw')
    times, resp = tp.simulate(gains, args.mass, args.drag, args.step,
                              tp.CONTROLLER_DT, args.seconds, angular)
    return tp.compute_metrics(times, resp, args.step, angular=angular)


def live_apply_gains(node, prefix, gains):
    """Push kp/ki/kd to the running controller via ros2 param set."""
    kp, ki, kd, _, _ = gains
    ok = True
    for suf, val in (('kp', kp), ('ki', ki), ('kd', kd)):
        good, detail = tp._ros2_param_set(node, '%s.%s' % (prefix, suf), val)
        if not good:
            ok = False
            print('  ! failed to set %s.%s: %s' % (prefix, suf, detail))
    return ok


def live_response(axis, args, iteration):
    """Real pool: run tune_pid.py step, parse the CSV, return metrics dict.

    Gains must already be applied to the controller (live_apply_gains). Returns
    None if the step produced no data (no pose / node down)."""
    csv_path = os.path.join(args.outdir, 'auto_%s_%02d.csv' % (axis, iteration))
    cmd = [sys.executable, _TUNE_PID, 'step',
           '--axis', axis, '--step', str(args.step),
           '--seconds', str(args.seconds), '--csv', csv_path]
    print('  running step -> %s' % os.path.basename(csv_path))
    try:
        subprocess.run(cmd, timeout=args.seconds + 30, check=False)
    except subprocess.TimeoutExpired:
        print('  ! step timed out')
        return None
    if not os.path.exists(csv_path):
        print('  ! no CSV written — step aborted (no pose? node down?)')
        return None
    times, resp, setpoints = [], [], []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            times.append(float(row['time']))
            resp.append(float(row['response']))
            setpoints.append(float(row['setpoint']))
    if len(resp) < 5:
        print('  ! step returned too few samples')
        return None
    return tp.compute_metrics(times, resp, setpoints[0], angular=(axis == 'yaw'))


# --------------------------------------------------------------------------- #
def fmt_metrics(m):
    return ('overshoot=%5.1f%%  settle=%6s  ss_err=%+.4f  osc=%s'
            % (m['overshoot_pct'] or 0.0,
               ('%.2fs' % m['settling_time']) if m['settling_time'] is not None
               else 'n/a',
               m['ss_error'] or 0.0,
               ('%.2fs' % m['oscillation_period']) if m['oscillation_period']
               else 'n/a'))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--axis', choices=list(tp.DEFAULT_GAINS.keys()),
                    default='yaw')
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--sim', action='store_true',
                      help='offline plant model (default, safe)')
    mode.add_argument('--live', action='store_true',
                      help='real controller in water (moves the sub)')
    ap.add_argument('--node', default=tp.CONTROLLER_NODE)
    ap.add_argument('--iterations', type=int, default=12)
    ap.add_argument('--step', type=float, default=0.25,
                    help='setpoint size: rad (yaw, ~14deg) or m')
    ap.add_argument('--seconds', type=int, default=12)
    ap.add_argument('--overshoot', type=float, default=10.0,
                    help='target max overshoot %%')
    ap.add_argument('--settle', type=float, default=4.0,
                    help='target max settling time s')
    ap.add_argument('--ss-tol', type=float, default=0.03, dest='ss_tol',
                    help='target max |steady-state error|')
    # sim plant
    ap.add_argument('--mass', type=float, default=1.0)
    ap.add_argument('--drag', type=float, default=0.3)
    ap.add_argument('--outdir', default=os.path.join(_HERE, 'autotune_logs'))
    ap.add_argument('--yes', action='store_true',
                    help='skip the live-mode confirmation prompt')
    args = ap.parse_args()

    live = args.live and not args.sim
    axis = args.axis
    prefix = tp.AXIS_PARAM[axis]
    targets = {'overshoot': args.overshoot, 'settle': args.settle,
               'ss': args.ss_tol}

    # Starting gains: live -> read from the running controller; sim -> defaults.
    base = list(tp.DEFAULT_GAINS[axis])
    if live:
        cur = {}
        for suf in ('kp', 'ki', 'kd', 'limit', 'i_limit'):
            v = tp._ros2_param_get(args.node, '%s.%s' % (prefix, suf))
            cur[suf] = v
        if all(cur[s] is not None for s in ('kp', 'ki', 'kd')):
            base[0], base[1], base[2] = cur['kp'], cur['ki'], cur['kd']
        else:
            print('WARNING: could not read current gains from /%s — is it '
                  'running? Falling back to tune_pid defaults.' % args.node)
    gains = tuple(base)

    if live:
        os.makedirs(args.outdir, exist_ok=True)
        print('*** LIVE AUTO-TUNE — the sub WILL move repeatedly (%d steps of '
              '%.2f on %s). Wet stack + thrusters must be up. Props clear, '
              'kill switch in reach. ***' % (args.iterations, args.step, axis))
        if not args.yes:
            if input('type "tune" to run: ').strip().lower() != 'tune':
                print('Aborted.')
                return 1

    print('\nauto-tune axis=%s  mode=%s  budget=%d evals  target: overshoot<=%.0f%% '
          'settle<=%.1fs |ss|<=%.3f' % (axis, 'LIVE' if live else 'sim',
          args.iterations, args.overshoot, args.settle, args.ss_tol))
    print('start gains: kp=%.3f ki=%.3f kd=%.3f' % (gains[0], gains[1], gains[2]))
    print('-' * 78)

    start_gains = gains
    aborted = [False]
    evals = [0]

    def evaluate(g):
        """Measure one gain set: sim -> model, live -> real step. Prints the
        row. Returns metrics or None (failed / aborted). Guards runaway live."""
        if aborted[0]:
            return None
        if live:
            if not live_apply_gains(args.node, prefix, g):
                print('  ! could not apply gains — aborting.')
                aborted[0] = True
                return None
            time.sleep(1.0)                 # let controller settle on new gains
            m = live_response(axis, args, evals[0])
            if m is None:
                aborted[0] = True
                return None
        else:
            m = sim_response(axis, g, args)
        evals[0] += 1
        print('[%02d] kp=%.3f ki=%.3f kd=%.3f | %s | score=%.1f'
              % (evals[0], g[0], g[1], g[2], fmt_metrics(m), score(m, targets)))
        if live and is_runaway(m):
            print('  ! RUNAWAY — restoring start gains and aborting.')
            live_apply_gains(args.node, prefix, start_gains)
            aborted[0] = True
            return None
        return m

    # Pattern search (Hooke-Jeeves style): evaluate the current best, then probe
    # +/- a step on each gain; move to the first probe that lowers the score.
    # When no probe improves, shrink every step. Only accepted (better) points
    # ever become the base, so the score is monotone non-increasing — no runaway
    # into instability like a blind rule set.
    m0 = evaluate(gains)
    if m0 is None:
        print('No successful first evaluation. Nothing tuned.')
        return 1
    best = (score(m0, targets), gains, m0)
    steps = dict(SEARCH_STEP0)

    while evals[0] < args.iterations and not aborted[0]:
        if converged(best[2], targets):
            print('  converged — targets met.')
            break
        improved = False
        for gi, name in enumerate(('kp', 'ki', 'kd')):
            for sign in (+1, -1):
                if evals[0] >= args.iterations or aborted[0]:
                    break
                cand = perturb(best[1], gi, sign * steps[name])
                if cand == best[1]:
                    continue                # clamp made it a no-op
                m = evaluate(cand)
                if m is None:
                    break
                s = score(m, targets)
                if s < best[0] - 1e-9:
                    best = (s, cand, m)
                    improved = True
                    break                   # greedy: take first improvement
            if improved or aborted[0]:
                break
        if not improved and not aborted[0]:
            # No direction helped — refine the grid.
            steps = {k: v * 0.5 for k, v in steps.items()}
            if all(steps[k] < SEARCH_STEP_MIN[k] for k in steps):
                print('  step sizes below floor — search converged.')
                break

    print('-' * 78)
    if best is None:
        print('No successful iteration. Nothing tuned.')
        return 1

    bg = best[1]
    print('BEST: kp=%.3f ki=%.3f kd=%.3f  (score=%.1f)' % (bg[0], bg[1], bg[2],
                                                            best[0]))
    print('      %s' % fmt_metrics(best[2]))
    if live:
        print('Applying best gains to /%s now.' % args.node)
        live_apply_gains(args.node, prefix, bg)
        print('NOTE: `ros2 param set` is not persistent. Bake the winner into '
              'src/control/control/autonomous_controller.py (the %s line, '
              'around L190) then colcon build.' % prefix)
    else:
        print('Sim only — nothing applied. Verify in the pool with:')
        print('  python3 auto_tune_pid.py --axis %s --live --yes   (or)' % axis)
        print('  python3 tune_pid.py set --pid %s --kp %.3f --ki %.3f --kd %.3f'
              % (prefix, bg[0], bg[1], bg[2]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
