#!/usr/bin/env python3
"""Automatic heading-PID tuner for the RoboSub motion stack (new ROS API).

The 2026 stack has ONE closed-loop PID left to tune: the HEADING (yaw) lock in
`motion_node`. Depth/roll/pitch are ArduSub's (ALT_HOLD); surge/strafe are
open-loop operator axes. So this tuner targets exactly those gains on
`motion_node`:

    heading_kp   heading_ki   heading_kd   i_limit

(The PID output limit is fixed at 1.0 in the node; the effective yaw-command
clamp is `max_yaw_correction`, a SAFETY bound this tuner reads but never sets.)

It hill-climbs the gains on a clean STEP RESPONSE. `motion_node` now accepts an
absolute heading command on `motion/heading` (a Float32 in radians) — the tuner
commands `cur_yaw + step`, the heading lock slews there, and the recovery is
measured off `imu/rpy`:

    rise time        (10% -> 90% of the step)
    overshoot %      (peak past the target, / step size)
    settling time    (into and staying within ±5% of the step)
    steady-state err (residual heading offset after settling)

Three loss knobs, interpretable: raise ki to kill steady-state offset, raise kd
(and/or lower kp) to damp overshoot + ringing, raise kp to speed a sluggish
rise. The optimiser is a Hooke-Jeeves pattern search: only accepted (lower
score) points ever become the base, so the score is monotone non-increasing —
it cannot wander into instability like a blind rule set.

Two modes:
  --sim  (DEFAULT, SAFE): tune against an offline angular plant using a faithful
         copy of control.pid.PID. No ROS, no thrusters, nothing moves. Run this
         first to watch the tuner and get a sane starting gain set for free.
         Caveat: the plant is a first-order guess (--inertia, --drag) — sim
         gains are a BALLPARK / starting point, not final numbers.
  --live: tune the REAL heading lock in the pool. The sub MUST already be
         submerged and HOLDING (launch the dive first — this tuner does NOT
         command it). Each iteration applies gains via `ros2 param set
         /motion_node`, commands a heading step on `motion/heading` (the SUB
         SLEWS), records the response from `imu/rpy`, then returns to the base
         heading. Needs the full wet stack up (thruster_node + imu bridge +
         motion_node) and the sub in HOLD. This IS the real controller on the
         real vehicle — trustworthy for heading-hold tuning.

SAFETY (live): bounded iterations, gain clamps, and a runaway guard that aborts
+ restores the starting gains if a response blows up (huge overshoot /
non-finite / never settles). WATCH every live iteration; kill on runaway. This
slews the vehicle repeatedly — treat it like any wet motor test.    

Usage:
    # safe: offline, no hardware — see the tuner work, get starting gains
    python3 auto_tune_pid.py --sim --inertia 1.0 --drag 0.3

    # real pool tune (wet stack up, sub already diving/HOLDING):
    python3 auto_tune_pid.py --live --iterations 12 --yes

    --sim/--live  offline plant (default) vs real heading lock in water
    --iterations  max tuning evaluations (default 12)
    --step        heading step size, rad (default 0.35 ≈ 20°)
    --seconds     response record window s (default 10)
    --overshoot   target max overshoot %% (default 10)
    --settle      target max settling time s (default 4.0)
    --ss-tol      target max |steady-state heading error|, rad (default 0.03)
    --inertia     sim plant rotational inertia (default 1.0)
    --drag        sim plant angular drag (default 0.3)
"""

import argparse
import csv
import math
import os
import sys
import time
from control.api import Auv, SubmergeError


# ── Node + parameter identity (the new API) ─────────────────────────────────
MOTION_NODE = 'motion_node'
GAIN_PARAMS = ('heading_kp', 'heading_ki', 'heading_kd')
I_LIMIT_PARAM = 'i_limit'

# Defaults mirror motion_node.PARAM_DEFAULTS so --sim starts where the real node
# starts. Order: (kp, ki, kd, i_limit).
DEFAULT_GAINS = (1.2, 0.0, 0.3, 0.3)
DEFAULT_MAX_YAW = 0.4         # motion_node max_yaw_correction default (the clamp)
OUTPUT_LIMIT = 1.0            # motion_node hard-codes PID(limit=1.0)

CONTROL_DT = 0.05             # motion_node runs the heading loop at 20 Hz
DERIV_CLAMP = 10.0            # matches control.pid.PID

_HERE = os.path.dirname(os.path.abspath(__file__))

# Absolute gain clamps so the search can't wander into an unstable regime.
GAIN_BOUNDS = {
    'kp': (0.05, 6.0),           # kp floor > 0: zero-P is degenerate (no hold)
    'ki': (0.0, 2.0),
    'kd': (0.0, 3.0),
}

SEARCH_STEP0 = {'kp': 0.4, 'ki': 0.06, 'kd': 0.2}
SEARCH_STEP_MIN = {'kp': 0.02, 'ki': 0.004, 'kd': 0.01}


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def clamp_gains(kp, ki, kd):
    return (clamp(kp, *GAIN_BOUNDS['kp']),
            clamp(ki, *GAIN_BOUNDS['ki']),
            clamp(kd, *GAIN_BOUNDS['kd']))


def wrap_angle(a):
    """Wrap radians to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class PID:
    """Faithful offline copy of control.pid.PID (the heading lock's real PID).

    Semantics pinned to that class: dt<=0 or dt>1 -> 0; non-finite in/out reset
    and return 0; integral clamped ±i_limit; derivative clamped ±10; output
    clamped ±limit; first call seeds prev_error (no derivative kick).
    """

    def __init__(self, kp, ki, kd, limit=OUTPUT_LIMIT, i_limit=0.3):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit, self.i_limit = limit, i_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def update(self, error, dt):
        if dt <= 0 or dt > 1.0:
            return 0.0
        if not math.isfinite(error):
            self.reset()
            return 0.0
        if not self._initialized:
            self._prev_error = error
            self._initialized = True
        self._integral += error * dt
        self._integral = max(-self.i_limit, min(self.i_limit, self._integral))
        derivative = (error - self._prev_error) / dt
        derivative = max(-DERIV_CLAMP, min(DERIV_CLAMP, derivative))
        self._prev_error = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        if not math.isfinite(out):
            self.reset()
            return 0.0
        return max(-self.limit, min(self.limit, out))


# --------------------------------------------------------------------------- #
# Step-response metrics — shared by sim and live. Angular-safe: each sample is
# remapped onto the branch nearest the setpoint so a response crossing the ±pi
# seam is not spurious overshoot. All metrics reference the STEP (y0 -> setpoint),
# because a live step starts at cur_yaw (e.g. -2.5 rad), not zero.
# --------------------------------------------------------------------------- #
def step_metrics(times, response, y0, setpoint):
    resp = [setpoint + wrap_angle(r - setpoint) for r in response]
    n = len(resp)
    if n == 0:
        return {}
    delta = wrap_angle(setpoint - y0)
    span = abs(delta) if abs(delta) > 1e-9 else 1.0

    # Rise time 10% -> 90% of the step (sign-agnostic: divide by delta).
    t10 = t90 = None
    for i in range(n):
        frac = (resp[i] - y0) / delta if abs(delta) > 1e-9 else 0.0
        if t10 is None and frac >= 0.10:
            t10 = times[i]
        if t90 is None and frac >= 0.90:
            t90 = times[i]
            break
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # Overshoot %: peak past setpoint in the step direction, / step span.
    if delta >= 0:
        peak = max(resp)
        overshoot = max(0.0, (peak - setpoint) / span * 100.0)
    else:
        peak = min(resp)
        overshoot = max(0.0, (setpoint - peak) / span * 100.0)

    # Settling time (±5% of step span): last time the response leaves the band.
    band = 0.05 * span
    settle = None
    for i in range(n - 1, -1, -1):
        if abs(resp[i] - setpoint) > band:
            settle = times[min(i + 1, n - 1)]
            break
    if settle is None:
        settle = times[0]                    # inside the band the whole time
    if abs(resp[-1] - setpoint) > band:
        settle = None                        # still out at the end: never settled

    tail = resp[max(0, n - max(1, n // 10)):]
    ss_val = sum(tail) / len(tail) if tail else resp[-1]
    ss_err = wrap_angle(ss_val - setpoint)

    period = None
    if t10 is not None:
        errs = [setpoint - r for r in resp]
        crossings, prev_sign = [], None
        for i in range(n):
            if times[i] < t10 or abs(errs[i]) < 1e-9:
                continue
            s = 1 if errs[i] > 0 else -1
            if prev_sign is not None and s != prev_sign:
                crossings.append(times[i])
            prev_sign = s
        if len(crossings) >= 2:
            intervals = [crossings[i + 1] - crossings[i]
                         for i in range(len(crossings) - 1)]
            period = 2.0 * (sum(intervals) / len(intervals))

    return {
        'rise_time': rise_time,
        'overshoot_pct': overshoot,
        'settling_time': settle,
        'ss_error': ss_err,
        'oscillation_period': period,
    }


def score(m):
    """Lower is better. Penalise overshoot, slow settle, steady-state error, and
    slow rise (so the optimiser can't win by going slow-and-lazy). A missing
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
    o = m.get('overshoot_pct')
    ss = m.get('ss_error')
    if o is None or ss is None:
        return False
    if not (math.isfinite(o) and math.isfinite(ss)):
        return True
    return o > 60.0              # >60% overshoot = abort before spin-out


def perturb(gains, gi, delta):
    """Return gains with element gi (0=kp,1=ki,2=kd) nudged by delta, clamped.
    i_limit (index 3) rides along unchanged."""
    kp, ki, kd, i_limit = gains
    vals = [kp, ki, kd]
    vals[gi] += delta
    kp, ki, kd = clamp_gains(vals[0], vals[1], vals[2])
    return (kp, ki, kd, i_limit)


# --------------------------------------------------------------------------- #
# Response sources
# --------------------------------------------------------------------------- #
def sim_response(gains, args):
    """Offline: a heading step on an angular plant inertia*θ'' + drag*θ' = torque
    using the real PID. Sign: error = wrap(θ - target) and the yaw command
    restores toward target, so the closed-loop torque is -correction (negative
    feedback). Magnitude scaling is absorbed into inertia/drag."""
    kp, ki, kd, i_limit = gains
    pid = PID(kp, ki, kd, limit=OUTPUT_LIMIT, i_limit=i_limit)
    inertia = max(1e-6, args.inertia)
    dt, max_yaw = CONTROL_DT, args.max_yaw

    theta, omega = 0.0, 0.0
    target = args.step                       # step from 0 -> args.step
    times, resp = [], []
    for i in range(int(round(args.seconds / dt))):
        err = wrap_angle(theta - target)
        correction = clamp(pid.update(err, dt), -max_yaw, max_yaw)
        omega += (-correction - args.drag * omega) * dt / inertia
        theta += omega * dt
        times.append(i * dt)
        resp.append(theta)
    return step_metrics(times, resp, y0=0.0, setpoint=target)


# --------------------------------------------------------------------------- #
# Live: apply gains + command a heading step against the real node
# --------------------------------------------------------------------------- #
def _ros2_param_set(node, name, value, timeout=8.0):
    """Set one ROS param via `ros2 param set` CLI. Returns (ok, detail)."""
    import subprocess
    try:
        proc = subprocess.run(
            ['ros2', 'param', 'set', '/%s' % node, name, repr(value)],
            capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        return False, 'ros2 CLI not found — source install/setup.bash'
    except subprocess.TimeoutExpired:
        return False, 'timed out (is /%s running?)' % node
    out = (proc.stdout + proc.stderr).strip()
    ok = proc.returncode == 0 and 'successful' in proc.stdout.lower()
    return ok, out


def _ros2_param_get(node, name, timeout=8.0):
    """Read one ROS param via `ros2 param get`. Returns float or None."""
    import re
    import subprocess
    try:
        proc = subprocess.run(
            ['ros2', 'param', 'get', '/%s' % node, name],
            capture_output=True, text=True, timeout=timeout)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0:
        return None
    m = re.search(r'(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*$', proc.stdout.strip())
    return float(m.group(1)) if m else None


def live_apply_gains(node, gains):
    """Push kp/ki/kd (+ i_limit) to /motion_node via `ros2 param set`."""
    kp, ki, kd, i_limit = gains
    ok = True
    for name, val in zip(GAIN_PARAMS + (I_LIMIT_PARAM,), (kp, ki, kd, i_limit)):
        good, detail = _ros2_param_set(node, name, val)
        if not good:
            ok = False
            print('  ! failed to set %s: %s' % (name, detail))
    return ok


class LiveHeadingTester:
    """One rclpy node: watches imu/rpy + submerge/state, commands heading steps
    on motion/heading, records the response. The sub must already be submerged
    and in HOLD (this does NOT command the dive). Steps are taken relative to a
    base heading captured on the first run and returned to after each, so the
    sub does not walk across the pool and every step has the same y0."""

    HOLD_STATE = 'hold'          # SubmergeState.HOLD.value

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32, String
        from geometry_msgs.msg import Vector3Stamped

        self._rclpy = rclpy
        self._Float32 = Float32
        self.node = Node('heading_autotune')
        self._yaw = None
        self._state = None
        self._base = None                    # captured base heading (rad)

        self.node.create_subscription(
            Vector3Stamped, 'imu/rpy', self._on_yaw, 10)
        self.node.create_subscription(
            String, 'submerge/state', self._on_state, 10)
        self._heading_pub = self.node.create_publisher(
            Float32, 'motion/heading', 10)

    def _on_yaw(self, msg):
        if math.isfinite(msg.vector.z):
            self._yaw = float(msg.vector.z)

    def _on_state(self, msg):
        self._state = msg.data

    def _spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._rclpy.spin_once(self.node, timeout_sec=0.02)

    def _command_heading(self, target):
        msg = self._Float32()
        msg.data = float(wrap_angle(target))
        self._heading_pub.publish(msg)

    def wait_ready(self, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._yaw is not None and self._state == self.HOLD_STATE:
                return None
        if self._yaw is None:
            return 'no yaw on imu/rpy — is the imu bridge + orientation node up?'
        return ('motion_node not in HOLD (state=%r) — submerge the sub first, '
                'e.g. via the mission/launch.' % self._state)

    def run_step(self, step, seconds, return_settle_s=4.0):
        """Command base+step, record the recovery, then return to base.

        Returns (times, yaws, y0, setpoint) or (None, reason)."""
        reason = self.wait_ready()
        if reason is not None:
            return None, reason
        if self._base is None:
            self._base = self._yaw           # first run captures the base
        y0 = self._base
        target = wrap_angle(self._base + step)

        self._command_heading(target)
        times, yaws = [], []
        t_start = time.monotonic()
        while time.monotonic() - t_start < seconds:
            self._rclpy.spin_once(self.node, timeout_sec=CONTROL_DT)
            if self._state != self.HOLD_STATE:
                self._command_heading(self._base)
                return None, 'left HOLD mid-step (state=%r) — aborted' % self._state
            if self._yaw is not None:
                times.append(time.monotonic() - t_start)
                yaws.append(self._yaw)

        # Return to base so the next step starts from the same place.
        self._command_heading(self._base)
        self._spin(return_settle_s)
        if len(yaws) < 5:
            return None, 'too few yaw samples recorded'
        return (times, yaws, y0, target), None

    def recenter(self):
        if self._base is not None:
            self._command_heading(self._base)
            self._spin(1.0)

    def close(self):
        try:
            self.recenter()
        except Exception:
            pass
        self.node.destroy_node()


def live_response(tester, args, iteration):
    """Run one live heading step, write a CSV, return metrics dict or None."""
    result, reason = tester.run_step(args.step, args.seconds)
    if result is None:
        print('  ! step failed — %s' % reason)
        return None
    times, yaws, y0, setpoint = result

    if args.outdir:
        os.makedirs(args.outdir, exist_ok=True)
        path = os.path.join(args.outdir, 'auto_yaw_%02d.csv' % iteration)
        try:
            with open(path, 'w') as f:
                f.write('time,setpoint,response,error\n')
                for t, y in zip(times, yaws):
                    f.write('%.4f,%.6f,%.6f,%.6f\n'
                            % (t, setpoint, y, wrap_angle(setpoint - y)))
            print('  recorded -> %s (%d samples)'
                  % (os.path.basename(path), len(times)))
        except OSError as e:
            print('  ! CSV write failed: %s' % e)

    return step_metrics(times, yaws, y0=y0, setpoint=setpoint)


# --------------------------------------------------------------------------- #
def fmt_metrics(m):
    return ('rise=%5s  overshoot=%5.1f%%  settle=%6s  ss_err=%+.4f  osc=%s'
            % (('%.2fs' % m['rise_time']) if m['rise_time'] is not None else 'n/a',
               m['overshoot_pct'] or 0.0,
               ('%.2fs' % m['settling_time']) if m['settling_time'] is not None
               else 'n/a',
               m['ss_error'] or 0.0,
               ('%.2fs' % m['oscillation_period']) if m['oscillation_period']
               else 'n/a'))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--sim', action='store_true',
                      help='offline plant model (default, safe)')
    mode.add_argument('--live', action='store_true',
                      help='real heading lock in water (slews the sub)')
    ap.add_argument('--node', default=MOTION_NODE)
    ap.add_argument('--iterations', type=int, default=12)
    ap.add_argument('--step', type=float, default=0.35,
                    help='heading step size, rad (~20deg)')
    ap.add_argument('--seconds', type=float, default=10.0,
                    help='response record window s')
    ap.add_argument('--overshoot', type=float, default=10.0,
                    help='target max overshoot %%')
    ap.add_argument('--settle', type=float, default=4.0,
                    help='target max settling time s')
    ap.add_argument('--ss-tol', type=float, default=0.03, dest='ss_tol',
                    help='target max |steady-state heading error|, rad')
    ap.add_argument('--inertia', type=float, default=1.0,
                    help='sim plant rotational inertia')
    ap.add_argument('--drag', type=float, default=0.3,
                    help='sim plant angular drag')
    ap.add_argument('--max-yaw', type=float, default=DEFAULT_MAX_YAW,
                    dest='max_yaw',
                    help='yaw-command clamp for the sim (motion_node '
                         'max_yaw_correction; read from the node in --live)')
    ap.add_argument('--outdir', default=os.path.join(_HERE, 'autotune_logs'))
    ap.add_argument('--yes', action='store_true',
                    help='skip the live-mode confirmation prompt')
    args = ap.parse_args()

    live = args.live and not args.sim
    targets = {'overshoot': args.overshoot, 'settle': args.settle,
               'ss': args.ss_tol}

    base = list(DEFAULT_GAINS)
    tester = None
    if live:
        cur = {name: _ros2_param_get(args.node, name)
               for name in GAIN_PARAMS + (I_LIMIT_PARAM,)}
        if all(cur[n] is not None for n in GAIN_PARAMS):
            base[0], base[1], base[2] = (cur['heading_kp'], cur['heading_ki'],
                                         cur['heading_kd'])
            if cur[I_LIMIT_PARAM] is not None:
                base[3] = cur[I_LIMIT_PARAM]
        else:
            print('WARNING: could not read current gains from /%s — is it '
                  'running? Falling back to motion_node defaults.' % args.node)
        mx = _ros2_param_get(args.node, 'max_yaw_correction')
        if mx is not None:
            args.max_yaw = mx
    gains = tuple(base)

    if live:
        os.makedirs(args.outdir, exist_ok=True)
        print('*** LIVE AUTO-TUNE — the sub WILL slew repeatedly (%d heading '
              'steps of %.2f rad). The sub must already be submerged and '
              'HOLDING. Props clear, kill switch in reach. ***'
              % (args.iterations, args.step))
        if not args.yes:
            if input('type "tune" to run: ').strip().lower() != 'tune':
                print('Aborted.')
                return 1
        try:
            import rclpy
        except ImportError as e:
            print('ERROR: ROS not available (%s). Use --sim for offline '
                  'tuning.' % e)
            return 1
        rclpy.init()
        tester = LiveHeadingTester()

    print('\nauto-tune HEADING PID  mode=%s  budget=%d evals  target: '
          'overshoot<=%.0f%% settle<=%.1fs |ss|<=%.3f'
          % ('LIVE' if live else 'sim', args.iterations, args.overshoot,
             args.settle, args.ss_tol))
    print('start gains: kp=%.3f ki=%.3f kd=%.3f  (i_limit=%.3f, yaw clamp ±%.2f)'
          % (gains[0], gains[1], gains[2], gains[3], args.max_yaw))
    print('-' * 78)

    start_gains = gains
    aborted = [False]
    evals = [0]

    def evaluate(g):
        if aborted[0]:
            return None
        if live:
            if not live_apply_gains(args.node, g):
                print('  ! could not apply gains — aborting.')
                aborted[0] = True
                return None
            time.sleep(1.0)                  # let the node settle on new gains
            m = live_response(tester, args, evals[0])
            if m is None:
                aborted[0] = True
                return None
        else:
            m = sim_response(g, args)
        evals[0] += 1
        print('[%02d] kp=%.3f ki=%.3f kd=%.3f | %s | score=%.1f'
              % (evals[0], g[0], g[1], g[2], fmt_metrics(m), score(m)))
        if live and is_runaway(m):
            print('  ! RUNAWAY — restoring start gains and aborting.')
            live_apply_gains(args.node, start_gains)
            aborted[0] = True
            return None
        return m

    try:
        with Auv() as auv:
            auv.submerge_to_depth(target_depth=1)   # blocks until 'hold'
        # Pattern search (Hooke-Jeeves): evaluate the current best, probe ± a
        # step on each gain, move to the first probe that lowers the score. When
        # no probe improves, shrink every step. Only accepted (better) points
        # become the base, so the score is monotone non-increasing.
        m0 = evaluate(gains)
        if m0 is None:
            print('No successful first evaluation. Nothing tuned.')
            return 1
        best = (score(m0), gains, m0)
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
                    s = score(m)
                    if s < best[0] - 1e-9:
                        best = (s, cand, m)
                        improved = True
                        break                   # greedy: first improvement
                if improved or aborted[0]:
                    break
            if not improved and not aborted[0]:
                steps = {k: v * 0.5 for k, v in steps.items()}
                if all(steps[k] < SEARCH_STEP_MIN[k] for k in steps):
                    print('  step sizes below floor — search converged.')
                    break

        print('-' * 78)
        bg = best[1]
        print('BEST: kp=%.3f ki=%.3f kd=%.3f  (score=%.1f)'
              % (bg[0], bg[1], bg[2], best[0]))
        print('      %ds' % fmt_metrics(best[2]))
        if live:
            print('Applying best gains to /%s now.' % args.node)
            live_apply_gains(args.node, bg)
            print('NOTE: `ros2 param set` is NOT persistent. Bake the winner into '
                  'motion_node.PARAM_DEFAULTS (heading_kp/ki/kd) in '
                  'src/control/control/motion_node.py, then colcon build '
                  '--symlink-install.')
        else:
            print('Sim only — nothing applied (BALLPARK gains). Verify in the '
                  'pool with:')
            print('  python3 auto_tune_pid.py --live --yes   (or, by hand)')
            print('  ros2 param set /%s heading_kp %.3f' % (args.node, bg[0]))
            print('  ros2 param set /%s heading_ki %.3f' % (args.node, bg[1]))
            print('  ros2 param set /%s heading_kd %.3f' % (args.node, bg[2]))
        return 0
    finally:
        if tester is not None:
            tester.close()
            auv.submerge_to_depth(target_depth=0)
            auv.stop()
            try:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    sys.exit(main())
