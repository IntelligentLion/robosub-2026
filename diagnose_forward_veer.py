#!/usr/bin/env python3
"""Diagnose "forward surge veers hard RIGHT" — evidence gathering, no fixes.

Observed symptom (2026-07-13): submerge/emerge is straight (verticals 5-8
balanced), but any forward command turns the sub drastically right. Thrusters
audibly run at different RPMs / pull different voltages at the same command.

Two credible root-cause families, plus one amplifier:

  A. HARDWARE imbalance — one or more horizontal thrusters (1-4) produce less
     thrust at the same PWM: damaged/fouled prop, worn bearing, ESC that never
     got a bidirectional throttle calibration, corroded connector.
  B. CONFIG — the FC never commanded symmetric PWM in the first place: drifted
     MOT_x_DIRECTION / SERVOx_* params, wrong mode, mixer problem.
  Amplifier: MANUAL mode (custom_mode 19) has NO yaw feedback, so any
     asymmetry from A or B integrates into a turn unopposed. STABILIZE (0)
     closes a heading loop and masks small asymmetry.

Per systematic debugging, this script only GATHERS EVIDENCE that separates
A from B; it never writes a parameter (read-only param policy, see
depth_hold_bar02_test.read_param). Phases:

  1. PARAM AUDIT (always, dry-safe, no arming): all 8 motors' MOT/SERVO
     params vs pixhawk_params_4.5.7_backup_2026-07-08.param. Mismatch =
     evidence for family B, and the wet phase refuses to run.
  2. CURRENT SWEEP (--sweep; ARMS, PROPS SPIN): spins each motor alone at the
     IDENTICAL PWM via DO_MOTOR_TEST while sampling battery current/voltage.
     Healthy identical thrusters draw matching current at the same PWM; the
     outlier is the bad one (family A). Verticals 5-8 are measured too as a
     control group — the symptom says they're balanced, so their spread is
     this method's noise floor. Run IN WATER (props loaded) for meaningful
     numbers; dry spins draw so little that only gross faults show.
  3. WET SURGE (--wet; sub in water at surface): MANUAL forward surge while
     logging ATTITUDE yaw/yawspeed and per-horizontal PWM offsets.
       - FC PWM offsets symmetric + sub still turns  -> hardware (A)
       - FC PWM offsets asymmetric                   -> config (B)
     --stabilize repeats the surge in STABILIZE: if the veer disappears, the
     heading loop can compensate — a usable band-aid while hardware is fixed.

The Pixhawk is mounted facing BACKWARD: vehicle-forward is autopilot -x, so
the surge sends negative x (same convention as submerge_forward_10ft.py).
Yaw sign is unaffected by that mounting: positive ATTITUDE.yawspeed is
clockwise seen from above = a RIGHT turn.

Usage:
    python3 diagnose_forward_veer.py                      # param audit only
    python3 diagnose_forward_veer.py --sweep              # + current sweep
    python3 diagnose_forward_veer.py --sweep --both       # fwd AND reverse
    python3 diagnose_forward_veer.py --wet --stabilize    # + in-water surges
    python3 diagnose_forward_veer.py --sweep --wet --stabilize   # everything

Safety:
  * stop thruster_node / anything else on the Pixhawk serial first
    (single-serial-reader rule)
  * --sweep arms and spins props: props clear, or sub in water
  * --wet drives the sub: in water, surface, tether clear, kill switch in reach
  * every armed phase ends in neutral + disarm, also on Ctrl-C
"""

import argparse
import statistics
import sys
import time

from pymavlink import mavutil

import depth_hold_bar02_test as dh   # import applies the pymavlink
                                     # add_message monkeypatch too
import motor_test as mt

MANUAL_MODE = 19                 # ArduSub custom_mode: no EKF/attitude loops
STABILIZE_MODE = 0               # roll/pitch level + heading hold on r=0
RATE_HZ = 10
SYS_STATUS_ID = 1
BATTERY_STATUS_ID = 147
ALL_MOTORS = dh.HORIZONTAL_MOTORS + dh.VERTICAL_MOTORS

SPIN_UP_SKIP_S = 1.0             # discard current samples while prop spins up
FLAG_PCT = 15.0                  # |deviation from group median| that flags
MIN_GROUP_MEDIAN_A = 0.05        # below this net draw the ratios are noise


# ---------------------------------------------------------------- analysis --
# Pure functions, unit-tested in tests/test_diagnose_veer.py.

def steady_mean(samples, skip_s=SPIN_UP_SKIP_S):
    """Mean (amps, volts, n) of samples after the spin-up transient.
    samples: list of (t_rel_s, amps, volts_or_None). (None, None, 0) if no
    sample survives the skip window."""
    use = [s for s in samples if s[0] >= skip_s]
    if not use:
        return None, None, 0
    amps = statistics.fmean(s[1] for s in use)
    volts_vals = [s[2] for s in use if s[2] is not None]
    volts = statistics.fmean(volts_vals) if volts_vals else None
    return amps, volts, len(use)


def analyze_group(net_amps, flag_pct=FLAG_PCT,
                  min_median=MIN_GROUP_MEDIAN_A):
    """Compare per-motor net current draw within one group.

    net_amps: {motor: net_amps_above_baseline}. Returns (median, rows) where
    rows = {motor: (net, dev_pct_or_None, verdict)} and verdict is 'WEAK',
    'STRONG' or ''. If the group median is below min_median the load is too
    light for ratios to mean anything (dry props) — every dev is None."""
    vals = [v for v in net_amps.values() if v is not None]
    if not vals:
        return None, {m: (None, None, '') for m in net_amps}
    med = statistics.median(vals)
    rows = {}
    for m, net in sorted(net_amps.items()):
        if net is None or med < min_median:
            rows[m] = (net, None, '')
            continue
        dev = (net - med) / med * 100.0
        verdict = ('WEAK' if dev < -flag_pct
                   else 'STRONG' if dev > flag_pct else '')
        rows[m] = (net, dev, verdict)
    return med, rows


def wrap_delta(prev_rad, cur_rad):
    """Smallest signed angle step prev->cur, radians in (-pi, pi]. Summing
    these unwraps a +/-pi-wrapped yaw series into a continuous total."""
    d = cur_rad - prev_rad
    while d > 3.14159265358979:
        d -= 2.0 * 3.14159265358979
    while d <= -3.14159265358979:
        d += 2.0 * 3.14159265358979
    return d


def pwm_symmetry(offsets):
    """offsets: {motor: mean_pwm_offset_from_trim}. Returns (spread_pct,
    symmetric) — spread of |offset| magnitudes relative to their mean.
    Symmetric (<10%) means the FC commanded near-equal authority on every
    horizontal, so a veer must come from the hardware side."""
    mags = [abs(v) for v in offsets.values()]
    if not mags or statistics.fmean(mags) < 1.0:
        return None, False
    mean = statistics.fmean(mags)
    spread = (max(mags) - min(mags)) / mean * 100.0
    return spread, spread < 10.0


# --------------------------------------------------------------- telemetry --

def request_current_stream(master, hz=RATE_HZ):
    """Ask for SYS_STATUS + BATTERY_STATUS at hz (battery amps/volts)."""
    interval_us = int(1e6 / hz)
    for msg_id in (SYS_STATUS_ID, BATTERY_STATUS_ID):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, interval_us, 0, 0, 0, 0, 0)


def _battery_sample(msg):
    """(amps, volts) out of a SYS_STATUS or BATTERY_STATUS msg, None fields
    when the FC reports 'unknown' (-1 cA / 65535 mV)."""
    amps = None
    volts = None
    if msg.get_type() == 'SYS_STATUS':
        if msg.current_battery != -1:
            amps = msg.current_battery / 100.0
        if msg.voltage_battery != 65535:
            volts = msg.voltage_battery / 1000.0
    else:                                        # BATTERY_STATUS
        if msg.current_battery != -1:
            amps = msg.current_battery / 100.0
        if msg.voltages and msg.voltages[0] != 65535:
            volts = msg.voltages[0] / 1000.0
    return amps, volts


def sample_window(master, duration, motor=None, throttle_pct=50):
    """Collect battery samples for `duration` s. With motor set, stream
    DO_MOTOR_TEST at 5 Hz the whole window so that ONE motor spins at
    throttle_pct (ArduSub dead-man: the test stops 500 ms after the last
    frame, and a lapsed session costs a 10 s cooldown — see motor_test.py).
    Returns (samples, ack_result): samples = [(t_rel, amps, volts), ...],
    ack_result = first DO_MOTOR_TEST COMMAND_ACK result or None."""
    samples = []
    ack = None
    start = time.monotonic()
    end = start + duration
    next_cmd = start
    while time.monotonic() < end:
        now = time.monotonic()
        if motor is not None and now >= next_cmd:
            mt._send_motor_test(master, motor, throttle_pct)
            next_cmd = now + mt.STREAM_PERIOD_S
        msg = master.recv_match(
            type=['SYS_STATUS', 'BATTERY_STATUS', 'COMMAND_ACK'],
            blocking=True, timeout=0.05)
        if msg is None:
            continue
        if msg.get_type() == 'COMMAND_ACK':
            if (ack is None
                    and msg.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST):
                ack = msg.result
            continue
        amps, volts = _battery_sample(msg)
        if amps is not None:
            samples.append((time.monotonic() - start, amps, volts))
    return samples, ack


# ------------------------------------------------------------ current sweep --

def current_sweep(master, motors, throttle_pct, duration, both):
    """Spin each motor alone at the identical PWM, record steady-state net
    current above idle baseline. Returns {(motor, pct): net_amps_or_None}
    plus prints as it goes. Caller has already armed."""
    pwm_us = mt.pct_to_pwm(throttle_pct)
    print(f'\nIdle baseline (no motors) …', flush=True)
    base_samples, _ = sample_window(master, 3.0)
    base_amps, base_volts, n = steady_mean(base_samples, skip_s=0.5)
    if base_amps is None:
        print('No battery current telemetry (SYS_STATUS/BATTERY_STATUS '
              'current unknown) — cannot run the sweep. Is a power module '
              'wired + BATT_MONITOR set?')
        return None, None
    print(f'  baseline {base_amps:.2f} A  '
          f'{base_volts:.2f} V  ({n} samples)' if base_volts is not None
          else f'  baseline {base_amps:.2f} A  ({n} samples)')

    steps = [throttle_pct] + ([100 - throttle_pct] if both else [])
    results = {}
    for m in motors:
        for pct in steps:
            arrow = 'FWD' if pct > 50 else 'REV'
            print(f'  motor {m}  {pct:3d}% ({arrow}, {mt.pct_to_pwm(pct)} µs) '
                  f'{duration:.0f}s … ', end='', flush=True)
            samples, ack = sample_window(master, duration, m, pct)
            # keep the dead-man session alive between steps at neutral
            sample_window(master, 0.7, m, 50)
            amps, volts, n = steady_mean(samples)
            if ack not in (None, 0):
                print(f'DO_MOTOR_TEST REJECTED result={ack} — skipping')
                results[(m, pct)] = None
                continue
            if amps is None:
                print('no current samples')
                results[(m, pct)] = None
                continue
            net = amps - base_amps
            results[(m, pct)] = net
            vtxt = f' {volts:.2f} V' if volts is not None else ''
            print(f'{amps:.2f} A (net {net:+.2f} A){vtxt}  [{n} samples]')

    print('Post-sweep baseline …')
    base2_samples, _ = sample_window(master, 3.0)
    base2, _v, _n = steady_mean(base2_samples, skip_s=0.5)
    if base2 is not None and abs(base2 - base_amps) > 0.2:
        print(f'  WARNING: idle baseline drifted {base_amps:.2f} -> '
              f'{base2:.2f} A during the sweep — treat small deltas '
              'with suspicion, rerun if marginal.')
    return results, base_amps


def report_sweep(results, throttle_pct, both):
    """Group results into horizontals/verticals and flag outliers."""
    steps = [throttle_pct] + ([100 - throttle_pct] if both else [])
    weak = set()
    for group, label in ((dh.HORIZONTAL_MOTORS, 'HORIZONTAL (1-4) — the '
                          'suspects for a forward veer'),
                         (dh.VERTICAL_MOTORS, 'VERTICAL (5-8) — control '
                          'group, symptom says these are balanced')):
        print(f'\n{label}')
        for pct in steps:
            net = {m: results.get((m, pct)) for m in group}
            med, rows = analyze_group(net)
            arrow = 'FWD' if pct > 50 else 'REV'
            print(f'  at {pct}% ({arrow}), group median '
                  + (f'{med:.2f} A:' if med is not None else 'n/a:'))
            if med is not None and med < MIN_GROUP_MEDIAN_A:
                print('    load too light for ratios (dry props?) — '
                      'values shown, no flags. Rerun in water.')
            for m, (netv, dev, verdict) in rows.items():
                nettxt = f'{netv:+.2f} A' if netv is not None else '   n/a'
                devtxt = f'{dev:+6.1f}%' if dev is not None else '      -'
                flag = f'  <-- {verdict}' if verdict else ''
                print(f'    motor {m}: {nettxt}  {devtxt}{flag}')
                if verdict == 'WEAK' and m in dh.HORIZONTAL_MOTORS:
                    weak.add(m)
    return weak


# ---------------------------------------------------------------- wet surge --

def set_mode(master, mode_num, name):
    """Command a custom mode and verify via heartbeat (ACK alone proves
    nothing — same pattern as depth_hold_bar02_test.set_alt_hold)."""
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num)
    ack = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3)
    print(f'{name} ACK: result={ack.result}' if ack
          else f'No ACK for set_mode {name} — verifying via heartbeat…')
    hb = None
    deadline = time.time() + 5.0
    while time.time() < deadline:
        hb = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
        if hb is not None and hb.custom_mode == mode_num:
            print(f'Mode verified: {name} active.')
            return True
    print(f'MODE VERIFY FAILED: not in {name} (last custom_mode='
          f'{hb.custom_mode if hb else "none received"}).')
    return False


def surge_phase(master, label, effort, secs, ramp_s=1.0):
    """Drive vehicle-forward (autopilot -x, Pixhawk mounted backward) at
    `effort` for `secs` while logging yaw + horizontal PWM. Caller set the
    mode and armed. Ends at neutral. Returns evidence dict or None if no
    ATTITUDE ever arrived."""
    x_full = -int(effort * 1000)
    pwm_mon = dh.PwmMonitor()
    period = 1.0 / RATE_HZ
    yaw_prev = None
    yaw_total = 0.0
    yawspeeds = []
    offsets = {m: [] for m in dh.HORIZONTAL_MOTORS}
    start = time.monotonic()
    end = start + ramp_s + secs
    last_print = 0.0
    while time.monotonic() < end:
        t0 = time.monotonic()
        ramp = min(1.0, (t0 - start) / ramp_s) if ramp_s > 0 else 1.0
        x = int(x_full * ramp)
        dh.send_frame(master, dh.NEUTRAL_Z, x=x)
        while True:
            msg = master.recv_match(type=['ATTITUDE', 'SERVO_OUTPUT_RAW'],
                                    blocking=False)
            if msg is None:
                break
            if msg.get_type() == 'ATTITUDE':
                if yaw_prev is not None and ramp >= 1.0:
                    yaw_total += wrap_delta(yaw_prev, msg.yaw)
                    yawspeeds.append(msg.yawspeed)
                yaw_prev = msg.yaw
            else:
                pwm = (msg.servo1_raw, msg.servo2_raw, msg.servo3_raw,
                       msg.servo4_raw, msg.servo5_raw, msg.servo6_raw,
                       msg.servo7_raw, msg.servo8_raw)
                pwm_mon.update(pwm, dh.NEUTRAL_Z, x_cmd=x)
                if ramp >= 1.0:
                    for m in dh.HORIZONTAL_MOTORS:
                        if pwm[m - 1] != 0:
                            offsets[m].append(
                                pwm[m - 1] - dh.EXPECT_SERVO_TRIM)
        if t0 - last_print >= 1.0:
            last_print = t0
            rate = (statistics.fmean(yawspeeds[-RATE_HZ:]) * 57.2958
                    if yawspeeds else float('nan'))
            print(f'  [{label}] t-{end - t0:4.1f}s x={x} '
                  f'yawrate~{rate:+.1f}°/s {pwm_mon.fmt()}')
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    for _ in range(RATE_HZ):                       # neutral, keep link alive
        dh.send_frame(master, dh.NEUTRAL_Z)
        time.sleep(period)
    if yaw_prev is None:
        print(f'  [{label}] no ATTITUDE messages — cannot measure yaw.')
        return None
    mean_rate = statistics.fmean(yawspeeds) * 57.2958 if yawspeeds else 0.0
    mean_off = {m: (statistics.fmean(v) if v else 0.0)
                for m, v in offsets.items()}
    return {'label': label, 'yaw_deg': yaw_total * 57.2958,
            'rate_dps': mean_rate, 'offsets': mean_off, 'secs': secs}


def report_surge(ev):
    turn = ('RIGHT (clockwise)' if ev['rate_dps'] > 0.5
            else 'LEFT (counter-clockwise)' if ev['rate_dps'] < -0.5
            else 'straight')
    print(f"\n  [{ev['label']}] yaw change {ev['yaw_deg']:+.1f}° over "
          f"{ev['secs']:.0f}s  (mean {ev['rate_dps']:+.2f}°/s) -> {turn}")
    spread, symmetric = pwm_symmetry(ev['offsets'])
    for m, off in sorted(ev['offsets'].items()):
        exp = -1000 * dh.FWD_FACTOR[m] * dh.EXPECT_MOT_DIRECTION[m]
        agree = '' if off == 0 else (
            ' (sign OK)' if off * exp > 0 else ' (sign WRONG)')
        print(f'    motor {m}: mean PWM offset {off:+6.1f} µs{agree}')
    if spread is None:
        print('    horizontals never left trim — mixer not responding?')
    elif symmetric:
        print(f'    FC output SYMMETRIC (spread {spread:.1f}%) — the FC '
              'commanded equal authority; a veer here is HARDWARE (family A).')
    else:
        print(f'    FC output ASYMMETRIC (spread {spread:.1f}%) — the FC '
              'itself commanded unequal PWM; look at CONFIG (family B).')


# --------------------------------------------------------------------- main --

def confirm(prompt, assume_yes):
    if assume_yes:
        return True
    return input(f'{prompt} [yes/NO] ').strip().lower() == 'yes'


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=dh.DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=dh.DEFAULT_BAUD)
    ap.add_argument('--sweep', action='store_true',
                    help='per-motor current sweep (ARMS, props spin)')
    ap.add_argument('--throttle', type=float, default=60.0,
                    help='sweep throttle PERCENT, 50=stop (default 60)')
    ap.add_argument('--duration', type=float, default=4.0,
                    help='seconds per motor spin (default 4)')
    ap.add_argument('--both', action='store_true',
                    help='sweep forward AND reverse mirror (100-throttle)')
    ap.add_argument('--wet', action='store_true',
                    help='in-water MANUAL surge, measures the actual veer')
    ap.add_argument('--stabilize', action='store_true',
                    help='with --wet: repeat the surge in STABILIZE')
    ap.add_argument('--effort', type=float, default=0.5,
                    help='surge thrust fraction 0..1 (default 0.5)')
    ap.add_argument('--surge-secs', type=float, default=6.0,
                    help='seconds per surge after ramp (default 6)')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompts')
    args = ap.parse_args()

    if not 50.0 < args.throttle <= 100.0:
        ap.error('--throttle must be in (50, 100]')
    if not 0.0 < args.effort <= 1.0:
        ap.error('--effort must be in (0, 1]')

    log_path = dh.tee_output_to_log()
    print(f'Logging to {log_path}')
    master = dh.connect(args.port, args.baud)
    dh.request_streams(master, RATE_HZ)
    request_current_stream(master)

    # ---- Phase 1: param audit (dry-safe, read-only, always) ----
    print('\n=== Phase 1: param audit vs known-good backup ===')
    audit_ok = dh.verify_thruster_params(master, ALL_MOTORS, 'all-8')
    if not audit_ok:
        print('\nEVIDENCE: params drifted from backup — that alone can turn '
              'a forward command into a spin (family B). Fix params first; '
              'the --wet phase is disabled this run. The --sweep phase is '
              'still allowed: DO_MOTOR_TEST addresses outputs directly, so '
              'its motor-to-motor comparison stays valid.')

    weak = set()
    try:
        # ---- Phase 2: per-motor current sweep ----
        if args.sweep:
            print(f'\n=== Phase 2: current sweep — each motor alone at '
                  f'{args.throttle:.0f}% ({mt.pct_to_pwm(args.throttle)} µs) '
                  f'===')
            print('THRUSTERS WILL SPIN. In water = meaningful numbers; '
                  'dry = gross faults only.')
            if not confirm('Props clear / sub in water, kill switch in '
                           'reach?', args.yes):
                print('Sweep skipped.')
            elif not dh.arm(master, True):
                print('Arm failed — sweep skipped.')
            else:
                try:
                    time.sleep(1.0)
                    results, _base = current_sweep(
                        master, ALL_MOTORS, int(round(args.throttle)),
                        args.duration, args.both)
                finally:
                    dh.arm(master, False)
                if results is not None:
                    weak = report_sweep(results, int(round(args.throttle)),
                                        args.both)

        # ---- Phase 3: wet surge ----
        if args.wet:
            print('\n=== Phase 3: in-water forward surge ===')
            if not audit_ok:
                print('REFUSED: param audit failed — driving with drifted '
                      'thruster params is exactly the failure this preflight '
                      'exists to stop. Fix params, rerun.')
            elif not confirm('Sub IN WATER at surface, tether clear, kill '
                             'switch in reach?', args.yes):
                print('Wet phase skipped.')
            else:
                surges = []
                try:
                    if set_mode(master, MANUAL_MODE, 'MANUAL') and \
                            dh.arm(master, True):
                        print(f'\nMANUAL surge: effort {args.effort:.2f}, '
                              f'{args.surge_secs:.0f}s, NO yaw feedback — '
                              'this measures the raw veer.')
                        ev = surge_phase(master, 'MANUAL', args.effort,
                                         args.surge_secs)
                        if ev:
                            surges.append(ev)
                        if args.stabilize:
                            if set_mode(master, STABILIZE_MODE, 'STABILIZE'):
                                print(f'\nSTABILIZE surge: same command, '
                                      'heading loop active.')
                                ev = surge_phase(master, 'STABILIZE',
                                                 args.effort, args.surge_secs)
                                if ev:
                                    surges.append(ev)
                finally:
                    print('\nNeutral + disarm…')
                    for _ in range(RATE_HZ):
                        dh.send_frame(master, dh.NEUTRAL_Z)
                        time.sleep(1.0 / RATE_HZ)
                    dh.arm(master, False)
                for ev in surges:
                    report_surge(ev)
                if len(surges) == 2:
                    m, s = surges[0], surges[1]
                    if abs(m['rate_dps']) > 2.0 and \
                            abs(s['rate_dps']) < abs(m['rate_dps']) / 3:
                        print('\n  STABILIZE kills the veer: the heading '
                              'loop can compensate. Usable band-aid while '
                              'the weak thruster is fixed — see the '
                              'stabilize/depth-hold migration plan.')
    except KeyboardInterrupt:
        print('\nInterrupted — neutral + disarm…')
        for _ in range(RATE_HZ):
            dh.send_frame(master, dh.NEUTRAL_Z)
            time.sleep(1.0 / RATE_HZ)
        dh.arm(master, False)
    finally:
        master.close()

    # ---- verdict ----
    print('\n=== What the evidence says ===')
    if not audit_ok:
        print('* Params drifted (family B). Restore them against '
              'pixhawk_params_4.5.7_backup_2026-07-08.param, rerun.')
    if weak:
        for m in sorted(weak):
            print(f'* Motor {m} draws significantly less current at the same '
                  'PWM — weak thruster (family A). Hardware checklist: prop '
                  'debris/damage, bearing, connector corrosion, ESC '
                  'bidirectional throttle calibration (weak one way only = '
                  'classic uncalibrated reverse half).')
        w = min(sorted(weak))
        print(f'  Software band-aid until fixed: '
              f'python3 motor_trim.py --weak {w} --factor 0.85   '
              '(derates the OTHER motors; re-run this sweep after to '
              'verify, and note the trim narrows SERVOn_MIN/MAX so the '
              'preflight gate expectations must be updated too — see the '
              'thruster-equalization plan).')
    if audit_ok and not weak and args.sweep:
        print('* No current outlier at matched PWM and params clean. If the '
              'wet surge still veers: look at asymmetric drag (tether pull, '
              'ballast/trim, a bent prop that loads normally), and run '
              '--wet --stabilize to see if closing the heading loop is '
              'enough.')
    if not args.sweep and not args.wet:
        print('* Param audit only. Next: --sweep (in water) for the '
              'per-motor current comparison, then --wet --stabilize.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
