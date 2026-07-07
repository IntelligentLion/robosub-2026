#!/usr/bin/env python3
"""PID tuning tool for the RoboSub autonomous controller.

Subcommands:
  sim   - Offline simulator (no ROS needed).
  step  - Pool step-response recorder (needs ROS).
  set   - Live gain applier (needs ROS).
  list  - Dump current PID params (needs ROS).
"""

import argparse
import math
import sys
import time
from typing import Any

# --------------------------------------------------------------------------- #
# Default gains — mirrors the controller's live parameter defaults.
# Each entry: (kp, ki, kd, limit, i_limit)
# --------------------------------------------------------------------------- #
DEFAULT_GAINS = {
    "yaw":       (1.2, 0.08, 0.30, 1.0, 2.0),   # pid_yaw
    "surge":     (0.8, 0.05, 0.20, 1.0, 2.0),   # pid_x
    "strafe":    (0.8, 0.05, 0.20, 1.0, 2.0),   # pid_y
    "heave":     (1.0, 0.10, 0.15, 1.0, 2.0),   # pid_z
    "vis_yaw":   (1.5, 0.00, 0.30, 1.0, 2.0),   # pid_vis_yaw
    "vis_strafe":(0.8, 0.00, 0.20, 1.0, 2.0),   # pid_vis_strafe
    "vis_depth": (1.0, 0.00, 0.20, 1.0, 2.0),   # pid_vis_depth
}

# axis -> ROS param prefix (for `set` / `list`)
AXIS_PARAM = {
    "yaw":        "pid_yaw",
    "surge":      "pid_x",
    "strafe":     "pid_y",
    "heave":      "pid_z",
    "vis_yaw":    "pid_vis_yaw",
    "vis_strafe": "pid_vis_strafe",
    "vis_depth":  "pid_vis_depth",
}

CONTROLLER_DT = 0.1  # 10 Hz
DERIV_CLAMP = 10.0   # matches controller


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class PID:
    """Inline PID mirroring the controller exactly.

    output = kp*err + ki*integral(err*dt) + kd*derivative
    integral clamped ±i_limit, derivative clamped ±DERIV_CLAMP,
    output clamped ±limit. ki is a rate gain (matches Z-N Ki = 1.2Ku/Tu).
    """

    def __init__(self, kp, ki, kd, limit, i_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.i_limit = i_limit
        self._integral = 0.0
        self._prev_err = None

    def reset(self):
        self._integral = 0.0
        self._prev_err = None

    def update(self, err, dt):
        # Integral (rate-gain form): ki already absorbs 1/T scaling.
        self._integral += err * dt
        if self._integral > self.i_limit:
            self._integral = self.i_limit
        elif self._integral < -self.i_limit:
            self._integral = -self.i_limit

        # Derivative
        if self._prev_err is None:
            deriv = 0.0
        else:
            deriv = (err - self._prev_err) / dt
        if deriv > DERIV_CLAMP:
            deriv = DERIV_CLAMP
        elif deriv < -DERIV_CLAMP:
            deriv = -DERIV_CLAMP
        self._prev_err = err

        out = self.kp * err + self.ki * self._integral + self.kd * deriv
        if out > self.limit:
            out = self.limit
        elif out < -self.limit:
            out = -self.limit
        return out


def ziegler_nichols(ku, tu):
    """Return Ziegler-Nichols P/PI/PID gain sets from ultimate gain & period.

    ki is a rate gain (1.2Ku/Tu) to match the controller.
    """
    if ku <= 0 or tu <= 0:
        return {}
    return {
        "P":   {"kp": 0.5 * ku, "ki": 0.0,                "kd": 0.0},
        "PI":  {"kp": 0.45 * ku, "ki": 0.54 * ku / tu,     "kd": 0.0},
        "PID": {"kp": 0.6 * ku,  "ki": 1.2 * ku / tu,      "kd": 0.075 * ku * tu},
    }


def simulate(gains, mass, drag, setpoint, dt, seconds, angular=False):
    """Simulate second-order plant m*x'' + b*x' = u with semi-implicit Euler.

    Returns (times, responses) lists.
    """
    kp, ki, kd, limit, i_limit = gains
    pid = PID(kp, ki, kd, limit, i_limit)

    n = int(round(seconds / dt))
    pos = 0.0
    vel = 0.0
    times = []
    responses = []
    for i in range(n):
        t = i * dt
        err = wrap_angle(setpoint - pos) if angular else (setpoint - pos)
        u = pid.update(err, dt)
        # Semi-implicit Euler
        vel += (u - drag * vel) * dt / mass
        pos += vel * dt
        times.append(t)
        responses.append(pos if not angular else wrap_angle(pos))
    return times, responses


def compute_metrics(times, response, setpoint):
    """Compute step-response metrics. Returns dict."""
    resp = response
    n = len(resp)
    if n == 0:
        return {}

    span = abs(setpoint) if abs(setpoint) > 1e-9 else 1.0
    thr10 = 0.10 * setpoint
    thr90 = 0.90 * setpoint

    # Rise time 10% -> 90%
    t10 = t90 = None
    for i in range(n):
        if t10 is None and abs(resp[i]) >= abs(thr10):
            t10 = times[i]
        if t90 is None and abs(resp[i]) >= abs(thr90):
            t90 = times[i]
            break
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # Overshoot %
    if setpoint >= 0:
        peak = max(resp)
        overshoot = max(0.0, (peak - setpoint) / span * 100.0) if span > 0 else 0.0
    else:
        peak = min(resp)
        overshoot = max(0.0, (setpoint - peak) / span * 100.0) if span > 0 else 0.0

    # Settling time (±5%): last time response leaves the band
    band = 0.05 * span
    settle = times[0]
    for i in range(n - 1, -1, -1):
        if abs(resp[i] - setpoint) > band:
            settle = times[min(i + 1, n - 1)]
            break

    # Steady-state error (last 10% average)
    tail = resp[max(0, n - max(1, n // 10)):]
    ss_val = sum(tail) / len(tail) if tail else resp[-1]
    ss_err = ss_val - setpoint

    # Oscillation period from zero crossings of error after first rise
    period = None
    if t10 is not None:
        errs = [setpoint - r for r in resp]
        crossings = []
        prev_sign = None
        for i in range(n):
            if times[i] < t10:
                continue
            if abs(errs[i]) < 1e-9:
                continue
            s = 1 if errs[i] > 0 else -1
            if prev_sign is not None and s != prev_sign:
                crossings.append(times[i])
            prev_sign = s
        if len(crossings) >= 2:
            intervals = [crossings[i + 1] - crossings[i] for i in range(len(crossings) - 1)]
            # half-periods -> full period = 2 * mean
            period = 2.0 * (sum(intervals) / len(intervals))

    return {
        "rise_time": rise_time,
        "overshoot_pct": overshoot,
        "settling_time": settle,
        "ss_error": ss_err,
        "oscillation_period": period,
    }


# --------------------------------------------------------------------------- #
# Command: sim
# --------------------------------------------------------------------------- #
def cmd_sim(args):
    base = DEFAULT_GAINS[args.axis]
    mass = args.mass
    drag = args.drag
    setpoint = args.setpoint
    dt = CONTROLLER_DT
    seconds = args.seconds
    angular = (args.axis == "yaw")

    runs = []  # list of (label, gains, times, resp)

    if args.compare:
        for idx, spec in enumerate(args.compare.split(";")):
            spec = spec.strip()
            if not spec:
                continue
            overrides = {}
            try:
                for token in spec.split(","):
                    k, v = token.split("=")
                    overrides[k.strip()] = float(v.strip())
            except (ValueError, TypeError) as e:
                print("ERROR parsing --compare token %r: %s" % (spec, e))
                return 1
            g = list(base)
            for i, name in enumerate(["kp", "ki", "kd", "limit", "i_limit"]):
                if name in overrides:
                    g[i] = overrides[name]
            label = "kp={kp},ki={ki},kd={kd}".format(
                kp=g[0], ki=g[1], kd=g[2])
            runs.append((label, tuple(g)))
    else:
        g = list(base)
        for i, name in enumerate(["kp", "ki", "kd", "limit", "i_limit"]):
            val = getattr(args, name)
            if val is not None:
                g[i] = val
        runs.append(("default", tuple(g)))

    print("sim: axis=%s mass=%.2f drag=%.2f setpoint=%.2f dt=%.2f s"
          % (args.axis, mass, drag, setpoint, dt))
    print("-" * 70)

    all_results = []
    for label, gains in runs:
        times, resp = simulate(gains, mass, drag, setpoint, dt, seconds, angular)
        m = compute_metrics(times, resp, setpoint)
        all_results.append((label, gains, times, resp, m))
        print("[%s] kp=%.3f ki=%.3f kd=%.3f" % (label, gains[0], gains[1], gains[2]))
        print("  rise_time=%.2fs  overshoot=%.1f%%  settle=%.2fs  ss_err=%.4f  osc_period=%s"
              % (m["rise_time"] if m["rise_time"] is not None else math.nan,
                 m["overshoot_pct"],
                 m["settling_time"] if m["settling_time"] is not None else math.nan,
                 m["ss_error"],
                 "%.2fs" % m["oscillation_period"] if m["oscillation_period"] else "n/a"))
        if m["oscillation_period"]:
            tu = m["oscillation_period"]
            print("  >> estimate Ku by increasing kp until sustained oscillation, "
                  "Tu≈%.2fs" % tu)
    print("-" * 70)

    if args.zn and args.ku is not None and args.tu is not None:
        zn = ziegler_nichols(args.ku, args.tu)
        print("Ziegler-Nichols (Ku=%.3f, Tu=%.3f):" % (args.ku, args.tu))
        for name, g in zn.items():
            print("  %-3s kp=%.3f ki=%.4f kd=%.3f" % (name, g["kp"], g["ki"], g["kd"]))

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("WARNING: matplotlib not available; --plot ignored.")
            return
        fig = plt.figure()
        ax: Any = fig.add_subplot(1, 1, 1)
        for label, gains, times, resp, _ in all_results:
            ax.plot(times, resp, label=label)
        ax.axhline(setpoint, color="k", linestyle="--", linewidth=0.8, label="setpoint")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("response")
        ax.set_title("PID sim — axis=%s" % args.axis)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()


# --------------------------------------------------------------------------- #
# Command: step
# --------------------------------------------------------------------------- #
def _yaw_from_pose(msg):
    q = msg.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    return wrap_angle(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def cmd_step(args):
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32
        from geometry_msgs.msg import PoseStamped
        from robosub_msgs.msg import NavigationCommand  # type: ignore[import-not-found]
    except ImportError as e:
        print("ERROR: ROS not available (%s). Use 'sim' for offline tuning." % e)
        return 1

    rclpy.init()
    node = Node("pid_step_recorder")

    state = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "depth": 0.0,
             "have_pose": False, "have_depth": False}

    def on_pose(msg):
        p = msg.pose.position
        state["x"] = p.x
        state["y"] = p.y
        state["z"] = p.z
        state["yaw"] = _yaw_from_pose(msg)
        state["have_pose"] = True

    def on_depth(msg):
        state["depth"] = msg.data
        state["have_depth"] = True

    node.create_subscription(PoseStamped, "localization/pose", on_pose, 10)
    node.create_subscription(Float32, "depth/sub_depth", on_depth, 10)
    pub = node.create_publisher(NavigationCommand, "navigation_command", 10)

    # Read current pose
    t0 = time.time()
    while time.time() - t0 < 2.0 and not state["have_pose"]:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not state["have_pose"]:
        print("ERROR: no pose received on localization/pose within 2s.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    cur_yaw = state["yaw"]
    axis = args.axis
    step = args.step

    cmd = NavigationCommand()
    cmd.speed = 0.0
    cmd.approach_dist = 0.0
    cmd.target_label = ""

    if axis == "yaw":
        target = wrap_angle(cur_yaw + step)
        cmd.mode = "heading_hold"
        cmd.target_yaw = target
        cmd.target_x = state["x"]
        cmd.target_y = state["y"]
        cmd.target_z = state["z"]
    else:
        cmd.mode = "waypoint"
        cmd.target_x = state["x"] + (step if axis == "surge" else 0.0)
        cmd.target_y = state["y"] + (step if axis == "strafe" else 0.0)
        cmd.target_z = state["z"] + (step if axis == "heave" else 0.0)
        cmd.target_yaw = cur_yaw
        target = cmd.target_x if axis == "surge" else (
            cmd.target_y if axis == "strafe" else cmd.target_z)

    print("step: axis=%s step=%.3f current_yaw=%.3f target=%.3f seconds=%d"
          % (axis, step, cur_yaw, target, args.seconds))
    pub.publish(cmd)

    # Record
    times, responses, setpoints = [], [], []
    t_start = time.time()
    while time.time() - t_start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        t = time.time() - t_start
        if axis == "yaw":
            val = state["yaw"]
        elif axis == "surge":
            val = state["x"]
        elif axis == "strafe":
            val = state["y"]
        else:
            val = state["z"]
        times.append(t)
        responses.append(val)
        setpoints.append(target)

    # Stop
    stop = NavigationCommand()
    stop.mode = "idle"
    pub.publish(stop)
    time.sleep(0.1)
    pub.publish(stop)

    node.destroy_node()
    rclpy.shutdown()

    # CSV
    if args.csv:
        fname = (args.csv if isinstance(args.csv, str) and args.csv != "__auto__"
                 else "step_%s.csv" % axis)
        try:
            with open(fname, "w") as f:
                f.write("time,setpoint,response,error\n")
                for t, sp, r in zip(times, setpoints, responses):
                    err = wrap_angle(sp - r) if axis == "yaw" else (sp - r)
                    f.write("%.4f,%.6f,%.6f,%.6f\n" % (t, sp, r, err))
        except OSError as e:
            print("ERROR writing CSV %s: %s" % (fname, e))
        else:
            print("Wrote %s (%d samples)" % (fname, len(times)))

    # Metrics
    m = compute_metrics(times, responses, target)
    print("-" * 60)
    print("rise_time=%s  overshoot=%.1f%%  settle=%s  ss_err=%.4f  osc_period=%s"
          % ("%.2fs" % m["rise_time"] if m["rise_time"] is not None else "n/a",
             m["overshoot_pct"],
             "%.2fs" % m["settling_time"] if m["settling_time"] is not None else "n/a",
             m["ss_error"],
             "%.2fs" % m["oscillation_period"] if m["oscillation_period"] else "n/a"))

    if args.zn:
        if m["oscillation_period"]:
            tu = m["oscillation_period"]
            print("\nZ-N guidance: increase kp until sustained oscillation (Ku), "
                  "then:")
            print("  Tu ≈ %.2fs (estimated from current oscillation)" % tu)
            print("  Z-N: kp=0.6*Ku, ki=1.2*Ku/Tu, kd=0.075*Ku*Tu")
            print("  Pass --ku <Ku> --tu %.2f to sim --zn for gain sets." % tu)
        else:
            print("\nNo clear oscillation detected. Increase kp until you see "
                  "sustained oscillation to find Ku, then estimate Tu.")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("WARNING: matplotlib not available; --plot ignored.")
            return 0
        fig = plt.figure()
        ax: Any = fig.add_subplot(1, 1, 1)
        ax.plot(times, responses, label="response")
        ax.plot(times, setpoints, "k--", linewidth=0.8, label="setpoint")
        ax.set_xlabel("time (s)")
        ax.set_ylabel(axis)
        ax.set_title("Step response — %s step=%.3f" % (axis, step))
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    return 0


# --------------------------------------------------------------------------- #
# Command: set
# --------------------------------------------------------------------------- #
def cmd_set(args):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.parameter_client import AsyncParameterClient  # type: ignore[import-not-found]
    except ImportError as e:
        print("ERROR: ROS not available (%s)." % e)
        return 1

    rclpy.init()
    node = Node("pid_setter")
    client = AsyncParameterClient(node, "autonomous_controller")

    prefix = args.pid
    params = []
    for name in ["kp", "ki", "kd", "limit", "i_limit"]:
        val = getattr(args, name)
        if val is not None:
            # val already typed float by argparse
            params.append(Parameter("%s.%s" % (prefix, name), value=val))

    if not params:
        print("ERROR: specify at least one of --kp/--ki/--kd/--limit/--i_limit")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    future = client.set_parameters(params)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()
    if future.done() and result is not None:
        results = result.results
        ok = all(r.successful for r in results)
        if ok:
            print("OK: set %d params on %s:" % (len(params), prefix))
            for p in params:
                print("  %s = %s" % (p.name, p.value))
        else:
            print("WARN: some params failed:")
            for r in results:
                if not r.successful:
                    print("  %s: %s" % (r, r.reason))
    else:
        print("ERROR: timed out talking to /autonomous_controller "
              "(is it running?)")

    node.destroy_node()
    rclpy.shutdown()
    return 0


# --------------------------------------------------------------------------- #
# Command: list
# --------------------------------------------------------------------------- #
def cmd_list(args):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter_client import AsyncParameterClient  # type: ignore[import-not-found]
    except ImportError as e:
        print("ERROR: ROS not available (%s)." % e)
        return 1

    rclpy.init()
    node = Node("pid_lister")
    client = AsyncParameterClient(node, "autonomous_controller")

    prefixes = ["pid_x", "pid_y", "pid_z", "pid_yaw",
                "pid_vis_yaw", "pid_vis_strafe", "pid_vis_depth"]
    suffixes = ["kp", "ki", "kd", "limit", "i_limit"]
    names = ["%s.%s" % (p, s) for p in prefixes for s in suffixes]

    future = client.get_parameters(names)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()
    if not future.done() or result is None:
        print("ERROR: timed out talking to /autonomous_controller "
              "(is it running?)")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    values = {n: v.value for n, v in zip(names, result.values)}
    print("%-14s %8s %8s %8s %8s %8s" % ("pid", "kp", "ki", "kd", "limit", "i_lim"))
    print("-" * 60)
    for p in prefixes:
        print("%-14s %8.4f %8.4f %8.4f %8.3f %8.3f" % (
            p,
            values["%s.kp" % p], values["%s.ki" % p], values["%s.kd" % p],
            values["%s.limit" % p], values["%s.i_limit" % p]))

    node.destroy_node()
    rclpy.shutdown()
    return 0


# --------------------------------------------------------------------------- #
# argparse
# --------------------------------------------------------------------------- #
def build_parser():
    p = argparse.ArgumentParser(
        prog="tune_pid.py",
        description="PID tuning tool for the RoboSub autonomous controller.")
    sub = p.add_subparsers(dest="command")

    # sim
    ps = sub.add_parser("sim", help="Offline simulator (no ROS).")
    ps.add_argument("--axis", choices=list(DEFAULT_GAINS.keys()), default="yaw")
    ps.add_argument("--kp", type=float, default=None)
    ps.add_argument("--ki", type=float, default=None)
    ps.add_argument("--kd", type=float, default=None)
    ps.add_argument("--limit", type=float, default=None)
    ps.add_argument("--i_limit", type=float, default=None)
    ps.add_argument("--mass", type=float, default=1.0)
    ps.add_argument("--drag", type=float, default=0.3)
    ps.add_argument("--setpoint", type=float, default=1.0)
    ps.add_argument("--seconds", type=int, default=10)
    ps.add_argument("--compare", type=str, default=None,
                    help='e.g. "kp=1.2,ki=0.08,kd=0.3;kp=1.5,ki=0,kd=0.4"')
    ps.add_argument("--plot", action="store_true")
    ps.add_argument("--zn", action="store_true",
                    help="Print Z-N gain sets (needs --ku/--tu).")
    ps.add_argument("--ku", type=float, default=None)
    ps.add_argument("--tu", type=float, default=None)
    ps.set_defaults(func=cmd_sim)

    # step
    pst = sub.add_parser("step", help="Pool step-response recorder (needs ROS).")
    pst.add_argument("--axis", choices=["yaw", "surge", "strafe", "heave"],
                     default="yaw")
    pst.add_argument("--step", type=float, default=0.5,
                     help="radians (yaw) or meters (position)")
    pst.add_argument("--seconds", type=int, default=12)
    pst.add_argument("--csv", nargs="?", const="__auto__", default=None)
    pst.add_argument("--plot", action="store_true")
    pst.add_argument("--zn", action="store_true")
    pst.set_defaults(func=cmd_step)

    # set
    pse = sub.add_parser("set", help="Live gain applier (needs ROS).")
    pse.add_argument("--pid", required=True,
                     choices=list(AXIS_PARAM.values()))
    pse.add_argument("--kp", type=float, default=None)
    pse.add_argument("--ki", type=float, default=None)
    pse.add_argument("--kd", type=float, default=None)
    pse.add_argument("--limit", type=float, default=None)
    pse.add_argument("--i_limit", type=float, default=None)
    pse.set_defaults(func=cmd_set)

    # list
    pl = sub.add_parser("list", help="Dump current PID params (needs ROS).")
    pl.set_defaults(func=cmd_list)
    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "command", None):
        parser.print_help()
        return 1
    rc = args.func(args)
    return rc or 0


if __name__ == "__main__":
    sys.exit(main())
