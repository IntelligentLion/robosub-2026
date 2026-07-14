# Testing submerge-and-hold

Four stages, cheapest first. Stages 1–2 need no hardware. Stage 3 needs the
Pixhawk powered but dry. Stage 4 is the pool.

Do not skip ahead. Each stage exists to catch a class of fault that would be
more expensive — or wetter — to find in the next one.

```bash
cd ~/robosub2026/robosub-2026
source /opt/ros/humble/setup.bash
colcon build --symlink-install        # ALWAYS --symlink-install; a stale install/ is the classic "my fix didn't take" trap
source install/setup.bash
```

---

## Stage 1 — offline test suite (no hardware, ~35 s)

```bash
python -m pytest tests/ -q
```

Expect **153 passed**. If anything fails here, stop; nothing downstream is
trustworthy.

What the feature-specific files pin:

| File | What it protects |
|---|---|
| `tests/test_depth_controller.py` | Dive tolerance, timeout, the ALT_HOLD throttle-deadzone floor, never diving on stale depth. |
| `tests/test_submerge.py` | Preflight → mode → arm ordering. Zero heave on **every** failure path. |
| `tests/test_motion.py` | Axis authority: the operator cannot touch heave, roll or pitch. |
| `tests/test_motion_node.py` | The three loss paths, the sole-publisher guard, the param bounds. |
| `tests/test_gateway.py` | Mode readback, hull-baro rejection, surface latch sanity. |
| `tests/test_gateway_services.py` | set_mode refusal reason, preflight fails closed on an unreadable param. |
| `tests/test_e2e_submerge.py` | The whole feature, end to end. See below. |
| `tests/test_pose_source.py` | Dead-reckoning always declares itself an estimate. |

### The one that actually proves the feature

```bash
python -m pytest tests/test_e2e_submerge.py -v
```

It runs a **live `motion_node`** against a simulated vehicle whose depth
responds to commanded heave and which **veers right under forward thrust** —
the 2026-07-13 symptom. Six assertions:

- reaches 2 m and enters ALT_HOLD
- releases heave at depth (ALT_HOLD takes over)
- holds the captured heading through 6 s of forward travel: a veer that would
  swing the sub ~2.1 rad open-loop is held under **0.15 rad**
- the veer is genuinely present (drives `movement_command` directly, bypassing
  the lock, and asserts the sub *does* swing — so the test above cannot pass
  vacuously)
- a **refused ALT_HOLD** aborts with the sub still at depth **0.0**
- a **failed preflight** aborts with the sub still at depth **0.0**

---

## Stage 2 — simulated launch (no hardware)

```bash
ros2 launch control submerge_hold.launch.py simulate:=true viz:=true rviz:=true
```

In a second terminal:

```bash
ros2 topic list | grep -E 'pixhawk|motion|heading|depth|submerge|viz'
ros2 service list | grep pixhawk
```

Expect all of:

```
/pixhawk/imu/data  /pixhawk/depth  /pixhawk/mode  /pixhawk/armed
/motion/cmd  /motion/submerge  /motion/forward_cmd  /motion/vertical_cmd
/heading/current  /heading/target  /heading/error  /heading/yaw_correction
/depth/current  /depth/target  /submerge/state  /movement_command
/viz/markers  /viz/path
/pixhawk/preflight  /pixhawk/set_mode
```

You will **not** get a dive here. In simulation `thruster_node` publishes no
depth, and `motion_node` correctly refuses to dive without it. **That refusal is
the test** — it is the "never dive blind" rule firing.

Confirms: launch wiring, RViz config loads, every topic and service appears.

---

## Stage 3 — dry bench, real Pixhawk

> ### ⚠ TAKE THE PROPELLERS OFF FIRST
> `thruster_node` **arms the vehicle when it connects**. The moment anything
> publishes `movement_command`, the motors spin. Props off, or the vehicle
> physically restrained and clear of hands, cables and the bench edge.

> Also: do **not** run `pix_imu/pixhawk_imu_bridge` at the same time. It opens
> the same serial port. Two readers on one port produce the
> `device reports readiness to read but returned no data` stall and both die.

### 3a. Bring up the gateway

```bash
ros2 launch control submerge_hold.launch.py
```

Watch the log. Expect:

```
MAVLink connected on /dev/ttyACM0
MAVLink gateway reader started
Depth source: SCALED_PRESSURE2
```

**If `Depth source:` never appears**, the Bar02 is not on I2C. That link has
dropped twice before (2026-07-09, 2026-07-13). Reseat the cable before going
further — with no external baro there is no depth and no ALT_HOLD.

### 3b. Preflight gate — the highest-value hardware test

Read-only, moves nothing, and catches the fault that has actually bitten:

```bash
ros2 service call /pixhawk/preflight std_srvs/srv/Trigger
```

**Expect:** `success=True`, `"thruster config matches the known-good backup"`.

**If it lists a mismatch** — e.g. `MOT_3_DIRECTION = -1 but backup says +1` —
**STOP.** A flipped horizontal turns a forward command into a spin; a flipped
vertical makes one thruster fight the other three on the dive. Fix it in QGC
against `pixhawk_params_4.5.7_backup_2026-07-08.param` before anything else.
There is no bypass flag, by design.

### 3c. Telemetry sanity

```bash
ros2 topic echo /pixhawk/depth --once     # ~0.0 dry (surface latched), NaN if no Bar02
ros2 topic echo /pixhawk/mode  --once     # the mode the vehicle is ACTUALLY in
ros2 topic echo /pixhawk/armed --once
ros2 topic echo /imu/rpy       --once     # yaw should track when you rotate the sub
```

Physically rotate the sub and confirm `/imu/rpy` `vector.z` changes. If yaw is
dead, the heading lock has nothing to lock onto and `submerge` will abort at the
capture step (by design — it refuses to lock a garbage heading).

### 3d. Prove the dead-Bar02 abort — do this deliberately, once

This is the failure that sinks subs, so provoke it while dry:

```bash
# unplug the Bar02 from I2C, then:
ros2 service call /pixhawk/set_mode auv_msgs/srv/SetFlightMode "{mode: 'ALT_HOLD'}"
```

**Expect:** `success=False`, and a reason containing
`Depth sensor is not connected.` (ArduSub's own STATUSTEXT).

Now confirm the abort propagates:

```bash
ros2 topic echo /submerge/state &
ros2 topic pub --once /motion/submerge std_msgs/Float32 '{data: 1.0}'
```

**Expect:** `failed: cannot enter ALT_HOLD: ... Depth sensor is not connected.`
and **no thruster movement at all**. If the motors spin, stop and report it —
that is the exact scenario the design exists to prevent.

Plug the Bar02 back in and restart the launch.

---

## Stage 4 — in water

Tether on **5 GHz or ethernet**. ZED USB3 noise jams 2.4 GHz and will drop your
SSH mid-dive.

### 4a. Launch and confirm the surface latch

```bash
ros2 launch control submerge_hold.launch.py rviz:=true
```

Expect in the log, **while the sub is floating at the surface**:

```
Surface latched at 1013.x hPa — depth live
```

That is the zero reference for every depth reading afterwards. If the sub was
already submerged when it latched, **every depth is wrong** — restart at the
surface. An implausible latch is rejected outright and depth stays `NaN`.

### 4b. Record before you dive

```bash
ros2 bag record /heading/current /heading/target /heading/error \
                /heading/yaw_correction /motion/forward_cmd /motion/vertical_cmd \
                /depth/current /depth/target /pixhawk/mode /submerge/state
```

```bash
rqt_plot /heading/error /heading/yaw_correction /motion/forward_cmd
```

### 4c. Shallow dive first — 1 m, not the full 2 m

```bash
ros2 topic echo /submerge/state &
ros2 topic pub --once /motion/submerge std_msgs/Float32 '{data: 1.0}'
```

**Expect the phases in order:**
`preflight` → `mode_set` → `arming` → `diving` → `hold`

Then, **before commanding any forward motion**, confirm two things:

1. `/pixhawk/mode` says `ALT_HOLD`.
2. The sub **stays** at depth with `/motion/vertical_cmd` at `0.0` — i.e.
   ArduSub is holding it, not us. Watch it for 20–30 s.

If it sinks or rises with vertical_cmd at zero, ALT_HOLD is not really holding.
Abort and investigate before adding forward thrust.

### 4d. Short forward run

```python
from control.api import Auv

with Auv() as auv:
    auv.move_forward(speed=0.3, duration=5)     # short. then look at the plot.
```

Watch `/heading/error` on `rqt_plot`. It should stay small and **not oscillate**.

### 4e. Then the real thing

```python
from control.api import Auv

with Auv() as auv:
    auv.submerge_to_depth(target_depth=2.0, dive_speed=0.3)
    auv.move_forward(speed=0.4, duration=10)
```

### Abort, any time

```bash
ros2 topic pub --once /motion/submerge std_msgs/Float32 '{data: 0.0}'
```

Stops movement and releases the lock. The vehicle stays in ALT_HOLD holding its
current depth — **this does not surface it**. Disarm to come up.

### Expected: it disarms if you idle on the surface

`motion_node` publishes nothing while idle, and `thruster_node`'s watchdog
disarms after 60 s without a `movement_command`. So if you launch and then spend
a couple of minutes setting up before diving, you will see
`Vehicle DISARMED unexpectedly – re-arming …`.

This is fine and self-correcting: the armed-status check re-arms within 5 s, and
the `arming` phase of a dive waits for it. Not a fault — do not chase it.

---

## Reading the results

`/heading/error` is the number that says whether this worked.

| Symptom | Meaning | Fix |
|---|---|---|
| Error small, flat | Working. | — |
| Error rings / oscillates | `heading_kp` too high. | `ros2 param set /motion_node heading_kp 0.8` |
| Error drifts one way, correction saturated at ±0.4 | Not enough authority, or a thruster fault. | Re-run preflight. Then raise `max_yaw_correction`. |
| Error creeps back slowly, never settles | Needs a little I. | `ros2 param set /motion_node heading_ki 0.05` |

Everything is live-tunable **except** `control_rate_hz` and `yaw_topic`, which
only apply at construction — so they are **rejected** rather than silently
ignored:

```bash
ros2 param set /motion_node heading_kp 1.6      # OK, applies immediately
ros2 param set /motion_node control_rate_hz 30  # REJECTED: "requires node restart"
```

The param bounds are **safety limits, not taste** (see the `PARAM_BOUNDS`
comment in `motion_node.py`). Two of them exist because the naive value silently
disables a safety contract while reporting success:

- `max_yaw_correction = 0` inverts the clamp into **constant full authority** —
  the sub spins regardless of error.
- `stale_duty_abort = 1.0` makes the degraded-source abort **unreachable**, even
  at 100 % stale ticks.

Both are rejected.

---

## What is NOT tested, because it is not implemented

**Station-keeping.** "Hold position when no command is given" needs XY position
feedback. A Bar02 and an IMU cannot provide it — no DVL, no GPS, no USBL, and
double-integrating IMU acceleration diverges within seconds.

With zero command the sub holds **depth, heading and attitude**, and **drifts
with the current in XY**. That is expected behaviour, not a bug. Do not go
looking for the fault.

RViz says so on screen: the dead-reckoned track renders orange and labelled
`POSITION ESTIMATED — no XY sensor`. It shows the *shape* of the path ("did the
heading lock keep us straight?"), not where the sub is. Depth is measured and
exact; only XY is a guess.
