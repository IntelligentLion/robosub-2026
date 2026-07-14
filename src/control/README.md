# control — submerge and hold

Dive to a depth, hold it, and drive forward in a straight line without steering.

```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch control submerge_hold.launch.py            # add rviz:=true for RViz
```

```python
from control.api import Auv

with Auv() as auv:
    auv.submerge_to_depth(target_depth=2.0, dive_speed=0.3)  # blocks until held
    auv.move_forward(speed=0.4, duration=10)                 # depth+heading+attitude auto-held
```

## Who controls what

The guiding rule: **ArduSub keeps the loops it is already good at.** It has the
vehicle's thrust model and runs at the autopilot's rate, so re-implementing its
controllers in Python would only mean two controllers fighting over one axis.

| Axis | Owner |
|---|---|
| depth | **ArduSub** (ALT_HOLD, on the Bar02). No custom depth PID. |
| roll, pitch | **ArduSub** (ALT_HOLD self-levels). Never commanded from here. |
| **heading** | **us** — the one genuine gap. ArduSub has no yaw-heading-hold mode. |
| surge, strafe | the operator. The only axes you touch. |

The dive itself is ours only because ALT_HOLD holds a depth rather than seeking
one; `DepthController` walks the sub down to the target and then hands the axis
back permanently.

## Nodes

```
Pixhawk ──serial── thruster_node ──┬─► pixhawk/imu/data, pixhawk/depth
                   (sole reader,   ├─► pixhawk/mode, pixhawk/armed
                    sole writer)   └─◄ movement_command
                        ▲ services: pixhawk/set_mode, pixhawk/preflight
                        │
                   motion_node  ── SOLE publisher of movement_command
                        │           SubmergeController → DepthController
                        │           HeadingController  → MotionController
                        ▼
                   rviz_visualizer  (subscribe-only; omit it freely)
```

`thruster_node` is the only process that may open the serial port, and the only
one that imports `pymavlink`. **Never run `pix_imu/pixhawk_imu_bridge` alongside
it** — two readers on one port produce the `device reports readiness to read but
returned no data` stall and both die. That bridge is for dry-bench work with no
`thruster_node` running.

`motion_node` is the only publisher of `movement_command`. It counts publishers
on that topic and inhibits itself if it is not alone, because two publishers
means two things fighting over the thrusters with no arbiter.

## Safety

The failure paths deliberately differ, because the right answer differs.

| Loss | Response |
|---|---|
| **preflight mismatch** | Abort before arming. A flipped `MOT_x_DIRECTION` turns forward thrust into a spin. Read-only, no bypass flag. |
| **ALT_HOLD refused** | Abort **on the surface**. ArduSub refuses the mode with no depth sensor and silently stays where it was; diving anyway means descending with no depth hold. |
| **heading lost** | Yaw correction → 0 immediately (never steer blind). Forward rides out `grace_s`, then stops. Depth hold is untouched. |
| **heading degraded** | Duty-cycle abort, latched until acknowledged — a source that keeps dropping *under* the stale timeout never trips the plain grace abort. |
| **depth lost** | Stop moving. Stay in ALT_HOLD; the autopilot may still be holding. |
| **mode leaves ALT_HOLD** | Depth hold is gone. Stop moving. |

## Position: what RViz is showing you

**A Bar02 and an IMU cannot determine XY position.** No DVL, no GPS, no USBL.
Integrating IMU acceleration twice diverges within seconds.

So the default `pose_source: pixhawk_imu` dead-reckons XY from the *commanded*
velocity through the measured heading, colours the path orange, and prints
`POSITION ESTIMATED — no XY sensor` on screen. It shows the **shape** of the
path ("did the heading lock keep us straight?"), not where the sub is. Depth is
measured and exact; only XY is a guess.

`pose_source: zed` relays the existing `vslam/odometry` instead, and is honest
about being measured. A DVL or EKF drops in as a third `PoseSource` with no
controller change.

**Station-keeping is not implemented and is not possible with these sensors.**
With zero command the sub holds depth, heading and attitude, and drifts with the
current in XY. Closing that needs real position feedback.

## Tuning

Everything is live-tunable except `control_rate_hz` and `yaw_topic` (which only
apply at construction, so they are rejected rather than silently ignored):

```bash
ros2 param set /motion_node heading_kp 1.6
ros2 topic echo /heading/error
rqt_plot /heading/error /heading/yaw_correction /motion/forward_cmd
```

The param bounds are **safety limits, not taste** — see the `PARAM_BOUNDS`
comment in `motion_node.py`. A `max_yaw_correction` of 0 inverts the clamp into
constant full authority and spins the sub; a `stale_duty_abort` of exactly 1.0
makes the degraded-source abort unreachable.
