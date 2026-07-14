# heading_lock — ZED-IMU Heading Stabilization for Forward Motion (Design)

**Date:** 2026-07-14
**Status:** Approved (brainstorm 2026-07-14)
**Problem:** Commanded forward motion veers right (see `docs/superpowers/plans/` 2026-07-13 veer plans). MANUAL-mode surge has no yaw feedback, so any thruster imbalance integrates into a turn. This feature closes the loop: capture yaw when forward motion starts, hold it with a PID for the whole leg.

## Goal

When the sub is commanded forward, it locks its current ZED 2i IMU yaw as the target heading and continuously corrects yaw so it drives straight, balancing the four vectored thrusters (motors 1 FR, 2 FL, 3 RR, 4 RL) left vs right. Lock releases when forward motion stops.

## Decisions made (with user, 2026-07-14)

| Question | Decision |
|---|---|
| Actuation path | MAVLink `MANUAL_CONTROL` x (surge) + r (yaw rate) via existing `ThrusterController`. ArduSub's vectored mixer produces the motor 1–4 left/right differential. **No literal per-motor writes** — the Jetson has no mission-safe per-motor path (DO_MOTOR_TEST is test-only: dead-man timer, bypasses mixer, kills depth hold). |
| Language | Python / rclpy (matches entire stack; symlink-install live edits at the pool; reuses existing PID + param patterns). |
| Integration | Pure-logic class + thin node in `src/control` (package `control`). Reusable by `autonomous_controller` / BT behaviors later. |
| Yaw source | `imu/rpy` (`geometry_msgs/Vector3Stamped`, `vector.z` = yaw rad, REP-103 CCW-positive) from `src/imu/orientation_node.py` (zeroed ZED 2i IMU). Topic name is a ROS parameter. |
| Stale yaw mid-run | Grace period then stop: correction→0 immediately; keep commanded forward for `grace_s`; still stale → publish stop, release lock, ERROR log. Yaw returning within grace resumes with the ORIGINAL target (no re-lock). |

## Architecture

```
zed-ros2-wrapper ─► orientation_node ─► imu/rpy (yaw rad, CCW+)
                                            │
heading_lock/cmd (Float32 speed) ──► heading_lock_node ── rate_hz ──► movement_command
                                            │                          {command:'axes',
                                      debug topics ×8                   surge:base,
                                                                        yaw_rate:corr}
                                                                ─► ThrusterController
                                                                ─► MANUAL_CONTROL(x, r)
                                                                ─► ArduSub mixer ─► motors 1–4
```

Depth is untouched: `heave = 0.0` in axes mode → ALT_HOLD keeps owning depth (z=500 neutral, see `thruster_node.set_axes`).

## Components

### 1. `src/control/control/pid.py` (extracted, not new logic)

The `PID` class currently defined inside `src/control/control/autonomous_controller.py` moves verbatim into `control/pid.py` (gains, `limit`, `i_limit` anti-windup, `reset()`, `set_gains()`, `update(error, dt)`). `autonomous_controller.py` imports it from there — behavior identical, no duplication.

### 2. `src/control/control/heading_lock.py` (pure logic, no ROS imports)

`HeadingLock` — unit-testable state machine + control law.

**States:** `IDLE → LOCKED → STALE_GRACE → ABORTED` (stop() from any state → IDLE).

**API:**
- `start(current_yaw_rad, base_speed)` — captures `target_yaw`, resets PID, → LOCKED.
- `update(yaw_or_none, now_s, dt_s) -> (surge, yaw_rate, state)` — one control tick. `yaw_or_none=None` or stale timestamp drives the STALE_GRACE/ABORTED path.
- `stop()` — → IDLE, resets PID integrator.
- Properties: `target_yaw`, `state`, `last_error`.

**Control law and sign convention (the load-bearing part):**

Input yaw is REP-103 (CCW-positive). `MovementCommand.yaw_rate` is CW-positive.

```
error_cw = wrap(current_yaw - target_yaw)          # wrap to [-pi, pi]; <0 when drifted CW
yaw_rate = clamp(PID(error_cw), ±max_yaw_authority)
surge    = base_speed                               # constant; correction rides on r only
```

Drifted CW (nose right): `current_yaw` decreased → `error_cw < 0` → `yaw_rate < 0` = CCW command → mixer raises RIGHT pair (motors 1 & 3), lowers LEFT pair (2 & 4) → nose returns. Mirror case symmetric.

> Note: the original request's prose ("CW drift → increase left thrust") is positive feedback and contradicts its own pseudocode; this spec implements the (correct) pseudocode semantics. Signs are pinned by unit tests and a bench check.

**Stale handling inside `update`:**
- fresh yaw: normal PID output.
- stale < `grace_s`: `yaw_rate = 0`, surge continues, state STALE_GRACE (PID integrator frozen, not reset).
- stale ≥ `grace_s`: returns `(0, 0, ABORTED)` once; caller must publish stop.
- yaw recovers during grace: back to LOCKED with the original target.

### 3. `src/control/control/heading_lock_node.py` (thin ROS wrapper)

**Subscriptions:**
- `imu/rpy` (`geometry_msgs/Vector3Stamped`) — topic from `yaw_topic` param. Staleness measured from node-clock ARRIVAL time (not `header.stamp` — immune to source clock skew): the node passes `None` to `HeadingLock.update()` when the last sample is older than `stale_timeout_s`.
- `heading_lock/cmd` (`std_msgs/Float32`) — `data > 0`: lock current yaw, drive forward at `min(data, max_forward_speed)`; `data <= 0`: stop + unlock. New positive value while LOCKED updates base speed WITHOUT re-locking the target.

**Publications:**
- `movement_command` (`auv_msgs/MovementCommand`, `command='axes'`, surge=base, yaw_rate=correction, all else 0) at `rate_hz`.
- Debug (all `std_msgs/Float32`, published every tick while not IDLE):
  - `heading_lock/current_yaw` (rad), `heading_lock/target_yaw` (rad), `heading_lock/error` (rad, wrapped), `heading_lock/pid_output`
  - `heading_lock/motor1` … `motor4` — **commanded intent**: motor2 = motor4 = base + correction (left), motor1 = motor3 = base − correction (right). Actual PWM belongs to ArduSub; reading back `SERVO_OUTPUT_RAW` is out of scope.

**Parameters (all declared, runtime-tunable where meaningful):**

| Param | Default | Notes |
|---|---|---|
| `kp` / `ki` / `kd` | 1.2 / 0.0 / 0.3 | live via `ros2 param set` (same callback pattern as `autonomous_controller`); starting values borrowed from `pid_yaw`, tune at pool. ki=0 → pure PD start per request ("P controller initially") |
| `max_yaw_authority` | 0.4 | clamp on yaw_rate output |
| `max_forward_speed` | 0.6 | clamp on commanded surge |
| `stale_timeout_s` | 0.5 | yaw older than this = stale |
| `grace_s` | 1.0 | open-loop forward budget after stale |
| `rate_hz` | 20.0 | control/publish rate (ThrusterController streams MAVLink at 10 Hz; 20 Hz keeps its inputs fresh) |
| `yaw_topic` | `imu/rpy` | future hook: point at a vslam-yaw adapter when the detector owns the ZED |

**Lifecycle/safety:** on ABORTED, node shutdown, or malformed cmd → publish `command='stop'` once and go IDLE. Exceptions in the tick are caught → stop + IDLE (mirrors `autonomous_controller._control_tick`).

**Console entry point:** `heading_lock_node` added to `control` package `setup.py` entry_points.

## Error handling summary

| Failure | Behavior |
|---|---|
| Yaw stale < grace_s | correction 0, keep forward, WARN once |
| Yaw stale ≥ grace_s | stop published, unlock, ERROR |
| Yaw recovers in grace | resume PID against original target |
| cmd ≤ 0 / NaN / node shutdown | stop + unlock |
| Tick exception | stop + IDLE, error logged |

## Testing

**Unit (`tests/test_heading_lock.py`, pure logic, no ROS):**
1. `start` captures target; state LOCKED.
2. CW drift (yaw decreases) → negative yaw_rate (CCW correction); CCW drift → positive. Both directions.
3. Wrap: target +170°, current −170° → error −20° (not +340°), correction CW.
4. Output clamped to ±max_yaw_authority; surge equals base_speed always.
5. Stale: within grace → yaw_rate 0 + surge continues; past grace → ABORTED once.
6. Recovery in grace keeps original target.
7. `stop()` resets integrator (no windup carryover into next leg).

**Bench (dry, disarmed):** wrapper + `orientation_node` + `thruster_node` up, `ros2 topic pub heading_lock/cmd`. Hand-rotate sub nose-right: `heading_lock/error` goes negative, `yaw_rate`/`pid_output` negative, motor1/motor3 intent rises above motor2/motor4. Nose-left mirrors. This is the sign-chain gate before water.

**Water:** forward leg with lock vs without; veer-right should null. Heading lock MASKS thruster imbalance — pool order from the veer diagnosis still applies: sweep → trim → re-sweep → then this closes the residual.

## Operational constraints

- **Single writer:** while `heading_lock_node` drives `movement_command`, do NOT run root field scripts (they send raw MANUAL_CONTROL — two writers on the FC fight). One command authority at a time.
- **ZED single owner:** `zed-ros2-wrapper` (IMU topic) and the vision detector cannot both own the ZED. This feature as specced runs in wrapper mode; `yaw_topic` param is the migration hook for a detector/vslam yaw source.
- **Preflight:** the non-skippable thruster param preflight (root scripts) is unchanged and still required before dives; this node assumes a correctly-configured FC.
- Build: `colcon build --symlink-install --packages-select control` (stale `install/` is the recurring trap).

## Out of scope

- Reading actual PWM back from the FC (`SERVO_OUTPUT_RAW`).
- Strafe/heave heading compensation; only forward legs.
- Pixhawk-yaw fusion (`heading_common` plan) — separate effort; this feature is ZED-IMU-only per request.
- Wiring into `autonomous_controller` modes / BT nodes (class is designed for it; wiring is a follow-up).
