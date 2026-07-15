# attic

Superseded scripts kept for reference, **not for flight use**.

## Pre-new-ROS-API toolkit (archived 2026-07-14)

The 2026 stack moved to a ROS-node control API — `thruster_node` (MAVLink
gateway), `pixhawk_imu_bridge`/orientation → `imu/rpy`, and **`motion_node`** as
the sole publisher of `movement_command`, driven through `control.api.Auv`.
Everything below predates that: it either opens its own MAVLink connection
(direct pymavlink), streams `movement_command` itself via the old
`field_common` engine (which `motion_node` now INHIBITS against), or drives the
retired `autonomous_controller` / `NavigationCommand` ROS API. They are kept
because several encode hard-won hardware lessons, and the diagnostics still run
standalone — but nothing in the live stack should import or launch them.

Scripts import each other, so they were moved together and still resolve within
this folder.

**Old control engine + movement/mission scripts**
- `field_common.py` — the old single-writer MAVLink streamer library.
- `act_center_gate.py`, `act_coords.py`, `act_forward.py`, `act_strafe_left.py`,
  `act_strafe_right.py`, `act_turn_left.py` — field_common action scripts.
- `depth_field_test.py`, `depth_hold.py`, `depth_hold_pix_test.py`,
  `depth_hold_bar02_test.py` — old depth-hold engines/tests.
- `submerge_forward.py`, `submerge_forward_10ft.py`, `run_course.py` —
  MANUAL-mode Bar02-only dive+forward runners.
- `diagnose_forward_veer.py` (+ `test_diagnose_veer.py`) — veer diagnosis.
- `gate_begin_assessment.py`, `gate_spin_pass.py`, `gate_task.py`,
  `stage_gate_detect.py`, `stage_marker.py`, `stage_marker_detect.py` — old
  field_common task runners.

**Retired ROS API (autonomous_controller / NavigationCommand)**
- `tune_pid.py`, `tune.launch.py`, `test_centering.py`, `test_mission.sh`.

**Direct-MAVLink hardware diagnostics** (talk raw MAVLink by necessity — they
calibrate/probe the Pixhawk below the control stack; still runnable standalone)
- `motor_test.py`, `motor_trim.py`, `dropper.py`, `dropper_test.py`,
  `servo_watch.py`, `boot_check.py`, `test_pixhawk.py`,
  `check_horizontal_direction.py`, `check_vertical_direction.py`, `qua.sh`.

## Older

- `gate_runner.py` — legacy gate script: opens its own MAVLink connection,
  flies STABILIZE with a constant down-thrust depth strategy, predates the
  Bar02 sanity checks and flight-mode lessons. Do not run at competition.
