# `mission` — LEGACY mission planner (BehaviorTree.CPP v3)

> **Status: legacy, but currently the only fully working mission brain.**

This package (`main.cpp`, executable `bt_runner`) is the original
vision-integrated behavior tree. It subscribes to `vision/detections`,
`depth/info`, `localization/pose` and publishes `movement_command` /
`navigation_command`, closing the loop on detected objects.

**It is still what `src/run_stack.sh` launches**, because the replacement —
`src/robosub2026/` (`bt_mission`, SHRUB v4, BehaviorTree.CPP v4) — has not yet
been fully ported and pool-verified.

Going forward, **new mission work happens in `bt_mission`**, not here. See
`src/robosub2026/MIGRATION.md` for the port plan. This package will be deleted
once SHRUB v4 drives the sub through the gate in the test tank.

Do not start new features in this package.
