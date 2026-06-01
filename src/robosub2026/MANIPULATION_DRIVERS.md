# Manipulation drivers — design + pseudocode

Today the BT actions `ReleaseMarker`, `LaunchTorpedo`, `ReleaseObject`, and
`ActivateTool` only log + update blackboard counters. This doc is the spec
for the four real drivers that need to land before pool. Each driver is a
small ROS 2 node that exposes a `std_srvs/Trigger` (or custom) service; the
BT action calls the service in `onStart()` and reports SUCCESS/FAILURE based
on the response.

The hardware path is **Pixhawk MAIN/AUX PWM rails via ArduSub
RC_OVERRIDE/SERVO_OUTPUT_RAW** for solenoids and servos, with optional
relay-board GPIO via libgpiod for actuators that draw too much current for
the Pixhawk rails directly. All four follow the same pattern, so they can
share a `BaseActuatorDriver` base class.

---

## Common pattern (all four)

```
node: <name>_driver
  parameters:
    pixhawk_serial_port   string   ''            # if empty, share via mavros / mavlink-router UDP
    pixhawk_udp_endpoint  string   ''
    channel               int      <N>           # ArduSub SERVO_N or relay GPIO line
    pwm_fire_us           int      1900          # solenoid energize / servo "open"
    pwm_idle_us           int      1500          # neutral / "closed"
    fire_duration_ms      int      300           # how long to hold pwm_fire_us
    cooldown_ms           int      500           # min time between calls (debounce)
    max_uses              int      <N>           # hard cap (markers: 2, torpedoes: 2)

  services:
    /actuator/<name>/fire    std_srvs/Trigger   -> trigger one shot

  state:
    uses_remaining: int = max_uses
    last_fire_t:    Time
    armed:          bool = True   # safety latch; flips False on E-stop

  on /actuator/<name>/fire request:
    if not armed or uses_remaining <= 0 or (now - last_fire_t) < cooldown_ms:
        return Trigger.Response(success=False, message=<reason>)

    # 1. Send pwm_fire_us on `channel` via MAV_CMD_DO_SET_SERVO
    mavlink.command_long_send(target_system, target_component,
        MAV_CMD_DO_SET_SERVO, 0, channel, pwm_fire_us, 0,0,0,0,0)

    # 2. Hold for fire_duration_ms
    sleep(fire_duration_ms / 1000.0)

    # 3. Return to neutral
    mavlink.command_long_send(target_system, target_component,
        MAV_CMD_DO_SET_SERVO, 0, channel, pwm_idle_us, 0,0,0,0,0)

    # 4. Update local state
    uses_remaining -= 1
    last_fire_t = now
    return Trigger.Response(success=True,
                            message=f'{name} fired ({uses_remaining} left)')
```

The BT-side actions call this service:

```cpp
class ReleaseMarker : public BT::StatefulActionNode {
  // onStart(): create client lazily if needed; send async Trigger request
  // onRunning(): poll future; on response -> SUCCESS or FAILURE
  //   on SUCCESS, decrement blackboard markers_remaining and set marker_dropped=true
  // onHalted(): cancel future (best-effort — actuators are short, usually done)
};
```

---

## 1. Marker dropper (`ReleaseMarker`)

**Hardware**: two BRUVS markers in a sprung tube; release by retracting a
servo arm on Pixhawk AUX1. Markers fall under gravity.

**Driver**: `marker_dropper_driver`
```
channel           = AUX1   (Pixhawk SERVO_9)
pwm_idle_us       = 1100   (arm closed)
pwm_fire_us       = 1900   (arm retracted, marker drops)
fire_duration_ms  = 600    (give marker time to clear)
cooldown_ms       = 1500   (let it physically reset)
max_uses          = 2

extra:
  # second marker uses a second mechanical position;
  # increment shot_index so successive fires use different channels:
  on fire:
    chan = AUX1 if shot_index == 0 else AUX2
    do_set_servo(chan, pwm_fire_us); sleep; do_set_servo(chan, pwm_idle_us)
    shot_index = min(shot_index + 1, max_uses)
```

**BT side** (`ReleaseMarker::tick()` — currently a stub):
```
on tick:
  if MissionIO::callTriggerSync("/actuator/marker_dropper/fire", 2.0s) succeeds:
      bb["markers_remaining"] -= 1
      bb["marker_dropped"]  = true
      return SUCCESS
  else:
      return FAILURE  # parent retries via DropMarker subtree's fallback
```

---

## 2. Torpedo launcher (`LaunchTorpedo`)

**Hardware**: two CO₂-driven or sprung-tube torpedo launchers, each fired by
a separate solenoid on Pixhawk AUX3 / AUX4. The `ArmLauncher` BT action
selects which tube; `LaunchTorpedo` actually fires.

**Driver**: `torpedo_launcher_driver`
```
channel_tube_1    = AUX3   (Pixhawk SERVO_11)
channel_tube_2    = AUX4   (Pixhawk SERVO_12)
pwm_idle_us       = 1100   (solenoid de-energized)
pwm_fire_us       = 1900   (solenoid energized for 200 ms)
fire_duration_ms  = 250
cooldown_ms       = 2000
max_uses          = 2

services:
  /actuator/torpedo/arm    std_srvs/SetBool   payload.data = tube_id (true=tube1, false=tube2)
  /actuator/torpedo/fire   std_srvs/Trigger   fires the currently armed tube

state:
  armed_tube: int|None = None

on arm:
  armed_tube = 1 if data else 2
  return Trigger.Response(success=True)

on fire:
  if armed_tube is None or uses_remaining <= 0:
      return Trigger.Response(success=False, message='not armed')
  chan = channel_tube_1 if armed_tube == 1 else channel_tube_2
  do_set_servo(chan, pwm_fire_us); sleep; do_set_servo(chan, pwm_idle_us)
  uses_remaining -= 1
  fired_tube = armed_tube
  armed_tube = None       # require explicit re-arm before next shot
  return Trigger.Response(success=True, message=f'tube {fired_tube} fired')
```

**BT side** (`ArmLauncher`, `LaunchTorpedo`):
```
ArmLauncher::tick():
  tube = getInput("tube_id")
  call /actuator/torpedo/arm with data=(tube == 1) → SUCCESS/FAILURE

LaunchTorpedo::tick():
  call /actuator/torpedo/fire:
    on success:
      bb["torpedoes_remaining"] -= 1
      bb["torpedo_fired"] = true
      bb["torpedo_hit"]   = true   # perception override flips this back if shot missed
      return SUCCESS
    on failure:
      return FAILURE  # FireTorpedo subtree's fallback handles retry
```

---

## 3. Gripper (`ReleaseObject` + a future `GrabObject`)

**Hardware**: Blue Robotics Newton subsea gripper (or similar), 4-wire
position-controlled servo on Pixhawk AUX5. Holding-current is non-trivial,
so we drive via the Pixhawk's PWM rail with the gripper's own ESC.

**Driver**: `gripper_driver`
```
channel           = AUX5   (Pixhawk SERVO_13)
pwm_open_us       = 1100   (gripper fully open)
pwm_closed_us     = 1900   (gripper closed — clamping force)
fire_duration_ms  = 0      (latch at the target PWM; no auto-release)
cooldown_ms       = 200

services:
  /actuator/gripper/open    std_srvs/Trigger   -> set pwm_open_us
  /actuator/gripper/close   std_srvs/Trigger   -> set pwm_closed_us

state:
  position: str = 'open'   # 'open' | 'closed'

on open:
  do_set_servo(channel, pwm_open_us)
  position = 'open'
  return Trigger.Response(success=True)

on close:
  do_set_servo(channel, pwm_closed_us)
  position = 'closed'
  return Trigger.Response(success=True)
```

**BT side** (`ReleaseObject` exists; `GrabObject` is not in the 2026 tree
yet — add to `shrub_nodes.hpp` + register if needed):
```
ReleaseObject::tick():
  call /actuator/gripper/open → SUCCESS
    bb["objects_delivered"] += 1
    bb["object_delivered"] = true
    return SUCCESS

GrabObject::tick():        # future
  call /actuator/gripper/close → SUCCESS
    return SUCCESS
```

---

## 4. Magnetic interaction tool (`ActivateTool`)

**Hardware**: high-current electromagnet (~5 A @ 12 V) on a 12 V GPIO-driven
relay board (the Pixhawk PWM rail can't source the current). We drive a
relay via the Jetson's GPIO using libgpiod — no MAVLink involved.

**Driver**: `magnetic_tool_driver`
```
gpio_chip         = '/dev/gpiochip0'
gpio_line         = 18         # BCM18 on Jetson 40-pin header
active_high       = True
energize_ms       = 1500       # hold magnet on long enough to latch the target
cooldown_ms       = 1000

import gpiod
chip = gpiod.Chip(gpio_chip)
line = chip.get_line(gpio_line)
line.request(consumer='shrub-magnet', type=gpiod.LINE_REQ_DIR_OUT)

services:
  /actuator/magnet/energize  std_srvs/Trigger

on energize:
  if (now - last_fire_t) < cooldown_ms:
      return Trigger.Response(success=False, message='cooldown')
  line.set_value(1 if active_high else 0)
  sleep(energize_ms / 1000.0)
  line.set_value(0 if active_high else 1)
  last_fire_t = now
  return Trigger.Response(success=True)
```

**BT side** (`ActivateTool`):
```
ActivateTool::tick():
  call /actuator/magnet/energize:
    on success:
      bb["light_off"] = true   # mission flag set when magnet has interacted
      return SUCCESS
    on failure (cooldown):
      return RUNNING           # quick retry next tick; the MagneticInteraction
                               # subtree's fallback handles persistent failure
```

---

## Service wiring on the C++ side

Today `MissionIO` only owns publishers / subscribers. Add a thin sync helper:

```cpp
// mission_io.hpp
bool callTrigger(const std::string& service_name, double timeout_s);

// mission_io.cpp
bool MissionIO::callTrigger(const std::string& service_name, double timeout_s) {
  auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_s))) {
    RCLCPP_WARN(node_->get_logger(), "service %s not available", service_name.c_str());
    return false;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto fut = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, fut,
        std::chrono::duration<double>(timeout_s)) != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  return fut.get()->success;
}
```

(Caveat: this is a blocking call. If you don't want to stall the BT tick,
make the action `StatefulActionNode` and poll the future across `onRunning()`
calls instead.)

---

## Ordering for landing the drivers

1. **Marker dropper** — simplest, cheap to test on the bench, biggest scoring
   leverage (drives `DropMarker` subtree which retries on failure).
2. **Gripper** — needed for the octagon's `ReleaseObject` chain (pickup +
   deliver, 2×).
3. **Torpedo launcher** — two-stage (arm + fire), needs the most safety
   plumbing.
4. **Magnetic tool** — purely Jetson-side GPIO, no Pixhawk coordination, but
   blocked on the relay board being wired.
