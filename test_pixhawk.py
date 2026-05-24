#!/usr/bin/env python3
"""
Master Pixhawk Test Script
===========================
Tests everything end-to-end: serial connection, heartbeat, sensor data,
arming/disarming, all thruster axes, and MAVLink message integrity.

Usage:
    python3 test_pixhawk.py                  # run all tests
    python3 test_pixhawk.py --skip-thrust    # skip live thruster tests
    python3 test_pixhawk.py --port /dev/ttyACM1 --baud 115200

SAFETY: Thruster tests use LOW power (20%) for SHORT durations (1.5s each).
        Keep the sub secured or in water before running with thrusters enabled.
"""

import argparse
import sys
import time
import math

try:
    from pymavlink import mavutil
except ImportError:
    print("[FATAL] pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)


# ─── ANSI colors ────────────────────────────────────────────────────────────
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

passed = 0
failed = 0
warnings = 0


def log_pass(msg):
    global passed
    passed += 1
    print(f"  {GREEN}[PASS]{RESET} {msg}")


def log_fail(msg):
    global failed
    failed += 1
    print(f"  {RED}[FAIL]{RESET} {msg}")


def log_warn(msg):
    global warnings
    warnings += 1
    print(f"  {YELLOW}[WARN]{RESET} {msg}")


def log_info(msg):
    print(f"  {CYAN}[INFO]{RESET} {msg}")


def section(title):
    print(f"\n{BOLD}{'='*60}{RESET}")
    print(f"{BOLD}  {title}{RESET}")
    print(f"{BOLD}{'='*60}{RESET}")


# ─── Test 1: Serial Connection ──────────────────────────────────────────────

def test_connection(port, baud):
    section("1. SERIAL CONNECTION & HEARTBEAT")
    master = None
    try:
        log_info(f"Connecting to {port} @ {baud} baud...")
        master = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        log_fail(f"Could not open serial port: {e}")
        return None

    log_pass(f"Serial port {port} opened")

    log_info("Waiting for heartbeat (timeout 10s)...")
    hb = master.wait_heartbeat(timeout=10)
    if hb is None:
        log_fail("No heartbeat received — Pixhawk not responding")
        return None

    log_pass(f"Heartbeat received — sysid={master.target_system}, "
             f"compid={master.target_component}")

    autopilot_map = {
        3: "ArduPilot",
        12: "PX4",
    }
    vehicle_map = {
        2: "Quadrotor",
        12: "Submarine (ArduSub)",
    }

    autopilot = autopilot_map.get(hb.autopilot, f"Unknown ({hb.autopilot})")
    vehicle = vehicle_map.get(hb.type, f"Unknown ({hb.type})")

    log_info(f"Autopilot: {autopilot}")
    log_info(f"Vehicle type: {vehicle}")

    if hb.type == 12:
        log_pass("Vehicle is ArduSub — correct for AUV")
    else:
        log_warn(f"Vehicle type is {vehicle}, expected Submarine (12)")

    return master


# ─── Test 2: Firmware / Parameter Check ─────────────────────────────────────

def test_firmware(master):
    section("2. FIRMWARE & PARAMETERS")

    master.mav.param_request_list_send(
        master.target_system, master.target_component)

    params = {}
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is None:
            break
        params[msg.param_id] = msg.param_value

    if len(params) == 0:
        log_fail("No parameters received from Pixhawk")
        return

    log_pass(f"Received {len(params)} parameters")

    important_params = {
        'ARMING_CHECK': ('Arming checks bitmask', None),
        'FS_GCS_ENABLE': ('GCS failsafe', None),
        'MOT_1_DIRECTION': ('Motor 1 direction', None),
        'PILOT_SPEED_UP': ('Pilot climb speed', None),
        'BRD_SAFETYENABLE': ('Safety switch', None),
    }

    for param, (desc, _) in important_params.items():
        clean_param = param.ljust(16, '\x00')[:16]
        found = False
        for p_id, p_val in params.items():
            if p_id.rstrip('\x00') == param:
                log_info(f"{param} = {p_val}  ({desc})")
                found = True
                break
        if not found:
            log_info(f"{param} — not found (may not apply to this firmware)")


# ─── Test 3: Sensor Data ────────────────────────────────────────────────────

def test_sensors(master):
    section("3. SENSOR DATA STREAMS")

    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    time.sleep(1)

    sensor_checks = {
        'SCALED_PRESSURE2': {'desc': 'Pressure/depth sensor', 'found': False, 'data': None},
        'ATTITUDE': {'desc': 'IMU attitude (roll/pitch/yaw)', 'found': False, 'data': None},
        'VFR_HUD': {'desc': 'HUD data (heading/speed/alt)', 'found': False, 'data': None},
        'SYS_STATUS': {'desc': 'System status & voltage', 'found': False, 'data': None},
        'RAW_IMU': {'desc': 'Raw IMU accel/gyro/mag', 'found': False, 'data': None},
        'GPS_RAW_INT': {'desc': 'GPS status', 'found': False, 'data': None},
    }

    deadline = time.time() + 5
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype in sensor_checks and not sensor_checks[mtype]['found']:
            sensor_checks[mtype]['found'] = True
            sensor_checks[mtype]['data'] = msg

    for mtype, info in sensor_checks.items():
        if info['found']:
            log_pass(f"{info['desc']} ({mtype})")
            msg = info['data']
            if mtype == 'ATTITUDE':
                log_info(f"  Roll={math.degrees(msg.roll):.1f}° "
                         f"Pitch={math.degrees(msg.pitch):.1f}° "
                         f"Yaw={math.degrees(msg.yaw):.1f}°")
            elif mtype == 'VFR_HUD':
                log_info(f"  Heading={msg.heading}° "
                         f"Groundspeed={msg.groundspeed:.2f} m/s "
                         f"Alt={msg.alt:.2f} m")
            elif mtype == 'SCALED_PRESSURE2':
                log_info(f"  Pressure={msg.press_abs:.1f} hPa "
                         f"Temp={msg.temperature / 100.0:.1f}°C")
            elif mtype == 'SYS_STATUS':
                voltage = msg.voltage_battery / 1000.0
                log_info(f"  Battery={voltage:.2f}V  "
                         f"Current={msg.current_battery / 100.0:.1f}A")
                if voltage > 0 and voltage < 10.0:
                    log_warn(f"Battery voltage low: {voltage:.2f}V")
                elif voltage >= 10.0:
                    log_pass(f"Battery voltage OK: {voltage:.2f}V")
            elif mtype == 'RAW_IMU':
                log_info(f"  Accel: x={msg.xacc} y={msg.yacc} z={msg.zacc}")
                log_info(f"  Gyro:  x={msg.xgyro} y={msg.ygyro} z={msg.zgyro}")
        else:
            if mtype == 'GPS_RAW_INT':
                log_info(f"{info['desc']} ({mtype}) — not found (normal for underwater)")
            else:
                log_fail(f"{info['desc']} ({mtype}) — NO DATA")


# ─── Test 4: Mode Switching ─────────────────────────────────────────────────

def test_modes(master):
    section("4. MODE SWITCHING")

    try:
        mode_map = master.mode_mapping()
        log_pass(f"Mode mapping available — {len(mode_map)} modes")
        log_info(f"Available modes: {', '.join(sorted(mode_map.keys()))}")
    except Exception as e:
        log_fail(f"Cannot get mode mapping: {e}")
        return

    for mode_name in ['MANUAL', 'STABILIZE', 'ALT_HOLD']:
        if mode_name not in mode_map:
            log_warn(f"Mode {mode_name} not available in firmware")
            continue

        mode_id = mode_map[mode_name]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.result == 0:
            log_pass(f"Set mode {mode_name} (id={mode_id}) — ACK OK")
        elif ack:
            log_warn(f"Set mode {mode_name} — ACK result={ack.result}")
        else:
            log_fail(f"Set mode {mode_name} — no ACK received")

        time.sleep(0.5)

    # Return to MANUAL for thruster tests
    if 'MANUAL' in mode_map:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_map['MANUAL'])
        master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)


# ─── Test 5: Arm / Disarm ──────────────────────────────────────────────────

def test_arm_disarm(master):
    section("5. ARM / DISARM")

    # Arm
    log_info("Sending ARM command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)

    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    armed = False
    if ack:
        if ack.result == 0:
            log_pass("ARM command accepted")
            armed = True
        else:
            log_fail(f"ARM rejected — result={ack.result} "
                     "(check pre-arm: accelerometer calibration, "
                     "battery voltage, safety switch)")
    else:
        log_fail("ARM command — no ACK received")

    if armed:
        time.sleep(1)

        # Verify armed state via heartbeat
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb:
            is_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if is_armed:
                log_pass("Vehicle confirmed ARMED via heartbeat")
            else:
                log_fail("ARM ACK was OK but heartbeat says DISARMED")
                armed = False

    # Disarm
    log_info("Sending DISARM command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)

    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == 0:
        log_pass("DISARM command accepted")
    elif ack:
        log_warn(f"DISARM result={ack.result}")
    else:
        log_fail("DISARM command — no ACK received")

    return armed


# ─── Test 6: Thruster Axes ──────────────────────────────────────────────────

def test_thrusters(master, arm_works):
    section("6. THRUSTER AXIS TEST (low power)")

    if not arm_works:
        log_warn("Skipping thruster tests — ARM failed in test 5")
        return

    print(f"\n  {YELLOW}*** SAFETY WARNING ***{RESET}")
    print(f"  {YELLOW}Thrusters will spin at ~20% power for 1.5 seconds each.{RESET}")
    print(f"  {YELLOW}Ensure the sub is secured or in water.{RESET}")

    try:
        response = input(f"\n  {BOLD}Proceed with thruster tests? [y/N]: {RESET}").strip().lower()
    except EOFError:
        response = 'n'

    if response != 'y':
        log_info("Thruster tests skipped by user")
        return

    # Re-arm for thruster tests
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 19)
    master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    time.sleep(0.3)

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if not ack or ack.result != 0:
        log_fail("Could not re-arm for thruster tests")
        return

    time.sleep(1)
    log_info("Armed — starting thruster axis tests\n")

    # Test each axis: (name, x, y, z, r)
    axis_tests = [
        ("Surge FORWARD   (x=+200)",   200,    0,  500,    0),
        ("Surge BACKWARD  (x=-200)",  -200,    0,  500,    0),
        ("Strafe RIGHT    (y=+200)",     0,  200,  500,    0),
        ("Strafe LEFT     (y=-200)",     0, -200,  500,    0),
        ("Descend         (z=300)",      0,    0,  300,    0),
        ("Ascend          (z=700)",      0,    0,  700,    0),
        ("Yaw CW          (r=+200)",     0,    0,  500,  200),
        ("Yaw CCW         (r=-200)",     0,    0,  500, -200),
    ]

    for name, x, y, z, r in axis_tests:
        log_info(f"Testing: {name}")

        try:
            # Send at 10 Hz for 1.5 seconds
            for _ in range(15):
                master.mav.manual_control_send(
                    master.target_system, x=x, y=y, z=z, r=r, buttons=0)
                # Also send heartbeat to prevent GCS failsafe
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                time.sleep(0.1)

            # Return to neutral
            for _ in range(5):
                master.mav.manual_control_send(
                    master.target_system, x=0, y=0, z=500, r=0, buttons=0)
                time.sleep(0.05)

            log_pass(f"{name}")
        except Exception as e:
            log_fail(f"{name} — error: {e}")

        time.sleep(0.5)

    # Disarm after thruster tests
    log_info("Disarming after thruster tests...")
    for _ in range(5):
        master.mav.manual_control_send(
            master.target_system, x=0, y=0, z=500, r=0, buttons=0)
        time.sleep(0.05)

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    log_pass("Disarmed after thruster tests")


# ─── Test 7: MAVLink Message Integrity ──────────────────────────────────────

def test_message_rate(master):
    section("7. MAVLINK MESSAGE RATE & INTEGRITY")

    counts = {}
    start = time.time()
    duration = 3.0

    while time.time() - start < duration:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        counts[mtype] = counts.get(mtype, 0) + 1

    total = sum(counts.values())
    rate = total / duration

    if total == 0:
        log_fail("No messages received in 3 seconds")
        return

    log_pass(f"Received {total} messages in {duration}s ({rate:.1f} msg/s)")

    important = ['HEARTBEAT', 'ATTITUDE', 'VFR_HUD', 'SCALED_PRESSURE2',
                 'SYS_STATUS', 'RAW_IMU']

    log_info("Message rates:")
    for mtype in sorted(counts, key=counts.get, reverse=True):
        hz = counts[mtype] / duration
        marker = " *" if mtype in important else ""
        print(f"       {mtype:30s} {counts[mtype]:4d} ({hz:.1f} Hz){marker}")

    # Check heartbeat rate
    hb_count = counts.get('HEARTBEAT', 0)
    hb_hz = hb_count / duration
    if hb_hz >= 0.5:
        log_pass(f"Heartbeat rate OK: {hb_hz:.1f} Hz")
    else:
        log_fail(f"Heartbeat rate too low: {hb_hz:.1f} Hz (expected ~1 Hz)")


# ─── Test 8: Pre-arm Checks ────────────────────────────────────────────────

def test_prearm(master):
    section("8. PRE-ARM & SAFETY STATUS")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
        0, 0, 0, 0, 0, 0, 0, 0)

    # Collect STATUSTEXT messages for pre-arm feedback
    prearm_msgs = []
    deadline = time.time() + 3
    while time.time() < deadline:
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
        if msg is None:
            continue
        text = msg.text.rstrip('\x00')
        prearm_msgs.append((msg.severity, text))

    if prearm_msgs:
        for severity, text in prearm_msgs:
            if severity <= 3:
                log_warn(f"[sev={severity}] {text}")
            else:
                log_info(f"[sev={severity}] {text}")
    else:
        log_info("No pre-arm status messages (may already be passing)")

    # Check SYS_STATUS for sensor health
    msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=3)
    if msg:
        present = msg.onboard_control_sensors_present
        enabled = msg.onboard_control_sensors_enabled
        health = msg.onboard_control_sensors_health

        unhealthy = enabled & ~health
        if unhealthy == 0:
            log_pass("All enabled sensors are healthy")
        else:
            log_warn(f"Unhealthy sensor bitmask: 0x{unhealthy:08X}")

        log_info(f"CPU load: {msg.load / 10.0:.1f}%")
        log_info(f"Comm drop rate: {msg.drop_rate_comm / 100.0:.1f}%")
        if msg.drop_rate_comm > 500:
            log_warn("High communication drop rate")
    else:
        log_warn("Could not read SYS_STATUS")


# ─── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Master Pixhawk test script — validates everything end-to-end")
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--skip-thrust', action='store_true',
                        help='Skip live thruster tests')
    args = parser.parse_args()

    print(f"\n{BOLD}{'#'*60}{RESET}")
    print(f"{BOLD}#   PIXHAWK MASTER TEST — {time.strftime('%Y-%m-%d %H:%M:%S')}{RESET}")
    print(f"{BOLD}#   Port: {args.port}  Baud: {args.baud}{RESET}")
    print(f"{BOLD}{'#'*60}{RESET}")

    # Test 1: Connection
    master = test_connection(args.port, args.baud)
    if master is None:
        print(f"\n{RED}{BOLD}ABORTED — cannot connect to Pixhawk.{RESET}")
        print(f"  Check: USB cable, port ({args.port}), baud rate ({args.baud})")
        print(f"  Try:   ls /dev/ttyACM* /dev/ttyUSB*")
        sys.exit(1)

    # Test 2: Firmware
    test_firmware(master)

    # Test 3: Sensors
    test_sensors(master)

    # Test 4: Modes
    test_modes(master)

    # Test 5: Arm/Disarm
    arm_works = test_arm_disarm(master)

    # Test 6: Thrusters (optional)
    if args.skip_thrust:
        section("6. THRUSTER AXIS TEST")
        log_info("Skipped via --skip-thrust flag")
    else:
        test_thrusters(master, arm_works)

    # Test 7: Message rates
    test_message_rate(master)

    # Test 8: Pre-arm checks
    test_prearm(master)

    # ─── Summary ────────────────────────────────────────────────────
    section("RESULTS SUMMARY")
    total = passed + failed
    print(f"  Tests passed:  {GREEN}{passed}{RESET}")
    print(f"  Tests failed:  {RED}{failed}{RESET}")
    print(f"  Warnings:      {YELLOW}{warnings}{RESET}")
    print()

    if failed == 0:
        print(f"  {GREEN}{BOLD}ALL TESTS PASSED{RESET}")
    else:
        print(f"  {RED}{BOLD}{failed} TEST(S) FAILED — review output above{RESET}")

    # Clean up
    try:
        master.close()
    except Exception:
        pass

    print()
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
