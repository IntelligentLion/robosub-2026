"""mission.launch.py — one integrated bring-up for the SHRUB v4 BT mission
(audit F14: shrub.launch.py used to start bt_executor alone, and everything
the tree's nav()/perception/manipulation calls depend on — thruster_node,
detector, safety_monitor, depth_node, localization, autonomous_controller —
had to be started by hand in the right order).

    thruster_node          MAVLink gateway: sole owner of the serial port.
                            simulate:=false explicitly — this launch is for
                            real runs, not bench dry-runs.
    safety_monitor_node    battery / leak — simulate:=false explicitly too.
    detector                vision/detections (ZED-fed).
    depth_node              depth/info, consumed by MissionIO + BT safety checks.
    localization_node       localization/pose, consumed by MissionIO + nav_nodes.
    autonomous_controller   navigation_command → the nav() BT action's target.
    bt_executor              the mission tree itself, started last.

DO NOT also run pix_imu/pixhawk_imu_bridge or any other node that opens the
same serial port as thruster_node — two readers on one port stall both
(single-serial-reader rule).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    flight_mode = LaunchConfiguration('flight_mode')
    safety_simulate = LaunchConfiguration('safety_simulate')

    bt_xml = LaunchConfiguration('bt_xml')
    tree_id = LaunchConfiguration('tree_id')
    coin_flip = LaunchConfiguration('coin_flip')
    role = LaunchConfiguration('role')
    gate_red_side = LaunchConfiguration('gate_red_side')
    run_mode = LaunchConfiguration('run_mode')
    style_enabled = LaunchConfiguration('style_enabled')
    tick_rate_ms = LaunchConfiguration('tick_rate_ms')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('flight_mode', default_value='ALT_HOLD'),
        # safety_monitor_node's own docstring: it must either be the *sole*
        # MAVLink owner or consume via a UDP forward from thruster_node's
        # link — opening the same serial port from both processes violates
        # the single-serial-reader rule. No UDP bridge (mavlink-router) is
        # set up in this repo, so default to simulate (honest 100%/no-leak)
        # until one exists; override once a bridge is running.
        DeclareLaunchArgument('safety_simulate', default_value='true'),

        DeclareLaunchArgument('bt_xml', default_value='robosub2026_mission.xml'),
        DeclareLaunchArgument('tree_id', default_value='MainTree'),
        DeclareLaunchArgument('coin_flip', default_value='normal'),
        DeclareLaunchArgument('role', default_value='survey_repair'),
        DeclareLaunchArgument('gate_red_side', default_value='right'),
        DeclareLaunchArgument('run_mode', default_value='semifinal'),
        DeclareLaunchArgument('style_enabled', default_value='true'),
        DeclareLaunchArgument('tick_rate_ms', default_value='50'),

        Node(
            package='mavlink_thruster_control',
            executable='thruster_node',
            name='thruster_controller',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'simulate': False,
                'flight_mode': flight_mode,
            }],
        ),
        Node(
            package='mavlink_thruster_control',
            executable='safety_monitor_node',
            name='safety_monitor',
            output='screen',
            parameters=[{
                'simulate': safety_simulate,
            }],
        ),
        Node(
            package='vision',
            executable='detector',
            name='detector',
            output='screen',
        ),
        Node(
            package='localization',
            executable='depth_node',
            name='depth_node',
            output='screen',
        ),
        Node(
            package='localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
        ),
        Node(
            package='control',
            executable='autonomous_controller',
            name='autonomous_controller',
            output='screen',
        ),

        Node(
            package='bt_mission',
            executable='bt_executor',
            name='shrub_executor',
            output='screen',
            parameters=[{
                'bt_xml': bt_xml,
                'tree_id': tree_id,
                'coin_flip': coin_flip,
                'role': role,
                'gate_red_side': gate_red_side,
                'run_mode': run_mode,
                'style_enabled': style_enabled,
                'tick_rate_ms': tick_rate_ms,
            }],
        ),
    ])
