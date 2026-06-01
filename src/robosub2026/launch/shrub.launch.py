"""SHRUB v4 — launch file for the RoboSub 2026 'Restore and Recovery' mission."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bt_xml', default_value='robosub2026_mission.xml',
                              description='BT XML filename'),
        DeclareLaunchArgument('tree_id', default_value='MainTree',
                              description='Root tree ID to execute'),
        DeclareLaunchArgument('coin_flip', default_value='normal',
                              description='Coin flip start orientation: normal | backward'),
        DeclareLaunchArgument('role', default_value='survey_repair',
                              description='Role: survey_repair | search_rescue'),
        DeclareLaunchArgument('gate_red_side', default_value='right',
                              description='Side of the red divider at the gate: right | left'),
        DeclareLaunchArgument('run_mode', default_value='semifinal',
                              description='Run mode: semifinal | final | qualification'),
        DeclareLaunchArgument('style_enabled', default_value='true',
                              description='Attempt style points through gate'),
        DeclareLaunchArgument('tick_rate_ms', default_value='50',
                              description='BT tick period in ms'),

        Node(
            package='bt_mission',
            executable='bt_executor',
            name='shrub_executor',
            output='screen',
            parameters=[{
                'bt_xml': LaunchConfiguration('bt_xml'),
                'tree_id': LaunchConfiguration('tree_id'),
                'coin_flip': LaunchConfiguration('coin_flip'),
                'role': LaunchConfiguration('role'),
                'gate_red_side': LaunchConfiguration('gate_red_side'),
                'run_mode': LaunchConfiguration('run_mode'),
                'style_enabled': LaunchConfiguration('style_enabled'),
                'tick_rate_ms': LaunchConfiguration('tick_rate_ms'),
            }],
        ),
    ])
