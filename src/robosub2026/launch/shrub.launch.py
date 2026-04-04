"""SHRUB v3 — ROS 2 launch file for RoboSub 2026 mission."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bt_xml', default_value='robosub2026_mission.xml',
                              description='BT XML filename'),
        DeclareLaunchArgument('tree_id', default_value='SHRUB',
                              description='Tree ID to execute'),
        DeclareLaunchArgument('coin_flip', default_value='none',
                              description='Coin flip result: heads/tails/none'),
        DeclareLaunchArgument('run_mode', default_value='semifinal',
                              description='Run mode: semifinal/final'),
        DeclareLaunchArgument('tick_rate_ms', default_value='50',
                              description='BT tick rate in ms'),

        Node(
            package='bt_mission',
            executable='bt_executor',
            name='shrub_executor',
            output='screen',
            parameters=[{
                'bt_xml': LaunchConfiguration('bt_xml'),
                'tree_id': LaunchConfiguration('tree_id'),
                'coin_flip': LaunchConfiguration('coin_flip'),
                'run_mode': LaunchConfiguration('run_mode'),
                'tick_rate_ms': LaunchConfiguration('tick_rate_ms'),
            }],
        ),
    ])
