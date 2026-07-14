"""One-command bringup for the ZED IMU orientation visualization.

Order: pre-kill any stale zed_wrapper node (single-owner + fresh fused
orientation, no cross-run accumulation) -> fresh ZED camera -> orientation /
diagnostics / marker nodes -> RViz. Set start_zed:=false to attach to an
already-running ZED and skip the pre-kill.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    TimerAction, GroupAction, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_number = LaunchConfiguration('serial_number')
    camera_name = LaunchConfiguration('camera_name')
    use_rviz = LaunchConfiguration('rviz')
    start_zed = LaunchConfiguration('start_zed')

    imu_share = get_package_share_directory('imu')
    rviz_cfg = os.path.join(imu_share, 'rviz', 'imu.rviz')

    # ---- pre-kill any running ZED node (best-effort; ok if none) ----
    prekill = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f zed_wrapper || true; sleep 3'],
        condition=IfCondition(start_zed),
        output='screen')

    # ---- fresh ZED camera (started after the settle) ----
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': camera_name,
            'serial_number': serial_number,
        }.items())
    zed_group = GroupAction(
        actions=[TimerAction(period=4.0, actions=[zed_launch])],
        condition=IfCondition(start_zed))

    orientation = Node(
        package='imu', executable='orientation_node',
        name='orientation_node', output='screen')
    diagnostics = Node(
        package='imu', executable='diagnostics_node',
        name='diagnostics_node', output='screen')
    markers = Node(
        package='imu', executable='marker_node',
        name='marker_node', output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument('serial_number', default_value='0',
                              description='ZED serial to select one of two cameras; 0 = first available'),
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_zed', default_value='true',
                              description='false = attach to a running ZED, skip pre-kill'),
        LogInfo(msg='=== ZED IMU orientation viz starting ==='),
        prekill,
        zed_group,
        orientation,
        diagnostics,
        markers,
        rviz,
    ])
