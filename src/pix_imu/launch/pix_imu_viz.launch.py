"""One-command bringup for the Pixhawk (ArduSub) IMU orientation viz.

Order: pixhawk_imu_bridge (MAVLink ATTITUDE/RAW_IMU -> sensor_msgs/Imu) ->
the imu package's generic orientation / diagnostics / marker nodes, pointed
at /pixhawk/imu/data instead of the ZED -> RViz. No ZED involved.

Set rviz:=false for headless. Override port:=/dev/ttyACM1 if the FC enumerates
elsewhere.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    use_rviz = LaunchConfiguration('rviz')

    imu_topic = '/pixhawk/imu/data'

    # RViz config is shared with the imu package (same frames/displays).
    imu_share = get_package_share_directory('imu')
    rviz_cfg = os.path.join(imu_share, 'rviz', 'imu.rviz')

    bridge = Node(
        package='pix_imu', executable='pixhawk_imu_bridge',
        name='pixhawk_imu_bridge', output='screen',
        parameters=[{'port': port, 'baud': baud, 'imu_topic': imu_topic}])

    orientation = Node(
        package='imu', executable='orientation_node',
        name='orientation_node', output='screen',
        parameters=[{'imu_topic': imu_topic}])
    diagnostics = Node(
        package='imu', executable='diagnostics_node',
        name='diagnostics_node', output='screen',
        parameters=[{'imu_topic': imu_topic}])
    markers = Node(
        package='imu', executable='marker_node',
        name='marker_node', output='screen',
        parameters=[{'imu_topic': imu_topic}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Pixhawk flight-controller serial'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('rviz', default_value='true'),
        LogInfo(msg='=== Pixhawk IMU orientation viz starting ==='),
        bridge,
        orientation,
        diagnostics,
        markers,
        rviz,
    ])
