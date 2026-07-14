"""submerge_hold — dive to depth, hold depth/heading/attitude, drive forward.

  thruster_node      MAVLink gateway: sole owner of /dev/ttyACM0. Publishes
                     pixhawk/{imu/data,depth,mode,armed}; serves
                     pixhawk/{set_mode,preflight}.
  orientation_node   pixhawk/imu/data → imu/rpy + TF (reused from the imu pkg).
  motion_node        THE movement node: sole publisher of movement_command.
  rviz_visualizer    subscribe-only markers/path/TF. Set viz:=false to omit.

DO NOT run pix_imu/pixhawk_imu_bridge alongside this. It opens the same serial
port, and two readers on one port produce the "device reports readiness to read
but returned no data" stall that kills both. The bridge exists for dry-bench /
IMU-only work, where thruster_node is NOT running. This launch replaces it.

Drive it with:
    ros2 topic pub --once /motion/submerge std_msgs/Float32 '{data: 2.0}'
or, from Python, `control.api.Auv`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    viz = LaunchConfiguration('viz')
    rviz = LaunchConfiguration('rviz')
    serial_port = LaunchConfiguration('serial_port')
    target_depth = LaunchConfiguration('target_depth')
    pose_source = LaunchConfiguration('pose_source')
    simulate = LaunchConfiguration('simulate')

    return LaunchDescription([
        DeclareLaunchArgument('viz', default_value='true',
                              description='run rviz_visualizer'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='also launch RViz itself'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('simulate', default_value='false'),
        DeclareLaunchArgument('target_depth', default_value='2.0'),
        DeclareLaunchArgument(
            'pose_source', default_value='pixhawk_imu',
            description='pixhawk_imu (dead-reckoned, drifts) | zed (vslam)'),

        Node(
            package='mavlink_thruster_control',
            executable='thruster_node',
            name='thruster_controller',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'simulate': simulate,
                # ALT_HOLD from the start: the autopilot owns depth and
                # self-levels roll/pitch for the whole run, including the dive.
                'flight_mode': 'ALT_HOLD',
            }],
        ),

        # Pixhawk IMU → imu/rpy (+ TF). motion_node reads yaw from imu/rpy.
        Node(
            package='imu',
            executable='orientation_node',
            name='orientation_node',
            output='screen',
            parameters=[{
                'imu_topic': '/pixhawk/imu/data',
                # rviz_visualizer owns odom→base_link here (it has the
                # dead-reckoned translation; this node only has orientation).
                # Two broadcasters on one TF edge fight.
                'publish_tf': False,
            }],
        ),

        Node(
            package='control',
            executable='motion_node',
            name='motion_node',
            output='screen',
            parameters=[{
                'target_depth': target_depth,
                'yaw_topic': 'imu/rpy',
            }],
        ),

        Node(
            package='control',
            executable='rviz_visualizer',
            name='rviz_visualizer',
            output='screen',
            condition=IfCondition(viz),
            parameters=[{'pose_source': pose_source}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('control'), 'rviz', 'submerge_hold.rviz'])],
        ),
    ])
