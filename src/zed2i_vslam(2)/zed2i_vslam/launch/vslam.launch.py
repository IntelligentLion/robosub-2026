"""
Launch file for the ZED2i VSLAM node.

Usage:
    ros2 launch zed2i_vslam vslam.launch.py
    ros2 launch zed2i_vslam vslam.launch.py resolution:=HD1080 fps:=30
    ros2 launch zed2i_vslam vslam.launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('zed2i_vslam')

    # ── Launch arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('resolution',          default_value='HD720',
            description='Camera resolution: HD2K | HD1080 | HD720 | VGA'),
        DeclareLaunchArgument('fps',                 default_value='30',
            description='Camera frame rate'),
        DeclareLaunchArgument('depth_mode',          default_value='ULTRA',
            description='Depth mode: ULTRA | QUALITY | PERFORMANCE | NEURAL'),
        DeclareLaunchArgument('enable_spatial_map',  default_value='true',
            description='Enable ZED spatial mapping'),
        DeclareLaunchArgument('publish_point_cloud', default_value='true',
            description='Publish PointCloud2 topic'),
        DeclareLaunchArgument('point_cloud_downsample', default_value='4',
            description='Keep every Nth row/col of the point cloud'),
        DeclareLaunchArgument('base_frame',   default_value='base_link'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('odom_frame',   default_value='odom'),
        DeclareLaunchArgument('map_frame',    default_value='map'),
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'vslam_params.yaml']),
            description='Path to YAML parameters file'),
        DeclareLaunchArgument('rviz', default_value='false',
            description='Launch RViz2 visualisation'),
        DeclareLaunchArgument('rviz_config',
            default_value=PathJoinSubstitution([pkg_share, 'rviz', 'vslam.rviz']),
            description='RViz2 config file'),
    ]

    # ── VSLAM node ───────────────────────────────────────────────────────────
    vslam_node = Node(
        package='zed2i_vslam',
        executable='vslam_node',
        name='zed2i_vslam',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'resolution':             LaunchConfiguration('resolution'),
                'fps':                    LaunchConfiguration('fps'),
                'depth_mode':             LaunchConfiguration('depth_mode'),
                'enable_spatial_map':     LaunchConfiguration('enable_spatial_map'),
                'publish_point_cloud':    LaunchConfiguration('publish_point_cloud'),
                'point_cloud_downsample': LaunchConfiguration('point_cloud_downsample'),
                'base_frame':             LaunchConfiguration('base_frame'),
                'camera_frame':           LaunchConfiguration('camera_frame'),
                'odom_frame':             LaunchConfiguration('odom_frame'),
                'map_frame':              LaunchConfiguration('map_frame'),
            },
        ],
        remappings=[
            # Remap to nav2-compatible names if desired:
            # ('/zed2i/odom', '/odom'),
            # ('/zed2i/pose', '/pose'),
        ],
    )

    # ── Static TF: odom → map  (identity; replace with EKF output in production)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id',       LaunchConfiguration('map_frame'),
            '--child-frame-id', LaunchConfiguration('odom_frame'),
        ],
    )

    # ── RViz2 ────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(args + [
        LogInfo(msg='Starting ZED2i VSLAM node...'),
        vslam_node,
        static_tf_map_odom,
        rviz_node,
    ])
