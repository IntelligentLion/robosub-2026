"""
Jetson-optimised launch for ZED2i VSLAM.

Key differences from vslam.launch.py:
  - VGA resolution    (saves ~60% VRAM vs HD720)
  - NEURAL depth mode (replaces deprecated ULTRA, similar accuracy, less VRAM)
  - Spatial mapping OFF (frees ~150 MB VRAM)
  - Point cloud downsample = 8 (quarter the point density)
  - RViz NOT launched on-board — run it on a remote desktop machine:
      export ROS_DOMAIN_ID=<same as Jetson>
      rviz2 -d <path>/vslam.rviz

Usage:
    ros2 launch zed2i_vslam vslam_jetson.launch.py
    ros2 launch zed2i_vslam vslam_jetson.launch.py fps:=15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('zed2i_vslam')

    args = [
        DeclareLaunchArgument('resolution',             default_value='VGA'),
        DeclareLaunchArgument('fps',                    default_value='30'),
        DeclareLaunchArgument('depth_mode',             default_value='NEURAL'),
        DeclareLaunchArgument('enable_spatial_map',     default_value='false'),
        DeclareLaunchArgument('publish_point_cloud',    default_value='true'),
        DeclareLaunchArgument('point_cloud_downsample', default_value='8'),
        DeclareLaunchArgument('base_frame',   default_value='base_link'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('odom_frame',   default_value='odom'),
        DeclareLaunchArgument('map_frame',    default_value='map'),
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'vslam_params.yaml'])),
    ]

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
    )

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

    return LaunchDescription(args + [
        LogInfo(msg='Starting ZED2i VSLAM (Jetson mode: VGA + NEURAL, no spatial map, no RViz)...'),
        vslam_node,
        static_tf_map_odom,
    ])
