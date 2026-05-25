"""Launch VSLAM ZED node and localization fusion node together.

Usage: ros2 launch localization vslam_localization_launch.py area_map_path:=/path/to/map.area
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    area_map_arg = DeclareLaunchArgument('area_map_path', default_value='')
    svo_arg = DeclareLaunchArgument('svo', default_value='')
    save_on_exit_arg = DeclareLaunchArgument('save_area_on_exit', default_value='false')

    vslam_node = Node(
        package='localization',
        executable='vslam_node',
        name='vslam_zed_node',
        output='screen',
        parameters=[{
            'enable_area_memory': True,
            'area_map_path': LaunchConfiguration('area_map_path'),
            'save_area_on_exit': LaunchConfiguration('save_area_on_exit'),
            'svo': LaunchConfiguration('svo'),
            'zed_fps': 30,
        }]
    )

    loc_node = Node(
        package='localization',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{}]
    )

    return LaunchDescription([
        area_map_arg,
        svo_arg,
        save_on_exit_arg,
        vslam_node,
        loc_node,
    ])
