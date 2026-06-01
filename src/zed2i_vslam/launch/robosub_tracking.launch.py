#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_zed_wrapper = get_package_share_directory("zed_wrapper")
    pkg_this        = get_package_share_directory("zed2i_vslam")
    return LaunchDescription([
        DeclareLaunchArgument("rviz",             default_value="true"),
        DeclareLaunchArgument("svo_path",         default_value=""),
        DeclareLaunchArgument("area_memory_path", default_value=""),
        LogInfo(msg="=== RoboSub ZED2i Tracking Stack Starting ==="),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_zed_wrapper, "launch", "zed_camera.launch.py")),
            launch_arguments={
                "camera_model":        "zed2i",
                "camera_name":         "zed2i",
                "config_path":         os.path.join(pkg_this, "config", "zed2i_robosub.yaml"),
                "publish_urdf":        "true",
                "publish_tf":          "true",
                "publish_map_tf":      "true",
                "svo_path":            LaunchConfiguration("svo_path"),
                "area_memory_db_path": LaunchConfiguration("area_memory_path"),
            }.items(),
        ),
        Node(
            package="zed2i_vslam", executable="zed2i_vslam_node",
            name="zed2i_vslam_node", output="screen",
            parameters=[os.path.join(pkg_this, "config", "vslam_node.yaml")],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2_tracking",
            arguments=["-d", os.path.join(pkg_this, "rviz", "robosub_tracking.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
