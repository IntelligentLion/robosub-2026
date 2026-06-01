#!/usr/bin/env python3
"""
robosub_tracking.launch.py

Launches:
  1. ZED2i wrapper node  (with RoboSub-optimized config)
  2. zed2i_vslam_node    (jerk-rejection + filtered pose/path)
  3. RViz2               (pre-configured tracking layout)

Usage:
  ros2 launch zed2i_vslam robosub_tracking.launch.py
  ros2 launch zed2i_vslam robosub_tracking.launch.py rviz:=false
  ros2 launch zed2i_vslam robosub_tracking.launch.py svo_path:=/path/to/file.svo2
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_zed_wrapper  = get_package_share_directory("zed_wrapper")
    pkg_this         = get_package_share_directory("zed2i_vslam")

    # ── Arguments ────────────────────────────────────────────────────────────
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Launch RViz2 for tracking visualization")

    svo_arg = DeclareLaunchArgument(
        "svo_path", default_value="",
        description="Path to SVO2 file for replay (leave empty for live camera)")

    area_memory_arg = DeclareLaunchArgument(
        "area_memory_path", default_value="",
        description="Path to .area file for pre-mapped pool relocalization")

    # ── ZED wrapper ───────────────────────────────────────────────────────────
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model":       "zed2i",
            "camera_name":        "zed2i",
            "config_path":        os.path.join(pkg_this, "config", "zed2i_robosub.yaml"),
            "publish_urdf":       "true",
            "publish_tf":         "true",
            "publish_map_tf":     "true",
            "svo_path":           LaunchConfiguration("svo_path"),
            "area_memory_db_path": LaunchConfiguration("area_memory_path"),
            # Depth mode overrideable at launch time
            "depth_mode":         "NEURAL",
        }.items(),
    )

    # ── VSLAM / filtering node ────────────────────────────────────────────────
    vslam_node = Node(
        package="zed2i_vslam",
        executable="zed2i_vslam_node",
        name="zed2i_vslam_node",
        output="screen",
        parameters=[
            os.path.join(pkg_this, "config", "vslam_node.yaml"),
        ],
        remappings=[
            # Already correct — just documenting the topic flow:
            # /zed2i/zed_node/pose  → filtered → ~/filtered_pose
            # /zed2i/zed_node/odom  → filtered → ~/filtered_odom
        ],
        # Increase priority for tracking thread
        additional_env={"RCUTILS_LOGGING_MIN_SEVERITY": "INFO"},
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_tracking",
        arguments=[
            "-d", os.path.join(pkg_this, "rviz", "robosub_tracking.rviz")
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        rviz_arg,
        svo_arg,
        area_memory_arg,
        LogInfo(msg="=== RoboSub ZED2i Tracking Stack Starting ==="),
        zed_launch,
        vslam_node,
        rviz_node,
    ])
