"""Launch the localization fusion node.

Global visual SLAM (the ZED SDK area-memory approach) was removed: RoboSub tasks
are landmark-relative and reactive (detect -> servo -> act), guided between tasks
by orange path markers and acoustic pingers — none of which award points for
global pose. Underwater visual SLAM also drifts badly (low texture, particulates,
refraction breaking the ZED's air-tuned stereo) and competes with YOLO for scarce
GPU/VRAM on the Orin.

localization_node still fuses whatever pose sources ARE published (the front
detector's VIO on vslam/odometry, depth, IMU) into localization/pose. The
dedicated vslam_node and the standalone zed2i_vslam package are gone.

Usage: ros2 launch localization vslam_localization_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    loc_node = Node(
        package='localization',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{}],
    )

    return LaunchDescription([
        loc_node,
    ])
