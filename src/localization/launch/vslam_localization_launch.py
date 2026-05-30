"""Launch the localization fusion node.

VSLAM (global visual SLAM via the ZED SDK) is INTENTIONALLY DISABLED for
competition. RoboSub tasks are landmark-relative and reactive (detect ->
servo -> act), guided between tasks by orange path markers and acoustic
pingers — none of which award points for global pose. Underwater visual
SLAM also drifts badly (low texture, particulates, refraction breaking the
ZED's air-tuned stereo; cf. drift_correction_node.py, which exists only to
patch that drift) and competes with YOLO for scarce GPU/VRAM on the Orin
Nano. Localization instead relies on the depth sensor (Z), the Pixhawk
IMU/compass (heading), and ZED stereo range-to-target for vision servoing.

The vslam_node source is kept as reference; re-add the commented block below
to the LaunchDescription if you want to bring global SLAM back online.

Usage: ros2 launch localization vslam_localization_launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  # noqa: F401 (used by disabled vslam_node)
from launch_ros.actions import Node


def generate_launch_description():
    area_map_arg = DeclareLaunchArgument('area_map_path', default_value='')
    svo_arg = DeclareLaunchArgument('svo', default_value='')
    save_on_exit_arg = DeclareLaunchArgument('save_area_on_exit', default_value='false')

    # --- VSLAM disabled for competition (kept for reference) ---
    # vslam_node = Node(
    #     package='localization',
    #     executable='vslam_node',
    #     name='vslam_zed_node',
    #     output='screen',
    #     parameters=[{
    #         'enable_area_memory': True,
    #         'area_map_path': LaunchConfiguration('area_map_path'),
    #         'save_area_on_exit': LaunchConfiguration('save_area_on_exit'),
    #         'svo': LaunchConfiguration('svo'),
    #         'zed_fps': 30,
    #     }]
    # )

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
        # vslam_node,   # disabled — see module docstring
        loc_node,
    ])
