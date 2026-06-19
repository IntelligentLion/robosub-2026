"""Deployment launch for the RoboSub 2026 prequalification run.

Brings up the full minimal stack needed for the scripted prequalification:
the vision detector (ZED + YOLO), the MAVLink thruster controller, and the
prequalification state machine. Vision and thrusters can each be toggled off
if they are already running elsewhere.

Every prequalification_node parameter — including all per-state timeouts — is
surfaced as a launch argument, so you can tune the run two ways:
  * edit config/prequalification.yaml in VSCode and rebuild, or
  * override on the command line (overrides win over the YAML).

Examples:
  # Real sub, full stack:
  ros2 launch prequalification prequalification.launch.py

  # Bench/dry run — simulate thrusters, no camera, just watch the FSM:
  ros2 launch prequalification prequalification.launch.py \\
      simulate:=true include_vision:=false publish_commands:=false

  # Retune timeouts + depth/distance fallbacks on the fly:
  ros2 launch prequalification prequalification.launch.py \\
      submerge_timeout_s:=15 forward_to_marker_timeout_s:=40 \\
      max_depth_m:=1.8 max_forward_distance_m:=10 marker_label:=slalom_pole
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# prequalification_node parameters that are exposed as launch arguments.
# Each becomes `name:=value` on the command line and overrides the YAML.
# Keep these defaults in sync with config/prequalification.yaml.
PARAM_ARGS = [
    # name, default
    ('gate_label', 'gate'),
    ('marker_label', 'marker'),
    ('publish_commands', 'true'),
    # Depth.
    ('depth_tol_m', '0.15'),
    ('max_depth_m', '1.5'),
    ('enable_depth_hold', 'true'),
    ('depth_hold_tol_m', '0.10'),
    ('depth_hold_gain', '1.0'),
    ('depth_hold_min_speed', '0.10'),
    ('depth_hold_max_speed', '0.40'),
    # Speeds.
    ('surge_speed', '0.35'),
    ('strafe_speed', '0.30'),
    ('turn_speed', '0.30'),
    ('submerge_speed', '0.40'),
    ('surface_speed', '0.45'),
    # Vision gating.
    ('detection_conf', '0.50'),
    ('detection_stale_s', '1.0'),
    ('center_tol', '0.10'),
    ('yaw_center_gain', '0.6'),
    ('marker_left_threshold', '0.30'),
    # Gate transit.
    ('gate_top_clear_y', '0.05'),
    ('gate_top_clear_extra_s', '1.5'),
    ('gate_close_bbox', '0.45'),
    ('gate_close_range_m', '1.2'),
    ('gate_pass_duration_s', '5.0'),
    ('final_forward_duration_s', '3.0'),
    # Marker maneuver.
    ('marker_lost_s', '1.5'),
    ('use_pose_for_turns', 'true'),
    ('turn_90_duration_s', '6.0'),
    ('turn_yaw_tol_rad', '0.10'),
    ('turn_min_s', '1.0'),
    ('max_forward_distance_m', '8.0'),
    # ── Per-state safety timeouts (seconds) ──
    ('submerge_timeout_s', '12.0'),
    ('submerge_clear_top_timeout_s', '8.0'),
    ('through_gate_timeout_s', '12.0'),
    ('forward_to_marker_timeout_s', '30.0'),
    ('strafe_timeout_s', '12.0'),
    ('forward_past_marker_timeout_s', '12.0'),
    ('turn_timeout_s', '12.0'),
    ('forward_marker_behind_timeout_s', '15.0'),
    ('strafe_to_gate_timeout_s', '15.0'),
    ('align_gate_timeout_s', '20.0'),
    ('final_forward_timeout_s', '8.0'),
    ('surface_timeout_s', '12.0'),
    ('control_hz', '10.0'),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('prequalification')
    default_params = os.path.join(pkg_share, 'config', 'prequalification.yaml')

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML of prequalification_node parameters.'),
        DeclareLaunchArgument(
            'include_vision', default_value='true',
            description='Launch the vision detector node.'),
        DeclareLaunchArgument(
            'include_thrusters', default_value='true',
            description='Launch the MAVLink thruster controller node.'),
        DeclareLaunchArgument(
            'simulate', default_value='false',
            description='Run the thruster node in simulation (no MAVLink).'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Pixhawk serial port for the thruster node.'),
        DeclareLaunchArgument(
            'view', default_value='false',
            description='Show the detector debug window (off on the sub).'),
    ]

    # Surface every tunable node parameter (timeouts included) as a launch arg.
    overrides = {}
    for name, default in PARAM_ARGS:
        args.append(DeclareLaunchArgument(name, default_value=default))
        overrides[name] = LaunchConfiguration(name)

    prequal_node = Node(
        package='prequalification',
        executable='prequalification_node',
        name='prequalification_node',
        output='screen',
        # YAML first, launch-arg overrides second (overrides win).
        parameters=[LaunchConfiguration('params_file'), overrides],
    )

    # `--view` is conditional, so build the detector twice (with/without it)
    # rather than threading a substitution into argparse.
    vision_node_view = Node(
        package='vision',
        executable='detector',
        name='vision_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('include_vision'), "' == 'true' and '",
            LaunchConfiguration('view'), "' == 'true'"])),
        arguments=['--view'],
    )
    vision_node_noview = Node(
        package='vision',
        executable='detector',
        name='vision_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('include_vision'), "' == 'true' and '",
            LaunchConfiguration('view'), "' != 'true'"])),
        arguments=[],
    )

    thruster_node = Node(
        package='mavlink_thruster_control',
        executable='thruster_node',
        name='thruster_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('include_thrusters')),
        parameters=[{
            'simulate': LaunchConfiguration('simulate'),
            'serial_port': LaunchConfiguration('serial_port'),
        }],
    )

    return LaunchDescription(args + [
        GroupAction([vision_node_view, vision_node_noview]),
        thruster_node,
        prequal_node,
    ])
