"""Deployment launch for the RoboSub 2026 prequalification run.

Brings up the full minimal stack needed for the scripted prequalification:
the vision detector (ZED + YOLO), the MAVLink thruster controller, and the
prequalification state machine. Vision and thrusters can each be toggled off
if they are already running elsewhere.

Examples:
  # Real sub, full stack:
  ros2 launch prequalification prequalification.launch.py

  # Bench/dry run — simulate thrusters, no camera, just watch the FSM:
  ros2 launch prequalification prequalification.launch.py \\
      simulate:=true include_vision:=false publish_commands:=false

  # Tune depth and the marker label on the fly:
  ros2 launch prequalification prequalification.launch.py \\
      target_depth_m:=1.2 marker_label:=slalom_pole
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
        # Common per-run overrides surfaced as launch args for convenience.
        DeclareLaunchArgument('gate_label', default_value='gate'),
        DeclareLaunchArgument('marker_label', default_value='marker'),
        DeclareLaunchArgument('target_depth_m', default_value='1.0'),
        DeclareLaunchArgument('publish_commands', default_value='true'),
    ]

    overrides = {
        'gate_label': LaunchConfiguration('gate_label'),
        'marker_label': LaunchConfiguration('marker_label'),
        'target_depth_m': LaunchConfiguration('target_depth_m'),
        'publish_commands': LaunchConfiguration('publish_commands'),
    }

    prequal_node = Node(
        package='prequalification',
        executable='prequalification_node',
        name='prequalification_node',
        output='screen',
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
