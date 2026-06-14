"""Bench DRY-TEST launch for the prequalification logic (no thrusters).

Brings up the vision detector (real camera) and the print-only
prequalification logic tester (``prequal_dry_test_node``). Nothing is ever
driven — the node publishes the real ``movement_command`` topic so you can run
``python3 dry_test.py --camera`` in another terminal to see what the thrusters
would do, but no Pixhawk / thruster node is started.

The detector's ``pointed_nose`` class plays the FULL GATE and ``round_nose``
plays the VERTICAL MARKER; hold the camera and point at each to drive the FSM.

Examples:
  # Detector + dry-test FSM (then run `python3 dry_test.py --camera` elsewhere):
  ros2 launch prequalification prequal_dry_test.launch.py

  # Detector already running elsewhere — just the FSM:
  ros2 launch prequalification prequal_dry_test.launch.py include_vision:=false

  # Show the detector debug window:
  ros2 launch prequalification prequal_dry_test.launch.py view:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'include_vision', default_value='true',
            description='Launch the vision detector node (real camera).'),
        DeclareLaunchArgument(
            'view', default_value='false',
            description='Show the detector debug window.'),
        DeclareLaunchArgument(
            'gate_label', default_value='pointed_nose',
            description='Detector class that plays the FULL GATE.'),
        DeclareLaunchArgument(
            'marker_label', default_value='round_nose',
            description='Detector class that plays the VERTICAL MARKER.'),
        DeclareLaunchArgument(
            'publish_commands', default_value='true',
            description='Publish movement_command (printed by dry_test.py).'),
    ]

    dry_test_node = Node(
        package='prequalification',
        executable='prequalification_dry_test',
        name='prequal_dry_test_node',
        output='screen',
        parameters=[{
            'gate_label': LaunchConfiguration('gate_label'),
            'marker_label': LaunchConfiguration('marker_label'),
            'publish_commands': LaunchConfiguration('publish_commands'),
        }],
    )

    # `--view` is conditional, so build the detector twice (with/without it).
    vision_node_view = Node(
        package='vision', executable='detector', name='vision_node',
        output='screen', arguments=['--view'],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('include_vision'), "' == 'true' and '",
            LaunchConfiguration('view'), "' == 'true'"])),
    )
    vision_node_noview = Node(
        package='vision', executable='detector', name='vision_node',
        output='screen', arguments=[],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('include_vision'), "' == 'true' and '",
            LaunchConfiguration('view'), "' != 'true'"])),
    )

    return LaunchDescription(args + [
        GroupAction([vision_node_view, vision_node_noview]),
        dry_test_node,
    ])
