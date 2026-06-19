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

  # Retune timeouts on the fly (every param below is a launch arg):
  ros2 launch prequalification prequal_dry_test.launch.py \\
      submerge_timeout_s:=20 turn_timeout_s:=10 gate_pass_duration_s:=4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# prequal_dry_test_node parameters exposed as launch arguments. Each becomes
# `name:=value` on the command line. Keep defaults in sync with the node.
PARAM_ARGS = [
    ('gate_label', 'pointed_nose'),
    ('marker_label', 'round_nose'),
    ('publish_commands', 'true'),
    # Speeds.
    ('surge_speed', '0.35'),
    ('strafe_speed', '0.30'),
    ('turn_speed', '0.30'),
    ('submerge_speed', '0.40'),
    ('surface_speed', '0.45'),
    # Vision gating.
    ('detection_conf', '0.50'),
    ('detection_stale_s', '1.0'),
    ('center_tol', '0.12'),
    ('yaw_center_gain', '0.6'),
    ('marker_left_threshold', '0.30'),
    # Gate transit.
    ('gate_top_clear_y', '0.05'),
    ('gate_top_clear_extra_s', '1.5'),
    ('gate_close_bbox', '0.45'),
    ('gate_close_range_m', '1.2'),
    ('align_commit_dwell_s', '1.5'),
    ('gate_pass_duration_s', '5.0'),
    ('final_forward_duration_s', '3.0'),
    ('surface_duration_s', '5.0'),
    # Marker maneuver.
    ('marker_lost_s', '1.5'),
    ('turn_90_duration_s', '6.0'),
    ('turn_min_s', '1.0'),
    # ── Per-state safety timeouts (seconds) ──
    ('submerge_timeout_s', '30.0'),
    ('submerge_clear_top_timeout_s', '10.0'),
    ('through_gate_timeout_s', '12.0'),
    ('forward_to_marker_timeout_s', '30.0'),
    ('strafe_timeout_s', '20.0'),
    ('forward_past_marker_timeout_s', '20.0'),
    ('turn_timeout_s', '12.0'),
    ('forward_marker_behind_timeout_s', '20.0'),
    ('strafe_to_gate_timeout_s', '30.0'),
    ('align_gate_timeout_s', '30.0'),
    ('final_forward_timeout_s', '8.0'),
    ('surface_timeout_s', '12.0'),
    ('control_hz', '10.0'),
]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'include_vision', default_value='true',
            description='Launch the vision detector node (real camera).'),
        DeclareLaunchArgument(
            'view', default_value='false',
            description='Show the detector debug window.'),
    ]

    # Surface every tunable node parameter (timeouts included) as a launch arg.
    overrides = {}
    for name, default in PARAM_ARGS:
        args.append(DeclareLaunchArgument(name, default_value=default))
        overrides[name] = LaunchConfiguration(name)

    dry_test_node = Node(
        package='prequalification',
        executable='prequalification_dry_test',
        name='prequal_dry_test_node',
        output='screen',
        parameters=[overrides],
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
