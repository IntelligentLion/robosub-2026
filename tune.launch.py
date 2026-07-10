"""Front-camera PID-tuning stack — one command brings up the nodes `tune_pid.py`
needs so you can run `step`/`set`/`list` against a live controller.

Nodes:
  autonomous_controller  — the PID controller being tuned
  detector (vision)      — front ZED: detections + depth/sub_depth + vslam/odometry
  localization_node      — fuses vslam/odometry -> localization/pose
  thruster_node          — movement_command -> Pixhawk thrusters (actual motion)

The front ZED is single-owner: this runs `detector` (which now publishes pose as
well as depth), so do NOT also start vslam_node or a depth_hold_* script. The
Pixhawk is single-owner too: thruster_node holds /dev/ttyACM0 — no motor_test /
depth_hold_* alongside.

Run (no build needed — launch takes a file path):
    ros2 launch ./tune.launch.py
    ros2 launch ./tune.launch.py model:=/abs/path/to/other.onnx
    ros2 launch ./tune.launch.py thrusters:=false      # dry, controller+sensing only

Then in another sourced terminal:
    ros2 topic hz localization/pose        # should tick ~10 Hz
    python3 tune_pid.py list
    python3 tune_pid.py step --axis yaw --step 0.5 --seconds 12 --csv yaw_a.csv --plot
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Forward-facing camera model, converted from ffc_rs_26.pt via
# convert_to_onnx.py (imgsz 416). Passed to detector as an absolute path so it
# resolves regardless of the build/install layout.
_DEFAULT_MODEL = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'src', 'vision', 'vision', 'ffc_rs_26.onnx')


def generate_launch_description():
    model = LaunchConfiguration('model')
    thrusters = LaunchConfiguration('thrusters')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=_DEFAULT_MODEL,
            description='ONNX model path for the front-camera detector'),
        # DEFAULT OFF: thruster_node arms the Pixhawk and goes live the instant
        # it starts (ALT_HOLD, 10 Hz loop) — thrusters spin immediately. Only
        # set true when the sub is in water, props clear, and you are ready to
        # move. Bring the sensing stack up first, confirm pose, THEN relaunch
        # with thrusters:=true (or start thruster_node in its own terminal).
        DeclareLaunchArgument(
            'thrusters', default_value='false',
            description='Start thruster_node — ARMS + spins thrusters on start. '
                        'Only true when in water, props clear, ready to move.'),

        Node(
            package='control', executable='autonomous_controller',
            name='autonomous_controller', output='screen'),

        # Front ZED: detections + depth/sub_depth + vslam/odometry (patched).
        Node(
            package='vision', executable='detector',
            name='vision_node', output='screen',
            arguments=['--onnx', model]),

        # vslam/odometry -> localization/pose
        Node(
            package='localization', executable='localization_node',
            name='localization_node', output='screen'),

        # movement_command -> Pixhawk. Skipped when thrusters:=false.
        Node(
            package='mavlink_thruster_control', executable='thruster_node',
            name='thruster_node', output='screen',
            condition=IfCondition(thrusters)),
    ])
