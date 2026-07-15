"""move_forward_depth_hold — the whole Move Forward + Depth Hold system, one command.

    ros2 launch control move_forward_depth_hold.launch.py \
        forward_speed:=0.4 target_depth:=2.0 forward_duration:=10.0

Nodes, in the order they come up:

  thruster_node       MAVLink gateway. SOLE owner of /dev/ttyACM0. Publishes
                      pixhawk/{imu/data,depth,mode,armed}; serves
                      pixhawk/{set_mode,preflight,disarm}. Starts in ALT_HOLD:
                      ArduSub owns depth and self-levels roll/pitch.
  orientation_node    pixhawk/imu/data -> imu/rpy (+no TF; see below).
  motion_node         THE movement node — sole publisher of movement_command.
                      SubmergeController -> DepthController (the dive) and
                      HeadingController (yaw, the one gap ALT_HOLD has no mode
                      for). Depth hold itself is ArduSub's, not ours.
  rviz_visualizer     subscribe-only markers/path/TF. viz:=false to omit.
  rviz2               the display. rviz:=false to omit (headless pool runs).
  forward_hold_mission  gates on the stack being ALIVE (Pixhawk answering, Bar02
                      reporting real depth, services up, motion_node listening),
                      then dives, holds, and drives forward. Nonzero exit on any
                      failed gate.

Ordering is by readiness, not by sleep: forward_hold_mission blocks on the
services/topics it needs before it commands anything, so it is safe to start all
processes at once. Launch can only sequence PROCESSES; a live thruster_node with
a dead Bar02 is exactly the failure this vehicle actually has.

If ANY required node exits — including the mission finishing or failing — the
handler below names it and shuts the whole stack down. Half a stack is worse
than none: motion_node without thruster_node is a controller shouting into a
dead topic, and thruster_node without motion_node is live thrusters with no
commander.

DO NOT run pix_imu/pixhawk_imu_bridge alongside this. Two readers on one serial
port produce the "device reports readiness to read but returned no data" stall
and both die.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    ('forward_speed', '0.4',
     'forward surge while holding depth, normalized 0.0-1.0'),
    ('forward_duration', '10.0',
     'seconds to drive forward once at depth; <=0 means hold only, no forward'),
    ('target_depth', '2.0', 'depth to dive to and hold, metres below surface'),
    ('dive_speed', '0.3',
     'descent rate during the dive, normalized 0.0-1.0 (motion_node param)'),
    ('max_forward_speed', '0.6',
     'safety clamp on surge inside motion_node; forward_speed is clamped to it'),
    ('startup_timeout', '30.0',
     'seconds to wait for Pixhawk link, depth data and services before abort'),
    ('dive_timeout', '60.0', 'seconds to reach target_depth before abort'),
    ('serial_port', '/dev/ttyACM0', 'Pixhawk serial port'),
    ('simulate', 'false', 'run thruster_node without a Pixhawk (dry test)'),
    ('auto_start', 'true',
     'run the mission. false = bring the stack up and drive it yourself with '
     'control.api.Auv or ros2 topic pub'),
    ('viz', 'true', 'run rviz_visualizer (markers/path/TF)'),
    ('rviz', 'true', 'run RViz itself with the submerge_hold config'),
    ('pose_source', 'pixhawk_imu',
     'pixhawk_imu (dead-reckoned XY, drifts) | zed (vslam/odometry)'),
]


def _fatal_on_exit(node_action, label):
    """Any exit of a required node takes the stack down with it, naming who died
    and with what code. A mission that ends cleanly lands here too — that is the
    intended shutdown path, not an error."""
    def on_exit(event, context):
        code = event.returncode
        verdict = ('finished cleanly' if code == 0
                   else f'EXITED UNEXPECTEDLY (return code {code})')
        return [
            LogInfo(msg=f'[move_forward_depth_hold] {label} {verdict} — '
                        'shutting the stack down cleanly.'),
            EmitEvent(event=Shutdown(
                reason=f'{label} exited with code {code}')),
        ]

    return RegisterEventHandler(
        OnProcessExit(target_action=node_action, on_exit=on_exit))


def generate_launch_description():
    cfg = {name: LaunchConfiguration(name) for name, _, _ in ARGS}

    thruster = Node(
        package='mavlink_thruster_control',
        executable='thruster_node',
        name='thruster_controller',
        output='screen',
        parameters=[{
            'serial_port': cfg['serial_port'],
            'simulate': cfg['simulate'],
            # ALT_HOLD from the start: the autopilot owns depth and self-levels
            # roll/pitch for the whole run, the dive included.
            'flight_mode': 'ALT_HOLD',
        }],
    )

    orientation = Node(
        package='imu',
        executable='orientation_node',
        name='orientation_node',
        output='screen',
        parameters=[{
            'imu_topic': '/pixhawk/imu/data',
            # rviz_visualizer owns odom->base_link (it has the dead-reckoned
            # translation; this node only has orientation). Two broadcasters on
            # one TF edge fight.
            'publish_tf': False,
        }],
    )

    motion = Node(
        package='control',
        executable='motion_node',
        name='motion_node',
        output='screen',
        parameters=[{
            'target_depth': cfg['target_depth'],
            'dive_speed': cfg['dive_speed'],
            # motion_node has no forward_speed of its own — surge is the
            # operator's axis and arrives on motion/cmd from the mission. What
            # lives here is the clamp that bounds it.
            'max_forward_speed': cfg['max_forward_speed'],
            'yaw_topic': 'imu/rpy',
        }],
    )

    mission = Node(
        package='control',
        executable='forward_hold_mission',
        name='forward_hold_mission',
        output='screen',
        condition=IfCondition(cfg['auto_start']),
        parameters=[{
            'target_depth': cfg['target_depth'],
            'forward_speed': cfg['forward_speed'],
            'forward_duration': cfg['forward_duration'],
            'startup_timeout': cfg['startup_timeout'],
            'dive_timeout': cfg['dive_timeout'],
        }],
    )

    visualizer = Node(
        package='control',
        executable='rviz_visualizer',
        name='rviz_visualizer',
        output='screen',
        condition=IfCondition(cfg['viz']),
        parameters=[{'pose_source': cfg['pose_source']}],
    )

    # RViz is deliberately NOT wired into _fatal_on_exit: closing the window is
    # an operator action, not a vehicle failure, and it must not stop a dive.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(cfg['rviz']),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('control'), 'rviz', 'submerge_hold.rviz'])],
    )

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, default, desc in ARGS],

        LogInfo(msg=['[move_forward_depth_hold] starting — target_depth=',
                     cfg['target_depth'], ' m, forward_speed=',
                     cfg['forward_speed'], ' for ', cfg['forward_duration'],
                     's, port=', cfg['serial_port'],
                     ', simulate=', cfg['simulate']]),
        LogInfo(msg='[move_forward_depth_hold] 1/4 thruster_node — MAVLink '
                    'gateway, ALT_HOLD (ArduSub owns depth)'),
        thruster,
        LogInfo(msg='[move_forward_depth_hold] 2/4 orientation_node — '
                    'pixhawk/imu/data -> imu/rpy'),
        orientation,
        LogInfo(msg='[move_forward_depth_hold] 3/4 motion_node — sole publisher '
                    'of movement_command (dive + heading hold)'),
        motion,
        LogInfo(msg='[move_forward_depth_hold] 4/4 rviz_visualizer + RViz — '
                    'pose, headings, depth, correction vectors'),
        visualizer,
        rviz,
        LogInfo(msg='[move_forward_depth_hold] mission gating on: Pixhawk link, '
                    'preflight/set_mode services, live Bar02 depth, motion_node '
                    'listening. It will NOT dive until all four pass.'),
        mission,

        _fatal_on_exit(thruster, 'thruster_node (MAVLink gateway)'),
        _fatal_on_exit(orientation, 'orientation_node (yaw source)'),
        _fatal_on_exit(motion, 'motion_node (movement)'),
        _fatal_on_exit(mission, 'forward_hold_mission'),
    ])
