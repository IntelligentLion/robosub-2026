"""mission_stack — the full autonomy stack: submerge_hold + BOTH cameras.

Brings up, in one launch:
  * the submerge_hold stack (thruster_node + orientation_node + motion_node
    [+ rviz_visualizer]) — see submerge_hold.launch.py
  * detector           front ZED 2i (serial 31166146) → vision/detections
                       (+ vslam/odometry, depth/sub_depth). Full depth/VIO.
  * bottom_camera      bottom ZED 2i (serial 30758628) → vision/path_markers.
                       Runs 2D-ONLY (see below).

Then drive it with a mission client that subscribes the detection topics and
commands control.api.Auv, e.g.:

    ros2 launch control mission_stack.launch.py
    python3 bt_coinflip.py --depth-ft 3 --yes

NOT CLOGGING TWO ZED 2i ON ONE JETSON — the levers this launch pulls:
  1. SERIALS PINNED. Each detector opens its OWN camera by serial, so the two
     never race for "first available" (which silently gave both nodes the same
     device). Front=31166146, bottom=30758628 are the node defaults now.
  2. BOTTOM IS 2D-ONLY (--twod_only). Path markers only need 2D boxes, so the
     bottom camera runs with NO ZED depth engine, NO positional tracking (VIO),
     NO 3D object pass — roughly HALVING its GPU + compute. As a bonus this
     removes the depth/sub_depth DOUBLE-PUBLISHER: in 2D-only the bottom node
     does not publish depth/sub_depth or odom/bottom at all, leaving the front
     camera the sole publisher. Set bottom_twod:=false to restore full VIO
     (only if the Jetson has the headroom — verify with tegrastats).
  3. FRONT FPS HALVED (front_fps default 30, was 60). Detection does not need
     60 fps; 30 halves USB3 bandwidth and inference load. The front camera keeps
     full depth because vslam/odometry feeds localization when pose_source:=zed.

Physical, NOT fixable in code (do these too):
  * ZED USB3 noise jams 2.4 GHz WiFi — worse with two cameras. Tether on 5 GHz
    or ethernet, never 2.4 GHz SSH.
  * One owner per ZED. Do NOT also run pix_imu / another ZED consumer — they
    will fight for the device.
  * Watch tegrastats the first time both run: if GPU/RAM saturates, drop
    front_fps further or set enable_bottom:=false.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                   PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_front = LaunchConfiguration('enable_front')
    enable_bottom = LaunchConfiguration('enable_bottom')
    bottom_twod = LaunchConfiguration('bottom_twod')
    front_fps = LaunchConfiguration('front_fps')
    viz = LaunchConfiguration('viz')
    target_depth = LaunchConfiguration('target_depth')
    simulate = LaunchConfiguration('simulate')

    # Node.arguments must be a list, so the --twod_only toggle can't live inside
    # one substitution. Run the bottom camera as two mutually-exclusive nodes:
    # lean (2D-only) when bottom_twod:=true, full VIO otherwise.
    bottom_lean = IfCondition(PythonExpression(
        ["'true' if '", enable_bottom, "' == 'true' and '", bottom_twod,
         "' == 'true' else 'false'"]))
    bottom_full = IfCondition(PythonExpression(
        ["'true' if '", enable_bottom, "' == 'true' and '", bottom_twod,
         "' == 'false' else 'false'"]))

    return LaunchDescription([
        DeclareLaunchArgument('enable_front', default_value='true',
                              description='run the front-camera detector'),
        DeclareLaunchArgument('enable_bottom', default_value='true',
                              description='run the bottom-camera detector'),
        DeclareLaunchArgument(
            'bottom_twod', default_value='true',
            description='bottom camera 2D-only (lean); false = full VIO'),
        DeclareLaunchArgument('front_fps', default_value='30',
                              description='front ZED fps (30 halves the load)'),
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument('target_depth', default_value='2.0'),
        DeclareLaunchArgument('simulate', default_value='false'),

        # ── the whole submerge-hold stack ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('control'), 'launch',
                'submerge_hold.launch.py'])),
            launch_arguments={
                'viz': viz,
                'target_depth': target_depth,
                'simulate': simulate,
            }.items(),
        ),

        # ── front camera: full depth/VIO, fps capped ──
        Node(
            package='vision',
            executable='detector',
            name='vision_node',
            output='screen',
            condition=IfCondition(enable_front),
            # serial defaults to the front cam (31166146) inside the node.
            arguments=['--zed_fps', front_fps],
        ),

        # ── bottom camera, lean: 2D-only (no depth/VIO/3D, no double-pub) ──
        Node(
            package='vision',
            executable='bottom_camera',
            name='bottom_camera_node',
            output='screen',
            condition=bottom_lean,
            # serial defaults to the bottom cam (30758628) inside the node.
            arguments=['--twod_only'],
        ),

        # ── bottom camera, full VIO (only if the Jetson has headroom) ──
        Node(
            package='vision',
            executable='bottom_camera',
            name='bottom_camera_node',
            output='screen',
            condition=bottom_full,
        ),
    ])
