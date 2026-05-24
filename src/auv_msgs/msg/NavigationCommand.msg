# High-level navigation command for the autonomous controller
#
# Modes:
#   idle          - stop all movement, no active control
#   station_keep  - hold current position and heading using localization
#   track_object  - use vision to center on and approach target_label
#   search        - rotate to find target_label, then switch to track
#   waypoint      - navigate to target position using localization
#   heading_hold  - maintain target_yaw while allowing other axes to be commanded
#
string mode
string target_label    # object class name for track_object / search modes
float32 target_x       # meters, for waypoint mode
float32 target_y       # meters, for waypoint mode
float32 target_z       # meters (depth, positive down), for waypoint mode
float32 target_yaw     # radians, for waypoint / heading_hold modes
float32 speed          # 0.0 to 1.0 normalized max speed
float32 approach_dist  # meters, stop this far from target (0 = use default)
