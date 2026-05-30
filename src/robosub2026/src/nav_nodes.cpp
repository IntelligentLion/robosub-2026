// SHRUB v3 — Navigation action nodes
// Each node follows the StatefulActionNode pattern:
//   onStart()   → send goal to ROS 2 action server, return RUNNING
//   onRunning() → check action server feedback, return RUNNING/SUCCESS/FAILURE
//   onHalted()  → cancel the action server goal
//
// TODO: Replace the placeholder topic/action names with your actual
// ROS 2 action servers from your sub's navigation stack.

#include <bt_mission/shrub_nodes.hpp>
#include <bt_mission/mission_io.hpp>

namespace shrub {

// ─── Submerge ───────────────────────────────────────────────────
BT::NodeStatus Submerge::onStart() {
  double target = 1.2;
  getInput("target_depth", target);
  // TODO: Send depth goal to your depth controller action server
  // e.g., depth_client_->async_send_goal({target});
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Submerge → %.1fm", target);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Submerge::onRunning() {
  // TODO: Check depth controller feedback
  // if (at_target_depth) return SUCCESS;
  // if (action_failed) return FAILURE;
  return BT::NodeStatus::SUCCESS; // placeholder
}
void Submerge::onHalted() {
  // TODO: Cancel depth goal
}

// ─── AscendTo ───────────────────────────────────────────────────
BT::NodeStatus AscendTo::onStart() {
  double target = 1.0;
  getInput("target_depth", target);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "AscendTo → %.1fm", target);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus AscendTo::onRunning() { return BT::NodeStatus::SUCCESS; }
void AscendTo::onHalted() {}

// ─── EmergencySurface ───────────────────────────────────────────
BT::NodeStatus EmergencySurface::tick() {
  std::string reason;
  getInput("reason", reason);
  RCLCPP_ERROR(rclcpp::get_logger("shrub"), "EMERGENCY SURFACE: %s", reason.c_str());
  // Command full upward thrust via the thruster stack. Open-loop and
  // intentionally unconditional — this is the safety abort path.
  if (MissionIO::ready()) {
    MissionIO::get().sendMovement("emerge", 0.8);
  }
  // Return FAILURE so the parent ReactiveSequence halts the mission.
  return BT::NodeStatus::FAILURE;
}

// ─── Turn ───────────────────────────────────────────────────────
BT::NodeStatus Turn::onStart() {
  double deg = 0;
  getInput("degrees", deg);
  // TODO: Send yaw command to attitude controller
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Turn %.0f°", deg);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Turn::onRunning() { return BT::NodeStatus::SUCCESS; }
void Turn::onHalted() {}

// ─── Navigate_to ────────────────────────────────────────────────
BT::NodeStatus Navigate_to::onStart() {
  std::string wp;
  getInput("waypoint", wp);
  // TODO: Look up waypoint coordinates, send to navigation action server
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Navigate to waypoint: %s", wp.c_str());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Navigate_to::onRunning() { return BT::NodeStatus::SUCCESS; }
void Navigate_to::onHalted() {}

// ─── Navigate_to_bearing ────────────────────────────────────────
BT::NodeStatus Navigate_to_bearing::onStart() {
  double bearing = 0, stop = 1.0;
  getInput("bearing", bearing);
  getInput("stop_dist", stop);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Navigate bearing %.1f°, stop at %.1fm",
              bearing, stop);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Navigate_to_bearing::onRunning() { return BT::NodeStatus::SUCCESS; }
void Navigate_to_bearing::onHalted() {}

// ─── Navigate_on_heading ────────────────────────────────────────
BT::NodeStatus Navigate_on_heading::onStart() {
  double heading = 0, dist = 0;
  getInput("heading", heading);
  getInput("dist", dist);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Navigate heading %.1f° for %.1fm",
              heading, dist);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Navigate_on_heading::onRunning() { return BT::NodeStatus::SUCCESS; }
void Navigate_on_heading::onHalted() {}

// ─── Navigate_forward ───────────────────────────────────────────
BT::NodeStatus Navigate_forward::onStart() {
  double dist = 0;
  getInput("dist", dist);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Navigate_forward::onRunning() { return BT::NodeStatus::SUCCESS; }
void Navigate_forward::onHalted() {}

// ─── Move_through_gate ──────────────────────────────────────────
BT::NodeStatus Move_through_gate::onStart() {
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Moving through gate...");
  // TODO: Drive forward at current heading until gate is behind
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Move_through_gate::onRunning() { return BT::NodeStatus::SUCCESS; }
void Move_through_gate::onHalted() {}

// ─── Reposition_to_gate_side ────────────────────────────────────
BT::NodeStatus Reposition_to_gate_side::onStart() {
  std::string side;
  getInput("side", side);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Reposition to gate side: %s", side.c_str());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Reposition_to_gate_side::onRunning() { return BT::NodeStatus::SUCCESS; }
void Reposition_to_gate_side::onHalted() {}

// ─── Record_heading ─────────────────────────────────────────────
BT::NodeStatus Record_heading::tick() {
  // TODO: Read current heading from IMU topic
  double heading = 0.0; // placeholder
  setOutput("heading", heading);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Recorded heading: %.1f°", heading);
  return BT::NodeStatus::SUCCESS;
}

// ─── Compute_reverse_heading ────────────────────────────────────
BT::NodeStatus Compute_reverse_heading::tick() {
  double gate_heading = 0;
  getInput("gate_heading", gate_heading);
  double reverse = std::fmod(gate_heading + 180.0, 360.0);
  setOutput("result", reverse);
  return BT::NodeStatus::SUCCESS;
}

// ─── Recalibrate_nav ────────────────────────────────────────────
BT::NodeStatus Recalibrate_nav::tick() {
  // TODO: Trigger DVL+IMU fusion recalibration service
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Navigation recalibrated");
  return BT::NodeStatus::SUCCESS;
}

// ─── Stabilize ──────────────────────────────────────────────────
BT::NodeStatus Stabilize::onStart() {
  int ms = 1000;
  getInput("msec", ms);
  end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Stabilize::onRunning() {
  // TODO: Also command position hold to thrusters
  if (std::chrono::steady_clock::now() >= end_time_)
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}
void Stabilize::onHalted() {}

// ─── Wait ───────────────────────────────────────────────────────
BT::NodeStatus Wait::onStart() {
  int ms = 0;
  getInput("msec", ms);
  end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Wait::onRunning() {
  if (std::chrono::steady_clock::now() >= end_time_)
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}
void Wait::onHalted() {}

// ─── Hold_depth ─────────────────────────────────────────────────
BT::NodeStatus Hold_depth::onStart() {
  double target = 1.4;
  getInput("target", target);
  // TODO: Set depth hold mode in controller
  return BT::NodeStatus::SUCCESS; // instant for slalom pass-through
}
BT::NodeStatus Hold_depth::onRunning() { return BT::NodeStatus::SUCCESS; }
void Hold_depth::onHalted() {}

// ─── Surface_in_float_area ──────────────────────────────────────
BT::NodeStatus Surface_in_float_area::onStart() {
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Surfacing inside floating area...");
  // TODO: Command controlled ascent to surface
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Surface_in_float_area::onRunning() { return BT::NodeStatus::SUCCESS; }
void Surface_in_float_area::onHalted() {}

// ─── Face_direction ─────────────────────────────────────────────
BT::NodeStatus Face_direction::onStart() {
  double dir = 0;
  getInput("dir", dir);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Face_direction::onRunning() { return BT::NodeStatus::SUCCESS; }
void Face_direction::onHalted() {}

// ─── Follow_path ────────────────────────────────────────────────
BT::NodeStatus Follow_path::onStart() { return BT::NodeStatus::RUNNING; }
BT::NodeStatus Follow_path::onRunning() { return BT::NodeStatus::SUCCESS; }
void Follow_path::onHalted() {}

// ─── Circle_around ──────────────────────────────────────────────
BT::NodeStatus Circle_around::onStart() { return BT::NodeStatus::RUNNING; }
BT::NodeStatus Circle_around::onRunning() { return BT::NodeStatus::SUCCESS; }
void Circle_around::onHalted() {}

} // namespace shrub
