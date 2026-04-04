// SHRUB v3 — Manipulation action nodes
// Wraps hardware actuators: gripper/claw, marker dropper, torpedo tubes.
// TODO: Replace with actual ROS 2 service calls to your actuator drivers.

#include <bt_mission/shrub_nodes.hpp>

namespace shrub {

// ─── Grab_object ────────────────────────────────────────────────
BT::NodeStatus Grab_object::onStart() {
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Claw closing — grabbing object...");
  // TODO: Call gripper action server → close claw
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Grab_object::onRunning() {
  // TODO: Check gripper feedback — object grasped?
  // From 4.2: "must be captured and constrained" for full points
  return BT::NodeStatus::SUCCESS;
}
void Grab_object::onHalted() {
  // TODO: Stop gripper motion
}

// ─── Release_object ─────────────────────────────────────────────
BT::NodeStatus Release_object::tick() {
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Releasing object...");
  // TODO: Call gripper service → open claw
  // From 4.2: "must fall free from the vehicle" for full points
  return BT::NodeStatus::SUCCESS;
}

// ─── Drop_marker ────────────────────────────────────────────────
BT::NodeStatus Drop_marker::tick() {
  std::string id;
  getInput("id", id);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Dropping marker %s", id.c_str());
  // TODO: Call marker dropper service
  // Markers must fit 2.0" sq × 6" long, ≤2.0 lbs each
  return BT::NodeStatus::SUCCESS;
}

// ─── Fire_torpedo ───────────────────────────────────────────────
BT::NodeStatus Fire_torpedo::tick() {
  std::string tube;
  getInput("tube", tube);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Firing torpedo tube %s", tube.c_str());
  // TODO: Call torpedo launcher service
  // Torpedoes must fit 2.0" sq × 6" long, ≤2.0 lbs each
  return BT::NodeStatus::SUCCESS;
}

} // namespace shrub
