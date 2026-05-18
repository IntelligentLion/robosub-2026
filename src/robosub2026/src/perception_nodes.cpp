// SHRUB v3 — Perception action nodes
// Each node wraps a ROS 2 service/action call to your perception pipeline.
// TODO: Replace placeholders with actual calls to your YOLOv8/OpenCV/sonar nodes.

#include <bt_mission/shrub_nodes.hpp>

namespace shrub {

// All perception nodes follow the same stub pattern.
// In production: onStart() sends a detection request,
// onRunning() checks for results, onHalted() cancels.

// Macro for the repeated stub implementation
#define IMPL_DETECT_NODE(ClassName, LogMsg)                             \
BT::NodeStatus ClassName::onStart() {                                   \
  RCLCPP_INFO(rclcpp::get_logger("shrub"), LogMsg);                     \
  return BT::NodeStatus::RUNNING;                                       \
}                                                                       \
BT::NodeStatus ClassName::onRunning() {                                 \
  /* TODO: Check detection result from perception service */            \
  return BT::NodeStatus::SUCCESS;                                       \
}                                                                       \
void ClassName::onHalted() {}

IMPL_DETECT_NODE(Detect_gate, "Detecting gate...")
IMPL_DETECT_NODE(Detect_animal_on_gate, "Detecting animal on gate...")
IMPL_DETECT_NODE(Detect_animal_image, "Detecting animal image...")
IMPL_DETECT_NODE(Detect_pinger, "Listening for pinger...")
IMPL_DETECT_NODE(Detect_float_area_below, "Detecting floating area from below...")
IMPL_DETECT_NODE(Confirm_overhead, "Confirming floating area overhead...")
IMPL_DETECT_NODE(Detect_buoy, "Detecting buoy...")
IMPL_DETECT_NODE(Detect_bin_below, "Detecting bin from above...")
IMPL_DETECT_NODE(Detect_object, "Detecting object...")
IMPL_DETECT_NODE(Detect_slalom_pipes, "Detecting slalom pipes...")
IMPL_DETECT_NODE(Detect_task_board, "Detecting task board...")
IMPL_DETECT_NODE(Detect_opening, "Detecting torpedo opening...")
IMPL_DETECT_NODE(Detect_path_marker, "Detecting path marker...")
IMPL_DETECT_NODE(Detect_vertical_marker, "Detecting vertical marker...")

#undef IMPL_DETECT_NODE

// ─── Alignment nodes (visual servoing) ──────────────────────────

BT::NodeStatus Align_to::onStart() {
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Aligning to target...");
  // TODO: Start visual servoing loop — center detection in frame
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Align_to::onRunning() { return BT::NodeStatus::SUCCESS; }
void Align_to::onHalted() {}

BT::NodeStatus Align_above::onStart() {
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Aligning above bin target...");
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Align_above::onRunning() { return BT::NodeStatus::SUCCESS; }
void Align_above::onHalted() {}

BT::NodeStatus Align_to_opening::onStart() { return BT::NodeStatus::RUNNING; }
BT::NodeStatus Align_to_opening::onRunning() { return BT::NodeStatus::SUCCESS; }
void Align_to_opening::onHalted() {}

BT::NodeStatus Align_to_basket::onStart() { return BT::NodeStatus::RUNNING; }
BT::NodeStatus Align_to_basket::onRunning() { return BT::NodeStatus::SUCCESS; }
void Align_to_basket::onHalted() {}

BT::NodeStatus Center_beneath::onStart() { return BT::NodeStatus::RUNNING; }
BT::NodeStatus Center_beneath::onRunning() { return BT::NodeStatus::SUCCESS; }
void Center_beneath::onHalted() {}

BT::NodeStatus Approach_and_touch::onStart() {
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Approaching and touching target...");
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Approach_and_touch::onRunning() { return BT::NodeStatus::SUCCESS; }
void Approach_and_touch::onHalted() {}

} // namespace shrub
