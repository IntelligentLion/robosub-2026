// SHRUB v4 — Manipulation action nodes.
//
// These actuator wrappers are stubs because no marker/torpedo/gripper driver
// exists yet. Each updates the relevant blackboard counter so the surrounding
// BT logic (MarkersRemaining, TorpedoesRemaining, object counters) advances
// realistically during dry runs. Replace each TODO with a real ROS service or
// action call when the driver lands.

#include <bt_mission/shrub_nodes.hpp>
#include <bt_mission/mission_io.hpp>

#include <algorithm>

namespace shrub {

namespace {
inline rclcpp::Logger lg() { return rclcpp::get_logger("shrub"); }
}  // namespace

// ─── Markers ────────────────────────────────────────────────────────
BT::NodeStatus ReleaseMarker::tick() {
  RCLCPP_INFO(lg(), "[manip] release marker (TODO: marker dropper driver)");
  if (auto bb = config().blackboard) {
    int remaining = 2;
    bb->get<int>("markers_remaining", remaining);
    bb->set("markers_remaining", std::max(0, remaining - 1));
    bb->set("marker_dropped", true);
  }
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus RetryMarkerDrop::tick() {
  RCLCPP_WARN(lg(), "[manip] retry marker drop");
  if (auto bb = config().blackboard) bb->set("marker_in_bin", true);
  return BT::NodeStatus::SUCCESS;
}

// ─── Magnetic interaction tool ──────────────────────────────────────
BT::NodeStatus ActivateTool::tick() {
  RCLCPP_INFO(lg(), "[manip] activate magnetic tool (TODO: tool driver)");
  if (auto bb = config().blackboard) bb->set("light_off", true);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus RetryInteraction::tick() {
  RCLCPP_WARN(lg(), "[manip] retry magnetic interaction");
  return BT::NodeStatus::SUCCESS;
}

// ─── Torpedoes ──────────────────────────────────────────────────────
BT::NodeStatus ArmLauncher::tick() {
  int tube = 1;
  getInput("tube_id", tube);
  RCLCPP_INFO(lg(), "[manip] arm launcher tube %d (TODO: launcher driver)", tube);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus LaunchTorpedo::tick() {
  RCLCPP_INFO(lg(), "[manip] LAUNCH TORPEDO (TODO: launcher driver)");
  if (auto bb = config().blackboard) {
    int remaining = 2;
    bb->get<int>("torpedoes_remaining", remaining);
    bb->set("torpedoes_remaining", std::max(0, remaining - 1));
    bb->set("torpedo_fired", true);
    bb->set("torpedo_hit", true);  // optimistic; perception update would correct
  }
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus RetryShot::tick() {
  RCLCPP_WARN(lg(), "[manip] retry shot");
  return BT::NodeStatus::SUCCESS;
}

// ─── Octagon: object release into basket ────────────────────────────
BT::NodeStatus ReleaseObject::tick() {
  RCLCPP_INFO(lg(), "[manip] release object (TODO: gripper driver)");
  if (auto bb = config().blackboard) {
    int delivered = 0;
    bb->get<int>("objects_delivered", delivered);
    bb->set("objects_delivered", delivered + 1);
    bb->set("object_delivered", true);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace shrub
