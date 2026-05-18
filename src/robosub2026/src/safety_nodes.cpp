// SHRUB v3 — Safety condition nodes
#include <bt_mission/shrub_nodes.hpp>

namespace shrub {

BT::NodeStatus IsBatteryOk::tick() {
  double min_pct = 20.0, cur = 100.0;
  getInput("min_pct", min_pct);
  getInput("battery_pct", cur);
  return (cur >= min_pct) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsLeakDetected::tick() {
  bool leak = false;
  getInput("leak", leak);
  return leak ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsDepthSafe::tick() {
  double max_d = 1.9, min_d = 0.0, cur = 0.0;
  getInput("max_depth", max_d);
  getInput("min_depth", min_d);
  getInput("depth", cur);
  return (cur <= max_d && cur >= min_d) ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsInsideFloatArea::tick() {
  bool inside = false;
  getInput("val", inside);
  return inside ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsTimeRemaining::tick() {
  double min_sec = 30.0, start = 0.0;
  getInput("min_sec", min_sec);
  getInput("start_time", start);
  // TODO: inject actual ROS clock. For now use wall clock.
  double now = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  double elapsed = now - start;
  double remaining = 900.0 - elapsed;  // 15 min performance window
  return (remaining > min_sec) ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
}

} // namespace shrub
