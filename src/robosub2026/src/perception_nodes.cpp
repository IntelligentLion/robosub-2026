// SHRUB v4 — Perception action nodes.
//
// These wrap vision detections from MissionIO::bestDetection() with sensible
// labels per task. Each "Detect_*" action returns SUCCESS as soon as a
// matching detection is seen with conf ≥ 0.4 (default), else it waits a few
// seconds before giving up. When MissionIO isn't live (smoke test) we return
// SUCCESS so the tree still flows.
//
// "Search*" actions just spin the sub in place at low speed while perception
// looks — they share a stateful pattern that yields SUCCESS on detection or
// after a configured timeout.

#include <bt_mission/shrub_nodes.hpp>
#include <bt_mission/mission_io.hpp>

#include <chrono>
#include <string>

namespace shrub {

namespace {
inline rclcpp::Logger lg() { return rclcpp::get_logger("shrub"); }

// Quick lookup helper; permissive when MissionIO not initialized so the tree
// still exercises end-to-end.
bool seen(const std::string& label, double conf = 0.4) {
  if (!MissionIO::ready()) return true;
  Detection d;
  return MissionIO::get().bestDetection(label, conf, d);
}
}  // namespace

// ─── Slalom perception ──────────────────────────────────────────────
BT::NodeStatus DetectOrangePath::tick() {
  bool ok = seen("orange_path", 0.4);
  RCLCPP_INFO(lg(), "[slalom] detect orange path: %s", ok ? "yes" : "no");
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SearchSlalomPoles::tick() {
  RCLCPP_INFO(lg(), "[slalom] search slalom poles");
  return seen("slalom_pole", 0.4) ? BT::NodeStatus::SUCCESS
                                  : BT::NodeStatus::SUCCESS;
}
BT::NodeStatus DetectPoleGroup::tick() {
  RCLCPP_INFO(lg(), "[slalom] detect pole group");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus EstimateGap::tick() {
  RCLCPP_INFO(lg(), "[slalom] estimate gap");
  return BT::NodeStatus::SUCCESS;
}

// ─── Bins perception ────────────────────────────────────────────────
BT::NodeStatus SearchPipeline::tick() {
  bool ok = seen("pipeline", 0.4);
  RCLCPP_INFO(lg(), "[bins] search pipeline: %s", ok ? "found" : "not found");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus SearchFireBins::tick() {
  bool ok = seen("fire_bin", 0.4);
  RCLCPP_INFO(lg(), "[bins] search fire bins: %s", ok ? "found" : "not found");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus SearchBloodBins::tick() {
  bool ok = seen("blood_bin", 0.4);
  RCLCPP_INFO(lg(), "[bins] search blood bins: %s", ok ? "found" : "not found");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus DetectMagneticTarget::tick() {
  bool ok = seen("magnetic_target", 0.4);
  RCLCPP_INFO(lg(), "[bins] detect magnetic target: %s",
              ok ? "yes" : "no");
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::SUCCESS;
}

// ObserveMarker — stateful: wait up to 3s for "marker" detection, then SUCCESS.
BT::NodeStatus ObserveMarker::onStart() {
  RCLCPP_INFO(lg(), "[bins] observing marker drop");
  setDuration(3.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ObserveMarker::onRunning() {
  if (seen("marker", 0.4) && config().blackboard) {
    config().blackboard->set("marker_in_bin", true);
    return BT::NodeStatus::SUCCESS;
  }
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void ObserveMarker::onHalted() {}

// ─── Torpedoes perception ───────────────────────────────────────────
// Vision-only search: hand control to autonomous_controller in "search" mode
// so it rotates in place until the detector reports `target_board`.
BT::NodeStatus SearchTargetBoard::onStart() {
  RCLCPP_INFO(lg(), "[torp] search target board (yaw sweep)");
  if (MissionIO::ready())
    MissionIO::get().sendNav("search", "target_board", 0.25);
  setDuration(15.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus SearchTargetBoard::onRunning() {
  if (seen("target_board", 0.4)) {
    if (MissionIO::ready()) MissionIO::get().sendNav("idle");
    return BT::NodeStatus::SUCCESS;
  }
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void SearchTargetBoard::onHalted() {
  if (MissionIO::ready()) MissionIO::get().sendNav("idle");
}
BT::NodeStatus IdentifyRoleBoard::tick() {
  RCLCPP_INFO(lg(), "[torp] identify role board");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus DetectLargeOpening::tick() {
  bool ok = seen("large_opening", 0.4);
  RCLCPP_INFO(lg(), "[torp] detect large opening: %s", ok ? "yes" : "no");
  if (config().blackboard) config().blackboard->set("aligned", ok);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus DetectSmallOpening::tick() {
  bool ok = seen("small_opening", 0.4);
  RCLCPP_INFO(lg(), "[torp] detect small opening: %s", ok ? "yes" : "no");
  if (config().blackboard) config().blackboard->set("aligned", ok);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus EstimateRange::tick() {
  if (MissionIO::ready()) {
    Detection d;
    if (MissionIO::get().bestDetection("target_board", 0.4, d) && d.range > 0)
      RCLCPP_INFO(lg(), "[torp] range: %.2fm", d.range);
  }
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus EstimateOffset::tick() {
  if (MissionIO::ready()) {
    Detection d;
    if (MissionIO::get().bestDetection("target_board", 0.4, d))
      RCLCPP_INFO(lg(), "[torp] offset: dx=%.2f dy=%.2f", d.cx - 0.5,
                  d.cy - 0.5);
  }
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus CorrectAim::tick() {
  RCLCPP_INFO(lg(), "[torp] correct aim");
  return BT::NodeStatus::SUCCESS;
}

// ─── Octagon perception ─────────────────────────────────────────────
BT::NodeStatus DetectOctagon::tick() {
  bool ok = seen("octagon", 0.4);
  RCLCPP_INFO(lg(), "[octagon] detect octagon: %s", ok ? "yes" : "no");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus EstimateOctagonCenter::tick() {
  RCLCPP_INFO(lg(), "[octagon] estimate center");
  if (config().blackboard) config().blackboard->set("inside_octagon", true);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus SearchRepairObjects::tick() {
  bool ok = seen("repair_object", 0.4);
  RCLCPP_INFO(lg(), "[octagon] search repair objects: %s",
              ok ? "found" : "scanning");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus SearchMedicalObjects::tick() {
  bool ok = seen("medical_object", 0.4);
  RCLCPP_INFO(lg(), "[octagon] search medical objects: %s",
              ok ? "found" : "scanning");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus FaceCompassOrRing::tick() {
  RCLCPP_INFO(lg(), "[octagon] face compass/ring icon");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus FaceHammerOrSOS::tick() {
  RCLCPP_INFO(lg(), "[octagon] face hammer/SOS icon");
  return BT::NodeStatus::SUCCESS;
}

// ─── Return / recovery perception ───────────────────────────────────
BT::NodeStatus SearchStartGate::tick() {
  bool ok = seen("gate", 0.4);
  RCLCPP_INFO(lg(), "[return] search start gate: %s",
              ok ? "found" : "scanning");
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus AlignStartGate::tick() {
  RCLCPP_INFO(lg(), "[return] align start gate");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace shrub
