// SHRUB v4 — Condition nodes for the 2026 mission tree.
//
// Conditions are kept side-effect free: each one reads either the blackboard
// (with a safe default if the key is missing) or live MissionIO state, and
// returns SUCCESS/FAILURE. None of them block.

#include <bt_mission/shrub_nodes.hpp>
#include <bt_mission/mission_io.hpp>

#include <chrono>
#include <cstdlib>
#include <string>

namespace shrub {

namespace {

// Tiny helpers so each condition stays one line. Each takes a Blackboard::Ptr
// instead of TreeNode* because TreeNode::config() is a protected member —
// callable from inside a node's tick() (where we got the blackboard), but not
// from a free function. So callers pass `config().blackboard` in directly.
template <typename T>
T bbGet(const BT::Blackboard::Ptr& bb, const std::string& key, T deflt) {
  T v = deflt;
  if (bb) (void)bb->get<T>(key, v);
  return v;
}

bool bbHasString(const BT::Blackboard::Ptr& bb,
                 const std::string& key, const std::string& want) {
  std::string v;
  if (bb) (void)bb->get<std::string>(key, v);
  return v == want;
}

double nowSec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

// ─── Coin flip — chosen by run config or random fallback ────────────
BT::NodeStatus CoinflipNormal::tick() {
  std::string cf = bbGet<std::string>(config().blackboard, "coin_flip", "normal");
  return (cf == "normal") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus CoinflipBackward::tick() {
  std::string cf = bbGet<std::string>(config().blackboard, "coin_flip", "normal");
  return (cf == "backward") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── Localization ───────────────────────────────────────────────────
BT::NodeStatus LocalizationStable::tick() {
  if (!MissionIO::ready()) return BT::NodeStatus::SUCCESS;  // permissive when MIO not up
  double x, y, z, yaw;
  return MissionIO::get().pose(x, y, z, yaw) ? BT::NodeStatus::SUCCESS
                                             : BT::NodeStatus::FAILURE;
}
BT::NodeStatus LocalizationLost::tick() {
  if (!MissionIO::ready()) return BT::NodeStatus::FAILURE;
  double x, y, z, yaw;
  return MissionIO::get().pose(x, y, z, yaw) ? BT::NodeStatus::FAILURE
                                             : BT::NodeStatus::SUCCESS;
}

// ─── Gate detection family ──────────────────────────────────────────
BT::NodeStatus GateDetected::tick() {
  if (!MissionIO::ready()) return BT::NodeStatus::SUCCESS;
  Detection d;
  return MissionIO::get().bestDetection("gate", 0.4, d) ? BT::NodeStatus::SUCCESS
                                                        : BT::NodeStatus::FAILURE;
}
BT::NodeStatus GateSearchTimeout::tick() {
  double timeout = 30.0;
  getInput("timeout_sec", timeout);
  double start = bbGet<double>(config().blackboard, "gate_search_start", 0.0);
  if (start <= 0.0) return BT::NodeStatus::FAILURE;  // search not started yet
  return (nowSec() - start > timeout) ? BT::NodeStatus::SUCCESS
                                      : BT::NodeStatus::FAILURE;
}

// ─── Role detection ─────────────────────────────────────────────────
BT::NodeStatus RoleSignsVisible::tick() {
  if (!MissionIO::ready()) return BT::NodeStatus::SUCCESS;
  Detection d;
  return (MissionIO::get().bestDetection("role_sign", 0.4, d) ||
          MissionIO::get().bestDetection("survey_repair", 0.4, d) ||
          MissionIO::get().bestDetection("search_rescue", 0.4, d))
             ? BT::NodeStatus::SUCCESS
             : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SurveyRepairDetected::tick() {
  if (!MissionIO::ready()) return BT::NodeStatus::FAILURE;
  Detection d;
  return MissionIO::get().bestDetection("survey_repair", 0.4, d)
             ? BT::NodeStatus::SUCCESS
             : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SearchRescueDetected::tick() {
  if (!MissionIO::ready()) return BT::NodeStatus::FAILURE;
  Detection d;
  return MissionIO::get().bestDetection("search_rescue", 0.4, d)
             ? BT::NodeStatus::SUCCESS
             : BT::NodeStatus::FAILURE;
}
// RandomRoleAssignment: pseudo-random fallback — always assigns (SUCCESS),
// the assignment itself happens in ReadAssignedRole.
BT::NodeStatus RandomRoleAssignment::tick() { return BT::NodeStatus::SUCCESS; }

// ─── Stability / orientation flags ──────────────────────────────────
BT::NodeStatus StyleModeEnabled::tick() {
  return bbGet<bool>(config().blackboard, "style_enabled", false) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
}
BT::NodeStatus VehicleStable::tick() {
  // Default-true: if a stability monitor publishes it can override via BB.
  return bbGet<bool>(config().blackboard, "vehicle_stable", true) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
}
BT::NodeStatus OrientationReached::tick() {
  return bbGet<bool>(config().blackboard, "orientation_reached", true) ? BT::NodeStatus::SUCCESS
                                                        : BT::NodeStatus::FAILURE;
}

// ─── Gate transit flags ─────────────────────────────────────────────
BT::NodeStatus DividerVerified::tick() {
  return bbGet<bool>(config().blackboard, "divider_verified", true) ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::FAILURE;
}
BT::NodeStatus NoCollisionRisk::tick() {
  return bbGet<bool>(config().blackboard, "obstacle_detected", false) ? BT::NodeStatus::FAILURE
                                                       : BT::NodeStatus::SUCCESS;
}
BT::NodeStatus GateCleared::tick() {
  return bbGet<bool>(config().blackboard, "gate_cleared", true) ? BT::NodeStatus::SUCCESS
                                                 : BT::NodeStatus::FAILURE;
}
BT::NodeStatus GateRedRight::tick() {
  return bbHasString(config().blackboard, "gate_red_side", "right") ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::FAILURE;
}
BT::NodeStatus GateRedLeft::tick() {
  return bbHasString(config().blackboard, "gate_red_side", "left") ? BT::NodeStatus::SUCCESS
                                                    : BT::NodeStatus::FAILURE;
}
BT::NodeStatus PlaneCrossed::tick() {
  return bbGet<bool>(config().blackboard, "plane_crossed", true) ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
}
BT::NodeStatus InsideBounds::tick() {
  return bbGet<bool>(config().blackboard, "inside_bounds", true) ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
}

// ─── Role flags ─────────────────────────────────────────────────────
BT::NodeStatus RoleSurveyRepair::tick() {
  return bbHasString(config().blackboard, "role", "survey_repair") ? BT::NodeStatus::SUCCESS
                                                    : BT::NodeStatus::FAILURE;
}
BT::NodeStatus RoleSearchRescue::tick() {
  return bbHasString(config().blackboard, "role", "search_rescue") ? BT::NodeStatus::SUCCESS
                                                    : BT::NodeStatus::FAILURE;
}

// ─── Drop / bin / interaction ───────────────────────────────────────
BT::NodeStatus ValidDropAltitude::tick() {
  double min_alt = 0.3;
  getInput("min_altitude_m", min_alt);
  // We treat altitude as max(min_alt, depth_above_floor). The depth sensor
  // gives depth-below-surface; conservative pass unless an explicit "altitude"
  // blackboard variable is set by a future altimeter publisher.
  double alt = bbGet<double>(config().blackboard, "altitude_m", min_alt + 0.5);
  return (alt >= min_alt) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus MarkerInBin::tick() {
  return bbGet<bool>(config().blackboard, "marker_in_bin", true) ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
}
BT::NodeStatus MarkersRemaining::tick() {
  return bbGet<int>(config().blackboard, "markers_remaining", 2) > 0 ? BT::NodeStatus::SUCCESS
                                                      : BT::NodeStatus::FAILURE;
}
BT::NodeStatus LightOff::tick() {
  return bbGet<bool>(config().blackboard, "light_off", true) ? BT::NodeStatus::SUCCESS
                                              : BT::NodeStatus::FAILURE;
}
BT::NodeStatus AlignmentConfirmed::tick() {
  return bbGet<bool>(config().blackboard, "aligned", true) ? BT::NodeStatus::SUCCESS
                                            : BT::NodeStatus::FAILURE;
}

// ─── Torpedo flags ──────────────────────────────────────────────────
BT::NodeStatus TorpedoHit::tick() {
  return bbGet<bool>(config().blackboard, "torpedo_hit", true) ? BT::NodeStatus::SUCCESS
                                                : BT::NodeStatus::FAILURE;
}
BT::NodeStatus TorpedoesRemaining::tick() {
  return bbGet<int>(config().blackboard, "torpedoes_remaining", 2) > 0 ? BT::NodeStatus::SUCCESS
                                                        : BT::NodeStatus::FAILURE;
}

// ─── Octagon / basket flags ─────────────────────────────────────────
BT::NodeStatus InsideOctagon::tick() {
  return bbGet<bool>(config().blackboard, "inside_octagon", true) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
}
BT::NodeStatus CorrectBasket::tick() {
  return bbGet<bool>(config().blackboard, "correct_basket", true) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
}
BT::NodeStatus ObjectDelivered::tick() {
  return bbGet<bool>(config().blackboard, "object_delivered", true) ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::FAILURE;
}
BT::NodeStatus OneItemInBasket::tick() {
  return bbGet<int>(config().blackboard, "objects_delivered", 0) == 1 ? BT::NodeStatus::SUCCESS
                                                       : BT::NodeStatus::FAILURE;
}
BT::NodeStatus TwoItemsInBasket::tick() {
  return bbGet<int>(config().blackboard, "objects_delivered", 0) >= 2 ? BT::NodeStatus::SUCCESS
                                                       : BT::NodeStatus::FAILURE;
}

// ─── Mission / failure flags ────────────────────────────────────────
BT::NodeStatus MissionFinished::tick() {
  return bbGet<bool>(config().blackboard, "mission_complete", true) ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::FAILURE;
}
BT::NodeStatus DepthUnstable::tick() {
  return bbGet<bool>(config().blackboard, "depth_unstable", false) ? BT::NodeStatus::SUCCESS
                                                    : BT::NodeStatus::FAILURE;
}
BT::NodeStatus ObstacleDetected::tick() {
  return bbGet<bool>(config().blackboard, "obstacle_detected", false) ? BT::NodeStatus::SUCCESS
                                                       : BT::NodeStatus::FAILURE;
}
BT::NodeStatus TaskTimeout::tick() {
  double timeout = 120.0;
  getInput("timeout_sec", timeout);
  double start = bbGet<double>(config().blackboard, "task_start_time", 0.0);
  if (start <= 0.0) return BT::NodeStatus::FAILURE;
  return (nowSec() - start > timeout) ? BT::NodeStatus::SUCCESS
                                      : BT::NodeStatus::FAILURE;
}
BT::NodeStatus CriticalFailure::tick() {
  return bbGet<bool>(config().blackboard, "critical_failure", false) ? BT::NodeStatus::SUCCESS
                                                      : BT::NodeStatus::FAILURE;
}

}  // namespace shrub
