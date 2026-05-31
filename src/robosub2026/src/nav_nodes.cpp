// SHRUB v4 — Navigation, movement, and initialization action nodes.
//
// Layout
//   TimedAction helpers   — shared base for stateful timed actions.
//   Init actions          — VerifyX/Load/Zero/Wait/SubmergeToMissionDepth.
//   Gate motion           — Rotate180, HoldDepth/Heading, YawSweep, ForwardSpiral, MoveForward, …
//   Slalom motion         — AlignThroughGap, TransitGap, FollowPath, etc.
//   Bins motion           — NavigateBin*, FollowPathMarker, etc.
//   Torpedo motion        — NavigateToPinger, RealignVehicle, …
//   Octagon motion        — NavigateToOctagon, EnterOctagon, AscendSlowly, BreakSurface, …
//   Return / recovery     — Surface, SurfaceSafely, NavigateToStart, RecoverDepth, BackAway, …
//
// Convention for stateful timed actions:
//   onStart()   send a MovementCommand (or NavigationCommand) and setDuration(...)
//   onRunning() return SUCCESS once the deadline passes (open-loop)
//   onHalted()  send "stop"
//
// All hardware-bound actions wrap MissionIO::ready() so the executor still ticks
// the tree end-to-end even if mission_io's publishers aren't connected yet.

#include <bt_mission/shrub_nodes.hpp>
#include <bt_mission/mission_io.hpp>

#include <chrono>
#include <cmath>

namespace shrub {

// ─── TimedAction helpers ────────────────────────────────────────────
void TimedAction::setDuration(double seconds) {
  if (seconds < 0) seconds = 0;
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::milliseconds(static_cast<int>(seconds * 1000));
}
bool TimedAction::deadlinePassed() const {
  return std::chrono::steady_clock::now() >= deadline_;
}

namespace {
inline rclcpp::Logger lg() { return rclcpp::get_logger("shrub"); }
inline void stop() {
  if (MissionIO::ready()) MissionIO::get().stop();
}
inline void move(const std::string& cmd, double speed = 0.0, double duration = 0.0) {
  if (MissionIO::ready()) MissionIO::get().sendMovement(cmd, speed, duration);
}
inline void nav(const std::string& mode, const std::string& label = "",
                double speed = 0.0, double approach = 0.0, double yaw = 0.0,
                double tx = 0.0, double ty = 0.0, double tz = 0.0) {
  if (MissionIO::ready())
    MissionIO::get().sendNav(mode, label, speed, approach, yaw, tx, ty, tz);
}
}  // namespace

// ═════════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═════════════════════════════════════════════════════════════════════
#define LOG_SYNC_OK(Cls, msg)                                        \
  BT::NodeStatus Cls::tick() {                                       \
    RCLCPP_INFO(lg(), msg);                                          \
    return BT::NodeStatus::SUCCESS;                                  \
  }

// Verify-* actions: today no driver reports back, so we log + pass. When real
// self-tests exist (e.g. thruster spin test, IMU bias check), wire them here.
LOG_SYNC_OK(VerifyThrusters,       "[init] verify thrusters (stub)")
LOG_SYNC_OK(VerifyIMU,             "[init] verify IMU (stub)")
LOG_SYNC_OK(VerifyDepthSensor,     "[init] verify depth sensor (stub)")
LOG_SYNC_OK(VerifyCameras,         "[init] verify cameras (stub)")
LOG_SYNC_OK(VerifyManipulators,    "[init] verify manipulators (stub)")
LOG_SYNC_OK(VerifyTorpedoLauncher, "[init] verify torpedo launcher (stub)")
LOG_SYNC_OK(LoadMissionParameters, "[init] load mission parameters")
LOG_SYNC_OK(ZeroStateEstimator,    "[init] zero state estimator")
LOG_SYNC_OK(WaitForStartSignal,    "[init] start signal received (passthrough)")

// Submerge to mission depth — open-loop submerge for ~6s with depth gate.
BT::NodeStatus SubmergeToMissionDepth::onStart() {
  double target = 1.5;
  getInput("depth_m", target);
  RCLCPP_INFO(lg(), "[init] submerge → %.2fm", target);
  move("submerge", 0.4, 6.0);
  setDuration(6.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus SubmergeToMissionDepth::onRunning() {
  // If MissionIO depth is live and we hit target, stop early.
  if (MissionIO::ready()) {
    double d = MissionIO::get().depth();
    double target = 1.5;
    getInput("depth_m", target);
    if (d > 0 && d >= target - 0.1) {
      move("depth_hold");
      return BT::NodeStatus::SUCCESS;
    }
  }
  if (deadlinePassed()) {
    move("depth_hold");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
void SubmergeToMissionDepth::onHalted() { stop(); }

// ═════════════════════════════════════════════════════════════════════
// GATE — short navigation primitives + hold/stab actions
// ═════════════════════════════════════════════════════════════════════
LOG_SYNC_OK(FaceGateEstimate,            "[gate] face gate estimate")
LOG_SYNC_OK(ContinueMission,             "[gate] continue mission")
LOG_SYNC_OK(SearchGateReverse,           "[gate] search behind (reverse start)")
LOG_SYNC_OK(EnableFrontCamera,           "[gate] front camera enabled")
LOG_SYNC_OK(LockGateTarget,              "[gate] gate target locked")
LOG_SYNC_OK(EstimateGateDivider,         "[gate] estimate divider")
LOG_SYNC_OK(EstimateGateOpening,         "[gate] estimate opening")
LOG_SYNC_OK(MaintainCenterline,          "[gate] maintain centerline")
LOG_SYNC_OK(MaintainDesiredDepth,        "[gate] maintain desired depth")
LOG_SYNC_OK(OffsetToSelectedSide,        "[gate] offset to selected side")
LOG_SYNC_OK(MaintainGateClearance,       "[gate] maintain clearance")
LOG_SYNC_OK(RecenterVehicle,             "[gate] recenter")
LOG_SYNC_OK(RetryGateTransit,            "[gate] retry transit")
LOG_SYNC_OK(RecordGateSide,              "[gate] record gate side")
LOG_SYNC_OK(ReestablishHeading,          "[gate] reestablish heading")
LOG_SYNC_OK(SearchPathMarker,            "[gate] search path marker")
LOG_SYNC_OK(RetryGateSearch,             "[gate] retry search (loop hint)")
LOG_SYNC_OK(SkipStyle,                   "[gate] skip style maneuver")
LOG_SYNC_OK(SetRoleSurveyRepair,         "[gate] role := survey_repair")
LOG_SYNC_OK(SelectSurveyRepairGateSide,  "[gate] selecting SR gate side")
LOG_SYNC_OK(SetRoleSearchRescue,         "[gate] role := search_rescue")
LOG_SYNC_OK(SelectSearchRescueGateSide,  "[gate] selecting SeR gate side")
LOG_SYNC_OK(ReadAssignedRole,            "[gate] role read from coin assignment")
LOG_SYNC_OK(StoreRoleVariable,           "[gate] role stored")
LOG_SYNC_OK(HoldRoll,                    "[gate] hold roll")
LOG_SYNC_OK(HoldPitch,                   "[gate] hold pitch")

// HoldDepth — set depth_hold mode (open-loop on the thruster side).
BT::NodeStatus HoldDepth::tick() {
  double d = 0.0;
  getInput("depth_m", d);
  RCLCPP_INFO(lg(), "[gate] hold depth %.2fm", d);
  move("depth_hold");
  return BT::NodeStatus::SUCCESS;
}

// HoldHeading — hand off to autonomous_controller in heading_hold mode.
BT::NodeStatus HoldHeading::tick() {
  double deg = 0.0;
  getInput("heading_deg", deg);
  double rad = deg * M_PI / 180.0;
  RCLCPP_INFO(lg(), "[gate] hold heading %.1f°", deg);
  nav("heading_hold", "", 0.3, 0.0, rad);
  return BT::NodeStatus::SUCCESS;
}

// StopMotion — used in collision-recovery sequences.
BT::NodeStatus StopMotion::tick() {
  stop();
  return BT::NodeStatus::SUCCESS;
}

// Reduce velocity — communicates a slow-down preference (BB hint) and asks
// thrusters to hold depth while controllers ease off.
BT::NodeStatus ReduceVelocity::tick() {
  double v = 0.3;
  getInput("target_speed_mps", v);
  if (auto bb = config().blackboard) bb->set("desired_speed_mps", v);
  RCLCPP_INFO(lg(), "[gate] reduce velocity to %.2fm/s", v);
  move("depth_hold");
  return BT::NodeStatus::SUCCESS;
}

// Rotate180 — open-loop yaw for ~4s.
BT::NodeStatus Rotate180::onStart() {
  RCLCPP_INFO(lg(), "[gate] rotate 180°");
  move("rotate_cw", 0.4, 4.0);
  setDuration(4.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Rotate180::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void Rotate180::onHalted() { stop(); }

// YawSweep — small left-right scan to locate the gate.
BT::NodeStatus YawSweep::onStart() {
  double sweep = 30.0;
  getInput("sweep_deg", sweep);
  RCLCPP_INFO(lg(), "[gate] yaw sweep ±%.1f°", sweep);
  // Estimate ~30°/s rotation, so total sweep cycle = 2*sweep/30
  setDuration(std::max(1.5, 2.0 * sweep / 30.0));
  move("rotate_cw", 0.25, 1.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus YawSweep::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void YawSweep::onHalted() { stop(); }

// ForwardSpiral — slow forward+rotate for 5s.
BT::NodeStatus ForwardSpiral::onStart() {
  RCLCPP_INFO(lg(), "[gate] forward spiral");
  move("surge_forward", 0.2, 5.0);
  setDuration(5.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ForwardSpiral::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void ForwardSpiral::onHalted() { stop(); }

// DepthModulation — bob up/down briefly to break perception ambiguity.
BT::NodeStatus DepthModulation::onStart() {
  RCLCPP_INFO(lg(), "[gate] depth modulation");
  move("submerge", 0.2, 1.5);
  setDuration(3.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus DepthModulation::onRunning() {
  if (deadlinePassed()) {
    move("depth_hold");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
void DepthModulation::onHalted() { stop(); }

// DeadReckonForward — surge for distance/speed seconds.
BT::NodeStatus DeadReckonForward::onStart() {
  double d = 2.0;
  getInput("distance_m", d);
  const double speed = 0.35;
  RCLCPP_INFO(lg(), "[gate] dead reckon %.1fm @%.2fm/s", d, speed);
  move("surge_forward", speed, d / speed);
  setDuration(d / speed);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus DeadReckonForward::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void DeadReckonForward::onHalted() { stop(); }

// MoveForward — short timed forward burst.
BT::NodeStatus MoveForward::onStart() {
  double d = 1.0;
  getInput("distance_m", d);
  const double speed = 0.3;
  RCLCPP_INFO(lg(), "[gate] move forward %.1fm", d);
  move("surge_forward", speed, d / speed);
  setDuration(d / speed);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus MoveForward::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void MoveForward::onHalted() { stop(); }

// AlignGateCenter — hand to autonomous_controller in track_object mode.
BT::NodeStatus AlignGateCenter::onStart() {
  RCLCPP_INFO(lg(), "[gate] aligning to gate center via vision");
  nav("track_object", "gate", 0.3, 0.8);
  setDuration(8.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus AlignGateCenter::onRunning() {
  if (MissionIO::ready()) {
    Detection d;
    if (MissionIO::get().bestDetection("gate", 0.5, d)) {
      const double ex = d.cx - 0.5;
      const double ey = d.cy - 0.5;
      if (std::abs(ex) < 0.08 && std::abs(ey) < 0.08) {
        nav("idle");
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  if (deadlinePassed()) {
    nav("idle");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
void AlignGateCenter::onHalted() { nav("idle"); }

// Style moves — open-loop primitives. Yaw via thruster, Roll/Pitch logged
// (no roll/pitch primitive in MovementCommand today; pure yaw is the safe one).
BT::NodeStatus Yaw90::onStart() {
  RCLCPP_INFO(lg(), "[gate] yaw 90°");
  move("rotate_cw", 0.4, 2.0);
  setDuration(2.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Yaw90::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void Yaw90::onHalted() { stop(); }

BT::NodeStatus Roll90::onStart() {
  RCLCPP_WARN(lg(), "[gate] roll 90° (no roll primitive — TODO: wire IMU+thruster mix)");
  setDuration(2.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Roll90::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void Roll90::onHalted() {}

BT::NodeStatus Pitch90::onStart() {
  RCLCPP_WARN(lg(), "[gate] pitch 90° (no pitch primitive — TODO: wire IMU+thruster mix)");
  setDuration(2.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Pitch90::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void Pitch90::onHalted() {}

// ExecuteStyle — schedule a 2x yaw spin for points.
BT::NodeStatus ExecuteStyle::onStart() {
  RCLCPP_INFO(lg(), "[gate] execute style: barrel-yaw");
  move("rotate_cw", 0.6, 3.0);
  setDuration(3.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ExecuteStyle::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void ExecuteStyle::onHalted() { stop(); }

// ForwardTransit — push through the gate.
BT::NodeStatus ForwardTransit::onStart() {
  RCLCPP_INFO(lg(), "[gate] forward transit through gate");
  move("surge_forward", 0.4, 4.0);
  setDuration(4.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ForwardTransit::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void ForwardTransit::onHalted() { stop(); }

// ═════════════════════════════════════════════════════════════════════
// SLALOM — motion primitives
// ═════════════════════════════════════════════════════════════════════
LOG_SYNC_OK(KeepRedPoleRight,   "[slalom] keep red pole on right")
LOG_SYNC_OK(KeepRedPoleLeft,    "[slalom] keep red pole on left")
LOG_SYNC_OK(ExitSlalom,         "[slalom] exit")
LOG_SYNC_OK(MaintainSlalomDepth,"[slalom] maintain slalom depth")
LOG_SYNC_OK(CorrectHeading,     "[slalom] correct heading")
LOG_SYNC_OK(ReenterSlalom,      "[slalom] reenter zone")

BT::NodeStatus FollowPath::onStart() {
  RCLCPP_INFO(lg(), "[slalom] follow path");
  nav("track_object", "path_marker", 0.3, 0.0);
  setDuration(6.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus FollowPath::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void FollowPath::onHalted() { nav("idle"); }

BT::NodeStatus AlignThroughGap::onStart() {
  RCLCPP_INFO(lg(), "[slalom] align through gap");
  nav("track_object", "slalom_gap", 0.3, 0.0);
  setDuration(4.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus AlignThroughGap::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void AlignThroughGap::onHalted() { nav("idle"); }

BT::NodeStatus TransitGap::onStart() {
  RCLCPP_INFO(lg(), "[slalom] transit gap");
  move("surge_forward", 0.35, 3.0);
  setDuration(3.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus TransitGap::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void TransitGap::onHalted() { stop(); }

// ═════════════════════════════════════════════════════════════════════
// BINS — motion primitives
// ═════════════════════════════════════════════════════════════════════
LOG_SYNC_OK(StationKeep,        "[bins] station keep")
LOG_SYNC_OK(EstimateBinCenter,  "[bins] estimate bin center")
LOG_SYNC_OK(TimeoutRecovery,    "[bins] timeout recovery")

BT::NodeStatus FollowPathMarker::onStart() {
  RCLCPP_INFO(lg(), "[bins] follow path marker");
  nav("track_object", "path_marker", 0.3, 0.0);
  setDuration(8.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus FollowPathMarker::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void FollowPathMarker::onHalted() { nav("idle"); }

BT::NodeStatus NavigateBin1::onStart() {
  RCLCPP_INFO(lg(), "[bins] navigate bin 1");
  nav("track_object", "bin1", 0.3, 0.5);
  setDuration(8.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus NavigateBin1::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void NavigateBin1::onHalted() { nav("idle"); }

BT::NodeStatus NavigateBin2::onStart() {
  RCLCPP_INFO(lg(), "[bins] navigate bin 2");
  nav("track_object", "bin2", 0.3, 0.5);
  setDuration(8.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus NavigateBin2::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void NavigateBin2::onHalted() { nav("idle"); }

BT::NodeStatus AlignInteractionTool::onStart() {
  RCLCPP_INFO(lg(), "[bins] align interaction tool");
  nav("track_object", "magnetic_target", 0.2, 0.0);
  setDuration(4.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus AlignInteractionTool::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void AlignInteractionTool::onHalted() { nav("idle"); }

BT::NodeStatus MoveToInteractionDistance::onStart() {
  double d = 0.1;
  getInput("distance_m", d);
  RCLCPP_INFO(lg(), "[bins] close to %.2fm", d);
  move("surge_forward", 0.15, 2.0);
  setDuration(2.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus MoveToInteractionDistance::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void MoveToInteractionDistance::onHalted() { stop(); }

// ═════════════════════════════════════════════════════════════════════
// TORPEDOES — motion primitives
// ═════════════════════════════════════════════════════════════════════
LOG_SYNC_OK(HoldPosition,    "[torp] hold position")
LOG_SYNC_OK(RealignVehicle,  "[torp] realign vehicle")

BT::NodeStatus ObserveResult::onStart() {
  RCLCPP_INFO(lg(), "[torp] observe result");
  setDuration(1.5);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ObserveResult::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void ObserveResult::onHalted() {}

// ═════════════════════════════════════════════════════════════════════
// OCTAGON — motion primitives
// ═════════════════════════════════════════════════════════════════════
LOG_SYNC_OK(MaintainPosition, "[octagon] maintain position")
LOG_SYNC_OK(HoldStation,      "[octagon] hold station")

BT::NodeStatus NavigateToOctagon::onStart() {
  RCLCPP_INFO(lg(), "[octagon] navigate to octagon");
  nav("track_object", "octagon", 0.3, 1.0);
  setDuration(12.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus NavigateToOctagon::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void NavigateToOctagon::onHalted() { nav("idle"); }

BT::NodeStatus EnterOctagon::onStart() {
  RCLCPP_INFO(lg(), "[octagon] enter");
  move("surge_forward", 0.3, 3.0);
  setDuration(3.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus EnterOctagon::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void EnterOctagon::onHalted() { stop(); }

BT::NodeStatus AscendSlowly::onStart() {
  double sp = 0.1;
  getInput("speed_mps", sp);
  RCLCPP_INFO(lg(), "[octagon] ascend slowly @%.2fm/s", sp);
  move("emerge", std::min(0.4, std::max(0.05, sp)), 8.0);
  setDuration(8.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus AscendSlowly::onRunning() {
  if (MissionIO::ready() && MissionIO::get().depth() >= 0 &&
      MissionIO::get().depth() < 0.2) {
    move("depth_hold");
    return BT::NodeStatus::SUCCESS;
  }
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void AscendSlowly::onHalted() { stop(); }

BT::NodeStatus BreakSurface::onStart() {
  RCLCPP_INFO(lg(), "[octagon] break surface");
  move("emerge", 0.5, 3.0);
  setDuration(3.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus BreakSurface::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void BreakSurface::onHalted() { stop(); }

BT::NodeStatus DescendSlightly::onStart() {
  double dlt = 0.3;
  getInput("delta_m", dlt);
  RCLCPP_INFO(lg(), "[octagon] descend %.2fm", dlt);
  move("submerge", 0.2, 2.0);
  setDuration(2.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus DescendSlightly::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void DescendSlightly::onHalted() { move("depth_hold"); }

BT::NodeStatus NavigateBasket::onStart() {
  RCLCPP_INFO(lg(), "[octagon] navigate to basket");
  nav("track_object", "basket", 0.25, 0.4);
  setDuration(6.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus NavigateBasket::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void NavigateBasket::onHalted() { nav("idle"); }

BT::NodeStatus ExecuteYawRotation::onStart() {
  int count = 1;
  getInput("rotation_count", count);
  if (count < 1) count = 1;
  RCLCPP_INFO(lg(), "[octagon] execute %d full yaw rotation(s)", count);
  double seconds = 6.0 * count;  // ~60°/s open-loop estimate
  move("rotate_cw", 0.5, seconds);
  setDuration(seconds);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ExecuteYawRotation::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void ExecuteYawRotation::onHalted() { stop(); }

// ═════════════════════════════════════════════════════════════════════
// RETURN / RECOVERY
// ═════════════════════════════════════════════════════════════════════
LOG_SYNC_OK(MissionComplete,  "[mission] complete")
LOG_SYNC_OK(ReplanPath,       "[recovery] replan path")
LOG_SYNC_OK(SkipTask,         "[recovery] skip current task")
LOG_SYNC_OK(AbortMission,     "[recovery] abort mission")
LOG_SYNC_OK(TransmitStatus,   "[recovery] transmit status")
LOG_SYNC_OK(Relocalize,       "[recovery] relocalize")

BT::NodeStatus StopVehicle::tick() {
  RCLCPP_WARN(lg(), "[recovery] STOP VEHICLE");
  stop();
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus NavigateToStart::onStart() {
  RCLCPP_INFO(lg(), "[return] navigate to start");
  nav("waypoint", "", 0.4, 0.0, 0.0, 0.0, 0.0, 1.5);
  setDuration(20.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus NavigateToStart::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void NavigateToStart::onHalted() { nav("idle"); }

BT::NodeStatus TransitStartGate::onStart() {
  RCLCPP_INFO(lg(), "[return] transit start gate");
  move("surge_forward", 0.4, 4.0);
  setDuration(4.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus TransitStartGate::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void TransitStartGate::onHalted() { stop(); }

BT::NodeStatus Surface::onStart() {
  RCLCPP_INFO(lg(), "[mission] surface");
  move("emerge", 0.4, 8.0);
  setDuration(8.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Surface::onRunning() {
  if (MissionIO::ready() && MissionIO::get().depth() >= 0 &&
      MissionIO::get().depth() < 0.1)
    return BT::NodeStatus::SUCCESS;
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void Surface::onHalted() { stop(); }

BT::NodeStatus SurfaceSafely::onStart() {
  RCLCPP_WARN(lg(), "[recovery] surface safely");
  move("emerge", 0.3, 12.0);
  setDuration(12.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus SurfaceSafely::onRunning() {
  if (MissionIO::ready() && MissionIO::get().depth() >= 0 &&
      MissionIO::get().depth() < 0.1)
    return BT::NodeStatus::SUCCESS;
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void SurfaceSafely::onHalted() { stop(); }

BT::NodeStatus RecoverDepth::onStart() {
  RCLCPP_WARN(lg(), "[recovery] recover depth");
  move("depth_hold");
  setDuration(2.0);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus RecoverDepth::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void RecoverDepth::onHalted() {}

BT::NodeStatus BackAway::onStart() {
  RCLCPP_WARN(lg(), "[recovery] back away");
  move("surge_backward", 0.3, 2.5);
  setDuration(2.5);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus BackAway::onRunning() {
  return deadlinePassed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void BackAway::onHalted() { stop(); }

#undef LOG_SYNC_OK

}  // namespace shrub
