#pragma once
// SHRUB v4 — All custom BehaviorTree.CPP v4 nodes for the 2026
// "Restore and Recovery" mission tree (bt_xml/robosub2026_mission.xml).
//
// Conventions
//   * Most ACTIONS are SyncActionNode stubs that log + return SUCCESS so the
//     full tree ticks end-to-end without hardware. Actions that map naturally
//     onto MissionIO send a short ROS command, then either return SUCCESS
//     (open-loop one-shot) or run as a StatefulActionNode (timed / closed loop).
//   * Most CONDITIONS read state from the blackboard with safe defaults so the
//     tree degrades gracefully when a publisher is missing.
//   * Hardware that does not exist yet (gripper, marker dropper, torpedo
//     launcher, magnetic tool, hydrophones, pinger) returns SUCCESS and logs
//     "TODO: wire <driver>" so it is greppable.

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>

namespace shrub {

// ─── Declaration macros ──────────────────────────────────────────────
#define SHRUB_SYNC(ClassName)                                           \
  class ClassName : public BT::SyncActionNode {                         \
   public:                                                              \
    ClassName(const std::string& n, const BT::NodeConfig& c)            \
        : SyncActionNode(n, c) {}                                       \
    static BT::PortsList providedPorts() { return {}; }                 \
    BT::NodeStatus tick() override;                                     \
  };

#define SHRUB_SYNC_P(ClassName, ...)                                    \
  class ClassName : public BT::SyncActionNode {                         \
   public:                                                              \
    ClassName(const std::string& n, const BT::NodeConfig& c)            \
        : SyncActionNode(n, c) {}                                       \
    static BT::PortsList providedPorts() { return { __VA_ARGS__ }; }    \
    BT::NodeStatus tick() override;                                     \
  };

#define SHRUB_COND(ClassName)                                           \
  class ClassName : public BT::ConditionNode {                          \
   public:                                                              \
    ClassName(const std::string& n, const BT::NodeConfig& c)            \
        : ConditionNode(n, c) {}                                        \
    static BT::PortsList providedPorts() { return {}; }                 \
    BT::NodeStatus tick() override;                                     \
  };

#define SHRUB_COND_P(ClassName, ...)                                    \
  class ClassName : public BT::ConditionNode {                          \
   public:                                                              \
    ClassName(const std::string& n, const BT::NodeConfig& c)            \
        : ConditionNode(n, c) {}                                        \
    static BT::PortsList providedPorts() { return { __VA_ARGS__ }; }    \
    BT::NodeStatus tick() override;                                     \
  };

// Stateful nodes carry a duration (or a "done" predicate); base type adds
// `deadline_` so derived classes can implement onRunning() in one line.
class TimedAction : public BT::StatefulActionNode {
 public:
  TimedAction(const std::string& n, const BT::NodeConfig& c)
      : StatefulActionNode(n, c) {}

 protected:
  void setDuration(double seconds);
  bool deadlinePassed() const;
  std::chrono::steady_clock::time_point deadline_{};
};

#define SHRUB_STATEFUL(ClassName)                                       \
  class ClassName : public TimedAction {                                \
   public:                                                              \
    ClassName(const std::string& n, const BT::NodeConfig& c)            \
        : TimedAction(n, c) {}                                          \
    static BT::PortsList providedPorts() { return {}; }                 \
    BT::NodeStatus onStart() override;                                  \
    BT::NodeStatus onRunning() override;                                \
    void onHalted() override;                                           \
  };

#define SHRUB_STATEFUL_P(ClassName, ...)                                \
  class ClassName : public TimedAction {                                \
   public:                                                              \
    ClassName(const std::string& n, const BT::NodeConfig& c)            \
        : TimedAction(n, c) {}                                          \
    static BT::PortsList providedPorts() { return { __VA_ARGS__ }; }    \
    BT::NodeStatus onStart() override;                                  \
    BT::NodeStatus onRunning() override;                                \
    void onHalted() override;                                           \
  };

// ═══════════════════════════════════════════════════════════════════
// CONDITIONS  (39)
// ═══════════════════════════════════════════════════════════════════
SHRUB_COND(CoinflipNormal)
SHRUB_COND(CoinflipBackward)
SHRUB_COND(LocalizationStable)
SHRUB_COND(GateDetected)
SHRUB_COND_P(GateSearchTimeout,
             BT::InputPort<double>("timeout_sec", 30.0, "search timeout (s)"))
SHRUB_COND(RoleSignsVisible)
SHRUB_COND(SurveyRepairDetected)
SHRUB_COND(SearchRescueDetected)
SHRUB_COND(RandomRoleAssignment)
SHRUB_COND(StyleModeEnabled)
SHRUB_COND(VehicleStable)
SHRUB_COND(OrientationReached)
SHRUB_COND(DividerVerified)
SHRUB_COND(NoCollisionRisk)
SHRUB_COND(GateCleared)
SHRUB_COND(GateRedRight)
SHRUB_COND(GateRedLeft)
SHRUB_COND(PlaneCrossed)
SHRUB_COND(InsideBounds)
SHRUB_COND(RoleSurveyRepair)
SHRUB_COND(RoleSearchRescue)
SHRUB_COND_P(ValidDropAltitude,
             BT::InputPort<double>("min_altitude_m", 0.3, "min drop altitude (m)"))
SHRUB_COND(MarkerInBin)
SHRUB_COND(MarkersRemaining)
SHRUB_COND(LightOff)
SHRUB_COND(AlignmentConfirmed)
SHRUB_COND(TorpedoHit)
SHRUB_COND(TorpedoesRemaining)
SHRUB_COND(InsideOctagon)
SHRUB_COND(CorrectBasket)
SHRUB_COND(ObjectDelivered)
SHRUB_COND(OneItemInBasket)
SHRUB_COND(TwoItemsInBasket)
SHRUB_COND(MissionFinished)
SHRUB_COND(LocalizationLost)
SHRUB_COND(DepthUnstable)
SHRUB_COND(ObstacleDetected)
SHRUB_COND_P(TaskTimeout,
             BT::InputPort<double>("timeout_sec", 120.0, "task timeout (s)"))
SHRUB_COND(CriticalFailure)

// ═══════════════════════════════════════════════════════════════════
// ACTIONS – Initialization  (10) — no hydrophones this season
// ═══════════════════════════════════════════════════════════════════
SHRUB_SYNC(VerifyThrusters)
SHRUB_SYNC(VerifyIMU)
SHRUB_SYNC(VerifyDepthSensor)
SHRUB_SYNC(VerifyCameras)
SHRUB_SYNC(VerifyManipulators)
SHRUB_SYNC(VerifyTorpedoLauncher)
SHRUB_SYNC(LoadMissionParameters)
SHRUB_SYNC(ZeroStateEstimator)
SHRUB_SYNC(WaitForStartSignal)
SHRUB_SYNC(ResetTaskTimer)  // sets `task_start_time` = now(); call at start of each task
SHRUB_STATEFUL_P(SubmergeToMissionDepth,
                 BT::InputPort<double>("depth_m", 1.5, "target depth (m)"))

// ═══════════════════════════════════════════════════════════════════
// ACTIONS – Gate  (41)
// ═══════════════════════════════════════════════════════════════════
SHRUB_SYNC(FaceGateEstimate)
SHRUB_SYNC(ContinueMission)
SHRUB_SYNC(SearchGateReverse)
SHRUB_STATEFUL(Rotate180)
SHRUB_SYNC_P(HoldDepth, BT::InputPort<double>("depth_m", 1.5, "depth (m)"))
SHRUB_SYNC(HoldRoll)
SHRUB_SYNC(HoldPitch)
SHRUB_SYNC_P(HoldHeading, BT::InputPort<double>("heading_deg", 0.0, "heading (deg)"))
SHRUB_SYNC(EnableFrontCamera)
SHRUB_SYNC(LockGateTarget)
SHRUB_STATEFUL_P(YawSweep, BT::InputPort<double>("sweep_deg", 30.0, "sweep (deg)"))
SHRUB_STATEFUL(ForwardSpiral)
SHRUB_STATEFUL(DepthModulation)
SHRUB_STATEFUL_P(DeadReckonForward,
                 BT::InputPort<double>("distance_m", 2.0, "distance (m)"))
SHRUB_SYNC(RetryGateSearch)
SHRUB_STATEFUL(AlignGateCenter)
SHRUB_SYNC(EstimateGateDivider)
SHRUB_SYNC(EstimateGateOpening)
SHRUB_SYNC(MaintainCenterline)
SHRUB_SYNC(MaintainDesiredDepth)
SHRUB_SYNC_P(ReduceVelocity,
             BT::InputPort<double>("target_speed_mps", 0.3, "target speed (m/s)"))
SHRUB_SYNC(SetRoleSurveyRepair)
SHRUB_SYNC(SelectSurveyRepairGateSide)
SHRUB_SYNC(SetRoleSearchRescue)
SHRUB_SYNC(SelectSearchRescueGateSide)
SHRUB_SYNC(ReadAssignedRole)
SHRUB_SYNC(StoreRoleVariable)
SHRUB_STATEFUL(Yaw90)
SHRUB_STATEFUL(Roll90)
SHRUB_STATEFUL(Pitch90)
SHRUB_STATEFUL(ExecuteStyle)
SHRUB_SYNC(SkipStyle)
SHRUB_SYNC(OffsetToSelectedSide)
SHRUB_SYNC(MaintainGateClearance)
SHRUB_STATEFUL(ForwardTransit)
SHRUB_SYNC(StopMotion)
SHRUB_SYNC(RecenterVehicle)
SHRUB_SYNC(RetryGateTransit)
SHRUB_SYNC(RecordGateSide)
SHRUB_STATEFUL_P(MoveForward, BT::InputPort<double>("distance_m", 1.0, "distance (m)"))
SHRUB_SYNC(ReestablishHeading)
SHRUB_SYNC(SearchPathMarker)

// ═══════════════════════════════════════════════════════════════════
// ACTIONS – Slalom  (13)
// ═══════════════════════════════════════════════════════════════════
SHRUB_SYNC(DetectOrangePath)
SHRUB_STATEFUL(FollowPath)
SHRUB_SYNC(SearchSlalomPoles)
SHRUB_SYNC(KeepRedPoleRight)
SHRUB_SYNC(KeepRedPoleLeft)
SHRUB_SYNC(ExitSlalom)
SHRUB_SYNC(DetectPoleGroup)
SHRUB_SYNC(EstimateGap)
SHRUB_STATEFUL(AlignThroughGap)
SHRUB_SYNC(MaintainSlalomDepth)
SHRUB_STATEFUL(TransitGap)
SHRUB_SYNC(CorrectHeading)
SHRUB_SYNC(ReenterSlalom)

// ═══════════════════════════════════════════════════════════════════
// ACTIONS – Bins  (17)
// ═══════════════════════════════════════════════════════════════════
SHRUB_STATEFUL(FollowPathMarker)
SHRUB_SYNC(SearchPipeline)
SHRUB_SYNC(SearchFireBins)
SHRUB_SYNC(SearchBloodBins)
SHRUB_STATEFUL(NavigateBin1)
SHRUB_STATEFUL(NavigateBin2)
SHRUB_SYNC(StationKeep)
SHRUB_SYNC(EstimateBinCenter)
SHRUB_SYNC(ReleaseMarker)
SHRUB_STATEFUL(ObserveMarker)
SHRUB_SYNC(RetryMarkerDrop)
SHRUB_SYNC(DetectMagneticTarget)
SHRUB_STATEFUL(AlignInteractionTool)
SHRUB_STATEFUL_P(MoveToInteractionDistance,
                 BT::InputPort<double>("distance_m", 0.1, "distance (m)"))
SHRUB_SYNC(ActivateTool)
SHRUB_SYNC(RetryInteraction)
SHRUB_SYNC(TimeoutRecovery)

// ═══════════════════════════════════════════════════════════════════
// ACTIONS – Torpedoes  (13) — vision-only entry, no pinger
// ═══════════════════════════════════════════════════════════════════
SHRUB_STATEFUL(SearchTargetBoard)
SHRUB_SYNC(IdentifyRoleBoard)
SHRUB_SYNC(DetectLargeOpening)
SHRUB_SYNC(DetectSmallOpening)
SHRUB_SYNC(EstimateRange)
SHRUB_SYNC(EstimateOffset)
SHRUB_SYNC(HoldPosition)
SHRUB_SYNC(CorrectAim)
SHRUB_SYNC_P(ArmLauncher, BT::InputPort<int>("tube_id", 1, "tube id"))
SHRUB_SYNC(LaunchTorpedo)
SHRUB_STATEFUL(ObserveResult)
SHRUB_SYNC(RealignVehicle)
SHRUB_SYNC(RetryShot)

// ═══════════════════════════════════════════════════════════════════
// ACTIONS – Octagon  (17) — vision-only entry, no pinger
// ═══════════════════════════════════════════════════════════════════
SHRUB_STATEFUL(NavigateToOctagon)
SHRUB_SYNC(DetectOctagon)
SHRUB_STATEFUL(EnterOctagon)
SHRUB_SYNC(EstimateOctagonCenter)
SHRUB_STATEFUL_P(AscendSlowly,
                 BT::InputPort<double>("speed_mps", 0.1, "speed (m/s)"))
SHRUB_STATEFUL(BreakSurface)
SHRUB_SYNC(MaintainPosition)
SHRUB_STATEFUL_P(DescendSlightly,
                 BT::InputPort<double>("delta_m", 0.3, "depth delta (m)"))
SHRUB_SYNC(SearchRepairObjects)
SHRUB_SYNC(SearchMedicalObjects)
SHRUB_STATEFUL(NavigateBasket)
SHRUB_SYNC(HoldStation)
SHRUB_SYNC(ReleaseObject)
SHRUB_SYNC(FaceCompassOrRing)
SHRUB_SYNC(FaceHammerOrSOS)
SHRUB_SYNC(CalculateRotationCount)
SHRUB_STATEFUL_P(ExecuteYawRotation,
                 BT::InputPort<int>("rotation_count", 1, "rotations"))

// ═══════════════════════════════════════════════════════════════════
// ACTIONS – Return / Recovery  (15)
// ═══════════════════════════════════════════════════════════════════
SHRUB_STATEFUL(NavigateToStart)
SHRUB_SYNC(SearchStartGate)
SHRUB_SYNC(AlignStartGate)
SHRUB_STATEFUL(TransitStartGate)
SHRUB_STATEFUL(Surface)
SHRUB_SYNC(MissionComplete)
SHRUB_SYNC(StopVehicle)
SHRUB_SYNC(Relocalize)
SHRUB_STATEFUL(RecoverDepth)
SHRUB_STATEFUL(BackAway)
SHRUB_SYNC(ReplanPath)
SHRUB_SYNC(SkipTask)
SHRUB_SYNC(AbortMission)
SHRUB_STATEFUL(SurfaceSafely)
SHRUB_SYNC(TransmitStatus)

// ═══════════════════════════════════════════════════════════════════
// Registration — call from bt_executor main()
// ═══════════════════════════════════════════════════════════════════
void registerAllNodes(BT::BehaviorTreeFactory& factory,
                      rclcpp::Node::SharedPtr ros_node);

}  // namespace shrub
