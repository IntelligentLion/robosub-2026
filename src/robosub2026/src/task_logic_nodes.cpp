// SHRUB v4 — Task-logic action nodes + node-type registration.
//
// "Task logic" actions are pure compute (no ROS calls). They typically read
// or write the blackboard so other nodes can branch on the result.

#include <bt_mission/shrub_nodes.hpp>
#include <bt_mission/mission_io.hpp>

namespace shrub {

namespace {
inline rclcpp::Logger lg() { return rclcpp::get_logger("shrub"); }
}  // namespace

// CalculateRotationCount — from handbook: "rotating the same number of turns
// as the trash collected in the baskets" — exact = max points, ±1 = partial.
BT::NodeStatus CalculateRotationCount::tick() {
  int delivered = 0;
  if (auto bb = config().blackboard) bb->get<int>("objects_delivered", delivered);
  if (delivered < 1) delivered = 1;  // award at least one rotation attempt
  if (auto bb = config().blackboard) bb->set("rotation_count", delivered);
  RCLCPP_INFO(lg(), "[octagon] rotation count = %d (objects delivered)", delivered);
  return BT::NodeStatus::SUCCESS;
}

// ═════════════════════════════════════════════════════════════════════
// NODE REGISTRATION — called from bt_executor main()
// ═════════════════════════════════════════════════════════════════════
void registerAllNodes(BT::BehaviorTreeFactory& factory,
                      rclcpp::Node::SharedPtr /*ros_node*/) {
#define REG(T) factory.registerNodeType<T>(#T)

  // --- Conditions ---
  REG(CoinflipNormal);
  REG(CoinflipBackward);
  REG(LocalizationStable);
  REG(GateDetected);
  REG(GateSearchTimeout);
  REG(RoleSignsVisible);
  REG(SurveyRepairDetected);
  REG(SearchRescueDetected);
  REG(RandomRoleAssignment);
  REG(StyleModeEnabled);
  REG(VehicleStable);
  REG(OrientationReached);
  REG(DividerVerified);
  REG(NoCollisionRisk);
  REG(GateCleared);
  REG(GateRedRight);
  REG(GateRedLeft);
  REG(PlaneCrossed);
  REG(InsideBounds);
  REG(RoleSurveyRepair);
  REG(RoleSearchRescue);
  REG(ValidDropAltitude);
  REG(MarkerInBin);
  REG(MarkersRemaining);
  REG(LightOff);
  REG(AlignmentConfirmed);
  REG(TorpedoHit);
  REG(TorpedoesRemaining);
  REG(InsideOctagon);
  REG(CorrectBasket);
  REG(ObjectDelivered);
  REG(OneItemInBasket);
  REG(TwoItemsInBasket);
  REG(MissionFinished);
  REG(LocalizationLost);
  REG(DepthUnstable);
  REG(ObstacleDetected);
  REG(TaskTimeout);
  REG(CriticalFailure);

  // --- Initialization ---
  REG(VerifyThrusters);
  REG(VerifyIMU);
  REG(VerifyDepthSensor);
  REG(VerifyCameras);
  REG(VerifyManipulators);
  REG(VerifyTorpedoLauncher);
  REG(LoadMissionParameters);
  REG(ZeroStateEstimator);
  REG(WaitForStartSignal);
  REG(SubmergeToMissionDepth);

  // --- Gate ---
  REG(FaceGateEstimate);
  REG(ContinueMission);
  REG(SearchGateReverse);
  REG(Rotate180);
  REG(HoldDepth);
  REG(HoldRoll);
  REG(HoldPitch);
  REG(HoldHeading);
  REG(EnableFrontCamera);
  REG(LockGateTarget);
  REG(YawSweep);
  REG(ForwardSpiral);
  REG(DepthModulation);
  REG(DeadReckonForward);
  REG(RetryGateSearch);
  REG(AlignGateCenter);
  REG(EstimateGateDivider);
  REG(EstimateGateOpening);
  REG(MaintainCenterline);
  REG(MaintainDesiredDepth);
  REG(ReduceVelocity);
  REG(SetRoleSurveyRepair);
  REG(SelectSurveyRepairGateSide);
  REG(SetRoleSearchRescue);
  REG(SelectSearchRescueGateSide);
  REG(ReadAssignedRole);
  REG(StoreRoleVariable);
  REG(Yaw90);
  REG(Roll90);
  REG(Pitch90);
  REG(ExecuteStyle);
  REG(SkipStyle);
  REG(OffsetToSelectedSide);
  REG(MaintainGateClearance);
  REG(ForwardTransit);
  REG(StopMotion);
  REG(RecenterVehicle);
  REG(RetryGateTransit);
  REG(RecordGateSide);
  REG(MoveForward);
  REG(ReestablishHeading);
  REG(SearchPathMarker);

  // --- Slalom ---
  REG(DetectOrangePath);
  REG(FollowPath);
  REG(SearchSlalomPoles);
  REG(KeepRedPoleRight);
  REG(KeepRedPoleLeft);
  REG(ExitSlalom);
  REG(DetectPoleGroup);
  REG(EstimateGap);
  REG(AlignThroughGap);
  REG(MaintainSlalomDepth);
  REG(TransitGap);
  REG(CorrectHeading);
  REG(ReenterSlalom);

  // --- Bins ---
  REG(FollowPathMarker);
  REG(SearchPipeline);
  REG(SearchFireBins);
  REG(SearchBloodBins);
  REG(NavigateBin1);
  REG(NavigateBin2);
  REG(StationKeep);
  REG(EstimateBinCenter);
  REG(ReleaseMarker);
  REG(ObserveMarker);
  REG(RetryMarkerDrop);
  REG(DetectMagneticTarget);
  REG(AlignInteractionTool);
  REG(MoveToInteractionDistance);
  REG(ActivateTool);
  REG(RetryInteraction);
  REG(TimeoutRecovery);

  // --- Torpedoes (vision-only, no pinger) ---
  REG(SearchTargetBoard);
  REG(IdentifyRoleBoard);
  REG(DetectLargeOpening);
  REG(DetectSmallOpening);
  REG(EstimateRange);
  REG(EstimateOffset);
  REG(HoldPosition);
  REG(CorrectAim);
  REG(ArmLauncher);
  REG(LaunchTorpedo);
  REG(ObserveResult);
  REG(RealignVehicle);
  REG(RetryShot);

  // --- Octagon (vision-only, no pinger) ---
  REG(NavigateToOctagon);
  REG(DetectOctagon);
  REG(EnterOctagon);
  REG(EstimateOctagonCenter);
  REG(AscendSlowly);
  REG(BreakSurface);
  REG(MaintainPosition);
  REG(DescendSlightly);
  REG(SearchRepairObjects);
  REG(SearchMedicalObjects);
  REG(NavigateBasket);
  REG(HoldStation);
  REG(ReleaseObject);
  REG(FaceCompassOrRing);
  REG(FaceHammerOrSOS);
  REG(CalculateRotationCount);
  REG(ExecuteYawRotation);

  // --- Return / Recovery ---
  REG(NavigateToStart);
  REG(SearchStartGate);
  REG(AlignStartGate);
  REG(TransitStartGate);
  REG(Surface);
  REG(MissionComplete);
  REG(StopVehicle);
  REG(Relocalize);
  REG(RecoverDepth);
  REG(BackAway);
  REG(ReplanPath);
  REG(SkipTask);
  REG(AbortMission);
  REG(SurfaceSafely);
  REG(TransmitStatus);

#undef REG

  RCLCPP_INFO(lg(), "Registered %zu node types", factory.manifests().size());
}

}  // namespace shrub
