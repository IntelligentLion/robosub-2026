// SHRUB v3 — Task logic nodes + node registration
#include <bt_mission/shrub_nodes.hpp>

namespace shrub {

// ─── Style_through_gate ─────────────────────────────────────────
// From handbook: Roll/Pitch worth more than Yaw per 90° change.
// "returning to the last previous orientation won't count"
BT::NodeStatus Style_through_gate::onStart() {
  std::string type;
  getInput("type", type);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Style through gate: %s", type.c_str());
  // TODO: Execute barrel roll (2×180° pitch = highest point density)
  // or yaw spins as fallback
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Style_through_gate::onRunning() { return BT::NodeStatus::SUCCESS; }
void Style_through_gate::onHalted() {}

// ─── Determine_basket ───────────────────────────────────────────
// From handbook: "the correct basket is the opposite color.
// yellow bottles → pink basket, pink ladles → yellow basket."
BT::NodeStatus Determine_basket::tick() {
  std::string obj_type;
  getInput("obj_type", obj_type);
  std::string basket;
  if (obj_type.find("bottle") != std::string::npos ||
      obj_type.find("yellow") != std::string::npos) {
    basket = "pink";
  } else {
    basket = "yellow";
  }
  setOutput("basket", basket);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "%s → %s basket",
              obj_type.c_str(), basket.c_str());
  return BT::NodeStatus::SUCCESS;
}

// ─── Compute_slalom_path ────────────────────────────────────────
// From handbook: "same side of the red pipe when it passed through
// the gate" earns more points. White left, red middle, white right.
BT::NodeStatus Compute_slalom_path::tick() {
  std::string side, pipes;
  getInput("side", side);
  getInput("pipes", pipes);
  // TODO: Compute actual waypoint from pipe detection + side preference
  std::string wp = (side == "L") ? "slalom_left_of_red" : "slalom_right_of_red";
  setOutput("waypoint", wp);
  return BT::NodeStatus::SUCCESS;
}

// ─── Rotation_bonus ─────────────────────────────────────────────
// From handbook: "rotating the same number of turns as the trash
// collected in the baskets" — exact = max pts, ±1 = partial.
BT::NodeStatus Rotation_bonus::onStart() {
  int count = 0;
  getInput("count", count);
  RCLCPP_INFO(rclcpp::get_logger("shrub"), "Rotation bonus: %d rotations", count);
  // TODO: Execute count full 360° rotations
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Rotation_bonus::onRunning() { return BT::NodeStatus::SUCCESS; }
void Rotation_bonus::onHalted() {}

// ─── Increment ──────────────────────────────────────────────────
BT::NodeStatus Increment::tick() {
  int val = 0;
  getInput("val", val);
  val++;
  setOutput("val", val);
  return BT::NodeStatus::SUCCESS;
}

// ═════════════════════════════════════════════════════════════════
// NODE REGISTRATION — called from bt_executor main()
// ═════════════════════════════════════════════════════════════════
void registerAllNodes(BT::BehaviorTreeFactory& factory,
                      rclcpp::Node::SharedPtr /*ros_node*/)
{
  // --- Conditions ---
  factory.registerNodeType<IsBatteryOk>("IsBatteryOk");
  factory.registerNodeType<IsLeakDetected>("IsLeakDetected");
  factory.registerNodeType<IsDepthSafe>("IsDepthSafe");
  factory.registerNodeType<IsInsideFloatArea>("IsInsideFloatArea");
  factory.registerNodeType<IsTimeRemaining>("IsTimeRemaining");
  factory.registerNodeType<AlwaysSuccess>("AlwaysSuccess");

  // --- Navigation ---
  factory.registerNodeType<Submerge>("Submerge");
  factory.registerNodeType<AscendTo>("AscendTo");
  factory.registerNodeType<EmergencySurface>("EmergencySurface");
  factory.registerNodeType<Turn>("Turn");
  factory.registerNodeType<Navigate_to>("Navigate_to");
  factory.registerNodeType<Navigate_to_bearing>("Navigate_to_bearing");
  factory.registerNodeType<Navigate_on_heading>("Navigate_on_heading");
  factory.registerNodeType<Navigate_forward>("Navigate_forward");
  factory.registerNodeType<Move_through_gate>("Move_through_gate");
  factory.registerNodeType<Reposition_to_gate_side>("Reposition_to_gate_side");
  factory.registerNodeType<Record_heading>("Record_heading");
  factory.registerNodeType<Compute_reverse_heading>("Compute_reverse_heading");
  factory.registerNodeType<Recalibrate_nav>("Recalibrate_nav");
  factory.registerNodeType<Stabilize>("Stabilize");
  factory.registerNodeType<Wait>("Wait");
  factory.registerNodeType<Hold_depth>("Hold_depth");
  factory.registerNodeType<Surface_in_float_area>("Surface_in_float_area");
  factory.registerNodeType<Face_direction>("Face_direction");
  factory.registerNodeType<Follow_path>("Follow_path");
  factory.registerNodeType<Circle_around>("Circle_around");

  // --- Perception ---
  factory.registerNodeType<Detect_gate>("Detect_gate");
  factory.registerNodeType<Detect_animal_on_gate>("Detect_animal_on_gate");
  factory.registerNodeType<Detect_animal_image>("Detect_animal_image");
  factory.registerNodeType<Detect_pinger>("Detect_pinger");
  factory.registerNodeType<Detect_float_area_below>("Detect_float_area_below");
  factory.registerNodeType<Confirm_overhead>("Confirm_overhead");
  factory.registerNodeType<Detect_buoy>("Detect_buoy");
  factory.registerNodeType<Detect_bin_below>("Detect_bin_below");
  factory.registerNodeType<Detect_object>("Detect_object");
  factory.registerNodeType<Detect_slalom_pipes>("Detect_slalom_pipes");
  factory.registerNodeType<Detect_task_board>("Detect_task_board");
  factory.registerNodeType<Detect_opening>("Detect_opening");
  factory.registerNodeType<Detect_path_marker>("Detect_path_marker");
  factory.registerNodeType<Detect_vertical_marker>("Detect_vertical_marker");
  factory.registerNodeType<Align_to>("Align_to");
  factory.registerNodeType<Align_above>("Align_above");
  factory.registerNodeType<Align_to_opening>("Align_to_opening");
  factory.registerNodeType<Align_to_basket>("Align_to_basket");
  factory.registerNodeType<Center_beneath>("Center_beneath");
  factory.registerNodeType<Approach_and_touch>("Approach_and_touch");

  // --- Manipulation ---
  factory.registerNodeType<Grab_object>("Grab_object");
  factory.registerNodeType<Release_object>("Release_object");
  factory.registerNodeType<Drop_marker>("Drop_marker");
  factory.registerNodeType<Fire_torpedo>("Fire_torpedo");

  // --- Task Logic ---
  factory.registerNodeType<Style_through_gate>("Style_through_gate");
  factory.registerNodeType<Determine_basket>("Determine_basket");
  factory.registerNodeType<Compute_slalom_path>("Compute_slalom_path");
  factory.registerNodeType<Rotation_bonus>("Rotation_bonus");
  factory.registerNodeType<Increment>("Increment");

  RCLCPP_INFO(rclcpp::get_logger("shrub"),
              "Registered %zu node types", factory.manifests().size());
}

} // namespace shrub
