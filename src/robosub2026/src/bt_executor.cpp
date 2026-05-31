// SHRUB v4 — BT Executor for the 2026 "Restore and Recovery" mission tree.
// Loads bt_xml/robosub2026_mission.xml, registers all custom nodes, and ticks
// the tree at a fixed rate while keeping the blackboard fresh from sensors.

#include <bt_mission/shrub_nodes.hpp>
#include <bt_mission/mission_io.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <thread>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<rclcpp::Node>("shrub_executor");

  // Wire the tree to the running Python autonomy stack.
  shrub::MissionIO::init(ros_node);

  // --- Parameters ---
  ros_node->declare_parameter<std::string>("bt_xml", "robosub2026_mission.xml");
  ros_node->declare_parameter<std::string>("tree_id", "MainTree");
  ros_node->declare_parameter<std::string>("coin_flip", "normal");      // normal|backward
  ros_node->declare_parameter<std::string>("role", "survey_repair");    // survey_repair|search_rescue
  ros_node->declare_parameter<std::string>("gate_red_side", "right");   // right|left
  ros_node->declare_parameter<std::string>("run_mode", "semifinal");
  ros_node->declare_parameter<bool>("style_enabled", true);
  ros_node->declare_parameter<int>("tick_rate_ms", 50);
  // Safety thresholds — trip critical_failure (→ GlobalRecovery) when crossed.
  ros_node->declare_parameter<double>("battery_critical_pct", 15.0);

  std::string xml_file, tree_id, coin_flip, role, gate_red_side, run_mode;
  bool style_enabled;
  int tick_rate_ms;
  double battery_critical_pct = 15.0;
  ros_node->get_parameter("bt_xml", xml_file);
  ros_node->get_parameter("tree_id", tree_id);
  ros_node->get_parameter("coin_flip", coin_flip);
  ros_node->get_parameter("role", role);
  ros_node->get_parameter("gate_red_side", gate_red_side);
  ros_node->get_parameter("run_mode", run_mode);
  ros_node->get_parameter("style_enabled", style_enabled);
  ros_node->get_parameter("tick_rate_ms", tick_rate_ms);
  ros_node->get_parameter("battery_critical_pct", battery_critical_pct);

  // --- Resolve XML path ---
  std::string xml_path;
  if (std::filesystem::exists(xml_file)) {
    xml_path = xml_file;
  } else {
    auto pkg_dir = ament_index_cpp::get_package_share_directory("bt_mission");
    xml_path = pkg_dir + "/bt_xml/" + xml_file;
  }
  RCLCPP_INFO(ros_node->get_logger(), "Loading BT from: %s", xml_path.c_str());
  RCLCPP_INFO(ros_node->get_logger(),
              "tree_id=%s coin_flip=%s role=%s gate_red_side=%s style=%s mode=%s",
              tree_id.c_str(), coin_flip.c_str(), role.c_str(),
              gate_red_side.c_str(), style_enabled ? "on" : "off",
              run_mode.c_str());

  // --- Register nodes + load XML ---
  BT::BehaviorTreeFactory factory;
  shrub::registerAllNodes(factory, ros_node);
  factory.registerBehaviorTreeFromFile(xml_path);

  auto tree = factory.createTree(tree_id);
  auto bb = tree.rootBlackboard();

  // --- Seed blackboard ---
  const double start_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  // Run-time inputs (set by parameters / coin flip):
  bb->set("coin_flip", coin_flip);
  bb->set("role", role);
  bb->set("gate_red_side", gate_red_side);
  bb->set("run_mode", run_mode);
  bb->set("style_enabled", style_enabled);
  bb->set("task_start_time", start_time);
  bb->set("gate_search_start", start_time);

  // Counter / flag defaults — let actions decrement/flip as the mission progresses.
  bb->set<int>("markers_remaining", 2);
  bb->set<int>("torpedoes_remaining", 2);
  bb->set<int>("objects_delivered", 0);
  bb->set("inside_float_area", false);
  bb->set("inside_octagon", false);
  bb->set("gate_cleared", false);
  bb->set("plane_crossed", true);
  bb->set("inside_bounds", true);
  bb->set("vehicle_stable", true);
  bb->set("orientation_reached", true);
  bb->set("divider_verified", true);
  bb->set("marker_in_bin", true);
  bb->set("light_off", true);
  bb->set("aligned", true);
  bb->set("torpedo_hit", true);
  bb->set("correct_basket", true);
  bb->set("object_delivered", false);
  bb->set("mission_complete", false);
  bb->set("depth_unstable", false);
  bb->set("obstacle_detected", false);
  bb->set("critical_failure", false);

  // Live values pushed each tick from MissionIO:
  bb->set("depth", 0.0);
  bb->set("altitude_m", 1.0);
  bb->set("battery_pct", 100.0);
  bb->set("leak_detected", false);

  BT::StdCoutLogger logger(tree);

  RCLCPP_INFO(ros_node->get_logger(), "════════════════════════════════════");
  RCLCPP_INFO(ros_node->get_logger(), "  SHRUB v4 — Mission started");
  RCLCPP_INFO(ros_node->get_logger(), "════════════════════════════════════");

  // --- Main tick loop ---
  auto tick_period = std::chrono::milliseconds(tick_rate_ms);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  constexpr double MISSION_TIMEOUT_S = 870.0;  // 14.5 min — finish before the 15 min window
  constexpr int MAX_TICKS = 200000;
  int tick_count = 0;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count() - start_time;
    if (elapsed >= MISSION_TIMEOUT_S) {
      RCLCPP_WARN(ros_node->get_logger(),
                  "MISSION TIMEOUT (%.0fs) — halting tree", elapsed);
      tree.haltTree();
      status = BT::NodeStatus::SUCCESS;
      break;
    }
    if (++tick_count >= MAX_TICKS) {
      RCLCPP_WARN(ros_node->get_logger(),
                  "Max tick count (%d) reached — halting tree", MAX_TICKS);
      tree.haltTree();
      status = BT::NodeStatus::FAILURE;
      break;
    }

    rclcpp::spin_some(ros_node);

    // Push live sensor values onto the blackboard for safety / drop-altitude checks.
    // depth < 0 means "not yet received" — keep the seeded default in that case.
    double live_depth = shrub::MissionIO::get().depth();
    if (live_depth >= 0.0) bb->set("depth", live_depth);

    // Battery + leak from safety_monitor_node. Flip critical_failure when
    // battery dips below the configured threshold OR a leak is detected;
    // GlobalRecovery's CriticalFailure branch then drives SurfaceSafely.
    double batt = shrub::MissionIO::get().batteryPct();
    if (std::isfinite(batt)) bb->set("battery_pct", batt);
    bool leak = shrub::MissionIO::get().leakDetected();
    bb->set("leak_detected", leak);
    bool already_critical = false;
    (void)bb->get<bool>("critical_failure", already_critical);
    bool should_critical = leak ||
                           (std::isfinite(batt) && batt < battery_critical_pct);
    if (should_critical && !already_critical) {
      RCLCPP_ERROR(ros_node->get_logger(),
                   "SAFETY: critical_failure tripped (battery=%.1f%%, leak=%s)",
                   batt, leak ? "true" : "false");
      bb->set("critical_failure", true);
    }

    try {
      status = tree.tickOnce();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(ros_node->get_logger(), "BT tick exception: %s", e.what());
      tree.haltTree();
      status = BT::NodeStatus::FAILURE;
      break;
    }

    std::this_thread::sleep_for(tick_period);
  }

  if (status == BT::NodeStatus::SUCCESS)
    RCLCPP_INFO(ros_node->get_logger(), "Mission completed: SUCCESS");
  else if (status == BT::NodeStatus::FAILURE)
    RCLCPP_WARN(ros_node->get_logger(), "Mission completed: FAILURE");

  double final_elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count() - start_time;
  double remaining = 900.0 - final_elapsed;
  if (remaining > 0) {
    double bonus = (std::floor(remaining / 60.0) +
                    std::fmod(remaining, 60.0) / 60.0) * 100.0;
    RCLCPP_INFO(ros_node->get_logger(),
                "Time remaining: %.1fs — potential time bonus: %.1f pts",
                remaining, bonus);
  }

  // Make sure the sub isn't left thrusting.
  if (shrub::MissionIO::ready()) shrub::MissionIO::get().stop();
  rclcpp::shutdown();
  return 0;
}
