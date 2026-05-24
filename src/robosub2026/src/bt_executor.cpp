// SHRUB v3 — BT Executor for RoboSub 2026
// Loads the mission XML, registers all custom nodes, ticks the tree.

#include <bt_mission/shrub_nodes.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<rclcpp::Node>("shrub_executor");

  // --- Declare parameters ---
  ros_node->declare_parameter<std::string>("bt_xml", "robosub2026_mission.xml");
  ros_node->declare_parameter<std::string>("tree_id", "SHRUB");
  ros_node->declare_parameter<std::string>("coin_flip", "none");
  ros_node->declare_parameter<std::string>("run_mode", "semifinal");
  ros_node->declare_parameter<int>("tick_rate_ms", 50);

  std::string xml_file, tree_id, coin_flip, run_mode;
  int tick_rate_ms;
  ros_node->get_parameter("bt_xml", xml_file);
  ros_node->get_parameter("tree_id", tree_id);
  ros_node->get_parameter("coin_flip", coin_flip);
  ros_node->get_parameter("run_mode", run_mode);
  ros_node->get_parameter("tick_rate_ms", tick_rate_ms);

  // --- Resolve XML path ---
  std::string xml_path;
  if (std::filesystem::exists(xml_file)) {
    xml_path = xml_file;
  } else {
    auto pkg_dir = ament_index_cpp::get_package_share_directory("bt_mission");
    xml_path = pkg_dir + "/bt_xml/" + xml_file;
  }
  RCLCPP_INFO(ros_node->get_logger(), "Loading BT from: %s", xml_path.c_str());
  RCLCPP_INFO(ros_node->get_logger(), "Tree ID: %s", tree_id.c_str());
  RCLCPP_INFO(ros_node->get_logger(), "Coin flip: %s | Run mode: %s",
              coin_flip.c_str(), run_mode.c_str());

  // --- Create factory and register all nodes ---
  BT::BehaviorTreeFactory factory;
  shrub::registerAllNodes(factory, ros_node);

  // --- Load all trees from XML ---
  factory.registerBehaviorTreeFromFile(xml_path);

  // --- Create the tree ---
  auto tree = factory.createTree(tree_id);

  // --- Seed blackboard with initial values ---
  auto bb = tree.rootBlackboard();
  bb->set("coin_flip", coin_flip);
  bb->set("run_mode", run_mode);
  bb->set("preferred_animal", std::string(""));
  bb->set("preferred_side", std::string(""));
  bb->set("gate_heading", 0.0);
  bb->set("inside_float_area", false);
  bb->set("buoy_touched", false);
  bb->set("marker_dropped", false);
  bb->set("torpedo_fired", false);
  bb->set("objects_placed", 0);
  bb->set("battery_pct", 100.0);
  bb->set("leak_detected", false);
  bb->set("depth", 0.0);
  double start_time = ros_node->now().seconds();
  bb->set("start_time", start_time);

  // --- Console logger for debugging ---
  BT::StdCoutLogger logger(tree);

  RCLCPP_INFO(ros_node->get_logger(), "════════════════════════════════════");
  RCLCPP_INFO(ros_node->get_logger(), "  SHRUB v3 — Mission started");
  RCLCPP_INFO(ros_node->get_logger(), "════════════════════════════════════");

  // --- Main tick loop ---
  auto tick_period = std::chrono::milliseconds(tick_rate_ms);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  constexpr double MISSION_TIMEOUT_S = 870.0;  // 14.5 min — stop before 15 min window ends
  constexpr int MAX_TICKS = 100000;             // hard upper bound on tick count
  int tick_count = 0;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    double elapsed = ros_node->now().seconds() - start_time;
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

    // Spin ROS callbacks (sensor updates, action feedback)
    rclcpp::spin_some(ros_node);

    // Tick the tree
    try {
      status = tree.tickOnce();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(ros_node->get_logger(), "BT tick exception: %s", e.what());
      tree.haltTree();
      status = BT::NodeStatus::FAILURE;
      break;
    }

    // Sleep to maintain tick rate
    std::this_thread::sleep_for(tick_period);
  }

  // --- Report final status ---
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(ros_node->get_logger(), "Mission completed: SUCCESS");
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(ros_node->get_logger(), "Mission completed: FAILURE");
  }

  double elapsed = ros_node->now().seconds() - start_time;
  double remaining = 900.0 - elapsed;  // 15 min performance window
  if (remaining > 0) {
    double time_bonus = (std::floor(remaining / 60.0) +
                         std::fmod(remaining, 60.0) / 60.0) * 100.0;
    RCLCPP_INFO(ros_node->get_logger(),
                "Time remaining: %.1fs — Potential time bonus: %.1f pts",
                remaining, time_bonus);
  }

  rclcpp::shutdown();
  return 0;
}
