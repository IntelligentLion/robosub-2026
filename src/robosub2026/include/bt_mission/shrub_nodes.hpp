#pragma once
// SHRUB v3 — All custom BehaviorTree.CPP v4 nodes for RoboSub 2026
// Each node maps 1:1 to a tag in robosub2026_mission.xml

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

namespace shrub {

// ═══════════════════════════════════════════════════════════════════
// CONDITION NODES — return SUCCESS/FAILURE, never RUNNING
// ═══════════════════════════════════════════════════════════════════

class IsBatteryOk : public BT::ConditionNode {
public:
  IsBatteryOk(const std::string& n, const BT::NodeConfig& c) : ConditionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("min_pct", 20.0, "Minimum battery %"),
             BT::InputPort<double>("battery_pct", "Current battery %") };
  }
  BT::NodeStatus tick() override;
};

class IsLeakDetected : public BT::ConditionNode {
public:
  IsLeakDetected(const std::string& n, const BT::NodeConfig& c) : ConditionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<bool>("leak", "Leak sensor value") };
  }
  BT::NodeStatus tick() override;
};

class IsDepthSafe : public BT::ConditionNode {
public:
  IsDepthSafe(const std::string& n, const BT::NodeConfig& c) : ConditionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("max_depth", 1.9, "Max safe depth (m)"),
             BT::InputPort<double>("min_depth", 0.0, "Min safe depth (m)"),
             BT::InputPort<double>("depth", "Current depth (m)") };
  }
  BT::NodeStatus tick() override;
};

class IsInsideFloatArea : public BT::ConditionNode {
public:
  IsInsideFloatArea(const std::string& n, const BT::NodeConfig& c) : ConditionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<bool>("val", "Inside floating area flag") };
  }
  BT::NodeStatus tick() override;
};

class IsTimeRemaining : public BT::ConditionNode {
public:
  IsTimeRemaining(const std::string& n, const BT::NodeConfig& c) : ConditionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("min_sec", 30.0, "Min seconds remaining"),
             BT::InputPort<double>("start_time", "Mission start ROS time") };
  }
  BT::NodeStatus tick() override;
};

class AlwaysSuccess : public BT::ConditionNode {
public:
  AlwaysSuccess(const std::string& n, const BT::NodeConfig& c) : ConditionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

// ═══════════════════════════════════════════════════════════════════
// ACTION NODES — Navigation
// Use StatefulActionNode for long-running actions (RUNNING state).
// Use SyncActionNode for instant operations.
// ═══════════════════════════════════════════════════════════════════

class Submerge : public BT::StatefulActionNode {
public:
  Submerge(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("target_depth", 1.2, "Target depth (m)") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class AscendTo : public BT::StatefulActionNode {
public:
  AscendTo(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("target_depth", "Target depth (m)") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class EmergencySurface : public BT::SyncActionNode {
public:
  EmergencySurface(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("reason", "Emergency reason for logging") };
  }
  BT::NodeStatus tick() override;
};

class Turn : public BT::StatefulActionNode {
public:
  Turn(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("degrees", "Degrees to rotate") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Navigate_to : public BT::StatefulActionNode {
public:
  Navigate_to(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("waypoint", "Named waypoint or blackboard ref") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Navigate_to_bearing : public BT::StatefulActionNode {
public:
  Navigate_to_bearing(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("bearing", "Bearing from pinger (deg)"),
             BT::InputPort<double>("stop_dist", 1.0, "Stop distance (m)") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Navigate_on_heading : public BT::StatefulActionNode {
public:
  Navigate_on_heading(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("heading", "Compass heading (deg)"),
             BT::InputPort<double>("dist", "Distance to travel (m)") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Navigate_forward : public BT::StatefulActionNode {
public:
  Navigate_forward(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("dist", "Distance (m)") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Move_through_gate : public BT::StatefulActionNode {
public:
  Move_through_gate(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Reposition_to_gate_side : public BT::StatefulActionNode {
public:
  Reposition_to_gate_side(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("side", "L or R") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Record_heading : public BT::SyncActionNode {
public:
  Record_heading(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::OutputPort<double>("heading", "Current heading written to BB") };
  }
  BT::NodeStatus tick() override;
};

class Compute_reverse_heading : public BT::SyncActionNode {
public:
  Compute_reverse_heading(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("gate_heading"),
             BT::OutputPort<double>("result") };
  }
  BT::NodeStatus tick() override;
};

class Recalibrate_nav : public BT::SyncActionNode {
public:
  Recalibrate_nav(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class Stabilize : public BT::StatefulActionNode {
public:
  Stabilize(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<int>("msec", 1000, "Hold time (ms)") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::chrono::steady_clock::time_point end_time_;
};

class Wait : public BT::StatefulActionNode {
public:
  Wait(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<int>("msec", "Wait duration (ms)") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::chrono::steady_clock::time_point end_time_;
};

class Hold_depth : public BT::StatefulActionNode {
public:
  Hold_depth(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("target"),
             BT::InputPort<double>("tol", 0.2, "Tolerance (m)") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Surface_in_float_area : public BT::StatefulActionNode {
public:
  Surface_in_float_area(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Face_direction : public BT::StatefulActionNode {
public:
  Face_direction(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("dir") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Follow_path : public BT::StatefulActionNode {
public:
  Follow_path(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("detection") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Circle_around : public BT::StatefulActionNode {
public:
  Circle_around(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("target") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

// ═══════════════════════════════════════════════════════════════════
// ACTION NODES — Perception (all write detection results to BB)
// ═══════════════════════════════════════════════════════════════════

// Macro: all perception nodes follow the same pattern
#define DECLARE_DETECT_NODE(ClassName, ...)                          \
class ClassName : public BT::StatefulActionNode {                    \
public:                                                              \
  ClassName(const std::string& n, const BT::NodeConfig& c)           \
    : StatefulActionNode(n, c) {}                                    \
  static BT::PortsList providedPorts() { return { __VA_ARGS__ }; }   \
  BT::NodeStatus onStart() override;                                 \
  BT::NodeStatus onRunning() override;                               \
  void onHalted() override;                                          \
};

DECLARE_DETECT_NODE(Detect_gate,
  BT::OutputPort<std::string>("result"))

DECLARE_DETECT_NODE(Detect_animal_on_gate,
  BT::OutputPort<std::string>("animal"),
  BT::OutputPort<std::string>("side"),
  BT::OutputPort<double>("conf"))

DECLARE_DETECT_NODE(Detect_animal_image,
  BT::InputPort<std::string>("target"),
  BT::OutputPort<double>("dir"))

DECLARE_DETECT_NODE(Detect_pinger,
  BT::InputPort<int>("freq_min", 25000, ""),
  BT::InputPort<int>("freq_max", 40000, ""),
  BT::OutputPort<double>("bearing"))

DECLARE_DETECT_NODE(Detect_float_area_below,
  BT::OutputPort<std::string>("result"))

DECLARE_DETECT_NODE(Confirm_overhead,
  BT::OutputPort<bool>("is_inside"))

DECLARE_DETECT_NODE(Detect_buoy,
  BT::OutputPort<std::string>("result"))

DECLARE_DETECT_NODE(Detect_bin_below,
  BT::OutputPort<std::string>("result"),
  BT::OutputPort<std::string>("correct_half"))

DECLARE_DETECT_NODE(Detect_object,
  BT::OutputPort<std::string>("result"),
  BT::OutputPort<std::string>("type"))

DECLARE_DETECT_NODE(Detect_slalom_pipes,
  BT::OutputPort<std::string>("result"))

DECLARE_DETECT_NODE(Detect_task_board,
  BT::InputPort<std::string>("task"),
  BT::OutputPort<std::string>("result"))

DECLARE_DETECT_NODE(Detect_opening,
  BT::InputPort<std::string>("animal"),
  BT::OutputPort<std::string>("result"))

DECLARE_DETECT_NODE(Detect_path_marker,
  BT::InputPort<std::string>("color"),
  BT::OutputPort<std::string>("result"))

DECLARE_DETECT_NODE(Detect_vertical_marker,
  BT::OutputPort<std::string>("result"))

#undef DECLARE_DETECT_NODE

// ═══════════════════════════════════════════════════════════════════
// ACTION NODES — Alignment (visual servoing to center on targets)
// ═══════════════════════════════════════════════════════════════════

class Align_to : public BT::StatefulActionNode {
public:
  Align_to(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("detection") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Align_above : public BT::StatefulActionNode {
public:
  Align_above(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("target"),
             BT::InputPort<std::string>("half"),
             BT::InputPort<std::string>("animal") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Align_to_opening : public BT::StatefulActionNode {
public:
  Align_to_opening(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("opening") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Align_to_basket : public BT::StatefulActionNode {
public:
  Align_to_basket(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("basket") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Center_beneath : public BT::StatefulActionNode {
public:
  Center_beneath(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("target") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Approach_and_touch : public BT::StatefulActionNode {
public:
  Approach_and_touch(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("target") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

// ═══════════════════════════════════════════════════════════════════
// ACTION NODES — Manipulation
// ═══════════════════════════════════════════════════════════════════

class Grab_object : public BT::StatefulActionNode {
public:
  Grab_object(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Release_object : public BT::SyncActionNode {
public:
  Release_object(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class Drop_marker : public BT::SyncActionNode {
public:
  Drop_marker(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("id", "Marker 1 or 2") };
  }
  BT::NodeStatus tick() override;
};

class Fire_torpedo : public BT::SyncActionNode {
public:
  Fire_torpedo(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("tube", "Tube 1 or 2") };
  }
  BT::NodeStatus tick() override;
};

// ═══════════════════════════════════════════════════════════════════
// ACTION NODES — Task Logic
// ═══════════════════════════════════════════════════════════════════

class Style_through_gate : public BT::StatefulActionNode {
public:
  Style_through_gate(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("type", "barrel_roll", "Style type"),
             BT::InputPort<std::string>("det") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Determine_basket : public BT::SyncActionNode {
public:
  Determine_basket(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("obj_type"),
             BT::OutputPort<std::string>("basket") };
  }
  BT::NodeStatus tick() override;
};

class Compute_slalom_path : public BT::SyncActionNode {
public:
  Compute_slalom_path(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("side"),
             BT::InputPort<std::string>("pipes"),
             BT::OutputPort<std::string>("waypoint") };
  }
  BT::NodeStatus tick() override;
};

class Rotation_bonus : public BT::StatefulActionNode {
public:
  Rotation_bonus(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<int>("count") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Increment : public BT::SyncActionNode {
public:
  Increment(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("key"),
             BT::BidirectionalPort<int>("val") };
  }
  BT::NodeStatus tick() override;
};

// ═══════════════════════════════════════════════════════════════════
// Registration function — call from bt_executor main()
// ═══════════════════════════════════════════════════════════════════
void registerAllNodes(BT::BehaviorTreeFactory& factory,
                      rclcpp::Node::SharedPtr ros_node);

} // namespace shrub
