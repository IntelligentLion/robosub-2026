#pragma once
// SHRUB v4 — Shared ROS I/O layer for all behavior-tree nodes.
//
// WHY THIS EXISTS
// ---------------
// The v4 BT nodes were originally stubs that returned SUCCESS instantly and
// never touched ROS. MissionIO is the single place that connects the tree to
// the (already working) Python autonomy stack:
//
//   BT node  ──MovementCommand──▶  movement_command   ──▶ thruster_node
//   BT node  ─NavigationCommand─▶  navigation_command ──▶ autonomous_controller
//   vision/detections   ──▶  cached detections   (read by perception nodes)
//   depth/info          ──▶  cached depth         (read by safety + nav)
//   localization/pose   ──▶  cached pose          (read by nav)
//
// It is a process-wide singleton created once in bt_executor from the executor's
// rclcpp::Node, then used by every node via MissionIO::get().
//
// NOTE: This file (and the nodes that use it) has NOT been compiled in this
// workspace — it must be verified with `colcon build --packages-select bt_mission`
// on the Jetson before relying on it. The message field names match
// src/auv_msgs/msg/*.msg as of this writing.

#include <rclcpp/rclcpp.hpp>
#include <auv_msgs/msg/movement_command.hpp>
#include <auv_msgs/msg/navigation_command.hpp>
#include <auv_msgs/msg/object_detection_array.hpp>
#include <auv_msgs/msg/depth_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace shrub {

// Lightweight, POD copy of one detection so callers don't hold ROS messages
// across ticks. position.x/.y are normalized image coords [0,1] from the
// vision detector; position.z is range-to-target in metres (-1 if unknown).
struct Detection {
  std::string label;
  double confidence{0.0};
  double cx{0.0};   // normalized image x [0,1]
  double cy{0.0};   // normalized image y [0,1]
  double range{-1.0};
  double bbox_w{0.0};
  double bbox_h{0.0};
};

class MissionIO {
public:
  // Create the singleton from the executor's node. Call once, in bt_executor.
  static void init(rclcpp::Node::SharedPtr node);
  // Access the singleton. Asserts (logs + returns a no-op) if init() wasn't called.
  static MissionIO& get();
  static bool ready();

  // ── Command publishers (mirror the Python MovementCommand / NavigationCommand) ──
  void sendMovement(const std::string& command, double speed = 0.0, double duration = 0.0);
  void sendNav(const std::string& mode, const std::string& target_label = "",
               double speed = 0.0, double approach_dist = 0.0,
               double target_yaw = 0.0,
               double target_x = 0.0, double target_y = 0.0, double target_z = 0.0);
  void stop();  // convenience: sendMovement("stop")

  // ── Cached sensor reads (thread-safe) ──
  double depth() const;                                   // metres, <0 if unknown
  std::vector<Detection> detections() const;              // latest frame's detections
  // Best detection matching `label` above `min_conf`; returns false if none/stale.
  bool bestDetection(const std::string& label, double min_conf, Detection& out) const;
  bool pose(double& x, double& y, double& z, double& yaw) const;  // false until first pose
  double batteryPct() const;  // 0..100, NaN if unknown
  bool leakDetected() const;  // false until first publish

  rclcpp::Node::SharedPtr node() const { return node_; }
  rclcpp::Logger logger() const { return node_->get_logger(); }

private:
  explicit MissionIO(rclcpp::Node::SharedPtr node);

  void onDetections(const auv_msgs::msg::ObjectDetectionArray::SharedPtr msg);
  void onDepth(const auv_msgs::msg::DepthInfo::SharedPtr msg);
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onBattery(const std_msgs::msg::Float32::SharedPtr msg);
  void onLeak(const std_msgs::msg::Bool::SharedPtr msg);

  static std::unique_ptr<MissionIO> inst_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<auv_msgs::msg::MovementCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<auv_msgs::msg::NavigationCommand>::SharedPtr nav_pub_;
  rclcpp::Subscription<auv_msgs::msg::ObjectDetectionArray>::SharedPtr det_sub_;
  rclcpp::Subscription<auv_msgs::msg::DepthInfo>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leak_sub_;

  mutable std::mutex mtx_;
  double depth_m_{-1.0};
  std::vector<Detection> dets_;
  rclcpp::Time dets_stamp_;
  double px_{0.0}, py_{0.0}, pz_{0.0}, pyaw_{0.0};
  bool pose_ok_{false};
  double battery_pct_{std::numeric_limits<double>::quiet_NaN()};
  bool leak_{false};
};

}  // namespace shrub
