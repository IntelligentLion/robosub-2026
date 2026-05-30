// SHRUB v4 — Shared ROS I/O layer implementation. See mission_io.hpp.
//
// NOT YET COMPILED IN THIS WORKSPACE — verify with
//   colcon build --packages-select bt_mission
#include <bt_mission/mission_io.hpp>

#include <cmath>
#include <stdexcept>

namespace shrub {

std::unique_ptr<MissionIO> MissionIO::inst_ = nullptr;

void MissionIO::init(rclcpp::Node::SharedPtr node) {
  if (inst_) {
    RCLCPP_WARN(node->get_logger(), "MissionIO::init called twice — ignoring");
    return;
  }
  inst_ = std::unique_ptr<MissionIO>(new MissionIO(node));
}

bool MissionIO::ready() { return inst_ != nullptr; }

MissionIO& MissionIO::get() {
  // If this fires, bt_executor forgot to call MissionIO::init(node) first.
  if (!inst_) {
    throw std::runtime_error("MissionIO::get() before init() — call init() in bt_executor");
  }
  return *inst_;
}

MissionIO::MissionIO(rclcpp::Node::SharedPtr node)
    : node_(node), dets_stamp_(node->now()) {
  move_pub_ = node_->create_publisher<auv_msgs::msg::MovementCommand>("movement_command", 10);
  nav_pub_  = node_->create_publisher<auv_msgs::msg::NavigationCommand>("navigation_command", 10);

  det_sub_ = node_->create_subscription<auv_msgs::msg::ObjectDetectionArray>(
      "vision/detections", 10,
      std::bind(&MissionIO::onDetections, this, std::placeholders::_1));
  depth_sub_ = node_->create_subscription<auv_msgs::msg::DepthInfo>(
      "depth/info", 10,
      std::bind(&MissionIO::onDepth, this, std::placeholders::_1));
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "localization/pose", 10,
      std::bind(&MissionIO::onPose, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
              "MissionIO ready — publishing movement_command / navigation_command, "
              "subscribed to vision/detections, depth/info, localization/pose");
}

// ─── Publishers ──────────────────────────────────────────────────
void MissionIO::sendMovement(const std::string& command, double speed, double duration) {
  auv_msgs::msg::MovementCommand msg;
  msg.command = command;
  msg.speed = static_cast<float>(speed);
  msg.duration = static_cast<float>(duration);
  move_pub_->publish(msg);
}

void MissionIO::sendNav(const std::string& mode, const std::string& target_label,
                        double speed, double approach_dist) {
  auv_msgs::msg::NavigationCommand msg;
  msg.mode = mode;
  msg.target_label = target_label;
  msg.speed = static_cast<float>(speed);
  msg.approach_dist = static_cast<float>(approach_dist);
  nav_pub_->publish(msg);
}

void MissionIO::stop() { sendMovement("stop"); }

// ─── Sensor callbacks ────────────────────────────────────────────
void MissionIO::onDetections(const auv_msgs::msg::ObjectDetectionArray::SharedPtr msg) {
  std::vector<Detection> out;
  out.reserve(msg->detections.size());
  for (const auto& d : msg->detections) {
    Detection det;
    det.label = d.label;
    det.confidence = d.confidence;
    det.cx = d.position.x;
    det.cy = d.position.y;
    det.range = d.position.z;
    det.bbox_w = d.bbox_width;
    det.bbox_h = d.bbox_height;
    out.push_back(std::move(det));
  }
  std::lock_guard<std::mutex> lk(mtx_);
  dets_ = std::move(out);
  dets_stamp_ = node_->now();
}

void MissionIO::onDepth(const auv_msgs::msg::DepthInfo::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (std::isfinite(msg->sub_depth_m)) depth_m_ = msg->sub_depth_m;
}

void MissionIO::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const auto& p = msg->pose.position;
  const auto& q = msg->pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  std::lock_guard<std::mutex> lk(mtx_);
  px_ = p.x; py_ = p.y; pz_ = p.z; pyaw_ = yaw;
  pose_ok_ = true;
}

// ─── Sensor reads ────────────────────────────────────────────────
double MissionIO::depth() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return depth_m_;
}

std::vector<Detection> MissionIO::detections() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return dets_;
}

bool MissionIO::bestDetection(const std::string& label, double min_conf, Detection& out) const {
  constexpr double STALE_S = 2.0;
  std::lock_guard<std::mutex> lk(mtx_);
  if ((node_->now() - dets_stamp_).seconds() > STALE_S) return false;
  bool found = false;
  for (const auto& d : dets_) {
    if (d.label == label && d.confidence >= min_conf &&
        (!found || d.confidence > out.confidence)) {
      out = d;
      found = true;
    }
  }
  return found;
}

bool MissionIO::pose(double& x, double& y, double& z, double& yaw) const {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!pose_ok_) return false;
  x = px_; y = py_; z = pz_; yaw = pyaw_;
  return true;
}

}  // namespace shrub
