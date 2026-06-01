#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <deque>
#include <mutex>
#include <cmath>
#include <chrono>
#include <string>
#include <array>

using namespace std::chrono_literals;

// ─── Constants ────────────────────────────────────────────────────────────────

// How far a single pose jump (metres) is considered "jerk" vs legitimate motion
static constexpr double JERK_DISTANCE_THRESH_M   = 0.50;   // 50 cm/frame
// How far an angular jump (radians) triggers jerk detection
static constexpr double JERK_ANGLE_THRESH_RAD     = 0.70;   // ~40 deg/frame
// How many consecutive bad frames before we treat jerk as real motion
static constexpr int    JERK_HOLD_FRAMES          = 3;
// Sliding-window size for velocity smoothing
static constexpr int    VELOCITY_WINDOW           = 10;
// Path history: max poses kept (set -1 for unlimited)
static constexpr int    MAX_PATH_POSES            = 2000;
// How often (seconds) we re-publish the full path (low rate, just for RViz)
static constexpr double PATH_PUB_RATE_HZ          = 5.0;

// ─── Helper: quaternion → roll/pitch/yaw ─────────────────────────────────────
static void quat_to_rpy(const geometry_msgs::msg::Quaternion & q,
                         double & r, double & p, double & y)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3(tq).getRPY(r, p, y);
}

static double pose_distance(const geometry_msgs::msg::Pose & a,
                             const geometry_msgs::msg::Pose & b)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  double dz = a.position.z - b.position.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

static double pose_angle_delta(const geometry_msgs::msg::Pose & a,
                                const geometry_msgs::msg::Pose & b)
{
  double ra, pa, ya, rb, pb, yb;
  quat_to_rpy(a.orientation, ra, pa, ya);
  quat_to_rpy(b.orientation, rb, pb, yb);
  double dr = ra-rb, dp = pa-pb, dy = ya-yb;
  return std::sqrt(dr*dr + dp*dp + dy*dy);
}

// ─── Node ─────────────────────────────────────────────────────────────────────

class Zed2iVslamNode : public rclcpp::Node
{
public:
  Zed2iVslamNode() : Node("zed2i_vslam_node")
  {
    // ── Declare parameters ──────────────────────────────────────────────────
    declare_parameter("camera_name",               "zed2i");
    declare_parameter("jerk_distance_thresh",      JERK_DISTANCE_THRESH_M);
    declare_parameter("jerk_angle_thresh",         JERK_ANGLE_THRESH_RAD);
    declare_parameter("jerk_hold_frames",          JERK_HOLD_FRAMES);
    declare_parameter("max_path_poses",            MAX_PATH_POSES);
    declare_parameter("path_pub_rate_hz",          PATH_PUB_RATE_HZ);
    declare_parameter("publish_diagnostics",       true);
    declare_parameter("odom_frame",                "odom");
    declare_parameter("map_frame",                 "map");
    declare_parameter("base_frame",                "base_link");

    camera_name_    = get_parameter("camera_name").as_string();
    jerk_dist_      = get_parameter("jerk_distance_thresh").as_double();
    jerk_angle_     = get_parameter("jerk_angle_thresh").as_double();
    jerk_hold_      = get_parameter("jerk_hold_frames").as_int();
    max_path_       = get_parameter("max_path_poses").as_int();
    odom_frame_     = get_parameter("odom_frame").as_string();
    map_frame_      = get_parameter("map_frame").as_string();
    base_frame_     = get_parameter("base_frame").as_string();
    pub_diag_       = get_parameter("publish_diagnostics").as_bool();
    double path_hz  = get_parameter("path_pub_rate_hz").as_double();

    std::string ns = "/" + camera_name_ + "/zed_node";

    // ── QoS: best_effort + volatile matches ZED wrapper publisher ───────────
    rclcpp::QoS sensor_qos(10);
    sensor_qos.best_effort().durability_volatile();

    rclcpp::QoS reliable_qos(10);
    reliable_qos.reliable().durability_volatile();

    // ── Subscribers ─────────────────────────────────────────────────────────
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/pose", sensor_qos,
      std::bind(&Zed2iVslamNode::pose_cb, this, std::placeholders::_1));

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      ns + "/odom", sensor_qos,
      std::bind(&Zed2iVslamNode::odom_cb, this, std::placeholders::_1));

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      ns + "/imu/data", sensor_qos,
      std::bind(&Zed2iVslamNode::imu_cb, this, std::placeholders::_1));

    // ── Publishers ───────────────────────────────────────────────────────────
    pub_filtered_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/filtered_pose", reliable_qos);

    pub_filtered_odom_ = create_publisher<nav_msgs::msg::Odometry>(
      "~/filtered_odom", reliable_qos);

    pub_path_map_ = create_publisher<nav_msgs::msg::Path>(
      "~/path_map", reliable_qos);

    pub_path_odom_ = create_publisher<nav_msgs::msg::Path>(
      "~/path_odom", reliable_qos);

    pub_velocity_ = create_publisher<std_msgs::msg::Float32>(
      "~/velocity_mps", reliable_qos);

    pub_jerk_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/jerk_events", reliable_qos);

    if (pub_diag_) {
      pub_diag_status_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "~/diagnostics", reliable_qos);
    }

    // ── Path publisher timer ─────────────────────────────────────────────────
    path_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / path_hz),
      std::bind(&Zed2iVslamNode::publish_paths, this));

    // ── Diagnostics timer ────────────────────────────────────────────────────
    if (pub_diag_) {
      diag_timer_ = create_wall_timer(1s,
        std::bind(&Zed2iVslamNode::publish_diagnostics, this));
    }

    RCLCPP_INFO(get_logger(),
      "zed2i_vslam_node started. Tracking camera: %s  |  jerk_dist=%.2fm  jerk_angle=%.2frad",
      camera_name_.c_str(), jerk_dist_, jerk_angle_);
  }

private:

  // ── IMU callback: track linear acceleration magnitude for jerk detection ──
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;
    // remove gravity (~9.81) from z to get dynamic acceleration
    double dyn_z = az - 9.81;
    imu_accel_mag_ = std::sqrt(ax*ax + ay*ay + dyn_z*dyn_z);

    // Track angular velocity for spin/roll detection
    double wx = msg->angular_velocity.x;
    double wy = msg->angular_velocity.y;
    double wz = msg->angular_velocity.z;
    imu_gyro_mag_ = std::sqrt(wx*wx + wy*wy + wz*wz);
  }

  // ── Pose callback: map-frame corrected pose from ZED VSLAM ───────────────
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    total_pose_msgs_++;

    // ── Jerk detection ───────────────────────────────────────────────────────
    bool is_jerk = false;
    if (has_last_pose_) {
      double dist  = pose_distance(msg->pose, last_pose_.pose);
      double angle = pose_angle_delta(msg->pose, last_pose_.pose);

      if (dist > jerk_dist_ || angle > jerk_angle_) {
        jerk_counter_++;
        is_jerk = (jerk_counter_ < jerk_hold_);
        if (!is_jerk) {
          // Confirmed real large motion — accept it and reset counter
          RCLCPP_WARN(get_logger(),
            "Large motion accepted after %d frames: dist=%.3fm angle=%.3frad",
            jerk_hold_, dist, angle);
          jerk_counter_ = 0;
          jerk_events_total_++;
          publish_jerk_marker(msg->header, msg->pose);
        }
      } else {
        jerk_counter_ = 0;
      }
    }

    if (is_jerk) {
      // Suppress this frame — use last valid pose with updated timestamp
      jerk_filtered_++;
      auto out = last_pose_;
      out.header.stamp = msg->header.stamp;
      pub_filtered_pose_->publish(out);

      // Still record path using last pose (no gap in path)
      append_map_path(out);
      return;
    }

    // ── Valid pose: update velocity estimate ─────────────────────────────────
    if (has_last_pose_) {
      double dt = (rclcpp::Time(msg->header.stamp) -
                   rclcpp::Time(last_pose_.header.stamp)).seconds();
      if (dt > 1e-6) {
        double dist = pose_distance(msg->pose, last_pose_.pose);
        double vel  = dist / dt;
        vel_window_.push_back(vel);
        if ((int)vel_window_.size() > VELOCITY_WINDOW) {
          vel_window_.pop_front();
        }
        // Publish smoothed velocity
        double sum = 0;
        for (auto v : vel_window_) sum += v;
        std_msgs::msg::Float32 vmsg;
        vmsg.data = static_cast<float>(sum / vel_window_.size());
        pub_velocity_->publish(vmsg);
        current_velocity_ = vmsg.data;
      }
    }

    last_pose_       = *msg;
    has_last_pose_   = true;

    pub_filtered_pose_->publish(*msg);
    append_map_path(*msg);
  }

  // ── Odometry callback: odom-frame VIO (higher rate) ──────────────────────
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    total_odom_msgs_++;

    bool is_jerk = false;
    if (has_last_odom_) {
      double dist  = pose_distance(msg->pose.pose, last_odom_.pose.pose);
      double angle = pose_angle_delta(msg->pose.pose, last_odom_.pose.pose);

      if (dist > jerk_dist_ || angle > jerk_angle_) {
        odom_jerk_counter_++;
        is_jerk = (odom_jerk_counter_ < jerk_hold_);
        if (!is_jerk) odom_jerk_counter_ = 0;
      } else {
        odom_jerk_counter_ = 0;
      }
    }

    if (is_jerk) {
      // Hold last odometry, update timestamp only
      auto out = last_odom_;
      out.header.stamp = msg->header.stamp;
      pub_filtered_odom_->publish(out);
      append_odom_path(out);
      return;
    }

    last_odom_     = *msg;
    has_last_odom_ = true;
    pub_filtered_odom_->publish(*msg);
    append_odom_path(*msg);
  }

  // ── Path helpers ─────────────────────────────────────────────────────────
  void append_map_path(const geometry_msgs::msg::PoseStamped & ps)
  {
    std::lock_guard<std::mutex> lk(path_mtx_);
    path_map_.header = ps.header;
    path_map_.header.frame_id = map_frame_;
    path_map_.poses.push_back(ps);
    if (max_path_ > 0 && (int)path_map_.poses.size() > max_path_) {
      path_map_.poses.erase(path_map_.poses.begin());
    }
  }

  void append_odom_path(const nav_msgs::msg::Odometry & odom)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.header.frame_id = odom_frame_;
    ps.pose = odom.pose.pose;

    std::lock_guard<std::mutex> lk(path_mtx_);
    path_odom_.header = ps.header;
    path_odom_.header.frame_id = odom_frame_;
    path_odom_.poses.push_back(ps);
    if (max_path_ > 0 && (int)path_odom_.poses.size() > max_path_) {
      path_odom_.poses.erase(path_odom_.poses.begin());
    }
  }

  // ── Periodic path publish ─────────────────────────────────────────────────
  void publish_paths()
  {
    std::lock_guard<std::mutex> lk(path_mtx_);
    if (!path_map_.poses.empty())  pub_path_map_->publish(path_map_);
    if (!path_odom_.poses.empty()) pub_path_odom_->publish(path_odom_);
  }

  // ── Jerk event marker for RViz ───────────────────────────────────────────
  void publish_jerk_marker(const std_msgs::msg::Header & header,
                            const geometry_msgs::msg::Pose & pose)
  {
    visualization_msgs::msg::Marker m;
    m.header         = header;
    m.header.frame_id = map_frame_;
    m.ns             = "jerk_events";
    m.id             = jerk_events_total_;
    m.type           = visualization_msgs::msg::Marker::SPHERE;
    m.action         = visualization_msgs::msg::Marker::ADD;
    m.pose           = pose;
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 0.9f;
    m.lifetime       = rclcpp::Duration(10, 0);  // visible for 10 s
    pub_jerk_marker_->publish(m);
  }

  // ── Diagnostics ──────────────────────────────────────────────────────────
  void publish_diagnostics()
  {
    if (!pub_diag_) return;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name    = "zed2i_vslam_node";
    status.hardware_id = camera_name_;

    auto add = [&](const std::string & key, const std::string & val) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key; kv.value = val;
      status.values.push_back(kv);
    };

    double imu_a, imu_g;
    {
      std::lock_guard<std::mutex> lk(imu_mtx_);
      imu_a = imu_accel_mag_;
      imu_g = imu_gyro_mag_;
    }

    add("total_pose_msgs",      std::to_string(total_pose_msgs_));
    add("total_odom_msgs",      std::to_string(total_odom_msgs_));
    add("jerk_filtered_frames", std::to_string(jerk_filtered_));
    add("jerk_events_total",    std::to_string(jerk_events_total_));
    add("velocity_mps",         std::to_string(current_velocity_));
    add("imu_accel_mag",        std::to_string(imu_a));
    add("imu_gyro_mag",         std::to_string(imu_g));
    add("map_path_size",        std::to_string(path_map_.poses.size()));
    add("odom_path_size",       std::to_string(path_odom_.poses.size()));

    bool ok = has_last_pose_;
    status.level   = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
                        : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = ok ? "Tracking OK" : "Waiting for first pose";

    pub_diag_status_->publish(status);
  }

  // ── Members ───────────────────────────────────────────────────────────────
  std::string camera_name_, odom_frame_, map_frame_, base_frame_;
  double      jerk_dist_   {JERK_DISTANCE_THRESH_M};
  double      jerk_angle_  {JERK_ANGLE_THRESH_RAD};
  int         jerk_hold_   {JERK_HOLD_FRAMES};
  int         max_path_    {MAX_PATH_POSES};
  bool        pub_diag_    {true};

  // pose state
  std::mutex                          pose_mtx_, odom_mtx_, imu_mtx_, path_mtx_;
  geometry_msgs::msg::PoseStamped     last_pose_;
  nav_msgs::msg::Odometry             last_odom_;
  bool                                has_last_pose_  {false};
  bool                                has_last_odom_  {false};
  int                                 jerk_counter_       {0};
  int                                 odom_jerk_counter_  {0};

  // stats
  uint64_t total_pose_msgs_    {0};
  uint64_t total_odom_msgs_    {0};
  uint64_t jerk_filtered_      {0};
  uint64_t jerk_events_total_  {0};
  float    current_velocity_   {0.0f};

  // IMU state
  double imu_accel_mag_ {0.0};
  double imu_gyro_mag_  {0.0};

  // velocity smoothing
  std::deque<double> vel_window_;

  // paths
  nav_msgs::msg::Path path_map_;
  nav_msgs::msg::Path path_odom_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    sub_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              sub_imu_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       pub_filtered_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               pub_filtered_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                   pub_path_map_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                   pub_path_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                pub_velocity_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr       pub_jerk_marker_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_status_;

  // timers
  rclcpp::TimerBase::SharedPtr path_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zed2iVslamNode>());
  rclcpp::shutdown();
  return 0;
}
