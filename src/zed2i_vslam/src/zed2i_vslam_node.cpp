#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>
#include <mutex>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

static void quat_to_rpy(const geometry_msgs::msg::Quaternion & q,
                         double & r, double & p, double & y) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3(tq).getRPY(r, p, y);
}
static double pose_distance(const geometry_msgs::msg::Pose & a,
                              const geometry_msgs::msg::Pose & b) {
  double dx=a.position.x-b.position.x, dy=a.position.y-b.position.y, dz=a.position.z-b.position.z;
  return std::sqrt(dx*dx+dy*dy+dz*dz);
}
static double pose_angle_delta(const geometry_msgs::msg::Pose & a,
                                 const geometry_msgs::msg::Pose & b) {
  double ra,pa,ya,rb,pb,yb;
  quat_to_rpy(a.orientation,ra,pa,ya);
  quat_to_rpy(b.orientation,rb,pb,yb);
  double dr=ra-rb,dp=pa-pb,dy=ya-yb;
  return std::sqrt(dr*dr+dp*dp+dy*dy);
}

class Zed2iVslamNode : public rclcpp::Node {
public:
  Zed2iVslamNode() : Node("zed2i_vslam_node") {
    declare_parameter("camera_name",          "zed2i");
    declare_parameter("jerk_distance_thresh", 0.35);
    declare_parameter("jerk_angle_thresh",    0.50);
    declare_parameter("jerk_hold_frames",     2);
    declare_parameter("max_path_poses",       3000);
    declare_parameter("path_pub_rate_hz",     10.0);
    declare_parameter("publish_diagnostics",  true);
    declare_parameter("odom_frame",           "odom");
    declare_parameter("map_frame",            "map");
    declare_parameter("base_frame",           "base_link");

    camera_name_ = get_parameter("camera_name").as_string();
    jerk_dist_   = get_parameter("jerk_distance_thresh").as_double();
    jerk_angle_  = get_parameter("jerk_angle_thresh").as_double();
    jerk_hold_   = get_parameter("jerk_hold_frames").as_int();
    max_path_    = get_parameter("max_path_poses").as_int();
    odom_frame_  = get_parameter("odom_frame").as_string();
    map_frame_   = get_parameter("map_frame").as_string();
    pub_diag_    = get_parameter("publish_diagnostics").as_bool();
    double path_hz = get_parameter("path_pub_rate_hz").as_double();

    std::string ns = "/" + camera_name_ + "/zed_node";

    rclcpp::QoS sq(10); sq.best_effort().durability_volatile();
    rclcpp::QoS rq(10); rq.reliable().durability_volatile();

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ns+"/pose", sq, std::bind(&Zed2iVslamNode::pose_cb, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      ns+"/odom", sq, std::bind(&Zed2iVslamNode::odom_cb, this, std::placeholders::_1));
    sub_imu_  = create_subscription<sensor_msgs::msg::Imu>(
      ns+"/imu/data", sq, std::bind(&Zed2iVslamNode::imu_cb, this, std::placeholders::_1));

    pub_filtered_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/filtered_pose", rq);
    pub_filtered_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/filtered_odom", rq);
    pub_path_map_      = create_publisher<nav_msgs::msg::Path>("~/path_map", rq);
    pub_path_odom_     = create_publisher<nav_msgs::msg::Path>("~/path_odom", rq);
    pub_velocity_      = create_publisher<std_msgs::msg::Float32>("~/velocity_mps", rq);
    pub_jerk_marker_   = create_publisher<visualization_msgs::msg::Marker>("~/jerk_events", rq);
    if (pub_diag_)
      pub_diag_status_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/diagnostics", rq);

    path_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0/path_hz),
      std::bind(&Zed2iVslamNode::publish_paths, this));
    if (pub_diag_)
      diag_timer_ = create_wall_timer(1s, std::bind(&Zed2iVslamNode::publish_diagnostics, this));

    RCLCPP_INFO(get_logger(), "zed2i_vslam_node started | camera=%s jerk_dist=%.2f jerk_angle=%.2f",
      camera_name_.c_str(), jerk_dist_, jerk_angle_);
  }

private:
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    double ax=msg->linear_acceleration.x, ay=msg->linear_acceleration.y, az=msg->linear_acceleration.z-9.81;
    imu_accel_mag_ = std::sqrt(ax*ax+ay*ay+az*az);
    double wx=msg->angular_velocity.x, wy=msg->angular_velocity.y, wz=msg->angular_velocity.z;
    imu_gyro_mag_ = std::sqrt(wx*wx+wy*wy+wz*wz);
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    total_pose_msgs_++;
    bool is_jerk = false;
    if (has_last_pose_) {
      double dist  = pose_distance(msg->pose, last_pose_.pose);
      double angle = pose_angle_delta(msg->pose, last_pose_.pose);
      if (dist > jerk_dist_ || angle > jerk_angle_) {
        jerk_counter_++;
        is_jerk = (jerk_counter_ < jerk_hold_);
        if (!is_jerk) {
          RCLCPP_WARN(get_logger(), "Large motion accepted after %d frames: dist=%.3fm angle=%.3frad",
            jerk_hold_, dist, angle);
          jerk_counter_ = 0;
          jerk_events_total_++;
          publish_jerk_marker(msg->header, msg->pose);
        }
      } else { jerk_counter_ = 0; }
    }
    if (is_jerk) {
      jerk_filtered_++;
      auto out = last_pose_; out.header.stamp = msg->header.stamp;
      pub_filtered_pose_->publish(out);
      append_map_path(out); return;
    }
    if (has_last_pose_) {
      double dt = (rclcpp::Time(msg->header.stamp)-rclcpp::Time(last_pose_.header.stamp)).seconds();
      if (dt > 1e-6) {
        vel_window_.push_back(pose_distance(msg->pose, last_pose_.pose)/dt);
        if ((int)vel_window_.size() > 10) vel_window_.pop_front();
        double sum=0; for (auto v:vel_window_) sum+=v;
        std_msgs::msg::Float32 vm; vm.data = static_cast<float>(sum/vel_window_.size());
        pub_velocity_->publish(vm); current_velocity_ = vm.data;
      }
    }
    last_pose_=*msg; has_last_pose_=true;
    pub_filtered_pose_->publish(*msg);
    append_map_path(*msg);
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
      } else { odom_jerk_counter_ = 0; }
    }
    if (is_jerk) {
      auto out = last_odom_; out.header.stamp = msg->header.stamp;
      pub_filtered_odom_->publish(out); append_odom_path(out); return;
    }
    last_odom_=*msg; has_last_odom_=true;
    pub_filtered_odom_->publish(*msg); append_odom_path(*msg);
  }

  void append_map_path(const geometry_msgs::msg::PoseStamped & ps) {
    std::lock_guard<std::mutex> lk(path_mtx_);
    path_map_.header=ps.header; path_map_.header.frame_id=map_frame_;
    path_map_.poses.push_back(ps);
    if (max_path_>0 && (int)path_map_.poses.size()>max_path_)
      path_map_.poses.erase(path_map_.poses.begin());
  }
  void append_odom_path(const nav_msgs::msg::Odometry & odom) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header=odom.header; ps.header.frame_id=odom_frame_; ps.pose=odom.pose.pose;
    std::lock_guard<std::mutex> lk(path_mtx_);
    path_odom_.header=ps.header; path_odom_.header.frame_id=odom_frame_;
    path_odom_.poses.push_back(ps);
    if (max_path_>0 && (int)path_odom_.poses.size()>max_path_)
      path_odom_.poses.erase(path_odom_.poses.begin());
  }
  void publish_paths() {
    std::lock_guard<std::mutex> lk(path_mtx_);
    if (!path_map_.poses.empty())  pub_path_map_->publish(path_map_);
    if (!path_odom_.poses.empty()) pub_path_odom_->publish(path_odom_);
  }
  void publish_jerk_marker(const std_msgs::msg::Header & header,
                             const geometry_msgs::msg::Pose & pose) {
    visualization_msgs::msg::Marker m;
    m.header=header; m.header.frame_id=map_frame_;
    m.ns="jerk_events"; m.id=jerk_events_total_;
    m.type=visualization_msgs::msg::Marker::SPHERE;
    m.action=visualization_msgs::msg::Marker::ADD;
    m.pose=pose; m.scale.x=m.scale.y=m.scale.z=0.15;
    m.color.r=1.0f; m.color.g=0.3f; m.color.b=0.0f; m.color.a=0.9f;
    m.lifetime=rclcpp::Duration(10,0);
    pub_jerk_marker_->publish(m);
  }
  void publish_diagnostics() {
    if (!pub_diag_) return;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name="zed2i_vslam_node"; status.hardware_id=camera_name_;
    auto add = [&](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue kv; kv.key=k; kv.value=v;
      status.values.push_back(kv);
    };
    add("total_pose_msgs",      std::to_string(total_pose_msgs_));
    add("total_odom_msgs",      std::to_string(total_odom_msgs_));
    add("jerk_filtered_frames", std::to_string(jerk_filtered_));
    add("jerk_events_total",    std::to_string(jerk_events_total_));
    add("velocity_mps",         std::to_string(current_velocity_));
    add("map_path_size",        std::to_string(path_map_.poses.size()));
    status.level   = has_last_pose_ ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                    : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = has_last_pose_ ? "Tracking OK" : "Waiting for first pose";
    pub_diag_status_->publish(status);
  }

  std::string camera_name_, odom_frame_, map_frame_;
  double jerk_dist_, jerk_angle_; int jerk_hold_, max_path_; bool pub_diag_;
  std::mutex pose_mtx_, odom_mtx_, imu_mtx_, path_mtx_;
  geometry_msgs::msg::PoseStamped last_pose_;
  nav_msgs::msg::Odometry last_odom_;
  bool has_last_pose_{false}, has_last_odom_{false};
  int jerk_counter_{0}, odom_jerk_counter_{0};
  uint64_t total_pose_msgs_{0}, total_odom_msgs_{0}, jerk_filtered_{0}, jerk_events_total_{0};
  float current_velocity_{0.0f};
  double imu_accel_mag_{0.0}, imu_gyro_mag_{0.0};
  std::deque<double> vel_window_;
  nav_msgs::msg::Path path_map_, path_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_filtered_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_filtered_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_map_, pub_path_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_velocity_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_jerk_marker_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_status_;
  rclcpp::TimerBase::SharedPtr path_timer_, diag_timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zed2iVslamNode>());
  rclcpp::shutdown();
  return 0;
}
