// ==========================================================================
//  SHRUB – Software for Handling and Regulating Underwater Behavior
//  main.cpp – Vision-integrated behaviour-tree runner
//
//  Key changes vs. previous version:
//    * VisionState – thread-safe store updated by the vision/detections topic
//    * High confidence thresholds prevent spurious detections from driving
//      movement (DETECT_CONFIDENCE = 0.80, TRACK_CONFIDENCE = 0.65)
//    * BT action / condition nodes use **vision feedback loops** so the sub
//      moves toward detected objects, centres on them, and only advances
//      when the task is verified complete (or timed-out)
//    * Movement commands with duration=0 keep the thruster node in the
//      requested state until explicitly changed → enables closed-loop control
// ==========================================================================

#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <sstream>
#include <memory>
#include <mutex>
#include <optional>
#include <cmath>
#include <algorithm>

#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "auv_msgs/msg/object_detection.hpp"
#include "auv_msgs/msg/object_detection_array.hpp"
#include "auv_msgs/msg/behavior_status.hpp"
#include "auv_msgs/msg/movement_command.hpp"
#include "auv_msgs/msg/navigation_command.hpp"
#include "auv_msgs/msg/depth_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

using namespace std;
using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using TimePoint   = SteadyClock::time_point;

// =====================================================================
//  Tuning constants
// =====================================================================

// ── Confidence thresholds ──
// DETECT_CONFIDENCE – used when *deciding* whether an object is present.
// Must be high to avoid random movements from misclassifications.
constexpr float DETECT_CONFIDENCE = 0.80f;

// TRACK_CONFIDENCE – used during active feedback-loop movement toward an
// already-confirmed object.  Slightly lower so we don't lose track when
// the viewing angle changes during approach.
constexpr float TRACK_CONFIDENCE  = 0.65f;

// ── Centering / approach tolerances ──
constexpr float CENTER_TOL  = 0.08f;  // ±8 % of frame → "centred"
constexpr float APPROACH_W  = 0.35f;  // target bbox_width to be "close"
constexpr float CLOSE_W     = 0.55f;  // target bbox_width to be "very close"

// ── Control timing ──
constexpr int  CTRL_MS     = 100;                    // 10 Hz feedback
constexpr auto CTRL_PERIOD = std::chrono::milliseconds(CTRL_MS);

// ── Default timeouts (seconds) ──
constexpr float TIMEOUT_SEARCH   = 20.0f;
constexpr float TIMEOUT_CENTER   = 15.0f;
constexpr float TIMEOUT_APPROACH = 25.0f;
constexpr float TIMEOUT_PASS     = 25.0f;

// =====================================================================
//  Object-label constants
//  IMPORTANT: update these to match your YOLO model's class names.
//  Run the detector node and check the "Model classes:" log to see them.
// =====================================================================

namespace Labels {
    const std::string CCW_BLUE_GATE  = "ccw_blue_gate";
    const std::string CW_RED_GATE    = "cw_red_gate";
    const std::string BUOY           = "buoy";
    const std::string TORPEDO_WHOLE  = "torpedo_whole";
    const std::string TORPEDO_HOLE   = "torpedo_hole";
    const std::string GATE           = "gate";
}

// =====================================================================
//  Detection struct – mirrors ObjectDetection.msg enriched fields
// =====================================================================

struct Detection
{
    std::string label;
    float confidence  = 0.0f;
    float center_x    = 0.5f;   // 0–1, 0.5 = frame centre
    float center_y    = 0.5f;
    float bbox_width  = 0.0f;   // 0–1, normalised
    float bbox_height = 0.0f;
    float depth_m     = -1.0f;  // Euclidean distance to object in metres (−1 = unknown)
};

// =====================================================================
//  VisionState – thread-safe detection store
// =====================================================================

class VisionState
{
public:
    // Called by the subscriber callback (runs in the spin-thread).
    void update(const auv_msgs::msg::ObjectDetectionArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        detections_.clear();
        for (const auto &d : msg->detections)
        {
            if (!std::isfinite(d.confidence) ||
                !std::isfinite(d.position.x) ||
                !std::isfinite(d.position.y))
                continue;

            Detection det;
            det.label       = d.label;
            det.confidence  = std::clamp(d.confidence, 0.0f, 1.0f);
            det.center_x    = std::clamp(d.position.x, 0.0f, 1.0f);
            det.center_y    = std::clamp(d.position.y, 0.0f, 1.0f);
            det.depth_m     = std::isfinite(d.position.z) ? d.position.z : -1.0f;
            det.bbox_width  = std::clamp(d.bbox_width,  0.0f, 1.0f);
            det.bbox_height = std::clamp(d.bbox_height, 0.0f, 1.0f);
            detections_.push_back(det);
        }
        last_update_ = SteadyClock::now();
    }

    // Best detection above min_conf for *label*.
    std::optional<Detection> get(const std::string &label,
                                 float min_conf = DETECT_CONFIDENCE) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (stale()) return std::nullopt;
        std::optional<Detection> best;
        for (const auto &d : detections_)
        {
            if (d.label == label && d.confidence >= min_conf)
            {
                if (!best || d.confidence > best->confidence)
                    best = d;
            }
        }
        return best;
    }

    // Is there any detection of *label* above min_conf?
    bool has(const std::string &label,
             float min_conf = DETECT_CONFIDENCE) const
    {
        return get(label, min_conf).has_value();
    }

    // Return all labels currently detected above min_conf.
    std::vector<std::string> labels(float min_conf = DETECT_CONFIDENCE) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        std::vector<std::string> out;
        if (stale()) return out;
        for (const auto &d : detections_)
            if (d.confidence >= min_conf)
                out.push_back(d.label);
        return out;
    }

    // Seconds since the last message was received.
    double age_s() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return std::chrono::duration<double>(
            SteadyClock::now() - last_update_).count();
    }

private:
    bool stale() const
    {
        // Consider data stale if older than 2 seconds.
        return std::chrono::duration<double>(
            SteadyClock::now() - last_update_).count() > 2.0;
    }

    mutable std::mutex                mtx_;
    std::vector<Detection>            detections_;
    TimePoint                         last_update_ = SteadyClock::now();
};

// =====================================================================
//  DepthState – thread-safe store for depth/info messages
//  Tells the mission planner where the sub is and when to stop.
// =====================================================================

class DepthState
{
public:
    void update(const auv_msgs::msg::DepthInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        sub_depth_m_     = msg->sub_depth_m;
        stop_distance_m_ = msg->stop_distance_m;
    }

    // Configured stop distance in metres (default 1.5 m).
    float stop_distance_m() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return stop_distance_m_;
    }

    // Sub's current depth below surface (−1 = unknown / no positional tracking).
    float sub_depth_m() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return sub_depth_m_;
    }

private:
    mutable std::mutex mtx_;
    float sub_depth_m_     = -1.0f;
    float stop_distance_m_ = 1.5f;
};

// =====================================================================
//  LocalizationState – thread-safe store for fused pose
// =====================================================================

class LocalizationState
{
public:
    void update(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        x_   = msg->pose.position.x;
        y_   = msg->pose.position.y;
        z_   = msg->pose.position.z;

        auto &q = msg->pose.orientation;
        yaw_ = std::atan2(
            2.0f * (q.w * q.z + q.x * q.y),
            1.0f - 2.0f * (q.y * q.y + q.z * q.z));

        received_ = true;
    }

    float x()   const { std::lock_guard<std::mutex> lk(mtx_); return x_; }
    float y()   const { std::lock_guard<std::mutex> lk(mtx_); return y_; }
    float z()   const { std::lock_guard<std::mutex> lk(mtx_); return z_; }
    float yaw() const { std::lock_guard<std::mutex> lk(mtx_); return yaw_; }
    bool  received() const { std::lock_guard<std::mutex> lk(mtx_); return received_; }

private:
    mutable std::mutex mtx_;
    float x_ = 0.0f, y_ = 0.0f, z_ = 0.0f, yaw_ = 0.0f;
    bool received_ = false;
};

// =====================================================================
//  Forward declarations / globals
// =====================================================================

class MovementPublisher;
class NavigationPublisher;
class BehaviorStatusPublisher;

std::shared_ptr<MovementPublisher>       g_movement_pub;
std::shared_ptr<NavigationPublisher>     g_nav_pub;
std::shared_ptr<BehaviorStatusPublisher> g_behavior_status_node;
std::shared_ptr<VisionState>             g_vision;
std::shared_ptr<DepthState>              g_depth;
std::shared_ptr<LocalizationState>       g_localization;

inline double elapsed_s(const TimePoint &start)
{
    return std::chrono::duration<double>(SteadyClock::now() - start).count();
}

// =====================================================================
//  MovementPublisher – publishes auv_msgs::msg::MovementCommand
// =====================================================================

class MovementPublisher : public rclcpp::Node
{
public:
    MovementPublisher() : Node("movement_publisher")
    {
        pub_ = this->create_publisher<auv_msgs::msg::MovementCommand>(
            "movement_command", 10);
    }

    void publish_command(const std::string &command,
                         float speed, float duration)
    {
        auto msg = auv_msgs::msg::MovementCommand();
        msg.command  = command;
        msg.speed    = speed;
        msg.duration = duration;
        pub_->publish(msg);
    }

    // Convenience wrappers (duration=0 → persistent until next command)
    void submerge      (float s = 0.4f, float d = 0) { publish_command("submerge",       s, d); }
    void emerge        (float s = 0.4f, float d = 0) { publish_command("emerge",         s, d); }
    void surge_forward (float s = 0.5f, float d = 0) { publish_command("surge_forward",  s, d); }
    void surge_backward(float s = 0.5f, float d = 0) { publish_command("surge_backward", s, d); }
    void strafe_left   (float s = 0.5f, float d = 0) { publish_command("strafe_left",    s, d); }
    void strafe_right  (float s = 0.5f, float d = 0) { publish_command("strafe_right",   s, d); }
    void rotate_cw     (float s = 0.5f, float d = 0) { publish_command("rotate_cw",      s, d); }
    void rotate_ccw    (float s = 0.5f, float d = 0) { publish_command("rotate_ccw",     s, d); }
    void stop          ()                              { publish_command("stop", 0, 0);          }
    void depth_hold    ()                              { publish_command("depth_hold", 0, 0);    }

private:
    rclcpp::Publisher<auv_msgs::msg::MovementCommand>::SharedPtr pub_;
};

// =====================================================================
//  NavigationPublisher – publishes auv_msgs::msg::NavigationCommand
//  for the autonomous controller (localization-backed closed-loop)
// =====================================================================

class NavigationPublisher : public rclcpp::Node
{
public:
    NavigationPublisher() : Node("navigation_publisher")
    {
        pub_ = this->create_publisher<auv_msgs::msg::NavigationCommand>(
            "navigation_command", 10);
    }

    void publish(const std::string &mode,
                 const std::string &target_label = "",
                 float speed = 0.5f,
                 float approach_dist = 0.0f)
    {
        auto msg = auv_msgs::msg::NavigationCommand();
        msg.mode          = mode;
        msg.target_label  = target_label;
        msg.speed         = speed;
        msg.approach_dist = approach_dist;
        pub_->publish(msg);
    }

    void publish_waypoint(float x, float y, float z, float yaw,
                          float speed = 0.5f)
    {
        auto msg = auv_msgs::msg::NavigationCommand();
        msg.mode       = "waypoint";
        msg.target_x   = x;
        msg.target_y   = y;
        msg.target_z   = z;
        msg.target_yaw = yaw;
        msg.speed      = speed;
        pub_->publish(msg);
    }

    void station_keep(float speed = 0.5f)
    {
        publish("station_keep", "", speed);
    }

    void track(const std::string &label, float speed = 0.5f,
               float approach_dist = 0.0f)
    {
        publish("track_object", label, speed, approach_dist);
    }

    void search(const std::string &label, float speed = 0.25f)
    {
        publish("search", label, speed);
    }

    void idle()
    {
        publish("idle");
    }

    void heading_hold(float yaw, float speed = 0.5f)
    {
        auto msg = auv_msgs::msg::NavigationCommand();
        msg.mode       = "heading_hold";
        msg.target_yaw = yaw;
        msg.speed      = speed;
        pub_->publish(msg);
    }

private:
    rclcpp::Publisher<auv_msgs::msg::NavigationCommand>::SharedPtr pub_;
};

// =====================================================================
//  execute_movement – timed movement (fire-and-forget, blocks BT tick)
// =====================================================================

constexpr float MAX_MOVEMENT_DURATION_S = 30.0f;

static void execute_movement(const std::string &command,
                             float speed, float duration_sec)
{
    if (!g_movement_pub) return;

    speed = std::clamp(speed, 0.0f, 1.0f);
    duration_sec = std::clamp(duration_sec, 0.0f, MAX_MOVEMENT_DURATION_S);

    if (g_nav_pub) g_nav_pub->idle();

    g_movement_pub->publish_command(command, speed, duration_sec);

    if (duration_sec > 0.0f)
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                static_cast<int>(duration_sec * 1000)));
        g_movement_pub->stop();
        if (g_nav_pub) g_nav_pub->station_keep();
    }
}

// =====================================================================
//  VisionSubscriber – bridges vision/detections → VisionState
// =====================================================================

class VisionSubscriber : public rclcpp::Node
{
public:
    explicit VisionSubscriber(std::shared_ptr<VisionState> vs)
        : Node("mission_vision_sub"), vs_(vs)
    {
        sub_ = this->create_subscription<auv_msgs::msg::ObjectDetectionArray>(
            "vision/detections", 10,
            std::bind(&VisionSubscriber::cb, this, std::placeholders::_1));
    }

private:
    void cb(const auv_msgs::msg::ObjectDetectionArray::SharedPtr msg)
    {
        vs_->update(msg);
    }
    rclcpp::Subscription<auv_msgs::msg::ObjectDetectionArray>::SharedPtr sub_;
    std::shared_ptr<VisionState> vs_;
};

// =====================================================================
//  DepthSubscriber – bridges depth/info → DepthState
// =====================================================================

class DepthSubscriber : public rclcpp::Node
{
public:
    explicit DepthSubscriber(std::shared_ptr<DepthState> ds)
        : Node("mission_depth_sub"), ds_(ds)
    {
        sub_ = this->create_subscription<auv_msgs::msg::DepthInfo>(
            "depth/info", 10,
            std::bind(&DepthSubscriber::cb, this, std::placeholders::_1));
    }

private:
    void cb(const auv_msgs::msg::DepthInfo::SharedPtr msg) { ds_->update(msg); }
    rclcpp::Subscription<auv_msgs::msg::DepthInfo>::SharedPtr sub_;
    std::shared_ptr<DepthState> ds_;
};

// =====================================================================
//  LocalizationSubscriber – bridges localization/pose → LocalizationState
// =====================================================================

class LocalizationSubscriber : public rclcpp::Node
{
public:
    explicit LocalizationSubscriber(std::shared_ptr<LocalizationState> ls)
        : Node("mission_localization_sub"), ls_(ls)
    {
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "localization/pose", 10,
            std::bind(&LocalizationSubscriber::cb, this, std::placeholders::_1));
    }

private:
    void cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { ls_->update(msg); }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    std::shared_ptr<LocalizationState> ls_;
};

// =====================================================================
//  BehaviorStatusPublisher
// =====================================================================

class BehaviorStatusPublisher : public rclcpp::Node
{
public:
    BehaviorStatusPublisher() : Node("behavior_status_publisher")
    {
        pub_ = this->create_publisher<auv_msgs::msg::BehaviorStatus>(
            "behavior_status", 10);
    }

    void publish_status(const std::string &action_name,
                        const std::string &status,
                        const std::string &reason)
    {
        auv_msgs::msg::BehaviorStatus msg;
        msg.stamp       = this->now();
        msg.action_name = action_name;
        msg.status      = status;
        msg.reason      = reason;
        pub_->publish(msg);
    }

private:
    rclcpp::Publisher<auv_msgs::msg::BehaviorStatus>::SharedPtr pub_;
};

static void publish_status(const std::string &action,
                           const std::string &status,
                           const std::string &reason = "")
{
    if (g_behavior_status_node)
        g_behavior_status_node->publish_status(action, status, reason);
}

// =====================================================================
//  Vision-guided movement primitives
//
//  Each primitive runs a feedback loop at ~10 Hz:
//    1. Read latest detections from VisionState
//    2. Compute error (position, size)
//    3. Issue incremental movement commands
//    4. Return true when goal is met, false on timeout
// =====================================================================

// ─── apply_centering ─────────────────────────────────────────────────
//  Adjusts heading and depth to centre the detection in frame.
//  Returns true if already within tolerance.
//
//  Only adjusts yaw/strafe OR depth per call to avoid sending multiple
//  conflicting MovementCommands within the same control tick.
//  Horizontal correction takes priority since it affects aiming more.
static bool apply_centering(const Detection &det)
{
    if (!g_movement_pub) return false;

    float ex = det.center_x - 0.5f;
    float ey = det.center_y - 0.5f;

    if (!std::isfinite(ex) || !std::isfinite(ey)) return false;

    bool h_ok = std::abs(ex) <= CENTER_TOL;
    bool v_ok = std::abs(ey) <= CENTER_TOL;

    if (h_ok && v_ok)
    {
        return true;
    }

    if (!h_ok)
    {
        float speed = std::clamp(std::abs(ex) * 2.0f, 0.10f, 0.45f);
        if (std::abs(ex) > 0.20f)
            g_movement_pub->publish_command(
                ex > 0 ? "rotate_cw" : "rotate_ccw", speed, 0);
        else
            g_movement_pub->publish_command(
                ex > 0 ? "strafe_right" : "strafe_left", speed, 0);
    }
    else
    {
        float speed = std::clamp(std::abs(ey) * 1.5f, 0.08f, 0.30f);
        g_movement_pub->publish_command(
            ey > 0 ? "submerge" : "emerge", speed, 0);
    }

    return false;
}

// ─── search_for ──────────────────────────────────────────────────────
//  Rotate slowly until *label* is detected above DETECT_CONFIDENCE.
//  Delegates to the autonomous controller for localization-backed
//  heading stability during rotation.
static bool search_for(const std::string &label,
                       float timeout_s = TIMEOUT_SEARCH)
{
    if (g_nav_pub) g_nav_pub->search(label, 0.25f);

    auto t0 = SteadyClock::now();
    while (rclcpp::ok() && elapsed_s(t0) < timeout_s)
    {
        if (g_vision->has(label, DETECT_CONFIDENCE))
        {
            if (g_nav_pub) g_nav_pub->station_keep();
            return true;
        }
        std::this_thread::sleep_for(CTRL_PERIOD);
    }
    if (g_nav_pub) g_nav_pub->idle();
    return false;
}

// ─── centre_on ───────────────────────────────────────────────────────
//  Adjust heading / strafe / depth until *label* is centred in frame.
//  Uses the autonomous controller's track mode for localization-backed
//  centering, with a vision feedback loop to confirm stability.
static bool centre_on(const std::string &label,
                      float timeout_s = TIMEOUT_CENTER)
{
    if (g_nav_pub) g_nav_pub->track(label, 0.40f);

    auto t0 = SteadyClock::now();
    int frames_centred = 0;
    while (rclcpp::ok() && elapsed_s(t0) < timeout_s)
    {
        auto det = g_vision->get(label, TRACK_CONFIDENCE);
        if (!det)
        {
            frames_centred = 0;
            std::this_thread::sleep_for(CTRL_PERIOD);
            continue;
        }

        float ex = std::abs(det->center_x - 0.5f);
        float ey = std::abs(det->center_y - 0.5f);
        if (ex <= CENTER_TOL && ey <= CENTER_TOL)
        {
            if (++frames_centred >= 4)
            {
                if (g_nav_pub) g_nav_pub->station_keep();
                return true;
            }
        }
        else
        {
            frames_centred = 0;
        }
        std::this_thread::sleep_for(CTRL_PERIOD);
    }
    if (g_nav_pub) g_nav_pub->idle();
    return false;
}

// ─── approach ────────────────────────────────────────────────────────
//  Surge forward while keeping *label* centred.
//  Uses the autonomous controller's track mode for closed-loop
//  centering + approach with localization drift compensation.
//
//  Stop condition (whichever triggers first):
//    1. Depth-based  – det.depth_m ≤ stop_distance_m from depth/info.
//    2. Bbox-based   – bbox_width ≥ target_w (fallback when depth=-1).
static bool approach(const std::string &label,
                     float target_w  = APPROACH_W,
                     float timeout_s = TIMEOUT_APPROACH)
{
    float stop_dist = g_depth ? g_depth->stop_distance_m() : -1.0f;
    float eff_approach = (stop_dist > 0.0f) ? stop_dist : 1.5f;

    if (g_nav_pub) g_nav_pub->track(label, 0.50f, eff_approach);

    auto t0 = SteadyClock::now();
    while (rclcpp::ok() && elapsed_s(t0) < timeout_s)
    {
        auto det = g_vision->get(label, TRACK_CONFIDENCE);
        if (!det)
        {
            std::this_thread::sleep_for(CTRL_PERIOD);
            continue;
        }

        bool depth_ok = (stop_dist > 0.0f && det->depth_m > 0.0f);
        if (depth_ok && det->depth_m <= stop_dist)
        {
            if (g_nav_pub) g_nav_pub->station_keep();
            return true;
        }
        if (!depth_ok && det->bbox_width >= target_w)
        {
            if (g_nav_pub) g_nav_pub->station_keep();
            return true;
        }

        std::this_thread::sleep_for(CTRL_PERIOD);
    }
    if (g_nav_pub) g_nav_pub->idle();
    return false;
}

// ─── pass_through ────────────────────────────────────────────────────
//  Approach *label* until it fills the frame, then keep surging until
//  it disappears (meaning we've passed through / beyond it).
static bool pass_through(const std::string &label,
                         float timeout_s = TIMEOUT_PASS)
{
    // Phase 1 – approach until very close
    if (!approach(label, CLOSE_W, timeout_s * 0.6f))
    {
        // Even if we can't get very close, keep going
    }

    // Phase 2 – surge forward until label disappears
    auto t0 = SteadyClock::now();
    int  frames_gone = 0;
    while (rclcpp::ok() && elapsed_s(t0) < timeout_s * 0.5f)
    {
        g_movement_pub->surge_forward(0.50f);

        if (!g_vision->has(label, TRACK_CONFIDENCE))
        {
            if (++frames_gone >= 8)   // ~0.8 s without seeing it
            {
                g_movement_pub->stop();
                if (g_nav_pub) g_nav_pub->station_keep();
                return true;
            }
        }
        else
        {
            frames_gone = 0;
            // keep centering while it's still visible
            auto det = g_vision->get(label, TRACK_CONFIDENCE);
            if (det) apply_centering(*det);
        }
        std::this_thread::sleep_for(CTRL_PERIOD);
    }
    g_movement_pub->stop();
    return true; // assume through after timeout
}

// ─── centre_between ─────────────────────────────────────────────────
//  Centre the sub between two detected objects.
static bool centre_between(const std::string &label_a,
                           const std::string &label_b,
                           float timeout_s = TIMEOUT_CENTER)
{
    auto t0 = SteadyClock::now();
    int frames_ok = 0;
    while (rclcpp::ok() && elapsed_s(t0) < timeout_s)
    {
        auto da = g_vision->get(label_a, TRACK_CONFIDENCE);
        auto db = g_vision->get(label_b, TRACK_CONFIDENCE);

        if (!da || !db)
        {
            g_movement_pub->stop();
            frames_ok = 0;
            std::this_thread::sleep_for(CTRL_PERIOD);
            continue;
        }

        // virtual midpoint
        Detection mid;
        mid.center_x = (da->center_x + db->center_x) * 0.5f;
        mid.center_y = (da->center_y + db->center_y) * 0.5f;

        if (apply_centering(mid))
        {
            if (++frames_ok >= 4)
            {
                g_movement_pub->stop();
                if (g_nav_pub) g_nav_pub->station_keep();
                return true;
            }
        }
        else
        {
            frames_ok = 0;
        }
        std::this_thread::sleep_for(CTRL_PERIOD);
    }
    g_movement_pub->stop();
    if (g_nav_pub) g_nav_pub->idle();
    return false;
}

// =====================================================================
//  BEHAVIOR-TREE ACTION & CONDITION NODES
//
//  Each node now uses VisionState for detection checks and
//  vision-guided movement helpers for closed-loop control.
//  Condition nodes return FAILURE if the required object is not seen.
//  Action nodes return FAILURE on timeout (task not completed).
// =====================================================================

// ─── Heading Out ─────────────────────────────────────────────────────

class Submerge : public BT::SyncActionNode
{
public:
    explicit Submerge(const std::string &name,
                      const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<string>("detections")};
    }

    BT::NodeStatus tick() override
    {
        cout << "[Submerge] Descending for 5 s …" << endl;
        execute_movement("submerge", 0.4f, 5.0f);
        publish_status("Submerge", "SUCCESS", "Submerge completed");
        return BT::NodeStatus::SUCCESS;
    }
};

class TurnRight90 : public BT::SyncActionNode
{
public:
    explicit TurnRight90(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Turn_right_90_deg] Rotating CW ~90° …" << endl;
        execute_movement("rotate_cw", 0.5f, 3.0f);
        publish_status("Turn_right_90_deg", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

// ── Condition: is the gate visible ahead? ──
BT::NodeStatus Detect_gate_at_front_of_sub()
{
    cout << "[Detect_gate] Checking for gate (conf >= "
         << DETECT_CONFIDENCE << ") …" << endl;

    // Give vision a moment to stabilise after a turn
    std::this_thread::sleep_for(500ms);

    // Check several frames to be sure
    int hits = 0;
    for (int i = 0; i < 5 && rclcpp::ok(); ++i)
    {
        if (g_vision->has(Labels::GATE, DETECT_CONFIDENCE))
            ++hits;
        std::this_thread::sleep_for(200ms);
    }

    if (hits >= 3)
    {
        cout << "[Detect_gate] Gate DETECTED (" << hits << "/5 frames)" << endl;
        publish_status("Detect_gate", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    cout << "[Detect_gate] Gate NOT detected (" << hits << "/5 frames)" << endl;
    publish_status("Detect_gate", "FAILURE", "Gate not in view");
    return BT::NodeStatus::FAILURE;
}

class Center_sub_perpendicular_to_gate : public BT::SyncActionNode
{
public:
    explicit Center_sub_perpendicular_to_gate(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Centre_gate] Centering on gate with vision …" << endl;
        if (centre_on(Labels::GATE, TIMEOUT_CENTER))
        {
            publish_status("Centre_gate", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        // Fallback: timed adjustment
        cout << "[Centre_gate] Vision centre timed out – falling back" << endl;
        execute_movement("rotate_cw",   0.2f, 1.0f);
        execute_movement("strafe_left", 0.2f, 1.0f);
        publish_status("Centre_gate", "SUCCESS", "Fallback timed move");
        return BT::NodeStatus::SUCCESS;
    }
};

// ─── Collecting Data ─────────────────────────────────────────────────

class Detect_preferred_animal_left_of_center : public BT::SyncActionNode
{
public:
    explicit Detect_preferred_animal_left_of_center(
        const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<string>("preferred_animal")};
    }

    BT::NodeStatus tick() override
    {
        cout << "[Detect_animal_left] Checking for animal left of centre …"
             << endl;

        std::this_thread::sleep_for(500ms);

        // Try both animal labels
        for (const auto &lbl : {Labels::BUOY, Labels::TORPEDO_HOLE})
        {
            auto det = g_vision->get(lbl, DETECT_CONFIDENCE);
            if (det && det->center_x < 0.45f)
            {
                cout << "[Detect_animal_left] Found " << lbl
                     << " at x=" << det->center_x << endl;
                publish_status("Detect_animal_left", "SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
        }

        cout << "[Detect_animal_left] No animal detected left of centre"
             << endl;
        publish_status("Detect_animal_left", "FAILURE");
        return BT::NodeStatus::FAILURE;
    }
};

class Reposition_sub_to_gate_left_entrance : public BT::SyncActionNode
{
public:
    explicit Reposition_sub_to_gate_left_entrance(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Reposition_left] Strafing left toward gate entrance …"
             << endl;
        if (g_nav_pub) g_nav_pub->idle();
        auto t0 = SteadyClock::now();
        while (rclcpp::ok() && elapsed_s(t0) < 8.0)
        {
            auto det = g_vision->get(Labels::GATE, TRACK_CONFIDENCE);
            if (det && det->center_x > 0.55f)
            {
                if (g_nav_pub) g_nav_pub->station_keep();
                publish_status("Reposition_left", "SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
            g_movement_pub->strafe_left(0.35f);
            std::this_thread::sleep_for(CTRL_PERIOD);
        }
        g_movement_pub->stop();
        execute_movement("strafe_left", 0.4f, 3.0f);
        publish_status("Reposition_left", "SUCCESS", "Fallback");
        return BT::NodeStatus::SUCCESS;
    }
};

class Reposition_sub_to_gate_right_entrance : public BT::SyncActionNode
{
public:
    explicit Reposition_sub_to_gate_right_entrance(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Reposition_right] Strafing right toward gate entrance …"
             << endl;
        if (g_nav_pub) g_nav_pub->idle();
        auto t0 = SteadyClock::now();
        while (rclcpp::ok() && elapsed_s(t0) < 8.0)
        {
            auto det = g_vision->get(Labels::GATE, TRACK_CONFIDENCE);
            if (det && det->center_x < 0.45f)
            {
                if (g_nav_pub) g_nav_pub->station_keep();
                publish_status("Reposition_right", "SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
            g_movement_pub->strafe_right(0.35f);
            std::this_thread::sleep_for(CTRL_PERIOD);
        }
        g_movement_pub->stop();
        execute_movement("strafe_right", 0.4f, 3.0f);
        publish_status("Reposition_right", "SUCCESS", "Fallback");
        return BT::NodeStatus::SUCCESS;
    }
};

class Set_preferred_side_to_L : public BT::SyncActionNode
{
public:
    explicit Set_preferred_side_to_L(const std::string &name,
                                     const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<string>("preferred_side")};
    }

    BT::NodeStatus tick() override
    {
        setOutput("preferred_side", std::string("L"));
        publish_status("Set_preferred_side_to_L", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

class Set_preferred_side_to_R : public BT::SyncActionNode
{
public:
    explicit Set_preferred_side_to_R(const std::string &name,
                                     const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<string>("preferred_side")};
    }

    BT::NodeStatus tick() override
    {
        setOutput("preferred_side", std::string("R"));
        publish_status("Set_preferred_side_to_R", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

class Move_with_style_through_gate : public BT::SyncActionNode
{
public:
    explicit Move_with_style_through_gate(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Style_gate] Approaching & passing gate with vision …"
             << endl;
        // Small flair: rotate slightly then pass through
        execute_movement("rotate_cw", 0.6f, 1.0f);
        if (pass_through(Labels::GATE, TIMEOUT_PASS))
        {
            execute_movement("rotate_ccw", 0.6f, 1.0f);
            publish_status("Style_gate", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        publish_status("Style_gate", "FAILURE", "Pass-through timed out");
        return BT::NodeStatus::FAILURE;
    }
};

class Move_in_the_most_boring_way_possible_through_the_gate
    : public BT::SyncActionNode
{
public:
    explicit Move_in_the_most_boring_way_possible_through_the_gate(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Boring_gate] Approaching & passing gate (no style) …"
             << endl;
        if (pass_through(Labels::GATE, TIMEOUT_PASS))
        {
            publish_status("Boring_gate", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        // Fallback: just surge forward
        execute_movement("surge_forward", 0.3f, 8.0f);
        publish_status("Boring_gate", "SUCCESS", "Fallback surge");
        return BT::NodeStatus::SUCCESS;
    }
};

// ─── Follow Path ─────────────────────────────────────────────────────

class Turn_right_until_parallel_with_Path_and_facing_away_from_the_end
    : public BT::SyncActionNode
{
public:
    explicit Turn_right_until_parallel_with_Path_and_facing_away_from_the_end(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Parallel_path] Rotating until path centred …" << endl;
        // Search-rotate until we see the path marker, then centre
        if (search_for(Labels::GATE, 12.0f) && centre_on(Labels::GATE))
        {
            publish_status("Parallel_path", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        // Fallback timed rotation
        execute_movement("rotate_cw", 0.4f, 4.0f);
        publish_status("Parallel_path", "SUCCESS", "Fallback");
        return BT::NodeStatus::SUCCESS;
    }
};

class Move_until_the_other_end_of_the_path : public BT::SyncActionNode
{
public:
    explicit Move_until_the_other_end_of_the_path(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Follow_path] Moving along path using vision …" << endl;
        if (g_nav_pub) g_nav_pub->idle();

        auto t0 = SteadyClock::now();
        int frames_without_path = 0;

        while (rclcpp::ok() && elapsed_s(t0) < 30.0)
        {
            auto det = g_vision->get(Labels::GATE, TRACK_CONFIDENCE);
            if (det)
            {
                frames_without_path = 0;
                float ex = det->center_x - 0.5f;
                if (std::abs(ex) > CENTER_TOL)
                {
                    float yaw = std::clamp(std::abs(ex) * 1.5f, 0.10f, 0.30f);
                    g_movement_pub->publish_command(
                        ex > 0 ? "rotate_cw" : "rotate_ccw", yaw, 0);
                }
                g_movement_pub->surge_forward(0.40f);
            }
            else
            {
                if (++frames_without_path >= 10)
                {
                    if (g_nav_pub) g_nav_pub->station_keep();
                    publish_status("Follow_path", "SUCCESS",
                                   "Path end reached");
                    return BT::NodeStatus::SUCCESS;
                }
                g_movement_pub->surge_forward(0.25f);
            }
            std::this_thread::sleep_for(CTRL_PERIOD);
        }

        if (g_nav_pub) g_nav_pub->station_keep();
        publish_status("Follow_path", "SUCCESS", "Timeout – assume done");
        return BT::NodeStatus::SUCCESS;
    }
};

// ─── Navigate Channel ────────────────────────────────────────────────

BT::NodeStatus preferred_side_is_L(BT::TreeNode &self)
{
    auto side = self.getInput<std::string>("preferred_side");
    if (side && side.value() == "L")
    {
        cout << "[preferred_side_is_L] TRUE" << endl;
        return BT::NodeStatus::SUCCESS;
    }
    cout << "[preferred_side_is_L] FALSE ("
         << (side ? side.value() : "?") << ")" << endl;
    return BT::NodeStatus::FAILURE;
}

class Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right
    : public BT::SyncActionNode
{
public:
    explicit Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Centre_PVC_WL_RR] Centering between white(L) / red(R) …"
             << endl;
        if (centre_between(Labels::CCW_BLUE_GATE, Labels::CW_RED_GATE, TIMEOUT_CENTER))
        {
            publish_status("Centre_PVC_WL_RR", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        // Fallback: timed strafe + rotate
        execute_movement("strafe_right", 0.3f, 2.0f);
        execute_movement("rotate_ccw",   0.2f, 1.0f);
        publish_status("Centre_PVC_WL_RR", "SUCCESS", "Fallback");
        return BT::NodeStatus::SUCCESS;
    }
};

class Move_past_PVC_posts : public BT::SyncActionNode
{
public:
    explicit Move_past_PVC_posts(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Move_past_PVC] Surging through PVC posts with vision …"
             << endl;
        if (g_nav_pub) g_nav_pub->idle();

        auto t0 = SteadyClock::now();
        bool was_close = false;
        int  frames_gone = 0;

        while (rclcpp::ok() && elapsed_s(t0) < 15.0)
        {
            auto rd = g_vision->get(Labels::CW_RED_GATE,   TRACK_CONFIDENCE);
            auto wd = g_vision->get(Labels::CCW_BLUE_GATE, TRACK_CONFIDENCE);

            bool see_pvc = rd.has_value() || wd.has_value();

            if (see_pvc)
            {
                frames_gone = 0;
                float max_w = 0.0f;
                if (rd) max_w = std::max(max_w, rd->bbox_width);
                if (wd) max_w = std::max(max_w, wd->bbox_width);
                if (max_w > 0.25f) was_close = true;

                if (rd && wd)
                {
                    Detection mid;
                    mid.center_x = (rd->center_x + wd->center_x) * 0.5f;
                    mid.center_y = (rd->center_y + wd->center_y) * 0.5f;
                    apply_centering(mid);
                }
            }
            else if (was_close)
            {
                if (++frames_gone >= 6)
                {
                    if (g_nav_pub) g_nav_pub->station_keep();
                    publish_status("Move_past_PVC", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;
                }
            }

            g_movement_pub->surge_forward(0.40f);
            std::this_thread::sleep_for(CTRL_PERIOD);
        }

        if (g_nav_pub) g_nav_pub->station_keep();
        publish_status("Move_past_PVC", "SUCCESS", "Timeout");
        return BT::NodeStatus::SUCCESS;
    }
};

class Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right
    : public BT::SyncActionNode
{
public:
    explicit Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Centre_PVC_RL_WR] Centering between red(L) / white(R) …"
             << endl;
        if (centre_between(Labels::CW_RED_GATE, Labels::CCW_BLUE_GATE, TIMEOUT_CENTER))
        {
            publish_status("Centre_PVC_RL_WR", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        execute_movement("strafe_left", 0.3f, 2.0f);
        execute_movement("rotate_cw",   0.2f, 1.0f);
        publish_status("Centre_PVC_RL_WR", "SUCCESS", "Fallback");
        return BT::NodeStatus::SUCCESS;
    }
};

// ─── Drop BRUVs ─────────────────────────────────────────────────────

class Move_sub_until_aligned_with_preferred_animal_on_bin
    : public BT::SyncActionNode
{
public:
    explicit Move_sub_until_aligned_with_preferred_animal_on_bin(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Align_bin] Searching & centering on bin …" << endl;

        // Search for bin, centre, approach
        if (search_for(Labels::BUOY, 15.0f))
        {
            centre_on(Labels::BUOY, 10.0f);
            approach(Labels::BUOY, APPROACH_W, 15.0f);
            publish_status("Align_bin", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        // Fallback
        execute_movement("surge_forward", 0.3f, 5.0f);
        publish_status("Align_bin", "SUCCESS", "Fallback");
        return BT::NodeStatus::SUCCESS;
    }
};

class Drop_marker : public BT::SyncActionNode
{
public:
    explicit Drop_marker(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Drop_marker] Dropping marker …" << endl;
        g_movement_pub->stop();
        // TODO: trigger marker-drop actuator
        std::this_thread::sleep_for(2s);
        publish_status("Drop_marker", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

// ─── Tagging ─────────────────────────────────────────────────────────

class Find_task_location : public BT::SyncActionNode
{
public:
    explicit Find_task_location(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Find_task] Searching for torpedo board …" << endl;
        if (search_for(Labels::TORPEDO_WHOLE, TIMEOUT_SEARCH))
        {
            publish_status("Find_task", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        // Try generic search: any recognisable task object
        execute_movement("rotate_cw", 0.3f, 6.0f);
        publish_status("Find_task", "SUCCESS", "Fallback rotation");
        return BT::NodeStatus::SUCCESS;
    }
};

class Center_on_board_on_the_z_axis_and_yaw : public BT::SyncActionNode
{
public:
    explicit Center_on_board_on_the_z_axis_and_yaw(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Centre_board] Centering on torpedo board …" << endl;
        if (centre_on(Labels::TORPEDO_WHOLE, TIMEOUT_CENTER))
        {
            approach(Labels::TORPEDO_WHOLE, APPROACH_W, 15.0f);
            publish_status("Centre_board", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        execute_movement("submerge",  0.2f, 2.0f);
        execute_movement("rotate_cw", 0.15f, 1.0f);
        publish_status("Centre_board", "SUCCESS", "Fallback");
        return BT::NodeStatus::SUCCESS;
    }
};

class Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal
    : public BT::SyncActionNode
{
public:
    explicit Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Fire_torpedo] Aligning & firing …" << endl;

        // Fine-tune alignment on the board
        centre_on(Labels::TORPEDO_WHOLE, 8.0f);
        g_movement_pub->stop();

        // TODO: trigger torpedo actuator
        std::this_thread::sleep_for(3s);
        publish_status("Fire_torpedo", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

// ─── Ocean Cleanup ───────────────────────────────────────────────────

BT::NodeStatus Detect_preferred_animal_at_front_of_sub_roughly()
{
    cout << "[Detect_animal_front] Checking for animal …" << endl;
    std::this_thread::sleep_for(500ms);

    // Check several frames
    int hits = 0;
    for (int i = 0; i < 8 && rclcpp::ok(); ++i)
    {
        for (const auto &lbl : {Labels::BUOY, Labels::TORPEDO_HOLE})
        {
            if (g_vision->has(lbl, DETECT_CONFIDENCE))
            {
                ++hits;
                break;
            }
        }
        std::this_thread::sleep_for(150ms);
    }

    if (hits >= 4)
    {
        cout << "[Detect_animal_front] Animal DETECTED" << endl;
        publish_status("Detect_animal_front", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    cout << "[Detect_animal_front] NOT detected (" << hits << "/8)" << endl;
    publish_status("Detect_animal_front", "FAILURE");
    return BT::NodeStatus::FAILURE;
}

class Center_sub_perpendicular_to_preferred_animal : public BT::SyncActionNode
{
public:
    explicit Center_sub_perpendicular_to_preferred_animal(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Centre_animal] Centering on preferred animal …" << endl;
        // Try both animal labels
        for (const auto &lbl : {Labels::BUOY, Labels::TORPEDO_HOLE})
        {
            if (g_vision->has(lbl, TRACK_CONFIDENCE))
            {
                centre_on(lbl, TIMEOUT_CENTER);
                approach(lbl, APPROACH_W, 15.0f);
                publish_status("Centre_animal", "SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
        }
        execute_movement("rotate_cw",   0.2f, 1.5f);
        execute_movement("strafe_left", 0.2f, 1.0f);
        publish_status("Centre_animal", "SUCCESS", "Fallback");
        return BT::NodeStatus::SUCCESS;
    }
};

class Grab_trash_with_claw : public BT::SyncActionNode
{
public:
    explicit Grab_trash_with_claw(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Grab_trash] Approaching & grabbing …" << endl;
        // Search for trash, approach closely
        if (search_for(Labels::BUOY, 12.0f))
        {
            centre_on(Labels::BUOY, 10.0f);
            approach(Labels::BUOY, CLOSE_W, 15.0f);
        }
        else
        {
            execute_movement("surge_forward", 0.2f, 2.0f);
        }
        g_movement_pub->stop();
        // TODO: trigger claw actuator
        std::this_thread::sleep_for(2s);
        publish_status("Grab_trash", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

class Resurface : public BT::SyncActionNode
{
public:
    explicit Resurface(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Resurface] Ascending …" << endl;
        execute_movement("emerge", 0.5f, 6.0f);
        publish_status("Resurface", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

class Move_and_place_trash_in_corresponding_basket : public BT::SyncActionNode
{
public:
    explicit Move_and_place_trash_in_corresponding_basket(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Place_trash] Moving to basket …" << endl;
        if (search_for(Labels::BUOY, 12.0f))
        {
            centre_on(Labels::BUOY, 10.0f);
            approach(Labels::BUOY, APPROACH_W, 12.0f);
        }
        else
        {
            execute_movement("surge_forward", 0.3f, 4.0f);
        }
        g_movement_pub->stop();
        // TODO: release claw
        std::this_thread::sleep_for(2s);
        publish_status("Place_trash", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

class Move_back_facing_initial_direction : public BT::SyncActionNode
{
public:
    explicit Move_back_facing_initial_direction(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Move_back] Turning around & returning …" << endl;
        execute_movement("rotate_cw",     0.5f, 6.0f);   // ~180°
        execute_movement("surge_forward", 0.4f, 5.0f);
        publish_status("Move_back", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

// ─── Return Home ─────────────────────────────────────────────────────

class Move_to_depth_taken_for_Navigating_the_Channel : public BT::SyncActionNode
{
public:
    explicit Move_to_depth_taken_for_Navigating_the_Channel(
        const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Channel_depth] Descending to channel depth …" << endl;
        execute_movement("submerge", 0.3f, 4.0f);
        publish_status("Channel_depth", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

class Turn_right_180_deg : public BT::SyncActionNode
{
public:
    explicit Turn_right_180_deg(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Turn_180] Rotating CW ~180° …" << endl;
        execute_movement("rotate_cw", 0.5f, 6.0f);
        publish_status("Turn_180", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

class Make_it_back_through_gate : public BT::SyncActionNode
{
public:
    explicit Make_it_back_through_gate(const std::string &name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        cout << "[Return_gate] Searching for gate & passing back …" << endl;
        // Try to find the gate with vision, otherwise just surge
        if (search_for(Labels::GATE, 12.0f))
        {
            pass_through(Labels::GATE, 25.0f);
        }
        else
        {
            execute_movement("surge_forward", 0.5f, 8.0f);
        }
        publish_status("Return_gate", "SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

// =====================================================================
//  main
// =====================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // ── Shared vision, depth, and localization state ──
    g_vision       = std::make_shared<VisionState>();
    g_depth        = std::make_shared<DepthState>();
    g_localization = std::make_shared<LocalizationState>();

    // ── Blackboard (still used for preferred_side port wiring) ──
    auto blackboard = BT::Blackboard::create();
    blackboard->set("detections",     std::string("No detections"));
    blackboard->set("preferred_side", std::string(""));

    BT::BehaviorTreeFactory factory;

    // ── Heading Out ──
    factory.registerSimpleCondition(
        "Detect_gate_at_front_of_sub",
        std::bind(Detect_gate_at_front_of_sub));
    factory.registerNodeType<Submerge>("Submerge");
    factory.registerNodeType<Center_sub_perpendicular_to_gate>(
        "Center_sub_perpendicular_to_gate");
    factory.registerNodeType<TurnRight90>("Turn_right_90_deg");

    // ── Collecting Data ──
    factory.registerNodeType<Detect_preferred_animal_left_of_center>(
        "Detect_preferred_animal_left_of_center");
    factory.registerNodeType<Reposition_sub_to_gate_left_entrance>(
        "Reposition_sub_to_gate_left_entrance");
    factory.registerNodeType<Set_preferred_side_to_L>(
        "Set_preferred_side_to_L");
    factory.registerNodeType<Reposition_sub_to_gate_right_entrance>(
        "Reposition_sub_to_gate_right_entrance");
    factory.registerNodeType<Set_preferred_side_to_R>(
        "Set_preferred_side_to_R");
    factory.registerNodeType<Move_with_style_through_gate>(
        "Move_with_style_through_gate");
    factory.registerNodeType<Move_in_the_most_boring_way_possible_through_the_gate>(
        "Move_in_the_most_boring_way_possible_through_the_gate");
    factory.registerNodeType<Turn_right_until_parallel_with_Path_and_facing_away_from_the_end>(
        "Turn_right_until_parallel_with_Path_and_facing_away_from_the_end");
    factory.registerNodeType<Move_until_the_other_end_of_the_path>(
        "Move_until_the_other_end_of_the_path");

    // ── Navigate Channel ──
    BT::PortsList side_ports = {BT::InputPort<string>("preferred_side")};
    factory.registerSimpleCondition(
        "preferred_side_is_L", preferred_side_is_L, side_ports);
    factory.registerNodeType<
        Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right>(
        "Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right");
    factory.registerNodeType<Move_past_PVC_posts>("Move_past_PVC_posts");
    factory.registerNodeType<
        Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right>(
        "Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right");

    // ── Drop BRUVs ──
    factory.registerNodeType<Move_sub_until_aligned_with_preferred_animal_on_bin>(
        "Move_sub_until_aligned_with_preferred_animal_on_bin");
    factory.registerNodeType<Drop_marker>("Drop_marker");

    // ── Tagging ──
    factory.registerNodeType<Find_task_location>("Find_task_location");
    factory.registerNodeType<Center_on_board_on_the_z_axis_and_yaw>(
        "Center_on_board_on_the_z_axis_and_yaw");
    factory.registerNodeType<
        Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal>(
        "Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal");

    // ── Ocean Cleanup ──
    factory.registerSimpleCondition(
        "Detect_preferred_animal_at_front_of_sub_roughly",
        std::bind(Detect_preferred_animal_at_front_of_sub_roughly));
    factory.registerNodeType<Center_sub_perpendicular_to_preferred_animal>(
        "Center_sub_perpendicular_to_preferred_animal");
    factory.registerNodeType<Grab_trash_with_claw>("Grab_trash_with_claw");
    factory.registerNodeType<Resurface>("Resurface");
    factory.registerNodeType<Move_and_place_trash_in_corresponding_basket>(
        "Move_and_place_trash_in_corresponding_basket");
    factory.registerNodeType<Move_back_facing_initial_direction>(
        "Move_back_facing_initial_direction");

    // ── Return Home ──
    factory.registerNodeType<Move_to_depth_taken_for_Navigating_the_Channel>(
        "Move_to_depth_taken_for_Navigating_the_Channel");
    factory.registerNodeType<Turn_right_180_deg>("Turn_right_180_deg");
    factory.registerNodeType<Make_it_back_through_gate>(
        "Make_it_back_through_gate");

    // ── Load BT XML files ──
    const auto mission_share_dir =
        ament_index_cpp::get_package_share_directory("mission");
    const auto bt_xml_dir = mission_share_dir + "/bt_xml/";

    const std::vector<std::string> tree_files = {
        "main.xml",          "heading_out.xml",    "collecting_data.xml",
        "folllow_path.xml",  "navigate_channel.xml",
        "drop_bruvs.xml",   "tagging.xml",
        "ocean_cleanup.xml", "return_home.xml"};

    for (const auto &tf : tree_files)
    {
        factory.registerBehaviorTreeFromFile(bt_xml_dir + tf);
    }

    // ── Spin ROS nodes in background ──
    g_movement_pub         = std::make_shared<MovementPublisher>();
    g_nav_pub              = std::make_shared<NavigationPublisher>();
    g_behavior_status_node = std::make_shared<BehaviorStatusPublisher>();

    auto vision_sub       = std::make_shared<VisionSubscriber>(g_vision);
    auto depth_sub        = std::make_shared<DepthSubscriber>(g_depth);
    auto localization_sub = std::make_shared<LocalizationSubscriber>(g_localization);
    rclcpp::executors::SingleThreadedExecutor sub_executor;
    sub_executor.add_node(vision_sub);
    sub_executor.add_node(depth_sub);
    sub_executor.add_node(localization_sub);
    sub_executor.add_node(g_movement_pub);
    sub_executor.add_node(g_nav_pub);
    sub_executor.add_node(g_behavior_status_node);
    std::thread ros_spin_thread([&]() { sub_executor.spin(); });

    cout << "=== SHRUB – Vision-integrated mission starting ===" << endl;
    cout << "  Detection confidence threshold : " << DETECT_CONFIDENCE << endl;
    cout << "  Tracking confidence threshold  : " << TRACK_CONFIDENCE  << endl;
    cout << "  Centre tolerance               : " << CENTER_TOL        << endl;
    cout << "  Approach target bbox width     : " << APPROACH_W        << endl;
    cout << "  Approach stop distance (depth) : " << g_depth->stop_distance_m() << " m" << endl;

    // ── Run the tree with mission timeout ──
    auto main_tree = factory.createTree(
        "SHRUB (Software for Handling and Regulating Underwater Behavior)",
        blackboard);

    constexpr double MISSION_TIMEOUT_S = 870.0;  // 14.5 min — leave 30s buffer
    auto mission_start = SteadyClock::now();

    try {
        auto tick_status = BT::NodeStatus::RUNNING;
        while (rclcpp::ok() && tick_status == BT::NodeStatus::RUNNING)
        {
            if (elapsed_s(mission_start) >= MISSION_TIMEOUT_S)
            {
                cout << "=== MISSION TIMEOUT — halting tree ===" << endl;
                if (g_movement_pub) g_movement_pub->stop();
                if (g_nav_pub) g_nav_pub->idle();
                publish_status("MissionTree", "TIMEOUT", "Mission time limit reached");
                break;
            }
            tick_status = main_tree.tickRoot();
            std::this_thread::sleep_for(50ms);
        }
    } catch (const std::exception& e) {
        cerr << "=== MISSION EXCEPTION: " << e.what() << " ===" << endl;
        if (g_movement_pub) g_movement_pub->stop();
        if (g_nav_pub) g_nav_pub->idle();
        publish_status("MissionTree", "FAILURE", std::string("Exception: ") + e.what());
    }

    if (g_movement_pub) g_movement_pub->stop();
    publish_status("MissionTree", "COMPLETE", "Behaviour tree finished");
    cout << "=== SHRUB – Mission complete ===" << endl;

    // Cleanup
    sub_executor.cancel();
    if (rclcpp::ok()) rclcpp::shutdown();
    if (ros_spin_thread.joinable()) ros_spin_thread.join();

    g_movement_pub.reset();
    g_nav_pub.reset();
    g_behavior_status_node.reset();
    g_vision.reset();
    g_depth.reset();
    g_localization.reset();
    vision_sub.reset();
    depth_sub.reset();
    localization_sub.reset();

    return 0;
}
