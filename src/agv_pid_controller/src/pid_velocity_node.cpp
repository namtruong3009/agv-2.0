#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "control_toolbox/pid.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double CMD_TIMEOUT_S = 0.5;
constexpr double ODOM_TIMEOUT_S = 0.5;
constexpr double TF_TIMEOUT_S = 1.0;

inline double deg2rad(double angle) { return angle * PI / 180.0; }
inline double rad2deg(double angle) { return angle * 180.0 / PI; }
inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

control_toolbox::AntiWindupStrategy makeAW()
{
  control_toolbox::AntiWindupStrategy aw;
  aw.type = control_toolbox::AntiWindupStrategy::CONDITIONAL_INTEGRATION;
  return aw;
}
}  // namespace

class PidVelocityNode : public rclcpp::Node
{
public:
  PidVelocityNode() : Node("pid_velocity_node")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    max_v_ = declare_parameter<double>("max_linear", 0.25);
    max_w_ = declare_parameter<double>("max_angular", 0.6);

    const double kp_v = declare_parameter<double>("kp_linear", 0.4);
    const double ki_v = declare_parameter<double>("ki_linear", 0.0);
    const double kd_v = declare_parameter<double>("kd_linear", 0.0);

    const double kp_h = declare_parameter<double>("kp_heading", 1.5);
    const double ki_h = declare_parameter<double>("ki_heading", 0.0);
    const double kd_h = declare_parameter<double>("kd_heading", 0.0);

    if (!std::isfinite(max_v_) || !std::isfinite(max_w_) ||
        max_v_ <= 0.0 || max_w_ <= 0.0 ||
        !std::isfinite(kp_v) || !std::isfinite(ki_v) || !std::isfinite(kd_v) ||
        !std::isfinite(kp_h) || !std::isfinite(ki_h) || !std::isfinite(kd_h)) {
      throw std::runtime_error("Invalid PID gains or speed limits");
    }

    const auto aw = makeAW();
    pid_speed_.initialize(kp_v, ki_v, kd_v, max_v_, -max_v_, aw);
    pid_heading_.initialize(kp_h, ki_h, kd_h, max_w_, -max_w_, aw);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_pid", 10);
    actual_heading_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/heading_actual_deg", 10);
    heading_error_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/heading_error_deg", 10);

    // Speed: optional. If absent or timed out, requested linear speed = 0.
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/target_speed_mps", 10,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
          return;
        }
        target_v_ = std::clamp(msg->data, -max_v_, max_v_);
        last_speed_ = std::chrono::steady_clock::now();
        has_speed_ = true;
      });

    // Absolute: heading in MAP frame, degrees [-180, 180].
    absolute_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/target_heading_absolute_deg", 10,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!validAngle(msg->data)) {
          RCLCPP_WARN(get_logger(), "Absolute heading must be in [-180, 180] deg");
          return;
        }
        heading_frame_ = "map";
        target_heading_ = normalizeAngle(deg2rad(msg->data));
        has_heading_ = true;
        rotate_first_ = true;
        pid_heading_.reset();
        RCLCPP_INFO(get_logger(), "ABSOLUTE /map target: %.1f deg", msg->data);
      });

    // Relative: add the delta ONCE to the current heading in ODOM.
    relative_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/target_heading_relative_deg", 10,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!validAngle(msg->data)) {
          RCLCPP_WARN(get_logger(), "Relative heading must be in [-180, 180] deg");
          return;
        }
        double current_yaw = 0.0;
        if (!getYaw("odom", current_yaw)) {
          RCLCPP_WARN(get_logger(), "Relative heading rejected: odom TF unavailable");
          return;
        }
        heading_frame_ = "odom";
        target_heading_ = normalizeAngle(current_yaw + deg2rad(msg->data));
        has_heading_ = true;
        rotate_first_ = true;
        pid_heading_.reset();
        RCLCPP_INFO(get_logger(), "RELATIVE %+.1f deg -> odom target %.1f deg",
          msg->data, rad2deg(target_heading_));
      });

    // Optional: clear the latched heading target (not a hardware E-stop).
    cancel_heading_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/cancel_heading", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        has_heading_ = false;
        rotate_first_ = false;
        pid_heading_.reset();
        RCLCPP_INFO(get_logger(), "Heading target canceled");
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        const double v = msg->twist.twist.linear.x;
        const double w = msg->twist.twist.angular.z;
        if (!std::isfinite(v) || !std::isfinite(w)) {
          return;
        }
        actual_v_ = v;
        actual_w_ = w;
        last_odom_ = std::chrono::steady_clock::now();
        has_odom_ = true;
      });

    last_update_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(20ms, std::bind(&PidVelocityNode::update, this));
    RCLCPP_INFO(get_logger(),
      "Speed / Heading PID ready: speed-only, heading-only, or both");
  }

private:
  static bool validAngle(double deg)
  {
    return std::isfinite(deg) && deg >= -180.0 && deg <= 180.0;
  }

  bool getYaw(const std::string & frame, double & yaw)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        frame, base_frame_, tf2::TimePointZero);

      // Run with use_sim_time:=true in Gazebo, so both clocks use /clock.
      const rclcpp::Time stamp(tf.header.stamp, get_clock()->get_clock_type());
      const double age = (get_clock()->now() - stamp).seconds();
      if (age > TF_TIMEOUT_S || age < -0.2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF %s -> %s stale or future dated (age %.2f s). Check use_sim_time.",
          frame.c_str(), base_frame_.c_str(), age);
        return false;
      }

      const auto & q = tf.transform.rotation;
      const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
      if (!std::isfinite(norm) || norm < 1e-9) {
        return false;
      }
      const double x = q.x / norm;
      const double y = q.y / norm;
      const double z = q.z / norm;
      const double w = q.w / norm;
      yaw = std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
      return std::isfinite(yaw);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF error (%s -> %s): %s", frame.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void publishStop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  void update()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_update_).count();
    last_update_ = now;

    // Only odometry and a healthy control-loop interval are mandatory.
    const bool odom_fresh = has_odom_ &&
      std::chrono::duration<double>(now - last_odom_).count() < ODOM_TIMEOUT_S;
    if (!odom_fresh || dt <= 0.0 || dt >= 0.2) {
      pid_speed_.reset();
      pid_heading_.reset();
      publishStop();
      return;
    }

    const bool speed_active = has_speed_ &&
      std::chrono::duration<double>(now - last_speed_).count() < CMD_TIMEOUT_S;

    geometry_msgs::msg::Twist output;
    double heading_error = 0.0;

    // Heading works independently of speed commands.
    if (has_heading_) {
      double current_yaw = 0.0;
      if (!getYaw(heading_frame_, current_yaw)) {
        pid_speed_.reset();
        pid_heading_.reset();
        publishStop();
        return;
      }

      heading_error = normalizeAngle(target_heading_ - current_yaw);
      std_msgs::msg::Float64 actual_msg;
      actual_msg.data = rad2deg(current_yaw);
      actual_heading_pub_->publish(actual_msg);
      std_msgs::msg::Float64 error_msg;
      error_msg.data = rad2deg(heading_error);
      heading_error_pub_->publish(error_msg);

      if (std::abs(heading_error) > deg2rad(2.0)) {
        output.angular.z = std::clamp(
          pid_heading_.compute_command(heading_error, -actual_w_, dt),
          -max_w_, max_w_);
      } else {
        pid_heading_.reset();
      }

      // Rotate-first hysteresis (15 deg to enable, 10 deg to release).
      if (std::abs(heading_error) > deg2rad(15.0)) {
        rotate_first_ = true;
      } else if (std::abs(heading_error) < deg2rad(10.0)) {
        rotate_first_ = false;
      }
    } else {
      // Speed-only: go straight (angular command = 0). No heading hold.
      pid_heading_.reset();
      rotate_first_ = false;
    }

    // Speed is OPTIONAL. Expiry only stops forward/backward movement;
    // it must not block a previously received heading command.
    if (speed_active && !rotate_first_ && std::abs(target_v_) > 1e-4) {
      const double error_v = target_v_ - actual_v_;
      const double correction = pid_speed_.compute_command(error_v, dt);
      output.linear.x = std::clamp(target_v_ + correction, -max_v_, max_v_);
    } else {
      pid_speed_.reset();
      output.linear.x = 0.0;
    }

    cmd_pub_->publish(output);
  }

  control_toolbox::Pid pid_speed_{
    0.0, 0.0, 0.0, 0.25, -0.25, makeAW()};
  control_toolbox::Pid pid_heading_{
    0.0, 0.0, 0.0, 0.6, -0.6, makeAW()};

  std::string base_frame_ = "base_link";
  std::string heading_frame_ = "map";
  double max_v_ = 0.25;
  double max_w_ = 0.6;
  double target_v_ = 0.0;
  double target_heading_ = 0.0;
  double actual_v_ = 0.0;
  double actual_w_ = 0.0;
  bool has_speed_ = false;
  bool has_heading_ = false;
  bool has_odom_ = false;
  bool rotate_first_ = false;

  std::chrono::steady_clock::time_point last_speed_;
  std::chrono::steady_clock::time_point last_odom_;
  std::chrono::steady_clock::time_point last_update_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    speed_sub_, absolute_sub_, relative_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_heading_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    actual_heading_pub_, heading_error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidVelocityNode>());
  rclcpp::shutdown();
  return 0;
}