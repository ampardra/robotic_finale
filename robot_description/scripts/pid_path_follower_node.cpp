#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>

class PidPathFollower : public rclcpp::Node
{
public:
  PidPathFollower()
  : rclcpp::Node("pid_path_follower"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // --- Params ---
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    path_topic_ = declare_parameter<std::string>("path_topic", "/global_path");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    max_linear_vel_  = declare_parameter<double>("max_linear_vel", 0.4);
    max_angular_vel_ = declare_parameter<double>("max_angular_vel", 1.2);

    // PID on heading error (main steering)
    ang_kp_ = declare_parameter<double>("ang_kp", 2.2);
    ang_ki_ = declare_parameter<double>("ang_ki", 0.0);
    ang_kd_ = declare_parameter<double>("ang_kd", 0.15);

    // Speed scheduling
    v_nominal_ = declare_parameter<double>("v_nominal", 0.25);
    v_min_     = declare_parameter<double>("v_min", 0.08);
    slow_down_angle_rad_ = declare_parameter<double>("slow_down_angle_rad", 0.8); // ~45deg

    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.6);
    goal_tolerance_     = declare_parameter<double>("goal_tolerance", 0.20);

    control_frequency_ = declare_parameter<double>("control_frequency", 20.0);

    integral_limit_ = declare_parameter<double>("integral_limit", 1.0);

    // --- ROS I/O ---
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10, std::bind(&PidPathFollower::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
      std::bind(&PidPathFollower::controlLoop, this));

    RCLCPP_INFO(get_logger(), "PID Path Follower started.");
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg || msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path.");
      path_received_ = false;
      return;
    }

    path_ = *msg;
    path_received_ = true;
    current_idx_ = 0;

    ang_integral_ = 0.0;
    ang_last_error_ = 0.0;

    RCLCPP_INFO(get_logger(), "New path received: %zu poses.", path_.poses.size());
  }

  bool getRobotPose(geometry_msgs::msg::Pose& pose)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      pose.position.x = tf.transform.translation.x;
      pose.position.y = tf.transform.translation.y;
      pose.position.z = tf.transform.translation.z;
      pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException&) {
      return false;
    }
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion& q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double wrapAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  int findClosestIdx(const geometry_msgs::msg::Pose& robot)
  {
    double best = std::numeric_limits<double>::max();
    int best_i = current_idx_;

    for (int i = current_idx_; i < static_cast<int>(path_.poses.size()); ++i) {
      const double dx = path_.poses[i].pose.position.x - robot.position.x;
      const double dy = path_.poses[i].pose.position.y - robot.position.y;
      const double d = std::hypot(dx, dy);
      if (d < best) {
        best = d;
        best_i = i;
      }
    }
    return best_i;
  }

  int findLookaheadIdx(const geometry_msgs::msg::Pose& robot, int start_i)
  {
    for (int i = start_i; i < static_cast<int>(path_.poses.size()); ++i) {
      const double dx = path_.poses[i].pose.position.x - robot.position.x;
      const double dy = path_.poses[i].pose.position.y - robot.position.y;
      if (std::hypot(dx, dy) >= lookahead_distance_) {
        return i;
      }
    }
    return static_cast<int>(path_.poses.size()) - 1;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void controlLoop()
  {
    if (!path_received_ || path_.poses.empty()) return;

    geometry_msgs::msg::Pose robot;
    if (!getRobotPose(robot)) return;

    current_idx_ = findClosestIdx(robot);
    const int target_idx = findLookaheadIdx(robot, current_idx_);
    const auto& target = path_.poses[target_idx].pose;

    const double dx = target.position.x - robot.position.x;
    const double dy = target.position.y - robot.position.y;
    const double dist_to_target = std::hypot(dx, dy);

    // If last waypoint and close enough -> stop
    if (target_idx == static_cast<int>(path_.poses.size()) - 1 && dist_to_target < goal_tolerance_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1500, "Goal reached. Stopping.");
      publishStop();
      path_received_ = false;
      return;
    }

    // Heading error
    const double target_yaw = std::atan2(dy, dx);
    const double robot_yaw = yawFromQuat(robot.orientation);
    const double ang_error = wrapAngle(target_yaw - robot_yaw);

    // PID on angle error
    const double dt = 1.0 / control_frequency_;
    ang_integral_ = std::clamp(ang_integral_ + ang_error * dt, -integral_limit_, integral_limit_);
    const double ang_deriv = (ang_error - ang_last_error_) / dt;
    ang_last_error_ = ang_error;

    double w = ang_kp_ * ang_error + ang_ki_ * ang_integral_ + ang_kd_ * ang_deriv;
    w = std::clamp(w, -max_angular_vel_, max_angular_vel_);

    // Speed scheduling: slow down when turning a lot
    double v = v_nominal_;
    const double a = std::min(std::abs(ang_error), slow_down_angle_rad_);
    const double ratio = 1.0 - (a / slow_down_angle_rad_);
    v = v_min_ + (v_nominal_ - v_min_) * std::max(0.0, ratio);
    v = std::clamp(v, 0.0, max_linear_vel_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Data
  nav_msgs::msg::Path path_;
  bool path_received_{false};
  int current_idx_{0};

  // Params
  std::string map_frame_, base_frame_;
  std::string path_topic_, cmd_vel_topic_;

  double max_linear_vel_{0.4};
  double max_angular_vel_{1.2};

  double ang_kp_{2.2}, ang_ki_{0.0}, ang_kd_{0.15};
  double v_nominal_{0.25}, v_min_{0.08};
  double slow_down_angle_rad_{0.8};

  double lookahead_distance_{0.6};
  double goal_tolerance_{0.20};
  double control_frequency_{20.0};
  double integral_limit_{1.0};

  // PID state
  double ang_integral_{0.0};
  double ang_last_error_{0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidPathFollower>());
  rclcpp::shutdown();
  return 0;
}
