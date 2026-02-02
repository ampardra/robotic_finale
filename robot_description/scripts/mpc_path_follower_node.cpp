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
#include <vector>
#include <chrono>

class SamplingMpcFollower : public rclcpp::Node
{
public:
  SamplingMpcFollower()
  : rclcpp::Node("mpc_path_follower"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    path_topic_ = declare_parameter<std::string>("path_topic", "/global_path");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    // Limits
    v_max_ = declare_parameter<double>("v_max", 0.45);
    w_max_ = declare_parameter<double>("w_max", 1.5);

    v_ref_ = declare_parameter<double>("v_ref", 0.25);

    // MPC settings
    dt_ = declare_parameter<double>("dt", 0.10);
    horizon_ = declare_parameter<int>("horizon", 12);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.7);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.20);

    // Cost weights
    Q_cte_ = declare_parameter<double>("Q_cte", 4.0);        // cross track error
    Q_heading_ = declare_parameter<double>("Q_heading", 2.0);// heading error
    R_w_ = declare_parameter<double>("R_w", 0.2);            // angular effort
    R_dw_ = declare_parameter<double>("R_dw", 0.4);          // angular smoothness
    R_v_ = declare_parameter<double>("R_v", 0.3);            // speed error

    // Candidate sampling
    w_samples_ = declare_parameter<int>("w_samples", 11);     // number of omega samples
    v_samples_ = declare_parameter<int>("v_samples", 3);      // number of v samples

    control_frequency_ = declare_parameter<double>("control_frequency", 10.0);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10, std::bind(&SamplingMpcFollower::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
      std::bind(&SamplingMpcFollower::loop, this));

    RCLCPP_INFO(get_logger(), "Sampling MPC Path Follower started.");
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg || msg->poses.empty()) {
      path_received_ = false;
      RCLCPP_WARN(get_logger(), "Empty path.");
      return;
    }
    path_ = *msg;
    path_received_ = true;
    current_idx_ = 0;
    last_w_ = 0.0;
    RCLCPP_INFO(get_logger(), "New path received: %zu poses.", path_.poses.size());
  }

  bool getRobotPose(double& x, double& y, double& yaw)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;

      const auto& q = tf.transform.rotation;
      const double siny = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      yaw = std::atan2(siny, cosy);
      return true;
    } catch (const tf2::TransformException&) {
      return false;
    }
  }

  static double wrap(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  int findClosestIdx(double rx, double ry)
  {
    double best = std::numeric_limits<double>::max();
    int best_i = current_idx_;

    for (int i = current_idx_; i < static_cast<int>(path_.poses.size()); ++i) {
      const double dx = path_.poses[i].pose.position.x - rx;
      const double dy = path_.poses[i].pose.position.y - ry;
      const double d = std::hypot(dx, dy);
      if (d < best) { best = d; best_i = i; }
    }
    return best_i;
  }

  int findLookaheadIdx(double rx, double ry, int start_i)
  {
    for (int i = start_i; i < static_cast<int>(path_.poses.size()); ++i) {
      const double dx = path_.poses[i].pose.position.x - rx;
      const double dy = path_.poses[i].pose.position.y - ry;
      if (std::hypot(dx, dy) >= lookahead_distance_) return i;
    }
    return static_cast<int>(path_.poses.size()) - 1;
  }

  // Compute simple cross-track + heading error w.r.t. segment (i -> i+1)
  void computeErrors(double rx, double ry, double ryaw, int idx,
                     double& cte, double& heading_err, double& path_yaw)
  {
    const int i0 = std::clamp(idx, 0, (int)path_.poses.size() - 2);
    const auto& p0 = path_.poses[i0].pose.position;
    const auto& p1 = path_.poses[i0 + 1].pose.position;

    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    path_yaw = std::atan2(vy, vx);

    // cross track error: signed distance to the segment line
    // Using normal of segment
    const double nx = -vy;
    const double ny =  vx;

    const double px = rx - p0.x;
    const double py = ry - p0.y;

    const double norm = std::hypot(nx, ny);
    cte = (norm > 1e-6) ? (px * nx + py * ny) / norm : 0.0;

    heading_err = wrap(path_yaw - ryaw);
  }

  // forward simulate unicycle with constant (v,w) for horizon and compute cost
  double rolloutCost(double rx, double ry, double ryaw,
                     int idx_start, double v, double w)
  {
    double x = rx, y = ry, yaw = ryaw;
    double cost = 0.0;
    double prev_w = last_w_;

    int idx = idx_start;

    for (int k = 0; k < horizon_; ++k) {
      // predict
      x += v * std::cos(yaw) * dt_;
      y += v * std::sin(yaw) * dt_;
      yaw = wrap(yaw + w * dt_);

      // move along path index a bit (cheap approximation)
      idx = std::min(idx + 1, (int)path_.poses.size() - 2);

      double cte, he, path_yaw;
      computeErrors(x, y, yaw, idx, cte, he, path_yaw);

      cost += Q_cte_ * (cte * cte);
      cost += Q_heading_ * (he * he);
      cost += R_w_ * (w * w);
      cost += R_dw_ * ((w - prev_w) * (w - prev_w));
      cost += R_v_ * ((v - v_ref_) * (v - v_ref_));

      prev_w = w;
    }
    return cost;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void loop()
  {
    if (!path_received_ || path_.poses.size() < 2) return;

    double rx, ry, ryaw;
    if (!getRobotPose(rx, ry, ryaw)) return;

    current_idx_ = findClosestIdx(rx, ry);
    const int look_idx = findLookaheadIdx(rx, ry, current_idx_);

    // stop at end
    const auto& lastp = path_.poses.back().pose.position;
    if (std::hypot(lastp.x - rx, lastp.y - ry) < goal_tolerance_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1500, "Goal reached. Stopping.");
      publishStop();
      path_received_ = false;
      return;
    }

    // sample controls
    double best_cost = std::numeric_limits<double>::max();
    double best_v = 0.0;
    double best_w = 0.0;

    for (int vi = 0; vi < v_samples_; ++vi) {
      const double v = (v_samples_ == 1) ? v_ref_
                      : (0.10 + (v_max_ - 0.10) * (double)vi / (double)(v_samples_ - 1));

      for (int wi = 0; wi < w_samples_; ++wi) {
        const double w = -w_max_ + 2.0 * w_max_ * (double)wi / (double)(w_samples_ - 1);

        const double cost = rolloutCost(rx, ry, ryaw, look_idx, v, w);
        if (cost < best_cost) {
          best_cost = cost;
          best_v = v;
          best_w = w;
        }
      }
    }

    // clamp
    best_v = std::clamp(best_v, 0.0, v_max_);
    best_w = std::clamp(best_w, -w_max_, w_max_);
    last_w_ = best_w;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = best_v;
    cmd.angular.z = best_w;
    cmd_pub_->publish(cmd);
  }

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Data
  nav_msgs::msg::Path path_;
  bool path_received_{false};
  int current_idx_{0};

  // Params
  std::string map_frame_, base_frame_, path_topic_, cmd_vel_topic_;
  double v_max_{0.45}, w_max_{1.5}, v_ref_{0.25};
  double dt_{0.10};
  int horizon_{12};
  double lookahead_distance_{0.7};
  double goal_tolerance_{0.20};

  double Q_cte_{4.0}, Q_heading_{2.0}, R_w_{0.2}, R_dw_{0.4}, R_v_{0.3};
  int w_samples_{11}, v_samples_{3};
  double control_frequency_{10.0};

  double last_w_{0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SamplingMpcFollower>());
  rclcpp::shutdown();
  return 0;
}
