#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "robot_description/srv/plan_path.hpp"

#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>

class AStarPlanner : public rclcpp::Node
{
public:
  AStarPlanner()
  : rclcpp::Node("astar_planner"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- Parameters (easy to tune from launch) ----
    map_topic_          = this->declare_parameter<std::string>("map_topic", "/map");
    amcl_pose_topic_    = this->declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    path_topic_         = this->declare_parameter<std::string>("path_topic", "/global_path");
    service_name_       = this->declare_parameter<std::string>("service_name", "/plan_path");

    map_frame_          = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_         = this->declare_parameter<std::string>("base_frame", "base_link");

    allow_unknown_      = this->declare_parameter<bool>("allow_unknown", true);
    occupied_threshold_ = this->declare_parameter<int>("occupied_threshold", 50);

    inflation_radius_m_ = this->declare_parameter<double>("inflation_radius", 0.20);

    // Limit expansions to avoid worst-case infinite searching
    max_iterations_     = this->declare_parameter<int>("max_iterations", 200000);

    RCLCPP_INFO(get_logger(), "A* Planner starting...");
    RCLCPP_INFO(get_logger(), " map_topic=%s amcl_pose_topic=%s path_topic=%s service=%s",
                map_topic_.c_str(), amcl_pose_topic_.c_str(), path_topic_.c_str(), service_name_.c_str());

    // ---- ROS I/O ----
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));

    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic_, 10,
      std::bind(&AStarPlanner::poseCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    plan_service_ = create_service<robot_description::srv::PlanPath>(
      service_name_,
      std::bind(&AStarPlanner::planPathCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "✓ Service %s ready", service_name_.c_str());
  }

private:
  // --- A* types ---
  struct GridKey {
    int x;
    int y;
    bool operator==(const GridKey& other) const { return x == other.x && y == other.y; }
  };

  struct GridKeyHash {
    std::size_t operator()(const GridKey& k) const {
      // decent hash for grid coords
      return (static_cast<std::size_t>(k.x) * 73856093u) ^ (static_cast<std::size_t>(k.y) * 19349663u);
    }
  };

  struct PQItem {
    GridKey key;
    double f;
    double g;
  };

  struct PQCompare {
    bool operator()(const PQItem& a, const PQItem& b) const {
      return a.f > b.f; // min-heap by f
    }
  };

  // --- Callbacks ---
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (!msg) return;

    map_ = msg;
    map_received_ = true;

    resolution_ = msg->info.resolution;
    origin_x_   = msg->info.origin.position.x;
    origin_y_   = msg->info.origin.position.y;
    width_      = static_cast<int>(msg->info.width);
    height_     = static_cast<int>(msg->info.height);

    // Precompute inflation in cells
    inflation_cells_ = std::max(0, static_cast<int>(std::ceil(inflation_radius_m_ / resolution_)));

    RCLCPP_INFO(get_logger(), "✓ Map received: %dx%d res=%.3f origin=(%.2f, %.2f) inflation=%d cells",
                width_, height_, resolution_, origin_x_, origin_y_, inflation_cells_);
  }

  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!msg) return;
    current_pose_ = msg->pose.pose;
    if (!current_pose_received_) {
      current_pose_received_ = true;
      RCLCPP_INFO(get_logger(), "✓ First AMCL pose: (%.2f, %.2f)",
                  current_pose_.position.x, current_pose_.position.y);
    }
  }

  // --- Grid helpers ---
  bool worldToGrid(double wx, double wy, int& gx, int& gy) const
  {
    if (!map_received_) return false;
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
  }

  void gridToWorld(int gx, int gy, double& wx, double& wy) const
  {
    // center of cell
    wx = origin_x_ + (gx + 0.5) * resolution_;
    wy = origin_y_ + (gy + 0.5) * resolution_;
  }

  bool isOccupiedCell(int x, int y) const
  {
    if (!map_received_) return true;
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return true;

    const int idx = y * width_ + x;
    if (idx < 0 || idx >= static_cast<int>(map_->data.size())) return true;

    const int v = map_->data[idx];

    if (v < 0) {
      // unknown
      return !allow_unknown_;
    }
    return (v >= occupied_threshold_);
  }

  bool isInflationFree(int x, int y) const
  {
    // if inflation=0, only check the cell itself
    for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
      for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (isOccupiedCell(nx, ny)) return false;
      }
    }
    return true;
  }

  static double heuristic(int x1, int y1, int x2, int y2)
  {
    // Euclidean
    return std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
  }

  std::vector<GridKey> neighbors8(const GridKey& k) const
  {
    static const int dirs[8][2] = {
      {-1,-1},{-1,0},{-1,1},
      { 0,-1},      { 0,1},
      { 1,-1},{ 1,0},{ 1,1}
    };

    std::vector<GridKey> out;
    out.reserve(8);

    for (const auto& d : dirs) {
      GridKey n{ k.x + d[0], k.y + d[1] };
      if (n.x < 0 || n.x >= width_ || n.y < 0 || n.y >= height_) continue;

      // Collision check
      if (!isInflationFree(n.x, n.y)) continue;

      if (d[0] != 0 && d[1] != 0) {
        if (!isInflationFree(k.x + d[0], k.y) || !isInflationFree(k.x, k.y + d[1])) {
          continue;
        }
      }

      out.push_back(n);
    }
    return out;
  }

  // --- A* planning ---
  nav_msgs::msg::Path planPath(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& goal)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();

    if (!map_received_) {
      RCLCPP_WARN(get_logger(), "No map yet, cannot plan.");
      return path;
    }

    int sx, sy, gx, gy;
    if (!worldToGrid(start.position.x, start.position.y, sx, sy)) {
      RCLCPP_ERROR(get_logger(), "Start out of bounds.");
      return path;
    }
    if (!worldToGrid(goal.position.x, goal.position.y, gx, gy)) {
      RCLCPP_ERROR(get_logger(), "Goal out of bounds.");
      return path;
    }

    if (!isInflationFree(sx, sy)) {
      RCLCPP_ERROR(get_logger(), "Start is in collision (inflated).");
      return path;
    }
    if (!isInflationFree(gx, gy)) {
      RCLCPP_ERROR(get_logger(), "Goal is in collision (inflated).");
      return path;
    }

    const GridKey start_k{sx, sy};
    const GridKey goal_k{gx, gy};

    std::priority_queue<PQItem, std::vector<PQItem>, PQCompare> open;
    std::unordered_map<GridKey, double, GridKeyHash> g_score;
    std::unordered_map<GridKey, GridKey, GridKeyHash> came_from;
    std::unordered_set<GridKey, GridKeyHash> closed;

    g_score[start_k] = 0.0;
    open.push(PQItem{start_k, heuristic(sx, sy, gx, gy), 0.0});

    int iterations = 0;
    bool found = false;

    while (!open.empty() && iterations++ < max_iterations_) {
      const auto current = open.top();
      open.pop();

      // Skip if already processed with better score
      if (closed.find(current.key) != closed.end()) continue;

      if (current.key == goal_k) {
        found = true;
        break;
      }

      closed.insert(current.key);

      for (const auto& n : neighbors8(current.key)) {
        if (closed.find(n) != closed.end()) continue;

        const bool diagonal = (n.x != current.key.x) && (n.y != current.key.y);
        const double step = diagonal ? std::sqrt(2.0) : 1.0;

        const double tentative_g = g_score[current.key] + step;

        auto it = g_score.find(n);
        if (it == g_score.end() || tentative_g < it->second) {
          g_score[n] = tentative_g;
          came_from[n] = current.key;

          const double h = heuristic(n.x, n.y, gx, gy);
          open.push(PQItem{n, tentative_g + h, tentative_g});
        }
      }
    }

    if (!found) {
      RCLCPP_WARN(get_logger(), "No path found (iters=%d / max=%d)", iterations, max_iterations_);
      return path;
    }

    // Reconstruct
    std::vector<GridKey> cells;
    cells.reserve(2048);

    GridKey cur = goal_k;
    cells.push_back(cur);
    while (!(cur == start_k)) {
      auto it = came_from.find(cur);
      if (it == came_from.end()) break;
      cur = it->second;
      cells.push_back(cur);
    }
    std::reverse(cells.begin(), cells.end());

    // Convert to Path
    path.poses.reserve(cells.size());
    for (const auto& c : cells) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = map_frame_;
      ps.header.stamp = now();
      gridToWorld(c.x, c.y, ps.pose.position.x, ps.pose.position.y);
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }

    RCLCPP_INFO(get_logger(), "✓ Path planned: %zu poses (iters=%d)", path.poses.size(), iterations);
    return path;
  }

  // --- Service ---
  void planPathCallback(
    const std::shared_ptr<robot_description::srv::PlanPath::Request> request,
    std::shared_ptr<robot_description::srv::PlanPath::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Service called: goal=(%.2f, %.2f) frame=%s",
                request->goal.pose.position.x,
                request->goal.pose.position.y,
                request->goal.header.frame_id.c_str());

    if (!map_received_) {
      RCLCPP_ERROR(get_logger(), "Cannot plan: no map received yet.");
      response->path.header.frame_id = map_frame_;
      response->path.header.stamp = now();
      return;
    }

    geometry_msgs::msg::Pose start_pose;

    if (current_pose_received_) {
      start_pose = current_pose_;
    } else {
      
      try {
        auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        start_pose.position.x = tf.transform.translation.x;
        start_pose.position.y = tf.transform.translation.y;
        start_pose.position.z = tf.transform.translation.z;
        start_pose.orientation = tf.transform.rotation;
      } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "TF lookup failed (%s -> %s): %s",
                     map_frame_.c_str(), base_frame_.c_str(), ex.what());
        response->path.header.frame_id = map_frame_;
        response->path.header.stamp = now();
        return;
      }
    }

    geometry_msgs::msg::Pose goal_pose = request->goal.pose;

    response->path = planPath(start_pose, goal_pose);

    if (!response->path.poses.empty()) {
      path_pub_->publish(response->path);
      RCLCPP_INFO(get_logger(), "Published path on %s", path_topic_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Returned empty path.");
    }
  }

  // --- Members ---
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<robot_description::srv::PlanPath>::SharedPtr plan_service_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  geometry_msgs::msg::Pose current_pose_;
  bool map_received_{false};
  bool current_pose_received_{false};

  double resolution_{0.05};
  double origin_x_{0.0}, origin_y_{0.0};
  int width_{0}, height_{0};

  bool allow_unknown_{true};
  int occupied_threshold_{50};
  double inflation_radius_m_{0.20};
  int inflation_cells_{0};
  int max_iterations_{200000};

  std::string map_topic_;
  std::string amcl_pose_topic_;
  std::string path_topic_;
  std::string service_name_;
  std::string map_frame_;
  std::string base_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}
