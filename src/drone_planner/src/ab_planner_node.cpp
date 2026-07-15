#include "ake_drone_sim/astar3d.hpp"
#include "ake_drone_sim/bspline.hpp"
#include "ake_drone_sim/mpc_smoother.hpp"
#include "ake_drone_sim/pointcloud_utils.hpp"
#include "drone_msgs/msg/trajectory_setpoint.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <deque>

using namespace std::chrono_literals;

namespace ake_drone_sim {
class PlannerNode final : public rclcpp::Node {
 public:
  PlannerNode()
      : Node("ab_planner_node"),
        astar_(declare_parameter("max_search_nodes", 250000),
               declare_parameter("heuristic_weight", 1.05)) {
    resolution_ = declare_parameter("resolution", 0.2);
    safety_ = declare_parameter("safety_distance", 0.5) +
        declare_parameter("vehicle_radius", 0.18);
    speed_ = declare_parameter("reference_speed", 0.8);
    cruise_ = declare_parameter("cruise_altitude", 1.5);
    lidar_timeout_ = declare_parameter("lidar_timeout", 2.0);
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.15);
    goal_velocity_tolerance_ = declare_parameter("goal_velocity_tolerance", 0.15);
    polyline_lookahead_ = declare_parameter("polyline_lookahead", 0.45);
    spline_lookahead_ = declare_parameter("spline_lookahead", 0.15);

    MpcConfig mpc_config;
    mpc_config.horizon = static_cast<int>(std::max<int64_t>(
        5, declare_parameter("mpc_horizon", 30)));
    mpc_config.dt = declare_parameter("mpc_dt", 0.05);
    mpc_config.max_jerk = declare_parameter("mpc_max_jerk", 3.0);
    mpc_config.max_accel = declare_parameter("mpc_max_accel", 2.0);
    mpc_config.max_velocity = declare_parameter("mpc_max_velocity", 1.2);
    mpc_config.kp = declare_parameter("mpc_kp", 2.0);
    mpc_config.kv = declare_parameter("mpc_kv", 2.5);
    mpc_ = MpcSmoother(mpc_config);
    mpc_horizon_ = mpc_config.horizon;
    mpc_dt_ = mpc_config.dt;
    mpc_max_accel_ = mpc_config.max_accel;
    command_index_ = std::clamp(
        static_cast<int>(declare_parameter("mpc_command_index", 3)),
        0, mpc_horizon_ - 1);

    grid_.configure({-2, -4, 0}, {12, 8, 5}, resolution_);
    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/map/obstacles", latched_qos,
        std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/drone/lidar/points", rclcpp::SensorDataQoS(),
        std::bind(&PlannerNode::lidarCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", 10,
        std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/drone/goal", 10,
        std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    reference_pub_ = create_publisher<drone_msgs::msg::TrajectorySetpoint>(
        "/drone/reference", 10);
    safe_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/drone/safe_goal", 10);
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/drone/planned_path", latched_qos);
    spline_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/drone/bspline_path", latched_qos);
    prediction_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/drone/mpc_prediction_path", latched_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/drone/planner_status", 10);

    goal_ = {0, 0, cruise_};
    have_goal_ = declare_parameter("auto_takeoff", true);
    timer_ = create_wall_timer(20ms, std::bind(&PlannerNode::tick, this));
    RCLCPP_INFO(get_logger(),
                "planner speed %.2f m/s, MPC dt %.2f s, command index %d (%.2f s lead)",
                speed_, mpc_dt_, command_index_, (command_index_ + 1) * mpc_dt_);
  }

 private:
  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    const bool first_map = !have_map_;
    grid_.clear();
    for (const auto &point : readCloud(*message)) {
      grid_.set(grid_.worldToGrid(point));
    }
    inflated_ = grid_.inflated(safety_);
    have_map_ = true;
    if (first_map) need_plan_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    position_ = eigen(message->pose.pose.position);
    velocity_ = eigen(message->twist.twist.linear);
    orientation_ = eigen(message->pose.pose.orientation).normalized();
    if (!have_odom_) takeoff_xy_ = position_.head<2>();
    have_odom_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    Eigen::Vector3d new_goal = eigen(message->pose.position);
    if (new_goal.z() < 0.2) new_goal.z() = cruise_;
    if (have_goal_ && (new_goal - goal_).norm() < 1e-4) return;

    goal_ = new_goal;
    Eigen::Quaterniond q = eigen(message->pose.orientation);
    goal_yaw_ = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                           1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    have_goal_ = true;
    need_plan_ = true;
    goal_reached_ = false;
    hold_active_ = false;
    path_ready_ = false;
    command_acceleration_.setZero();
    lidar_replan_count_ = 0;
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    std::vector<Eigen::Vector3d> frame;
    frame.reserve(message->width * message->height);
    for (const auto &point : readCloud(*message)) {
      frame.push_back(position_ + orientation_ * point);
    }
    lidar_history_.push_back(std::move(frame));
    while (lidar_history_.size() > 3) lidar_history_.pop_front();
    last_lidar_ = now();
    have_lidar_ = true;

    if (!goal_reached_ && spline_.valid() && !use_polyline_) {
      int hits = 0;
      for (const auto &history_frame : lidar_history_) {
        for (const auto &obstacle : history_frame) {
          for (double u = current_u_;
               u <= std::min(spline_.maxU(), current_u_ + 2.0); u += 0.08) {
            if ((spline_.position(u) - obstacle).norm() < 0.18) {
              ++hits;
              break;
            }
          }
        }
      }
      if (hits > 0) {
        if (++threat_hits_ >= 5 && lidar_replan_count_ == 0) {
          need_plan_ = true;
          ++lidar_replan_count_;
        }
      } else {
        threat_hits_ = 0;
      }
    }
  }

  bool splineSafe(const BSpline &spline) const {
    for (double u = 0; u <= spline.maxU(); u += 0.03) {
      if (inflated_.occupiedWorld(spline.position(u))) return false;
    }
    return true;
  }

  void publishPath(const std::vector<Eigen::Vector3d> &points,
                   const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher) {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (const auto &point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = pointMsg(point);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    publisher->publish(path);
  }

  void plan() {
    need_plan_ = false;
    hold_active_ = false;
    command_acceleration_.setZero();
    const auto result = astar_.plan(inflated_, position_, goal_);
    if (!result.success) {
      path_ready_ = false;
      status("PLAN_FAILED:" + result.reason);
      return;
    }

    path_ = result.path;
    publishPath(path_, planned_path_pub_);
    use_polyline_ = false;
    auto controls = makeSplineControlPoints(path_, 0.55);
    BSpline candidate;
    candidate.setControlPoints(controls, 0.7);
    if (candidate.valid() && splineSafe(candidate)) {
      spline_ = candidate;
    } else {
      if (path_.size() < 2) {
        path_ready_ = false;
        return;
      }
      controls = makeSplineControlPoints(path_, 0.1);
      candidate.setControlPoints(controls, 0.25);
      if (!candidate.valid() || !splineSafe(candidate)) {
        use_polyline_ = true;
        path_ready_ = true;
        publishPath(path_, spline_path_pub_);
        status("EXECUTING_ASTAR_FALLBACK expanded=" +
               std::to_string(result.expanded));
        return;
      }
      spline_ = candidate;
    }

    table_.build(spline_);
    current_u_ = 0.0;
    current_s_ = 0.0;
    path_ready_ = true;
    std::vector<Eigen::Vector3d> samples;
    for (double u = 0; u <= spline_.maxU(); u += 0.03) {
      samples.push_back(spline_.position(u));
    }
    publishPath(samples, spline_path_pub_);
    status("EXECUTING expanded=" + std::to_string(result.expanded));
  }

  void publishReference(const Eigen::Vector3d &position,
                        const Eigen::Vector3d &velocity,
                        const Eigen::Vector3d &acceleration, double yaw) {
    drone_msgs::msg::TrajectorySetpoint message;
    message.header.stamp = now();
    message.header.frame_id = "map";
    message.position = pointMsg(position);
    message.velocity = vectorMsg(velocity);
    message.acceleration = vectorMsg(acceleration);
    message.yaw = yaw;
    reference_pub_->publish(message);

    geometry_msgs::msg::PoseStamped safe_goal;
    safe_goal.header = message.header;
    safe_goal.pose.position = message.position;
    safe_goal.pose.orientation = quatMsg(quaternionFromYaw(yaw));
    safe_goal_pub_->publish(safe_goal);
  }

  void hold() {
    if (!hold_active_) {
      hold_position_ = position_;
      hold_position_.z() = std::max(hold_position_.z(), 0.05);
      hold_active_ = true;
      command_acceleration_.setZero();
    }
    publishReference(hold_position_, Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(), goal_yaw_);
  }

  bool reachedGoal() const {
    return (position_ - goal_).norm() < goal_tolerance_ &&
           velocity_.norm() < goal_velocity_tolerance_;
  }

  void lockGoal() {
    goal_reached_ = true;
    hold_active_ = false;
    command_acceleration_.setZero();
    publishReference(goal_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     goal_yaw_);
    status("GOAL_REACHED");
  }

  void trackPolyline() {
    double best_error = 1e100;
    size_t best_segment = 0;
    double best_t = 0.0;
    for (size_t i = 0; i + 1 < path_.size(); ++i) {
      const Eigen::Vector3d direction = path_[i + 1] - path_[i];
      const double t = clamp(
          (position_ - path_[i]).dot(direction) /
              std::max(1e-9, direction.squaredNorm()),
          0.0, 1.0);
      const double error =
          (position_ - (path_[i] + t * direction)).squaredNorm();
      if (error < best_error) {
        best_error = error;
        best_segment = i;
        best_t = t;
      }
    }

    double remaining_lookahead = polyline_lookahead_;
    Eigen::Vector3d target = path_[best_segment] +
        best_t * (path_[best_segment + 1] - path_[best_segment]);
    for (size_t i = best_segment;
         i + 1 < path_.size() && remaining_lookahead > 0.0; ++i) {
      const Eigen::Vector3d end = path_[i + 1];
      const double length = (end - target).norm();
      if (length >= remaining_lookahead) {
        target += (end - target).normalized() * remaining_lookahead;
        remaining_lookahead = 0.0;
      } else {
        remaining_lookahead -= length;
        target = end;
      }
    }

    const Eigen::Vector3d delta = target - position_;
    Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();
    if (delta.norm() > 0.08) {
      const double stopping_speed = std::sqrt(
          2.0 * mpc_max_accel_ * std::max(0.0, (goal_ - position_).norm()));
      target_velocity = delta.normalized() *
          std::min({speed_, delta.norm(), stopping_speed});
    }
    hold_active_ = false;
    publishReference(target, target_velocity, Eigen::Vector3d::Zero(), goal_yaw_);

    // The safe A* fallback still has a rolling local prediction. Publishing it
    // makes the orange RViz path available in every planner mode instead of
    // only when a collision-free B-spline was accepted.
    std::vector<KinematicState> references(
        static_cast<size_t>(mpc_horizon_),
        KinematicState{target, target_velocity, Eigen::Vector3d::Zero()});
    const auto prediction = mpc_.solve(
        KinematicState{position_, velocity_, command_acceleration_}, references);
    std::vector<Eigen::Vector3d> prediction_points;
    prediction_points.reserve(prediction.size());
    for (const auto &state : prediction) {
      if (inflated_.occupiedWorld(state.p)) break;
      prediction_points.push_back(state.p);
    }
    if (prediction_points.size() >= 2) {
      publishPath(prediction_points, prediction_path_pub_);
    }
  }

  std::vector<KinematicState> splineReferences() {
    current_u_ = table_.closestU(spline_, position_, current_u_);
    current_s_ = std::max(current_s_, table_.lengthFromU(current_u_));
    const double start_s = std::min(table_.length(), current_s_ + spline_lookahead_);
    std::vector<KinematicState> references;
    references.reserve(mpc_horizon_);
    Eigen::Vector3d previous_velocity = velocity_;

    for (int k = 0; k < mpc_horizon_; ++k) {
      const double sample_s = std::min(
          table_.length(), start_s + speed_ * mpc_dt_ * (k + 1));
      const double u = table_.uFromLength(sample_s);
      const Eigen::Vector3d position = spline_.position(u);
      Eigen::Vector3d tangent = spline_.velocity(u);
      if (tangent.norm() > 1e-6) tangent.normalize();
      else tangent.setZero();

      const double distance_to_end = table_.length() - sample_s;
      const double target_speed = std::min(
          speed_, std::sqrt(std::max(0.0, 2.0 * mpc_max_accel_ * distance_to_end)));
      const Eigen::Vector3d target_velocity = tangent * target_speed;
      Eigen::Vector3d target_acceleration =
          (target_velocity - previous_velocity) / std::max(1e-3, mpc_dt_);
      if (target_acceleration.norm() > mpc_max_accel_) {
        target_acceleration = target_acceleration.normalized() * mpc_max_accel_;
      }
      references.push_back({position, target_velocity, target_acceleration});
      previous_velocity = target_velocity;
    }
    return references;
  }

  void trackSpline() {
    const auto references = splineReferences();
    const KinematicState initial{position_, velocity_, command_acceleration_};
    const auto prediction = mpc_.solve(initial, references);
    if (prediction.empty()) {
      status("MPC_FAILED_HOLD");
      hold();
      return;
    }

    for (const auto &state : prediction) {
      if (inflated_.occupiedWorld(state.p)) {
        status("MPC_COLLISION_ASTAR_FALLBACK");
        use_polyline_ = true;
        command_acceleration_.setZero();
        trackPolyline();
        return;
      }
    }

    const size_t index = std::min<size_t>(command_index_, prediction.size() - 1);
    const auto &command = prediction[index];
    command_acceleration_ = command.a;
    hold_active_ = false;
    publishReference(command.p, command.v, command.a, goal_yaw_);

    std::vector<Eigen::Vector3d> prediction_points;
    prediction_points.reserve(prediction.size());
    for (const auto &state : prediction) prediction_points.push_back(state.p);
    publishPath(prediction_points, prediction_path_pub_);
  }

  void tick() {
    if (!have_odom_) return;

    if (position_.z() < cruise_ - 0.25) {
      hold_active_ = false;
      publishReference({takeoff_xy_.x(), takeoff_xy_.y(), cruise_},
                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), goal_yaw_);
      return;
    }
    if (!have_map_ || !have_goal_) {
      hold();
      return;
    }
    if (have_lidar_ && (now() - last_lidar_).seconds() > lidar_timeout_) {
      status("LIDAR_TIMEOUT_HOLD");
      hold();
      return;
    }
    if (goal_reached_) {
      publishReference(goal_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       goal_yaw_);
      return;
    }
    if (reachedGoal()) {
      lockGoal();
      return;
    }
    if (need_plan_) plan();
    if (!path_ready_) {
      hold();
      return;
    }

    if (use_polyline_) trackPolyline();
    else trackSpline();
  }

  void status(const std::string &value) {
    if (value == last_status_) return;
    last_status_ = value;
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", value.c_str());
  }

  VoxelGrid grid_;
  VoxelGrid inflated_;
  AStar3D astar_;
  MpcSmoother mpc_;
  BSpline spline_;
  ArcLengthTable table_;
  std::vector<Eigen::Vector3d> path_;
  std::deque<std::vector<Eigen::Vector3d>> lidar_history_;
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d goal_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d command_acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d takeoff_xy_{Eigen::Vector2d::Zero()};
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
  double resolution_{0.2};
  double safety_{0.68};
  double speed_{0.8};
  double cruise_{1.5};
  double lidar_timeout_{2.0};
  double goal_tolerance_{0.15};
  double goal_velocity_tolerance_{0.15};
  double polyline_lookahead_{0.45};
  double spline_lookahead_{0.15};
  double goal_yaw_{0.0};
  double current_u_{0.0};
  double current_s_{0.0};
  double mpc_dt_{0.05};
  double mpc_max_accel_{2.0};
  int mpc_horizon_{30};
  int command_index_{3};
  bool have_map_{false};
  bool have_odom_{false};
  bool have_goal_{false};
  bool have_lidar_{false};
  bool need_plan_{true};
  bool path_ready_{false};
  bool use_polyline_{false};
  bool goal_reached_{false};
  bool hold_active_{false};
  int threat_hits_{0};
  int lidar_replan_count_{0};
  rclcpp::Time last_lidar_{0, 0, RCL_ROS_TIME};
  std::string last_status_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<drone_msgs::msg::TrajectorySetpoint>::SharedPtr reference_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr safe_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr spline_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prediction_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace ake_drone_sim

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ake_drone_sim::PlannerNode>());
  rclcpp::shutdown();
}
