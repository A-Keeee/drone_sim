#include "ake_drone_sim/se3_controller.hpp"
#include "ake_drone_sim/ros_utils.hpp"
#include "drone_msgs/msg/trajectory_setpoint.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stdexcept>

using namespace std::chrono_literals;

namespace ake_drone_sim {
class ControllerNode final : public rclcpp::Node {
 public:
  ControllerNode() : Node("position_controller_node") {
    VehicleParams vehicle;
    vehicle.mass = declare_parameter("mass", 1.0);
    vehicle.arm_length = declare_parameter("arm_length", 0.17);
    vehicle.inertia = vectorParameter("inertia", {0.008, 0.008, 0.014});
    vehicle.thrust_coefficient = declare_parameter("thrust_coefficient", 8.54858e-6);
    vehicle.torque_coefficient = declare_parameter("torque_coefficient", 1.6e-7);
    vehicle.min_rpm = declare_parameter("min_rpm", 0.0);
    vehicle.max_rpm = declare_parameter("max_rpm", 9000.0);

    ControllerGains gains;
    gains.kp = vectorParameter("kp", {2.2, 2.2, 3.5});
    gains.kv = vectorParameter("kv", {3.0, 3.0, 3.8});
    gains.kr = vectorParameter("kr", {0.12, 0.12, 0.08});
    gains.kw = vectorParameter("kw", {0.065, 0.065, 0.065});
    gains.max_torque = vectorParameter("max_torque", {0.35, 0.35, 0.20});
    gains.max_desired_rate = vectorParameter("max_desired_rate", {1.5, 1.5, 1.0});
    gains.desired_rate_filter_tau = declare_parameter("desired_rate_filter_tau", 0.10);
    gains.max_horizontal_accel = declare_parameter("max_horizontal_accel", 2.0);
    gains.max_vertical_accel = declare_parameter("max_vertical_accel", 2.5);
    gains.max_tilt = declare_parameter("max_tilt", 0.40);
    gains.max_total_thrust_ratio = declare_parameter("max_total_thrust_ratio", 1.8);
    odom_timeout_ = declare_parameter("odom_timeout", 0.20);
    reference_timeout_ = declare_parameter("reference_timeout", 2.50);
    controller_ = Se3Controller(vehicle, gains);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr message) {
          state_.position = eigen(message->pose.pose.position);
          state_.velocity = eigen(message->twist.twist.linear);
          state_.orientation = eigen(message->pose.pose.orientation).normalized();
          state_.body_rate = eigen(message->twist.twist.angular);
          last_odom_ = now();
          have_odom_ = true;
        });
    reference_sub_ = create_subscription<drone_msgs::msg::TrajectorySetpoint>(
        "/drone/reference", 10,
        [this](drone_msgs::msg::TrajectorySetpoint::SharedPtr message) {
          reference_.position = eigen(message->position);
          reference_.velocity = eigen(message->velocity);
          reference_.acceleration = eigen(message->acceleration);
          reference_.yaw = message->yaw;
          reference_.yaw_rate = message->yaw_rate;
          last_reference_ = now();
          have_reference_ = true;
        });
    rpm_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/drone/motor_rpm_cmd", 10);
    timer_ = create_wall_timer(10ms, std::bind(&ControllerNode::tick, this));

    RCLCPP_INFO(get_logger(),
                "SE3 gains kp=[%.2f %.2f %.2f] kv=[%.2f %.2f %.2f] "
                "kr=[%.3f %.3f %.3f] kw=[%.3f %.3f %.3f]",
                gains.kp.x(), gains.kp.y(), gains.kp.z(),
                gains.kv.x(), gains.kv.y(), gains.kv.z(),
                gains.kr.x(), gains.kr.y(), gains.kr.z(),
                gains.kw.x(), gains.kw.y(), gains.kw.z());
  }

 private:
  Eigen::Vector3d vectorParameter(const std::string &name,
                                  const std::vector<double> &defaults) {
    const auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain exactly three values");
    }
    return {values[0], values[1], values[2]};
  }

  void tick() {
    const auto stamp = now();
    double dt = 0.01;
    if (last_tick_.nanoseconds() != 0) {
      dt = clamp((stamp - last_tick_).seconds(), 0.001, 0.05);
    }
    last_tick_ = stamp;

    Eigen::Vector4d rpm = Eigen::Vector4d::Zero();
    const bool odom_fresh = have_odom_ &&
        (stamp - last_odom_).seconds() <= odom_timeout_;
    const bool reference_fresh = have_reference_ &&
        (stamp - last_reference_).seconds() <= reference_timeout_;
    if (odom_fresh && reference_fresh) {
      rpm = controller_.compute(state_, reference_, dt);
      reference_hold_active_ = false;
      zero_rpm_active_ = false;
    } else if (odom_fresh && have_reference_) {
      if (!reference_hold_active_) {
        failsafe_reference_ = reference_;
        failsafe_reference_.position = state_.position;
        failsafe_reference_.velocity.setZero();
        failsafe_reference_.acceleration.setZero();
        failsafe_reference_.yaw_rate = 0.0;
        controller_.reset();
        reference_hold_active_ = true;
        RCLCPP_WARN(get_logger(),
                    "stale reference: holding current position instead of zero RPM");
      }
      rpm = controller_.compute(state_, failsafe_reference_, dt);
      zero_rpm_active_ = false;
    } else if (!zero_rpm_active_) {
      controller_.reset();
      reference_hold_active_ = false;
      zero_rpm_active_ = true;
      RCLCPP_WARN(get_logger(),
                  "stale odometry or no reference: commanding zero RPM");
    }

    std_msgs::msg::Float32MultiArray message;
    message.data.reserve(4);
    for (int i = 0; i < 4; ++i) {
      message.data.push_back(static_cast<float>(rpm[i]));
    }
    rpm_pub_->publish(message);
  }

  Se3Controller controller_;
  ControlState state_;
  ControlReference reference_;
  ControlReference failsafe_reference_;
  double odom_timeout_{0.20};
  double reference_timeout_{2.50};
  bool have_odom_{false};
  bool have_reference_{false};
  bool reference_hold_active_{false};
  bool zero_rpm_active_{true};
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_reference_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<drone_msgs::msg::TrajectorySetpoint>::SharedPtr reference_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace ake_drone_sim

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ake_drone_sim::ControllerNode>());
  rclcpp::shutdown();
}
