#pragma once

#include "ake_drone_sim/dynamics_model.hpp"

namespace ake_drone_sim {

struct ControllerGains {
  Eigen::Vector3d kp{2.2, 2.2, 3.5};
  Eigen::Vector3d kv{3.0, 3.0, 3.8};
  Eigen::Vector3d kr{0.12, 0.12, 0.08};
  Eigen::Vector3d kw{0.065, 0.065, 0.065};
  Eigen::Vector3d max_torque{0.35, 0.35, 0.20};
  Eigen::Vector3d max_desired_rate{1.5, 1.5, 1.0};
  double desired_rate_filter_tau{0.10};
  double max_horizontal_accel{2.0};
  double max_vertical_accel{2.5};
  double max_tilt{0.40};
  double max_total_thrust_ratio{1.8};
};

struct ControlState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
};

struct ControlReference {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

class Se3Controller {
 public:
  Se3Controller(const VehicleParams &vehicle = {}, const ControllerGains &gains = {});
  Eigen::Vector4d compute(const ControlState &state, const ControlReference &reference,
                          double dt = 0.01);
  Eigen::Vector4d mix(double thrust, const Eigen::Vector3d &torque) const;
  Eigen::Vector4d lastWrench() const { return last_wrench_; }
  Eigen::Vector3d lastDesiredRate() const { return filtered_desired_rate_; }
  void reset();

 private:
  VehicleParams vehicle_;
  ControllerGains gains_;
  Eigen::Vector4d last_wrench_{Eigen::Vector4d::Zero()};
  Eigen::Matrix3d last_desired_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d filtered_desired_rate_{Eigen::Vector3d::Zero()};
  bool have_last_desired_rotation_{false};
};

}  // namespace ake_drone_sim
