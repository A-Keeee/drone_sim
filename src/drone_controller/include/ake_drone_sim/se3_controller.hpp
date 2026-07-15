#pragma once

#include "ake_drone_sim/dynamics_model.hpp"

namespace ake_drone_sim {

struct ControllerGains {
  Eigen::Vector3d kp{3.8, 3.8, 5.5};
  Eigen::Vector3d kv{2.8, 2.8, 3.6};
  Eigen::Vector3d kr{0.18, 0.18, 0.12};
  Eigen::Vector3d kw{0.035, 0.035, 0.025};
  double max_horizontal_accel{4.0};
  double max_vertical_accel{4.0};
  double max_tilt{0.61};
  double max_total_thrust_ratio{2.5};
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
  Eigen::Vector4d compute(const ControlState &state, const ControlReference &reference);
  Eigen::Vector4d mix(double thrust, const Eigen::Vector3d &torque) const;
  Eigen::Vector4d lastWrench() const { return last_wrench_; }

 private:
  VehicleParams vehicle_;
  ControllerGains gains_;
  Eigen::Vector4d last_wrench_{Eigen::Vector4d::Zero()};
};

}  // namespace ake_drone_sim
