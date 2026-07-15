#pragma once

#include "ake_drone_sim/common.hpp"
#include <array>

namespace ake_drone_sim {

struct VehicleParams {
  double mass{1.0};
  Eigen::Vector3d inertia{0.008, 0.008, 0.014};
  double arm_length{0.17};
  double thrust_coefficient{8.54858e-6};
  double torque_coefficient{1.6e-7};
  double motor_time_constant{0.035};
  double min_rpm{0.0};
  double max_rpm{9000.0};
  double linear_drag{0.08};
};

struct VehicleState {
  Eigen::Vector3d position{0.0, 0.0, 0.05};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
  Eigen::Vector4d motor_rpm{Eigen::Vector4d::Zero()};
  Eigen::Vector3d world_acceleration{Eigen::Vector3d::Zero()};
};

class DynamicsModel {
 public:
  explicit DynamicsModel(const VehicleParams &params = {});
  void reset(const VehicleState &state = {});
  void setMotorCommand(const Eigen::Vector4d &rpm);
  void setExternalForce(const Eigen::Vector3d &force) { external_force_ = force; }
  void step(double dt);
  const VehicleState &state() const { return state_; }
  const VehicleParams &params() const { return params_; }
  Eigen::Vector4d motorForces() const;
  Eigen::Vector4d wrench() const;

 private:
  VehicleParams params_;
  VehicleState state_;
  Eigen::Vector4d command_rpm_{Eigen::Vector4d::Zero()};
  Eigen::Vector3d external_force_{Eigen::Vector3d::Zero()};
};

}  // namespace ake_drone_sim
