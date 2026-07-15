#include "ake_drone_sim/dynamics_model.hpp"

namespace ake_drone_sim {

DynamicsModel::DynamicsModel(const VehicleParams &params) : params_(params) {}

void DynamicsModel::reset(const VehicleState &state) {
  state_ = state;
  state_.orientation.normalize();
  command_rpm_.setZero();
}

void DynamicsModel::setMotorCommand(const Eigen::Vector4d &rpm) {
  for (int i = 0; i < 4; ++i) {
    command_rpm_[i] = std::isfinite(rpm[i]) ? clamp(rpm[i], params_.min_rpm, params_.max_rpm) : 0.0;
  }
}

Eigen::Vector4d DynamicsModel::motorForces() const {
  return params_.thrust_coefficient *
         (state_.motor_rpm * kRpmToRad).array().square().matrix();
}

Eigen::Vector4d DynamicsModel::wrench() const {
  const Eigen::Vector4d f = motorForces();
  const double a = params_.arm_length / std::sqrt(2.0);
  Eigen::Vector4d out;
  out << f.sum(), a * (f[0] + f[1] - f[2] - f[3]),
      a * (-f[0] + f[1] + f[2] - f[3]),
      (params_.torque_coefficient / params_.thrust_coefficient) *
          (f[0] - f[1] + f[2] - f[3]);
  return out;
}

void DynamicsModel::step(double dt) {
  if (!(dt > 0.0) || dt > 0.05) return;
  const double alpha = 1.0 - std::exp(-dt / std::max(1e-4, params_.motor_time_constant));
  state_.motor_rpm += alpha * (command_rpm_ - state_.motor_rpm);
  const Eigen::Vector4d wt = wrench();
  const Eigen::Matrix3d rotation = state_.orientation.toRotationMatrix();
  const Eigen::Vector3d thrust_world = rotation * Eigen::Vector3d(0.0, 0.0, wt[0]);
  state_.world_acceleration = (thrust_world + external_force_ -
      params_.linear_drag * state_.velocity) / params_.mass -
      kGravity * Eigen::Vector3d::UnitZ();
  state_.velocity += state_.world_acceleration * dt;
  state_.position += state_.velocity * dt;

  const Eigen::Matrix3d inertia = params_.inertia.asDiagonal();
  const Eigen::Vector3d torque = wt.tail<3>();
  const Eigen::Vector3d rate_dot = inertia.inverse() *
      (torque - state_.body_rate.cross(inertia * state_.body_rate));
  state_.body_rate += rate_dot * dt;
  state_.orientation = integrateQuaternion(state_.orientation, state_.body_rate, dt);

  if (state_.position.z() < 0.05) {
    state_.position.z() = 0.05;
    if (state_.velocity.z() < 0.0) {
      state_.velocity.z() = 0.0;
      state_.world_acceleration.z() = 0.0;
    }
    // A minimal landing-gear contact model prevents an unpowered vehicle from
    // rotating through the floor while still allowing a normal takeoff.
    if (wt[0] < 1.05 * params_.mass * kGravity) {
      state_.velocity.x() = 0.0;
      state_.velocity.y() = 0.0;
      state_.body_rate *= std::exp(-12.0 * dt);
      const double yaw = std::atan2(
          2.0 * (state_.orientation.w() * state_.orientation.z() +
                 state_.orientation.x() * state_.orientation.y()),
          1.0 - 2.0 * (state_.orientation.y() * state_.orientation.y() +
                       state_.orientation.z() * state_.orientation.z()));
      state_.orientation = state_.orientation.slerp(
          1.0 - std::exp(-15.0 * dt), quaternionFromYaw(yaw)).normalized();
    }
  }
  if (!state_.position.allFinite() || !state_.velocity.allFinite() ||
      !state_.body_rate.allFinite() || !state_.orientation.coeffs().allFinite()) {
    reset();
  }
}

}  // namespace ake_drone_sim
