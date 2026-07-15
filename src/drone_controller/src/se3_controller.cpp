#include "ake_drone_sim/se3_controller.hpp"

namespace ake_drone_sim {

Se3Controller::Se3Controller(const VehicleParams &vehicle, const ControllerGains &gains)
    : vehicle_(vehicle), gains_(gains) {}

Eigen::Vector4d Se3Controller::mix(double thrust, const Eigen::Vector3d &torque) const {
  const double a = vehicle_.arm_length / std::sqrt(2.0);
  const double c = vehicle_.torque_coefficient / vehicle_.thrust_coefficient;
  Eigen::Matrix4d allocation;
  allocation << 1, 1, 1, 1,
      a, a, -a, -a,
      -a, a, a, -a,
      c, -c, c, -c;
  Eigen::Vector4d forces = allocation.inverse() *
      (Eigen::Vector4d() << thrust, torque.x(), torque.y(), torque.z()).finished();
  Eigen::Vector4d rpm;
  for (int i = 0; i < 4; ++i) {
    const double omega2 = std::max(0.0, forces[i] / vehicle_.thrust_coefficient);
    rpm[i] = clamp(std::sqrt(omega2) / kRpmToRad, vehicle_.min_rpm, vehicle_.max_rpm);
  }
  return rpm;
}

Eigen::Vector4d Se3Controller::compute(const ControlState &s, const ControlReference &r) {
  Eigen::Vector3d accel = gains_.kp.cwiseProduct(r.position - s.position) +
      gains_.kv.cwiseProduct(r.velocity - s.velocity) + r.acceleration;
  Eigen::Vector2d horizontal = accel.head<2>();
  if (horizontal.norm() > gains_.max_horizontal_accel)
    accel.head<2>() = horizontal.normalized() * gains_.max_horizontal_accel;
  accel.z() = clamp(accel.z(), -gains_.max_vertical_accel, gains_.max_vertical_accel);
  Eigen::Vector3d desired_force = vehicle_.mass *
      (accel + kGravity * Eigen::Vector3d::UnitZ());
  Eigen::Vector3d b3 = desired_force.normalized();
  const double max_horizontal = std::tan(gains_.max_tilt) * std::max(0.1, b3.z());
  if (b3.head<2>().norm() > max_horizontal) {
    b3.head<2>() = b3.head<2>().normalized() * max_horizontal;
    b3.normalize();
  }
  const Eigen::Vector3d heading(std::cos(r.yaw), std::sin(r.yaw), 0.0);
  Eigen::Vector3d b2 = b3.cross(heading);
  if (b2.norm() < 1e-5) b2 = Eigen::Vector3d::UnitY();
  b2.normalize();
  const Eigen::Vector3d b1 = b2.cross(b3).normalized();
  Eigen::Matrix3d desired_rotation;
  desired_rotation << b1, b2, b3;
  const Eigen::Matrix3d rotation = s.orientation.toRotationMatrix();
  const Eigen::Vector3d attitude_error = 0.5 * vee(
      desired_rotation.transpose() * rotation - rotation.transpose() * desired_rotation);
  const Eigen::Vector3d desired_rate(0.0, 0.0, r.yaw_rate);
  const Eigen::Vector3d rate_error = s.body_rate - rotation.transpose() * desired_rotation * desired_rate;
  const Eigen::Matrix3d inertia = vehicle_.inertia.asDiagonal();
  const Eigen::Vector3d torque = -gains_.kr.cwiseProduct(attitude_error) -
      gains_.kw.cwiseProduct(rate_error) + s.body_rate.cross(inertia * s.body_rate);
  const double max_thrust = gains_.max_total_thrust_ratio * vehicle_.mass * kGravity;
  const double thrust = clamp(desired_force.dot(rotation.col(2)), 0.0, max_thrust);
  last_wrench_ << thrust, torque.x(), torque.y(), torque.z();
  return mix(thrust, torque);
}

}  // namespace ake_drone_sim
