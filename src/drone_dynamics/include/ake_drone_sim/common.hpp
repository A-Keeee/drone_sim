#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace ake_drone_sim {

constexpr double kGravity = 9.80665;
constexpr double kRpmToRad = 2.0 * M_PI / 60.0;

template<typename T> T clamp(T value, T lo, T hi) {
  return std::max(lo, std::min(value, hi));
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d s;
  s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return s;
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d &s) {
  return {s(2, 1), s(0, 2), s(1, 0)};
}

inline Eigen::Quaterniond integrateQuaternion(
    const Eigen::Quaterniond &q, const Eigen::Vector3d &body_rate, double dt) {
  const double angle = body_rate.norm() * dt;
  Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
  if (angle > 1e-12) {
    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, body_rate.normalized()));
  }
  Eigen::Quaterniond out = q * dq;
  out.normalize();
  return out;
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

}  // namespace ake_drone_sim
