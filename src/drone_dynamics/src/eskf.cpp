#include "ake_drone_sim/eskf.hpp"

namespace ake_drone_sim {

Eskf::Eskf(const EskfParams &params) : params_(params) { covariance_ *= 0.1; }

void Eskf::initialize(const Eigen::Vector3d &position,
                      const Eigen::Quaterniond &orientation, double stamp) {
  p_ = position; v_.setZero(); q_ = orientation.normalized();
  bg_.setZero(); ba_.setZero(); body_rate_.setZero();
  covariance_.setIdentity(); covariance_ *= 0.05;
  last_stamp_ = stamp; initialized_ = true;
}

bool Eskf::predict(const Eigen::Vector3d &gyro,
                   const Eigen::Vector3d &specific_force, double stamp) {
  if (!initialized_) return false;
  const double dt = stamp - last_stamp_;
  if (!(dt > 0.0) || dt > 0.1) { last_stamp_ = stamp; return false; }
  body_rate_ = gyro - bg_;
  const Eigen::Vector3d accel = specific_force - ba_;
  const Eigen::Matrix3d rotation = q_.toRotationMatrix();
  const Eigen::Vector3d world_accel = rotation * accel - kGravity * Eigen::Vector3d::UnitZ();
  p_ += v_ * dt + 0.5 * world_accel * dt * dt;
  v_ += world_accel * dt;
  q_ = integrateQuaternion(q_, body_rate_, dt);

  Eigen::Matrix<double, 15, 15> f = Eigen::Matrix<double, 15, 15>::Zero();
  f.block<3, 3>(0, 3).setIdentity();
  f.block<3, 3>(3, 6) = -rotation * skew(accel);
  f.block<3, 3>(3, 12) = -rotation;
  f.block<3, 3>(6, 6) = -skew(body_rate_);
  f.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();
  const auto fd = Eigen::Matrix<double, 15, 15>::Identity() + f * dt;
  Eigen::Matrix<double, 15, 15> qn = Eigen::Matrix<double, 15, 15>::Zero();
  qn.block<3, 3>(3, 3).diagonal().setConstant(params_.accel_noise * params_.accel_noise);
  qn.block<3, 3>(6, 6).diagonal().setConstant(params_.gyro_noise * params_.gyro_noise);
  qn.block<3, 3>(9, 9).diagonal().setConstant(params_.gyro_bias_noise * params_.gyro_bias_noise);
  qn.block<3, 3>(12, 12).diagonal().setConstant(params_.accel_bias_noise * params_.accel_bias_noise);
  covariance_ = fd * covariance_ * fd.transpose() + qn * dt;
  last_stamp_ = stamp;
  return true;
}

bool Eskf::updatePose(const Eigen::Vector3d &position,
                      const Eigen::Quaterniond &orientation) {
  if (!initialized_) return false;
  Eigen::Matrix<double, 6, 15> h = Eigen::Matrix<double, 6, 15>::Zero();
  h.block<3, 3>(0, 0).setIdentity(); h.block<3, 3>(3, 6).setIdentity();
  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = position - p_;
  Eigen::Quaterniond dq = q_.conjugate() * orientation.normalized();
  if (dq.w() < 0.0) dq.coeffs() *= -1.0;
  residual.tail<3>() = 2.0 * dq.vec();
  Eigen::Matrix<double, 6, 6> r = Eigen::Matrix<double, 6, 6>::Zero();
  r.block<3, 3>(0, 0).diagonal().setConstant(params_.gps_position_noise * params_.gps_position_noise);
  r.block<3, 3>(3, 3).diagonal().setConstant(params_.gps_orientation_noise * params_.gps_orientation_noise);
  const auto s = h * covariance_ * h.transpose() + r;
  const Eigen::Matrix<double, 15, 6> k = covariance_ * h.transpose() * s.inverse();
  const Eigen::Matrix<double, 15, 1> dx = k * residual;
  p_ += dx.segment<3>(0); v_ += dx.segment<3>(3);
  q_ = integrateQuaternion(q_, dx.segment<3>(6), 1.0);
  bg_ += dx.segment<3>(9); ba_ += dx.segment<3>(12);
  const auto ident = Eigen::Matrix<double, 15, 15>::Identity();
  covariance_ = (ident - k * h) * covariance_ * (ident - k * h).transpose() + k * r * k.transpose();
  return p_.allFinite() && v_.allFinite() && q_.coeffs().allFinite();
}

void Eskf::observeImuOrientation(const Eigen::Quaterniond &orientation, double gain) {
  if (!initialized_ || !orientation.coeffs().allFinite()) return;
  Eigen::Quaterniond measurement = orientation.normalized();
  if (q_.dot(measurement) < 0.0) measurement.coeffs() *= -1.0;
  q_ = q_.slerp(clamp(gain, 0.0, 1.0), measurement).normalized();
}

}  // namespace ake_drone_sim
