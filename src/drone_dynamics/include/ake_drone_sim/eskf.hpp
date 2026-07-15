#pragma once

#include "ake_drone_sim/common.hpp"

namespace ake_drone_sim {

struct EskfParams {
  double gyro_noise{0.004};
  double accel_noise{0.08};
  double gyro_bias_noise{0.0002};
  double accel_bias_noise{0.002};
  double gps_position_noise{0.08};
  double gps_orientation_noise{0.03};
};

class Eskf {
 public:
  explicit Eskf(const EskfParams &params = {});
  void initialize(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, double stamp);
  bool predict(const Eigen::Vector3d &gyro, const Eigen::Vector3d &specific_force, double stamp);
  bool updatePose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);
  void observeImuOrientation(const Eigen::Quaterniond &orientation, double gain = 0.08);
  bool initialized() const { return initialized_; }
  const Eigen::Vector3d &position() const { return p_; }
  const Eigen::Vector3d &velocity() const { return v_; }
  const Eigen::Quaterniond &orientation() const { return q_; }
  const Eigen::Vector3d &bodyRate() const { return body_rate_; }
  const Eigen::Matrix<double, 15, 15> &covariance() const { return covariance_; }

 private:
  EskfParams params_;
  bool initialized_{false};
  double last_stamp_{0.0};
  Eigen::Vector3d p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_rate_{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 15, 15> covariance_{Eigen::Matrix<double, 15, 15>::Identity()};
};

}  // namespace ake_drone_sim
