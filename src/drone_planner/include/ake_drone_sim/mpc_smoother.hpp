#pragma once
#include "ake_drone_sim/common.hpp"
#include <vector>
namespace ake_drone_sim {
struct KinematicState { Eigen::Vector3d p{Eigen::Vector3d::Zero()},v{Eigen::Vector3d::Zero()},a{Eigen::Vector3d::Zero()}; };
struct MpcConfig { int horizon{30};double dt{0.05},max_jerk{8.0},max_accel{4.0},max_velocity{2.0},kp{4.0},kv{3.0}; };
class MpcSmoother {
 public:
  explicit MpcSmoother(const MpcConfig&c={}):config_(c){}
  std::vector<KinematicState> solve(const KinematicState&initial,const std::vector<KinematicState>&reference)const;
 private:MpcConfig config_;
};
}  // namespace ake_drone_sim
