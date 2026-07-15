#pragma once

#include "ake_drone_sim/voxel_grid.hpp"
#include <string>

namespace ake_drone_sim {
class AStar3D {
 public:
  struct Result { bool success{false}; std::string reason; size_t expanded{0}; std::vector<Eigen::Vector3d> path; };
  explicit AStar3D(size_t max_nodes = 250000, double heuristic_weight = 1.05)
      : max_nodes_(max_nodes), weight_(heuristic_weight) {}
  Result plan(const VoxelGrid &grid, const Eigen::Vector3d &start, const Eigen::Vector3d &goal) const;
  static std::vector<Eigen::Vector3d> simplify(const VoxelGrid &grid,
                                               const std::vector<Eigen::Vector3d> &path);
 private:
  size_t max_nodes_; double weight_;
};
}  // namespace ake_drone_sim
