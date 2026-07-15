#pragma once

#include "ake_drone_sim/common.hpp"
#include <cstdint>
#include <vector>

namespace ake_drone_sim {

class VoxelGrid {
 public:
  VoxelGrid() = default;
  VoxelGrid(const Eigen::Vector3d &origin, const Eigen::Vector3d &size, double resolution);
  void configure(const Eigen::Vector3d &origin, const Eigen::Vector3d &size, double resolution);
  void clear();
  bool valid() const { return !data_.empty(); }
  bool inside(const Eigen::Vector3i &index) const;
  bool insideWorld(const Eigen::Vector3d &point) const;
  Eigen::Vector3i worldToGrid(const Eigen::Vector3d &point) const;
  Eigen::Vector3d gridToWorld(const Eigen::Vector3i &index) const;
  bool occupied(const Eigen::Vector3i &index) const;
  bool occupiedWorld(const Eigen::Vector3d &point) const;
  void set(const Eigen::Vector3i &index, bool occupied = true);
  void addBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size);
  VoxelGrid inflated(double distance) const;
  bool segmentFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                   double step = 0.0) const;
  std::vector<Eigen::Vector3d> occupiedPoints() const;
  const Eigen::Vector3d &origin() const { return origin_; }
  const Eigen::Vector3d &size() const { return size_; }
  const Eigen::Vector3i &dimensions() const { return dims_; }
  double resolution() const { return resolution_; }

 private:
  size_t linear(const Eigen::Vector3i &index) const;
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size_{Eigen::Vector3d::Zero()};
  Eigen::Vector3i dims_{Eigen::Vector3i::Zero()};
  double resolution_{0.2};
  std::vector<uint8_t> data_;
};

}  // namespace ake_drone_sim
