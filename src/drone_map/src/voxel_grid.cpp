#include "ake_drone_sim/voxel_grid.hpp"

namespace ake_drone_sim {

VoxelGrid::VoxelGrid(const Eigen::Vector3d &o, const Eigen::Vector3d &s, double r) { configure(o, s, r); }
void VoxelGrid::configure(const Eigen::Vector3d &o, const Eigen::Vector3d &s, double r) {
  origin_ = o; size_ = s; resolution_ = std::max(0.02, r);
  dims_ = (size_ / resolution_).array().ceil().cast<int>();
  data_.assign(static_cast<size_t>(dims_.x()) * dims_.y() * dims_.z(), 0);
}
void VoxelGrid::clear() { std::fill(data_.begin(), data_.end(), 0); }
bool VoxelGrid::inside(const Eigen::Vector3i &i) const {
  return (i.array() >= 0).all() && (i.array() < dims_.array()).all();
}
bool VoxelGrid::insideWorld(const Eigen::Vector3d &p) const { return inside(worldToGrid(p)); }
Eigen::Vector3i VoxelGrid::worldToGrid(const Eigen::Vector3d &p) const {
  return ((p - origin_) / resolution_).array().floor().cast<int>();
}
Eigen::Vector3d VoxelGrid::gridToWorld(const Eigen::Vector3i &i) const {
  return origin_ + (i.cast<double>().array() + 0.5).matrix() * resolution_;
}
size_t VoxelGrid::linear(const Eigen::Vector3i &i) const {
  return (static_cast<size_t>(i.x()) * dims_.y() + i.y()) * dims_.z() + i.z();
}
bool VoxelGrid::occupied(const Eigen::Vector3i &i) const { return !inside(i) || data_[linear(i)] != 0; }
bool VoxelGrid::occupiedWorld(const Eigen::Vector3d &p) const { return occupied(worldToGrid(p)); }
void VoxelGrid::set(const Eigen::Vector3i &i, bool value) { if (inside(i)) data_[linear(i)] = value; }
void VoxelGrid::addBox(const Eigen::Vector3d &c, const Eigen::Vector3d &s) {
  Eigen::Vector3i lo = worldToGrid(c - 0.5 * s), hi = worldToGrid(c + 0.5 * s);
  lo = lo.cwiseMax(Eigen::Vector3i::Zero()); hi = hi.cwiseMin(dims_ - Eigen::Vector3i::Ones());
  for (int x = lo.x(); x <= hi.x(); ++x) for (int y = lo.y(); y <= hi.y(); ++y)
    for (int z = lo.z(); z <= hi.z(); ++z) set({x, y, z});
}
VoxelGrid VoxelGrid::inflated(double distance) const {
  VoxelGrid out(origin_, size_, resolution_);
  const int n = static_cast<int>(std::ceil(std::max(0.0, distance) / resolution_));
  for (int x = 0; x < dims_.x(); ++x) for (int y = 0; y < dims_.y(); ++y)
    for (int z = 0; z < dims_.z(); ++z) if (data_[linear({x,y,z})])
      for (int dx=-n; dx<=n; ++dx) for (int dy=-n; dy<=n; ++dy) for (int dz=-n; dz<=n; ++dz)
        if (dx*dx+dy*dy+dz*dz <= n*n) out.set({x+dx,y+dy,z+dz});
  return out;
}
bool VoxelGrid::segmentFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double step) const {
  const double length = (b-a).norm(); step = step > 0.0 ? step : 0.45 * resolution_;
  const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int i=0; i<=samples; ++i) if (occupiedWorld(a + (b-a)*(static_cast<double>(i)/samples))) return false;
  return true;
}
std::vector<Eigen::Vector3d> VoxelGrid::occupiedPoints() const {
  std::vector<Eigen::Vector3d> out;
  for (int x=0;x<dims_.x();++x) for(int y=0;y<dims_.y();++y) for(int z=0;z<dims_.z();++z)
    if(data_[linear({x,y,z})]) out.push_back(gridToWorld({x,y,z}));
  return out;
}

}  // namespace ake_drone_sim
