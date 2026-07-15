#pragma once

#include "ake_drone_sim/common.hpp"
#include <vector>

namespace ake_drone_sim {
class BSpline {
 public:
  void setControlPoints(const std::vector<Eigen::Vector3d> &points, double knot_interval = 1.0);
  bool valid() const { return control_.size() >= 4; }
  double maxU() const { return valid() ? static_cast<double>(control_.size()-3) : 0.0; }
  Eigen::Vector3d position(double u) const;
  Eigen::Vector3d velocity(double u) const;
  Eigen::Vector3d acceleration(double u) const;
  const std::vector<Eigen::Vector3d> &controlPoints() const { return control_; }
 private:
  std::vector<Eigen::Vector3d> control_; double dt_{1.0};
};

class ArcLengthTable {
 public:
  void build(const BSpline &spline, int samples = 600);
  double length() const { return lengths_.empty()?0.0:lengths_.back(); }
  double uFromLength(double length) const;
  double lengthFromU(double u) const;
  double closestU(const BSpline &spline, const Eigen::Vector3d &point, double guess) const;
 private:
  std::vector<double> us_, lengths_;
};

std::vector<Eigen::Vector3d> makeSplineControlPoints(const std::vector<Eigen::Vector3d> &path,
                                                     double spacing);
}  // namespace ake_drone_sim
