#include "ake_drone_sim/voxel_grid.hpp"
#include "ake_drone_sim/pointcloud_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

using namespace std::chrono_literals;

namespace ake_drone_sim {
class VoxelMapNode final : public rclcpp::Node {
 public:
  VoxelMapNode() : Node("voxel_map_node") {
    const double resolution = declare_parameter("resolution", 0.2);
    const auto origin = vectorParameter("origin", {-2.0, -4.0, 0.0});
    const auto size = vectorParameter("size", {12.0, 8.0, 5.0});
    grid_.configure({origin[0], origin[1], origin[2]},
                    {size[0], size[1], size[2]}, resolution);

    const bool use_configured = declare_parameter("use_configured_obstacles", true);
    const auto configured = declare_parameter<std::vector<double>>(
        "obstacles",
        {1.5, 0.0, 1.2, 0.6, 2.0, 2.4,
         3.0, -1.2, 2.0, 0.8, 1.6, 4.0,
         4.0, 1.2, 2.0, 0.8, 1.6, 4.0,
         5.4, 0.0, 3.7, 0.8, 3.5, 1.0,
         6.8, 0.0, 1.3, 0.8, 4.5, 2.6});
    if (use_configured) {
      if (configured.size() % 6 != 0) {
        throw std::runtime_error("obstacles must contain center xyz + size xyz tuples");
      }
      boxes_ = configured;
    }

    const int requested_random = static_cast<int>(std::max<int64_t>(
        0, declare_parameter("random_obstacle_count", 0)));
    const int random_seed = static_cast<int>(declare_parameter("random_seed", 42));
    if (requested_random > 0) {
      generateRandomObstacles(requested_random, random_seed, origin, size);
    }

    for (size_t i = 0; i + 5 < boxes_.size(); i += 6) {
      grid_.addBox({boxes_[i], boxes_[i + 1], boxes_[i + 2]},
                   {boxes_[i + 3], boxes_[i + 4], boxes_[i + 5]});
    }

    RCLCPP_INFO(get_logger(),
                "voxel map: %zu boxes, %zu occupied voxels, resolution %.2f m",
                boxes_.size() / 6, grid_.occupiedPoints().size(), resolution);
    if (requested_random > 0) {
      RCLCPP_INFO(get_logger(), "random scene seed: %d", random_seed);
    }

    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map/obstacles", qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/map/obstacle_markers", qos);
    timer_ = create_wall_timer(1s, std::bind(&VoxelMapNode::publish, this));
    publish();
  }

 private:
  std::vector<double> vectorParameter(const std::string &name,
                                      const std::vector<double> &defaults) {
    auto value = declare_parameter<std::vector<double>>(name, defaults);
    if (value.size() != 3) {
      throw std::runtime_error(name + " must contain exactly three values");
    }
    return value;
  }

  // Distance outside an axis-aligned box. It is zero for a point inside it.
  static double clearanceToBox(const Eigen::Vector3d &point,
                               const Eigen::Vector3d &center,
                               const Eigen::Vector3d &size) {
    return ((point - center).cwiseAbs() - size * 0.5)
        .cwiseMax(Eigen::Vector3d::Zero())
        .norm();
  }

  bool overlapsExisting(const Eigen::Vector3d &center,
                        const Eigen::Vector3d &size, double margin) const {
    for (size_t i = 0; i + 5 < boxes_.size(); i += 6) {
      const Eigen::Vector3d other_center{boxes_[i], boxes_[i + 1], boxes_[i + 2]};
      const Eigen::Vector3d other_size{boxes_[i + 3], boxes_[i + 4], boxes_[i + 5]};
      const Eigen::Array3d separation = (center - other_center).cwiseAbs().array();
      const Eigen::Array3d required = 0.5 * (size + other_size).array() + margin;
      if ((separation < required).all()) return true;
    }
    return false;
  }

  void generateRandomObstacles(int count, int seed,
                               const std::vector<double> &origin,
                               const std::vector<double> &map_size) {
    auto center_min = vectorParameter("random_center_min", {1.0, -3.0, 0.7});
    auto center_max = vectorParameter("random_center_max", {7.3, 3.0, 3.7});
    auto size_min = vectorParameter("random_size_min", {0.45, 0.45, 0.8});
    auto size_max = vectorParameter("random_size_max", {0.9, 1.4, 2.5});
    const auto start_values = vectorParameter("random_start", {0.0, 0.0, 1.5});
    const auto goal_values = vectorParameter("random_goal", {8.0, 0.0, 1.5});
    const double endpoint_clearance = declare_parameter("random_endpoint_clearance", 1.0);
    const double corridor_half_width = declare_parameter("random_corridor_half_width", 0.8);
    const double corridor_fraction = std::clamp(
        declare_parameter("random_corridor_fraction", 0.55), 0.0, 1.0);

    const Eigen::Vector3d map_min{origin[0], origin[1], origin[2]};
    const Eigen::Vector3d map_max = map_min + Eigen::Vector3d{
        map_size[0], map_size[1], map_size[2]};
    const Eigen::Vector3d start{start_values[0], start_values[1], start_values[2]};
    const Eigen::Vector3d goal{goal_values[0], goal_values[1], goal_values[2]};

    for (int axis = 0; axis < 3; ++axis) {
      if (center_min[axis] > center_max[axis] || size_min[axis] <= 0.0 ||
          size_min[axis] > size_max[axis]) {
        throw std::runtime_error("invalid random obstacle bounds");
      }
    }

    std::mt19937 generator(static_cast<std::mt19937::result_type>(seed));
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> signed_unit(-1.0, 1.0);
    const int corridor_count = static_cast<int>(std::ceil(count * corridor_fraction));
    int generated = 0;
    const int max_attempts = std::max(200, count * 250);

    for (int attempt = 0; attempt < max_attempts && generated < count; ++attempt) {
      Eigen::Vector3d obstacle_size;
      Eigen::Vector3d center;
      for (int axis = 0; axis < 3; ++axis) {
        obstacle_size[axis] = size_min[axis] +
            unit(generator) * (size_max[axis] - size_min[axis]);
        center[axis] = center_min[axis] +
            unit(generator) * (center_max[axis] - center_min[axis]);
      }

      // Bias part of the random set around the direct start-goal segment. These
      // boxes make an A* detour visible while the remaining boxes add 3-D variety.
      if (generated < corridor_count) {
        const double t = 0.12 + 0.76 * unit(generator);
        const Eigen::Vector3d on_segment = start + t * (goal - start);
        center.x() = on_segment.x();
        center.y() = on_segment.y() + corridor_half_width * signed_unit(generator);
        center.z() = on_segment.z() + 0.55 * signed_unit(generator);
      }

      for (int axis = 0; axis < 3; ++axis) {
        const double lower = std::max(center_min[axis], map_min[axis] + 0.5 * obstacle_size[axis]);
        const double upper = std::min(center_max[axis], map_max[axis] - 0.5 * obstacle_size[axis]);
        center[axis] = std::clamp(center[axis], lower, upper);
      }

      if (clearanceToBox(start, center, obstacle_size) < endpoint_clearance ||
          clearanceToBox(goal, center, obstacle_size) < endpoint_clearance ||
          overlapsExisting(center, obstacle_size, 0.12)) {
        continue;
      }

      boxes_.insert(boxes_.end(), {center.x(), center.y(), center.z(),
                                   obstacle_size.x(), obstacle_size.y(), obstacle_size.z()});
      ++generated;
    }

    if (generated != count) {
      RCLCPP_WARN(get_logger(), "requested %d random obstacles but generated %d", count,
                  generated);
    }
  }

  void publish() {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "map";
    cloud_pub_->publish(makeCloud(header, grid_.occupiedPoints()));

    visualization_msgs::msg::MarkerArray array;
    for (size_t i = 0, id = 0; i + 5 < boxes_.size(); i += 6, ++id) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "obstacles";
      marker.id = static_cast<int>(id);
      marker.type = marker.CUBE;
      marker.action = marker.ADD;
      marker.pose.position = pointMsg({boxes_[i], boxes_[i + 1], boxes_[i + 2]});
      marker.pose.orientation.w = 1.0;
      marker.scale = vectorMsg({boxes_[i + 3], boxes_[i + 4], boxes_[i + 5]});
      marker.color.r = 0.72f + 0.12f * static_cast<float>(id % 3) / 2.0f;
      marker.color.g = 0.18f + 0.12f * static_cast<float>((id + 1) % 3) / 2.0f;
      marker.color.b = 0.12f;
      marker.color.a = 0.68f;
      array.markers.push_back(marker);
    }
    marker_pub_->publish(array);
  }

  VoxelGrid grid_;
  std::vector<double> boxes_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace ake_drone_sim

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ake_drone_sim::VoxelMapNode>());
  rclcpp::shutdown();
}
