// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from drone_msgs:msg/TrajectorySetpoint.idl
// generated code does not contain a copyright notice

#ifndef DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__STRUCT_HPP_
#define DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_msgs__msg__TrajectorySetpoint __attribute__((deprecated))
#else
# define DEPRECATED__drone_msgs__msg__TrajectorySetpoint __declspec(deprecated)
#endif

namespace drone_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectorySetpoint_
{
  using Type = TrajectorySetpoint_<ContainerAllocator>;

  explicit TrajectorySetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init),
    acceleration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->yaw_rate = 0.0;
    }
  }

  explicit TrajectorySetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init),
    acceleration(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->yaw_rate = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _yaw_rate_type =
    double;
  _yaw_rate_type yaw_rate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const double & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_msgs__msg__TrajectorySetpoint
    std::shared_ptr<drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_msgs__msg__TrajectorySetpoint
    std::shared_ptr<drone_msgs::msg::TrajectorySetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectorySetpoint_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectorySetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectorySetpoint_

// alias to use template instance with default allocator
using TrajectorySetpoint =
  drone_msgs::msg::TrajectorySetpoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace drone_msgs

#endif  // DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__STRUCT_HPP_
