// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drone_msgs:msg/TrajectorySetpoint.idl
// generated code does not contain a copyright notice

#ifndef DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__STRUCT_H_
#define DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/TrajectorySetpoint in the package drone_msgs.
typedef struct drone_msgs__msg__TrajectorySetpoint
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Vector3 acceleration;
  double yaw;
  double yaw_rate;
} drone_msgs__msg__TrajectorySetpoint;

// Struct for a sequence of drone_msgs__msg__TrajectorySetpoint.
typedef struct drone_msgs__msg__TrajectorySetpoint__Sequence
{
  drone_msgs__msg__TrajectorySetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_msgs__msg__TrajectorySetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__STRUCT_H_
