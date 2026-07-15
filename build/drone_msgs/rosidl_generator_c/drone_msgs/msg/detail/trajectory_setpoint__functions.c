// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from drone_msgs:msg/TrajectorySetpoint.idl
// generated code does not contain a copyright notice
#include "drone_msgs/msg/detail/trajectory_setpoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
drone_msgs__msg__TrajectorySetpoint__init(drone_msgs__msg__TrajectorySetpoint * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    drone_msgs__msg__TrajectorySetpoint__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    drone_msgs__msg__TrajectorySetpoint__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    drone_msgs__msg__TrajectorySetpoint__fini(msg);
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration)) {
    drone_msgs__msg__TrajectorySetpoint__fini(msg);
    return false;
  }
  // yaw
  // yaw_rate
  return true;
}

void
drone_msgs__msg__TrajectorySetpoint__fini(drone_msgs__msg__TrajectorySetpoint * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // acceleration
  geometry_msgs__msg__Vector3__fini(&msg->acceleration);
  // yaw
  // yaw_rate
}

bool
drone_msgs__msg__TrajectorySetpoint__are_equal(const drone_msgs__msg__TrajectorySetpoint * lhs, const drone_msgs__msg__TrajectorySetpoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // yaw_rate
  if (lhs->yaw_rate != rhs->yaw_rate) {
    return false;
  }
  return true;
}

bool
drone_msgs__msg__TrajectorySetpoint__copy(
  const drone_msgs__msg__TrajectorySetpoint * input,
  drone_msgs__msg__TrajectorySetpoint * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // yaw_rate
  output->yaw_rate = input->yaw_rate;
  return true;
}

drone_msgs__msg__TrajectorySetpoint *
drone_msgs__msg__TrajectorySetpoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_msgs__msg__TrajectorySetpoint * msg = (drone_msgs__msg__TrajectorySetpoint *)allocator.allocate(sizeof(drone_msgs__msg__TrajectorySetpoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(drone_msgs__msg__TrajectorySetpoint));
  bool success = drone_msgs__msg__TrajectorySetpoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
drone_msgs__msg__TrajectorySetpoint__destroy(drone_msgs__msg__TrajectorySetpoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    drone_msgs__msg__TrajectorySetpoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
drone_msgs__msg__TrajectorySetpoint__Sequence__init(drone_msgs__msg__TrajectorySetpoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_msgs__msg__TrajectorySetpoint * data = NULL;

  if (size) {
    data = (drone_msgs__msg__TrajectorySetpoint *)allocator.zero_allocate(size, sizeof(drone_msgs__msg__TrajectorySetpoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = drone_msgs__msg__TrajectorySetpoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        drone_msgs__msg__TrajectorySetpoint__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
drone_msgs__msg__TrajectorySetpoint__Sequence__fini(drone_msgs__msg__TrajectorySetpoint__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      drone_msgs__msg__TrajectorySetpoint__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

drone_msgs__msg__TrajectorySetpoint__Sequence *
drone_msgs__msg__TrajectorySetpoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_msgs__msg__TrajectorySetpoint__Sequence * array = (drone_msgs__msg__TrajectorySetpoint__Sequence *)allocator.allocate(sizeof(drone_msgs__msg__TrajectorySetpoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = drone_msgs__msg__TrajectorySetpoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
drone_msgs__msg__TrajectorySetpoint__Sequence__destroy(drone_msgs__msg__TrajectorySetpoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    drone_msgs__msg__TrajectorySetpoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
drone_msgs__msg__TrajectorySetpoint__Sequence__are_equal(const drone_msgs__msg__TrajectorySetpoint__Sequence * lhs, const drone_msgs__msg__TrajectorySetpoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!drone_msgs__msg__TrajectorySetpoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
drone_msgs__msg__TrajectorySetpoint__Sequence__copy(
  const drone_msgs__msg__TrajectorySetpoint__Sequence * input,
  drone_msgs__msg__TrajectorySetpoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(drone_msgs__msg__TrajectorySetpoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    drone_msgs__msg__TrajectorySetpoint * data =
      (drone_msgs__msg__TrajectorySetpoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!drone_msgs__msg__TrajectorySetpoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          drone_msgs__msg__TrajectorySetpoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!drone_msgs__msg__TrajectorySetpoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
