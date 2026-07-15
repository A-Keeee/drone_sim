// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drone_msgs:msg/TrajectorySetpoint.idl
// generated code does not contain a copyright notice

#ifndef DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__BUILDER_HPP_
#define DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drone_msgs/msg/detail/trajectory_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drone_msgs
{

namespace msg
{

namespace builder
{

class Init_TrajectorySetpoint_yaw_rate
{
public:
  explicit Init_TrajectorySetpoint_yaw_rate(::drone_msgs::msg::TrajectorySetpoint & msg)
  : msg_(msg)
  {}
  ::drone_msgs::msg::TrajectorySetpoint yaw_rate(::drone_msgs::msg::TrajectorySetpoint::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_msgs::msg::TrajectorySetpoint msg_;
};

class Init_TrajectorySetpoint_yaw
{
public:
  explicit Init_TrajectorySetpoint_yaw(::drone_msgs::msg::TrajectorySetpoint & msg)
  : msg_(msg)
  {}
  Init_TrajectorySetpoint_yaw_rate yaw(::drone_msgs::msg::TrajectorySetpoint::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_TrajectorySetpoint_yaw_rate(msg_);
  }

private:
  ::drone_msgs::msg::TrajectorySetpoint msg_;
};

class Init_TrajectorySetpoint_acceleration
{
public:
  explicit Init_TrajectorySetpoint_acceleration(::drone_msgs::msg::TrajectorySetpoint & msg)
  : msg_(msg)
  {}
  Init_TrajectorySetpoint_yaw acceleration(::drone_msgs::msg::TrajectorySetpoint::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_TrajectorySetpoint_yaw(msg_);
  }

private:
  ::drone_msgs::msg::TrajectorySetpoint msg_;
};

class Init_TrajectorySetpoint_velocity
{
public:
  explicit Init_TrajectorySetpoint_velocity(::drone_msgs::msg::TrajectorySetpoint & msg)
  : msg_(msg)
  {}
  Init_TrajectorySetpoint_acceleration velocity(::drone_msgs::msg::TrajectorySetpoint::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_TrajectorySetpoint_acceleration(msg_);
  }

private:
  ::drone_msgs::msg::TrajectorySetpoint msg_;
};

class Init_TrajectorySetpoint_position
{
public:
  explicit Init_TrajectorySetpoint_position(::drone_msgs::msg::TrajectorySetpoint & msg)
  : msg_(msg)
  {}
  Init_TrajectorySetpoint_velocity position(::drone_msgs::msg::TrajectorySetpoint::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_TrajectorySetpoint_velocity(msg_);
  }

private:
  ::drone_msgs::msg::TrajectorySetpoint msg_;
};

class Init_TrajectorySetpoint_header
{
public:
  Init_TrajectorySetpoint_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectorySetpoint_position header(::drone_msgs::msg::TrajectorySetpoint::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrajectorySetpoint_position(msg_);
  }

private:
  ::drone_msgs::msg::TrajectorySetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_msgs::msg::TrajectorySetpoint>()
{
  return drone_msgs::msg::builder::Init_TrajectorySetpoint_header();
}

}  // namespace drone_msgs

#endif  // DRONE_MSGS__MSG__DETAIL__TRAJECTORY_SETPOINT__BUILDER_HPP_
