// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/msg/detail/vel_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_VelCmd_yaw_mode
{
public:
  explicit Init_VelCmd_yaw_mode(::airsim_interfaces::msg::VelCmd & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::VelCmd yaw_mode(::airsim_interfaces::msg::VelCmd::_yaw_mode_type arg)
  {
    msg_.yaw_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::VelCmd msg_;
};

class Init_VelCmd_drivetrain
{
public:
  explicit Init_VelCmd_drivetrain(::airsim_interfaces::msg::VelCmd & msg)
  : msg_(msg)
  {}
  Init_VelCmd_yaw_mode drivetrain(::airsim_interfaces::msg::VelCmd::_drivetrain_type arg)
  {
    msg_.drivetrain = std::move(arg);
    return Init_VelCmd_yaw_mode(msg_);
  }

private:
  ::airsim_interfaces::msg::VelCmd msg_;
};

class Init_VelCmd_twist
{
public:
  Init_VelCmd_twist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelCmd_drivetrain twist(::airsim_interfaces::msg::VelCmd::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_VelCmd_drivetrain(msg_);
  }

private:
  ::airsim_interfaces::msg::VelCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::VelCmd>()
{
  return airsim_interfaces::msg::builder::Init_VelCmd_twist();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__BUILDER_HPP_
