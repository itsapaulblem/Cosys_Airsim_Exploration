// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/ComputerVisionState.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__COMPUTER_VISION_STATE__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__COMPUTER_VISION_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/msg/detail/computer_vision_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_ComputerVisionState_twist
{
public:
  explicit Init_ComputerVisionState_twist(::airsim_interfaces::msg::ComputerVisionState & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::ComputerVisionState twist(::airsim_interfaces::msg::ComputerVisionState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::ComputerVisionState msg_;
};

class Init_ComputerVisionState_pose
{
public:
  explicit Init_ComputerVisionState_pose(::airsim_interfaces::msg::ComputerVisionState & msg)
  : msg_(msg)
  {}
  Init_ComputerVisionState_twist pose(::airsim_interfaces::msg::ComputerVisionState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ComputerVisionState_twist(msg_);
  }

private:
  ::airsim_interfaces::msg::ComputerVisionState msg_;
};

class Init_ComputerVisionState_header
{
public:
  Init_ComputerVisionState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputerVisionState_pose header(::airsim_interfaces::msg::ComputerVisionState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ComputerVisionState_pose(msg_);
  }

private:
  ::airsim_interfaces::msg::ComputerVisionState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::ComputerVisionState>()
{
  return airsim_interfaces::msg::builder::Init_ComputerVisionState_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__COMPUTER_VISION_STATE__BUILDER_HPP_
