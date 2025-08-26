// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/YawMode.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/msg/detail/yaw_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_YawMode_yaw_or_rate
{
public:
  explicit Init_YawMode_yaw_or_rate(::airsim_interfaces::msg::YawMode & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::YawMode yaw_or_rate(::airsim_interfaces::msg::YawMode::_yaw_or_rate_type arg)
  {
    msg_.yaw_or_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::YawMode msg_;
};

class Init_YawMode_is_rate
{
public:
  Init_YawMode_is_rate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YawMode_yaw_or_rate is_rate(::airsim_interfaces::msg::YawMode::_is_rate_type arg)
  {
    msg_.is_rate = std::move(arg);
    return Init_YawMode_yaw_or_rate(msg_);
  }

private:
  ::airsim_interfaces::msg::YawMode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::YawMode>()
{
  return airsim_interfaces::msg::builder::Init_YawMode_is_rate();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__BUILDER_HPP_
