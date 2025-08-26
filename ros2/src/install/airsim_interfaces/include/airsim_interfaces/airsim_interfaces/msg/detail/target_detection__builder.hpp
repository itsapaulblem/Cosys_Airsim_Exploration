// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/TargetDetection.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__TARGET_DETECTION__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__TARGET_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/msg/detail/target_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TargetDetection_confidence
{
public:
  explicit Init_TargetDetection_confidence(::airsim_interfaces::msg::TargetDetection & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::TargetDetection confidence(::airsim_interfaces::msg::TargetDetection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::TargetDetection msg_;
};

class Init_TargetDetection_target_z
{
public:
  explicit Init_TargetDetection_target_z(::airsim_interfaces::msg::TargetDetection & msg)
  : msg_(msg)
  {}
  Init_TargetDetection_confidence target_z(::airsim_interfaces::msg::TargetDetection::_target_z_type arg)
  {
    msg_.target_z = std::move(arg);
    return Init_TargetDetection_confidence(msg_);
  }

private:
  ::airsim_interfaces::msg::TargetDetection msg_;
};

class Init_TargetDetection_target_y
{
public:
  explicit Init_TargetDetection_target_y(::airsim_interfaces::msg::TargetDetection & msg)
  : msg_(msg)
  {}
  Init_TargetDetection_target_z target_y(::airsim_interfaces::msg::TargetDetection::_target_y_type arg)
  {
    msg_.target_y = std::move(arg);
    return Init_TargetDetection_target_z(msg_);
  }

private:
  ::airsim_interfaces::msg::TargetDetection msg_;
};

class Init_TargetDetection_target_x
{
public:
  explicit Init_TargetDetection_target_x(::airsim_interfaces::msg::TargetDetection & msg)
  : msg_(msg)
  {}
  Init_TargetDetection_target_y target_x(::airsim_interfaces::msg::TargetDetection::_target_x_type arg)
  {
    msg_.target_x = std::move(arg);
    return Init_TargetDetection_target_y(msg_);
  }

private:
  ::airsim_interfaces::msg::TargetDetection msg_;
};

class Init_TargetDetection_vehicle_name
{
public:
  explicit Init_TargetDetection_vehicle_name(::airsim_interfaces::msg::TargetDetection & msg)
  : msg_(msg)
  {}
  Init_TargetDetection_target_x vehicle_name(::airsim_interfaces::msg::TargetDetection::_vehicle_name_type arg)
  {
    msg_.vehicle_name = std::move(arg);
    return Init_TargetDetection_target_x(msg_);
  }

private:
  ::airsim_interfaces::msg::TargetDetection msg_;
};

class Init_TargetDetection_header
{
public:
  Init_TargetDetection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetDetection_vehicle_name header(::airsim_interfaces::msg::TargetDetection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TargetDetection_vehicle_name(msg_);
  }

private:
  ::airsim_interfaces::msg::TargetDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::TargetDetection>()
{
  return airsim_interfaces::msg::builder::Init_TargetDetection_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__TARGET_DETECTION__BUILDER_HPP_
