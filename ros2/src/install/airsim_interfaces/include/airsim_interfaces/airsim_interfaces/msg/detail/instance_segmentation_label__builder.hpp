// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/InstanceSegmentationLabel.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/msg/detail/instance_segmentation_label__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_InstanceSegmentationLabel_b
{
public:
  explicit Init_InstanceSegmentationLabel_b(::airsim_interfaces::msg::InstanceSegmentationLabel & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::InstanceSegmentationLabel b(::airsim_interfaces::msg::InstanceSegmentationLabel::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::InstanceSegmentationLabel msg_;
};

class Init_InstanceSegmentationLabel_g
{
public:
  explicit Init_InstanceSegmentationLabel_g(::airsim_interfaces::msg::InstanceSegmentationLabel & msg)
  : msg_(msg)
  {}
  Init_InstanceSegmentationLabel_b g(::airsim_interfaces::msg::InstanceSegmentationLabel::_g_type arg)
  {
    msg_.g = std::move(arg);
    return Init_InstanceSegmentationLabel_b(msg_);
  }

private:
  ::airsim_interfaces::msg::InstanceSegmentationLabel msg_;
};

class Init_InstanceSegmentationLabel_r
{
public:
  explicit Init_InstanceSegmentationLabel_r(::airsim_interfaces::msg::InstanceSegmentationLabel & msg)
  : msg_(msg)
  {}
  Init_InstanceSegmentationLabel_g r(::airsim_interfaces::msg::InstanceSegmentationLabel::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_InstanceSegmentationLabel_g(msg_);
  }

private:
  ::airsim_interfaces::msg::InstanceSegmentationLabel msg_;
};

class Init_InstanceSegmentationLabel_index
{
public:
  explicit Init_InstanceSegmentationLabel_index(::airsim_interfaces::msg::InstanceSegmentationLabel & msg)
  : msg_(msg)
  {}
  Init_InstanceSegmentationLabel_r index(::airsim_interfaces::msg::InstanceSegmentationLabel::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_InstanceSegmentationLabel_r(msg_);
  }

private:
  ::airsim_interfaces::msg::InstanceSegmentationLabel msg_;
};

class Init_InstanceSegmentationLabel_name
{
public:
  Init_InstanceSegmentationLabel_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InstanceSegmentationLabel_index name(::airsim_interfaces::msg::InstanceSegmentationLabel::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_InstanceSegmentationLabel_index(msg_);
  }

private:
  ::airsim_interfaces::msg::InstanceSegmentationLabel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::InstanceSegmentationLabel>()
{
  return airsim_interfaces::msg::builder::Init_InstanceSegmentationLabel_name();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__BUILDER_HPP_
