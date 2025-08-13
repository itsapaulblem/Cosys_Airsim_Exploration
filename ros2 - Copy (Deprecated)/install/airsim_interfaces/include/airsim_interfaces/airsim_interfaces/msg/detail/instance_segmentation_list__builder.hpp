// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/InstanceSegmentationList.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LIST__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/msg/detail/instance_segmentation_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_InstanceSegmentationList_labels
{
public:
  explicit Init_InstanceSegmentationList_labels(::airsim_interfaces::msg::InstanceSegmentationList & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::InstanceSegmentationList labels(::airsim_interfaces::msg::InstanceSegmentationList::_labels_type arg)
  {
    msg_.labels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::InstanceSegmentationList msg_;
};

class Init_InstanceSegmentationList_header
{
public:
  Init_InstanceSegmentationList_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InstanceSegmentationList_labels header(::airsim_interfaces::msg::InstanceSegmentationList::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_InstanceSegmentationList_labels(msg_);
  }

private:
  ::airsim_interfaces::msg::InstanceSegmentationList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::InstanceSegmentationList>()
{
  return airsim_interfaces::msg::builder::Init_InstanceSegmentationList_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LIST__BUILDER_HPP_
