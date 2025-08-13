// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/ObjectTransformsList.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__OBJECT_TRANSFORMS_LIST__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__OBJECT_TRANSFORMS_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/msg/detail/object_transforms_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectTransformsList_objects
{
public:
  explicit Init_ObjectTransformsList_objects(::airsim_interfaces::msg::ObjectTransformsList & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::ObjectTransformsList objects(::airsim_interfaces::msg::ObjectTransformsList::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::ObjectTransformsList msg_;
};

class Init_ObjectTransformsList_header
{
public:
  Init_ObjectTransformsList_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectTransformsList_objects header(::airsim_interfaces::msg::ObjectTransformsList::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectTransformsList_objects(msg_);
  }

private:
  ::airsim_interfaces::msg::ObjectTransformsList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::ObjectTransformsList>()
{
  return airsim_interfaces::msg::builder::Init_ObjectTransformsList_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__OBJECT_TRANSFORMS_LIST__BUILDER_HPP_
