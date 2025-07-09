// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/StringArray.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__STRING_ARRAY__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__STRING_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/msg/detail/string_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_StringArray_data
{
public:
  explicit Init_StringArray_data(::airsim_interfaces::msg::StringArray & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::StringArray data(::airsim_interfaces::msg::StringArray::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::StringArray msg_;
};

class Init_StringArray_header
{
public:
  Init_StringArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StringArray_data header(::airsim_interfaces::msg::StringArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StringArray_data(msg_);
  }

private:
  ::airsim_interfaces::msg::StringArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::StringArray>()
{
  return airsim_interfaces::msg::builder::Init_StringArray_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__STRING_ARRAY__BUILDER_HPP_
