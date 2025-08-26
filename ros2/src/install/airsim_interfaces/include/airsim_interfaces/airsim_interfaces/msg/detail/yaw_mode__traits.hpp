// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/YawMode.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/msg/detail/yaw_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const YawMode & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_rate
  {
    out << "is_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.is_rate, out);
    out << ", ";
  }

  // member: yaw_or_rate
  {
    out << "yaw_or_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_or_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const YawMode & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.is_rate, out);
    out << "\n";
  }

  // member: yaw_or_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_or_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_or_rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const YawMode & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace airsim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use airsim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const airsim_interfaces::msg::YawMode & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::msg::YawMode & msg)
{
  return airsim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::msg::YawMode>()
{
  return "airsim_interfaces::msg::YawMode";
}

template<>
inline const char * name<airsim_interfaces::msg::YawMode>()
{
  return "airsim_interfaces/msg/YawMode";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::YawMode>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::YawMode>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::msg::YawMode>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__TRAITS_HPP_
