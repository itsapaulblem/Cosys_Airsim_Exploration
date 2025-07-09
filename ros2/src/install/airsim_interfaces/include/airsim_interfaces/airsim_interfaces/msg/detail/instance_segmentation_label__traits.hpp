// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/InstanceSegmentationLabel.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/msg/detail/instance_segmentation_label__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const InstanceSegmentationLabel & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: index
  {
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << ", ";
  }

  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: g
  {
    out << "g: ";
    rosidl_generator_traits::value_to_yaml(msg.g, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InstanceSegmentationLabel & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << "\n";
  }

  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: g
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g: ";
    rosidl_generator_traits::value_to_yaml(msg.g, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InstanceSegmentationLabel & msg, bool use_flow_style = false)
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
  const airsim_interfaces::msg::InstanceSegmentationLabel & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::msg::InstanceSegmentationLabel & msg)
{
  return airsim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::msg::InstanceSegmentationLabel>()
{
  return "airsim_interfaces::msg::InstanceSegmentationLabel";
}

template<>
inline const char * name<airsim_interfaces::msg::InstanceSegmentationLabel>()
{
  return "airsim_interfaces/msg/InstanceSegmentationLabel";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::InstanceSegmentationLabel>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::InstanceSegmentationLabel>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::msg::InstanceSegmentationLabel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__TRAITS_HPP_
