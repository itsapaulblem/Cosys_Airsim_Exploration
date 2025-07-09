// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/VelCmdGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/msg/detail/vel_cmd_group__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'vel_cmd'
#include "airsim_interfaces/msg/detail/vel_cmd__traits.hpp"

namespace airsim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const VelCmdGroup & msg,
  std::ostream & out)
{
  out << "{";
  // member: vel_cmd
  {
    out << "vel_cmd: ";
    to_flow_style_yaml(msg.vel_cmd, out);
    out << ", ";
  }

  // member: vehicle_names
  {
    if (msg.vehicle_names.size() == 0) {
      out << "vehicle_names: []";
    } else {
      out << "vehicle_names: [";
      size_t pending_items = msg.vehicle_names.size();
      for (auto item : msg.vehicle_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VelCmdGroup & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vel_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_cmd:\n";
    to_block_style_yaml(msg.vel_cmd, out, indentation + 2);
  }

  // member: vehicle_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vehicle_names.size() == 0) {
      out << "vehicle_names: []\n";
    } else {
      out << "vehicle_names:\n";
      for (auto item : msg.vehicle_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VelCmdGroup & msg, bool use_flow_style = false)
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
  const airsim_interfaces::msg::VelCmdGroup & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::msg::VelCmdGroup & msg)
{
  return airsim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::msg::VelCmdGroup>()
{
  return "airsim_interfaces::msg::VelCmdGroup";
}

template<>
inline const char * name<airsim_interfaces::msg::VelCmdGroup>()
{
  return "airsim_interfaces/msg/VelCmdGroup";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::VelCmdGroup>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::VelCmdGroup>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::msg::VelCmdGroup>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__TRAITS_HPP_
