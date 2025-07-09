// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/GimbalAngleQuatCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/msg/detail/gimbal_angle_quat_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace airsim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalAngleQuatCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: camera_name
  {
    out << "camera_name: ";
    rosidl_generator_traits::value_to_yaml(msg.camera_name, out);
    out << ", ";
  }

  // member: vehicle_name
  {
    out << "vehicle_name: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_name, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    to_flow_style_yaml(msg.orientation, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalAngleQuatCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: camera_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_name: ";
    rosidl_generator_traits::value_to_yaml(msg.camera_name, out);
    out << "\n";
  }

  // member: vehicle_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_name: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_name, out);
    out << "\n";
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_block_style_yaml(msg.orientation, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalAngleQuatCmd & msg, bool use_flow_style = false)
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
  const airsim_interfaces::msg::GimbalAngleQuatCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::msg::GimbalAngleQuatCmd & msg)
{
  return airsim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::msg::GimbalAngleQuatCmd>()
{
  return "airsim_interfaces::msg::GimbalAngleQuatCmd";
}

template<>
inline const char * name<airsim_interfaces::msg::GimbalAngleQuatCmd>()
{
  return "airsim_interfaces/msg/GimbalAngleQuatCmd";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::GimbalAngleQuatCmd>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::GimbalAngleQuatCmd>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::msg::GimbalAngleQuatCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__TRAITS_HPP_
