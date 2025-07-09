// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/GPSYaw.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/msg/detail/gps_yaw__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GPSYaw & msg,
  std::ostream & out)
{
  out << "{";
  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GPSYaw & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GPSYaw & msg, bool use_flow_style = false)
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
  const airsim_interfaces::msg::GPSYaw & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::msg::GPSYaw & msg)
{
  return airsim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::msg::GPSYaw>()
{
  return "airsim_interfaces::msg::GPSYaw";
}

template<>
inline const char * name<airsim_interfaces::msg::GPSYaw>()
{
  return "airsim_interfaces/msg/GPSYaw";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::GPSYaw>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::GPSYaw>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::msg::GPSYaw>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__TRAITS_HPP_
