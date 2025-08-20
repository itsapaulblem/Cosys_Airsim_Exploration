// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/Environment.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/msg/detail/environment__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
// Member 'gravity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'geo_point'
#include "geographic_msgs/msg/detail/geo_point__traits.hpp"

namespace airsim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Environment & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: geo_point
  {
    out << "geo_point: ";
    to_flow_style_yaml(msg.geo_point, out);
    out << ", ";
  }

  // member: gravity
  {
    out << "gravity: ";
    to_flow_style_yaml(msg.gravity, out);
    out << ", ";
  }

  // member: air_pressure
  {
    out << "air_pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.air_pressure, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: air_density
  {
    out << "air_density: ";
    rosidl_generator_traits::value_to_yaml(msg.air_density, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Environment & msg,
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

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: geo_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geo_point:\n";
    to_block_style_yaml(msg.geo_point, out, indentation + 2);
  }

  // member: gravity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gravity:\n";
    to_block_style_yaml(msg.gravity, out, indentation + 2);
  }

  // member: air_pressure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "air_pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.air_pressure, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: air_density
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "air_density: ";
    rosidl_generator_traits::value_to_yaml(msg.air_density, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Environment & msg, bool use_flow_style = false)
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
  const airsim_interfaces::msg::Environment & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::msg::Environment & msg)
{
  return airsim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::msg::Environment>()
{
  return "airsim_interfaces::msg::Environment";
}

template<>
inline const char * name<airsim_interfaces::msg::Environment>()
{
  return "airsim_interfaces/msg/Environment";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::Environment>
  : std::integral_constant<bool, has_fixed_size<geographic_msgs::msg::GeoPoint>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::Environment>
  : std::integral_constant<bool, has_bounded_size<geographic_msgs::msg::GeoPoint>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<airsim_interfaces::msg::Environment>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__TRAITS_HPP_
