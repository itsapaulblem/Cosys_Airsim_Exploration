// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/CarControls.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/msg/detail/car_controls__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace airsim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarControls & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: brake
  {
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << ", ";
  }

  // member: steering
  {
    out << "steering: ";
    rosidl_generator_traits::value_to_yaml(msg.steering, out);
    out << ", ";
  }

  // member: handbrake
  {
    out << "handbrake: ";
    rosidl_generator_traits::value_to_yaml(msg.handbrake, out);
    out << ", ";
  }

  // member: manual
  {
    out << "manual: ";
    rosidl_generator_traits::value_to_yaml(msg.manual, out);
    out << ", ";
  }

  // member: manual_gear
  {
    out << "manual_gear: ";
    rosidl_generator_traits::value_to_yaml(msg.manual_gear, out);
    out << ", ";
  }

  // member: gear_immediate
  {
    out << "gear_immediate: ";
    rosidl_generator_traits::value_to_yaml(msg.gear_immediate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarControls & msg,
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

  // member: throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << "\n";
  }

  // member: brake
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << "\n";
  }

  // member: steering
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering: ";
    rosidl_generator_traits::value_to_yaml(msg.steering, out);
    out << "\n";
  }

  // member: handbrake
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "handbrake: ";
    rosidl_generator_traits::value_to_yaml(msg.handbrake, out);
    out << "\n";
  }

  // member: manual
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "manual: ";
    rosidl_generator_traits::value_to_yaml(msg.manual, out);
    out << "\n";
  }

  // member: manual_gear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "manual_gear: ";
    rosidl_generator_traits::value_to_yaml(msg.manual_gear, out);
    out << "\n";
  }

  // member: gear_immediate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gear_immediate: ";
    rosidl_generator_traits::value_to_yaml(msg.gear_immediate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarControls & msg, bool use_flow_style = false)
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
  const airsim_interfaces::msg::CarControls & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::msg::CarControls & msg)
{
  return airsim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::msg::CarControls>()
{
  return "airsim_interfaces::msg::CarControls";
}

template<>
inline const char * name<airsim_interfaces::msg::CarControls>()
{
  return "airsim_interfaces/msg/CarControls";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::CarControls>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::CarControls>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<airsim_interfaces::msg::CarControls>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__TRAITS_HPP_
