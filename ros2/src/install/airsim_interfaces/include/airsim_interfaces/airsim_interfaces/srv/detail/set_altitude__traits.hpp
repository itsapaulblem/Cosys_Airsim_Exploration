// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:srv/SetAltitude.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_ALTITUDE__TRAITS_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_ALTITUDE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/srv/detail/set_altitude__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetAltitude_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: wait_on_last_task
  {
    out << "wait_on_last_task: ";
    rosidl_generator_traits::value_to_yaml(msg.wait_on_last_task, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetAltitude_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: wait_on_last_task
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wait_on_last_task: ";
    rosidl_generator_traits::value_to_yaml(msg.wait_on_last_task, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetAltitude_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace airsim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use airsim_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const airsim_interfaces::srv::SetAltitude_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::SetAltitude_Request & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::SetAltitude_Request>()
{
  return "airsim_interfaces::srv::SetAltitude_Request";
}

template<>
inline const char * name<airsim_interfaces::srv::SetAltitude_Request>()
{
  return "airsim_interfaces/srv/SetAltitude_Request";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetAltitude_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetAltitude_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::srv::SetAltitude_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetAltitude_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetAltitude_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetAltitude_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace airsim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use airsim_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const airsim_interfaces::srv::SetAltitude_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::SetAltitude_Response & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::SetAltitude_Response>()
{
  return "airsim_interfaces::srv::SetAltitude_Response";
}

template<>
inline const char * name<airsim_interfaces::srv::SetAltitude_Response>()
{
  return "airsim_interfaces/srv/SetAltitude_Response";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetAltitude_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetAltitude_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::srv::SetAltitude_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::SetAltitude>()
{
  return "airsim_interfaces::srv::SetAltitude";
}

template<>
inline const char * name<airsim_interfaces::srv::SetAltitude>()
{
  return "airsim_interfaces/srv/SetAltitude";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetAltitude>
  : std::integral_constant<
    bool,
    has_fixed_size<airsim_interfaces::srv::SetAltitude_Request>::value &&
    has_fixed_size<airsim_interfaces::srv::SetAltitude_Response>::value
  >
{
};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetAltitude>
  : std::integral_constant<
    bool,
    has_bounded_size<airsim_interfaces::srv::SetAltitude_Request>::value &&
    has_bounded_size<airsim_interfaces::srv::SetAltitude_Response>::value
  >
{
};

template<>
struct is_service<airsim_interfaces::srv::SetAltitude>
  : std::true_type
{
};

template<>
struct is_service_request<airsim_interfaces::srv::SetAltitude_Request>
  : std::true_type
{
};

template<>
struct is_service_response<airsim_interfaces::srv::SetAltitude_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_ALTITUDE__TRAITS_HPP_
