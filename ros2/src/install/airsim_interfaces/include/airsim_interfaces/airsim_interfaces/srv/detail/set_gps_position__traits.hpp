// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:srv/SetGPSPosition.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__TRAITS_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/srv/detail/set_gps_position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetGPSPosition_Request & msg,
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
    out << ", ";
  }

  // member: vehicle_name
  {
    out << "vehicle_name: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetGPSPosition_Request & msg,
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

  // member: vehicle_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_name: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetGPSPosition_Request & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::SetGPSPosition_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::SetGPSPosition_Request & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::SetGPSPosition_Request>()
{
  return "airsim_interfaces::srv::SetGPSPosition_Request";
}

template<>
inline const char * name<airsim_interfaces::srv::SetGPSPosition_Request>()
{
  return "airsim_interfaces/srv/SetGPSPosition_Request";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetGPSPosition_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetGPSPosition_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::srv::SetGPSPosition_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetGPSPosition_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetGPSPosition_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetGPSPosition_Response & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::SetGPSPosition_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::SetGPSPosition_Response & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::SetGPSPosition_Response>()
{
  return "airsim_interfaces::srv::SetGPSPosition_Response";
}

template<>
inline const char * name<airsim_interfaces::srv::SetGPSPosition_Response>()
{
  return "airsim_interfaces/srv/SetGPSPosition_Response";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetGPSPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetGPSPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::srv::SetGPSPosition_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::SetGPSPosition>()
{
  return "airsim_interfaces::srv::SetGPSPosition";
}

template<>
inline const char * name<airsim_interfaces::srv::SetGPSPosition>()
{
  return "airsim_interfaces/srv/SetGPSPosition";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetGPSPosition>
  : std::integral_constant<
    bool,
    has_fixed_size<airsim_interfaces::srv::SetGPSPosition_Request>::value &&
    has_fixed_size<airsim_interfaces::srv::SetGPSPosition_Response>::value
  >
{
};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetGPSPosition>
  : std::integral_constant<
    bool,
    has_bounded_size<airsim_interfaces::srv::SetGPSPosition_Request>::value &&
    has_bounded_size<airsim_interfaces::srv::SetGPSPosition_Response>::value
  >
{
};

template<>
struct is_service<airsim_interfaces::srv::SetGPSPosition>
  : std::true_type
{
};

template<>
struct is_service_request<airsim_interfaces::srv::SetGPSPosition_Request>
  : std::true_type
{
};

template<>
struct is_service_response<airsim_interfaces::srv::SetGPSPosition_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__TRAITS_HPP_
