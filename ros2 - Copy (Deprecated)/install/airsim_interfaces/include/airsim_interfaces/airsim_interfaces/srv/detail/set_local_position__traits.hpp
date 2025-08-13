// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:srv/SetLocalPosition.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__TRAITS_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/srv/detail/set_local_position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLocalPosition_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
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
  const SetLocalPosition_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
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

inline std::string to_yaml(const SetLocalPosition_Request & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::SetLocalPosition_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::SetLocalPosition_Request & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::SetLocalPosition_Request>()
{
  return "airsim_interfaces::srv::SetLocalPosition_Request";
}

template<>
inline const char * name<airsim_interfaces::srv::SetLocalPosition_Request>()
{
  return "airsim_interfaces/srv/SetLocalPosition_Request";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetLocalPosition_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetLocalPosition_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::srv::SetLocalPosition_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLocalPosition_Response & msg,
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
  const SetLocalPosition_Response & msg,
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

inline std::string to_yaml(const SetLocalPosition_Response & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::SetLocalPosition_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::SetLocalPosition_Response & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::SetLocalPosition_Response>()
{
  return "airsim_interfaces::srv::SetLocalPosition_Response";
}

template<>
inline const char * name<airsim_interfaces::srv::SetLocalPosition_Response>()
{
  return "airsim_interfaces/srv/SetLocalPosition_Response";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetLocalPosition_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetLocalPosition_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::srv::SetLocalPosition_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::SetLocalPosition>()
{
  return "airsim_interfaces::srv::SetLocalPosition";
}

template<>
inline const char * name<airsim_interfaces::srv::SetLocalPosition>()
{
  return "airsim_interfaces/srv/SetLocalPosition";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::SetLocalPosition>
  : std::integral_constant<
    bool,
    has_fixed_size<airsim_interfaces::srv::SetLocalPosition_Request>::value &&
    has_fixed_size<airsim_interfaces::srv::SetLocalPosition_Response>::value
  >
{
};

template<>
struct has_bounded_size<airsim_interfaces::srv::SetLocalPosition>
  : std::integral_constant<
    bool,
    has_bounded_size<airsim_interfaces::srv::SetLocalPosition_Request>::value &&
    has_bounded_size<airsim_interfaces::srv::SetLocalPosition_Response>::value
  >
{
};

template<>
struct is_service<airsim_interfaces::srv::SetLocalPosition>
  : std::true_type
{
};

template<>
struct is_service_request<airsim_interfaces::srv::SetLocalPosition_Request>
  : std::true_type
{
};

template<>
struct is_service_response<airsim_interfaces::srv::SetLocalPosition_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__TRAITS_HPP_
