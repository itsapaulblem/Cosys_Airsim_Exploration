// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:srv/RefreshObjectTransforms.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__REFRESH_OBJECT_TRANSFORMS__TRAITS_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__REFRESH_OBJECT_TRANSFORMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/srv/detail/refresh_object_transforms__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RefreshObjectTransforms_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RefreshObjectTransforms_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RefreshObjectTransforms_Request & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::RefreshObjectTransforms_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::RefreshObjectTransforms_Request & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::RefreshObjectTransforms_Request>()
{
  return "airsim_interfaces::srv::RefreshObjectTransforms_Request";
}

template<>
inline const char * name<airsim_interfaces::srv::RefreshObjectTransforms_Request>()
{
  return "airsim_interfaces/srv/RefreshObjectTransforms_Request";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::RefreshObjectTransforms_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::RefreshObjectTransforms_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::srv::RefreshObjectTransforms_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RefreshObjectTransforms_Response & msg,
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
  const RefreshObjectTransforms_Response & msg,
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

inline std::string to_yaml(const RefreshObjectTransforms_Response & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::RefreshObjectTransforms_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::RefreshObjectTransforms_Response & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::RefreshObjectTransforms_Response>()
{
  return "airsim_interfaces::srv::RefreshObjectTransforms_Response";
}

template<>
inline const char * name<airsim_interfaces::srv::RefreshObjectTransforms_Response>()
{
  return "airsim_interfaces/srv/RefreshObjectTransforms_Response";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::RefreshObjectTransforms_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::RefreshObjectTransforms_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::srv::RefreshObjectTransforms_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::RefreshObjectTransforms>()
{
  return "airsim_interfaces::srv::RefreshObjectTransforms";
}

template<>
inline const char * name<airsim_interfaces::srv::RefreshObjectTransforms>()
{
  return "airsim_interfaces/srv/RefreshObjectTransforms";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::RefreshObjectTransforms>
  : std::integral_constant<
    bool,
    has_fixed_size<airsim_interfaces::srv::RefreshObjectTransforms_Request>::value &&
    has_fixed_size<airsim_interfaces::srv::RefreshObjectTransforms_Response>::value
  >
{
};

template<>
struct has_bounded_size<airsim_interfaces::srv::RefreshObjectTransforms>
  : std::integral_constant<
    bool,
    has_bounded_size<airsim_interfaces::srv::RefreshObjectTransforms_Request>::value &&
    has_bounded_size<airsim_interfaces::srv::RefreshObjectTransforms_Response>::value
  >
{
};

template<>
struct is_service<airsim_interfaces::srv::RefreshObjectTransforms>
  : std::true_type
{
};

template<>
struct is_service_request<airsim_interfaces::srv::RefreshObjectTransforms_Request>
  : std::true_type
{
};

template<>
struct is_service_response<airsim_interfaces::srv::RefreshObjectTransforms_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__REFRESH_OBJECT_TRANSFORMS__TRAITS_HPP_
