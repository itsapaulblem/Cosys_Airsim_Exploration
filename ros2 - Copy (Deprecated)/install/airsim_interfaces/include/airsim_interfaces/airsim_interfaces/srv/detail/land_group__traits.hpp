// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:srv/LandGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__TRAITS_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/srv/detail/land_group__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LandGroup_Request & msg,
  std::ostream & out)
{
  out << "{";
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
  const LandGroup_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
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

inline std::string to_yaml(const LandGroup_Request & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::LandGroup_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::LandGroup_Request & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::LandGroup_Request>()
{
  return "airsim_interfaces::srv::LandGroup_Request";
}

template<>
inline const char * name<airsim_interfaces::srv::LandGroup_Request>()
{
  return "airsim_interfaces/srv/LandGroup_Request";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::LandGroup_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::LandGroup_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::srv::LandGroup_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LandGroup_Response & msg,
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
  const LandGroup_Response & msg,
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

inline std::string to_yaml(const LandGroup_Response & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::LandGroup_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::LandGroup_Response & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::LandGroup_Response>()
{
  return "airsim_interfaces::srv::LandGroup_Response";
}

template<>
inline const char * name<airsim_interfaces::srv::LandGroup_Response>()
{
  return "airsim_interfaces/srv/LandGroup_Response";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::LandGroup_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::LandGroup_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::srv::LandGroup_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::LandGroup>()
{
  return "airsim_interfaces::srv::LandGroup";
}

template<>
inline const char * name<airsim_interfaces::srv::LandGroup>()
{
  return "airsim_interfaces/srv/LandGroup";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::LandGroup>
  : std::integral_constant<
    bool,
    has_fixed_size<airsim_interfaces::srv::LandGroup_Request>::value &&
    has_fixed_size<airsim_interfaces::srv::LandGroup_Response>::value
  >
{
};

template<>
struct has_bounded_size<airsim_interfaces::srv::LandGroup>
  : std::integral_constant<
    bool,
    has_bounded_size<airsim_interfaces::srv::LandGroup_Request>::value &&
    has_bounded_size<airsim_interfaces::srv::LandGroup_Response>::value
  >
{
};

template<>
struct is_service<airsim_interfaces::srv::LandGroup>
  : std::true_type
{
};

template<>
struct is_service_request<airsim_interfaces::srv::LandGroup_Request>
  : std::true_type
{
};

template<>
struct is_service_response<airsim_interfaces::srv::LandGroup_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__TRAITS_HPP_
