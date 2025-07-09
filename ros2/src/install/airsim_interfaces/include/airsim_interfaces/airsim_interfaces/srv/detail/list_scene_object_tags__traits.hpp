// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:srv/ListSceneObjectTags.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__TRAITS_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "airsim_interfaces/srv/detail/list_scene_object_tags__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ListSceneObjectTags_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: regex_name
  {
    out << "regex_name: ";
    rosidl_generator_traits::value_to_yaml(msg.regex_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ListSceneObjectTags_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: regex_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "regex_name: ";
    rosidl_generator_traits::value_to_yaml(msg.regex_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ListSceneObjectTags_Request & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::ListSceneObjectTags_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::ListSceneObjectTags_Request & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::ListSceneObjectTags_Request>()
{
  return "airsim_interfaces::srv::ListSceneObjectTags_Request";
}

template<>
inline const char * name<airsim_interfaces::srv::ListSceneObjectTags_Request>()
{
  return "airsim_interfaces/srv/ListSceneObjectTags_Request";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::ListSceneObjectTags_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::ListSceneObjectTags_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::srv::ListSceneObjectTags_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace airsim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ListSceneObjectTags_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: objects
  {
    if (msg.objects.size() == 0) {
      out << "objects: []";
    } else {
      out << "objects: [";
      size_t pending_items = msg.objects.size();
      for (auto item : msg.objects) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tags
  {
    if (msg.tags.size() == 0) {
      out << "tags: []";
    } else {
      out << "tags: [";
      size_t pending_items = msg.tags.size();
      for (auto item : msg.tags) {
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
  const ListSceneObjectTags_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: objects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.objects.size() == 0) {
      out << "objects: []\n";
    } else {
      out << "objects:\n";
      for (auto item : msg.objects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tags.size() == 0) {
      out << "tags: []\n";
    } else {
      out << "tags:\n";
      for (auto item : msg.tags) {
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

inline std::string to_yaml(const ListSceneObjectTags_Response & msg, bool use_flow_style = false)
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
  const airsim_interfaces::srv::ListSceneObjectTags_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  airsim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use airsim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const airsim_interfaces::srv::ListSceneObjectTags_Response & msg)
{
  return airsim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<airsim_interfaces::srv::ListSceneObjectTags_Response>()
{
  return "airsim_interfaces::srv::ListSceneObjectTags_Response";
}

template<>
inline const char * name<airsim_interfaces::srv::ListSceneObjectTags_Response>()
{
  return "airsim_interfaces/srv/ListSceneObjectTags_Response";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::ListSceneObjectTags_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::ListSceneObjectTags_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::srv::ListSceneObjectTags_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::ListSceneObjectTags>()
{
  return "airsim_interfaces::srv::ListSceneObjectTags";
}

template<>
inline const char * name<airsim_interfaces::srv::ListSceneObjectTags>()
{
  return "airsim_interfaces/srv/ListSceneObjectTags";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::ListSceneObjectTags>
  : std::integral_constant<
    bool,
    has_fixed_size<airsim_interfaces::srv::ListSceneObjectTags_Request>::value &&
    has_fixed_size<airsim_interfaces::srv::ListSceneObjectTags_Response>::value
  >
{
};

template<>
struct has_bounded_size<airsim_interfaces::srv::ListSceneObjectTags>
  : std::integral_constant<
    bool,
    has_bounded_size<airsim_interfaces::srv::ListSceneObjectTags_Request>::value &&
    has_bounded_size<airsim_interfaces::srv::ListSceneObjectTags_Response>::value
  >
{
};

template<>
struct is_service<airsim_interfaces::srv::ListSceneObjectTags>
  : std::true_type
{
};

template<>
struct is_service_request<airsim_interfaces::srv::ListSceneObjectTags_Request>
  : std::true_type
{
};

template<>
struct is_service_response<airsim_interfaces::srv::ListSceneObjectTags_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__TRAITS_HPP_
