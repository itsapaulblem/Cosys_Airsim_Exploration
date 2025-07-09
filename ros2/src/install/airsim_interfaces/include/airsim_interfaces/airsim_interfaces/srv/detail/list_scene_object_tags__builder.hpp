// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/ListSceneObjectTags.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/srv/detail/list_scene_object_tags__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_ListSceneObjectTags_Request_regex_name
{
public:
  Init_ListSceneObjectTags_Request_regex_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::airsim_interfaces::srv::ListSceneObjectTags_Request regex_name(::airsim_interfaces::srv::ListSceneObjectTags_Request::_regex_name_type arg)
  {
    msg_.regex_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::ListSceneObjectTags_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::ListSceneObjectTags_Request>()
{
  return airsim_interfaces::srv::builder::Init_ListSceneObjectTags_Request_regex_name();
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_ListSceneObjectTags_Response_tags
{
public:
  explicit Init_ListSceneObjectTags_Response_tags(::airsim_interfaces::srv::ListSceneObjectTags_Response & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::ListSceneObjectTags_Response tags(::airsim_interfaces::srv::ListSceneObjectTags_Response::_tags_type arg)
  {
    msg_.tags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::ListSceneObjectTags_Response msg_;
};

class Init_ListSceneObjectTags_Response_objects
{
public:
  Init_ListSceneObjectTags_Response_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ListSceneObjectTags_Response_tags objects(::airsim_interfaces::srv::ListSceneObjectTags_Response::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return Init_ListSceneObjectTags_Response_tags(msg_);
  }

private:
  ::airsim_interfaces::srv::ListSceneObjectTags_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::ListSceneObjectTags_Response>()
{
  return airsim_interfaces::srv::builder::Init_ListSceneObjectTags_Response_objects();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__BUILDER_HPP_
