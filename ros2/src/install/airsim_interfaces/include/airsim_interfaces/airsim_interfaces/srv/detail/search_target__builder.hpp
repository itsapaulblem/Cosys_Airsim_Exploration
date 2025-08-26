// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/SearchTarget.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SEARCH_TARGET__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SEARCH_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/srv/detail/search_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::SearchTarget_Request>()
{
  return ::airsim_interfaces::srv::SearchTarget_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_SearchTarget_Response_message
{
public:
  explicit Init_SearchTarget_Response_message(::airsim_interfaces::srv::SearchTarget_Response & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::SearchTarget_Response message(::airsim_interfaces::srv::SearchTarget_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::SearchTarget_Response msg_;
};

class Init_SearchTarget_Response_confidence
{
public:
  explicit Init_SearchTarget_Response_confidence(::airsim_interfaces::srv::SearchTarget_Response & msg)
  : msg_(msg)
  {}
  Init_SearchTarget_Response_message confidence(::airsim_interfaces::srv::SearchTarget_Response::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_SearchTarget_Response_message(msg_);
  }

private:
  ::airsim_interfaces::srv::SearchTarget_Response msg_;
};

class Init_SearchTarget_Response_target_z
{
public:
  explicit Init_SearchTarget_Response_target_z(::airsim_interfaces::srv::SearchTarget_Response & msg)
  : msg_(msg)
  {}
  Init_SearchTarget_Response_confidence target_z(::airsim_interfaces::srv::SearchTarget_Response::_target_z_type arg)
  {
    msg_.target_z = std::move(arg);
    return Init_SearchTarget_Response_confidence(msg_);
  }

private:
  ::airsim_interfaces::srv::SearchTarget_Response msg_;
};

class Init_SearchTarget_Response_target_y
{
public:
  explicit Init_SearchTarget_Response_target_y(::airsim_interfaces::srv::SearchTarget_Response & msg)
  : msg_(msg)
  {}
  Init_SearchTarget_Response_target_z target_y(::airsim_interfaces::srv::SearchTarget_Response::_target_y_type arg)
  {
    msg_.target_y = std::move(arg);
    return Init_SearchTarget_Response_target_z(msg_);
  }

private:
  ::airsim_interfaces::srv::SearchTarget_Response msg_;
};

class Init_SearchTarget_Response_target_x
{
public:
  explicit Init_SearchTarget_Response_target_x(::airsim_interfaces::srv::SearchTarget_Response & msg)
  : msg_(msg)
  {}
  Init_SearchTarget_Response_target_y target_x(::airsim_interfaces::srv::SearchTarget_Response::_target_x_type arg)
  {
    msg_.target_x = std::move(arg);
    return Init_SearchTarget_Response_target_y(msg_);
  }

private:
  ::airsim_interfaces::srv::SearchTarget_Response msg_;
};

class Init_SearchTarget_Response_success
{
public:
  Init_SearchTarget_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchTarget_Response_target_x success(::airsim_interfaces::srv::SearchTarget_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SearchTarget_Response_target_x(msg_);
  }

private:
  ::airsim_interfaces::srv::SearchTarget_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::SearchTarget_Response>()
{
  return airsim_interfaces::srv::builder::Init_SearchTarget_Response_success();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SEARCH_TARGET__BUILDER_HPP_
