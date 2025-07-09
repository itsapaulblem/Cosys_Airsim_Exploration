// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/SetAltitude.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_ALTITUDE__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_ALTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/srv/detail/set_altitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetAltitude_Request_wait_on_last_task
{
public:
  explicit Init_SetAltitude_Request_wait_on_last_task(::airsim_interfaces::srv::SetAltitude_Request & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::SetAltitude_Request wait_on_last_task(::airsim_interfaces::srv::SetAltitude_Request::_wait_on_last_task_type arg)
  {
    msg_.wait_on_last_task = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::SetAltitude_Request msg_;
};

class Init_SetAltitude_Request_velocity
{
public:
  explicit Init_SetAltitude_Request_velocity(::airsim_interfaces::srv::SetAltitude_Request & msg)
  : msg_(msg)
  {}
  Init_SetAltitude_Request_wait_on_last_task velocity(::airsim_interfaces::srv::SetAltitude_Request::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_SetAltitude_Request_wait_on_last_task(msg_);
  }

private:
  ::airsim_interfaces::srv::SetAltitude_Request msg_;
};

class Init_SetAltitude_Request_altitude
{
public:
  Init_SetAltitude_Request_altitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetAltitude_Request_velocity altitude(::airsim_interfaces::srv::SetAltitude_Request::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_SetAltitude_Request_velocity(msg_);
  }

private:
  ::airsim_interfaces::srv::SetAltitude_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::SetAltitude_Request>()
{
  return airsim_interfaces::srv::builder::Init_SetAltitude_Request_altitude();
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetAltitude_Response_message
{
public:
  explicit Init_SetAltitude_Response_message(::airsim_interfaces::srv::SetAltitude_Response & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::SetAltitude_Response message(::airsim_interfaces::srv::SetAltitude_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::SetAltitude_Response msg_;
};

class Init_SetAltitude_Response_success
{
public:
  Init_SetAltitude_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetAltitude_Response_message success(::airsim_interfaces::srv::SetAltitude_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetAltitude_Response_message(msg_);
  }

private:
  ::airsim_interfaces::srv::SetAltitude_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::SetAltitude_Response>()
{
  return airsim_interfaces::srv::builder::Init_SetAltitude_Response_success();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_ALTITUDE__BUILDER_HPP_
