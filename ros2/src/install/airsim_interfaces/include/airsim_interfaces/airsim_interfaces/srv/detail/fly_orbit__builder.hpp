// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/FlyOrbit.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__FLY_ORBIT__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__FLY_ORBIT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/srv/detail/fly_orbit__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_FlyOrbit_Request_duration
{
public:
  explicit Init_FlyOrbit_Request_duration(::airsim_interfaces::srv::FlyOrbit_Request & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::FlyOrbit_Request duration(::airsim_interfaces::srv::FlyOrbit_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::FlyOrbit_Request msg_;
};

class Init_FlyOrbit_Request_yaw_mode
{
public:
  explicit Init_FlyOrbit_Request_yaw_mode(::airsim_interfaces::srv::FlyOrbit_Request & msg)
  : msg_(msg)
  {}
  Init_FlyOrbit_Request_duration yaw_mode(::airsim_interfaces::srv::FlyOrbit_Request::_yaw_mode_type arg)
  {
    msg_.yaw_mode = std::move(arg);
    return Init_FlyOrbit_Request_duration(msg_);
  }

private:
  ::airsim_interfaces::srv::FlyOrbit_Request msg_;
};

class Init_FlyOrbit_Request_velocity
{
public:
  explicit Init_FlyOrbit_Request_velocity(::airsim_interfaces::srv::FlyOrbit_Request & msg)
  : msg_(msg)
  {}
  Init_FlyOrbit_Request_yaw_mode velocity(::airsim_interfaces::srv::FlyOrbit_Request::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_FlyOrbit_Request_yaw_mode(msg_);
  }

private:
  ::airsim_interfaces::srv::FlyOrbit_Request msg_;
};

class Init_FlyOrbit_Request_radius
{
public:
  explicit Init_FlyOrbit_Request_radius(::airsim_interfaces::srv::FlyOrbit_Request & msg)
  : msg_(msg)
  {}
  Init_FlyOrbit_Request_velocity radius(::airsim_interfaces::srv::FlyOrbit_Request::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return Init_FlyOrbit_Request_velocity(msg_);
  }

private:
  ::airsim_interfaces::srv::FlyOrbit_Request msg_;
};

class Init_FlyOrbit_Request_center
{
public:
  Init_FlyOrbit_Request_center()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FlyOrbit_Request_radius center(::airsim_interfaces::srv::FlyOrbit_Request::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_FlyOrbit_Request_radius(msg_);
  }

private:
  ::airsim_interfaces::srv::FlyOrbit_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::FlyOrbit_Request>()
{
  return airsim_interfaces::srv::builder::Init_FlyOrbit_Request_center();
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_FlyOrbit_Response_message
{
public:
  explicit Init_FlyOrbit_Response_message(::airsim_interfaces::srv::FlyOrbit_Response & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::FlyOrbit_Response message(::airsim_interfaces::srv::FlyOrbit_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::FlyOrbit_Response msg_;
};

class Init_FlyOrbit_Response_success
{
public:
  Init_FlyOrbit_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FlyOrbit_Response_message success(::airsim_interfaces::srv::FlyOrbit_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FlyOrbit_Response_message(msg_);
  }

private:
  ::airsim_interfaces::srv::FlyOrbit_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::FlyOrbit_Response>()
{
  return airsim_interfaces::srv::builder::Init_FlyOrbit_Response_success();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__FLY_ORBIT__BUILDER_HPP_
