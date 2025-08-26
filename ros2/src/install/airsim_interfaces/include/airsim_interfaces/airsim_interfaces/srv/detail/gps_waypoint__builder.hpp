// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/GpsWaypoint.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__GPS_WAYPOINT__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__GPS_WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "airsim_interfaces/srv/detail/gps_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_GpsWaypoint_Request_wait_on_last_task
{
public:
  explicit Init_GpsWaypoint_Request_wait_on_last_task(::airsim_interfaces::srv::GpsWaypoint_Request & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::GpsWaypoint_Request wait_on_last_task(::airsim_interfaces::srv::GpsWaypoint_Request::_wait_on_last_task_type arg)
  {
    msg_.wait_on_last_task = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Request msg_;
};

class Init_GpsWaypoint_Request_tolerance
{
public:
  explicit Init_GpsWaypoint_Request_tolerance(::airsim_interfaces::srv::GpsWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_Request_wait_on_last_task tolerance(::airsim_interfaces::srv::GpsWaypoint_Request::_tolerance_type arg)
  {
    msg_.tolerance = std::move(arg);
    return Init_GpsWaypoint_Request_wait_on_last_task(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Request msg_;
};

class Init_GpsWaypoint_Request_speed
{
public:
  explicit Init_GpsWaypoint_Request_speed(::airsim_interfaces::srv::GpsWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_Request_tolerance speed(::airsim_interfaces::srv::GpsWaypoint_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_GpsWaypoint_Request_tolerance(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Request msg_;
};

class Init_GpsWaypoint_Request_altitude
{
public:
  explicit Init_GpsWaypoint_Request_altitude(::airsim_interfaces::srv::GpsWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_Request_speed altitude(::airsim_interfaces::srv::GpsWaypoint_Request::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GpsWaypoint_Request_speed(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Request msg_;
};

class Init_GpsWaypoint_Request_longitude
{
public:
  explicit Init_GpsWaypoint_Request_longitude(::airsim_interfaces::srv::GpsWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_Request_altitude longitude(::airsim_interfaces::srv::GpsWaypoint_Request::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GpsWaypoint_Request_altitude(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Request msg_;
};

class Init_GpsWaypoint_Request_latitude
{
public:
  Init_GpsWaypoint_Request_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsWaypoint_Request_longitude latitude(::airsim_interfaces::srv::GpsWaypoint_Request::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GpsWaypoint_Request_longitude(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::GpsWaypoint_Request>()
{
  return airsim_interfaces::srv::builder::Init_GpsWaypoint_Request_latitude();
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_GpsWaypoint_Response_final_distance
{
public:
  explicit Init_GpsWaypoint_Response_final_distance(::airsim_interfaces::srv::GpsWaypoint_Response & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::GpsWaypoint_Response final_distance(::airsim_interfaces::srv::GpsWaypoint_Response::_final_distance_type arg)
  {
    msg_.final_distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Response msg_;
};

class Init_GpsWaypoint_Response_message
{
public:
  explicit Init_GpsWaypoint_Response_message(::airsim_interfaces::srv::GpsWaypoint_Response & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_Response_final_distance message(::airsim_interfaces::srv::GpsWaypoint_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GpsWaypoint_Response_final_distance(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Response msg_;
};

class Init_GpsWaypoint_Response_success
{
public:
  Init_GpsWaypoint_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsWaypoint_Response_message success(::airsim_interfaces::srv::GpsWaypoint_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GpsWaypoint_Response_message(msg_);
  }

private:
  ::airsim_interfaces::srv::GpsWaypoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::GpsWaypoint_Response>()
{
  return airsim_interfaces::srv::builder::Init_GpsWaypoint_Response_success();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__GPS_WAYPOINT__BUILDER_HPP_
