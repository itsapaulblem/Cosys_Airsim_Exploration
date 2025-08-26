// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/YawMode.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__YawMode __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__YawMode __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YawMode_
{
  using Type = YawMode_<ContainerAllocator>;

  explicit YawMode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_rate = false;
      this->yaw_or_rate = 0.0f;
    }
  }

  explicit YawMode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_rate = false;
      this->yaw_or_rate = 0.0f;
    }
  }

  // field types and members
  using _is_rate_type =
    bool;
  _is_rate_type is_rate;
  using _yaw_or_rate_type =
    float;
  _yaw_or_rate_type yaw_or_rate;

  // setters for named parameter idiom
  Type & set__is_rate(
    const bool & _arg)
  {
    this->is_rate = _arg;
    return *this;
  }
  Type & set__yaw_or_rate(
    const float & _arg)
  {
    this->yaw_or_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::YawMode_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::YawMode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::YawMode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::YawMode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::YawMode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::YawMode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::YawMode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::YawMode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::YawMode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::YawMode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__YawMode
    std::shared_ptr<airsim_interfaces::msg::YawMode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__YawMode
    std::shared_ptr<airsim_interfaces::msg::YawMode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YawMode_ & other) const
  {
    if (this->is_rate != other.is_rate) {
      return false;
    }
    if (this->yaw_or_rate != other.yaw_or_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const YawMode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YawMode_

// alias to use template instance with default allocator
using YawMode =
  airsim_interfaces::msg::YawMode_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__STRUCT_HPP_
