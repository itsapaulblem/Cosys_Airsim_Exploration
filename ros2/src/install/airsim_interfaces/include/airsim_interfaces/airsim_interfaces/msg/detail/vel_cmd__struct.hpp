// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'yaw_mode'
#include "airsim_interfaces/msg/detail/yaw_mode__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__VelCmd __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__VelCmd __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelCmd_
{
  using Type = VelCmd_<ContainerAllocator>;

  explicit VelCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_init),
    yaw_mode(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drivetrain = 0;
    }
  }

  explicit VelCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_alloc, _init),
    yaw_mode(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drivetrain = 0;
    }
  }

  // field types and members
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;
  using _drivetrain_type =
    uint8_t;
  _drivetrain_type drivetrain;
  using _yaw_mode_type =
    airsim_interfaces::msg::YawMode_<ContainerAllocator>;
  _yaw_mode_type yaw_mode;

  // setters for named parameter idiom
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__drivetrain(
    const uint8_t & _arg)
  {
    this->drivetrain = _arg;
    return *this;
  }
  Type & set__yaw_mode(
    const airsim_interfaces::msg::YawMode_<ContainerAllocator> & _arg)
  {
    this->yaw_mode = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t DRIVETRAIN_MAX_DEGREE_OF_FREEDOM =
    0u;
  static constexpr uint8_t DRIVETRAIN_FORWARD_ONLY =
    1u;

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::VelCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::VelCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::VelCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::VelCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__VelCmd
    std::shared_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__VelCmd
    std::shared_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelCmd_ & other) const
  {
    if (this->twist != other.twist) {
      return false;
    }
    if (this->drivetrain != other.drivetrain) {
      return false;
    }
    if (this->yaw_mode != other.yaw_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelCmd_

// alias to use template instance with default allocator
using VelCmd =
  airsim_interfaces::msg::VelCmd_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t VelCmd_<ContainerAllocator>::DRIVETRAIN_MAX_DEGREE_OF_FREEDOM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t VelCmd_<ContainerAllocator>::DRIVETRAIN_FORWARD_ONLY;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_HPP_
