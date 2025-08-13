// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/VelCmdGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'vel_cmd'
#include "airsim_interfaces/msg/detail/vel_cmd__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__VelCmdGroup __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__VelCmdGroup __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelCmdGroup_
{
  using Type = VelCmdGroup_<ContainerAllocator>;

  explicit VelCmdGroup_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vel_cmd(_init)
  {
    (void)_init;
  }

  explicit VelCmdGroup_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vel_cmd(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _vel_cmd_type =
    airsim_interfaces::msg::VelCmd_<ContainerAllocator>;
  _vel_cmd_type vel_cmd;
  using _vehicle_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _vehicle_names_type vehicle_names;

  // setters for named parameter idiom
  Type & set__vel_cmd(
    const airsim_interfaces::msg::VelCmd_<ContainerAllocator> & _arg)
  {
    this->vel_cmd = _arg;
    return *this;
  }
  Type & set__vehicle_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->vehicle_names = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__VelCmdGroup
    std::shared_ptr<airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__VelCmdGroup
    std::shared_ptr<airsim_interfaces::msg::VelCmdGroup_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelCmdGroup_ & other) const
  {
    if (this->vel_cmd != other.vel_cmd) {
      return false;
    }
    if (this->vehicle_names != other.vehicle_names) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelCmdGroup_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelCmdGroup_

// alias to use template instance with default allocator
using VelCmdGroup =
  airsim_interfaces::msg::VelCmdGroup_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__STRUCT_HPP_
