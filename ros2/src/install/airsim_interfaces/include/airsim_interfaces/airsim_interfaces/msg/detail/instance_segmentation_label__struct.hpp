// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/InstanceSegmentationLabel.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__InstanceSegmentationLabel __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__InstanceSegmentationLabel __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InstanceSegmentationLabel_
{
  using Type = InstanceSegmentationLabel_<ContainerAllocator>;

  explicit InstanceSegmentationLabel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->index = 0ul;
      this->r = 0;
      this->g = 0;
      this->b = 0;
    }
  }

  explicit InstanceSegmentationLabel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->index = 0ul;
      this->r = 0;
      this->g = 0;
      this->b = 0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _index_type =
    uint32_t;
  _index_type index;
  using _r_type =
    uint8_t;
  _r_type r;
  using _g_type =
    uint8_t;
  _g_type g;
  using _b_type =
    uint8_t;
  _b_type b;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__index(
    const uint32_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__r(
    const uint8_t & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__g(
    const uint8_t & _arg)
  {
    this->g = _arg;
    return *this;
  }
  Type & set__b(
    const uint8_t & _arg)
  {
    this->b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__InstanceSegmentationLabel
    std::shared_ptr<airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__InstanceSegmentationLabel
    std::shared_ptr<airsim_interfaces::msg::InstanceSegmentationLabel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InstanceSegmentationLabel_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->g != other.g) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    return true;
  }
  bool operator!=(const InstanceSegmentationLabel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InstanceSegmentationLabel_

// alias to use template instance with default allocator
using InstanceSegmentationLabel =
  airsim_interfaces::msg::InstanceSegmentationLabel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__STRUCT_HPP_
