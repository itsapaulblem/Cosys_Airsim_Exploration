// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:srv/ListSceneObjectTags.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__STRUCT_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__ListSceneObjectTags_Request __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__ListSceneObjectTags_Request __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ListSceneObjectTags_Request_
{
  using Type = ListSceneObjectTags_Request_<ContainerAllocator>;

  explicit ListSceneObjectTags_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->regex_name = "";
    }
  }

  explicit ListSceneObjectTags_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : regex_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->regex_name = "";
    }
  }

  // field types and members
  using _regex_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _regex_name_type regex_name;

  // setters for named parameter idiom
  Type & set__regex_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->regex_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__ListSceneObjectTags_Request
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__ListSceneObjectTags_Request
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListSceneObjectTags_Request_ & other) const
  {
    if (this->regex_name != other.regex_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const ListSceneObjectTags_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListSceneObjectTags_Request_

// alias to use template instance with default allocator
using ListSceneObjectTags_Request =
  airsim_interfaces::srv::ListSceneObjectTags_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__ListSceneObjectTags_Response __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__ListSceneObjectTags_Response __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ListSceneObjectTags_Response_
{
  using Type = ListSceneObjectTags_Response_<ContainerAllocator>;

  explicit ListSceneObjectTags_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ListSceneObjectTags_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _objects_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _objects_type objects;
  using _tags_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _tags_type tags;

  // setters for named parameter idiom
  Type & set__objects(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }
  Type & set__tags(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->tags = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__ListSceneObjectTags_Response
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__ListSceneObjectTags_Response
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListSceneObjectTags_Response_ & other) const
  {
    if (this->objects != other.objects) {
      return false;
    }
    if (this->tags != other.tags) {
      return false;
    }
    return true;
  }
  bool operator!=(const ListSceneObjectTags_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListSceneObjectTags_Response_

// alias to use template instance with default allocator
using ListSceneObjectTags_Response =
  airsim_interfaces::srv::ListSceneObjectTags_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces

namespace airsim_interfaces
{

namespace srv
{

struct ListSceneObjectTags
{
  using Request = airsim_interfaces::srv::ListSceneObjectTags_Request;
  using Response = airsim_interfaces::srv::ListSceneObjectTags_Response;
};

}  // namespace srv

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__STRUCT_HPP_
