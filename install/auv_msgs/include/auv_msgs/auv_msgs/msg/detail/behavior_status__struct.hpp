// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auv_msgs:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/behavior_status.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_HPP_
#define AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auv_msgs__msg__BehaviorStatus __attribute__((deprecated))
#else
# define DEPRECATED__auv_msgs__msg__BehaviorStatus __declspec(deprecated)
#endif

namespace auv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BehaviorStatus_
{
  using Type = BehaviorStatus_<ContainerAllocator>;

  explicit BehaviorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action_name = "";
      this->status = "";
      this->reason = "";
    }
  }

  explicit BehaviorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    action_name(_alloc),
    status(_alloc),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action_name = "";
      this->status = "";
      this->reason = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _action_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _action_name_type action_name;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__action_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->action_name = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auv_msgs::msg::BehaviorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const auv_msgs::msg::BehaviorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auv_msgs::msg::BehaviorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auv_msgs::msg::BehaviorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::BehaviorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::BehaviorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::BehaviorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::BehaviorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auv_msgs::msg::BehaviorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auv_msgs::msg::BehaviorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auv_msgs__msg__BehaviorStatus
    std::shared_ptr<auv_msgs::msg::BehaviorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auv_msgs__msg__BehaviorStatus
    std::shared_ptr<auv_msgs::msg::BehaviorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BehaviorStatus_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->action_name != other.action_name) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const BehaviorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BehaviorStatus_

// alias to use template instance with default allocator
using BehaviorStatus =
  auv_msgs::msg::BehaviorStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_HPP_
