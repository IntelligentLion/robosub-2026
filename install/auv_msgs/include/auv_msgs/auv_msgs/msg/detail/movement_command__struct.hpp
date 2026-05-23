// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auv_msgs:msg/MovementCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/movement_command.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__STRUCT_HPP_
#define AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__auv_msgs__msg__MovementCommand __attribute__((deprecated))
#else
# define DEPRECATED__auv_msgs__msg__MovementCommand __declspec(deprecated)
#endif

namespace auv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MovementCommand_
{
  using Type = MovementCommand_<ContainerAllocator>;

  explicit MovementCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->speed = 0.0f;
      this->duration = 0.0f;
    }
  }

  explicit MovementCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->speed = 0.0f;
      this->duration = 0.0f;
    }
  }

  // field types and members
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;
  using _speed_type =
    float;
  _speed_type speed;
  using _duration_type =
    float;
  _duration_type duration;

  // setters for named parameter idiom
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__duration(
    const float & _arg)
  {
    this->duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auv_msgs::msg::MovementCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const auv_msgs::msg::MovementCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auv_msgs::msg::MovementCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auv_msgs::msg::MovementCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::MovementCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::MovementCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::MovementCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::MovementCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auv_msgs::msg::MovementCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auv_msgs::msg::MovementCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auv_msgs__msg__MovementCommand
    std::shared_ptr<auv_msgs::msg::MovementCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auv_msgs__msg__MovementCommand
    std::shared_ptr<auv_msgs::msg::MovementCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MovementCommand_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const MovementCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MovementCommand_

// alias to use template instance with default allocator
using MovementCommand =
  auv_msgs::msg::MovementCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__STRUCT_HPP_
