// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auv_msgs:msg/DepthInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/depth_info.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__DEPTH_INFO__STRUCT_HPP_
#define AUV_MSGS__MSG__DETAIL__DEPTH_INFO__STRUCT_HPP_

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
# define DEPRECATED__auv_msgs__msg__DepthInfo __attribute__((deprecated))
#else
# define DEPRECATED__auv_msgs__msg__DepthInfo __declspec(deprecated)
#endif

namespace auv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DepthInfo_
{
  using Type = DepthInfo_<ContainerAllocator>;

  explicit DepthInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sub_depth_m = 0.0f;
      this->stop_distance_m = 0.0f;
    }
  }

  explicit DepthInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sub_depth_m = 0.0f;
      this->stop_distance_m = 0.0f;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _sub_depth_m_type =
    float;
  _sub_depth_m_type sub_depth_m;
  using _stop_distance_m_type =
    float;
  _stop_distance_m_type stop_distance_m;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__sub_depth_m(
    const float & _arg)
  {
    this->sub_depth_m = _arg;
    return *this;
  }
  Type & set__stop_distance_m(
    const float & _arg)
  {
    this->stop_distance_m = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auv_msgs::msg::DepthInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const auv_msgs::msg::DepthInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auv_msgs::msg::DepthInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auv_msgs::msg::DepthInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::DepthInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::DepthInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::DepthInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::DepthInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auv_msgs::msg::DepthInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auv_msgs::msg::DepthInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auv_msgs__msg__DepthInfo
    std::shared_ptr<auv_msgs::msg::DepthInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auv_msgs__msg__DepthInfo
    std::shared_ptr<auv_msgs::msg::DepthInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DepthInfo_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->sub_depth_m != other.sub_depth_m) {
      return false;
    }
    if (this->stop_distance_m != other.stop_distance_m) {
      return false;
    }
    return true;
  }
  bool operator!=(const DepthInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DepthInfo_

// alias to use template instance with default allocator
using DepthInfo =
  auv_msgs::msg::DepthInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__DEPTH_INFO__STRUCT_HPP_
