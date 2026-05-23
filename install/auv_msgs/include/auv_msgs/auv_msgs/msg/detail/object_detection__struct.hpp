// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auv_msgs:msg/ObjectDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/object_detection.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__STRUCT_HPP_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auv_msgs__msg__ObjectDetection __attribute__((deprecated))
#else
# define DEPRECATED__auv_msgs__msg__ObjectDetection __declspec(deprecated)
#endif

namespace auv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectDetection_
{
  using Type = ObjectDetection_<ContainerAllocator>;

  explicit ObjectDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->confidence = 0.0f;
      this->bbox_width = 0.0f;
      this->bbox_height = 0.0f;
    }
  }

  explicit ObjectDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : label(_alloc),
    position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->confidence = 0.0f;
      this->bbox_width = 0.0f;
      this->bbox_height = 0.0f;
    }
  }

  // field types and members
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _label_type label;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _bbox_width_type =
    float;
  _bbox_width_type bbox_width;
  using _bbox_height_type =
    float;
  _bbox_height_type bbox_height;

  // setters for named parameter idiom
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__bbox_width(
    const float & _arg)
  {
    this->bbox_width = _arg;
    return *this;
  }
  Type & set__bbox_height(
    const float & _arg)
  {
    this->bbox_height = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auv_msgs::msg::ObjectDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const auv_msgs::msg::ObjectDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auv_msgs::msg::ObjectDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auv_msgs::msg::ObjectDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::ObjectDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::ObjectDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::ObjectDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::ObjectDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auv_msgs::msg::ObjectDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auv_msgs::msg::ObjectDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auv_msgs__msg__ObjectDetection
    std::shared_ptr<auv_msgs::msg::ObjectDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auv_msgs__msg__ObjectDetection
    std::shared_ptr<auv_msgs::msg::ObjectDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectDetection_ & other) const
  {
    if (this->label != other.label) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->bbox_width != other.bbox_width) {
      return false;
    }
    if (this->bbox_height != other.bbox_height) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectDetection_

// alias to use template instance with default allocator
using ObjectDetection =
  auv_msgs::msg::ObjectDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__STRUCT_HPP_
