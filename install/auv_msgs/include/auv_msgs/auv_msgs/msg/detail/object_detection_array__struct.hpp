// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auv_msgs:msg/ObjectDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/object_detection_array.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__STRUCT_HPP_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'detections'
#include "auv_msgs/msg/detail/object_detection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auv_msgs__msg__ObjectDetectionArray __attribute__((deprecated))
#else
# define DEPRECATED__auv_msgs__msg__ObjectDetectionArray __declspec(deprecated)
#endif

namespace auv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectDetectionArray_
{
  using Type = ObjectDetectionArray_<ContainerAllocator>;

  explicit ObjectDetectionArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ObjectDetectionArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _detections_type =
    std::vector<auv_msgs::msg::ObjectDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<auv_msgs::msg::ObjectDetection_<ContainerAllocator>>>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__detections(
    const std::vector<auv_msgs::msg::ObjectDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<auv_msgs::msg::ObjectDetection_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auv_msgs__msg__ObjectDetectionArray
    std::shared_ptr<auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auv_msgs__msg__ObjectDetectionArray
    std::shared_ptr<auv_msgs::msg::ObjectDetectionArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectDetectionArray_ & other) const
  {
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectDetectionArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectDetectionArray_

// alias to use template instance with default allocator
using ObjectDetectionArray =
  auv_msgs::msg::ObjectDetectionArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__STRUCT_HPP_
