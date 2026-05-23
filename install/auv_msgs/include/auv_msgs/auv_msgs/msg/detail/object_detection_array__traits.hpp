// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auv_msgs:msg/ObjectDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/object_detection_array.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__TRAITS_HPP_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auv_msgs/msg/detail/object_detection_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'detections'
#include "auv_msgs/msg/detail/object_detection__traits.hpp"

namespace auv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectDetectionArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: detections
  {
    if (msg.detections.size() == 0) {
      out << "detections: []";
    } else {
      out << "detections: [";
      size_t pending_items = msg.detections.size();
      for (auto item : msg.detections) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectDetectionArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: detections
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.detections.size() == 0) {
      out << "detections: []\n";
    } else {
      out << "detections:\n";
      for (auto item : msg.detections) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectDetectionArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace auv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use auv_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auv_msgs::msg::ObjectDetectionArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  auv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const auv_msgs::msg::ObjectDetectionArray & msg)
{
  return auv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auv_msgs::msg::ObjectDetectionArray>()
{
  return "auv_msgs::msg::ObjectDetectionArray";
}

template<>
inline const char * name<auv_msgs::msg::ObjectDetectionArray>()
{
  return "auv_msgs/msg/ObjectDetectionArray";
}

template<>
struct has_fixed_size<auv_msgs::msg::ObjectDetectionArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<auv_msgs::msg::ObjectDetectionArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<auv_msgs::msg::ObjectDetectionArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__TRAITS_HPP_
