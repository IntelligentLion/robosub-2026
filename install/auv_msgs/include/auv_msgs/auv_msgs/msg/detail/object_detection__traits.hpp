// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auv_msgs:msg/ObjectDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/object_detection.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__TRAITS_HPP_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auv_msgs/msg/detail/object_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace auv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: bbox_width
  {
    out << "bbox_width: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_width, out);
    out << ", ";
  }

  // member: bbox_height
  {
    out << "bbox_height: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_height, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: bbox_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_width: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_width, out);
    out << "\n";
  }

  // member: bbox_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_height: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_height, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectDetection & msg, bool use_flow_style = false)
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
  const auv_msgs::msg::ObjectDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  auv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const auv_msgs::msg::ObjectDetection & msg)
{
  return auv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auv_msgs::msg::ObjectDetection>()
{
  return "auv_msgs::msg::ObjectDetection";
}

template<>
inline const char * name<auv_msgs::msg::ObjectDetection>()
{
  return "auv_msgs/msg/ObjectDetection";
}

template<>
struct has_fixed_size<auv_msgs::msg::ObjectDetection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<auv_msgs::msg::ObjectDetection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<auv_msgs::msg::ObjectDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__TRAITS_HPP_
