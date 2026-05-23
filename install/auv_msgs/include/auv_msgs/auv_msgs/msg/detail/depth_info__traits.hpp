// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auv_msgs:msg/DepthInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/depth_info.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__DEPTH_INFO__TRAITS_HPP_
#define AUV_MSGS__MSG__DETAIL__DEPTH_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auv_msgs/msg/detail/depth_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace auv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DepthInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: sub_depth_m
  {
    out << "sub_depth_m: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_depth_m, out);
    out << ", ";
  }

  // member: stop_distance_m
  {
    out << "stop_distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_distance_m, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DepthInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: sub_depth_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sub_depth_m: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_depth_m, out);
    out << "\n";
  }

  // member: stop_distance_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_distance_m, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DepthInfo & msg, bool use_flow_style = false)
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
  const auv_msgs::msg::DepthInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  auv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const auv_msgs::msg::DepthInfo & msg)
{
  return auv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auv_msgs::msg::DepthInfo>()
{
  return "auv_msgs::msg::DepthInfo";
}

template<>
inline const char * name<auv_msgs::msg::DepthInfo>()
{
  return "auv_msgs/msg/DepthInfo";
}

template<>
struct has_fixed_size<auv_msgs::msg::DepthInfo>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<auv_msgs::msg::DepthInfo>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<auv_msgs::msg::DepthInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUV_MSGS__MSG__DETAIL__DEPTH_INFO__TRAITS_HPP_
