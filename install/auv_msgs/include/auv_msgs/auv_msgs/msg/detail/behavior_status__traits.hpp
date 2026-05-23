// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auv_msgs:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/behavior_status.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__TRAITS_HPP_
#define AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auv_msgs/msg/detail/behavior_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace auv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BehaviorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: action_name
  {
    out << "action_name: ";
    rosidl_generator_traits::value_to_yaml(msg.action_name, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BehaviorStatus & msg,
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

  // member: action_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action_name: ";
    rosidl_generator_traits::value_to_yaml(msg.action_name, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BehaviorStatus & msg, bool use_flow_style = false)
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
  const auv_msgs::msg::BehaviorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  auv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const auv_msgs::msg::BehaviorStatus & msg)
{
  return auv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auv_msgs::msg::BehaviorStatus>()
{
  return "auv_msgs::msg::BehaviorStatus";
}

template<>
inline const char * name<auv_msgs::msg::BehaviorStatus>()
{
  return "auv_msgs/msg/BehaviorStatus";
}

template<>
struct has_fixed_size<auv_msgs::msg::BehaviorStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<auv_msgs::msg::BehaviorStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<auv_msgs::msg::BehaviorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__TRAITS_HPP_
