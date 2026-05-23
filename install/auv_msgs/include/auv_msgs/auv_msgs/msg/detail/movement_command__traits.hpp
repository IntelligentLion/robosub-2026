// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auv_msgs:msg/MovementCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/movement_command.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__TRAITS_HPP_
#define AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auv_msgs/msg/detail/movement_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MovementCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MovementCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MovementCommand & msg, bool use_flow_style = false)
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
  const auv_msgs::msg::MovementCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  auv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const auv_msgs::msg::MovementCommand & msg)
{
  return auv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auv_msgs::msg::MovementCommand>()
{
  return "auv_msgs::msg::MovementCommand";
}

template<>
inline const char * name<auv_msgs::msg::MovementCommand>()
{
  return "auv_msgs/msg/MovementCommand";
}

template<>
struct has_fixed_size<auv_msgs::msg::MovementCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<auv_msgs::msg::MovementCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<auv_msgs::msg::MovementCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__TRAITS_HPP_
