// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auv_msgs:msg/MovementCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/movement_command.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__BUILDER_HPP_
#define AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auv_msgs/msg/detail/movement_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auv_msgs
{

namespace msg
{

namespace builder
{

class Init_MovementCommand_duration
{
public:
  explicit Init_MovementCommand_duration(::auv_msgs::msg::MovementCommand & msg)
  : msg_(msg)
  {}
  ::auv_msgs::msg::MovementCommand duration(::auv_msgs::msg::MovementCommand::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auv_msgs::msg::MovementCommand msg_;
};

class Init_MovementCommand_speed
{
public:
  explicit Init_MovementCommand_speed(::auv_msgs::msg::MovementCommand & msg)
  : msg_(msg)
  {}
  Init_MovementCommand_duration speed(::auv_msgs::msg::MovementCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_MovementCommand_duration(msg_);
  }

private:
  ::auv_msgs::msg::MovementCommand msg_;
};

class Init_MovementCommand_command
{
public:
  Init_MovementCommand_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovementCommand_speed command(::auv_msgs::msg::MovementCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_MovementCommand_speed(msg_);
  }

private:
  ::auv_msgs::msg::MovementCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auv_msgs::msg::MovementCommand>()
{
  return auv_msgs::msg::builder::Init_MovementCommand_command();
}

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__BUILDER_HPP_
