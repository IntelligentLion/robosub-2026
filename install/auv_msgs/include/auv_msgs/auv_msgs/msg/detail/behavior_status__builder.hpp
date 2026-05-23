// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auv_msgs:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/behavior_status.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__BUILDER_HPP_
#define AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auv_msgs/msg/detail/behavior_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auv_msgs
{

namespace msg
{

namespace builder
{

class Init_BehaviorStatus_reason
{
public:
  explicit Init_BehaviorStatus_reason(::auv_msgs::msg::BehaviorStatus & msg)
  : msg_(msg)
  {}
  ::auv_msgs::msg::BehaviorStatus reason(::auv_msgs::msg::BehaviorStatus::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auv_msgs::msg::BehaviorStatus msg_;
};

class Init_BehaviorStatus_status
{
public:
  explicit Init_BehaviorStatus_status(::auv_msgs::msg::BehaviorStatus & msg)
  : msg_(msg)
  {}
  Init_BehaviorStatus_reason status(::auv_msgs::msg::BehaviorStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_BehaviorStatus_reason(msg_);
  }

private:
  ::auv_msgs::msg::BehaviorStatus msg_;
};

class Init_BehaviorStatus_action_name
{
public:
  explicit Init_BehaviorStatus_action_name(::auv_msgs::msg::BehaviorStatus & msg)
  : msg_(msg)
  {}
  Init_BehaviorStatus_status action_name(::auv_msgs::msg::BehaviorStatus::_action_name_type arg)
  {
    msg_.action_name = std::move(arg);
    return Init_BehaviorStatus_status(msg_);
  }

private:
  ::auv_msgs::msg::BehaviorStatus msg_;
};

class Init_BehaviorStatus_stamp
{
public:
  Init_BehaviorStatus_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BehaviorStatus_action_name stamp(::auv_msgs::msg::BehaviorStatus::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_BehaviorStatus_action_name(msg_);
  }

private:
  ::auv_msgs::msg::BehaviorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auv_msgs::msg::BehaviorStatus>()
{
  return auv_msgs::msg::builder::Init_BehaviorStatus_stamp();
}

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__BUILDER_HPP_
