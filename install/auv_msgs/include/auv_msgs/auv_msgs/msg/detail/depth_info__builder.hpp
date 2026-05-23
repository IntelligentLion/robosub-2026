// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auv_msgs:msg/DepthInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/depth_info.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__DEPTH_INFO__BUILDER_HPP_
#define AUV_MSGS__MSG__DETAIL__DEPTH_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auv_msgs/msg/detail/depth_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auv_msgs
{

namespace msg
{

namespace builder
{

class Init_DepthInfo_stop_distance_m
{
public:
  explicit Init_DepthInfo_stop_distance_m(::auv_msgs::msg::DepthInfo & msg)
  : msg_(msg)
  {}
  ::auv_msgs::msg::DepthInfo stop_distance_m(::auv_msgs::msg::DepthInfo::_stop_distance_m_type arg)
  {
    msg_.stop_distance_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auv_msgs::msg::DepthInfo msg_;
};

class Init_DepthInfo_sub_depth_m
{
public:
  explicit Init_DepthInfo_sub_depth_m(::auv_msgs::msg::DepthInfo & msg)
  : msg_(msg)
  {}
  Init_DepthInfo_stop_distance_m sub_depth_m(::auv_msgs::msg::DepthInfo::_sub_depth_m_type arg)
  {
    msg_.sub_depth_m = std::move(arg);
    return Init_DepthInfo_stop_distance_m(msg_);
  }

private:
  ::auv_msgs::msg::DepthInfo msg_;
};

class Init_DepthInfo_stamp
{
public:
  Init_DepthInfo_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DepthInfo_sub_depth_m stamp(::auv_msgs::msg::DepthInfo::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_DepthInfo_sub_depth_m(msg_);
  }

private:
  ::auv_msgs::msg::DepthInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auv_msgs::msg::DepthInfo>()
{
  return auv_msgs::msg::builder::Init_DepthInfo_stamp();
}

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__DEPTH_INFO__BUILDER_HPP_
