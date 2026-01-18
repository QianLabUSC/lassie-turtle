// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from traveler_msgs:msg/SetState.idl
// generated code does not contain a copyright notice

#ifndef TRAVELER_MSGS__MSG__DETAIL__SET_STATE__BUILDER_HPP_
#define TRAVELER_MSGS__MSG__DETAIL__SET_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "traveler_msgs/msg/detail/set_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace traveler_msgs
{

namespace msg
{

namespace builder
{

class Init_SetState_set_state
{
public:
  explicit Init_SetState_set_state(::traveler_msgs::msg::SetState & msg)
  : msg_(msg)
  {}
  ::traveler_msgs::msg::SetState set_state(::traveler_msgs::msg::SetState::_set_state_type arg)
  {
    msg_.set_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::traveler_msgs::msg::SetState msg_;
};

class Init_SetState_axis
{
public:
  explicit Init_SetState_axis(::traveler_msgs::msg::SetState & msg)
  : msg_(msg)
  {}
  Init_SetState_set_state axis(::traveler_msgs::msg::SetState::_axis_type arg)
  {
    msg_.axis = std::move(arg);
    return Init_SetState_set_state(msg_);
  }

private:
  ::traveler_msgs::msg::SetState msg_;
};

class Init_SetState_can_channel
{
public:
  Init_SetState_can_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetState_axis can_channel(::traveler_msgs::msg::SetState::_can_channel_type arg)
  {
    msg_.can_channel = std::move(arg);
    return Init_SetState_axis(msg_);
  }

private:
  ::traveler_msgs::msg::SetState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::traveler_msgs::msg::SetState>()
{
  return traveler_msgs::msg::builder::Init_SetState_can_channel();
}

}  // namespace traveler_msgs

#endif  // TRAVELER_MSGS__MSG__DETAIL__SET_STATE__BUILDER_HPP_
