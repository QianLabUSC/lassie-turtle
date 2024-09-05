// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from traveler_msgs:msg/TravelerConfig.idl
// generated code does not contain a copyright notice

#ifndef TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__BUILDER_HPP_
#define TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "traveler_msgs/msg/detail/traveler_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace traveler_msgs
{

namespace msg
{

namespace builder
{

class Init_TravelerConfig_back_speed
{
public:
  explicit Init_TravelerConfig_back_speed(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  ::traveler_msgs::msg::TravelerConfig back_speed(::traveler_msgs::msg::TravelerConfig::_back_speed_type arg)
  {
    msg_.back_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_ground_height
{
public:
  explicit Init_TravelerConfig_ground_height(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_back_speed ground_height(::traveler_msgs::msg::TravelerConfig::_ground_height_type arg)
  {
    msg_.ground_height = std::move(arg);
    return Init_TravelerConfig_back_speed(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_search_end
{
public:
  explicit Init_TravelerConfig_search_end(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_ground_height search_end(::traveler_msgs::msg::TravelerConfig::_search_end_type arg)
  {
    msg_.search_end = std::move(arg);
    return Init_TravelerConfig_ground_height(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_search_start
{
public:
  explicit Init_TravelerConfig_search_start(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_search_end search_start(::traveler_msgs::msg::TravelerConfig::_search_start_type arg)
  {
    msg_.search_start = std::move(arg);
    return Init_TravelerConfig_search_end(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_static_angle
{
public:
  explicit Init_TravelerConfig_static_angle(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_search_start static_angle(::traveler_msgs::msg::TravelerConfig::_static_angle_type arg)
  {
    msg_.static_angle = std::move(arg);
    return Init_TravelerConfig_search_start(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_static_length
{
public:
  explicit Init_TravelerConfig_static_length(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_static_angle static_length(::traveler_msgs::msg::TravelerConfig::_static_length_type arg)
  {
    msg_.static_length = std::move(arg);
    return Init_TravelerConfig_static_angle(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_workspace_time_delay
{
public:
  explicit Init_TravelerConfig_workspace_time_delay(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_static_length workspace_time_delay(::traveler_msgs::msg::TravelerConfig::_workspace_time_delay_type arg)
  {
    msg_.workspace_time_delay = std::move(arg);
    return Init_TravelerConfig_static_length(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_workspace_moving_angle
{
public:
  explicit Init_TravelerConfig_workspace_moving_angle(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_workspace_time_delay workspace_moving_angle(::traveler_msgs::msg::TravelerConfig::_workspace_moving_angle_type arg)
  {
    msg_.workspace_moving_angle = std::move(arg);
    return Init_TravelerConfig_workspace_time_delay(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_workspace_angular_speed
{
public:
  explicit Init_TravelerConfig_workspace_angular_speed(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_workspace_moving_angle workspace_angular_speed(::traveler_msgs::msg::TravelerConfig::_workspace_angular_speed_type arg)
  {
    msg_.workspace_angular_speed = std::move(arg);
    return Init_TravelerConfig_workspace_moving_angle(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_shear_return_speed
{
public:
  explicit Init_TravelerConfig_shear_return_speed(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_workspace_angular_speed shear_return_speed(::traveler_msgs::msg::TravelerConfig::_shear_return_speed_type arg)
  {
    msg_.shear_return_speed = std::move(arg);
    return Init_TravelerConfig_workspace_angular_speed(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_shear_delay
{
public:
  explicit Init_TravelerConfig_shear_delay(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_shear_return_speed shear_delay(::traveler_msgs::msg::TravelerConfig::_shear_delay_type arg)
  {
    msg_.shear_delay = std::move(arg);
    return Init_TravelerConfig_shear_return_speed(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_shear_speed
{
public:
  explicit Init_TravelerConfig_shear_speed(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_shear_delay shear_speed(::traveler_msgs::msg::TravelerConfig::_shear_speed_type arg)
  {
    msg_.shear_speed = std::move(arg);
    return Init_TravelerConfig_shear_delay(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_shear_length
{
public:
  explicit Init_TravelerConfig_shear_length(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_shear_speed shear_length(::traveler_msgs::msg::TravelerConfig::_shear_length_type arg)
  {
    msg_.shear_length = std::move(arg);
    return Init_TravelerConfig_shear_speed(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_shear_penetration_delay
{
public:
  explicit Init_TravelerConfig_shear_penetration_delay(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_shear_length shear_penetration_delay(::traveler_msgs::msg::TravelerConfig::_shear_penetration_delay_type arg)
  {
    msg_.shear_penetration_delay = std::move(arg);
    return Init_TravelerConfig_shear_length(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_shear_penetration_speed
{
public:
  explicit Init_TravelerConfig_shear_penetration_speed(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_shear_penetration_delay shear_penetration_speed(::traveler_msgs::msg::TravelerConfig::_shear_penetration_speed_type arg)
  {
    msg_.shear_penetration_speed = std::move(arg);
    return Init_TravelerConfig_shear_penetration_delay(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_shear_penetration_depth
{
public:
  explicit Init_TravelerConfig_shear_penetration_depth(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_shear_penetration_speed shear_penetration_depth(::traveler_msgs::msg::TravelerConfig::_shear_penetration_depth_type arg)
  {
    msg_.shear_penetration_depth = std::move(arg);
    return Init_TravelerConfig_shear_penetration_speed(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_extrude_depth
{
public:
  explicit Init_TravelerConfig_extrude_depth(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_shear_penetration_depth extrude_depth(::traveler_msgs::msg::TravelerConfig::_extrude_depth_type arg)
  {
    msg_.extrude_depth = std::move(arg);
    return Init_TravelerConfig_shear_penetration_depth(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_extrude_angle
{
public:
  explicit Init_TravelerConfig_extrude_angle(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_extrude_depth extrude_angle(::traveler_msgs::msg::TravelerConfig::_extrude_angle_type arg)
  {
    msg_.extrude_angle = std::move(arg);
    return Init_TravelerConfig_extrude_depth(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_extrude_speed
{
public:
  explicit Init_TravelerConfig_extrude_speed(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_extrude_angle extrude_speed(::traveler_msgs::msg::TravelerConfig::_extrude_speed_type arg)
  {
    msg_.extrude_speed = std::move(arg);
    return Init_TravelerConfig_extrude_angle(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_filename
{
public:
  explicit Init_TravelerConfig_filename(::traveler_msgs::msg::TravelerConfig & msg)
  : msg_(msg)
  {}
  Init_TravelerConfig_extrude_speed filename(::traveler_msgs::msg::TravelerConfig::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return Init_TravelerConfig_extrude_speed(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

class Init_TravelerConfig_running_scenario
{
public:
  Init_TravelerConfig_running_scenario()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TravelerConfig_filename running_scenario(::traveler_msgs::msg::TravelerConfig::_running_scenario_type arg)
  {
    msg_.running_scenario = std::move(arg);
    return Init_TravelerConfig_filename(msg_);
  }

private:
  ::traveler_msgs::msg::TravelerConfig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::traveler_msgs::msg::TravelerConfig>()
{
  return traveler_msgs::msg::builder::Init_TravelerConfig_running_scenario();
}

}  // namespace traveler_msgs

#endif  // TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__BUILDER_HPP_
