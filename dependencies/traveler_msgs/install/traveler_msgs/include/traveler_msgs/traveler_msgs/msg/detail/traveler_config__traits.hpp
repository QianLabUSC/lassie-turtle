// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from traveler_msgs:msg/TravelerConfig.idl
// generated code does not contain a copyright notice

#ifndef TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__TRAITS_HPP_
#define TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "traveler_msgs/msg/detail/traveler_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace traveler_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TravelerConfig & msg,
  std::ostream & out)
{
  out << "{";
  // member: running_scenario
  {
    out << "running_scenario: ";
    rosidl_generator_traits::value_to_yaml(msg.running_scenario, out);
    out << ", ";
  }

  // member: filename
  {
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << ", ";
  }

  // member: drag_traj
  {
    out << "drag_traj: ";
    rosidl_generator_traits::value_to_yaml(msg.drag_traj, out);
    out << ", ";
  }

  // member: extrude_speed
  {
    out << "extrude_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.extrude_speed, out);
    out << ", ";
  }

  // member: extrude_angle
  {
    out << "extrude_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.extrude_angle, out);
    out << ", ";
  }

  // member: extrude_depth
  {
    out << "extrude_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.extrude_depth, out);
    out << ", ";
  }

  // member: shear_penetration_depth
  {
    out << "shear_penetration_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_penetration_depth, out);
    out << ", ";
  }

  // member: shear_penetration_speed
  {
    out << "shear_penetration_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_penetration_speed, out);
    out << ", ";
  }

  // member: shear_penetration_delay
  {
    out << "shear_penetration_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_penetration_delay, out);
    out << ", ";
  }

  // member: shear_length
  {
    out << "shear_length: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_length, out);
    out << ", ";
  }

  // member: shear_speed
  {
    out << "shear_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_speed, out);
    out << ", ";
  }

  // member: shear_delay
  {
    out << "shear_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_delay, out);
    out << ", ";
  }

  // member: shear_return_speed
  {
    out << "shear_return_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_return_speed, out);
    out << ", ";
  }

  // member: workspace_angular_speed
  {
    out << "workspace_angular_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_angular_speed, out);
    out << ", ";
  }

  // member: workspace_moving_angle
  {
    out << "workspace_moving_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_moving_angle, out);
    out << ", ";
  }

  // member: workspace_time_delay
  {
    out << "workspace_time_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_time_delay, out);
    out << ", ";
  }

  // member: static_length
  {
    out << "static_length: ";
    rosidl_generator_traits::value_to_yaml(msg.static_length, out);
    out << ", ";
  }

  // member: static_angle
  {
    out << "static_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.static_angle, out);
    out << ", ";
  }

  // member: search_start
  {
    out << "search_start: ";
    rosidl_generator_traits::value_to_yaml(msg.search_start, out);
    out << ", ";
  }

  // member: search_end
  {
    out << "search_end: ";
    rosidl_generator_traits::value_to_yaml(msg.search_end, out);
    out << ", ";
  }

  // member: ground_height
  {
    out << "ground_height: ";
    rosidl_generator_traits::value_to_yaml(msg.ground_height, out);
    out << ", ";
  }

  // member: back_speed
  {
    out << "back_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.back_speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TravelerConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: running_scenario
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "running_scenario: ";
    rosidl_generator_traits::value_to_yaml(msg.running_scenario, out);
    out << "\n";
  }

  // member: filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << "\n";
  }

  // member: drag_traj
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drag_traj: ";
    rosidl_generator_traits::value_to_yaml(msg.drag_traj, out);
    out << "\n";
  }

  // member: extrude_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "extrude_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.extrude_speed, out);
    out << "\n";
  }

  // member: extrude_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "extrude_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.extrude_angle, out);
    out << "\n";
  }

  // member: extrude_depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "extrude_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.extrude_depth, out);
    out << "\n";
  }

  // member: shear_penetration_depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shear_penetration_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_penetration_depth, out);
    out << "\n";
  }

  // member: shear_penetration_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shear_penetration_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_penetration_speed, out);
    out << "\n";
  }

  // member: shear_penetration_delay
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shear_penetration_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_penetration_delay, out);
    out << "\n";
  }

  // member: shear_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shear_length: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_length, out);
    out << "\n";
  }

  // member: shear_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shear_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_speed, out);
    out << "\n";
  }

  // member: shear_delay
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shear_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_delay, out);
    out << "\n";
  }

  // member: shear_return_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shear_return_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.shear_return_speed, out);
    out << "\n";
  }

  // member: workspace_angular_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "workspace_angular_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_angular_speed, out);
    out << "\n";
  }

  // member: workspace_moving_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "workspace_moving_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_moving_angle, out);
    out << "\n";
  }

  // member: workspace_time_delay
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "workspace_time_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_time_delay, out);
    out << "\n";
  }

  // member: static_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "static_length: ";
    rosidl_generator_traits::value_to_yaml(msg.static_length, out);
    out << "\n";
  }

  // member: static_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "static_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.static_angle, out);
    out << "\n";
  }

  // member: search_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "search_start: ";
    rosidl_generator_traits::value_to_yaml(msg.search_start, out);
    out << "\n";
  }

  // member: search_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "search_end: ";
    rosidl_generator_traits::value_to_yaml(msg.search_end, out);
    out << "\n";
  }

  // member: ground_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ground_height: ";
    rosidl_generator_traits::value_to_yaml(msg.ground_height, out);
    out << "\n";
  }

  // member: back_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "back_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.back_speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TravelerConfig & msg, bool use_flow_style = false)
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

}  // namespace traveler_msgs

namespace rosidl_generator_traits
{

[[deprecated("use traveler_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const traveler_msgs::msg::TravelerConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  traveler_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use traveler_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const traveler_msgs::msg::TravelerConfig & msg)
{
  return traveler_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<traveler_msgs::msg::TravelerConfig>()
{
  return "traveler_msgs::msg::TravelerConfig";
}

template<>
inline const char * name<traveler_msgs::msg::TravelerConfig>()
{
  return "traveler_msgs/msg/TravelerConfig";
}

template<>
struct has_fixed_size<traveler_msgs::msg::TravelerConfig>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<traveler_msgs::msg::TravelerConfig>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<traveler_msgs::msg::TravelerConfig>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__TRAITS_HPP_
