// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from traveler_msgs:msg/TravelerConfig.idl
// generated code does not contain a copyright notice

#ifndef TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__STRUCT_HPP_
#define TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__traveler_msgs__msg__TravelerConfig __attribute__((deprecated))
#else
# define DEPRECATED__traveler_msgs__msg__TravelerConfig __declspec(deprecated)
#endif

namespace traveler_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TravelerConfig_
{
  using Type = TravelerConfig_<ContainerAllocator>;

  explicit TravelerConfig_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->running_scenario = "";
      this->filename = "";
      this->drag_traj = 0.0f;
      this->extrude_speed = 0.0f;
      this->extrude_angle = 0.0f;
      this->extrude_depth = 0.0f;
      this->shear_penetration_depth = 0.0f;
      this->shear_penetration_speed = 0.0f;
      this->shear_penetration_delay = 0.0f;
      this->shear_length = 0.0f;
      this->shear_speed = 0.0f;
      this->shear_delay = 0.0f;
      this->shear_return_speed = 0.0f;
      this->workspace_angular_speed = 0.0f;
      this->workspace_moving_angle = 0.0f;
      this->workspace_time_delay = 0.0f;
      this->static_length = 0.0f;
      this->static_angle = 0.0f;
      this->search_start = 0.0f;
      this->search_end = 0.0f;
      this->ground_height = 0.0f;
      this->back_speed = 0.0f;
    }
  }

  explicit TravelerConfig_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : running_scenario(_alloc),
    filename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->running_scenario = "";
      this->filename = "";
      this->drag_traj = 0.0f;
      this->extrude_speed = 0.0f;
      this->extrude_angle = 0.0f;
      this->extrude_depth = 0.0f;
      this->shear_penetration_depth = 0.0f;
      this->shear_penetration_speed = 0.0f;
      this->shear_penetration_delay = 0.0f;
      this->shear_length = 0.0f;
      this->shear_speed = 0.0f;
      this->shear_delay = 0.0f;
      this->shear_return_speed = 0.0f;
      this->workspace_angular_speed = 0.0f;
      this->workspace_moving_angle = 0.0f;
      this->workspace_time_delay = 0.0f;
      this->static_length = 0.0f;
      this->static_angle = 0.0f;
      this->search_start = 0.0f;
      this->search_end = 0.0f;
      this->ground_height = 0.0f;
      this->back_speed = 0.0f;
    }
  }

  // field types and members
  using _running_scenario_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _running_scenario_type running_scenario;
  using _filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _filename_type filename;
  using _drag_traj_type =
    float;
  _drag_traj_type drag_traj;
  using _extrude_speed_type =
    float;
  _extrude_speed_type extrude_speed;
  using _extrude_angle_type =
    float;
  _extrude_angle_type extrude_angle;
  using _extrude_depth_type =
    float;
  _extrude_depth_type extrude_depth;
  using _shear_penetration_depth_type =
    float;
  _shear_penetration_depth_type shear_penetration_depth;
  using _shear_penetration_speed_type =
    float;
  _shear_penetration_speed_type shear_penetration_speed;
  using _shear_penetration_delay_type =
    float;
  _shear_penetration_delay_type shear_penetration_delay;
  using _shear_length_type =
    float;
  _shear_length_type shear_length;
  using _shear_speed_type =
    float;
  _shear_speed_type shear_speed;
  using _shear_delay_type =
    float;
  _shear_delay_type shear_delay;
  using _shear_return_speed_type =
    float;
  _shear_return_speed_type shear_return_speed;
  using _workspace_angular_speed_type =
    float;
  _workspace_angular_speed_type workspace_angular_speed;
  using _workspace_moving_angle_type =
    float;
  _workspace_moving_angle_type workspace_moving_angle;
  using _workspace_time_delay_type =
    float;
  _workspace_time_delay_type workspace_time_delay;
  using _static_length_type =
    float;
  _static_length_type static_length;
  using _static_angle_type =
    float;
  _static_angle_type static_angle;
  using _search_start_type =
    float;
  _search_start_type search_start;
  using _search_end_type =
    float;
  _search_end_type search_end;
  using _ground_height_type =
    float;
  _ground_height_type ground_height;
  using _back_speed_type =
    float;
  _back_speed_type back_speed;

  // setters for named parameter idiom
  Type & set__running_scenario(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->running_scenario = _arg;
    return *this;
  }
  Type & set__filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->filename = _arg;
    return *this;
  }
  Type & set__drag_traj(
    const float & _arg)
  {
    this->drag_traj = _arg;
    return *this;
  }
  Type & set__extrude_speed(
    const float & _arg)
  {
    this->extrude_speed = _arg;
    return *this;
  }
  Type & set__extrude_angle(
    const float & _arg)
  {
    this->extrude_angle = _arg;
    return *this;
  }
  Type & set__extrude_depth(
    const float & _arg)
  {
    this->extrude_depth = _arg;
    return *this;
  }
  Type & set__shear_penetration_depth(
    const float & _arg)
  {
    this->shear_penetration_depth = _arg;
    return *this;
  }
  Type & set__shear_penetration_speed(
    const float & _arg)
  {
    this->shear_penetration_speed = _arg;
    return *this;
  }
  Type & set__shear_penetration_delay(
    const float & _arg)
  {
    this->shear_penetration_delay = _arg;
    return *this;
  }
  Type & set__shear_length(
    const float & _arg)
  {
    this->shear_length = _arg;
    return *this;
  }
  Type & set__shear_speed(
    const float & _arg)
  {
    this->shear_speed = _arg;
    return *this;
  }
  Type & set__shear_delay(
    const float & _arg)
  {
    this->shear_delay = _arg;
    return *this;
  }
  Type & set__shear_return_speed(
    const float & _arg)
  {
    this->shear_return_speed = _arg;
    return *this;
  }
  Type & set__workspace_angular_speed(
    const float & _arg)
  {
    this->workspace_angular_speed = _arg;
    return *this;
  }
  Type & set__workspace_moving_angle(
    const float & _arg)
  {
    this->workspace_moving_angle = _arg;
    return *this;
  }
  Type & set__workspace_time_delay(
    const float & _arg)
  {
    this->workspace_time_delay = _arg;
    return *this;
  }
  Type & set__static_length(
    const float & _arg)
  {
    this->static_length = _arg;
    return *this;
  }
  Type & set__static_angle(
    const float & _arg)
  {
    this->static_angle = _arg;
    return *this;
  }
  Type & set__search_start(
    const float & _arg)
  {
    this->search_start = _arg;
    return *this;
  }
  Type & set__search_end(
    const float & _arg)
  {
    this->search_end = _arg;
    return *this;
  }
  Type & set__ground_height(
    const float & _arg)
  {
    this->ground_height = _arg;
    return *this;
  }
  Type & set__back_speed(
    const float & _arg)
  {
    this->back_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    traveler_msgs::msg::TravelerConfig_<ContainerAllocator> *;
  using ConstRawPtr =
    const traveler_msgs::msg::TravelerConfig_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<traveler_msgs::msg::TravelerConfig_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<traveler_msgs::msg::TravelerConfig_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      traveler_msgs::msg::TravelerConfig_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<traveler_msgs::msg::TravelerConfig_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      traveler_msgs::msg::TravelerConfig_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<traveler_msgs::msg::TravelerConfig_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<traveler_msgs::msg::TravelerConfig_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<traveler_msgs::msg::TravelerConfig_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__traveler_msgs__msg__TravelerConfig
    std::shared_ptr<traveler_msgs::msg::TravelerConfig_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__traveler_msgs__msg__TravelerConfig
    std::shared_ptr<traveler_msgs::msg::TravelerConfig_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TravelerConfig_ & other) const
  {
    if (this->running_scenario != other.running_scenario) {
      return false;
    }
    if (this->filename != other.filename) {
      return false;
    }
    if (this->drag_traj != other.drag_traj) {
      return false;
    }
    if (this->extrude_speed != other.extrude_speed) {
      return false;
    }
    if (this->extrude_angle != other.extrude_angle) {
      return false;
    }
    if (this->extrude_depth != other.extrude_depth) {
      return false;
    }
    if (this->shear_penetration_depth != other.shear_penetration_depth) {
      return false;
    }
    if (this->shear_penetration_speed != other.shear_penetration_speed) {
      return false;
    }
    if (this->shear_penetration_delay != other.shear_penetration_delay) {
      return false;
    }
    if (this->shear_length != other.shear_length) {
      return false;
    }
    if (this->shear_speed != other.shear_speed) {
      return false;
    }
    if (this->shear_delay != other.shear_delay) {
      return false;
    }
    if (this->shear_return_speed != other.shear_return_speed) {
      return false;
    }
    if (this->workspace_angular_speed != other.workspace_angular_speed) {
      return false;
    }
    if (this->workspace_moving_angle != other.workspace_moving_angle) {
      return false;
    }
    if (this->workspace_time_delay != other.workspace_time_delay) {
      return false;
    }
    if (this->static_length != other.static_length) {
      return false;
    }
    if (this->static_angle != other.static_angle) {
      return false;
    }
    if (this->search_start != other.search_start) {
      return false;
    }
    if (this->search_end != other.search_end) {
      return false;
    }
    if (this->ground_height != other.ground_height) {
      return false;
    }
    if (this->back_speed != other.back_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const TravelerConfig_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TravelerConfig_

// alias to use template instance with default allocator
using TravelerConfig =
  traveler_msgs::msg::TravelerConfig_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace traveler_msgs

#endif  // TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__STRUCT_HPP_
