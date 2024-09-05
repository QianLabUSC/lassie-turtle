// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from traveler_msgs:msg/SetState.idl
// generated code does not contain a copyright notice

#ifndef TRAVELER_MSGS__MSG__DETAIL__SET_STATE__STRUCT_HPP_
#define TRAVELER_MSGS__MSG__DETAIL__SET_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__traveler_msgs__msg__SetState __attribute__((deprecated))
#else
# define DEPRECATED__traveler_msgs__msg__SetState __declspec(deprecated)
#endif

namespace traveler_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetState_
{
  using Type = SetState_<ContainerAllocator>;

  explicit SetState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_channel = 0;
      this->axis = 0;
      this->set_state = 0ul;
    }
  }

  explicit SetState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_channel = 0;
      this->axis = 0;
      this->set_state = 0ul;
    }
  }

  // field types and members
  using _can_channel_type =
    uint8_t;
  _can_channel_type can_channel;
  using _axis_type =
    uint8_t;
  _axis_type axis;
  using _set_state_type =
    uint32_t;
  _set_state_type set_state;

  // setters for named parameter idiom
  Type & set__can_channel(
    const uint8_t & _arg)
  {
    this->can_channel = _arg;
    return *this;
  }
  Type & set__axis(
    const uint8_t & _arg)
  {
    this->axis = _arg;
    return *this;
  }
  Type & set__set_state(
    const uint32_t & _arg)
  {
    this->set_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    traveler_msgs::msg::SetState_<ContainerAllocator> *;
  using ConstRawPtr =
    const traveler_msgs::msg::SetState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<traveler_msgs::msg::SetState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<traveler_msgs::msg::SetState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      traveler_msgs::msg::SetState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<traveler_msgs::msg::SetState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      traveler_msgs::msg::SetState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<traveler_msgs::msg::SetState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<traveler_msgs::msg::SetState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<traveler_msgs::msg::SetState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__traveler_msgs__msg__SetState
    std::shared_ptr<traveler_msgs::msg::SetState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__traveler_msgs__msg__SetState
    std::shared_ptr<traveler_msgs::msg::SetState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetState_ & other) const
  {
    if (this->can_channel != other.can_channel) {
      return false;
    }
    if (this->axis != other.axis) {
      return false;
    }
    if (this->set_state != other.set_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetState_

// alias to use template instance with default allocator
using SetState =
  traveler_msgs::msg::SetState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace traveler_msgs

#endif  // TRAVELER_MSGS__MSG__DETAIL__SET_STATE__STRUCT_HPP_
