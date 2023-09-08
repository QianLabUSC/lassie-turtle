// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from traveler_msgs:msg/TravelerStatus.idl
// generated code does not contain a copyright notice
#include "traveler_msgs/msg/detail/traveler_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "traveler_msgs/msg/detail/traveler_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace traveler_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traveler_msgs
cdr_serialize(
  const traveler_msgs::msg::TravelerStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: time
  cdr << ros_message.time;
  // Member: toeforce_x
  cdr << ros_message.toeforce_x;
  // Member: toeforce_y
  cdr << ros_message.toeforce_y;
  // Member: toe_pos_x
  cdr << ros_message.toe_pos_x;
  // Member: toe_pos_y
  cdr << ros_message.toe_pos_y;
  // Member: motor0_pos
  cdr << ros_message.motor0_pos;
  // Member: motor1_pos
  cdr << ros_message.motor1_pos;
  // Member: motor0_torque
  cdr << ros_message.motor0_torque;
  // Member: motor1_torque
  cdr << ros_message.motor1_torque;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traveler_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  traveler_msgs::msg::TravelerStatus & ros_message)
{
  // Member: time
  cdr >> ros_message.time;

  // Member: toeforce_x
  cdr >> ros_message.toeforce_x;

  // Member: toeforce_y
  cdr >> ros_message.toeforce_y;

  // Member: toe_pos_x
  cdr >> ros_message.toe_pos_x;

  // Member: toe_pos_y
  cdr >> ros_message.toe_pos_y;

  // Member: motor0_pos
  cdr >> ros_message.motor0_pos;

  // Member: motor1_pos
  cdr >> ros_message.motor1_pos;

  // Member: motor0_torque
  cdr >> ros_message.motor0_torque;

  // Member: motor1_torque
  cdr >> ros_message.motor1_torque;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traveler_msgs
get_serialized_size(
  const traveler_msgs::msg::TravelerStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: time
  {
    size_t item_size = sizeof(ros_message.time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: toeforce_x
  {
    size_t item_size = sizeof(ros_message.toeforce_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: toeforce_y
  {
    size_t item_size = sizeof(ros_message.toeforce_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: toe_pos_x
  {
    size_t item_size = sizeof(ros_message.toe_pos_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: toe_pos_y
  {
    size_t item_size = sizeof(ros_message.toe_pos_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor0_pos
  {
    size_t item_size = sizeof(ros_message.motor0_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor1_pos
  {
    size_t item_size = sizeof(ros_message.motor1_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor0_torque
  {
    size_t item_size = sizeof(ros_message.motor0_torque);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor1_torque
  {
    size_t item_size = sizeof(ros_message.motor1_torque);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traveler_msgs
max_serialized_size_TravelerStatus(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: toeforce_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: toeforce_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: toe_pos_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: toe_pos_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: motor0_pos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: motor1_pos
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: motor0_torque
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: motor1_torque
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _TravelerStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const traveler_msgs::msg::TravelerStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TravelerStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<traveler_msgs::msg::TravelerStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TravelerStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const traveler_msgs::msg::TravelerStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TravelerStatus__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_TravelerStatus(full_bounded, 0);
}

static message_type_support_callbacks_t _TravelerStatus__callbacks = {
  "traveler_msgs::msg",
  "TravelerStatus",
  _TravelerStatus__cdr_serialize,
  _TravelerStatus__cdr_deserialize,
  _TravelerStatus__get_serialized_size,
  _TravelerStatus__max_serialized_size
};

static rosidl_message_type_support_t _TravelerStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TravelerStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace traveler_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_traveler_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<traveler_msgs::msg::TravelerStatus>()
{
  return &traveler_msgs::msg::typesupport_fastrtps_cpp::_TravelerStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, traveler_msgs, msg, TravelerStatus)() {
  return &traveler_msgs::msg::typesupport_fastrtps_cpp::_TravelerStatus__handle;
}

#ifdef __cplusplus
}
#endif
