// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from traveler_msgs:msg/SetState.idl
// generated code does not contain a copyright notice
#include "traveler_msgs/msg/detail/set_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "traveler_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "traveler_msgs/msg/detail/set_state__struct.h"
#include "traveler_msgs/msg/detail/set_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SetState__ros_msg_type = traveler_msgs__msg__SetState;

static bool _SetState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetState__ros_msg_type * ros_message = static_cast<const _SetState__ros_msg_type *>(untyped_ros_message);
  // Field name: can_channel
  {
    cdr << ros_message->can_channel;
  }

  // Field name: axis
  {
    cdr << ros_message->axis;
  }

  // Field name: set_state
  {
    cdr << ros_message->set_state;
  }

  return true;
}

static bool _SetState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetState__ros_msg_type * ros_message = static_cast<_SetState__ros_msg_type *>(untyped_ros_message);
  // Field name: can_channel
  {
    cdr >> ros_message->can_channel;
  }

  // Field name: axis
  {
    cdr >> ros_message->axis;
  }

  // Field name: set_state
  {
    cdr >> ros_message->set_state;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_traveler_msgs
size_t get_serialized_size_traveler_msgs__msg__SetState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetState__ros_msg_type * ros_message = static_cast<const _SetState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name can_channel
  {
    size_t item_size = sizeof(ros_message->can_channel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name axis
  {
    size_t item_size = sizeof(ros_message->axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name set_state
  {
    size_t item_size = sizeof(ros_message->set_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SetState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_traveler_msgs__msg__SetState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_traveler_msgs
size_t max_serialized_size_traveler_msgs__msg__SetState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: can_channel
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: axis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: set_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = traveler_msgs__msg__SetState;
    is_plain =
      (
      offsetof(DataType, set_state) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SetState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_traveler_msgs__msg__SetState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetState = {
  "traveler_msgs::msg",
  "SetState",
  _SetState__cdr_serialize,
  _SetState__cdr_deserialize,
  _SetState__get_serialized_size,
  _SetState__max_serialized_size
};

static rosidl_message_type_support_t _SetState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, traveler_msgs, msg, SetState)() {
  return &_SetState__type_support;
}

#if defined(__cplusplus)
}
#endif
