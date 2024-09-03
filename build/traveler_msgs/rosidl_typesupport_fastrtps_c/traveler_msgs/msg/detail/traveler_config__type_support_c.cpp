// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from traveler_msgs:msg/TravelerConfig.idl
// generated code does not contain a copyright notice
#include "traveler_msgs/msg/detail/traveler_config__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "traveler_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "traveler_msgs/msg/detail/traveler_config__struct.h"
#include "traveler_msgs/msg/detail/traveler_config__functions.h"
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

#include "rosidl_runtime_c/string.h"  // filename, running_scenario
#include "rosidl_runtime_c/string_functions.h"  // filename, running_scenario

// forward declare type support functions


using _TravelerConfig__ros_msg_type = traveler_msgs__msg__TravelerConfig;

static bool _TravelerConfig__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TravelerConfig__ros_msg_type * ros_message = static_cast<const _TravelerConfig__ros_msg_type *>(untyped_ros_message);
  // Field name: running_scenario
  {
    const rosidl_runtime_c__String * str = &ros_message->running_scenario;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: filename
  {
    const rosidl_runtime_c__String * str = &ros_message->filename;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: extrude_speed
  {
    cdr << ros_message->extrude_speed;
  }

  // Field name: extrude_angle
  {
    cdr << ros_message->extrude_angle;
  }

  // Field name: extrude_depth
  {
    cdr << ros_message->extrude_depth;
  }

  // Field name: shear_penetration_depth
  {
    cdr << ros_message->shear_penetration_depth;
  }

  // Field name: shear_penetration_speed
  {
    cdr << ros_message->shear_penetration_speed;
  }

  // Field name: shear_penetration_delay
  {
    cdr << ros_message->shear_penetration_delay;
  }

  // Field name: shear_length
  {
    cdr << ros_message->shear_length;
  }

  // Field name: shear_speed
  {
    cdr << ros_message->shear_speed;
  }

  // Field name: shear_delay
  {
    cdr << ros_message->shear_delay;
  }

  // Field name: shear_return_speed
  {
    cdr << ros_message->shear_return_speed;
  }

  // Field name: workspace_angular_speed
  {
    cdr << ros_message->workspace_angular_speed;
  }

  // Field name: workspace_moving_angle
  {
    cdr << ros_message->workspace_moving_angle;
  }

  // Field name: workspace_time_delay
  {
    cdr << ros_message->workspace_time_delay;
  }

  // Field name: static_length
  {
    cdr << ros_message->static_length;
  }

  // Field name: static_angle
  {
    cdr << ros_message->static_angle;
  }

  // Field name: search_start
  {
    cdr << ros_message->search_start;
  }

  // Field name: search_end
  {
    cdr << ros_message->search_end;
  }

  // Field name: ground_height
  {
    cdr << ros_message->ground_height;
  }

  // Field name: back_speed
  {
    cdr << ros_message->back_speed;
  }

  return true;
}

static bool _TravelerConfig__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TravelerConfig__ros_msg_type * ros_message = static_cast<_TravelerConfig__ros_msg_type *>(untyped_ros_message);
  // Field name: running_scenario
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->running_scenario.data) {
      rosidl_runtime_c__String__init(&ros_message->running_scenario);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->running_scenario,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'running_scenario'\n");
      return false;
    }
  }

  // Field name: filename
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->filename.data) {
      rosidl_runtime_c__String__init(&ros_message->filename);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->filename,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'filename'\n");
      return false;
    }
  }

  // Field name: extrude_speed
  {
    cdr >> ros_message->extrude_speed;
  }

  // Field name: extrude_angle
  {
    cdr >> ros_message->extrude_angle;
  }

  // Field name: extrude_depth
  {
    cdr >> ros_message->extrude_depth;
  }

  // Field name: shear_penetration_depth
  {
    cdr >> ros_message->shear_penetration_depth;
  }

  // Field name: shear_penetration_speed
  {
    cdr >> ros_message->shear_penetration_speed;
  }

  // Field name: shear_penetration_delay
  {
    cdr >> ros_message->shear_penetration_delay;
  }

  // Field name: shear_length
  {
    cdr >> ros_message->shear_length;
  }

  // Field name: shear_speed
  {
    cdr >> ros_message->shear_speed;
  }

  // Field name: shear_delay
  {
    cdr >> ros_message->shear_delay;
  }

  // Field name: shear_return_speed
  {
    cdr >> ros_message->shear_return_speed;
  }

  // Field name: workspace_angular_speed
  {
    cdr >> ros_message->workspace_angular_speed;
  }

  // Field name: workspace_moving_angle
  {
    cdr >> ros_message->workspace_moving_angle;
  }

  // Field name: workspace_time_delay
  {
    cdr >> ros_message->workspace_time_delay;
  }

  // Field name: static_length
  {
    cdr >> ros_message->static_length;
  }

  // Field name: static_angle
  {
    cdr >> ros_message->static_angle;
  }

  // Field name: search_start
  {
    cdr >> ros_message->search_start;
  }

  // Field name: search_end
  {
    cdr >> ros_message->search_end;
  }

  // Field name: ground_height
  {
    cdr >> ros_message->ground_height;
  }

  // Field name: back_speed
  {
    cdr >> ros_message->back_speed;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_traveler_msgs
size_t get_serialized_size_traveler_msgs__msg__TravelerConfig(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TravelerConfig__ros_msg_type * ros_message = static_cast<const _TravelerConfig__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name running_scenario
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->running_scenario.size + 1);
  // field.name filename
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->filename.size + 1);
  // field.name extrude_speed
  {
    size_t item_size = sizeof(ros_message->extrude_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name extrude_angle
  {
    size_t item_size = sizeof(ros_message->extrude_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name extrude_depth
  {
    size_t item_size = sizeof(ros_message->extrude_depth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name shear_penetration_depth
  {
    size_t item_size = sizeof(ros_message->shear_penetration_depth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name shear_penetration_speed
  {
    size_t item_size = sizeof(ros_message->shear_penetration_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name shear_penetration_delay
  {
    size_t item_size = sizeof(ros_message->shear_penetration_delay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name shear_length
  {
    size_t item_size = sizeof(ros_message->shear_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name shear_speed
  {
    size_t item_size = sizeof(ros_message->shear_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name shear_delay
  {
    size_t item_size = sizeof(ros_message->shear_delay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name shear_return_speed
  {
    size_t item_size = sizeof(ros_message->shear_return_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name workspace_angular_speed
  {
    size_t item_size = sizeof(ros_message->workspace_angular_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name workspace_moving_angle
  {
    size_t item_size = sizeof(ros_message->workspace_moving_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name workspace_time_delay
  {
    size_t item_size = sizeof(ros_message->workspace_time_delay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name static_length
  {
    size_t item_size = sizeof(ros_message->static_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name static_angle
  {
    size_t item_size = sizeof(ros_message->static_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name search_start
  {
    size_t item_size = sizeof(ros_message->search_start);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name search_end
  {
    size_t item_size = sizeof(ros_message->search_end);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ground_height
  {
    size_t item_size = sizeof(ros_message->ground_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name back_speed
  {
    size_t item_size = sizeof(ros_message->back_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TravelerConfig__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_traveler_msgs__msg__TravelerConfig(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_traveler_msgs
size_t max_serialized_size_traveler_msgs__msg__TravelerConfig(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: running_scenario
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: filename
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: extrude_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: extrude_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: extrude_depth
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: shear_penetration_depth
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: shear_penetration_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: shear_penetration_delay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: shear_length
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: shear_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: shear_delay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: shear_return_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: workspace_angular_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: workspace_moving_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: workspace_time_delay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: static_length
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: static_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: search_start
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: search_end
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ground_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: back_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _TravelerConfig__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_traveler_msgs__msg__TravelerConfig(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_TravelerConfig = {
  "traveler_msgs::msg",
  "TravelerConfig",
  _TravelerConfig__cdr_serialize,
  _TravelerConfig__cdr_deserialize,
  _TravelerConfig__get_serialized_size,
  _TravelerConfig__max_serialized_size
};

static rosidl_message_type_support_t _TravelerConfig__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TravelerConfig,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, traveler_msgs, msg, TravelerConfig)() {
  return &_TravelerConfig__type_support;
}

#if defined(__cplusplus)
}
#endif
