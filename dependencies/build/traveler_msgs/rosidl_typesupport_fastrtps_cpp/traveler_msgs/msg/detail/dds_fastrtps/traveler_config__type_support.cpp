// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from traveler_msgs:msg/TravelerConfig.idl
// generated code does not contain a copyright notice
#include "traveler_msgs/msg/detail/traveler_config__rosidl_typesupport_fastrtps_cpp.hpp"
#include "traveler_msgs/msg/detail/traveler_config__struct.hpp"

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
  const traveler_msgs::msg::TravelerConfig & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: running_scenario
  cdr << ros_message.running_scenario;
  // Member: filename
  cdr << ros_message.filename;
  // Member: extrude_speed
  cdr << ros_message.extrude_speed;
  // Member: extrude_angle
  cdr << ros_message.extrude_angle;
  // Member: extrude_depth
  cdr << ros_message.extrude_depth;
  // Member: shear_penetration_depth
  cdr << ros_message.shear_penetration_depth;
  // Member: shear_penetration_speed
  cdr << ros_message.shear_penetration_speed;
  // Member: shear_penetration_delay
  cdr << ros_message.shear_penetration_delay;
  // Member: shear_length
  cdr << ros_message.shear_length;
  // Member: shear_speed
  cdr << ros_message.shear_speed;
  // Member: shear_delay
  cdr << ros_message.shear_delay;
  // Member: shear_return_speed
  cdr << ros_message.shear_return_speed;
  // Member: workspace_angular_speed
  cdr << ros_message.workspace_angular_speed;
  // Member: workspace_moving_angle
  cdr << ros_message.workspace_moving_angle;
  // Member: workspace_time_delay
  cdr << ros_message.workspace_time_delay;
  // Member: static_length
  cdr << ros_message.static_length;
  // Member: static_angle
  cdr << ros_message.static_angle;
  // Member: search_start
  cdr << ros_message.search_start;
  // Member: search_end
  cdr << ros_message.search_end;
  // Member: ground_height
  cdr << ros_message.ground_height;
  // Member: back_speed
  cdr << ros_message.back_speed;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traveler_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  traveler_msgs::msg::TravelerConfig & ros_message)
{
  // Member: running_scenario
  cdr >> ros_message.running_scenario;

  // Member: filename
  cdr >> ros_message.filename;

  // Member: extrude_speed
  cdr >> ros_message.extrude_speed;

  // Member: extrude_angle
  cdr >> ros_message.extrude_angle;

  // Member: extrude_depth
  cdr >> ros_message.extrude_depth;

  // Member: shear_penetration_depth
  cdr >> ros_message.shear_penetration_depth;

  // Member: shear_penetration_speed
  cdr >> ros_message.shear_penetration_speed;

  // Member: shear_penetration_delay
  cdr >> ros_message.shear_penetration_delay;

  // Member: shear_length
  cdr >> ros_message.shear_length;

  // Member: shear_speed
  cdr >> ros_message.shear_speed;

  // Member: shear_delay
  cdr >> ros_message.shear_delay;

  // Member: shear_return_speed
  cdr >> ros_message.shear_return_speed;

  // Member: workspace_angular_speed
  cdr >> ros_message.workspace_angular_speed;

  // Member: workspace_moving_angle
  cdr >> ros_message.workspace_moving_angle;

  // Member: workspace_time_delay
  cdr >> ros_message.workspace_time_delay;

  // Member: static_length
  cdr >> ros_message.static_length;

  // Member: static_angle
  cdr >> ros_message.static_angle;

  // Member: search_start
  cdr >> ros_message.search_start;

  // Member: search_end
  cdr >> ros_message.search_end;

  // Member: ground_height
  cdr >> ros_message.ground_height;

  // Member: back_speed
  cdr >> ros_message.back_speed;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traveler_msgs
get_serialized_size(
  const traveler_msgs::msg::TravelerConfig & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: running_scenario
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.running_scenario.size() + 1);
  // Member: filename
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.filename.size() + 1);
  // Member: extrude_speed
  {
    size_t item_size = sizeof(ros_message.extrude_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: extrude_angle
  {
    size_t item_size = sizeof(ros_message.extrude_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: extrude_depth
  {
    size_t item_size = sizeof(ros_message.extrude_depth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: shear_penetration_depth
  {
    size_t item_size = sizeof(ros_message.shear_penetration_depth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: shear_penetration_speed
  {
    size_t item_size = sizeof(ros_message.shear_penetration_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: shear_penetration_delay
  {
    size_t item_size = sizeof(ros_message.shear_penetration_delay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: shear_length
  {
    size_t item_size = sizeof(ros_message.shear_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: shear_speed
  {
    size_t item_size = sizeof(ros_message.shear_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: shear_delay
  {
    size_t item_size = sizeof(ros_message.shear_delay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: shear_return_speed
  {
    size_t item_size = sizeof(ros_message.shear_return_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: workspace_angular_speed
  {
    size_t item_size = sizeof(ros_message.workspace_angular_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: workspace_moving_angle
  {
    size_t item_size = sizeof(ros_message.workspace_moving_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: workspace_time_delay
  {
    size_t item_size = sizeof(ros_message.workspace_time_delay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: static_length
  {
    size_t item_size = sizeof(ros_message.static_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: static_angle
  {
    size_t item_size = sizeof(ros_message.static_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: search_start
  {
    size_t item_size = sizeof(ros_message.search_start);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: search_end
  {
    size_t item_size = sizeof(ros_message.search_end);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ground_height
  {
    size_t item_size = sizeof(ros_message.ground_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: back_speed
  {
    size_t item_size = sizeof(ros_message.back_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traveler_msgs
max_serialized_size_TravelerConfig(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: running_scenario
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: filename
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: extrude_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: extrude_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: extrude_depth
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: shear_penetration_depth
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: shear_penetration_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: shear_penetration_delay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: shear_length
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: shear_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: shear_delay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: shear_return_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: workspace_angular_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: workspace_moving_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: workspace_time_delay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: static_length
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: static_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: search_start
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: search_end
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ground_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: back_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _TravelerConfig__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const traveler_msgs::msg::TravelerConfig *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TravelerConfig__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<traveler_msgs::msg::TravelerConfig *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TravelerConfig__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const traveler_msgs::msg::TravelerConfig *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TravelerConfig__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_TravelerConfig(full_bounded, 0);
}

static message_type_support_callbacks_t _TravelerConfig__callbacks = {
  "traveler_msgs::msg",
  "TravelerConfig",
  _TravelerConfig__cdr_serialize,
  _TravelerConfig__cdr_deserialize,
  _TravelerConfig__get_serialized_size,
  _TravelerConfig__max_serialized_size
};

static rosidl_message_type_support_t _TravelerConfig__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TravelerConfig__callbacks,
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
get_message_type_support_handle<traveler_msgs::msg::TravelerConfig>()
{
  return &traveler_msgs::msg::typesupport_fastrtps_cpp::_TravelerConfig__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, traveler_msgs, msg, TravelerConfig)() {
  return &traveler_msgs::msg::typesupport_fastrtps_cpp::_TravelerConfig__handle;
}

#ifdef __cplusplus
}
#endif
