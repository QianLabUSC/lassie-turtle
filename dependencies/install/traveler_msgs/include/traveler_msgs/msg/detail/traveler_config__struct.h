// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from traveler_msgs:msg/TravelerConfig.idl
// generated code does not contain a copyright notice

#ifndef TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__STRUCT_H_
#define TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'running_scenario'
// Member 'filename'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/TravelerConfig in the package traveler_msgs.
typedef struct traveler_msgs__msg__TravelerConfig
{
  rosidl_runtime_c__String running_scenario;
  rosidl_runtime_c__String filename;
  float extrude_speed;
  float extrude_angle;
  float extrude_depth;
  float shear_penetration_depth;
  float shear_penetration_speed;
  float shear_penetration_delay;
  float shear_length;
  float shear_speed;
  float shear_delay;
  float shear_return_speed;
  float workspace_angular_speed;
  float workspace_moving_angle;
  float workspace_time_delay;
  float static_length;
  float static_angle;
  float search_start;
  float search_end;
  float ground_height;
  float back_speed;
} traveler_msgs__msg__TravelerConfig;

// Struct for a sequence of traveler_msgs__msg__TravelerConfig.
typedef struct traveler_msgs__msg__TravelerConfig__Sequence
{
  traveler_msgs__msg__TravelerConfig * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} traveler_msgs__msg__TravelerConfig__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAVELER_MSGS__MSG__DETAIL__TRAVELER_CONFIG__STRUCT_H_
