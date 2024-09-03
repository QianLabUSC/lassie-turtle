// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from traveler_msgs:msg/SetState.idl
// generated code does not contain a copyright notice

#ifndef TRAVELER_MSGS__MSG__DETAIL__SET_STATE__STRUCT_H_
#define TRAVELER_MSGS__MSG__DETAIL__SET_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SetState in the package traveler_msgs.
typedef struct traveler_msgs__msg__SetState
{
  uint8_t can_channel;
  uint8_t axis;
  uint32_t set_state;
} traveler_msgs__msg__SetState;

// Struct for a sequence of traveler_msgs__msg__SetState.
typedef struct traveler_msgs__msg__SetState__Sequence
{
  traveler_msgs__msg__SetState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} traveler_msgs__msg__SetState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAVELER_MSGS__MSG__DETAIL__SET_STATE__STRUCT_H_
