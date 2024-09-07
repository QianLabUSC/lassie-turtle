// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from traveler_msgs:msg/TravelerConfig.idl
// generated code does not contain a copyright notice
#include "traveler_msgs/msg/detail/traveler_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `running_scenario`
// Member `filename`
#include "rosidl_runtime_c/string_functions.h"

bool
traveler_msgs__msg__TravelerConfig__init(traveler_msgs__msg__TravelerConfig * msg)
{
  if (!msg) {
    return false;
  }
  // running_scenario
  if (!rosidl_runtime_c__String__init(&msg->running_scenario)) {
    traveler_msgs__msg__TravelerConfig__fini(msg);
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__init(&msg->filename)) {
    traveler_msgs__msg__TravelerConfig__fini(msg);
    return false;
  }
  // extrude_speed
  // extrude_angle
  // extrude_depth
  // shear_penetration_depth
  // shear_penetration_speed
  // shear_penetration_delay
  // shear_length
  // shear_speed
  // shear_delay
  // shear_return_speed
  // workspace_angular_speed
  // workspace_moving_angle
  // workspace_time_delay
  // static_length
  // static_angle
  // search_start
  // search_end
  // ground_height
  // back_speed
  return true;
}

void
traveler_msgs__msg__TravelerConfig__fini(traveler_msgs__msg__TravelerConfig * msg)
{
  if (!msg) {
    return;
  }
  // running_scenario
  rosidl_runtime_c__String__fini(&msg->running_scenario);
  // filename
  rosidl_runtime_c__String__fini(&msg->filename);
  // extrude_speed
  // extrude_angle
  // extrude_depth
  // shear_penetration_depth
  // shear_penetration_speed
  // shear_penetration_delay
  // shear_length
  // shear_speed
  // shear_delay
  // shear_return_speed
  // workspace_angular_speed
  // workspace_moving_angle
  // workspace_time_delay
  // static_length
  // static_angle
  // search_start
  // search_end
  // ground_height
  // back_speed
}

bool
traveler_msgs__msg__TravelerConfig__are_equal(const traveler_msgs__msg__TravelerConfig * lhs, const traveler_msgs__msg__TravelerConfig * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // running_scenario
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->running_scenario), &(rhs->running_scenario)))
  {
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->filename), &(rhs->filename)))
  {
    return false;
  }
  // extrude_speed
  if (lhs->extrude_speed != rhs->extrude_speed) {
    return false;
  }
  // extrude_angle
  if (lhs->extrude_angle != rhs->extrude_angle) {
    return false;
  }
  // extrude_depth
  if (lhs->extrude_depth != rhs->extrude_depth) {
    return false;
  }
  // shear_penetration_depth
  if (lhs->shear_penetration_depth != rhs->shear_penetration_depth) {
    return false;
  }
  // shear_penetration_speed
  if (lhs->shear_penetration_speed != rhs->shear_penetration_speed) {
    return false;
  }
  // shear_penetration_delay
  if (lhs->shear_penetration_delay != rhs->shear_penetration_delay) {
    return false;
  }
  // shear_length
  if (lhs->shear_length != rhs->shear_length) {
    return false;
  }
  // shear_speed
  if (lhs->shear_speed != rhs->shear_speed) {
    return false;
  }
  // shear_delay
  if (lhs->shear_delay != rhs->shear_delay) {
    return false;
  }
  // shear_return_speed
  if (lhs->shear_return_speed != rhs->shear_return_speed) {
    return false;
  }
  // workspace_angular_speed
  if (lhs->workspace_angular_speed != rhs->workspace_angular_speed) {
    return false;
  }
  // workspace_moving_angle
  if (lhs->workspace_moving_angle != rhs->workspace_moving_angle) {
    return false;
  }
  // workspace_time_delay
  if (lhs->workspace_time_delay != rhs->workspace_time_delay) {
    return false;
  }
  // static_length
  if (lhs->static_length != rhs->static_length) {
    return false;
  }
  // static_angle
  if (lhs->static_angle != rhs->static_angle) {
    return false;
  }
  // search_start
  if (lhs->search_start != rhs->search_start) {
    return false;
  }
  // search_end
  if (lhs->search_end != rhs->search_end) {
    return false;
  }
  // ground_height
  if (lhs->ground_height != rhs->ground_height) {
    return false;
  }
  // back_speed
  if (lhs->back_speed != rhs->back_speed) {
    return false;
  }
  return true;
}

bool
traveler_msgs__msg__TravelerConfig__copy(
  const traveler_msgs__msg__TravelerConfig * input,
  traveler_msgs__msg__TravelerConfig * output)
{
  if (!input || !output) {
    return false;
  }
  // running_scenario
  if (!rosidl_runtime_c__String__copy(
      &(input->running_scenario), &(output->running_scenario)))
  {
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__copy(
      &(input->filename), &(output->filename)))
  {
    return false;
  }
  // extrude_speed
  output->extrude_speed = input->extrude_speed;
  // extrude_angle
  output->extrude_angle = input->extrude_angle;
  // extrude_depth
  output->extrude_depth = input->extrude_depth;
  // shear_penetration_depth
  output->shear_penetration_depth = input->shear_penetration_depth;
  // shear_penetration_speed
  output->shear_penetration_speed = input->shear_penetration_speed;
  // shear_penetration_delay
  output->shear_penetration_delay = input->shear_penetration_delay;
  // shear_length
  output->shear_length = input->shear_length;
  // shear_speed
  output->shear_speed = input->shear_speed;
  // shear_delay
  output->shear_delay = input->shear_delay;
  // shear_return_speed
  output->shear_return_speed = input->shear_return_speed;
  // workspace_angular_speed
  output->workspace_angular_speed = input->workspace_angular_speed;
  // workspace_moving_angle
  output->workspace_moving_angle = input->workspace_moving_angle;
  // workspace_time_delay
  output->workspace_time_delay = input->workspace_time_delay;
  // static_length
  output->static_length = input->static_length;
  // static_angle
  output->static_angle = input->static_angle;
  // search_start
  output->search_start = input->search_start;
  // search_end
  output->search_end = input->search_end;
  // ground_height
  output->ground_height = input->ground_height;
  // back_speed
  output->back_speed = input->back_speed;
  return true;
}

traveler_msgs__msg__TravelerConfig *
traveler_msgs__msg__TravelerConfig__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traveler_msgs__msg__TravelerConfig * msg = (traveler_msgs__msg__TravelerConfig *)allocator.allocate(sizeof(traveler_msgs__msg__TravelerConfig), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(traveler_msgs__msg__TravelerConfig));
  bool success = traveler_msgs__msg__TravelerConfig__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
traveler_msgs__msg__TravelerConfig__destroy(traveler_msgs__msg__TravelerConfig * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    traveler_msgs__msg__TravelerConfig__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
traveler_msgs__msg__TravelerConfig__Sequence__init(traveler_msgs__msg__TravelerConfig__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traveler_msgs__msg__TravelerConfig * data = NULL;

  if (size) {
    data = (traveler_msgs__msg__TravelerConfig *)allocator.zero_allocate(size, sizeof(traveler_msgs__msg__TravelerConfig), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = traveler_msgs__msg__TravelerConfig__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        traveler_msgs__msg__TravelerConfig__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
traveler_msgs__msg__TravelerConfig__Sequence__fini(traveler_msgs__msg__TravelerConfig__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      traveler_msgs__msg__TravelerConfig__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

traveler_msgs__msg__TravelerConfig__Sequence *
traveler_msgs__msg__TravelerConfig__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traveler_msgs__msg__TravelerConfig__Sequence * array = (traveler_msgs__msg__TravelerConfig__Sequence *)allocator.allocate(sizeof(traveler_msgs__msg__TravelerConfig__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = traveler_msgs__msg__TravelerConfig__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
traveler_msgs__msg__TravelerConfig__Sequence__destroy(traveler_msgs__msg__TravelerConfig__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    traveler_msgs__msg__TravelerConfig__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
traveler_msgs__msg__TravelerConfig__Sequence__are_equal(const traveler_msgs__msg__TravelerConfig__Sequence * lhs, const traveler_msgs__msg__TravelerConfig__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!traveler_msgs__msg__TravelerConfig__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
traveler_msgs__msg__TravelerConfig__Sequence__copy(
  const traveler_msgs__msg__TravelerConfig__Sequence * input,
  traveler_msgs__msg__TravelerConfig__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(traveler_msgs__msg__TravelerConfig);
    traveler_msgs__msg__TravelerConfig * data =
      (traveler_msgs__msg__TravelerConfig *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!traveler_msgs__msg__TravelerConfig__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          traveler_msgs__msg__TravelerConfig__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!traveler_msgs__msg__TravelerConfig__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
