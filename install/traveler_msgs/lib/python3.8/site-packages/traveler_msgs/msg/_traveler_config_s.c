// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from traveler_msgs:msg/TravelerConfig.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "traveler_msgs/msg/detail/traveler_config__struct.h"
#include "traveler_msgs/msg/detail/traveler_config__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool traveler_msgs__msg__traveler_config__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("traveler_msgs.msg._traveler_config.TravelerConfig", full_classname_dest, 49) == 0);
  }
  traveler_msgs__msg__TravelerConfig * ros_message = _ros_message;
  {  // running_scenario
    PyObject * field = PyObject_GetAttrString(_pymsg, "running_scenario");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->running_scenario, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // filename
    PyObject * field = PyObject_GetAttrString(_pymsg, "filename");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->filename, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // extrude_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "extrude_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->extrude_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // extrude_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "extrude_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->extrude_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // extrude_depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "extrude_depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->extrude_depth = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shear_penetration_depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "shear_penetration_depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shear_penetration_depth = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shear_penetration_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "shear_penetration_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shear_penetration_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shear_penetration_delay
    PyObject * field = PyObject_GetAttrString(_pymsg, "shear_penetration_delay");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shear_penetration_delay = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shear_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "shear_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shear_length = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shear_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "shear_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shear_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shear_delay
    PyObject * field = PyObject_GetAttrString(_pymsg, "shear_delay");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shear_delay = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shear_return_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "shear_return_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shear_return_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // workspace_angular_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "workspace_angular_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->workspace_angular_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // workspace_moving_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "workspace_moving_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->workspace_moving_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // workspace_time_delay
    PyObject * field = PyObject_GetAttrString(_pymsg, "workspace_time_delay");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->workspace_time_delay = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // static_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "static_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->static_length = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // static_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "static_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->static_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // search_start
    PyObject * field = PyObject_GetAttrString(_pymsg, "search_start");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->search_start = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // search_end
    PyObject * field = PyObject_GetAttrString(_pymsg, "search_end");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->search_end = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ground_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "ground_height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ground_height = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // back_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "back_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->back_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * traveler_msgs__msg__traveler_config__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TravelerConfig */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("traveler_msgs.msg._traveler_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TravelerConfig");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  traveler_msgs__msg__TravelerConfig * ros_message = (traveler_msgs__msg__TravelerConfig *)raw_ros_message;
  {  // running_scenario
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->running_scenario.data,
      strlen(ros_message->running_scenario.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "running_scenario", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // filename
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->filename.data,
      strlen(ros_message->filename.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "filename", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // extrude_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->extrude_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "extrude_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // extrude_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->extrude_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "extrude_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // extrude_depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->extrude_depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "extrude_depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shear_penetration_depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shear_penetration_depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shear_penetration_depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shear_penetration_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shear_penetration_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shear_penetration_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shear_penetration_delay
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shear_penetration_delay);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shear_penetration_delay", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shear_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shear_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shear_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shear_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shear_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shear_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shear_delay
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shear_delay);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shear_delay", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shear_return_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shear_return_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shear_return_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // workspace_angular_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->workspace_angular_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "workspace_angular_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // workspace_moving_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->workspace_moving_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "workspace_moving_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // workspace_time_delay
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->workspace_time_delay);
    {
      int rc = PyObject_SetAttrString(_pymessage, "workspace_time_delay", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // static_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->static_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "static_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // static_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->static_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "static_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // search_start
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->search_start);
    {
      int rc = PyObject_SetAttrString(_pymessage, "search_start", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // search_end
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->search_end);
    {
      int rc = PyObject_SetAttrString(_pymessage, "search_end", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ground_height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ground_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ground_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // back_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->back_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "back_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
