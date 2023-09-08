// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from traveler_msgs:msg/TravelerMode.idl
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
#include "traveler_msgs/msg/detail/traveler_mode__struct.h"
#include "traveler_msgs/msg/detail/traveler_mode__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool traveler_msgs__msg__traveler_mode__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("traveler_msgs.msg._traveler_mode.TravelerMode", full_classname_dest, 45) == 0);
  }
  traveler_msgs__msg__TravelerMode * ros_message = _ros_message;
  {  // start_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->start_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // traveler_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "traveler_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->traveler_mode = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * traveler_msgs__msg__traveler_mode__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TravelerMode */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("traveler_msgs.msg._traveler_mode");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TravelerMode");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  traveler_msgs__msg__TravelerMode * ros_message = (traveler_msgs__msg__TravelerMode *)raw_ros_message;
  {  // start_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->start_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // traveler_mode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->traveler_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "traveler_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}