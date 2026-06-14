// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from uv_msgs:msg/PidState.idl
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
#include "uv_msgs/msg/detail/pid_state__struct.h"
#include "uv_msgs/msg/detail/pid_state__functions.h"

bool uv_msgs__msg__pid_gains__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * uv_msgs__msg__pid_gains__convert_to_py(void * raw_ros_message);
bool uv_msgs__msg__pid_gains__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * uv_msgs__msg__pid_gains__convert_to_py(void * raw_ros_message);
bool uv_msgs__msg__pid_gains__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * uv_msgs__msg__pid_gains__convert_to_py(void * raw_ros_message);
bool uv_msgs__msg__pid_gains__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * uv_msgs__msg__pid_gains__convert_to_py(void * raw_ros_message);
bool uv_msgs__msg__pid_gains__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * uv_msgs__msg__pid_gains__convert_to_py(void * raw_ros_message);
bool uv_msgs__msg__pid_gains__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * uv_msgs__msg__pid_gains__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool uv_msgs__msg__pid_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[32];
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
    assert(strncmp("uv_msgs.msg._pid_state.PidState", full_classname_dest, 31) == 0);
  }
  uv_msgs__msg__PidState * ros_message = _ros_message;
  {  // speed_mag
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_mag");
    if (!field) {
      return false;
    }
    if (!uv_msgs__msg__pid_gains__convert_from_py(field, &ros_message->speed_mag)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // vel_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_x");
    if (!field) {
      return false;
    }
    if (!uv_msgs__msg__pid_gains__convert_from_py(field, &ros_message->vel_x)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // vel_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_y");
    if (!field) {
      return false;
    }
    if (!uv_msgs__msg__pid_gains__convert_from_py(field, &ros_message->vel_y)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // pos_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_z");
    if (!field) {
      return false;
    }
    if (!uv_msgs__msg__pid_gains__convert_from_py(field, &ros_message->pos_z)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    if (!uv_msgs__msg__pid_gains__convert_from_py(field, &ros_message->yaw)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // yaw_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_rate");
    if (!field) {
      return false;
    }
    if (!uv_msgs__msg__pid_gains__convert_from_py(field, &ros_message->yaw_rate)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // error_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->error_length = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // error_angle_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_angle_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->error_angle_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed_cmd
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_cmd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_cmd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vx_body_cmd
    PyObject * field = PyObject_GetAttrString(_pymsg, "vx_body_cmd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vx_body_cmd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vy_body_cmd
    PyObject * field = PyObject_GetAttrString(_pymsg, "vy_body_cmd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vy_body_cmd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * uv_msgs__msg__pid_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PidState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("uv_msgs.msg._pid_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PidState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  uv_msgs__msg__PidState * ros_message = (uv_msgs__msg__PidState *)raw_ros_message;
  {  // speed_mag
    PyObject * field = NULL;
    field = uv_msgs__msg__pid_gains__convert_to_py(&ros_message->speed_mag);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_mag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_x
    PyObject * field = NULL;
    field = uv_msgs__msg__pid_gains__convert_to_py(&ros_message->vel_x);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_y
    PyObject * field = NULL;
    field = uv_msgs__msg__pid_gains__convert_to_py(&ros_message->vel_y);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_z
    PyObject * field = NULL;
    field = uv_msgs__msg__pid_gains__convert_to_py(&ros_message->pos_z);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = uv_msgs__msg__pid_gains__convert_to_py(&ros_message->yaw);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_rate
    PyObject * field = NULL;
    field = uv_msgs__msg__pid_gains__convert_to_py(&ros_message->yaw_rate);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->error_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_angle_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->error_angle_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_angle_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_cmd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_cmd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_cmd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vx_body_cmd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vx_body_cmd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vx_body_cmd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vy_body_cmd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vy_body_cmd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vy_body_cmd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
