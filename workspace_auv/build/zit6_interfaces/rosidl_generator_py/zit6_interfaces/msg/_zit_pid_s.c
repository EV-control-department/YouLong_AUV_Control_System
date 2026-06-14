// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from zit6_interfaces:msg/ZitPid.idl
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
#include "zit6_interfaces/msg/detail/zit_pid__struct.h"
#include "zit6_interfaces/msg/detail/zit_pid__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool zit6_interfaces__msg__zit_pid__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
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
    assert(strncmp("zit6_interfaces.msg._zit_pid.ZitPid", full_classname_dest, 35) == 0);
  }
  zit6_interfaces__msg__ZitPid * ros_message = _ros_message;
  {  // axis
    PyObject * field = PyObject_GetAttrString(_pymsg, "axis");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->axis = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // is_pos_ring
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_pos_ring");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_pos_ring = (Py_True == field);
    Py_DECREF(field);
  }
  {  // kp
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ki
    PyObject * field = PyObject_GetAttrString(_pymsg, "ki");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ki = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kd
    PyObject * field = PyObject_GetAttrString(_pymsg, "kd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // i_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "i_limit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->i_limit = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // out_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "out_limit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->out_limit = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_v
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_v");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_v = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_a
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_a");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_a = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * zit6_interfaces__msg__zit_pid__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ZitPid */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("zit6_interfaces.msg._zit_pid");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ZitPid");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  zit6_interfaces__msg__ZitPid * ros_message = (zit6_interfaces__msg__ZitPid *)raw_ros_message;
  {  // axis
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->axis);
    {
      int rc = PyObject_SetAttrString(_pymessage, "axis", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_pos_ring
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_pos_ring ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_pos_ring", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ki
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ki);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ki", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // i_limit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->i_limit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "i_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // out_limit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->out_limit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "out_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_v
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_v);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_v", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_a
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_a);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_a", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
