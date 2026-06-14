// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from uv_msgs:msg/ObjectPosition.idl
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
#include "uv_msgs/msg/detail/object_position__struct.h"
#include "uv_msgs/msg/detail/object_position__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool uv_msgs__msg__object_position__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("uv_msgs.msg._object_position.ObjectPosition", full_classname_dest, 43) == 0);
  }
  uv_msgs__msg__ObjectPosition * ros_message = _ros_message;
  {  // class_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "class_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->class_id = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // class_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "class_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->class_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // world_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "world_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->world_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // world_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "world_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->world_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // world_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "world_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->world_z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // num_observations
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_observations");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_observations = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * uv_msgs__msg__object_position__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ObjectPosition */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("uv_msgs.msg._object_position");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ObjectPosition");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  uv_msgs__msg__ObjectPosition * ros_message = (uv_msgs__msg__ObjectPosition *)raw_ros_message;
  {  // class_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->class_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "class_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // class_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->class_name.data,
      strlen(ros_message->class_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "class_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // world_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->world_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "world_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // world_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->world_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "world_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // world_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->world_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "world_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_observations
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_observations);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_observations", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
