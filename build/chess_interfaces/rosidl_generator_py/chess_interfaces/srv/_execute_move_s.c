// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from chess_interfaces:srv/ExecuteMove.idl
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
#include "chess_interfaces/srv/detail/execute_move__struct.h"
#include "chess_interfaces/srv/detail/execute_move__functions.h"

bool chess_interfaces__msg__chess_move__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * chess_interfaces__msg__chess_move__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool chess_interfaces__srv__execute_move__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[55];
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
    assert(strncmp("chess_interfaces.srv._execute_move.ExecuteMove_Request", full_classname_dest, 54) == 0);
  }
  chess_interfaces__srv__ExecuteMove_Request * ros_message = _ros_message;
  {  // chess_move
    PyObject * field = PyObject_GetAttrString(_pymsg, "chess_move");
    if (!field) {
      return false;
    }
    if (!chess_interfaces__msg__chess_move__convert_from_py(field, &ros_message->chess_move)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // confirm_execution
    PyObject * field = PyObject_GetAttrString(_pymsg, "confirm_execution");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->confirm_execution = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * chess_interfaces__srv__execute_move__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ExecuteMove_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("chess_interfaces.srv._execute_move");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ExecuteMove_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  chess_interfaces__srv__ExecuteMove_Request * ros_message = (chess_interfaces__srv__ExecuteMove_Request *)raw_ros_message;
  {  // chess_move
    PyObject * field = NULL;
    field = chess_interfaces__msg__chess_move__convert_to_py(&ros_message->chess_move);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "chess_move", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confirm_execution
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->confirm_execution ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confirm_execution", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "chess_interfaces/srv/detail/execute_move__struct.h"
// already included above
// #include "chess_interfaces/srv/detail/execute_move__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool chess_interfaces__srv__execute_move__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("chess_interfaces.srv._execute_move.ExecuteMove_Response", full_classname_dest, 55) == 0);
  }
  chess_interfaces__srv__ExecuteMove_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // error_message
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->error_message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // actual_execution_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "actual_execution_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->actual_execution_time = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // piece_captured
    PyObject * field = PyObject_GetAttrString(_pymsg, "piece_captured");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->piece_captured = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * chess_interfaces__srv__execute_move__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ExecuteMove_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("chess_interfaces.srv._execute_move");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ExecuteMove_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  chess_interfaces__srv__ExecuteMove_Response * ros_message = (chess_interfaces__srv__ExecuteMove_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->error_message.data,
      strlen(ros_message->error_message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // actual_execution_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->actual_execution_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "actual_execution_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // piece_captured
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->piece_captured ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "piece_captured", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
