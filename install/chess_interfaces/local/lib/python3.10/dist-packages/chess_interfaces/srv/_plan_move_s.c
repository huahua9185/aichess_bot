// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from chess_interfaces:srv/PlanMove.idl
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
#include "chess_interfaces/srv/detail/plan_move__struct.h"
#include "chess_interfaces/srv/detail/plan_move__functions.h"

bool chess_interfaces__msg__chess_move__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * chess_interfaces__msg__chess_move__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool chess_interfaces__srv__plan_move__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("chess_interfaces.srv._plan_move.PlanMove_Request", full_classname_dest, 48) == 0);
  }
  chess_interfaces__srv__PlanMove_Request * ros_message = _ros_message;
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
  {  // speed_factor
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_factor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_factor = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // avoid_pieces
    PyObject * field = PyObject_GetAttrString(_pymsg, "avoid_pieces");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->avoid_pieces = (Py_True == field);
    Py_DECREF(field);
  }
  {  // use_safe_approach
    PyObject * field = PyObject_GetAttrString(_pymsg, "use_safe_approach");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->use_safe_approach = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * chess_interfaces__srv__plan_move__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PlanMove_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("chess_interfaces.srv._plan_move");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PlanMove_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  chess_interfaces__srv__PlanMove_Request * ros_message = (chess_interfaces__srv__PlanMove_Request *)raw_ros_message;
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
  {  // speed_factor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_factor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_factor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // avoid_pieces
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->avoid_pieces ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "avoid_pieces", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // use_safe_approach
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->use_safe_approach ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "use_safe_approach", field);
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
// #include "chess_interfaces/srv/detail/plan_move__struct.h"
// already included above
// #include "chess_interfaces/srv/detail/plan_move__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool chess_interfaces__srv__plan_move__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("chess_interfaces.srv._plan_move.PlanMove_Response", full_classname_dest, 49) == 0);
  }
  chess_interfaces__srv__PlanMove_Response * ros_message = _ros_message;
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
  {  // execution_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "execution_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->execution_time = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * chess_interfaces__srv__plan_move__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PlanMove_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("chess_interfaces.srv._plan_move");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PlanMove_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  chess_interfaces__srv__PlanMove_Response * ros_message = (chess_interfaces__srv__PlanMove_Response *)raw_ros_message;
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
  {  // execution_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->execution_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "execution_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
