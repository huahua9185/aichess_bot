// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from chess_interfaces:msg/ChessMove.idl
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
#include "chess_interfaces/msg/detail/chess_move__struct.h"
#include "chess_interfaces/msg/detail/chess_move__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool chess_interfaces__msg__chess_move__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("chess_interfaces.msg._chess_move.ChessMove", full_classname_dest, 42) == 0);
  }
  chess_interfaces__msg__ChessMove * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // from_square
    PyObject * field = PyObject_GetAttrString(_pymsg, "from_square");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->from_square = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // to_square
    PyObject * field = PyObject_GetAttrString(_pymsg, "to_square");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->to_square = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // piece_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "piece_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->piece_type = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // captured_piece
    PyObject * field = PyObject_GetAttrString(_pymsg, "captured_piece");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->captured_piece = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // promotion
    PyObject * field = PyObject_GetAttrString(_pymsg, "promotion");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->promotion = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // is_castling
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_castling");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_castling = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_en_passant
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_en_passant");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_en_passant = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_check
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_check");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_check = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_checkmate
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_checkmate");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_checkmate = (Py_True == field);
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
  {  // thinking_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "thinking_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thinking_time = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * chess_interfaces__msg__chess_move__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ChessMove */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("chess_interfaces.msg._chess_move");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ChessMove");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  chess_interfaces__msg__ChessMove * ros_message = (chess_interfaces__msg__ChessMove *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // from_square
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->from_square);
    {
      int rc = PyObject_SetAttrString(_pymessage, "from_square", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // to_square
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->to_square);
    {
      int rc = PyObject_SetAttrString(_pymessage, "to_square", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // piece_type
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->piece_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "piece_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // captured_piece
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->captured_piece);
    {
      int rc = PyObject_SetAttrString(_pymessage, "captured_piece", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // promotion
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->promotion);
    {
      int rc = PyObject_SetAttrString(_pymessage, "promotion", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_castling
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_castling ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_castling", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_en_passant
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_en_passant ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_en_passant", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_check
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_check ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_check", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_checkmate
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_checkmate ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_checkmate", field);
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
  {  // thinking_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thinking_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thinking_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
