// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from chess_interfaces:msg/BoardState.idl
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
#include "chess_interfaces/msg/detail/board_state__struct.h"
#include "chess_interfaces/msg/detail/board_state__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "geometry_msgs/msg/detail/point__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool chess_interfaces__msg__board_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("chess_interfaces.msg._board_state.BoardState", full_classname_dest, 44) == 0);
  }
  chess_interfaces__msg__BoardState * ros_message = _ros_message;
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
  {  // board_squares
    PyObject * field = PyObject_GetAttrString(_pymsg, "board_squares");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_INT8);
      Py_ssize_t size = 64;
      int8_t * dest = ros_message->board_squares;
      for (Py_ssize_t i = 0; i < size; ++i) {
        int8_t tmp = *(npy_int8 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(int8_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // square_positions_3d
    PyObject * field = PyObject_GetAttrString(_pymsg, "square_positions_3d");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'square_positions_3d'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = 64;
    geometry_msgs__msg__Point * dest = ros_message->square_positions_3d;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!geometry_msgs__msg__point__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // white_to_move
    PyObject * field = PyObject_GetAttrString(_pymsg, "white_to_move");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->white_to_move = (Py_True == field);
    Py_DECREF(field);
  }
  {  // castling_rights
    PyObject * field = PyObject_GetAttrString(_pymsg, "castling_rights");
    if (!field) {
      return false;
    }
    {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'castling_rights'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = 4;
      bool * dest = ros_message->castling_rights;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyBool_Check(item));
        bool tmp = (item == Py_True);
        memcpy(&dest[i], &tmp, sizeof(bool));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // en_passant_square
    PyObject * field = PyObject_GetAttrString(_pymsg, "en_passant_square");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->en_passant_square = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // halfmove_clock
    PyObject * field = PyObject_GetAttrString(_pymsg, "halfmove_clock");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->halfmove_clock = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // fullmove_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "fullmove_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fullmove_number = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * chess_interfaces__msg__board_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BoardState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("chess_interfaces.msg._board_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BoardState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  chess_interfaces__msg__BoardState * ros_message = (chess_interfaces__msg__BoardState *)raw_ros_message;
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
  {  // board_squares
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "board_squares");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_INT8);
    assert(sizeof(npy_int8) == sizeof(int8_t));
    npy_int8 * dst = (npy_int8 *)PyArray_GETPTR1(seq_field, 0);
    int8_t * src = &(ros_message->board_squares[0]);
    memcpy(dst, src, 64 * sizeof(int8_t));
    Py_DECREF(field);
  }
  {  // square_positions_3d
    PyObject * field = NULL;
    size_t size = 64;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    geometry_msgs__msg__Point * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->square_positions_3d[i]);
      PyObject * pyitem = geometry_msgs__msg__point__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "square_positions_3d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // white_to_move
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->white_to_move ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "white_to_move", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // castling_rights
    PyObject * field = NULL;
    size_t size = 4;
    bool * src = ros_message->castling_rights;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      int rc = PyList_SetItem(field, i, PyBool_FromLong(src[i] ? 1 : 0));
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "castling_rights", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // en_passant_square
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->en_passant_square);
    {
      int rc = PyObject_SetAttrString(_pymessage, "en_passant_square", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // halfmove_clock
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->halfmove_clock);
    {
      int rc = PyObject_SetAttrString(_pymessage, "halfmove_clock", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fullmove_number
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->fullmove_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fullmove_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
