// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from chess_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "chess_interfaces/msg/detail/piece_detection__rosidl_typesupport_introspection_c.h"
#include "chess_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "chess_interfaces/msg/detail/piece_detection__functions.h"
#include "chess_interfaces/msg/detail/piece_detection__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position_3d`
// Member `position_2d`
#include "geometry_msgs/msg/point.h"
// Member `position_3d`
// Member `position_2d`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `roi_image`
#include "sensor_msgs/msg/image.h"
// Member `roi_image`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  chess_interfaces__msg__PieceDetection__init(message_memory);
}

void chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_fini_function(void * message_memory)
{
  chess_interfaces__msg__PieceDetection__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__PieceDetection, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_3d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__PieceDetection, position_3d),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_2d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__PieceDetection, position_2d),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "piece_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__PieceDetection, piece_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "piece_color",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__PieceDetection, piece_color),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__PieceDetection, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roi_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__PieceDetection, roi_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_members = {
  "chess_interfaces__msg",  // message namespace
  "PieceDetection",  // message name
  7,  // number of fields
  sizeof(chess_interfaces__msg__PieceDetection),
  chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_member_array,  // message members
  chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_type_support_handle = {
  0,
  &chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_chess_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, msg, PieceDetection)() {
  chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_type_support_handle.typesupport_identifier) {
    chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &chess_interfaces__msg__PieceDetection__rosidl_typesupport_introspection_c__PieceDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
