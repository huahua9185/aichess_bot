// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from chess_interfaces:srv/ExecuteMove.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "chess_interfaces/srv/detail/execute_move__rosidl_typesupport_introspection_c.h"
#include "chess_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "chess_interfaces/srv/detail/execute_move__functions.h"
#include "chess_interfaces/srv/detail/execute_move__struct.h"


// Include directives for member types
// Member `chess_move`
#include "chess_interfaces/msg/chess_move.h"
// Member `chess_move`
#include "chess_interfaces/msg/detail/chess_move__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  chess_interfaces__srv__ExecuteMove_Request__init(message_memory);
}

void chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_fini_function(void * message_memory)
{
  chess_interfaces__srv__ExecuteMove_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_member_array[2] = {
  {
    "chess_move",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__srv__ExecuteMove_Request, chess_move),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confirm_execution",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__srv__ExecuteMove_Request, confirm_execution),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_members = {
  "chess_interfaces__srv",  // message namespace
  "ExecuteMove_Request",  // message name
  2,  // number of fields
  sizeof(chess_interfaces__srv__ExecuteMove_Request),
  chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_member_array,  // message members
  chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_type_support_handle = {
  0,
  &chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_chess_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, srv, ExecuteMove_Request)() {
  chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, msg, ChessMove)();
  if (!chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_type_support_handle.typesupport_identifier) {
    chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &chess_interfaces__srv__ExecuteMove_Request__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "chess_interfaces/srv/detail/execute_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "chess_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "chess_interfaces/srv/detail/execute_move__functions.h"
// already included above
// #include "chess_interfaces/srv/detail/execute_move__struct.h"


// Include directives for member types
// Member `error_message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  chess_interfaces__srv__ExecuteMove_Response__init(message_memory);
}

void chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_fini_function(void * message_memory)
{
  chess_interfaces__srv__ExecuteMove_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_member_array[4] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__srv__ExecuteMove_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__srv__ExecuteMove_Response, error_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actual_execution_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__srv__ExecuteMove_Response, actual_execution_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "piece_captured",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__srv__ExecuteMove_Response, piece_captured),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_members = {
  "chess_interfaces__srv",  // message namespace
  "ExecuteMove_Response",  // message name
  4,  // number of fields
  sizeof(chess_interfaces__srv__ExecuteMove_Response),
  chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_member_array,  // message members
  chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_type_support_handle = {
  0,
  &chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_chess_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, srv, ExecuteMove_Response)() {
  if (!chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_type_support_handle.typesupport_identifier) {
    chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &chess_interfaces__srv__ExecuteMove_Response__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "chess_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "chess_interfaces/srv/detail/execute_move__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_service_members = {
  "chess_interfaces__srv",  // service namespace
  "ExecuteMove",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_Request_message_type_support_handle,
  NULL  // response message
  // chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_Response_message_type_support_handle
};

static rosidl_service_type_support_t chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_service_type_support_handle = {
  0,
  &chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, srv, ExecuteMove_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, srv, ExecuteMove_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_chess_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, srv, ExecuteMove)() {
  if (!chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_service_type_support_handle.typesupport_identifier) {
    chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, srv, ExecuteMove_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, srv, ExecuteMove_Response)()->data;
  }

  return &chess_interfaces__srv__detail__execute_move__rosidl_typesupport_introspection_c__ExecuteMove_service_type_support_handle;
}
