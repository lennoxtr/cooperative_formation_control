// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from velocity_msg:msg/Velocity.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "velocity_msg/msg/detail/velocity__rosidl_typesupport_introspection_c.h"
#include "velocity_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "velocity_msg/msg/detail/velocity__functions.h"
#include "velocity_msg/msg/detail/velocity__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  velocity_msg__msg__Velocity__init(message_memory);
}

void velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_fini_function(void * message_memory)
{
  velocity_msg__msg__Velocity__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_message_member_array[3] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_msg__msg__Velocity, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_msg__msg__Velocity, linear_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_msg__msg__Velocity, linear_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_message_members = {
  "velocity_msg__msg",  // message namespace
  "Velocity",  // message name
  3,  // number of fields
  sizeof(velocity_msg__msg__Velocity),
  velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_message_member_array,  // message members
  velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_init_function,  // function to initialize message memory (memory has to be allocated)
  velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_message_type_support_handle = {
  0,
  &velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_velocity_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, velocity_msg, msg, Velocity)() {
  if (!velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_message_type_support_handle.typesupport_identifier) {
    velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &velocity_msg__msg__Velocity__rosidl_typesupport_introspection_c__Velocity_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
