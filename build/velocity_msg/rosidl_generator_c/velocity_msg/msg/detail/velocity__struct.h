// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from velocity_msg:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_MSG__MSG__DETAIL__VELOCITY__STRUCT_H_
#define VELOCITY_MSG__MSG__DETAIL__VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Velocity in the package velocity_msg.
typedef struct velocity_msg__msg__Velocity
{
  int32_t robot_id;
  float linear_x;
  float linear_y;
} velocity_msg__msg__Velocity;

// Struct for a sequence of velocity_msg__msg__Velocity.
typedef struct velocity_msg__msg__Velocity__Sequence
{
  velocity_msg__msg__Velocity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velocity_msg__msg__Velocity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VELOCITY_MSG__MSG__DETAIL__VELOCITY__STRUCT_H_
