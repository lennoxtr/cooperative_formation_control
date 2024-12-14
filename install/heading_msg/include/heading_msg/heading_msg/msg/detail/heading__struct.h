// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from heading_msg:msg/Heading.idl
// generated code does not contain a copyright notice

#ifndef HEADING_MSG__MSG__DETAIL__HEADING__STRUCT_H_
#define HEADING_MSG__MSG__DETAIL__HEADING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Heading in the package heading_msg.
typedef struct heading_msg__msg__Heading
{
  int32_t robot_id;
  float heading;
} heading_msg__msg__Heading;

// Struct for a sequence of heading_msg__msg__Heading.
typedef struct heading_msg__msg__Heading__Sequence
{
  heading_msg__msg__Heading * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} heading_msg__msg__Heading__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HEADING_MSG__MSG__DETAIL__HEADING__STRUCT_H_
