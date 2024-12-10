// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_goal:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_GOAL__MSG__DETAIL__GOAL__STRUCT_H_
#define ROBOT_GOAL__MSG__DETAIL__GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Goal in the package robot_goal.
typedef struct robot_goal__msg__Goal
{
  float goal_x;
  float goal_y;
} robot_goal__msg__Goal;

// Struct for a sequence of robot_goal__msg__Goal.
typedef struct robot_goal__msg__Goal__Sequence
{
  robot_goal__msg__Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_goal__msg__Goal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_GOAL__MSG__DETAIL__GOAL__STRUCT_H_
