// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from robot_goal:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_GOAL__MSG__DETAIL__GOAL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ROBOT_GOAL__MSG__DETAIL__GOAL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_goal/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "robot_goal/msg/detail/goal__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace robot_goal
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_goal
cdr_serialize(
  const robot_goal::msg::Goal & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_goal
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robot_goal::msg::Goal & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_goal
get_serialized_size(
  const robot_goal::msg::Goal & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_goal
max_serialized_size_Goal(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace robot_goal

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_goal
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_goal, msg, Goal)();

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_GOAL__MSG__DETAIL__GOAL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
