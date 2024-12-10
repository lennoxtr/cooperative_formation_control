// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_goal:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_GOAL__MSG__DETAIL__GOAL__TRAITS_HPP_
#define ROBOT_GOAL__MSG__DETAIL__GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_goal/msg/detail/goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_goal
{

namespace msg
{

inline void to_flow_style_yaml(
  const Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_x
  {
    out << "goal_x: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_x, out);
    out << ", ";
  }

  // member: goal_y
  {
    out << "goal_y: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_x: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_x, out);
    out << "\n";
  }

  // member: goal_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_y: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_goal

namespace rosidl_generator_traits
{

[[deprecated("use robot_goal::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_goal::msg::Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_goal::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_goal::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_goal::msg::Goal & msg)
{
  return robot_goal::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_goal::msg::Goal>()
{
  return "robot_goal::msg::Goal";
}

template<>
inline const char * name<robot_goal::msg::Goal>()
{
  return "robot_goal/msg/Goal";
}

template<>
struct has_fixed_size<robot_goal::msg::Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_goal::msg::Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_goal::msg::Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_GOAL__MSG__DETAIL__GOAL__TRAITS_HPP_
