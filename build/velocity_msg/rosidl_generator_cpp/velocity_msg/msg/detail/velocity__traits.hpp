// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from velocity_msg:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_MSG__MSG__DETAIL__VELOCITY__TRAITS_HPP_
#define VELOCITY_MSG__MSG__DETAIL__VELOCITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "velocity_msg/msg/detail/velocity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace velocity_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Velocity & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: linear_x
  {
    out << "linear_x: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_x, out);
    out << ", ";
  }

  // member: linear_y
  {
    out << "linear_y: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Velocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: linear_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_x: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_x, out);
    out << "\n";
  }

  // member: linear_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_y: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Velocity & msg, bool use_flow_style = false)
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

}  // namespace velocity_msg

namespace rosidl_generator_traits
{

[[deprecated("use velocity_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const velocity_msg::msg::Velocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  velocity_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use velocity_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const velocity_msg::msg::Velocity & msg)
{
  return velocity_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<velocity_msg::msg::Velocity>()
{
  return "velocity_msg::msg::Velocity";
}

template<>
inline const char * name<velocity_msg::msg::Velocity>()
{
  return "velocity_msg/msg/Velocity";
}

template<>
struct has_fixed_size<velocity_msg::msg::Velocity>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<velocity_msg::msg::Velocity>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<velocity_msg::msg::Velocity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VELOCITY_MSG__MSG__DETAIL__VELOCITY__TRAITS_HPP_
